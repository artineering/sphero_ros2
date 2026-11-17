#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Per-Pi HTTP launcher agent for the distributed BLE worker fleet.

Exposes a tiny bearer-token-authenticated HTTP API so the coordinator
(multirobot_webserver on rpi5-main) can spawn / tear down / inspect Sphero
instance trees on this worker:

    POST   /spawn          {name, port, external_localization?, fleet_hold?}
    DELETE /spawn/<name>
    GET    /status
    GET    /health
    GET    /scan?duration=<s>  - advertising Spheros (takes the per-host BLE lock)

The agent runs the existing
`ros2 run sphero_instance_controller sphero_instance_websocket_server.py`
locally (the 3 controllers self-launch unchanged), injects the CycloneDDS env
into every spawned process, captures per-Sphero logs, and does process-group
teardown. It binds to a configurable host/port (default the LAN interface, not
0.0.0.0) and authenticates every request with a shared bearer token.

Configuration (env vars):
    WORKER_AGENT_TOKEN   - shared bearer token (REQUIRED).
    WORKER_AGENT_HOST    - bind address (default 127.0.0.1; set to the LAN IP).
    WORKER_AGENT_PORT    - bind port (default 8181).
    WORKER_AGENT_LOG_DIR - per-Sphero log directory (default
                           /tmp/sphero_worker_agent).
    ROS_DOMAIN_ID, RMW_IMPLEMENTATION, CYCLONEDDS_URI - injected into every
                           spawned Sphero process (set once per Pi).
    WORKER_CONNECT_QUEUE - 1 (default): run an rclpy node publishing this host's
                           BLE connect queue on /worker/<host>/connect_queue
                           (see connect_queue.py); 0: Flask only (rollback).
    WORKER_CONNECT_ENTRY_TIMEOUT / _FIRST_BEACON / _BEACON_STALE - queue entry
                           limits in seconds (defaults 150 / 60 / 10).

Run via: ros2 run sphero_worker_agent agent
"""

import os
import signal
import socket
import sys
import threading
from functools import wraps

from flask import Flask, jsonify, request

from sphero_worker_agent import ble_scan
from sphero_worker_agent.manager import SpheroProcessManager

# DDS env keys injected into every spawned Sphero process (set once per Pi).
DDS_ENV_KEYS = ('ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'CYCLONEDDS_URI')

DEFAULT_HOST = '127.0.0.1'
DEFAULT_PORT = 8181


def _collect_dds_env() -> dict:
    """Pull the CycloneDDS env vars present in the agent's environment."""
    return {k: os.environ[k] for k in DDS_ENV_KEYS if k in os.environ}


def create_app(manager: SpheroProcessManager, token: str,
               worker_name: str = None, scan_fn=ble_scan.scan) -> Flask:
    """Build the Flask app. Token auth guards every route except nothing."""
    app = Flask(__name__)
    worker = worker_name or socket.gethostname()

    def require_token(fn):
        @wraps(fn)
        def wrapper(*args, **kwargs):
            auth = request.headers.get('Authorization', '')
            expected = f'Bearer {token}'
            if not token or auth != expected:
                return jsonify({'error': 'unauthorized'}), 401
            return fn(*args, **kwargs)
        return wrapper

    @app.route('/health', methods=['GET'])
    @require_token
    def health():
        return jsonify({'status': 'ok', 'worker': worker})

    @app.route('/status', methods=['GET'])
    @require_token
    def status():
        return jsonify({**manager.status(), 'worker': worker})

    @app.route('/scan', methods=['GET'])
    @require_token
    def scan():
        duration = ble_scan.clamp_duration(request.args.get('duration'))
        try:
            result, robots = scan_fn(duration)
        except Exception as exc:  # noqa: BLE001 - report any BLE failure
            print(f'scan failed: {exc!r}')
            return jsonify({'worker': worker, 'error': str(exc)}), 500
        if result == 'busy':
            return jsonify({'worker': worker, 'busy': True,
                            'error': 'BLE connect in progress'}), 409
        return jsonify({'worker': worker, 'duration_s': duration,
                        'robots': robots})

    @app.route('/spawn', methods=['POST'])
    @require_token
    def spawn():
        body = request.get_json(silent=True) or {}
        name = body.get('name')
        port = body.get('port')
        if not name or port is None:
            return jsonify({'error': 'name and port are required'}), 400
        try:
            port = int(port)
        except (TypeError, ValueError):
            return jsonify({'error': 'port must be an integer'}), 400
        external = bool(body.get('external_localization', False))
        # fleet_hold: the robot stays BLE-quiet after connecting until the
        # webserver's /fleet/ble_release names it (SPHERO_BLE_FLEET_HOLD=1).
        fleet_hold = body.get('fleet_hold', False) is True
        result = manager.spawn(name, port, external_localization=external,
                               fleet_hold=fleet_hold)
        return jsonify(result), (200 if result['success'] else 409)

    @app.route('/spawn/<name>', methods=['DELETE'])
    @require_token
    def remove(name):
        result = manager.teardown(name)
        return jsonify(result), (200 if result['success'] else 404)

    return app


def _start_connect_queue(manager: SpheroProcessManager, host: str):
    """
    Start the connect-queue rclpy node on a background executor thread.

    Returns (node, executor) or (None, None). Any failure leaves the agent
    serving HTTP without a queue: device controllers then fall back to the
    per-host flock.
    """
    try:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.signals import SignalHandlerOptions

        from sphero_worker_agent.connect_queue import make_node

        # Keep our own SIGTERM/SIGINT handlers (tree teardown on stop).
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = make_node(host, manager.is_alive)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        threading.Thread(target=executor.spin, daemon=True,
                         name='connect-queue-executor').start()
        manager.queue_hooks = node
        return node, executor
    except Exception as exc:  # noqa: BLE001 - never stop serving over DDS
        print(f'connect queue disabled: {exc!r}', flush=True)
        try:
            import rclpy
            rclpy.try_shutdown()
        except Exception:  # noqa: BLE001
            pass
        return None, None


def _stop_connect_queue(node, executor) -> None:
    if node is None:
        return
    try:
        import rclpy
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        rclpy.try_shutdown()
    except Exception as exc:  # noqa: BLE001 - best-effort on exit
        print(f'connect queue shutdown: {exc!r}', flush=True)


def main(argv=None):
    """Console entry point: ros2 run sphero_worker_agent agent."""
    token = os.environ.get('WORKER_AGENT_TOKEN', '')
    if not token:
        print('ERROR: WORKER_AGENT_TOKEN must be set (shared bearer token).',
              file=sys.stderr)
        return 1

    host = os.environ.get('WORKER_AGENT_HOST', DEFAULT_HOST)
    port = int(os.environ.get('WORKER_AGENT_PORT', str(DEFAULT_PORT)))
    log_dir = os.environ.get('WORKER_AGENT_LOG_DIR',
                             '/tmp/sphero_worker_agent')

    dds_env = _collect_dds_env()
    # Optional launch-cmd override (testing only); falls back to the real
    # `ros2 run sphero_instance_controller ...` invocation when unset.
    launch_override = os.environ.get('WORKER_AGENT_LAUNCH_CMD')
    launch_cmd = launch_override.split() if launch_override else None
    # Same resolution the device controllers use; injected into every spawn
    # as SPHERO_WORKER_HOST so both sides name the same queue topic.
    from sphero_instance_controller.ble_connect_queue import resolve_host
    worker_host = resolve_host()
    manager = SpheroProcessManager(dds_env=dds_env, log_dir=log_dir,
                                   launch_cmd=launch_cmd,
                                   worker_host=worker_host)
    queue_node, executor = None, None
    if os.environ.get('WORKER_CONNECT_QUEUE', '1').strip() != '0':
        queue_node, executor = _start_connect_queue(manager, worker_host)
    else:
        print('connect queue off (WORKER_CONNECT_QUEUE=0)', flush=True)
    app = create_app(manager, token, worker_name=socket.gethostname())

    print(f'sphero_worker_agent listening on {host}:{port}')
    print(f'  log dir: {log_dir}')
    print(f'  injected DDS env: {sorted(dds_env.keys()) or "(none)"}')
    print(f'  connect queue: {queue_node.topic if queue_node else "off"}',
          flush=True)

    # Run Flask's server in a daemon thread and keep the MAIN thread free to
    # take signals. The dev server's serving loop blocks its own thread in a
    # syscall and can swallow/defer a signal handler; by parking the main
    # thread on an Event we guarantee SIGTERM/SIGINT (systemd stop, Ctrl-C, or
    # ros2 run forwarding the signal) runs our teardown -- releasing every
    # spawned Sphero tree's BLE links -- before the process exits.
    stop = threading.Event()

    def _shutdown(signum, _frame):
        print(f'sphero_worker_agent received signal {signum}; '
              f'tearing down all Sphero trees')
        stop.set()

    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)

    server = threading.Thread(
        target=lambda: app.run(host=host, port=port, threaded=True,
                               use_reloader=False),
        daemon=True,
    )
    server.start()
    try:
        stop.wait()
    finally:
        manager.teardown_all()
        _stop_connect_queue(queue_node, executor)
    return 0


if __name__ == '__main__':
    sys.exit(main())
