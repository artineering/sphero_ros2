#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Per-Pi HTTP launcher agent for the distributed BLE worker fleet.

Exposes a tiny bearer-token-authenticated HTTP API so the coordinator
(multirobot_webserver on rpi5-main) can spawn / tear down / inspect Sphero
instance trees on this worker:

    POST   /spawn          {name, port, external_localization?}
    DELETE /spawn/<name>
    GET    /status
    GET    /health

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

Run via: ros2 run sphero_worker_agent agent
"""

import os
import signal
import sys
import threading
from functools import wraps

from flask import Flask, jsonify, request

from sphero_worker_agent.manager import SpheroProcessManager

# DDS env keys injected into every spawned Sphero process (set once per Pi).
DDS_ENV_KEYS = ('ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'CYCLONEDDS_URI')

DEFAULT_HOST = '127.0.0.1'
DEFAULT_PORT = 8181


def _collect_dds_env() -> dict:
    """Pull the CycloneDDS env vars present in the agent's environment."""
    return {k: os.environ[k] for k in DDS_ENV_KEYS if k in os.environ}


def create_app(manager: SpheroProcessManager, token: str) -> Flask:
    """Build the Flask app. Token auth guards every route except nothing."""
    app = Flask(__name__)

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
        return jsonify({'status': 'ok'})

    @app.route('/status', methods=['GET'])
    @require_token
    def status():
        return jsonify(manager.status())

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
        result = manager.spawn(name, port, external_localization=external)
        return jsonify(result), (200 if result['success'] else 409)

    @app.route('/spawn/<name>', methods=['DELETE'])
    @require_token
    def remove(name):
        result = manager.teardown(name)
        return jsonify(result), (200 if result['success'] else 404)

    return app


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
    manager = SpheroProcessManager(dds_env=dds_env, log_dir=log_dir,
                                   launch_cmd=launch_cmd)
    app = create_app(manager, token)

    print(f'sphero_worker_agent listening on {host}:{port}')
    print(f'  log dir: {log_dir}')
    print(f'  injected DDS env: {sorted(dds_env.keys()) or "(none)"}')

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
    return 0


if __name__ == '__main__':
    sys.exit(main())
