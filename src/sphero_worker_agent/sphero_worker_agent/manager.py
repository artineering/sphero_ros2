#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Local Sphero process manager for the worker agent.

Spawns and tears down per-Sphero process trees on this worker host using the
SAME `ros2 run sphero_instance_controller sphero_instance_websocket_server.py`
invocation the coordinator's add_sphero uses today. The websocket server
self-launches its three controllers, so the unit of placement is one Sphero ->
one 4-process tree on this host.

Teardown reuses the proven process-group pattern (os.killpg with
SIGTERM -> SIGKILL after a timeout) so the whole tree -- including the
controllers spawned in their own process groups -- is released cleanly and the
BLE link is dropped deterministically.
"""

import os
import signal
import subprocess
import threading
import time
import uuid
from typing import Dict, List, Optional


# Seconds to wait for a graceful SIGTERM before escalating to SIGKILL.
TERM_TIMEOUT_S = 10


class SpheroProcessManager:
    """Tracks the Sphero instance trees running on this worker."""

    def __init__(self, dds_env: Optional[Dict[str, str]] = None,
                 log_dir: str = '/tmp/sphero_worker_agent',
                 launch_cmd: Optional[List[str]] = None,
                 queue_hooks=None, worker_host: Optional[str] = None):
        """
        Args:
            dds_env: CycloneDDS env vars injected into every spawned process
                (ROS_DOMAIN_ID, RMW_IMPLEMENTATION, CYCLONEDDS_URI). Each Pi is
                configured once; spawned children inherit these.
            log_dir: Directory for per-Sphero stdout/stderr log files.
            launch_cmd: Base `ros2 run` invocation; overridable for testing.
            queue_hooks: Optional connect-queue hooks with on_spawn(name,
                spawn_id) (called BEFORE Popen) and on_gone(name, reason,
                spawn_id) (Popen failure / reap). None = no queue (old behaviour).
            worker_host: Injected as SPHERO_WORKER_HOST so the device
                controller subscribes to this host's connect queue.
        """
        self.dds_env = dict(dds_env or {})
        self.log_dir = log_dir
        os.makedirs(self.log_dir, exist_ok=True)
        self._launch_cmd = launch_cmd or [
            'ros2', 'run', 'sphero_instance_controller',
            'sphero_instance_websocket_server.py',
        ]
        # name -> {process, port, external_localization, started_at, log_file}
        self._spheros: Dict[str, Dict] = {}
        self._lock = threading.Lock()
        self.queue_hooks = queue_hooks
        self.worker_host = worker_host

    def _build_env(self, extra: Optional[Dict[str, str]] = None) -> Dict[str, str]:
        env = os.environ.copy()
        env.update(self.dds_env)
        env.update(extra or {})
        return env

    def spawn(self, name: str, port: int,
              external_localization: bool = False,
              fleet_hold: bool = False) -> Dict:
        """
        Spawn the Sphero instance tree for `name`.

        Returns a dict {'success': bool, 'message': str, 'sphero': <info>|None}.
        Mirrors add_sphero's startup check: wait briefly, then confirm the root
        process is still alive.
        """
        with self._lock:
            if name in self._spheros and self._is_alive(name):
                return {
                    'success': False,
                    'message': f'{name} already running',
                    'sphero': self._info(name),
                }

            cmd = list(self._launch_cmd) + [name, str(port)]
            if external_localization:
                cmd.append('true')

            log_path = os.path.join(self.log_dir, f'{name}.log')
            log_file = open(log_path, 'ab', buffering=0)
            # Identifies this process generation of `name` on /status, so the
            # connect queue ignores messages from an earlier process.
            spawn_id = uuid.uuid4().hex
            extra_env = {'SPHERO_SPAWN_ID': spawn_id}
            if self.worker_host:
                extra_env['SPHERO_WORKER_HOST'] = self.worker_host
            # Always set (1/0) so a value in the agent's own env can't leak
            # into a spawn: absent/false in the request = no fleet hold.
            extra_env['SPHERO_BLE_FLEET_HOLD'] = '1' if fleet_hold else '0'
            # Listed before Popen: neighbours hold from this moment, not from
            # the controller's main() 15-25 s later.
            self._hook('on_spawn', name, spawn_id)
            try:
                # New process group so we can killpg the whole tree on teardown,
                # same pattern the websocket server uses for its controllers.
                process = subprocess.Popen(
                    cmd,
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                    env=self._build_env(extra_env),
                    preexec_fn=os.setpgrp,
                )
            except Exception as exc:  # noqa: BLE001 - report any spawn failure
                log_file.close()
                self._hook('on_gone', name, 'spawn_failed', spawn_id)
                return {
                    'success': False,
                    'message': f'spawn failed: {exc}',
                    'sphero': None,
                }

            self._spheros[name] = {
                'process': process,
                'port': port,
                'external_localization': external_localization,
                'started_at': time.time(),
                'log_file': log_file,
                'log_path': log_path,
                'spawn_id': spawn_id,
                'fleet_hold': fleet_hold,
            }

        # Brief startup confirmation (outside the lock so spawns don't serialize
        # on the sleep), matching add_sphero's 2s poll.
        time.sleep(2)
        if not self._is_alive(name):
            self._reap(name, 'exited_during_startup')
            return {
                'success': False,
                'message': f'{name} process exited during startup '
                           f'(see {log_path})',
                'sphero': None,
            }
        return {
            'success': True,
            'message': f'{name} started on port {port}',
            'sphero': self._info(name),
        }

    def teardown(self, name: str) -> Dict:
        """Stop the Sphero tree for `name` via process-group SIGTERM->SIGKILL."""
        with self._lock:
            entry = self._spheros.get(name)
            if entry is None:
                return {'success': False, 'message': f'{name} not running'}
            process = entry['process']

        try:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            except (ProcessLookupError, AttributeError):
                process.terminate()
            try:
                process.wait(timeout=TERM_TIMEOUT_S)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                except (ProcessLookupError, AttributeError):
                    process.kill()
                process.wait()
        finally:
            self._reap(name, 'teardown')
        return {'success': True, 'message': f'{name} stopped'}

    def teardown_all(self) -> None:
        """Tear down every running Sphero tree (agent shutdown)."""
        for name in list(self._spheros.keys()):
            self.teardown(name)

    def status(self) -> Dict:
        """Per-Sphero liveness report for GET /status."""
        with self._lock:
            return {
                'spheros': [self._info(name) for name in self._spheros],
                'count': len(self._spheros),
            }

    def is_alive(self, name: str) -> bool:
        """Thread-safe liveness of `name`'s root process (False if unknown)."""
        with self._lock:
            return self._is_alive(name)

    # --- internal helpers -------------------------------------------------

    def _hook(self, method: str, *args) -> None:
        """Call a connect-queue hook; a failing hook never breaks spawn/teardown."""
        if self.queue_hooks is None:
            return
        try:
            getattr(self.queue_hooks, method)(*args)
        except Exception as exc:  # noqa: BLE001 - best-effort queue update
            print(f'connect-queue hook {method} failed: {exc!r}', flush=True)

    def _is_alive(self, name: str) -> bool:
        entry = self._spheros.get(name)
        return entry is not None and entry['process'].poll() is None

    def _info(self, name: str) -> Optional[Dict]:
        entry = self._spheros.get(name)
        if entry is None:
            return None
        return {
            'name': name,
            'port': entry['port'],
            'external_localization': entry['external_localization'],
            'started_at': entry['started_at'],
            'alive': entry['process'].poll() is None,
            'fleet_hold': entry.get('fleet_hold', False),
            'log_path': entry['log_path'],
        }

    def _reap(self, name: str, reason: str = 'reaped') -> None:
        with self._lock:
            entry = self._spheros.pop(name, None)
        self._hook('on_gone', name, reason,
                   entry.get('spawn_id') if entry is not None else None)
        if entry is not None:
            try:
                entry['log_file'].close()
            except Exception:  # noqa: BLE001 - best-effort log close
                pass
