#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Process-lifecycle tests for the worker agent's SpheroProcessManager.

Uses a fake long-running launch command (a child that spawns its own
grandchild in the same process group) instead of a real Sphero tree, so the
test verifies spawn/liveness/process-group teardown without any BLE hardware.
"""

import os
import signal
import sys
import time

import pytest

from sphero_worker_agent.manager import SpheroProcessManager


def _fake_cmd(tmp_path, pidfile):
    """A python child that forks a grandchild; both sleep. Returns argv prefix.

    The grandchild writes its PID to `pidfile` so the test can confirm the
    whole process group is killed on teardown. The manager appends
    [name, port] to this prefix; the fake ignores those trailing args.
    """
    script = tmp_path / 'fake_tree.py'
    script.write_text(
        'import os, time\n'
        f'pidfile = {str(pidfile)!r}\n'
        'pid = os.fork()\n'
        'if pid == 0:\n'
        '    open(pidfile, "w").write(str(os.getpid()))\n'
        '    time.sleep(300)\n'
        'else:\n'
        '    time.sleep(300)\n'
    )
    return [sys.executable, str(script)]


def _alive(pid):
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def test_spawn_status_teardown_no_orphans(tmp_path):
    pidfile = tmp_path / 'grandchild.pid'
    mgr = SpheroProcessManager(
        dds_env={'ROS_DOMAIN_ID': '42'},
        log_dir=str(tmp_path / 'logs'),
        launch_cmd=_fake_cmd(tmp_path, pidfile),
    )
    result = mgr.spawn('SB-TEST', port=5001)
    assert result['success'], result

    st = mgr.status()
    assert st['count'] == 1
    assert st['spheros'][0]['alive'] is True

    # Grandchild should be running and in the same process group as the root.
    time.sleep(0.5)
    assert pidfile.exists()
    grandchild = int(pidfile.read_text())
    assert _alive(grandchild)

    down = mgr.teardown('SB-TEST')
    assert down['success']
    assert mgr.status()['count'] == 0

    # No orphaned grandchild: process-group kill must have reaped it.
    time.sleep(0.5)
    assert not _alive(grandchild)


def test_teardown_unknown_returns_failure(tmp_path):
    mgr = SpheroProcessManager(log_dir=str(tmp_path / 'logs'),
                               launch_cmd=_fake_cmd(tmp_path,
                                                    tmp_path / 'x.pid'))
    res = mgr.teardown('nope')
    assert res['success'] is False


def test_log_file_created(tmp_path):
    log_dir = tmp_path / 'logs'
    mgr = SpheroProcessManager(log_dir=str(log_dir),
                               launch_cmd=_fake_cmd(tmp_path,
                                                    tmp_path / 'gc.pid'))
    mgr.spawn('SB-LOG', port=5002)
    assert (log_dir / 'SB-LOG.log').exists()
    mgr.teardown('SB-LOG')


# ----- connect-queue hooks -----

class RecordingHooks:
    """Records hook calls; on_spawn notes whether the child already exists."""

    def __init__(self, pidfile=None):
        self.calls = []
        self.pidfile = pidfile

    def on_spawn(self, name, spawn_id):
        started = self.pidfile.exists() if self.pidfile else None
        self.calls.append(('spawn', name, spawn_id, started))

    def on_gone(self, name, reason, spawn_id=None):
        self.calls.append(('gone', name, reason, spawn_id))


def _env_dump_cmd(tmp_path, out):
    """A child that writes its env as JSON, then sleeps."""
    script = tmp_path / 'env_dump.py'
    script.write_text(
        'import json, os, time\n'
        f'open({str(out)!r}, "w").write(json.dumps(dict(os.environ)))\n'
        'time.sleep(300)\n')
    return [sys.executable, str(script)]


def test_on_spawn_runs_before_popen_and_gone_on_teardown(tmp_path):
    pidfile = tmp_path / 'gc.pid'
    hooks = RecordingHooks(pidfile)
    mgr = SpheroProcessManager(log_dir=str(tmp_path / 'logs'),
                               launch_cmd=_fake_cmd(tmp_path, pidfile),
                               queue_hooks=hooks)
    assert mgr.spawn('SB-Q', port=5003)['success']
    kind, name, spawn_id, started = hooks.calls[0]
    assert (kind, name, started) == ('spawn', 'SB-Q', False)
    assert mgr.is_alive('SB-Q')
    mgr.teardown('SB-Q')
    assert hooks.calls[-1] == ('gone', 'SB-Q', 'teardown', spawn_id)
    assert not mgr.is_alive('SB-Q')


def test_spawn_env_carries_host_and_spawn_id(tmp_path):
    import json
    out = tmp_path / 'env.json'
    hooks = RecordingHooks()
    mgr = SpheroProcessManager(dds_env={'ROS_DOMAIN_ID': '7'},
                               log_dir=str(tmp_path / 'logs'),
                               launch_cmd=_env_dump_cmd(tmp_path, out),
                               queue_hooks=hooks, worker_host='rpi4-node09')
    assert mgr.spawn('SB-ENV', port=5004)['success']
    for _ in range(50):
        if out.exists() and out.read_text():
            break
        time.sleep(0.1)
    env = json.loads(out.read_text())
    assert env['SPHERO_WORKER_HOST'] == 'rpi4-node09'
    assert env['SPHERO_SPAWN_ID'] == hooks.calls[0][2]
    assert env['ROS_DOMAIN_ID'] == '7'
    mgr.teardown('SB-ENV')


def test_respawn_gets_a_new_spawn_id(tmp_path):
    hooks = RecordingHooks()
    mgr = SpheroProcessManager(log_dir=str(tmp_path / 'logs'),
                               launch_cmd=_fake_cmd(tmp_path, tmp_path / 'r.pid'),
                               queue_hooks=hooks)
    mgr.spawn('SB-R', port=5005)
    mgr.teardown('SB-R')
    mgr.spawn('SB-R', port=5005)
    mgr.teardown('SB-R')
    ids = [c[2] for c in hooks.calls if c[0] == 'spawn']
    assert len(ids) == 2 and ids[0] != ids[1]


def test_gone_when_child_exits_during_startup(tmp_path):
    hooks = RecordingHooks()
    mgr = SpheroProcessManager(log_dir=str(tmp_path / 'logs'),
                               launch_cmd=[sys.executable, '-c', 'import sys; sys.exit(1)'],
                               queue_hooks=hooks)
    assert not mgr.spawn('SB-DEAD', port=5006)['success']
    assert hooks.calls[-1][:3] == ('gone', 'SB-DEAD', 'exited_during_startup')


def test_gone_when_popen_fails(tmp_path):
    hooks = RecordingHooks()
    mgr = SpheroProcessManager(log_dir=str(tmp_path / 'logs'),
                               launch_cmd=[str(tmp_path / 'no-such-binary')],
                               queue_hooks=hooks)
    assert not mgr.spawn('SB-BAD', port=5007)['success']
    spawn_id = hooks.calls[0][2]
    assert hooks.calls[-1] == ('gone', 'SB-BAD', 'spawn_failed', spawn_id)


def test_failing_hook_never_breaks_spawn(tmp_path):
    class Broken:

        def on_spawn(self, *a):
            raise RuntimeError('dds down')

        def on_gone(self, *a):
            raise RuntimeError('dds down')

    mgr = SpheroProcessManager(log_dir=str(tmp_path / 'logs'),
                               launch_cmd=_fake_cmd(tmp_path, tmp_path / 'b.pid'),
                               queue_hooks=Broken())
    assert mgr.spawn('SB-B', port=5008)['success']
    assert mgr.teardown('SB-B')['success']


# ----- fleet hold passthrough -----

def _spawn_env(tmp_path, **spawn_kwargs):
    import json
    out = tmp_path / 'env.json'
    mgr = SpheroProcessManager(log_dir=str(tmp_path / 'logs'),
                               launch_cmd=_env_dump_cmd(tmp_path, out))
    assert mgr.spawn('SB-FH', port=5009, **spawn_kwargs)['success']
    for _ in range(50):
        if out.exists() and out.read_text():
            break
        time.sleep(0.1)
    info = mgr.status()['spheros'][0]
    mgr.teardown('SB-FH')
    return json.loads(out.read_text()), info


def test_fleet_hold_flag_reaches_spawn_env(tmp_path):
    env, info = _spawn_env(tmp_path, fleet_hold=True)
    assert env['SPHERO_BLE_FLEET_HOLD'] == '1'
    assert info['fleet_hold'] is True


def test_no_fleet_hold_is_explicitly_off(tmp_path, monkeypatch):
    # Even if the agent's own env carries the flag, a plain spawn is off.
    monkeypatch.setenv('SPHERO_BLE_FLEET_HOLD', '1')
    env, info = _spawn_env(tmp_path)
    assert env['SPHERO_BLE_FLEET_HOLD'] == '0'
    assert info['fleet_hold'] is False


def test_spawn_route_passes_fleet_hold():
    from sphero_worker_agent.agent import create_app

    class FakeManager:
        def __init__(self):
            self.calls = []

        def spawn(self, name, port, external_localization=False, fleet_hold=False):
            self.calls.append((name, port, external_localization, fleet_hold))
            return {'success': True, 'message': 'ok', 'sphero': None}

    mgr = FakeManager()
    client = create_app(mgr, 'tok', worker_name='w').test_client()
    auth = {'Authorization': 'Bearer tok'}
    for body, expect in (({'fleet_hold': True}, True), ({'fleet_hold': False}, False),
                         ({}, False), ({'fleet_hold': 'yes'}, False)):
        res = client.post('/spawn', json={'name': 'SB-A', 'port': 5000, **body},
                          headers=auth)
        assert res.status_code == 200
        assert mgr.calls[-1][3] is expect
