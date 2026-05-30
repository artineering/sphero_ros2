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
