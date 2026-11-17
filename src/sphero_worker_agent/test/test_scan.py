#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Tests for the worker agent's GET /scan and the shared BLE flock.

No BLE hardware: discovery is stubbed, and the lock tests take a real flock
on a temp lockfile (via SPHERO_BLE_CONNECT_LOCK) to stand in for a device
controller mid scan+connect.
"""

import fcntl
import time
from unittest.mock import MagicMock

import pytest

from sphero_worker_agent import ble_scan
from sphero_worker_agent.agent import create_app

TOKEN = 'tok'
AUTH = {'Authorization': f'Bearer {TOKEN}'}
PREFIXES = ble_scan.FALLBACK_PREFIXES


@pytest.fixture
def lock_path(tmp_path, monkeypatch):
    path = tmp_path / 'ble.lock'
    monkeypatch.setenv('SPHERO_BLE_CONNECT_LOCK', str(path))
    return path


def _hold(path):
    """Take the flock from a separate fd, like a controller would."""
    fd = open(path, 'w')
    fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    return fd


def _client(scan_fn, manager=None):
    manager = manager or MagicMock()
    app = create_app(manager, TOKEN, worker_name='rpi4-test', scan_fn=scan_fn)
    return app.test_client()


# --- lock -----------------------------------------------------------------

def test_lock_acquire_release_reacquire(lock_path):
    with ble_scan.ble_lock(0.5) as ok:
        assert ok
    with ble_scan.ble_lock(0.5) as ok:
        assert ok


def test_lock_times_out_when_held(lock_path):
    holder = _hold(lock_path)
    try:
        start = time.monotonic()
        with ble_scan.ble_lock(0.3) as ok:
            assert not ok
        assert time.monotonic() - start < 1.0
    finally:
        holder.close()


def test_lock_released_on_exception(lock_path):
    with pytest.raises(RuntimeError):
        with ble_scan.ble_lock(0.5) as ok:
            assert ok
            raise RuntimeError('boom')
    holder = _hold(lock_path)  # would raise BlockingIOError if still held
    holder.close()


# --- filtering / clamping -------------------------------------------------

def test_filter_and_dedupe():
    raw = [
        ('SB-3660', 'AA', -70),
        ('SB-3660', 'AA', -55),
        ('SB-1FA8', 'BB', -80),
        ('BB-1234', 'CC', -60),
        ('SK-0001', 'DD', -90),
        ('Pixel 7', 'EE', -30),
        (None, 'FF', -20),
    ]
    robots = ble_scan.filter_and_dedupe(raw, PREFIXES)
    assert robots == [
        {'name': 'SB-3660', 'address': 'AA', 'rssi': -55},
        {'name': 'BB-1234', 'address': 'CC', 'rssi': -60},
        {'name': 'SB-1FA8', 'address': 'BB', 'rssi': -80},
        {'name': 'SK-0001', 'address': 'DD', 'rssi': -90},
    ]


def test_sphero_prefixes_include_bolt():
    assert 'SB-' in ble_scan.sphero_prefixes()


@pytest.mark.parametrize('raw,expected', [
    (None, 4.0), ('0', 1.0), ('99', 10.0), ('abc', 4.0), ('2.5', 2.5),
    ('nan', 4.0),
])
def test_clamp_duration(raw, expected):
    assert ble_scan.clamp_duration(raw) == expected


# --- routes ---------------------------------------------------------------

def test_scan_requires_token():
    resp = _client(lambda d: ('ok', [])).get('/scan')
    assert resp.status_code == 401


def test_scan_ok_shape_and_clamp():
    seen = []

    def fake_scan(duration):
        seen.append(duration)
        return 'ok', [{'name': 'SB-3660', 'address': 'AA', 'rssi': -61}]

    resp = _client(fake_scan).get('/scan?duration=50', headers=AUTH)
    assert resp.status_code == 200
    assert resp.get_json() == {
        'worker': 'rpi4-test', 'duration_s': 10.0,
        'robots': [{'name': 'SB-3660', 'address': 'AA', 'rssi': -61}],
    }
    assert seen == [10.0]


def test_scan_default_duration():
    resp = _client(lambda d: ('ok', [])).get('/scan', headers=AUTH)
    assert resp.get_json()['duration_s'] == 4.0


def test_scan_busy_409():
    resp = _client(lambda d: ('busy', None)).get('/scan', headers=AUTH)
    assert resp.status_code == 409
    assert resp.get_json() == {'worker': 'rpi4-test', 'busy': True,
                               'error': 'BLE connect in progress'}


def test_scan_error_500():
    def boom(_d):
        raise RuntimeError('adapter down')

    resp = _client(boom).get('/scan', headers=AUTH)
    assert resp.status_code == 500
    assert resp.get_json() == {'worker': 'rpi4-test', 'error': 'adapter down'}


def test_scan_end_to_end_busy_when_lock_held(lock_path, monkeypatch):
    monkeypatch.setattr(ble_scan, 'LOCK_SLACK_S', -0.7)  # 1 s -> 0.3 s wait
    discover = MagicMock(return_value=[])

    def real_scan(duration):
        return ble_scan.scan(duration, discover_fn=discover)

    holder = _hold(lock_path)
    try:
        resp = _client(real_scan).get('/scan?duration=1', headers=AUTH)
    finally:
        holder.close()
    assert resp.status_code == 409
    discover.assert_not_called()


def test_scan_end_to_end_ok(lock_path):
    discover = MagicMock(return_value=[('SB-33E3', 'AA', -50),
                                       ('Speaker', 'BB', -40)])
    status, robots = ble_scan.scan(1.0, discover_fn=discover)
    assert status == 'ok'
    assert robots == [{'name': 'SB-33E3', 'address': 'AA', 'rssi': -50}]
    discover.assert_called_once_with(1.0)


def test_health_and_status_report_worker():
    manager = MagicMock()
    manager.status.return_value = {'spheros': [], 'count': 0}
    client = _client(lambda d: ('ok', []), manager=manager)
    assert client.get('/health', headers=AUTH).get_json() == {
        'status': 'ok', 'worker': 'rpi4-test'}
    assert client.get('/status', headers=AUTH).get_json() == {
        'spheros': [], 'count': 0, 'worker': 'rpi4-test'}
