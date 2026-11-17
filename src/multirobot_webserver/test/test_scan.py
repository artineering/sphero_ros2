#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Hardware-free tests for the BLE scan fan-out (GET /api/scan): merge logic,
per-worker /scan client, and the manager fan-out. Worker HTTP is mocked; the
manager methods are bound to SimpleNamespace stubs -- no rclpy init.
"""

import types
from unittest import mock

import requests

from multirobot_webserver import multirobot_webapp as webapp
from multirobot_webserver.multirobot_webapp import (
    SpheroInstanceManager,
    merge_scan_results,
    SCAN_TIMEOUT_MARGIN,
)
from multirobot_webserver.worker_registry import WorkerRegistry


def _registry():
    return WorkerRegistry.from_config({
        'agent': {'token': 'tok'},
        'workers': [{'name': f'rpi4-node0{i}', 'host': f'10.0.0.1{i}'}
                    for i in range(1, 5)],
    })


def _mgr(registry=None, instances=(), roster=()):
    mgr = types.SimpleNamespace(
        worker_registry=registry,
        instances={n: {} for n in instances},
        fleet_node=types.SimpleNamespace(robots={n: {} for n in roster}),
    )
    mgr._agent_headers = lambda: SpheroInstanceManager._agent_headers(mgr)
    mgr._agent_scan = (
        lambda w, d: SpheroInstanceManager._agent_scan(mgr, w, d))
    return mgr


def _resp(status, payload=None, bad_json=False):
    r = mock.Mock(status_code=status)
    if bad_json:
        r.json.side_effect = ValueError('no json')
    else:
        r.json.return_value = payload
    return r


def _ok(worker, robots):
    return {'name': worker, 'ok': True, 'busy': False, 'error': None,
            'robots': robots}


# --- merge_scan_results ------------------------------------------------------

def test_merge_keeps_strongest_rssi_and_best_worker():
    out = merge_scan_results([
        _ok('w1', [{'name': 'SB-3660', 'rssi': -80}]),
        _ok('w2', [{'name': 'SB-3660', 'rssi': -55}]),
    ], exclude=set())
    assert len(out) == 1
    assert out[0]['rssi'] == -55
    assert out[0]['best_worker'] == 'w2'
    assert out[0]['seen_by'] == [{'worker': 'w2', 'rssi': -55},
                                 {'worker': 'w1', 'rssi': -80}]


def test_merge_drops_excluded_and_sorts_strongest_first():
    out = merge_scan_results([
        _ok('w1', [{'name': 'SB-A', 'rssi': -90},
                   {'name': 'SB-B', 'rssi': -40},
                   {'name': 'SB-GONE', 'rssi': -30}]),
    ], exclude={'SB-GONE'})
    assert [r['name'] for r in out] == ['SB-B', 'SB-A']


def test_merge_missing_rssi_sorts_last_and_malformed_skipped():
    out = merge_scan_results([
        _ok('w1', [{'name': 'SB-N'}, {'name': 'SB-X', 'rssi': -70},
                   'garbage', {'rssi': -10}, {'name': '  '}]),
    ], exclude=set())
    assert [r['name'] for r in out] == ['SB-X', 'SB-N']
    assert out[1]['rssi'] is None and out[1]['best_worker'] == 'w1'


def test_merge_ignores_failed_workers():
    failed = {'name': 'w1', 'ok': False, 'busy': True, 'error': 'busy',
              'robots': [{'name': 'SB-A', 'rssi': -10}]}
    assert merge_scan_results([failed], exclude=set()) == []


# --- _agent_scan -------------------------------------------------------------

def test_agent_scan_ok_keys_by_registry_worker_and_sends_auth_timeout():
    reg = _registry()
    w = reg.get('rpi4-node02')
    mgr = _mgr(reg)
    payload = {'worker': 'some-other-hostname', 'duration_s': 4.0,
               'robots': [{'name': 'SB-1', 'address': 'x', 'rssi': -60}]}
    with mock.patch.object(webapp.requests, 'get',
                           return_value=_resp(200, payload)) as get:
        out = mgr._agent_scan(w, 4.0)
    assert out['ok'] and out['name'] == 'rpi4-node02'
    assert out['robots'] == payload['robots']
    _, kwargs = get.call_args
    assert get.call_args[0][0] == 'http://10.0.0.12:8181/scan'
    assert kwargs['params'] == {'duration': 4.0}
    assert kwargs['headers'] == {'Authorization': 'Bearer tok'}
    assert kwargs['timeout'][1] == 2 * 4.0 + SCAN_TIMEOUT_MARGIN
    assert kwargs['timeout'][1] >= 2 * 4.0 + 7


def test_agent_scan_409_is_busy():
    reg = _registry()
    with mock.patch.object(webapp.requests, 'get', return_value=_resp(
            409, {'busy': True, 'error': 'BLE connect in progress'})):
        out = _mgr(reg)._agent_scan(reg.get('rpi4-node01'), 4.0)
    assert not out['ok'] and out['busy']
    assert out['error'] == 'BLE connect in progress'


def test_agent_scan_500_reports_payload_error():
    reg = _registry()
    with mock.patch.object(webapp.requests, 'get',
                           return_value=_resp(500, {'error': 'hci0 down'})):
        out = _mgr(reg)._agent_scan(reg.get('rpi4-node01'), 4.0)
    assert not out['ok'] and not out['busy'] and out['error'] == 'hci0 down'


def test_agent_scan_404_without_json():
    reg = _registry()
    with mock.patch.object(webapp.requests, 'get',
                           return_value=_resp(404, bad_json=True)):
        out = _mgr(reg)._agent_scan(reg.get('rpi4-node01'), 4.0)
    assert not out['ok'] and out['error'] == 'HTTP 404'


def test_agent_scan_bad_json_200():
    reg = _registry()
    with mock.patch.object(webapp.requests, 'get',
                           return_value=_resp(200, bad_json=True)):
        out = _mgr(reg)._agent_scan(reg.get('rpi4-node01'), 4.0)
    assert not out['ok'] and out['error'] == 'bad response'


def test_agent_scan_unreachable_never_raises():
    reg = _registry()
    for exc in (requests.ConnectionError('x'), requests.Timeout('x')):
        with mock.patch.object(webapp.requests, 'get', side_effect=exc):
            out = _mgr(reg)._agent_scan(reg.get('rpi4-node01'), 4.0)
        assert not out['ok'] and out['error'].startswith('unreachable')


# --- scan_nearby ---------------------------------------------------------------

def _fake_get(url, **kwargs):
    if '10.0.0.11' in url:
        return _resp(200, {'robots': [{'name': 'SB-3660', 'rssi': -70},
                                      {'name': 'SB-DEPLOYED', 'rssi': -40}]})
    if '10.0.0.12' in url:
        return _resp(200, {'robots': [{'name': 'SB-3660', 'rssi': -50},
                                      {'name': 'SB-ROSTER', 'rssi': -45},
                                      {'name': 'SB-1FA8', 'rssi': -65}]})
    if '10.0.0.13' in url:
        return _resp(409, {'busy': True, 'error': 'BLE connect in progress'})
    raise requests.Timeout('slow')


def test_scan_nearby_mixed_workers_never_fails():
    mgr = _mgr(_registry(), instances=['SB-DEPLOYED'], roster=['SB-ROSTER'])
    with mock.patch.object(webapp.requests, 'get', side_effect=_fake_get):
        out = SpheroInstanceManager.scan_nearby(mgr, 4.0)
    assert [r['name'] for r in out['robots']] == ['SB-3660', 'SB-1FA8']
    assert out['robots'][0]['best_worker'] == 'rpi4-node02'
    workers = {w['name']: w for w in out['workers']}
    assert [w['name'] for w in out['workers']] == [
        'rpi4-node01', 'rpi4-node02', 'rpi4-node03', 'rpi4-node04']
    assert workers['rpi4-node01']['ok'] and workers['rpi4-node02']['ok']
    assert workers['rpi4-node03']['busy']
    assert workers['rpi4-node04']['error'].startswith('unreachable')
    assert set(out['workers'][0]) == {'name', 'ok', 'busy', 'error'}


def test_scan_nearby_clamps_duration():
    mgr = _mgr(_registry())
    for asked, sent in ((0, 1.0), (99, 10.0), (float('inf'), 10.0)):
        with mock.patch.object(webapp.requests, 'get',
                               return_value=_resp(200, {'robots': []})) as get:
            out = SpheroInstanceManager.scan_nearby(mgr, asked)
        assert out['duration_s'] == sent
        assert get.call_args[1]['params'] == {'duration': sent}


def test_scan_nearby_without_registry_makes_no_calls():
    mgr = _mgr(None)
    with mock.patch.object(webapp.requests, 'get') as get:
        out = SpheroInstanceManager.scan_nearby(mgr, 4.0)
    assert out == {'registry_loaded': False, 'robots': [], 'workers': []}
    get.assert_not_called()


# --- route -------------------------------------------------------------------

def test_scan_route_passes_duration_and_defaults_bad_input():
    client = webapp.app.test_client()
    with mock.patch.object(webapp.manager, 'scan_nearby',
                           return_value={'robots': [], 'workers': []}) as scan:
        r = client.get('/api/scan?duration=6')
        assert r.status_code == 200 and r.get_json()['success'] is True
        scan.assert_called_with(6.0)
        client.get('/api/scan?duration=abc')
        scan.assert_called_with(webapp.SCAN_DEFAULT_SEC)
        client.get('/api/scan?duration=nan')
        scan.assert_called_with(webapp.SCAN_DEFAULT_SEC)
        client.get('/api/scan')
        scan.assert_called_with(webapp.SCAN_DEFAULT_SEC)
