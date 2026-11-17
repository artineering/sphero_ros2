#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Unit tests for the worker agent's ConnectQueue (no rclpy, fake clocks)."""

import types

import pytest

from sphero_worker_agent.connect_queue import ConnectQueue


@pytest.fixture
def q():
    clock = [100.0]
    sent, lines = [], []
    queue = ConnectQueue('rpi4-node01', sent.append, clock=lambda: clock[0],
                         wall=lambda: 1763000000.0 + clock[0],
                         entry_timeout=150, first_beacon=60, beacon_stale=10,
                         refresh=5, log=lines.append)
    return types.SimpleNamespace(q=queue, clock=clock, sent=sent, lines=lines)


def names(payload):
    return [p['name'] for p in payload['pending']]


def status(state, spawn_id=None):
    d = {'connection_state': state}
    if spawn_id is not None:
        d['spawn_id'] = spawn_id
    return d


def test_add_publishes_spawned_entry_and_logs_epoch(q):
    q.q.add('SB-A', 'id1')
    p = q.sent[-1]
    assert p['host'] == 'rpi4-node01' and p['seq'] == 1
    assert p['pending'] == [{'name': 'SB-A', 'spawn_id': 'id1',
                             'since': 1763000100.0, 'stage': 'spawned'}]
    assert q.lines[-1].startswith('[connect-queue] t=1763000100.000 host=rpi4-node01 add SB-A')


@pytest.mark.parametrize('state', ['connected', 'disconnected'])
def test_terminal_status_with_matching_id_removes(q, state):
    q.q.add('SB-A', 'id1')
    q.q.on_status('SB-A', status(state, 'id1'))
    assert names(q.sent[-1]) == []
    assert f'remove SB-A reason=status_{state}' in q.lines[-1]


def test_status_from_other_spawn_is_ignored_legacy_accepted(q):
    q.q.add('SB-A', 'id1')
    q.q.on_status('SB-A', status('connected', 'old'))
    assert names(q.sent[-1]) == ['SB-A']
    q.q.on_status('SB-A', status('connected'))  # no spawn_id: legacy controller
    assert names(q.sent[-1]) == []


def test_unknown_robot_ignored(q):
    q.q.on_status('SB-X', status('connected'))
    q.q.on_device_error('SB-X', {'error': 'toy_not_found'})
    assert q.sent == []


def test_stage_changes_publish_once(q):
    q.q.add('SB-A', 'id1')
    n = len(q.sent)
    q.q.on_status('SB-A', status('initializing', 'id1'))
    q.q.on_status('SB-A', status('initializing', 'id1'))
    q.q.on_status('SB-A', status('connecting', 'id1'))
    q.q.on_status('SB-A', status('connecting', 'id1'))
    assert len(q.sent) == n + 2
    assert q.sent[-1]['pending'][0]['stage'] == 'connecting'
    assert 'stage SB-A connecting' in q.lines[-1]


def test_device_error_removes(q):
    q.q.add('SB-A', 'id1')
    q.q.on_device_error('SB-A', {'error': 'toy_not_found', 'spawn_id': 'id1'})
    assert names(q.sent[-1]) == []
    assert 'reason=device_error_toy_not_found' in q.lines[-1]


def test_housekeep_process_exit(q):
    q.q.add('SB-A', 'id1')
    q.q.add('SB-B', 'id2')
    q.q.housekeep(lambda name: name != 'SB-A')
    assert names(q.sent[-1]) == ['SB-B']
    assert 'reason=process_exit' in q.lines[-1]


def test_housekeep_beacon_stale(q):
    q.q.add('SB-A', 'id1')
    q.q.on_status('SB-A', status('connecting', 'id1'))
    q.clock[0] += 10
    q.q.housekeep(lambda n: True)
    assert names(q.sent[-1]) == ['SB-A']
    q.clock[0] += 0.5
    q.q.housekeep(lambda n: True)
    assert names(q.sent[-1]) == []
    assert 'reason=beacon_stale' in q.lines[-1]


def test_housekeep_no_beacon_and_hard_timeout(q):
    q.q.add('SB-SILENT', 'id1')
    q.q.add('SB-SLOW', 'id2')
    for t in range(0, 150, 5):  # SB-SLOW keeps beaconing
        q.clock[0] = 100.0 + t
        q.q.on_status('SB-SLOW', status('connecting', 'id2'))
        q.q.housekeep(lambda n: True)
        if t == 65:  # first tick past FIRST_BEACON (60 s)
            assert names(q.sent[-1]) == ['SB-SLOW']
    assert any('remove SB-SILENT reason=no_beacon' in x for x in q.lines)
    q.clock[0] = 100.0 + 150.5
    q.q.on_status('SB-SLOW', status('connecting', 'id2'))
    q.q.housekeep(lambda n: True)
    assert names(q.sent[-1]) == []
    assert 'remove SB-SLOW reason=timeout' in q.lines[-1]


def test_housekeep_refreshes_every_5s(q):
    q.q.publish_now()
    n = len(q.sent)
    q.clock[0] += 4
    q.q.housekeep(lambda n: True)
    assert len(q.sent) == n
    q.clock[0] += 1
    q.q.housekeep(lambda n: True)
    assert len(q.sent) == n + 1 and q.sent[-1]['seq'] == n + 1


def test_respawn_ignores_late_messages_from_old_process(q):
    q.q.add('SB-A', 'id1')
    q.q.remove('SB-A', 'teardown', 'id1')
    q.q.add('SB-A', 'id2')
    q.q.on_status('SB-A', status('disconnected', 'id1'))
    q.q.on_device_error('SB-A', {'error': 'ble_lost', 'spawn_id': 'id1'})
    assert names(q.sent[-1]) == ['SB-A']
    assert not q.q.remove('SB-A', 'teardown', 'id1')  # stale reap
    assert q.q.remove('SB-A', 'teardown', 'id2')


def test_housekeep_calls_is_alive_without_holding_lock(q):
    q.q.add('SB-A', 'id1')

    def is_alive(name):
        q.q.add('SB-B', 'id2')  # would deadlock if the queue lock were held
        return True
    q.q.housekeep(is_alive)
    assert set(names(q.sent[-1])) == {'SB-A', 'SB-B'}


def test_publish_failure_is_logged_not_raised(q):
    def boom(_payload):
        raise RuntimeError('no context')
    queue = ConnectQueue('h', boom, log=q.lines.append)
    queue.add('SB-A', 'id1')
    assert 'publish failed' in q.lines[-1]
