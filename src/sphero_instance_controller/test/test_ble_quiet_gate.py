"""
Unit tests for the per-host BLE quiet gate in the device controller node.

A controller holds a shared flock on the pending-connect file until its
initial connect ends; a connected robot stays BLE-quiet (stream stopped
inside the connect lock, no LEDs / battery / liveness round trips) until the
file has read free for BLE_QUIET_SETTLE seconds, or BLE_QUIET_MAX_HOLD passed.

Reuses the rclpy-free node fixture from test_ble_liveness.py.
"""
from contextlib import contextmanager
import fcntl
import json
import subprocess
import sys
import types
from unittest.mock import MagicMock

import pytest

pytest.importorskip('rclpy')

from spherov2.toy.bolt import BOLT  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from test_ble_liveness import (  # noqa: E402,F401  (node is a fixture)
    _mask, _streaming_toy, dev, node, states)


@pytest.fixture
def pending(tmp_path, monkeypatch):
    path = tmp_path / 'pending.lock'
    monkeypatch.setenv('SPHERO_BLE_PENDING_LOCK', str(path))
    return path


@contextmanager
def holder(path):
    """Another controller still connecting (a separate open file description)."""
    fd = open(path, 'a')
    fcntl.flock(fd, fcntl.LOCK_SH)
    try:
        yield
    finally:
        fd.close()


@pytest.fixture
def gated(node, pending, monkeypatch):  # noqa: F811
    """Return the node with the gate on, a fake clock and the BLE setup mocked."""
    monkeypatch.setattr(dev, 'BLE_QUIET_GATE', True)
    monkeypatch.setattr(dev, 'BLE_QUIET_SETTLE', 3.0)
    monkeypatch.setattr(dev, 'BLE_QUIET_MAX_HOLD', 60.0)
    # The flock source: these tests cover the fallback path; the queue
    # source is covered further down (queue_gated).
    monkeypatch.setattr(dev, 'BLE_GATE_SOURCE', 'flock')
    monkeypatch.setattr(dev, 'BLE_HOLD_KEEPALIVE', 0.0)  # covered separately
    clock = [1000.0]
    monkeypatch.setattr(dev, 'time', types.SimpleNamespace(
        monotonic=lambda: clock[0], sleep=lambda _s: None))
    node._apply_sensor_stream = MagicMock()
    node.sphero.set_led = MagicMock()
    node.sphero.set_matrix = MagicMock()
    node.sphero.update_battery = MagicMock()
    node.sphero.robot.is_connected = True
    return types.SimpleNamespace(node=node, clock=clock, path=pending)


def advance(g, seconds):
    g.clock[0] += seconds
    g.node._quiet_gate_tick()


def setup_calls(n):
    return (n._apply_sensor_stream.call_count, n.sphero.set_led.call_count,
            n.sphero.update_battery.call_count)


def logs(n):
    return [c.args[0] for c in n.get_logger().info.call_args_list]


def released_log(n):
    """The last 'released (...)' line (startup-complete is logged after it)."""
    return [m for m in logs(n) if ' released (' in m][-1]


# ----- pending-connect marker -----

def test_pending_connects_tracks_marker(pending):
    assert not dev.pending_connects()
    with dev.pending_connect_marker('SB-A'):
        assert dev.pending_connects()
        with dev.pending_connect_marker('SB-B'):
            assert dev.pending_connects()
        assert dev.pending_connects()
    assert not dev.pending_connects()


def test_marker_released_on_exception(pending):
    with pytest.raises(RuntimeError):
        with dev.pending_connect_marker('SB-A'):
            raise RuntimeError('connect blew up')
    assert not dev.pending_connects()


def test_marker_of_killed_process_is_dropped(pending):
    code = ('import fcntl, sys, time; f = open(sys.argv[1], "a"); '
            'fcntl.flock(f, fcntl.LOCK_SH); print("held", flush=True); time.sleep(60)')
    proc = subprocess.Popen([sys.executable, '-c', code, str(pending)],
                            stdout=subprocess.PIPE, text=True)
    try:
        assert proc.stdout.readline().strip() == 'held'
        assert dev.pending_connects()
    finally:
        proc.kill()
        proc.wait()
    assert not dev.pending_connects()


def test_unopenable_marker_fails_open(tmp_path, monkeypatch):
    monkeypatch.setenv('SPHERO_BLE_PENDING_LOCK', str(tmp_path / 'missing' / 'x.lock'))
    with dev.pending_connect_marker('SB-A'):
        assert not dev.pending_connects()


# ----- hold / release -----

def test_hold_defers_setup_while_neighbours_connect(gated):
    n = gated.node
    with holder(gated.path):
        n._quiet_gate_enter()
        assert n._quiet_hold
        assert setup_calls(n) == (0, 0, 0)
        assert 'waiting on flock holders (source=flock)' in logs(n)[-1]


def test_hold_silences_probe_sensors_and_battery(gated):
    n = gated.node
    with holder(gated.path):
        n._quiet_gate_enter()
        n._ble_liveness_probe()
        n.publish_sensors()
        n.publish_heartbeat()
    n.sphero.check_link.assert_not_called()
    n.state_pub.publish.assert_not_called()
    n.sensor_pub.publish.assert_not_called()
    n.sphero.update_battery.assert_not_called()
    assert states(n) == ['connected']  # heartbeat /status still goes out


def test_release_needs_queue_empty_for_whole_settle(gated):
    n = gated.node
    with holder(gated.path):
        n._quiet_gate_enter()
        advance(gated, 10)
    advance(gated, 0.5)            # queue empty: settle starts
    advance(gated, 2.0)
    assert n._quiet_hold
    with holder(gated.path):       # a late arrival resets the settle
        advance(gated, 0.5)
    advance(gated, 0.5)            # settle restarts here
    advance(gated, 2.5)
    assert n._quiet_hold
    advance(gated, 0.5)
    assert not n._quiet_hold
    assert setup_calls(n) == (1, 3, 1)
    n.sphero.set_matrix.assert_called_once_with('arrow_right', None, 128, 0, 256)
    assert states(n)[-1] == 'connected'
    assert 'released (reason=queue_empty, source=flock) after 16.5 s' in released_log(n)


def test_no_pending_connects_starts_after_settle(gated):
    n = gated.node
    n._quiet_gate_enter()
    assert 'no pending connects' in logs(n)[-1]
    advance(gated, 2.5)
    assert n._quiet_hold
    advance(gated, 0.5)
    assert not n._quiet_hold
    assert setup_calls(n) == (1, 3, 1)


def test_max_hold_releases_with_timeout(gated):
    n = gated.node
    with holder(gated.path):
        n._quiet_gate_enter()
        advance(gated, dev.BLE_QUIET_MAX_HOLD - 1)
        assert n._quiet_hold
        advance(gated, 1)
        assert not n._quiet_hold
    assert 'released (reason=timeout, source=flock) after 60.0 s' in released_log(n)
    assert setup_calls(n) == (1, 3, 1)


def test_released_robot_is_never_held_again(gated):
    n = gated.node
    n._quiet_gate_enter()
    advance(gated, dev.BLE_QUIET_SETTLE)
    assert not n._quiet_hold
    with holder(gated.path):       # a late spawn / neighbour reconnect
        for _ in range(5):
            advance(gated, 1)
            n.publish_heartbeat()
        assert not n._quiet_hold
    assert n.sphero.update_battery.call_count == 6  # release + 5 heartbeats


def test_commands_pass_through_during_hold(gated):
    n = gated.node
    with holder(gated.path):
        n._quiet_gate_enter()
        n.led_callback(String(data=json.dumps({'red': 1, 'green': 2, 'blue': 3})))
    n.sphero.set_led.assert_called_once_with(1, 2, 3, 'main')
    assert n._quiet_hold  # a command doesn't end the hold


def test_tick_paused_while_link_down(gated):
    n = gated.node
    with holder(gated.path):
        n._quiet_gate_enter()
        n.ble_link_down = True
        advance(gated, dev.BLE_QUIET_MAX_HOLD + 5)
    assert n._quiet_hold
    assert setup_calls(n) == (0, 0, 0)


def test_reconnect_holds_while_neighbours_connect(gated):
    n = gated.node
    n._quiet_gate_enter()
    advance(gated, dev.BLE_QUIET_SETTLE)
    n.mark_link_down('test')
    before = setup_calls(n)
    with holder(gated.path):
        n.rebind_connection(MagicMock(), MagicMock())
        assert n._quiet_hold
        assert setup_calls(n) == before
        assert states(n)[-1] == 'connected'
        advance(gated, 30)
        assert n._quiet_hold  # the hold restarted at rebind
    advance(gated, 1)
    advance(gated, dev.BLE_QUIET_SETTLE)
    assert not n._quiet_hold
    n.stop_reconnect_beacon()


def test_gate_disabled_starts_immediately(gated, monkeypatch):
    monkeypatch.setattr(dev, 'BLE_QUIET_GATE', False)
    n = gated.node
    with holder(gated.path):
        n._quiet_gate_enter()
    assert not n._quiet_hold
    assert setup_calls(n) == (1, 3, 1)


# ----- stream stop inside the connect lock -----

def test_quiet_stream_stops_and_release_restarts_trimmed(node, monkeypatch):  # noqa: F811
    monkeypatch.setattr(dev, 'BLE_QUIET_GATE', True)
    toy = _streaming_toy(BOLT)
    assert dev.quiet_sensor_stream(toy, ['attitude'], 500)
    assert toy.set_sensor_streaming_mask.call_args.args[2] == 0
    toy.set_extended_sensor_streaming_mask.assert_called_once_with(0)
    node.sphero.robot = toy
    node._apply_sensor_stream()
    assert toy.set_sensor_streaming_mask.call_args.args == (
        500, 0, _mask(BOLT, 'attitude'))


@pytest.mark.parametrize('wanted, interval', [
    ([dev.SENSOR_STREAM_KEEP_DEFAULT], 500),
    (['bogus'], 500),
    (['attitude'], 0),
    ([], 500),
])
def test_quiet_stream_skipped_when_node_would_not_restart_it(monkeypatch, wanted, interval):
    monkeypatch.setattr(dev, 'BLE_QUIET_GATE', True)
    toy = _streaming_toy(BOLT)
    assert not dev.quiet_sensor_stream(toy, wanted, interval)
    toy.set_sensor_streaming_mask.assert_not_called()


def test_quiet_stream_skipped_when_gate_disabled(monkeypatch):
    monkeypatch.setattr(dev, 'BLE_QUIET_GATE', False)
    toy = _streaming_toy(BOLT)
    assert not dev.quiet_sensor_stream(toy, ['attitude'], 500)
    toy.set_sensor_streaming_mask.assert_not_called()


def test_quiet_stream_never_raises(monkeypatch):
    monkeypatch.setattr(dev, 'BLE_QUIET_GATE', True)
    toy = _streaming_toy(BOLT)
    toy.sensor_control.disable_all = MagicMock(side_effect=RuntimeError('link gone'))
    assert not dev.quiet_sensor_stream(toy, ['attitude'], 500)


def test_reconnect_stops_stream_inside_lock(node, monkeypatch):  # noqa: F811
    events = []

    @contextmanager
    def lock(label=''):
        events.append('lock')
        yield
        events.append('unlock')

    new_robot, cm = MagicMock(), MagicMock()
    monkeypatch.setattr(dev, 'BLE_RECONNECT_BACKOFF', 0.0)
    monkeypatch.setattr(dev, 'ble_connect_lock', lock)
    monkeypatch.setattr(dev.scanner, 'find_toy', MagicMock(return_value=new_robot))
    monkeypatch.setattr(dev, 'SpheroEduAPI', MagicMock(return_value=cm))
    cm.__enter__.side_effect = lambda: events.append('connect') or MagicMock()
    monkeypatch.setattr(dev, 'quiet_sensor_stream',
                        lambda robot, wanted, ms: events.append(('quiet', robot, wanted, ms)))
    node.rebind_connection = MagicMock(side_effect=lambda *a: events.append('rebind'))
    node.mark_link_down('test')
    got_cm, err = dev._reconnect(node, 'SB-TEST')
    assert got_cm is cm and err is None
    assert events == ['lock', 'connect', ('quiet', new_robot, ['attitude'], 500),
                      'unlock', 'rebind']
    node.stop_reconnect_beacon()


# ----- worker-agent connect queue (source=auto) -----

@pytest.fixture
def queue_gated(gated, monkeypatch):
    """The gated node on the queue source; the flock fallback is a fake."""
    monkeypatch.setattr(dev, 'BLE_GATE_SOURCE', 'auto')
    monkeypatch.setattr(dev, 'BLE_QUIET_SETTLE', 1.0)
    monkeypatch.setattr(dev, 'BLE_QUIET_MAX_HOLD', 120.0)
    monkeypatch.setattr(dev, 'BLE_QUEUE_WAIT', 5.0)
    monkeypatch.setattr(dev, 'BLE_QUEUE_STALE', 15.0)
    flock = types.SimpleNamespace(held=False, calls=0)

    def fake_pending():
        flock.calls += 1
        return flock.held
    monkeypatch.setattr(dev, 'pending_connects', fake_pending)
    gated.flock = flock
    return gated


def queue(n, *names):
    """Deliver a connect_queue message listing names (as the agent would)."""
    n._on_connect_queue(String(data=json.dumps({
        'host': 'rpi4-node01', 'seq': 1, 'stamp': 0.0,
        'pending': [{'name': x, 'spawn_id': 'id-' + x, 'since': 0.0,
                     'stage': 'spawned'} for x in names]})))


def test_queue_holds_while_others_listed(queue_gated):
    n = queue_gated.node
    queue(n, 'SB-TEST', 'SB-OTHER')
    n._quiet_gate_enter()
    assert n._quiet_hold
    assert 'waiting on [SB-OTHER] (source=queue)' in logs(n)[-1]
    for _ in range(6):
        advance(queue_gated, 5)
        queue(n, 'SB-TEST', 'SB-OTHER')  # the agent's 5 s refresh
    assert n._quiet_hold
    assert setup_calls(n) == (0, 0, 0)
    assert queue_gated.flock.calls == 0  # the queue is authoritative


def test_queue_listing_only_self_releases_after_settle(queue_gated):
    n = queue_gated.node
    queue(n, 'SB-TEST')  # the agent hasn't seen our 'connected' yet
    n._quiet_gate_enter()
    assert 'no pending connects (source=queue)' in logs(n)[-1]
    advance(queue_gated, 0.5)
    assert n._quiet_hold
    advance(queue_gated, 0.5)
    assert not n._quiet_hold
    assert setup_calls(n) == (1, 3, 1)
    assert 'released (reason=queue_empty, source=queue) after 1.0 s' in released_log(n)


def test_queue_change_is_logged_once_and_readd_resets_settle(queue_gated):
    n = queue_gated.node
    queue(n, 'SB-A', 'SB-B')
    n._quiet_gate_enter()
    queue(n, 'SB-B')
    advance(queue_gated, 2)
    advance(queue_gated, 2)
    changes = [m for m in logs(n) if 'now waiting on' in m]
    assert changes == ['[ble-gate] SB-TEST now waiting on [SB-B] (source=queue) after 2.0 s']
    queue(n)
    advance(queue_gated, 0.5)      # empty: settle starts
    queue(n, 'SB-C')               # a respawn lands inside the settle
    advance(queue_gated, 0.5)
    queue(n)
    advance(queue_gated, 0.5)      # settle restarts here
    advance(queue_gated, 0.5)
    assert n._quiet_hold
    advance(queue_gated, 0.5)
    assert not n._quiet_hold


def test_no_queue_yet_stays_quiet_then_falls_back_to_flock(queue_gated):
    n = queue_gated.node
    queue_gated.flock.held = True
    n._quiet_gate_enter()
    assert 'waiting on first connect_queue message (source=awaiting_queue)' in logs(n)[-1]
    for _ in range(9):
        advance(queue_gated, 0.5)  # 4.5 s: still inside QUEUE_WAIT
    assert n._quiet_hold and queue_gated.flock.calls == 0
    advance(queue_gated, 0.5)
    assert n._quiet_hold and queue_gated.flock.calls == 1
    assert 'waiting on flock holders (source=flock)' in logs(n)[-1]
    queue_gated.flock.held = False
    advance(queue_gated, 0.5)
    advance(queue_gated, 1.0)
    assert not n._quiet_hold
    assert 'source=flock' in released_log(n)


def test_late_queue_inside_wait_takes_over(queue_gated):
    n = queue_gated.node
    n._quiet_gate_enter()
    advance(queue_gated, 2)
    queue(n, 'SB-OTHER')
    advance(queue_gated, 10)
    assert n._quiet_hold and queue_gated.flock.calls == 0


def test_stale_queue_falls_back_to_flock(queue_gated):
    n = queue_gated.node
    queue(n, 'SB-OTHER')
    n._quiet_gate_enter()
    advance(queue_gated, 15)
    assert n._quiet_hold and queue_gated.flock.calls == 0
    advance(queue_gated, 0.5)      # agent silent > QUEUE_STALE: dead agent
    assert queue_gated.flock.calls == 1
    advance(queue_gated, 1.0)
    assert not n._quiet_hold


def test_source_flock_ignores_queue(queue_gated, monkeypatch):
    monkeypatch.setattr(dev, 'BLE_GATE_SOURCE', 'flock')
    n = queue_gated.node
    queue(n, 'SB-OTHER')
    n._quiet_gate_enter()
    advance(queue_gated, 0.5)
    advance(queue_gated, 0.5)
    assert not n._quiet_hold


def test_source_queue_never_uses_flock(queue_gated, monkeypatch):
    monkeypatch.setattr(dev, 'BLE_GATE_SOURCE', 'queue')
    n = queue_gated.node
    queue_gated.flock.held = True
    n._quiet_gate_enter()
    advance(queue_gated, 5)
    advance(queue_gated, 1)
    assert not n._quiet_hold
    assert queue_gated.flock.calls == 0


def test_malformed_queue_is_ignored(queue_gated):
    n = queue_gated.node
    for bad in ('not json', '[]', '{"pending": "x"}'):
        n._on_connect_queue(String(data=bad))
    assert n._queue_rx is None
    assert n.get_logger().warning.called


def test_queue_max_hold_times_out_at_120(queue_gated):
    n = queue_gated.node
    queue(n, 'SB-STUCK')
    n._quiet_gate_enter()
    for _ in range(11):
        advance(queue_gated, 10)
        queue(n, 'SB-STUCK')       # agent refreshes every 5 s
    assert n._quiet_hold
    advance(queue_gated, 10)
    assert not n._quiet_hold
    assert 'released (reason=timeout, source=queue) after 120.0 s' in released_log(n)


def test_default_hold_and_settle():
    import os
    if 'SPHERO_BLE_QUIET_MAX_HOLD' not in os.environ:
        assert dev.BLE_QUIET_MAX_HOLD == 120.0
    if 'SPHERO_BLE_QUIET_SETTLE' not in os.environ:
        assert dev.BLE_QUIET_SETTLE == 1.0


def test_status_payload_reports_holding_and_spawn_id(queue_gated):
    n = queue_gated.node
    n.spawn_id = 'abc123'
    queue(n, 'SB-OTHER')
    n._quiet_gate_enter()
    n.publish_connection_state('connected')
    payload = json.loads(n.status_pub.publish.call_args.args[0].data)
    assert payload['holding'] is True and payload['spawn_id'] == 'abc123'
    queue(n)
    advance(queue_gated, 0.5)
    advance(queue_gated, 1.0)
    payload = json.loads(n.status_pub.publish.call_args.args[0].data)
    assert payload['connection_state'] == 'connected'
    assert payload['holding'] is False


# ----- pre-connect /status beacon -----

def _beacon_states(pub):
    return [json.loads(c.args[0].data) for c in pub.publish.call_args_list]


def test_preconnect_beacon_stages_and_repeats(monkeypatch):
    import time as real_time
    monkeypatch.setattr(dev, 'BLE_PRECONNECT_STATUS', True)
    pub = MagicMock()
    b = dev.PreConnectBeacon(pub, 'SB-TEST', 'sid', period=0.02)
    b.set_stage('initializing')
    first = _beacon_states(pub)[0]
    assert first['connection_state'] == 'initializing'
    assert first['spawn_id'] == 'sid' and first['holding'] is False
    assert first['sphero_name'] == 'SB-TEST'
    b.set_stage('connecting')
    real_time.sleep(0.1)
    b.stop()
    got = [p['connection_state'] for p in _beacon_states(pub)]
    assert got[0] == 'initializing' and got[1] == 'connecting'
    assert got.count('connecting') >= 3  # repeated while waiting
    n = pub.publish.call_count
    real_time.sleep(0.05)
    assert pub.publish.call_count == n   # stopped


def test_preconnect_beacon_disabled(monkeypatch):
    monkeypatch.setattr(dev, 'BLE_PRECONNECT_STATUS', False)
    pub = MagicMock()
    b = dev.PreConnectBeacon(pub, 'SB-TEST', None, period=0.01)
    b.set_stage('initializing')
    b.stop()
    pub.publish.assert_not_called()


# ----- fleet hold (/fleet/ble_release) -----

CONNECTED_AT = 1763300000.0


@pytest.fixture
def fleet(queue_gated, monkeypatch):
    """A fleet-hold robot (spawn_id 'sid-me') on the queue source."""
    monkeypatch.setattr(dev, 'BLE_FLEET_HOLD_MAX', 900.0)
    monkeypatch.setattr(dev, 'BLE_FLEET_RELEASE_SKEW', 2.0)
    n = queue_gated.node
    n.spawn_id = 'sid-me'
    n._fleet_hold = True
    n._connected_at = CONNECTED_AT
    return queue_gated


def release(n, spawn_ids, stamp=CONNECTED_AT + 60, seq=1, reason='deploy_complete'):
    n._on_fleet_release(String(data=json.dumps(
        {'seq': seq, 'stamp': stamp, 'spawn_ids': spawn_ids, 'reason': reason})))


def test_fleet_hold_ignores_empty_queue_and_max_hold(fleet):
    n = fleet.node
    queue(n)                       # nobody else connecting on this Pi
    n._quiet_gate_enter()
    assert n._quiet_hold
    assert 'fleet hold: waiting for /fleet/ble_release naming spawn_id=sid-me' in logs(n)[-1]
    for _ in range(60):            # 300 s: past the 120 s per-host cap
        advance(fleet, 5)
        queue(n)
        n.publish_heartbeat()
    assert n._quiet_hold
    assert setup_calls(n) == (0, 0, 0)
    payload = json.loads(n.status_pub.publish.call_args.args[0].data)
    assert payload['connection_state'] == 'connected' and payload['holding'] is True


def test_fleet_release_naming_us_releases_at_once(fleet):
    n = fleet.node
    queue(n)
    n._quiet_gate_enter()
    advance(fleet, 100)
    queue(n)
    release(n, ['sid-other'])      # not us: still holding, no warning
    assert n._quiet_hold
    assert not n.get_logger().warning.called
    release(n, ['sid-other', 'sid-me'], seq=4)
    assert not n._quiet_hold
    assert setup_calls(n) == (1, 3, 1)
    text = logs(n)
    assert any('fleet release (reason=deploy_complete, seq=4, match=named) after 100.0 s held'
               in m for m in text)
    assert 'released (reason=fleet_deploy_complete, source=queue)' in released_log(n)
    payload = json.loads(n.status_pub.publish.call_args.args[0].data)
    assert payload['holding'] is False


def test_release_all_star_releases(fleet):
    n = fleet.node
    queue(n)
    n._quiet_gate_enter()
    release(n, ['*'], stamp=CONNECTED_AT + 5, reason='manual')
    assert not n._quiet_hold
    assert any('match=release_all' in m for m in logs(n))


def test_stale_release_all_is_ignored(fleet):
    n = fleet.node
    queue(n)
    n._quiet_gate_enter()
    release(n, ['*'], stamp=CONNECTED_AT - 300)   # latched from an earlier deploy
    assert n._quiet_hold and n._fleet_hold
    warn = n.get_logger().warning.call_args.args[0]
    assert 'stale_release_all (300.0 s before connect)' in warn
    release(n, ['*'], stamp=CONNECTED_AT - 1.5)   # inside the 2 s skew
    assert not n._quiet_hold


def test_old_named_release_never_matches_new_spawn(fleet):
    n = fleet.node
    queue(n)
    n._quiet_gate_enter()
    release(n, ['sid-previous-process'], stamp=CONNECTED_AT + 999)
    assert n._quiet_hold


def test_fleet_cap_releases_with_warning(fleet):
    n = fleet.node
    queue(n)
    n._quiet_gate_enter()
    for _ in range(179):
        advance(fleet, 5)
        queue(n)
    assert n._quiet_hold
    advance(fleet, 5)
    assert not n._quiet_hold
    assert 'fleet hold cap 900 s reached' in n.get_logger().warning.call_args.args[0]
    assert 'released (reason=fleet_cap' in released_log(n)


def test_fleet_release_hands_over_to_per_host_gate(fleet):
    n = fleet.node
    queue(n, 'SB-NEIGHBOUR')       # a neighbour is still connecting on this Pi
    n._quiet_gate_enter()
    release(n, ['sid-me'])
    assert n._quiet_hold and not n._fleet_hold
    assert 'per-host gate still waiting on [SB-NEIGHBOUR]' in logs(n)[-1]
    queue(n)
    advance(fleet, 0.5)
    advance(fleet, 1.0)
    assert not n._quiet_hold


def test_after_fleet_release_reconnect_uses_per_host_rule(fleet):
    n = fleet.node
    queue(n)
    n._quiet_gate_enter()
    release(n, ['sid-me'])
    n.mark_link_down('test')
    queue(n)
    n.rebind_connection(MagicMock(), MagicMock())
    assert 'no pending connects (source=queue)' in logs(n)[-2]
    assert not n._fleet_hold
    advance(fleet, 0.5)
    advance(fleet, 1.0)
    assert not n._quiet_hold
    n.stop_reconnect_beacon()


def test_reconnect_during_fleet_hold_keeps_holding(fleet):
    n = fleet.node
    queue(n)
    n._quiet_gate_enter()
    advance(fleet, 50)
    n.mark_link_down('test')
    n.rebind_connection(MagicMock(), MagicMock())
    assert n._quiet_hold and n._fleet_hold
    assert '(50.0 s held so far)' in [m for m in logs(n) if 'fleet hold' in m][-1]
    n.stop_reconnect_beacon()


def test_no_fleet_hold_is_old_behaviour(queue_gated):
    n = queue_gated.node
    assert n._fleet_hold is False
    release(n, ['*'], stamp=CONNECTED_AT + 1)  # ignored entirely
    queue(n)
    n._quiet_gate_enter()
    advance(queue_gated, 0.5)
    advance(queue_gated, 0.5)
    assert not n._quiet_hold


def test_hold_keepalive_round_trip_every_30s(fleet, monkeypatch):
    monkeypatch.setattr(dev, 'BLE_HOLD_KEEPALIVE', 30.0)
    n = fleet.node
    queue(n)
    n._quiet_gate_enter()
    for _ in range(25):            # 125 s
        advance(fleet, 5)
        queue(n)
    assert n.sphero.check_link.call_count == 4
    n.sphero.check_link.side_effect = RuntimeError('timeout')
    advance(fleet, 5)
    advance(fleet, 30)             # a failed keepalive is only logged
    assert n._quiet_hold
    assert 'hold keepalive failed' in n.get_logger().warning.call_args.args[0]


def test_malformed_release_ignored(fleet):
    n = fleet.node
    n._quiet_gate_enter()
    for bad in ('nope', '{}', '{"spawn_ids": "*"}'):
        n._on_fleet_release(String(data=bad))
    assert n._fleet_hold
