"""
Sensor stream off by default ('none') and the /status startup_complete flag.

Reuses the rclpy-free node fixtures of test_ble_liveness / test_ble_quiet_gate.
"""
import json
import os
from unittest.mock import MagicMock

import pytest

pytest.importorskip('rclpy')

from spherov2.toy.bolt import BOLT  # noqa: E402
from test_ble_liveness import _streaming_toy, dev, node  # noqa: E402,F401,I201
from test_ble_quiet_gate import (  # noqa: E402,F401,I201  (fixtures)
    advance, fleet, gated, logs, pending, queue, queue_gated, release)


def payloads(n):
    return [json.loads(c.args[0].data) for c in n.status_pub.publish.call_args_list]


# ----- no sensor stream -----

def test_default_stream_is_none():
    if 'SPHERO_SENSOR_STREAM' not in os.environ:
        assert dev.SENSOR_STREAM == ['none']


@pytest.mark.parametrize('gate', [True, False])
def test_none_stops_stream_in_lock_even_without_gate(monkeypatch, gate):
    monkeypatch.setattr(dev, 'BLE_QUIET_GATE', gate)
    toy = _streaming_toy(BOLT)
    assert dev.quiet_sensor_stream(toy, ['none'], 500)
    assert toy.set_sensor_streaming_mask.call_args.args[2] == 0


def test_none_never_reenables_stream(node):  # noqa: F811
    node.sensor_stream = ['none']
    node._stream_on = False
    toy = _streaming_toy(BOLT)
    node.sphero.robot = toy
    node._apply_sensor_stream()
    toy.set_sensor_streaming_mask.assert_not_called()


def test_liveness_period_slow_without_stream(node, monkeypatch):  # noqa: F811
    monkeypatch.setattr(dev, 'BLE_LIVENESS_PERIOD', 1.0)
    monkeypatch.setattr(dev, 'BLE_LIVENESS_PERIOD_NO_STREAM', 15.0)
    assert node._liveness_period() == 1.0
    node._stream_on = False
    assert node._liveness_period() == 15.0


def test_no_stream_sensors_publish_zeros_not_stale_cache(node):  # noqa: F811
    node._stream_on = False
    api = node.sphero.api
    api.get_orientation.return_value = {'pitch': 5.0, 'roll': 6.0, 'yaw': 123.0}
    api.get_acceleration.return_value = {'x': 1.0, 'y': 1.0, 'z': 1.0}
    api.get_gyroscope.return_value = {'x': 2.0, 'y': 2.0, 'z': 2.0}
    api.get_velocity.return_value = {'x': 3.0, 'y': 3.0}
    api.get_heading.return_value = 90
    api.get_speed.return_value = 0
    api.get_main_led.return_value = {'red': 0, 'green': 0, 'blue': 0}
    node.sphero.robot.is_connected = True
    node.publish_sensors()
    state = node.sphero.state
    assert state.orientation.yaw == 0.0 and state.accelerometer.x == 0.0
    assert state.gyroscope.x == 0.0 and state.velocity.x == 0.0
    assert state.get('motion', 'heading') == 90  # commanded, not streamed
    api.get_orientation.assert_not_called()
    published = json.loads(node.state_pub.publish.call_args.args[0].data)
    assert published['orientation']['yaw'] == 0.0


def test_stream_on_still_reads_stream_values(node):  # noqa: F811
    api = node.sphero.api
    api.get_orientation.return_value = {'pitch': 0.0, 'roll': 0.0, 'yaw': 45.0}
    node.sphero.robot.is_connected = True
    node.publish_sensors()
    assert node.sphero.state.orientation.yaw == 45.0


# ----- startup_complete -----

def test_status_carries_startup_complete(node):  # noqa: F811
    node.publish_connection_state('connected')
    assert payloads(node)[-1]['startup_complete'] is False


def test_release_publishes_holding_false_before_startup(gated):  # noqa: F811
    n = gated.node
    order = []
    n._apply_initial_leds = MagicMock(side_effect=lambda: order.append(
        ('leds', payloads(n)[-1]['holding'], payloads(n)[-1]['startup_complete'])))
    n._quiet_gate_enter()
    assert payloads(n) == [] or payloads(n)[-1]['holding'] is True
    advance(gated, 3.0)
    assert not n._quiet_hold
    # holding:false was already published when the LED setup ran ...
    assert order == [('leds', False, False)]
    # ... and startup_complete:true follows once it is done.
    last = payloads(n)[-1]
    assert last['connection_state'] == 'connected'
    assert last['holding'] is False and last['startup_complete'] is True
    n.publish_heartbeat()
    assert payloads(n)[-1]['startup_complete'] is True
    assert any('startup complete in' in m for m in logs(n))


def test_startup_complete_even_if_setup_fails(gated):  # noqa: F811
    n = gated.node
    n._apply_initial_leds = MagicMock(side_effect=RuntimeError('link gone'))
    n._quiet_gate_enter()
    advance(gated, 3.0)
    assert payloads(n)[-1]['startup_complete'] is True
    assert 'startup setup failed' in n.get_logger().error.call_args.args[0]


def test_reconnect_resets_startup_until_its_own_release(gated):  # noqa: F811
    n = gated.node
    n._quiet_gate_enter()
    advance(gated, 3.0)
    assert n._startup_complete
    n.mark_link_down('test')
    assert payloads(n)[-1]['connection_state'] == 'reconnecting'
    assert payloads(n)[-1]['startup_complete'] is False
    n.rebind_connection(MagicMock(), MagicMock())
    assert payloads(n)[-1]['holding'] is True
    assert payloads(n)[-1]['startup_complete'] is False
    advance(gated, 0.5)
    advance(gated, 3.0)
    assert payloads(n)[-1]['startup_complete'] is True
    n.stop_reconnect_beacon()


def test_gate_disabled_marks_startup_complete(gated, monkeypatch):  # noqa: F811
    monkeypatch.setattr(dev, 'BLE_QUIET_GATE', False)
    n = gated.node
    n._quiet_gate_enter()
    n.publish_connection_state('connected')
    assert payloads(n)[-1]['startup_complete'] is True
    assert payloads(n)[-1]['holding'] is False


def test_fleet_release_publishes_holding_false_then_startup(fleet):  # noqa: F811
    n = fleet.node
    queue(n)
    n._quiet_gate_enter()
    n.publish_heartbeat()
    assert payloads(n)[-1]['holding'] is True
    assert payloads(n)[-1]['startup_complete'] is False
    before = len(payloads(n))
    release(n, ['sid-me'])
    after = payloads(n)[before:]
    assert [(p['holding'], p['startup_complete']) for p in after] == [
        (False, False), (False, True)]


def test_keepalive_only_while_holding(fleet, monkeypatch):  # noqa: F811
    monkeypatch.setattr(dev, 'BLE_HOLD_KEEPALIVE', 30.0)
    n = fleet.node
    queue(n)
    n._quiet_gate_enter()
    release(n, ['sid-me'])
    n.sphero.check_link.reset_mock()
    for _ in range(10):
        advance(fleet, 30)
    n.sphero.check_link.assert_not_called()
