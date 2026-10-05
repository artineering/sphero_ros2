"""
Unit tests for BLE link-state reporting in the device controller node.

Covers the /status connection_state contract ('connected' | 'reconnecting' |
'disconnected'), the liveness probe (no LED/matrix side effects, fast trip on
ToyDisconnectedError, 3-strike fallback), sensor suppression while the link is
down, the disconnect listener, the reconnect beacon, and _reconnect().

No rclpy context is created: the node is built with object.__new__ and only
the attributes these methods use. (An rclpy context leaves DDS threads alive,
which deadlocks the forked workers of the later ament_flake8 test.)
"""
from contextlib import contextmanager
import json
import sys
import time
import types
from unittest.mock import MagicMock

import pytest

pytest.importorskip('rclpy')

# conftest puts the src package first on sys.path, which has no generated msg
# module; stub it so the node module imports without a colcon build. Any real
# ROS type works as the placeholder: these tests never publish it.
try:
    import sphero_instance_controller.msg  # noqa: F401
except ImportError:
    from std_msgs.msg import String as _Placeholder
    _msg = types.ModuleType('sphero_instance_controller.msg')
    _msg.SpheroSensor = _Placeholder
    sys.modules['sphero_instance_controller.msg'] = _msg

from sphero_instance_controller import sphero_instance_device_controller_node as dev  # noqa: E402
from sphero_instance_controller.core.sphero import sphero as sphero_mod  # noqa: E402
from sphero_instance_controller.core.sphero.sphero import Sphero  # noqa: E402

Ctl = dev.SpheroInstanceDeviceController


def _fake_clock():
    msg = types.SimpleNamespace(sec=0)
    return types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: msg))


@pytest.fixture
def node(monkeypatch):
    n = object.__new__(Ctl)
    logger = MagicMock()
    n.get_logger = lambda: logger
    n.get_clock = _fake_clock
    n.sphero_name = 'SB-TEST'
    n.topic_prefix = 'sphero/SB_TEST'
    n.sphero = Sphero(MagicMock(), MagicMock(), 'SB-TEST')
    n.sphero.check_link = MagicMock()
    for pub in ('status_pub', 'state_pub', 'sensor_pub', 'battery_pub'):
        setattr(n, pub, MagicMock())
    n._ble_fail_streak = 0
    n.ble_link_down = False
    n._device_error_pub = MagicMock()
    n.connection_state = 'connected'
    n._ble_lost_evt = dev.threading.Event()
    n._beacon_stop = None
    n._attach_disconnect_listener()
    monkeypatch.setattr(dev.rclpy, 'spin_once', lambda *a, **k: None)
    yield n
    n.stop_reconnect_beacon()


def states(n):
    return [json.loads(c.args[0].data)['connection_state']
            for c in n.status_pub.publish.call_args_list]


def test_publish_connection_state_shape(node):
    node.publish_connection_state('reconnecting')
    payload = json.loads(node.status_pub.publish.call_args.args[0].data)
    assert payload['sphero_name'] == 'SB-TEST'
    assert payload['connection_state'] == 'reconnecting'
    assert node.connection_state == 'reconnecting'
    assert {'timestamp', 'battery', 'is_healthy'} <= payload.keys()


def test_probe_uses_check_link_and_never_touches_leds_or_matrix(node):
    node.sphero.api.reset_mock()
    node._ble_liveness_probe()
    node.sphero.check_link.assert_called_once()
    node.sphero.api.set_main_led.assert_not_called()
    node.sphero.api.set_matrix_pixel.assert_not_called()
    node.sphero.api.clear_matrix.assert_not_called()


def test_check_link_reraises(monkeypatch):
    def _boom(toy):
        raise TimeoutError('no response')
    monkeypatch.setattr(sphero_mod.Power, 'get_battery_voltage', staticmethod(_boom))
    s = Sphero(MagicMock(), MagicMock(), 'SB-TEST')
    with pytest.raises(TimeoutError):
        s.check_link()


@pytest.mark.skipif(dev.ToyDisconnectedError is None,
                    reason='spherov2 < 0.13.0 has no ToyDisconnectedError')
def test_toy_disconnected_trips_on_first_failure(node):
    node.status_pub.reset_mock()
    node.sphero.check_link.side_effect = dev.ToyDisconnectedError('gone')
    node._ble_liveness_probe()
    assert node.ble_link_down
    assert states(node)[0] == 'reconnecting'


def test_generic_dead_error_needs_threshold(node):
    node.sphero.check_link.side_effect = TimeoutError('timeout')
    for _ in range(dev.BLE_LIVENESS_FAIL_THRESHOLD - 1):
        node._ble_liveness_probe()
        assert not node.ble_link_down
    node._ble_liveness_probe()
    assert node.ble_link_down


def test_transient_error_resets_streak(node):
    node.sphero.check_link.side_effect = TimeoutError('timeout')
    node._ble_liveness_probe()
    node.sphero.check_link.side_effect = ValueError('packet decode glitch')
    node._ble_liveness_probe()
    assert node._ble_fail_streak == 0
    assert not node.ble_link_down


def test_sensors_published_while_connected(node):
    node.sphero.robot.is_connected = True
    node.sphero.update_sensors = MagicMock()
    node.sphero.get_state_dict = MagicMock(return_value={})
    node.sphero.get_sensor_msg = MagicMock(return_value=None)
    node.publish_sensors()
    node.state_pub.publish.assert_called_once()


def test_sensors_suppressed_when_link_down(node):
    node.ble_link_down = True
    node.publish_sensors()
    node.state_pub.publish.assert_not_called()
    node.sensor_pub.publish.assert_not_called()


def test_sensors_suppressed_when_robot_not_connected(node):
    node.sphero.robot.is_connected = False
    node.publish_sensors()
    node.state_pub.publish.assert_not_called()


def test_heartbeat_skipped_while_down(node):
    node.status_pub.reset_mock()
    node.ble_link_down = True
    node.publish_heartbeat()
    node.status_pub.publish.assert_not_called()


def test_listener_only_sets_event(node):
    node.status_pub.reset_mock()
    node._on_ble_disconnect()
    assert node._ble_lost_evt.is_set()
    assert not node.ble_link_down
    node.status_pub.publish.assert_not_called()


def test_listener_registered_and_detached(node):
    robot = node.sphero.robot
    robot.add_disconnect_listener.assert_called_once_with(node._on_ble_disconnect)
    node.detach_disconnect_listener()
    robot.remove_disconnect_listener.assert_called_once_with(node._on_ble_disconnect)


def test_beacon_repeats_reconnecting_until_stopped(node, monkeypatch):
    monkeypatch.setattr(dev, 'BLE_RECONNECTING_REPEAT', 0.05)
    node.status_pub.reset_mock()
    node.mark_link_down('test')
    time.sleep(0.3)
    node.stop_reconnect_beacon()
    time.sleep(0.1)
    count = len(states(node))
    assert count >= 3
    assert set(states(node)) == {'reconnecting'}
    time.sleep(0.15)
    assert len(states(node)) == count


def test_rebind_publishes_connected_and_stops_beacon(node, monkeypatch):
    monkeypatch.setattr(dev, 'BLE_RECONNECTING_REPEAT', 0.05)
    node.mark_link_down('test')
    node._ble_lost_evt.set()
    new_robot = MagicMock()
    node.rebind_connection(new_robot, MagicMock())
    assert not node.ble_link_down
    assert not node._ble_lost_evt.is_set()
    assert node._beacon_stop is None
    assert states(node)[-1] == 'connected'
    new_robot.add_disconnect_listener.assert_called_once_with(node._on_ble_disconnect)


@contextmanager
def _no_lock(label=''):
    yield


def test_reconnect_exhausted_returns_error(node, monkeypatch):
    monkeypatch.setattr(dev, 'BLE_RECONNECT_BACKOFF', 0.0)
    monkeypatch.setattr(dev, 'ble_connect_lock', _no_lock)
    err = RuntimeError('toy not found')
    monkeypatch.setattr(dev.scanner, 'find_toy', MagicMock(side_effect=err))
    node.mark_link_down('test')
    cm, last = dev._reconnect(node, 'SB-TEST')
    assert cm is None
    assert last is err
    assert node.ble_link_down


def test_reconnect_success_rebinds(node, monkeypatch):
    monkeypatch.setattr(dev, 'BLE_RECONNECT_BACKOFF', 0.0)
    monkeypatch.setattr(dev, 'ble_connect_lock', _no_lock)
    new_robot, new_api, cm = MagicMock(), MagicMock(), MagicMock()
    cm.__enter__.return_value = new_api
    monkeypatch.setattr(dev.scanner, 'find_toy', MagicMock(return_value=new_robot))
    monkeypatch.setattr(dev, 'SpheroEduAPI', MagicMock(return_value=cm))
    node.mark_link_down('test')
    got_cm, err = dev._reconnect(node, 'SB-TEST')
    assert got_cm is cm and err is None
    assert node.sphero.api is new_api
    assert states(node)[-1] == 'connected'


def test_publish_ble_lost_reports_disconnected(node):
    node.mark_link_down('test')
    node.publish_ble_lost(RuntimeError('gone'))
    assert node._beacon_stop is None
    assert states(node)[-1] == 'disconnected'
