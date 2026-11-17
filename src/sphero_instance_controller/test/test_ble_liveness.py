"""
Unit tests for BLE link-state reporting in the device controller node.

Covers the /status connection_state contract ('connected' | 'reconnecting' |
'disconnected'), the liveness probe (no LED/matrix side effects, fast trip on
ToyDisconnectedError, 3-strike fallback, skipped while the sensor stream is
fresh), battery round-trip budget, sensor suppression while the link is
down, the disconnect listener, the reconnect beacon, _reconnect(), and the
trimmed sensor stream (sensor_stream / sensor_stream_interval_ms).

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
    n._last_stream_rx = None
    n._attach_stream_listener()
    n.sensor_stream = ['attitude']
    n._stream_on = True
    n._startup_complete = False
    n.sensor_stream_interval_ms = 500
    n.external_location = True
    n._arrow_rgb = (128, 0, 256)
    # Quiet gate off: these tests cover the start-immediately path
    # (test_ble_quiet_gate.py covers the gate).
    monkeypatch.setattr(dev, 'BLE_QUIET_GATE', False)
    n._quiet_hold = False
    n._hold_t0 = None
    n._settle_t0 = None
    n._gate_view = ('none', ())
    n.spawn_id = None
    n._queue = None
    n._queue_rx = None
    n._fleet_hold = False
    n._fleet_t0 = None
    n._last_keepalive = None
    n._connected_at = 0.0
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


def test_rebind_restores_initial_leds_and_arrow_colour(node):
    node.sphero.set_led = MagicMock()
    node.sphero.set_matrix = MagicMock()
    for _ in range(2):
        node.rebind_connection(MagicMock(), MagicMock())
    leds = [c.args for c in node.sphero.set_led.call_args_list]
    assert leds == [(0, 255, 0, 'front'), (255, 0, 0, 'back'), (0, 0, 0, 'main')] * 2
    arrows = [c.args for c in node.sphero.set_matrix.call_args_list]
    assert arrows == [('arrow_right', None, 128, 0, 256)] * 2


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


# ----- BLE round-trip budget (battery per heartbeat, stream-gated probe) -----

@pytest.fixture
def power_calls(monkeypatch):
    """Count blocking Power round trips (state.py and sphero.py share the class)."""
    calls = {'voltage': 0, 'percentage': 0}

    def _voltage(toy, proc=None):
        calls['voltage'] += 1
        return 3.9

    def _percentage(toy, proc=None):
        calls['percentage'] += 1
        return 77

    monkeypatch.setattr(sphero_mod.Power, 'get_battery_voltage', staticmethod(_voltage))
    monkeypatch.setattr(sphero_mod.Power, 'get_battery_percentage', staticmethod(_percentage))
    return calls


def test_publish_sensors_makes_no_battery_round_trips(node, power_calls):
    node.sphero.robot.is_connected = True
    for _ in range(50):
        node.publish_sensors()
    assert node.sphero.api.get_orientation.call_count == 50  # cache reads ran
    assert power_calls == {'voltage': 0, 'percentage': 0}


def test_each_heartbeat_reads_battery_once(node, power_calls):
    for i in range(1, 4):
        node.publish_heartbeat()
        assert power_calls == {'voltage': i, 'percentage': i}
    payload = json.loads(node.status_pub.publish.call_args.args[0].data)
    assert payload['battery']['percentage'] == 77
    node.battery_pub.publish.assert_called()


def test_rebind_reads_battery_before_connected(node, power_calls):
    node.mark_link_down('test')
    node.rebind_connection(MagicMock(), MagicMock())
    assert power_calls == {'voltage': 1, 'percentage': 1}
    payload = json.loads(node.status_pub.publish.call_args.args[0].data)
    assert payload['connection_state'] == 'connected'
    assert payload['battery']['percentage'] == 77


def test_stream_listener_registered_and_reattached_on_rebind(node):
    robot = node.sphero.robot
    robot.add_sensor_streaming_data_notify_listener.assert_called_once_with(
        node._on_stream_packet)
    node._on_stream_packet([0.0])
    new_robot = MagicMock()
    node.rebind_connection(new_robot, MagicMock())
    assert node._last_stream_rx is None  # old link's stamp doesn't count
    new_robot.add_sensor_streaming_data_notify_listener.assert_called_once_with(
        node._on_stream_packet)


def test_probe_no_round_trip_while_stream_fresh(node, power_calls):
    del node.sphero.check_link  # use the real check_link (Power voltage read)
    for _ in range(5):
        node._on_stream_packet([0.0])
        node._ble_liveness_probe()
    assert power_calls['voltage'] == 0


def test_probe_one_round_trip_when_stream_stale(node, power_calls):
    del node.sphero.check_link
    node._last_stream_rx = time.monotonic() - 2 * dev.BLE_STREAM_FRESH_SEC
    node._ble_liveness_probe()
    assert power_calls['voltage'] == 1
    assert power_calls['percentage'] == 0


def test_probe_round_trip_before_first_stream_packet(node):
    node._ble_liveness_probe()
    node.sphero.check_link.assert_called_once()


def test_stale_stream_failures_still_trip_after_threshold(node):
    node._on_stream_packet([0.0])
    node.sphero.check_link.side_effect = TimeoutError('timeout')
    node._ble_liveness_probe()
    node.sphero.check_link.assert_not_called()  # fresh: no probe, no strike
    node._last_stream_rx = time.monotonic() - 2 * dev.BLE_STREAM_FRESH_SEC
    for _ in range(dev.BLE_LIVENESS_FAIL_THRESHOLD - 1):
        node._ble_liveness_probe()
        assert not node.ble_link_down
    node._ble_liveness_probe()
    assert node.ble_link_down


@pytest.mark.skipif(dev.ToyDisconnectedError is None,
                    reason='spherov2 < 0.13.0 has no ToyDisconnectedError')
def test_stale_stream_toy_disconnected_trips_immediately(node):
    node._last_stream_rx = time.monotonic() - 2 * dev.BLE_STREAM_FRESH_SEC
    node.sphero.check_link.side_effect = dev.ToyDisconnectedError('gone')
    node._ble_liveness_probe()
    assert node.ble_link_down


# ----- Trimmed sensor stream (sensor_stream / sensor_stream_interval_ms) -----

from spherov2.controls.v2 import SensorControl  # noqa: E402
from spherov2.toy.bolt import BOLT  # noqa: E402
from spherov2.toy.mini import Mini  # noqa: E402

# What SpheroEduAPI.__enter__ enables (spherov2 0.13.0 __start_capturing_sensor_data)
EDU_SENSORS = {
    BOLT: ['accel_one', 'accelerometer', 'ambient_light', 'attitude', 'core_time',
           'gyroscope', 'locator', 'quaternion', 'velocity'],
    Mini: ['attitude', 'accelerometer', 'gyroscope', 'locator', 'velocity'],
}


def _streaming_toy(model):
    """A mock toy with the model's sensor tables and the real SensorControl,
    left in the state SpheroEduAPI.__enter__ leaves it (150 ms, all sensors)."""
    toy = MagicMock()
    toy.sensors = model.sensors
    toy.extended_sensors = model.extended_sensors
    toy.sensor_control = SensorControl(toy)
    toy.sensor_control.set_interval(150)  # ToyUtil.set_robot_state_on_start
    toy.sensor_control.enable(*EDU_SENSORS[model])
    toy.set_sensor_streaming_mask.reset_mock()
    toy.set_extended_sensor_streaming_mask.reset_mock()
    return toy


def _mask(model, *names):
    return sum(c.bit for n in names for c in model.sensors[n].values())


def _feed(toy, values):
    """Push one raw stream packet through SensorControl's decoder."""
    decoder = toy.add_sensor_streaming_data_notify_listener.call_args_list[0].args[0]
    got = []
    toy.sensor_control.add_sensor_data_listener(got.append)
    decoder(list(values))
    for _ in range(50):
        if got:
            break
        time.sleep(0.01)
    return got[0]


@pytest.mark.parametrize('model', [BOLT, Mini])
def test_default_stream_is_attitude_at_500ms_in_one_update(node, model):
    toy = _streaming_toy(model)
    node.sphero.robot = toy
    node._apply_sensor_stream()
    att = _mask(model, 'attitude')
    # one SensorControl update: stop (interval 0), extended mask, start
    assert [c.args for c in toy.set_sensor_streaming_mask.call_args_list] == [
        (0, 0, att), (500, 0, att)]
    toy.set_extended_sensor_streaming_mask.assert_called_once_with(0)
    # the decode table matches: 3 floats -> attitude only
    assert _feed(toy, [1.0, 2.0, 3.0]) == {
        'attitude': {'pitch': 1.0, 'roll': 2.0, 'yaw': 3.0}}


def test_keep_default_leaves_stream_untouched(node):
    toy = _streaming_toy(BOLT)
    node.sphero.robot = toy
    node.sensor_stream = [dev.SENSOR_STREAM_KEEP_DEFAULT]
    node._apply_sensor_stream()
    node.sensor_stream = []
    node._apply_sensor_stream()
    toy.set_sensor_streaming_mask.assert_not_called()
    toy.set_extended_sensor_streaming_mask.assert_not_called()


def test_unknown_sensor_names_skipped_with_warning(node):
    toy = _streaming_toy(Mini)
    node.sphero.robot = toy
    node.sensor_stream = ['attitude', 'ambient_light', 'bogus']  # Mini: no light
    node._apply_sensor_stream()
    assert toy.set_sensor_streaming_mask.call_args.args == (500, 0, _mask(Mini, 'attitude'))
    warning = node.get_logger().warning.call_args.args[0]
    assert "'ambient_light'" in warning and "'bogus'" in warning


def test_only_unknown_names_leave_default_stream_on(node):
    toy = _streaming_toy(BOLT)
    node.sphero.robot = toy
    node.sensor_stream = ['bogus']
    node._apply_sensor_stream()
    toy.set_sensor_streaming_mask.assert_not_called()


def test_extended_sensor_goes_in_extended_mask(node):
    toy = _streaming_toy(BOLT)
    node.sphero.robot = toy
    node.sensor_stream = ['attitude', 'gyroscope']
    node._apply_sensor_stream()
    gyro = sum(c.bit for c in BOLT.extended_sensors['gyroscope'].values())
    toy.set_extended_sensor_streaming_mask.assert_called_once_with(gyro)


def test_stream_reapplied_on_rebind(node):
    node.mark_link_down('test')
    toy = _streaming_toy(BOLT)
    node.rebind_connection(toy, MagicMock())
    assert toy.set_sensor_streaming_mask.call_args.args == (500, 0, _mask(BOLT, 'attitude'))


def test_publish_sensors_with_only_attitude_fresh(node, monkeypatch):
    """EduAPI getters return None for sensors not in the stream."""
    fields = {}
    monkeypatch.setattr(sys.modules['sphero_instance_controller.msg'], 'SpheroSensor',
                        lambda: fields.setdefault('msg', types.SimpleNamespace()))
    api = node.sphero.api
    node.sphero.state.set_api(api)
    node.sphero.robot.is_connected = True
    api.get_orientation.return_value = {'pitch': 1.0, 'roll': 2.0, 'yaw': 33.0}
    for getter in ('get_acceleration', 'get_gyroscope', 'get_location', 'get_velocity'):
        getattr(api, getter).return_value = None
    api.get_heading.return_value = 90
    api.get_speed.return_value = 0
    api.get_main_led.return_value = {'red': 0, 'green': 0, 'blue': 0}
    node.publish_sensors()
    node.get_logger().error.assert_not_called()
    node.state_pub.publish.assert_called_once()
    msg = node.sensor_pub.publish.call_args.args[0]
    assert msg.yaw == 33.0
    assert (msg.accel_x, msg.gyro_z, msg.velocity_x, msg.x) == (0.0, 0.0, 0.0, 0.0)


def test_reset_aim_without_locator(node):
    node.sphero.api.get_location.return_value = None
    assert node.sphero.reset_aim() is True
    node.sphero.api.reset_aim.assert_called_once()
