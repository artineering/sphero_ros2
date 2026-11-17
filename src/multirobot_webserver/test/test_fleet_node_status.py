#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Hardware-free unit tests for FleetNode's /status, /device_error and /sensors
handling of the explicit connection_state contract. The unbound methods are
bound to a minimal stub (robots dict + lock + _entry) -- no rclpy init.
"""

import json
import threading
import types

from std_msgs.msg import String

from multirobot_webserver.multirobot_webapp import FleetNode

NAME = 'SB-TEST'


def _node(link_state=None, last_seen=0.0):
    robots = {NAME: {'last_seen': last_seen, 'battery': 50, 'heading': 0,
                     'link_state': link_state, 'link_state_at': 0.0}}
    node = types.SimpleNamespace(robots=robots, _lock=threading.Lock())
    node._entry = lambda name: robots.get(name)
    node.logs = []
    logger = types.SimpleNamespace(**{
        level: (lambda text, _l=level: node.logs.append((_l, text)))
        for level in ('info', 'warning', 'error')})
    node.get_logger = lambda: logger
    return node


def _str(payload):
    return String(data=payload if isinstance(payload, str) else json.dumps(payload))


def _status(node, state, **extra):
    payload = {'sphero_name': NAME, 'battery': 3, 'is_healthy': False, **extra}
    if state is not None:
        payload['connection_state'] = state
    FleetNode._on_status(node, NAME, _str(payload))


def _sensor(node):
    msg = types.SimpleNamespace(battery_percentage=80, yaw=90)
    FleetNode._on_sensor(node, NAME, msg)


def test_reconnecting_marks_down_and_zeroes_last_seen():
    node = _node(last_seen=123.0)
    _status(node, 'reconnecting')
    entry = node.robots[NAME]
    assert entry['link_state'] == 'reconnecting'
    assert entry['link_state_at'] > 0.0
    assert entry['last_seen'] == 0.0


def test_non_connected_status_does_not_touch_battery():
    node = _node()
    _status(node, 'reconnecting')
    _status(node, 'disconnected')
    assert node.robots[NAME]['battery'] == 50


def test_late_sensor_cannot_resurrect_link_down():
    for state in ('reconnecting', 'disconnected'):
        node = _node()
        _status(node, state)
        _sensor(node)
        entry = node.robots[NAME]
        assert entry['last_seen'] == 0.0
        assert entry['link_state'] == state


def test_connected_clears_down_state():
    node = _node()
    _status(node, 'reconnecting')
    _status(node, 'connected')
    entry = node.robots[NAME]
    assert entry['link_state'] == 'connected'
    assert entry['last_seen'] > 0.0
    _sensor(node)
    assert entry['battery'] == 80


def test_legacy_heartbeat_refreshes_last_seen():
    # Missing state, unknown state and unparsable payloads all behave like the
    # old content-blind heartbeat.
    for payload in (None, 'weird'):
        node = _node()
        _status(node, payload)
        assert node.robots[NAME]['last_seen'] > 0.0
        assert node.robots[NAME]['link_state'] is None
    for raw in ('not json', '5'):
        node = _node()
        FleetNode._on_status(node, NAME, String(data=raw))
        assert node.robots[NAME]['last_seen'] > 0.0


def test_ble_lost_device_error_is_disconnected():
    node = _node(link_state='connected', last_seen=123.0)
    FleetNode._on_device_error(node, NAME, _str({'error': 'ble_lost'}))
    entry = node.robots[NAME]
    assert entry['link_state'] == 'disconnected'
    assert entry['last_seen'] == 0.0


def test_other_device_errors_ignored():
    node = _node(link_state='connected', last_seen=123.0)
    FleetNode._on_device_error(node, NAME, _str({'error': 'something_else'}))
    FleetNode._on_device_error(node, NAME, String(data='not json'))
    entry = node.robots[NAME]
    assert entry['link_state'] == 'connected'
    assert entry['last_seen'] == 123.0


def test_unknown_robot_is_ignored():
    node = _node()
    node._entry = lambda name: None
    _status(node, 'reconnecting')
    FleetNode._on_device_error(node, NAME, _str({'error': 'ble_lost'}))
    assert node.robots[NAME]['link_state'] is None


def test_transitions_log_once_heartbeats_silent():
    node = _node(link_state='connected', last_seen=123.0)
    node.robots[NAME]['worker'] = 'rpi4-1'
    _status(node, 'reconnecting')
    _status(node, 'reconnecting')  # ~1 Hz repeat: silent
    _status(node, 'connected')
    for _ in range(5):
        _status(node, 'connected')  # heartbeats: silent
    assert [lvl for lvl, _ in node.logs] == ['warning', 'info']
    assert node.logs[0][1] == f'{NAME} link reconnecting [worker rpi4-1]'
    assert node.logs[1][1].startswith(f'{NAME} link connected (was down ')
    assert node.logs[1][1].endswith('s) [worker rpi4-1]')


def test_first_connected_after_spawn_logs_once():
    node = _node()
    _status(node, 'connected')
    _status(node, 'connected')
    assert node.logs == [('info', f'{NAME} link connected')]


def test_ble_lost_burst_logs_once():
    node = _node(link_state='connected', last_seen=123.0)
    for _ in range(10):
        FleetNode._on_device_error(node, NAME, _str(
            {'error': 'ble_lost', 'sphero_name': NAME, 'message': 'timeout'}))
    assert node.logs == [
        ('error', f'{NAME} link lost: controller exited (ble_lost: timeout)')]


def test_ble_lost_burst_interleaved_with_disconnected_status():
    # Controller order: status 'disconnected' then device_error, x10.
    node = _node(link_state='reconnecting')
    for _ in range(10):
        _status(node, 'disconnected')
        FleetNode._on_device_error(node, NAME, _str(
            {'error': 'ble_lost', 'message': 'timeout'}))
    assert [lvl for lvl, _ in node.logs] == ['warning', 'error']


def test_real_rclpy_logger_accepts_mixed_severities():
    # Regression: one getattr(logger, level)(...) call site logging info then
    # warning made rclpy raise 'Logger severity cannot be changed between
    # calls', which killed the webserver's executor thread.
    import rclpy.logging
    node = _node()
    real = rclpy.logging.get_logger('test_fleet_node_status_mixed')
    node.get_logger = lambda: real
    _status(node, 'connected')
    _status(node, 'reconnecting')
    _status(node, 'connected')
    _status(node, 'disconnected')


# --- Pre-connect states (initializing / connecting) and holding -------------

def test_pre_connect_sets_link_state_without_last_seen():
    for state in ('initializing', 'connecting'):
        node = _node(last_seen=0.0)
        _status(node, state)
        entry = node.robots[NAME]
        assert entry['link_state'] == state
        assert entry['link_state_at'] > 0.0
        assert entry['last_seen'] == 0.0


def test_pre_connect_leaves_existing_last_seen_alone():
    node = _node(last_seen=123.0)
    _status(node, 'initializing')
    assert node.robots[NAME]['last_seen'] == 123.0


def test_pre_connect_then_connected_logs_each_transition_once():
    node = _node()
    node.robots[NAME]['worker'] = 'rpi4-1'
    for _ in range(3):
        _status(node, 'initializing', spawn_id='a1')  # ~1 Hz beacon
    for _ in range(3):
        _status(node, 'connecting', spawn_id='a1')
    _status(node, 'connected', spawn_id='a1')
    _status(node, 'connected', spawn_id='a1')
    assert node.logs == [
        ('info', f'{NAME} link initializing [worker rpi4-1]'),
        ('info', f'{NAME} link connecting [worker rpi4-1]'),
        ('info', f'{NAME} link connected [worker rpi4-1]'),
    ]
    entry = node.robots[NAME]
    assert entry['link_state'] == 'connected'
    assert entry['last_seen'] > 0.0
    assert entry['spawn_id'] == 'a1'


def test_new_spawn_id_relogs_same_pre_connect_state():
    node = _node()
    _status(node, 'initializing', spawn_id='a1')
    _status(node, 'initializing', spawn_id='b2')  # respawned
    _status(node, 'initializing', spawn_id='b2')
    assert [t for _, t in node.logs] == [f'{NAME} link initializing'] * 2


def test_holding_is_stored_from_any_status():
    node = _node()
    _status(node, 'connected', holding=True)
    assert node.robots[NAME]['holding'] is True
    _status(node, 'connected', holding=False)
    assert node.robots[NAME]['holding'] is False
    _status(node, 'connected', holding=True)
    _status(node, 'connected')  # field absent -> not holding
    assert node.robots[NAME]['holding'] is False


def test_real_rclpy_logger_with_pre_connect_states():
    import rclpy.logging
    node = _node()
    real = rclpy.logging.get_logger('test_fleet_node_status_preconnect')
    node.get_logger = lambda: real
    _status(node, 'initializing')
    _status(node, 'connecting')
    _status(node, 'connected')
    _status(node, 'reconnecting')
    _status(node, 'connected')


def test_startup_complete_stored_default_false():
    node = _node()
    _status(node, 'connected', startup_complete=True)
    assert node.robots[NAME]['startup_complete'] is True
    _status(node, 'connected')
    assert node.robots[NAME]['startup_complete'] is False


# --- Heading from the overhead tracker's pose orientation --------------------

def _pose(yaw_deg=None, quat=None, x_cm=120.0, y_cm=-40.0):
    import math
    from geometry_msgs.msg import PoseStamped
    msg = PoseStamped()
    msg.pose.position.x = x_cm
    msg.pose.position.y = y_cm
    if quat is not None:
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = quat
    elif yaw_deg is not None:
        th = math.radians(yaw_deg)
        msg.pose.orientation.z = math.sin(0.5 * th)
        msg.pose.orientation.w = math.cos(0.5 * th)
    return msg


def test_quat_to_yaw_deg():
    import math
    from multirobot_webserver.multirobot_webapp import quat_to_yaw_deg
    for deg in (1.0, 45.0, 90.0, 135.0, -30.0, -90.0, -179.0, 180.0):
        th = math.radians(deg)
        got = quat_to_yaw_deg(0.0, 0.0, math.sin(th / 2), math.cos(th / 2))
        assert abs(got - deg) < 1e-6, (deg, got)
    # Same rotation, negated quaternion.
    th = math.radians(60.0)
    assert abs(quat_to_yaw_deg(0.0, 0.0, -math.sin(th / 2),
                               -math.cos(th / 2)) - 60.0) < 1e-6
    # No heading yet: identity / all zeros.
    assert quat_to_yaw_deg(0.0, 0.0, 0.0, 1.0) is None
    assert quat_to_yaw_deg(0.0, 0.0, 0.0, -1.0) is None
    assert quat_to_yaw_deg(0.0, 0.0, 0.0, 0.0) is None


def test_localization_sets_heading_and_keeps_it_without_a_fix():
    node = _node()
    entry = node.robots[NAME]
    entry.update(x=0.0, y=0.0, heading=0)
    FleetNode._on_localization(node, NAME, _pose(yaw_deg=90.4))
    assert (entry['x'], entry['y'], entry['heading']) == (1.2, -0.4, 90)
    FleetNode._on_localization(node, NAME, _pose(yaw_deg=-135.0))
    assert entry['heading'] == -135
    # Identity / zero orientation = no heading fix: keep the last heading,
    # still update the position.
    FleetNode._on_localization(node, NAME, _pose(quat=(0.0, 0.0, 0.0, 1.0),
                                                 x_cm=10.0))
    FleetNode._on_localization(node, NAME, _pose(quat=(0.0, 0.0, 0.0, 0.0)))
    assert entry['heading'] == -135 and entry['x'] == 1.2


def test_sensors_no_longer_override_heading():
    node = _node()
    entry = node.robots[NAME]
    entry.update(x=0.0, y=0.0, heading=0)
    FleetNode._on_localization(node, NAME, _pose(yaw_deg=30.0))
    _sensor(node)  # yaw=90, battery=80
    assert entry['heading'] == 30
    assert entry['battery'] == 80
