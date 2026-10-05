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
