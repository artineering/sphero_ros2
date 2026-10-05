#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Hardware-free unit tests for SpheroInstanceManager._instance_status().

Status is derived from HEARTBEAT freshness: a Sphero reads 'running' only if
FleetNode received telemetry (a /sphero/<name>/status heartbeat or a sensor
message) within HEARTBEAT_FRESH_SEC -- those only flow while the BLE link is
live. A spawned-but-not-connected unit emits no heartbeat, so it never reads
'running'. We exercise the state logic by binding the unbound method to a
minimal stub with a fake fleet_node -- no ROS node, no subprocess, no BLE.
"""

import time
import types

from multirobot_webserver.multirobot_webapp import (
    SpheroInstanceManager,
    INSTANCE_CONNECT_WINDOW,
    HEARTBEAT_FRESH_SEC,
)


class _Proc:
    """Stand-in for subprocess.Popen: poll() -> None alive, else exit code."""

    def __init__(self, exit_code=None):
        self._code = exit_code

    def poll(self):
        return self._code


def _status(*, fresh=None, alive=True, age=0.0, link=None, link_age=0.0):
    """
    Run _instance_status for a LOCAL instance (worker=None).

    `fresh` = seconds since the last heartbeat (None => never seen). `age` =
    seconds since spawn. `alive` = subprocess still running. `link` = explicit
    connection_state (None => legacy controller, key absent), set `link_age`
    seconds ago.
    """
    now = time.time()
    name = 'SB-TEST'
    instance = {
        'name': name,
        'process': _Proc(None if alive else 1),
        'added_at': now - age,
    }
    last_seen = 0.0 if fresh is None else now - fresh
    entry = {'last_seen': last_seen, 'added_at': now - age}
    if link is not None:
        entry['link_state'] = link
        entry['link_state_at'] = now - link_age
    fleet_node = types.SimpleNamespace(robots={name: entry})
    mgr = types.SimpleNamespace(fleet_node=fleet_node)
    return SpheroInstanceManager._instance_status(mgr, instance)


def test_dead_process_is_failed():
    # Process exited -> failed regardless of any recent heartbeat.
    assert _status(fresh=1.0, alive=False) == 'failed'


def test_fresh_heartbeat_is_running():
    # Recent heartbeat => live BLE link => running.
    assert _status(fresh=1.0) == 'running'


def test_never_seen_within_window_is_connecting():
    # No heartbeat yet, still inside the connect grace window -> connecting.
    assert _status(fresh=None, age=1.0) == 'connecting'


def test_stale_heartbeat_within_window_is_connecting():
    # Heartbeat older than the freshness window but still within the connect
    # grace window -> connecting (link is coming up / flaky, not yet failed).
    assert _status(fresh=HEARTBEAT_FRESH_SEC + 5.0, age=1.0) == 'connecting'


def test_no_link_past_window_is_failed():
    # The false-'running' case we exist to catch: spawned, past the connect
    # window, no fresh heartbeat -> failed, never running.
    past = INSTANCE_CONNECT_WINDOW + 5.0
    assert _status(fresh=None, age=past) == 'failed'
    assert _status(fresh=HEARTBEAT_FRESH_SEC + past, age=past) == 'failed'


def test_reconnecting_overrides_freshness():
    # Explicit link-down state wins over a still-fresh heartbeat.
    assert _status(fresh=1.0, link='reconnecting') == 'reconnecting'
    assert _status(fresh=None, link='reconnecting', link_age=1.0) == 'reconnecting'


def test_stale_reconnecting_falls_through():
    # Repeats stopped (wedged process / worker off the network): fall back to
    # the existing logic instead of reading 'reconnecting' forever.
    stale = HEARTBEAT_FRESH_SEC + 5.0
    past = INSTANCE_CONNECT_WINDOW + 5.0
    assert _status(link='reconnecting', link_age=stale, age=past) == 'failed'
    assert _status(link='reconnecting', link_age=stale, age=1.0) == 'connecting'


def test_reconnecting_with_dead_process_is_failed():
    assert _status(link='reconnecting', alive=False) == 'failed'


def test_disconnected_is_terminal_even_after_exit():
    # Shown alike for local and remote: wins over the dead-process check.
    assert _status(link='disconnected') == 'disconnected'
    assert _status(link='disconnected', alive=False) == 'disconnected'
    assert _status(link='disconnected', link_age=1000.0,
                   age=INSTANCE_CONNECT_WINDOW + 5.0) == 'disconnected'


def test_connected_or_legacy_uses_freshness():
    assert _status(fresh=1.0, link='connected') == 'running'
    assert _status(fresh=1.0, link=None) == 'running'
    assert _status(fresh=HEARTBEAT_FRESH_SEC + 5.0, link='connected',
                   age=1.0) == 'connecting'
