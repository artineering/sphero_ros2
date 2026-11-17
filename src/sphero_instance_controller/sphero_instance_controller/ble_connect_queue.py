#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Per-host BLE connect queue: shared names and helpers.

The worker agent publishes the robots it has spawned that have not finished
their initial BLE connect on /worker/<host_token>/connect_queue (std_msgs/String
JSON, reliable + transient_local, depth 1):

    {"host": "rpi4-node01", "seq": 17, "stamp": 1763335130.2,
     "pending": [{"name": "SB-1E2D", "spawn_id": "6f1c...",
                  "since": 1763335120.4, "stage": "spawned"}]}

Device controllers on the same host subscribe to it for the BLE quiet gate.

Fleet hold: a robot spawned with SPHERO_BLE_FLEET_HOLD=1 stays quiet after its
connect until the webserver names it on /fleet/ble_release (same QoS):

    {"seq": 3, "stamp": 1763335200.5, "spawn_ids": ["6f1c...", ...],
     "reason": "deploy_complete" | "manual"}

"spawn_ids": ["*"] releases every holding fleet-hold robot that connected
before (stamp + FLEET_RELEASE_SKEW); see release_matches().
Both sides build the topic with these helpers so the host token always
matches; the agent also injects SPHERO_WORKER_HOST into every spawn.
"""

import json
import os
import re
import socket

ENV_WORKER_HOST = 'SPHERO_WORKER_HOST'
ENV_SPAWN_ID = 'SPHERO_SPAWN_ID'
ENV_FLEET_HOLD = 'SPHERO_BLE_FLEET_HOLD'
FLEET_RELEASE_TOPIC = '/fleet/ble_release'
RELEASE_ALL = '*'
# /status connection_state values a controller reports before its node exists.
PRE_CONNECT_STATES = ('initializing', 'connecting')


def resolve_host():
    """Return this host's name: SPHERO_WORKER_HOST if set, else gethostname()."""
    return os.environ.get(ENV_WORKER_HOST) or socket.gethostname()


def host_token(host):
    """Turn a host name into a valid ROS name token ('rpi4-node01' -> 'rpi4_node01')."""
    token = re.sub(r'[^A-Za-z0-9_]', '_', host or '') or 'unknown'
    if token[0].isdigit():
        token = 'h_' + token
    return token


def connect_queue_topic(host):
    """Return the absolute connect-queue topic for host."""
    return f'/worker/{host_token(host)}/connect_queue'


def connect_queue_qos():
    """Return the QoS shared by the queue publisher and its subscribers."""
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def parse_queue(data):
    """Parse a connect-queue payload; return the dict, or None if malformed."""
    try:
        queue = json.loads(data)
    except (TypeError, ValueError):
        return None
    if not isinstance(queue, dict) or not isinstance(queue.get('pending'), list):
        return None
    return queue


def others_pending(queue, self_name):
    """Return the names in queue's pending list other than self_name."""
    names = []
    for item in queue.get('pending', []):
        name = item.get('name') if isinstance(item, dict) else None
        if name and name != self_name:
            names.append(name)
    return names


def parse_release(data):
    """Parse a /fleet/ble_release payload; return the dict, or None if malformed."""
    try:
        release = json.loads(data)
    except (TypeError, ValueError):
        return None
    if not isinstance(release, dict) or not isinstance(release.get('spawn_ids'), list):
        return None
    return release


def release_matches(release, spawn_id, connected_at, skew):
    """
    Decide whether a fleet release applies to one held robot.

    Returns (matches, why). Its own spawn_id always matches (a new spawn has
    a new id, so an old latched release can never name it). A '*' matches only
    if it was stamped no earlier than `skew` seconds before this robot
    connected (connected_at, epoch): a '*' latched from an earlier deploy must
    not release new spawns. Hosts are NTP-synced, so skew only has to absorb
    milliseconds of clock offset plus publish latency.
    """
    ids = release.get('spawn_ids') or []
    if spawn_id and spawn_id in ids:
        return True, 'named'
    if RELEASE_ALL in ids:
        try:
            stamp = float(release.get('stamp'))
        except (TypeError, ValueError):
            return False, 'release_all_without_stamp'
        if stamp >= connected_at - skew:
            return True, 'release_all'
        return False, f'stale_release_all ({connected_at - stamp:.1f} s before connect)'
    return False, 'not_named'
