#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Per-host BLE connect queue published by the worker agent.

The agent knows a robot is coming the moment it spawns it, 15-25 s before
that robot's device controller (slow NFS startup) can say so itself. So the
agent owns the "still connecting" list for its host and publishes it, latched,
on /worker/<host_token>/connect_queue (see
sphero_instance_controller.ble_connect_queue). Connected robots on the host
stay BLE-quiet while anyone else is listed.

An entry is added at spawn (before Popen) and removed when the robot's
/status reports 'connected' or 'disconnected', on any device_error, on
teardown / process exit, when its pre-connect /status beacon goes silent, if
no beacon ever arrives, or at a hard per-entry timeout. Messages carrying a
different spawn_id (an earlier process of the same robot) are ignored.

ConnectQueue holds the logic and has no rclpy dependency (unit-tested with
fakes); ConnectQueueNode wires it to ROS. Every add / remove / stage change is
logged with an epoch timestamp so drops can be correlated across hosts.
"""

import json
import os
import threading
import time

PRE_CONNECT_STATES = ('initializing', 'connecting')
TERMINAL_STATES = ('connected', 'disconnected')


def _env_float(name, default):
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


ENTRY_TIMEOUT = _env_float('WORKER_CONNECT_ENTRY_TIMEOUT', 150.0)
FIRST_BEACON = _env_float('WORKER_CONNECT_FIRST_BEACON', 60.0)
BEACON_STALE = _env_float('WORKER_CONNECT_BEACON_STALE', 10.0)
REFRESH_PERIOD = 5.0  # seconds: republish unchanged content (dead-agent detection)


class ConnectQueue:
    """Robots spawned on this host whose initial BLE connect hasn't finished."""

    def __init__(self, host, publish, clock=time.monotonic, wall=time.time,
                 entry_timeout=ENTRY_TIMEOUT, first_beacon=FIRST_BEACON,
                 beacon_stale=BEACON_STALE, refresh=REFRESH_PERIOD, log=None):
        """
        Args:
            host: this host's name (goes into the payload).
            publish: callable(dict) that sends the payload.
            clock / wall: monotonic clock for timeouts, epoch clock for
                payload stamps and logs (both injectable for tests).
        """
        self.host = host
        self._publish_fn = publish
        self._clock = clock
        self._wall = wall
        self.entry_timeout = entry_timeout
        self.first_beacon = first_beacon
        self.beacon_stale = beacon_stale
        self.refresh = refresh
        self._log_fn = log or (lambda text: print(text, flush=True))
        self._lock = threading.Lock()
        self._entries = {}   # name -> dict(spawn_id, since, t0, stage, last_beacon)
        self._seq = 0
        self._last_publish = None

    # --- mutators (each publishes on change) --------------------------------

    def add(self, name, spawn_id):
        """A robot is being spawned: list it (replaces an older entry of name)."""
        with self._lock:
            self._entries[name] = {
                'spawn_id': spawn_id, 'since': self._wall(), 't0': self._clock(),
                'stage': 'spawned', 'last_beacon': None}
            self._log(f'add {name} spawn_id={spawn_id}')
            self._publish_locked()

    def remove(self, name, reason, spawn_id=None):
        """Drop name (only if spawn_id matches, when given). Returns True if removed."""
        with self._lock:
            return self._remove_locked(name, reason, spawn_id)

    def on_status(self, name, payload):
        """A /sphero/<ns>/status payload (dict) arrived for name."""
        state = payload.get('connection_state')
        sid = payload.get('spawn_id')
        with self._lock:
            entry = self._entries.get(name)
            if entry is None or (sid is not None and sid != entry['spawn_id']):
                return
            if state in TERMINAL_STATES:
                self._remove_locked(name, f'status_{state}', None)
            elif state in PRE_CONNECT_STATES:
                entry['last_beacon'] = self._clock()
                if state != entry['stage']:
                    entry['stage'] = state
                    self._log(f'stage {name} {state} after '
                              f'{self._clock() - entry["t0"]:.1f} s')
                    self._publish_locked()

    def on_device_error(self, name, payload):
        """Any device_error (toy_not_found / connect_failed / ...) ends the connect."""
        sid = payload.get('spawn_id')
        with self._lock:
            entry = self._entries.get(name)
            if entry is None or (sid is not None and sid != entry['spawn_id']):
                return
            self._remove_locked(name, f'device_error_{payload.get("error")}', None)

    def housekeep(self, is_alive):
        """
        Expire dead / silent / overdue entries and do the periodic refresh.

        is_alive(name) is called WITHOUT this queue's lock held (it takes the
        process manager's lock, which is taken before ours on spawn).
        """
        with self._lock:
            names = list(self._entries)
        alive = {name: is_alive(name) for name in names}
        with self._lock:
            now = self._clock()
            for name, entry in list(self._entries.items()):
                age = now - entry['t0']
                beacon = entry['last_beacon']
                if not alive.get(name, True):
                    reason = 'process_exit'
                elif beacon is not None and now - beacon > self.beacon_stale:
                    reason = 'beacon_stale'
                elif beacon is None and age > self.first_beacon:
                    reason = 'no_beacon'
                elif age > self.entry_timeout:
                    reason = 'timeout'
                else:
                    continue
                self._remove_locked(name, reason, None)
            if self._last_publish is None or now - self._last_publish >= self.refresh:
                self._publish_locked()

    def publish_now(self):
        """Publish the current content (agent start)."""
        with self._lock:
            self._log('publish (agent start)')
            self._publish_locked()

    # --- readers -------------------------------------------------------------

    def names(self):
        with self._lock:
            return list(self._entries)

    def snapshot(self):
        with self._lock:
            return self._payload_locked()

    # --- internals (call with self._lock held) --------------------------------

    def _remove_locked(self, name, reason, spawn_id):
        entry = self._entries.get(name)
        if entry is None or (spawn_id is not None and spawn_id != entry['spawn_id']):
            return False
        del self._entries[name]
        self._log(f'remove {name} reason={reason} after '
                  f'{self._clock() - entry["t0"]:.1f} s (stage={entry["stage"]})')
        self._publish_locked()
        return True

    def _payload_locked(self):
        return {
            'host': self.host,
            'seq': self._seq,
            'stamp': self._wall(),
            'pending': [{'name': n, 'spawn_id': e['spawn_id'], 'since': e['since'],
                         'stage': e['stage']} for n, e in self._entries.items()],
        }

    def _publish_locked(self):
        self._seq += 1
        self._last_publish = self._clock()
        try:
            self._publish_fn(self._payload_locked())
        except Exception as exc:  # noqa: BLE001 - never break spawn on a publish
            self._log(f'publish failed: {exc!r}')

    def _log(self, text):
        self._log_fn(f'[connect-queue] t={self._wall():.3f} host={self.host} {text} '
                     f'pending={list(self._entries)}')


def make_node(host, is_alive):
    """Build the rclpy ConnectQueueNode (rclpy imported lazily)."""
    from rclpy.node import Node
    from std_msgs.msg import String

    from sphero_instance_controller.ble_connect_queue import (
        connect_queue_qos, connect_queue_topic, host_token)

    class ConnectQueueNode(Node):
        """ROS side: latched queue publisher + per-pending-robot subscriptions."""

        def __init__(self):
            super().__init__(f'sphero_worker_agent_{host_token(host)}')
            self.topic = connect_queue_topic(host)
            self._pub = self.create_publisher(String, self.topic, connect_queue_qos())
            self.queue = ConnectQueue(host, self._send)
            self._is_alive = is_alive
            self._subs = {}  # name -> (status_sub, device_error_sub)
            self.create_timer(1.0, self._tick)
            self.queue.publish_now()
            print(f'[connect-queue] publishing on {self.topic}', flush=True)

        # Process-manager hooks (called from Flask threads).
        def on_spawn(self, name, spawn_id):
            self.queue.add(name, spawn_id)

        def on_gone(self, name, reason, spawn_id=None):
            self.queue.remove(name, reason, spawn_id)

        def _send(self, payload):
            self._pub.publish(String(data=json.dumps(payload)))

        def _tick(self):
            self.queue.housekeep(self._is_alive)
            self._sync_subs()

        def _sync_subs(self):
            """Subscribe to /status + /device_error only while a robot is pending.

            Runs on the executor thread, so entity creation never races spin.
            """
            wanted = set(self.queue.names())
            for name in list(self._subs):
                if name not in wanted:
                    for sub in self._subs.pop(name):
                        self.destroy_subscription(sub)
            for name in wanted - set(self._subs):
                ns = name.replace('-', '_')
                self._subs[name] = (
                    self.create_subscription(
                        String, f'/sphero/{ns}/status',
                        lambda msg, n=name: self._on_msg(n, msg, self.queue.on_status), 10),
                    self.create_subscription(
                        String, f'/sphero/{ns}/device_error',
                        lambda msg, n=name: self._on_msg(n, msg, self.queue.on_device_error),
                        10),
                )

        @staticmethod
        def _on_msg(name, msg, handler):
            try:
                payload = json.loads(msg.data)
            except (TypeError, ValueError):
                return
            if isinstance(payload, dict):
                handler(name, payload)

    return ConnectQueueNode()
