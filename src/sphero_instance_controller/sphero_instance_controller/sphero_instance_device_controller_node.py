#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Sphero Instance Device Controller Node for Multi-Robot Setup.

This node provides low-level device control for a single Sphero robot using 
namespaced topics to support multiple concurrent Sphero instances without cross-talk.

All topics are namespaced under 'sphero/<sphero_name>/' to allow multiple
instances to run simultaneously.
"""

import time
import json
import signal
import threading
import random
import os
import fcntl
from contextlib import contextmanager
from time import time as _epoch  # wall clock for log correlation across hosts

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped

from sphero_instance_controller.msg import SpheroSensor

from spherov2 import scanner
from spherov2.sphero_edu import SpheroEduAPI
try:
    # spherov2 >= 0.13.0 raises this the moment the adapter reports the drop.
    from spherov2.toy import ToyDisconnectedError
except ImportError:
    ToyDisconnectedError = None

from sphero_instance_controller.ble_connect_queue import (
    ENV_SPAWN_ID, FLEET_RELEASE_TOPIC, connect_queue_qos, connect_queue_topic,
    others_pending, parse_queue, parse_release, release_matches, resolve_host)
from sphero_instance_controller.core.sphero import Sphero
from sphero_instance_controller.spherov2_collision_patch import apply_collision_patch

# Apply collision detection patch for 16-byte collision responses
apply_collision_patch()


# Per-host serialization of the BLE scan+connect phase.
#
# BlueZ rejects parallel connects on a single adapter (org.bluez.Error.InProgress),
# so when several device-controller processes start at once on one Pi they starve
# each other's scan/connect. We serialize that critical section across processes on
# the same host with a file lock at a host-local path (NOT the NFS workspace).
DEFAULT_BLE_CONNECT_LOCK = '/tmp/sphero_ble_connect.lock'
BLE_CONNECT_LOCK_TIMEOUT = 90.0  # seconds: bounded blocking acquire, then proceed
BLE_CONNECT_LOCK_POLL = 0.25     # seconds between non-blocking acquire attempts


def _env_float(name, default):
    """Read a float env var, falling back to default on missing/invalid."""
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


def _env_int(name, default):
    """Read an int env var, falling back to default on missing/invalid."""
    try:
        return int(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


def _env_list(name, default):
    """Read a comma-separated env var as a list of strings; default if unset/empty."""
    items = [s.strip() for s in os.environ.get(name, '').split(',') if s.strip()]
    return items or default


# Runtime BLE reconnection tunables (env-overridable).
#
# A live BLE link can drop mid-run (unit powered off, out of range, adapter
# hiccup). A dedicated liveness probe (see SpheroInstanceDeviceController) does
# a real round-trip every LIVENESS_PERIOD seconds; LIVENESS_FAIL_THRESHOLD
# consecutive link-dead failures declare the link down. main() then tries to
# reconnect RECONNECT_ATTEMPTS times with RECONNECT_BACKOFF between attempts.
BLE_RECONNECT_ATTEMPTS = _env_int('SPHERO_BLE_RECONNECT_ATTEMPTS', 3)
BLE_RECONNECT_BACKOFF = _env_float('SPHERO_BLE_RECONNECT_BACKOFF', 2.5)
BLE_LIVENESS_PERIOD = _env_float('SPHERO_BLE_LIVENESS_PERIOD', 1.0)
BLE_LIVENESS_FAIL_THRESHOLD = _env_int('SPHERO_BLE_LIVENESS_FAIL_THRESHOLD', 3)
# The robot streams sensor packets every `sensor_stream_interval_ms` (500 ms by
# default). A packet younger than this proves the link is alive, so the probe
# skips its round trip; only a quiet stream falls back to check_link().
BLE_STREAM_FRESH_SEC = _env_float('SPHERO_BLE_STREAM_FRESH_SEC', 1.5)
# With no sensor stream (sensor_stream 'none', the default) there are no packets
# to prove liveness. Drops are caught by spherov2's disconnect listener (0.26 s in
# the 0.13.0 qualification, link-layer supervision timeout, no app traffic
# needed); the probe is only a backstop for a link BlueZ never reports, so it
# runs this rarely instead of every LIVENESS_PERIOD.
BLE_LIVENESS_PERIOD_NO_STREAM = _env_float('SPHERO_BLE_LIVENESS_PERIOD_NO_STREAM', 15.0)

# Per-host BLE quiet gate. A freshly (re)connected robot stays BLE-quiet
# (stream stopped, no LEDs / battery reads / liveness round trips) until no
# other robot on this host is still connecting: "pending" must read empty for
# QUIET_SETTLE seconds, or QUIET_MAX_HOLD has passed.
# "Pending" comes from the worker agent's /worker/<host>/connect_queue (robots
# are added at spawn, before their slow NFS startup). Until that topic is heard
# (QUEUE_WAIT after gate entry), or once it goes stale (QUEUE_STALE), the
# per-host flock fallback is used: every controller holds a shared flock on the
# pending-connect file from the start of main() until its initial connect ends
# (pending_connect_marker). SPHERO_BLE_GATE_SOURCE: auto | queue | flock.
# SPHERO_BLE_QUIET_GATE=0 restores the old start-immediately behaviour.
DEFAULT_BLE_PENDING_LOCK = '/tmp/sphero_ble_pending.lock'
BLE_QUIET_GATE = _env_int('SPHERO_BLE_QUIET_GATE', 1) != 0
BLE_QUIET_MAX_HOLD = _env_float('SPHERO_BLE_QUIET_MAX_HOLD', 120.0)
BLE_QUIET_SETTLE = _env_float('SPHERO_BLE_QUIET_SETTLE', 1.0)
BLE_QUIET_POLL = 0.5  # seconds between pending-connect checks while holding
BLE_GATE_SOURCE = os.environ.get('SPHERO_BLE_GATE_SOURCE', 'auto').strip().lower()
BLE_QUEUE_WAIT = _env_float('SPHERO_BLE_QUEUE_WAIT', 5.0)
BLE_QUEUE_STALE = _env_float('SPHERO_BLE_QUEUE_STALE', 15.0)
# Report 'initializing' / 'connecting' on /status before the node exists.
BLE_PRECONNECT_STATUS = _env_int('SPHERO_BLE_PRECONNECT_STATUS', 1) != 0
BLE_PRECONNECT_REPEAT = 1.0  # seconds between pre-connect /status repeats
# Fleet hold (SPHERO_BLE_FLEET_HOLD=1, set per spawn by the worker agent when
# the webserver deploys a roster): after its initial connect the robot always
# holds, whatever the per-host queue says, until /fleet/ble_release names its
# spawn_id (or '*' stamped no earlier than FLEET_RELEASE_SKEW before this robot
# connected), or FLEET_HOLD_MAX passes. Then the per-host gate applies as usual.
BLE_FLEET_HOLD = _env_int('SPHERO_BLE_FLEET_HOLD', 0) != 0
BLE_FLEET_HOLD_MAX = _env_float('SPHERO_BLE_FLEET_HOLD_MAX', 900.0)
BLE_FLEET_RELEASE_SKEW = _env_float('SPHERO_BLE_FLEET_RELEASE_SKEW', 2.0)
# While holding, one cheap round trip (battery voltage read) this often keeps
# the robot from treating the silent link as inactivity. 0 disables.
BLE_HOLD_KEEPALIVE = _env_float('SPHERO_BLE_HOLD_KEEPALIVE', 30.0)

# Sensor stream left running after connect (see _apply_sensor_stream). Defaults
# come from the env so the spawn chain (worker agent / webserver -> websocket
# server -> this node) passes them through, like the SPHERO_BLE_* tunables.
# ['default'] leaves spherov2's own stream (every sensor, 150 ms) untouched; an
# empty list can't be passed with -p (ROS2 rejects `[]`), hence the names.
# ['none'] (the default; localization comes from the overhead camera) stops the
# stream inside the connect lock and never restarts it: /sensors then carries
# only battery (heartbeat reads), commanded motion and the external position;
# orientation / accel / gyro / velocity stay 0.
SENSOR_STREAM_KEEP_DEFAULT = 'default'
SENSOR_STREAM_OFF = 'none'
SENSOR_STREAM = _env_list('SPHERO_SENSOR_STREAM', [SENSOR_STREAM_OFF])
SENSOR_STREAM_INTERVAL_MS = _env_int('SPHERO_SENSOR_STREAM_INTERVAL_MS', 500)
# While reconnecting, /status repeats 'reconnecting' this often so a webserver
# that starts late still learns the state.
BLE_RECONNECTING_REPEAT = 1.0

# Substrings (lowercased) identifying a dead/broken BLE link vs. a transient.
# spherov2 raises RuntimeError('Use toys in context manager') once the adapter
# is gone; bleak surfaces BleakError / disconnection / EOFError; a stalled
# device trips a concurrent.futures TimeoutError. A PacketDecodingException is
# a transient packet collision and is deliberately NOT in this set.
_LINK_DEAD_MARKERS = (
    'use toys in context manager',
    'bleak',
    'disconnect',
    'eoferror',
    'timeout',
    'not connected',
)


def _is_link_dead_error(exc) -> bool:
    """Classify an exception as a dead-link signature (vs. a transient)."""
    text = f'{type(exc).__name__}: {exc}'.lower()
    return any(marker in text for marker in _LINK_DEAD_MARKERS)


@contextmanager
def ble_connect_lock(label=''):
    """Serialize the BLE scan+connect phase per host via an flock'd lockfile.

    Bounded blocking acquire: polls a non-blocking exclusive flock for up to
    BLE_CONNECT_LOCK_TIMEOUT seconds, then proceeds anyway rather than deadlocking.
    The lock is always released (LOCK_UN) and the fd closed on exit, even on
    exception inside the critical section.
    """
    lock_path = os.environ.get('SPHERO_BLE_CONNECT_LOCK', DEFAULT_BLE_CONNECT_LOCK)
    fd = open(lock_path, 'w')
    acquired = False
    deadline = time.monotonic() + BLE_CONNECT_LOCK_TIMEOUT
    waited = False
    try:
        while True:
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                acquired = True
                break
            except BlockingIOError:
                if not waited:
                    print(f"[ble-lock] waiting on {lock_path} for {label}...")
                    waited = True
                if time.monotonic() >= deadline:
                    print(f"[ble-lock] timed out after {BLE_CONNECT_LOCK_TIMEOUT}s "
                          f"waiting for {label}; proceeding without lock")
                    break
                time.sleep(BLE_CONNECT_LOCK_POLL)
        if acquired:
            print(f"[ble-lock] acquired {lock_path} for {label}")
        yield
    finally:
        if acquired:
            try:
                fcntl.flock(fd, fcntl.LOCK_UN)
            except OSError:
                pass
            print(f"[ble-lock] released {lock_path} for {label}")
        fd.close()


def _pending_lock_path():
    return os.environ.get('SPHERO_BLE_PENDING_LOCK', DEFAULT_BLE_PENDING_LOCK)


@contextmanager
def pending_connect_marker(label=''):
    """
    Mark this process as 'still connecting' for the per-host quiet gate.

    Holds a shared flock on the pending-connect file for the duration. The
    kernel drops it if the process dies, so a crash never leaves a stale
    marker. If the file can't be opened, nobody waits on us (fail open).
    """
    fd = None
    try:
        fd = open(_pending_lock_path(), 'a')
        # Blocking: a checker holds LOCK_EX only for microseconds.
        fcntl.flock(fd, fcntl.LOCK_SH)
        print(f'[ble-gate] t={_epoch():.3f} pending-connect marker taken for {label}')
    except OSError as e:
        print(f'[ble-gate] no pending-connect marker for {label}: {e}')
        if fd is not None:
            fd.close()
            fd = None
    try:
        yield
    finally:
        if fd is not None:
            try:
                fcntl.flock(fd, fcntl.LOCK_UN)
            except OSError:
                pass
            fd.close()
            print(f'[ble-gate] t={_epoch():.3f} pending-connect marker dropped for {label}')


def pending_connects() -> bool:
    """Report whether any controller on this host holds a pending-connect marker."""
    try:
        fd = open(_pending_lock_path(), 'a')
    except OSError:
        return False
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        return True
    except OSError:
        return False
    else:
        fcntl.flock(fd, fcntl.LOCK_UN)
        return False
    finally:
        fd.close()


class PreConnectBeacon:
    """
    Repeat 'initializing' / 'connecting' on /status until the node exists.

    Publishes immediately on every stage change and then every `period`
    seconds: /status is volatile and the first sends go out before DDS has
    matched any subscriber. A daemon thread; stop() before the publisher's
    node is destroyed. Does nothing when SPHERO_BLE_PRECONNECT_STATUS=0.
    """

    def __init__(self, publisher, sphero_name, spawn_id, period=BLE_PRECONNECT_REPEAT):
        self._pub = publisher
        self._name = sphero_name
        self._spawn_id = spawn_id
        self._period = period
        self._stage = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None

    def set_stage(self, stage):
        """Switch to stage, log the transition and publish it right away."""
        if not BLE_PRECONNECT_STATUS:
            return
        with self._lock:
            if stage == self._stage:
                return
            self._stage = stage
        print(f'[ble-gate] t={_epoch():.3f} {self._name} stage={stage} '
              f'spawn_id={self._spawn_id}', flush=True)
        self._publish()
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()

    def stop(self):
        """Stop repeating; safe to call more than once."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _publish(self):
        with self._lock:
            stage = self._stage
        if stage is None or self._stop.is_set():
            return
        payload = {
            'sphero_name': self._name,
            'timestamp': int(_epoch()),
            'connection_state': stage,
            'spawn_id': self._spawn_id,
            'holding': False,
        }
        try:
            self._pub.publish(String(data=json.dumps(payload)))
        except Exception as e:
            print(f'[ble-gate] pre-connect /status publish failed: {e}')

    def _run(self):
        while not self._stop.wait(self._period):
            self._publish()


def _stream_is_managed(robot, wanted, interval_ms) -> bool:
    """Report whether _apply_sensor_stream will (re)start a trimmed stream on robot."""
    if not wanted or wanted == [SENSOR_STREAM_KEEP_DEFAULT] or interval_ms <= 0:
        return False
    control = getattr(robot, 'sensor_control', None)
    if control is None or not hasattr(control, '_SensorControl__enabled_extended'):
        return False
    supported = set(robot.sensors) | set(robot.extended_sensors)
    return any(s in supported for s in wanted)


def quiet_sensor_stream(robot, wanted, interval_ms) -> bool:
    """
    Stop the full stream SpheroEduAPI.__enter__ just started.

    Called inside ble_connect_lock, so these writes can't overlap a
    neighbour's scan+connect. With sensor_stream ['none'] the stream is
    stopped for good (gate or not). Otherwise only when the quiet gate is on
    and the node will restart a trimmed stream at gate release; else the
    stream is left alone. Never raises.
    """
    stream_off = wanted == [SENSOR_STREAM_OFF]
    if not stream_off and (not BLE_QUIET_GATE
                           or not _stream_is_managed(robot, wanted, interval_ms)):
        return False
    control = getattr(robot, 'sensor_control', None)
    if control is None:
        return False
    try:
        control.disable_all()
    except Exception as e:
        print(f'[ble-gate] could not stop the sensor stream: {e}')
        return False
    if stream_off:
        print('[ble-gate] sensor stream stopped (sensor_stream=none: stays off)')
    else:
        print('[ble-gate] sensor stream stopped until the quiet gate releases')
    return True


class SpheroInstanceDeviceController(Node):
    """
    ROS2 node for low-level device control of a single Sphero robot instance.

    This controller uses namespaced topics to support multi-robot setups.
    All topics are prefixed with 'sphero/<sphero_name>/' where sphero_name
    is provided as a ROS parameter.

    Provides topic-based interface for:
    - LED control
    - Movement (roll, spin, heading, speed)
    - Matrix display (for BOLT)
    - Sensor data publishing
    - Stop command
    """

    def __init__(self, robot, api, sphero_name: str):
        """
        Initialize the Sphero instance device controller node.

        Args:
            robot: The Sphero robot object from scanner
            api: The SpheroEduAPI object
            sphero_name: Name of the Sphero (used for topic namespacing)
        """
        # Sanitize name: replace hyphens with underscores (ROS2 naming rules)
        name_safe = sphero_name.replace("-", "_")
        # Create unique node name with sphero name suffix
        node_name = f'sphero_device_controller_{name_safe}'
        super().__init__(node_name)

        # Declare and get ROS parameters
        self.declare_parameter('sensor_rate', 10.0)  # Default: 10 Hz
        # Paired with multirobot_webserver's HEARTBEAT_FRESH_SEC, which must stay
        # at least ~3x this. Shortening this without widening that (or vice
        # versa) makes the fleet dashboard unable to hold a robot at 'running'
        # on the heartbeat alone.
        self.declare_parameter('heartbeat_rate', 5)  # Default: 5 seconds
        self.declare_parameter('external_localization', False) # Default: turned OFF
        self.declare_parameter('sensor_stream', SENSOR_STREAM)
        self.declare_parameter('sensor_stream_interval_ms', SENSOR_STREAM_INTERVAL_MS)

        self.sensor_rate = self.get_parameter('sensor_rate').value
        self.heartbeat_rate = self.get_parameter('heartbeat_rate').value
        self.external_location = self.get_parameter('external_localization').value
        self.sensor_stream = list(self.get_parameter('sensor_stream').value)
        self.sensor_stream_interval_ms = self.get_parameter('sensor_stream_interval_ms').value
        # Fewer than 2 packets per freshness window: one lost packet sends the
        # liveness probe back to round trips (still correct, just not free).
        self._stream_on = self.sensor_stream != [SENSOR_STREAM_OFF]
        if (self._stream_on and self.sensor_stream != [SENSOR_STREAM_KEEP_DEFAULT]
                and self.sensor_stream_interval_ms > BLE_STREAM_FRESH_SEC * 500):
            self.get_logger().warning(
                f'sensor_stream_interval_ms={self.sensor_stream_interval_ms} is more '
                f'than half of BLE_STREAM_FRESH_SEC ({BLE_STREAM_FRESH_SEC} s); the '
                f'liveness probe will often fall back to BLE round trips.')


        self.sphero_name = sphero_name
        # Sanitize topic name: replace hyphens with underscores (ROS2 topic naming rules)
        self.topic_name_safe = name_safe
        self.topic_prefix = f'sphero/{self.topic_name_safe}'

        # Initialize Sphero core class
        self.sphero = Sphero(robot, api, sphero_name, self.external_location)

        
        # Calculate sensor timer period (1/frequency)
        self.sensor_period = 1.0 / self.sensor_rate if self.sensor_rate > 0 else 0.1

        # Create subscribers for Sphero commands
        self._create_subscribers()

        # Create publishers for sensor data and status
        self._create_publishers()

        # Runtime BLE-liveness tracking. The probe timer fires on the node's
        # executor; the spin loop in main() reads ble_link_down to drive the
        # reconnect lifecycle (which owns the context manager + the host lock).
        self._ble_fail_streak = 0
        self.ble_link_down = False
        self._device_error_pub = None  # lazily created on first ble_lost publish
        # Reported on /status: 'connected' | 'reconnecting' | 'disconnected'.
        self.connection_state = 'connected'
        # Set by spherov2's disconnect listener (a worker thread); main() polls
        # it, so all node state stays on the main thread.
        self._ble_lost_evt = threading.Event()
        self._beacon_stop = None  # Event stopping the 'reconnecting' repeater
        self._attach_disconnect_listener()
        # time.monotonic() of the last sensor-stream packet (None = none yet).
        self._last_stream_rx = None
        self._attach_stream_listener()
        # Per-host quiet gate (see BLE_QUIET_GATE); set by _quiet_gate_enter.
        self._quiet_hold = False
        # /status startup_complete: True once the post-connect setup
        # (_start_transmitting) has run for the current link.
        self._startup_complete = False
        self._hold_t0 = None
        self._settle_t0 = None
        self._gate_view = ('none', ())
        # Worker agent's connect queue for this host (see ble_connect_queue).
        self.spawn_id = os.environ.get(ENV_SPAWN_ID) or None
        self._queue = None
        self._queue_rx = None  # time.monotonic() of the last good queue message
        if BLE_QUIET_GATE and BLE_GATE_SOURCE != 'flock':
            queue_topic = connect_queue_topic(resolve_host())
            self.connect_queue_sub = self.create_subscription(
                String, queue_topic, self._on_connect_queue, connect_queue_qos())
            self.get_logger().info(f'[ble-gate] listening on {queue_topic} '
                                   f'(source={BLE_GATE_SOURCE}, spawn_id={self.spawn_id})')
        # Fleet hold (see BLE_FLEET_HOLD): pending until a release names us.
        self._connected_at = _epoch()  # wall clock of the initial connect
        self._fleet_hold = BLE_FLEET_HOLD and BLE_QUIET_GATE
        self._fleet_t0 = None
        self._last_keepalive = None
        if BLE_FLEET_HOLD and not BLE_QUIET_GATE:
            self.get_logger().warning(
                '[ble-gate] SPHERO_BLE_FLEET_HOLD ignored: SPHERO_BLE_QUIET_GATE=0')
        if self._fleet_hold:
            self.fleet_release_sub = self.create_subscription(
                String, FLEET_RELEASE_TOPIC, self._on_fleet_release, connect_queue_qos())

        # Create timers
        self.sensor_timer = self.create_timer(self.sensor_period, self.publish_sensors)

        liveness_period = self._liveness_period()
        self.liveness_timer = self.create_timer(liveness_period, self._ble_liveness_probe)
        if not self._stream_on:
            self.get_logger().info(
                f'Sensor stream: off (sensor_stream=none); drops via the disconnect '
                f'listener, liveness probe every {liveness_period:.0f} s')

        if self.heartbeat_rate > 0:
            self.heartbeat_timer = self.create_timer(self.heartbeat_rate, self.publish_heartbeat)

        if BLE_QUIET_GATE:
            self.quiet_gate_timer = self.create_timer(BLE_QUIET_POLL, self._quiet_gate_tick)

        # Log initialization info
        self._log_initialization()

        # Arrow colour is picked once so a reconnect redraws the same one
        red = 128 * random.randint(0,2)
        green = 128 * random.randint(0,2)
        blue = 128 * random.randint(0,2)
        self._arrow_rgb = (red, green, blue)
        # Stream, LEDs and battery now, or at gate release if neighbours on
        # this host are still connecting.
        self._quiet_gate_enter()

        # Announce 'connected' now rather than at the first heartbeat tick.
        # The battery value is real unless the gate is holding (then it is
        # refreshed and re-announced at release).
        self.publish_connection_state('connected')

    def _liveness_period(self):
        """Probe period: fast while stream packets can prove liveness, slow without."""
        return BLE_LIVENESS_PERIOD if self._stream_on else BLE_LIVENESS_PERIOD_NO_STREAM

    def _start_transmitting(self):
        """Run the post-connect BLE setup: stream (if any), LEDs + arrow, battery.

        Marks startup_complete when done, whether or not every step worked:
        the webserver uses it to pace releases, so it must never stick at False.
        """
        t0 = time.monotonic()
        try:
            self._apply_sensor_stream()
            self._apply_initial_leds()
            self.sphero.update_battery()
        except Exception as e:
            self.get_logger().error(f'[ble-gate] {self.sphero_name} startup setup failed: {e}')
        finally:
            self._startup_complete = True
        self.get_logger().info(
            f'[ble-gate] t={_epoch():.3f} {self.sphero_name} startup complete in '
            f'{time.monotonic() - t0:.1f} s')

    def _quiet_gate_enter(self):
        """
        Start transmitting, or hold quiet while this host has pending connects.

        Called after the initial connect and after every reconnect. The
        stream was already stopped inside the connect lock
        (quiet_sensor_stream); the release restarts it.
        """
        if not BLE_QUIET_GATE:
            self._start_transmitting()
            return
        now = time.monotonic()
        self._quiet_hold = True
        self._startup_complete = False
        self._hold_t0 = now
        self._last_keepalive = None
        if self._fleet_hold:
            if self._fleet_t0 is None:
                self._fleet_t0 = now
            self._settle_t0 = None
            self._gate_view = ('fleet', ())
            self.get_logger().info(
                f'[ble-gate] t={_epoch():.3f} {self.sphero_name} fleet hold: waiting for '
                f'{FLEET_RELEASE_TOPIC} naming spawn_id={self.spawn_id}; cap '
                f'{BLE_FLEET_HOLD_MAX:.0f} s ({now - self._fleet_t0:.1f} s held so far)')
            return
        pending, source, names = self._pending_connects_now(now)
        self._gate_view = (source, tuple(names))
        if pending:
            self._settle_t0 = None
            self.get_logger().info(
                f'[ble-gate] {self.sphero_name} quiet: waiting on {self._waiting_text()} '
                f'(source={source}); holding up to {BLE_QUIET_MAX_HOLD:.1f} s')
        else:
            self._settle_t0 = now
            self.get_logger().info(
                f'[ble-gate] {self.sphero_name}: no pending connects (source={source}); '
                f'starting after {BLE_QUIET_SETTLE:.1f} s settle')

    def _quiet_gate_tick(self):
        """Gate timer: release once the queue stays empty, or at the max hold."""
        # While the link is down main() owns the lifecycle; rebind re-enters.
        if not self._quiet_hold or self.ble_link_down:
            return
        now = time.monotonic()
        self._hold_keepalive(now)
        if self._fleet_hold:
            if now - self._fleet_t0 >= BLE_FLEET_HOLD_MAX:
                self.get_logger().warning(
                    f'[ble-gate] t={_epoch():.3f} {self.sphero_name} fleet hold cap '
                    f'{BLE_FLEET_HOLD_MAX:.0f} s reached without a release; releasing itself')
                self._end_fleet_hold('cap', None, 'cap')
            return
        if now - self._hold_t0 >= BLE_QUIET_MAX_HOLD:
            self._quiet_gate_release('timeout')
            return
        pending, source, names = self._pending_connects_now(now)
        view = (source, tuple(names))
        if view != self._gate_view:
            self._gate_view = view
            self.get_logger().info(
                f'[ble-gate] {self.sphero_name} now waiting on {self._waiting_text()} '
                f'(source={source}) after {now - self._hold_t0:.1f} s')
        if pending:
            self._settle_t0 = None
        elif self._settle_t0 is None:
            self._settle_t0 = now
        elif now - self._settle_t0 >= BLE_QUIET_SETTLE:
            self._quiet_gate_release('queue_empty')

    def _quiet_gate_release(self, reason):
        """End the hold: run the deferred setup and re-announce 'connected'."""
        waited = time.monotonic() - self._hold_t0
        self._quiet_hold = False
        self.get_logger().info(
            f'[ble-gate] {self.sphero_name} released (reason={reason}, '
            f'source={self._gate_view[0]}) after {waited:.1f} s')
        # holding:false goes out BEFORE the (multi-second) setup, then
        # startup_complete:true once it has finished.
        self.publish_connection_state('connected')
        self._start_transmitting()
        self.publish_connection_state('connected')

    def _hold_keepalive(self, now):
        """One cheap round trip every BLE_HOLD_KEEPALIVE s while holding.

        spherov2 also pings on the firmware's will-sleep notification, but that
        path has only been bench-checked for 60 s holds; a fleet hold can last
        many minutes. Errors are only logged: the disconnect listener owns drop
        detection.
        """
        if BLE_HOLD_KEEPALIVE <= 0:
            return
        last = self._last_keepalive if self._last_keepalive is not None else self._hold_t0
        if now - last < BLE_HOLD_KEEPALIVE:
            return
        self._last_keepalive = now
        try:
            self.sphero.check_link()
        except Exception as e:
            self.get_logger().warning(
                f'[ble-gate] {self.sphero_name} hold keepalive failed: {e}',
                throttle_duration_sec=30.0)

    def _on_fleet_release(self, msg):
        """/fleet/ble_release subscriber: end the fleet hold if it names us."""
        if not self._fleet_hold:
            return
        release = parse_release(msg.data)
        if release is None:
            self.get_logger().warning(
                f'[ble-gate] ignoring malformed {FLEET_RELEASE_TOPIC} message: {msg.data!r}',
                throttle_duration_sec=30.0)
            return
        matches, why = release_matches(release, self.spawn_id, self._connected_at,
                                       BLE_FLEET_RELEASE_SKEW)
        if not matches:
            if why != 'not_named':
                self.get_logger().warning(
                    f'[ble-gate] {self.sphero_name} ignoring fleet release '
                    f'seq={release.get("seq")}: {why}')
            return
        self._end_fleet_hold(release.get('reason'), release.get('seq'), why)

    def _end_fleet_hold(self, reason, seq, match):
        """Leave the fleet hold; the per-host gate then decides (normally at once)."""
        now = time.monotonic()
        held = now - self._fleet_t0 if self._fleet_t0 is not None else 0.0
        self._fleet_hold = False
        self.get_logger().info(
            f'[ble-gate] t={_epoch():.3f} {self.sphero_name} fleet release '
            f'(reason={reason}, seq={seq}, match={match}) after {held:.1f} s held')
        if not self._quiet_hold or self.ble_link_down:
            return  # a reconnect in progress re-enters the per-host gate itself
        pending, source, names = self._pending_connects_now(now)
        self._gate_view = (source, tuple(names))
        if not pending:
            self._quiet_gate_release(f'fleet_{reason}')
            return
        self._hold_t0 = now
        self._settle_t0 = None
        self.get_logger().info(
            f'[ble-gate] {self.sphero_name} per-host gate still waiting on '
            f'{self._waiting_text()} (source={source})')

    def _waiting_text(self):
        source, names = self._gate_view
        if source == 'queue':
            return '[' + ', '.join(names) + ']' if names else 'nobody'
        if source == 'awaiting_queue':
            return 'first connect_queue message'
        return 'flock holders' if names else 'nobody'

    def _on_connect_queue(self, msg):
        """connect_queue subscriber: keep the latest well-formed queue."""
        queue = parse_queue(msg.data)
        if queue is None:
            self.get_logger().warning(
                f'[ble-gate] ignoring malformed connect_queue message: {msg.data!r}',
                throttle_duration_sec=30.0)
            return
        self._queue = queue
        self._queue_rx = time.monotonic()

    def _pending_connects_now(self, now):
        """
        Report whether other robots on this host are still connecting.

        Returns (pending, source, names). source is 'queue' (the worker
        agent's connect_queue; this robot's own entry is ignored),
        'awaiting_queue' (none heard yet, still inside QUEUE_WAIT: stay
        quiet), 'flock' (fallback), or 'none' (queue-only mode, no queue).
        """
        if BLE_GATE_SOURCE != 'flock':
            if self._queue_rx is not None and now - self._queue_rx <= BLE_QUEUE_STALE:
                names = others_pending(self._queue, self.sphero_name)
                return bool(names), 'queue', names
            if self._queue_rx is None and now - self._hold_t0 < BLE_QUEUE_WAIT:
                return True, 'awaiting_queue', []
            if BLE_GATE_SOURCE == 'queue':
                return False, 'none', []
        held = pending_connects()
        return held, 'flock', (['flock'] if held else [])

    def _apply_initial_leds(self):
        """Set the startup LEDs and heading arrow (a fresh connection starts dark)."""
        # Initial front and back LEDs
        self.sphero.set_led(0, 255, 0, 'front')
        self.sphero.set_led(255, 0, 0, 'back')
        self.sphero.set_led(0, 0, 0, 'main')

        # Set the arrow symbol to indicate heading
        self.sphero.set_matrix('arrow_right', None, *self._arrow_rgb)

    def _create_subscribers(self):

        if self.external_location:
            """Create ROS subscriber for localization topic."""
            # Shared "localization position contract": /localization/<name_safe>/position
            topic_name = f'/localization/{self.topic_name_safe}/position'
            self.localization_sub = self.create_subscription(
                PoseStamped, topic_name, self._localization_callback, 10)
    
        
        """Create all ROS subscribers for command topics."""
        self.led_sub = self.create_subscription(
            String, f'{self.topic_prefix}/led', self.led_callback, 10)

        self.roll_sub = self.create_subscription(
            String, f'{self.topic_prefix}/roll', self.roll_callback, 10)

        self.spin_sub = self.create_subscription(
            String, f'{self.topic_prefix}/spin', self.spin_callback, 10)

        self.heading_sub = self.create_subscription(
            String, f'{self.topic_prefix}/heading', self.heading_callback, 10)

        self.speed_sub = self.create_subscription(
            String, f'{self.topic_prefix}/speed', self.speed_callback, 10)

        self.raw_motor_sub = self.create_subscription(
            String, f'{self.topic_prefix}/raw_motor', self.raw_motor_callback, 10)

        self.stop_sub = self.create_subscription(
            String, f'{self.topic_prefix}/stop', self.stop_callback, 10)

        self.reset_aim_sub = self.create_subscription(
            String, f'{self.topic_prefix}/reset_aim', self.reset_aim_callback, 10)

        self.calibrate_compass_sub = self.create_subscription(
            String, f'{self.topic_prefix}/calibrate_compass', self.calibrate_compass_callback, 10)

        self.matrix_sub = self.create_subscription(
            String, f'{self.topic_prefix}/matrix', self.matrix_callback, 10)

        self.collision_sub = self.create_subscription(
            String, f'{self.topic_prefix}/collision', self.collision_callback, 10)

        self.stabilization_sub = self.create_subscription(
            String, f'{self.topic_prefix}/stabilization', self.stabilization_callback, 10)

        self.ir_sub = self.create_subscription(
            String, f'{self.topic_prefix}/ir', self.ir_callback, 10)

    def _create_publishers(self):
        """Create all ROS publishers for status and sensor topics."""
        self.sensor_pub = self.create_publisher(
            SpheroSensor, f'{self.topic_prefix}/sensors', 10)

        self.state_pub = self.create_publisher(
            String, f'{self.topic_prefix}/state', 10)

        self.battery_pub = self.create_publisher(
            BatteryState, f'{self.topic_prefix}/battery', 10)

        self.status_pub = self.create_publisher(
            String, f'{self.topic_prefix}/status', 10)

        self.tap_pub = self.create_publisher(
            String, f'{self.topic_prefix}/tap', 10)

        # Commanded motion, echoed for anything that needs to predict where this
        # robot is going -- the overhead tracker feeds it straight into its
        # Kalman control input. A SEPARATE topic, not a republish of /roll:
        # this node subscribes to /roll, so echoing there would feed its own
        # commands back to itself.
        self.motion_cmd_pub = self.create_publisher(
            String, f'{self.topic_prefix}/motion_cmd', 10)

        # Hook the CORE object, not the callbacks above. Every motion path ends
        # at Sphero.roll/set_speed/set_heading/stop, but only some of them come
        # in over a topic: the task stack calls the core directly in-process
        # (direct_task_executor), so a /fleet/policy broadcast never appears on
        # /roll. Hooking here is the one place that sees both.
        self.sphero.on_motion = self._publish_motion_cmd

        self.obstacle_pub = self.create_publisher(
            String, f'{self.topic_prefix}/obstacle', 10)

        # Completion signal for compass calibration. calibrate_compass_callback
        # BLOCKS (~seconds) while the robot spins; an overhead registrar fires
        # calibration on many robots in parallel and waits for each to report
        # done. Published in a finally so the registrar always unblocks.
        self.calibrate_compass_done_pub = self.create_publisher(
            Bool, f'{self.topic_prefix}/calibrate_compass_done', 10)

    def _log_initialization(self):
        """Log initialization information."""
        self.get_logger().info('='*70)
        self.get_logger().info('🎮 DEVICE CONTROLLER ACTIVATED')
        self.get_logger().info('='*70)
        self.get_logger().info(f'Sphero Name: {self.sphero_name}')
        self.get_logger().info(f'Topic Prefix: {self.topic_prefix}')
        self.get_logger().info(f'Sensor Rate: {self.sensor_rate} Hz')
        self.get_logger().info(f'Heartbeat: Every {self.heartbeat_rate} seconds')
        self.get_logger().info('Subscribed Topics:')
        self.get_logger().info(f'  - {self.topic_prefix}/led')
        self.get_logger().info(f'  - {self.topic_prefix}/roll')
        self.get_logger().info(f'  - {self.topic_prefix}/raw_motor')
        self.get_logger().info(f'  - {self.topic_prefix}/stop')
        self.get_logger().info(f'  - {self.topic_prefix}/matrix')
        self.get_logger().info(f'  - {self.topic_prefix}/ir')
        self.get_logger().info(f'  - ... and 6 more')
        self.get_logger().info('Publishing Topics:')
        self.get_logger().info(f'  - {self.topic_prefix}/sensors')
        self.get_logger().info(f'  - {self.topic_prefix}/state')
        self.get_logger().info(f'  - {self.topic_prefix}/battery')
        self.get_logger().info(f'  - {self.topic_prefix}/status')
        self.get_logger().info('='*70)
        self.get_logger().info('✅ Device Controller READY')

    # ===== Localization Callback

    def _localization_callback(self, msg: PoseStamped):
        self.sphero.set_external_location(msg.pose.position.x, msg.pose.position.y)

    # ===== Command Callbacks =====

    def led_callback(self, msg: String):
        """Handle LED color commands."""
        try:
            data = json.loads(msg.data)
            red = data.get('red', 0)
            green = data.get('green', 0)
            blue = data.get('blue', 0)
            # Accept both 'type' and 'led' for backwards compatibility
            led_type = data.get('type', data.get('led', 'main')).lower()

            success = self.sphero.set_led(red, green, blue, led_type)
            if success:
                self.get_logger().info(f'{led_type.capitalize()} LED set to RGB({red}, {green}, {blue})')
            else:
                self.get_logger().warning(f'{led_type.capitalize()} LED not supported or failed')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in LED command: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error in LED callback: {str(e)}')

    def _publish_motion_cmd(self, speed: int, heading: int, is_moving: bool):
        """Echo the commanded motion for external consumers.

        Called from whichever thread issued the command, including the task
        executor's. Publishing is cheap and rclpy publishers are thread-safe, so
        it happens inline; a failure here must never break the actuator call.
        """
        try:
            self.motion_cmd_pub.publish(String(data=json.dumps({
                'speed': int(speed),
                'heading': int(heading),
                'is_moving': bool(is_moving),
            })))
        except Exception as e:                                  # noqa: BLE001
            self.get_logger().warning(f'motion_cmd publish failed: {e}')

    def roll_callback(self, msg: String):
        """Handle roll commands."""
        try:
            data = json.loads(msg.data)
            heading = int(data.get('heading', 0))
            speed = int(data.get('speed', 100))
            duration = float(data.get('duration', 0))

            success = self.sphero.roll(heading, speed, duration)
            if success:
                if duration > 0:
                    self.get_logger().info(
                        f'Rolling at heading {heading}deg with speed {speed} for {duration}s')
                else:
                    self.get_logger().info(
                        f'Rolling continuously at heading {heading}deg with speed {speed}')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in roll command: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error in roll callback: {str(e)}')

    def spin_callback(self, msg: String):
        """Handle spin commands.

        spherov2's ``api.spin()`` BUSY-LOOPS for the whole ``duration`` (ramping
        heading) and holds its movement lock the entire time. Calling it inline
        would block this node's single callback thread and starve every other
        device command -- e.g. a concurrent Similarity blink's ``set_led`` would
        be queued until the spin ends, making blink and spin run sequentially
        instead of together. Run the spin on a background daemon thread so this
        callback returns immediately and LED/matrix/sensor callbacks keep flowing.

        Safe to overlap: ``set_main_led`` / ``set_matrix_pixel`` do NOT take
        spherov2's movement lock, and all BLE writes are serialized onto the
        bleak adapter's own loop thread, so the spin's heading writes and the
        blink's LED writes interleave without corruption.
        """
        try:
            data = json.loads(msg.data)
            angle = int(data.get('angle', 360))
            duration = float(data.get('duration', 1.0))

            def _run_spin():
                try:
                    self.sphero.spin(angle, duration)
                except Exception as e:
                    self.get_logger().error(f'Error during background spin: {str(e)}')

            threading.Thread(
                target=_run_spin, name='sphero-spin', daemon=True).start()
            self.get_logger().info(f'Spinning {angle}deg over {duration}s (async)')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in spin command: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error in spin callback: {str(e)}')

    def heading_callback(self, msg: String):
        """Handle heading commands."""
        try:
            data = json.loads(msg.data)
            heading = int(data.get('heading', 0))

            success = self.sphero.set_heading(heading)
            if success:
                self.get_logger().info(f'Heading set to {heading}deg')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in heading command: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error in heading callback: {str(e)}')

    def speed_callback(self, msg: String):
        """Handle speed commands."""
        try:
            data = json.loads(msg.data)
            speed = int(data.get('speed', 0))
            duration = float(data.get('duration', 0))

            success = self.sphero.set_speed(speed, duration)
            if success:
                self.get_logger().info(f'Speed set to {speed}')
                if duration > 0:
                    self.get_logger().info(f'Duration: {duration}s')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in speed command: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error in speed callback: {str(e)}')

    def raw_motor_callback(self, msg: String):
        """Handle raw motor commands."""
        try:
            from spherov2.commands.sphero import RawMotorModes

            data = json.loads(msg.data)
            left_mode_str = data.get('left_mode', 'forward').lower()
            left_speed = int(data.get('left_speed', 0))
            right_mode_str = data.get('right_mode', 'forward').lower()
            right_speed = int(data.get('right_speed', 0))

            # Map mode strings to RawMotorModes enum
            mode_map = {
                'forward': RawMotorModes.FORWARD,
                'fwd': RawMotorModes.FORWARD,
                'reverse': RawMotorModes.REVERSE,
                'rev': RawMotorModes.REVERSE,
                'brake': RawMotorModes.BRAKE,
                'off': RawMotorModes.OFF,
                'ignore': RawMotorModes.IGNORE
            }

            left_mode = mode_map.get(left_mode_str, RawMotorModes.FORWARD)
            right_mode = mode_map.get(right_mode_str, RawMotorModes.FORWARD)

            success = self.sphero.set_raw_motor_speed(left_mode, left_speed, right_mode, right_speed)
            if success:
                self.get_logger().info(
                    f'Raw motors set - Left: {left_mode_str}@{left_speed}, Right: {right_mode_str}@{right_speed}')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in raw_motor command: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error in raw_motor callback: {str(e)}')

    def stop_callback(self, msg: String):
        """Handle stop commands."""
        try:
            success = self.sphero.stop()
            if success:
                self.get_logger().info('Sphero stopped')

        except Exception as e:
            self.get_logger().error(f'Error in stop callback: {str(e)}')

    def reset_aim_callback(self, msg: String):
        """Handle reset aim commands."""
        try:
            success = self.sphero.reset_aim()
            if success:
                self.get_logger().info('Sphero reset to origin: heading=0°, position=(0, 0)')

        except Exception as e:
            self.get_logger().error(f'Error in reset_aim callback: {str(e)}')

    def calibrate_compass_callback(self, msg: String):
        """Handle compass calibration commands (BOLT only)."""
        success = False
        try:
            self.get_logger().info('Calibrating compass (robot will spin)...')
            # NOTE: this BLOCKS this callback thread until calibration completes
            # (~seconds): the robot physically spins and spherov2 waits for the
            # magnetometer calibration notify before returning.
            success = self.sphero.calibrate_compass()
            if success:
                self.get_logger().info('Compass calibrated')
            else:
                self.get_logger().warning(
                    'Compass calibration not supported (BOLT only) or failed')

        except Exception as e:
            self.get_logger().error(f'Error in calibrate_compass callback: {str(e)}')
        finally:
            # Always signal completion so a waiting registrar unblocks even on
            # failure/unsupported (success flag carries the outcome).
            done = Bool()
            done.data = bool(success)
            self.calibrate_compass_done_pub.publish(done)

    def matrix_callback(self, msg: String):
        """Handle LED matrix commands (BOLT only)."""
        try:
            data = json.loads(msg.data)
            pattern = data.get('pattern', '')
            custom_matrix = data.get('matrix', [])
            red = int(data.get('red', 255))
            green = int(data.get('green', 255))
            blue = int(data.get('blue', 255))
            duration = float(data.get('duration', 0))

            # Clear convention (e.g. /api/matrix/clear and _stop_lane(MATRIX)):
            # empty pattern + no custom matrix means "blank the device". set_matrix
            # no-ops on empty input, so route this to clear_matrix() explicitly.
            if not pattern and not custom_matrix:
                success = self.sphero.clear_matrix()
                if success:
                    self.get_logger().info('Matrix cleared')
                else:
                    self.get_logger().warning('Matrix clear not supported')
                return

            success = self.sphero.set_matrix(
                pattern=pattern if pattern else None,
                custom_matrix=custom_matrix if custom_matrix else None,
                red=red, green=green, blue=blue,
                duration=duration
            )

            if success:
                self.get_logger().info(f'Matrix pattern "{pattern}" displayed')
            else:
                self.get_logger().warning('Matrix not supported or invalid pattern')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in matrix command: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error in matrix callback: {str(e)}')

    def collision_callback(self, msg: String):
        """Handle collision detection commands."""
        try:
            data = json.loads(msg.data)
            action = data.get('action', '')

            if action == 'start':
                mode = data.get('mode', 'obstacle')
                sensitivity = data.get('sensitivity', 'HIGH')

                # Set up callbacks for publishing collision events
                def tap_callback(timestamp):
                    tap_msg = String()
                    tap_msg.data = json.dumps({
                        'type': 'tap',
                        'timestamp': timestamp
                    })
                    self.tap_pub.publish(tap_msg)
                    self.get_logger().info('Tap detected!')

                def obstacle_callback(timestamp):
                    obstacle_msg = String()
                    obstacle_msg.data = json.dumps({
                        'type': 'obstacle',
                        'timestamp': timestamp
                    })
                    self.obstacle_pub.publish(obstacle_msg)
                    self.get_logger().info('Obstacle collision detected!')

                success = self.sphero.start_collision_detection(
                    mode=mode,
                    sensitivity=sensitivity,
                    tap_callback=tap_callback,
                    obstacle_callback=obstacle_callback
                )

                if success:
                    self.get_logger().info(
                        f'Started collision detection in "{mode}" mode with {sensitivity} sensitivity')
                else:
                    self.get_logger().error('Failed to start collision detection')

            elif action == 'stop':
                success = self.sphero.stop_collision_detection()
                if success:
                    self.get_logger().info('Stopped collision detection')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in collision command: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error in collision callback: {str(e)}')

    def stabilization_callback(self, msg: String):
        """Handle stabilization commands."""
        try:
            data = json.loads(msg.data)
            enable = data.get('enable', True)

            success = self.sphero.set_stabilization(enable)
            if success:
                status = "enabled" if enable else "disabled"
                self.get_logger().info(f'Stabilization {status}')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in stabilization command: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error in stabilization callback: {str(e)}')

    def ir_callback(self, msg: String):
        """Handle robot-to-robot IR commands (BOLT only).

        One topic multiplexes all six IR actions via the `action` field:
        broadcast / follow / evade (carry near, far) and their *_stop variants.
        """
        try:
            data = json.loads(msg.data)
            action = data.get('action', '')
            near = int(data.get('near', 0))
            far = int(data.get('far', 0))

            if action == 'broadcast':
                self.sphero.start_ir_broadcast(near, far)
            elif action == 'follow':
                self.sphero.start_ir_follow(near, far)
            elif action == 'evade':
                self.sphero.start_ir_evade(near, far)
            elif action == 'broadcast_stop':
                self.sphero.stop_ir_broadcast()
            elif action == 'follow_stop':
                self.sphero.stop_ir_follow()
            elif action == 'evade_stop':
                self.sphero.stop_ir_evade()
            else:
                self.get_logger().warning(f'Unknown IR action: {action}')
                return
            self.get_logger().info(f'IR command: {action} (near={near}, far={far})')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in ir command: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error in ir callback: {str(e)}')

    # ===== Publishing Methods =====

    def publish_sensors(self):
        """Publish sensor data periodically."""
        # No /state or /sensors from a dead link: update_sensors swallows read
        # errors, so these would otherwise be stale cached values.
        # Nor while the quiet gate holds: the stream is stopped, the cache stale.
        if (self._quiet_hold or self.ble_link_down
                or not getattr(self.sphero.robot, 'is_connected', True)):
            return
        try:
            # Update sensors from device
            self.sphero.update_sensors(streamed=self._stream_on)

            # Publish complete state as JSON
            state_msg = String()
            state_msg.data = json.dumps(self.sphero.get_state_dict())
            self.state_pub.publish(state_msg)

            # Publish sensor data using SpheroSensor message format
            sensor_msg = self.sphero.get_sensor_msg()
            if sensor_msg is not None:
                sensor_msg.timestamp = self.get_clock().now().to_msg()
                self.sensor_pub.publish(sensor_msg)

        except Exception as e:
            self.get_logger().error(f'Error publishing sensors: {str(e)}', throttle_duration_sec=5.0)

    def publish_heartbeat(self):
        """Publish heartbeat with battery health information."""
        # While the link is down main() owns /status ('reconnecting' repeats).
        if self.ble_link_down:
            return
        try:
            # The only periodic battery read (two BLE round trips). Skipped
            # while the quiet gate holds; /status still goes out (cached).
            if not self._quiet_hold:
                self.sphero.update_battery()
            battery_percentage = self.sphero.get_battery_percentage()

            # Publish heartbeat as JSON
            heartbeat_msg = String()
            heartbeat_msg.data = json.dumps(self._status_payload())
            self.status_pub.publish(heartbeat_msg)

            # Publish battery state message
            battery_msg = self.sphero.get_battery_msg()
            if battery_msg is not None:
                battery_msg.header.stamp = self.get_clock().now().to_msg()
                battery_msg.header.frame_id = f'sphero_{self.sphero_name}'
                self.battery_pub.publish(battery_msg)

            # Log warnings for low battery
            if battery_percentage < 20:
                self.get_logger().warning(
                    f'Low battery: {battery_percentage}%',
                    throttle_duration_sec=30.0)
            elif battery_percentage < 10:
                self.get_logger().error(
                    f'Critical battery: {battery_percentage}%',
                    throttle_duration_sec=30.0)

            # Log heartbeat
            self.get_logger().info(
                f'Heartbeat: {self.sphero_name} - Battery: {battery_percentage}% - Healthy: {self.sphero.is_healthy()}',
                throttle_duration_sec=30.0)

        except Exception as e:
            self.get_logger().error(f'Error publishing heartbeat: {str(e)}')

    def _status_payload(self):
        """Build the /status JSON dict for the current connection_state.

        battery / is_healthy are last-known cached values; they are only
        meaningful while connection_state is 'connected'.
        """
        battery_percentage = self.sphero.get_battery_percentage()
        return {
            'sphero_name': self.sphero_name,
            'timestamp': self.get_clock().now().to_msg().sec,
            'connection_state': self.connection_state,
            'battery': {
                'percentage': battery_percentage,
                'health': 'GOOD' if battery_percentage > 10 else 'DEAD',
                'status': 'FULL' if battery_percentage >= 100 else 'DISCHARGING'
            },
            'is_healthy': self.sphero.is_healthy(),
            # BLE quiet gate holding (connected, stream/LEDs deferred).
            'holding': bool(self._quiet_hold),
            # Post-connect setup (LEDs, arrow, battery) finished for this link;
            # False while holding, during that setup and while reconnecting.
            'startup_complete': bool(self._startup_complete),
            # Worker-agent spawn identity (SPHERO_SPAWN_ID), None if not spawned by one.
            'spawn_id': self.spawn_id,
        }

    def publish_connection_state(self, state):
        """Set connection_state and publish it on /status immediately.

        Safe to call without spinning and from the reconnect beacon thread
        (rclpy publishers are thread-safe).
        """
        self.connection_state = state
        try:
            self.status_pub.publish(String(data=json.dumps(self._status_payload())))
        except Exception as e:
            self.get_logger().error(f'Error publishing connection state: {str(e)}')

    # ===== Runtime BLE Liveness / Reconnect Support =====

    def _on_ble_disconnect(self):
        """spherov2 disconnect listener (runs on a spherov2 worker thread).

        Only flags the event; main() turns it into mark_link_down().
        """
        self._ble_lost_evt.set()

    def _attach_disconnect_listener(self):
        """Register _on_ble_disconnect on the current robot (spherov2 >= 0.13.0)."""
        add = getattr(self.sphero.robot, 'add_disconnect_listener', None)
        if add is not None:
            add(self._on_ble_disconnect)

    def _on_stream_packet(self, _data):
        """Sensor-stream listener (spherov2 worker thread): stamp arrival time."""
        self._last_stream_rx = time.monotonic()

    def _attach_stream_listener(self):
        """Register _on_stream_packet on the current robot's sensor stream.

        Same toy-level hook spherov2's SensorControl uses. Models without it
        never stamp, so the liveness probe keeps doing round trips.
        """
        add = getattr(self.sphero.robot, 'add_sensor_streaming_data_notify_listener', None)
        if add is not None:
            add(self._on_stream_packet)

    def _apply_sensor_stream(self):
        """Replace spherov2's sensor stream with `sensor_stream` every
        `sensor_stream_interval_ms`.

        SpheroEduAPI.__enter__ enables every sensor of the model at 150 ms; we
        consume only attitude (yaw), but the stream itself keeps the liveness
        probe off the air, so it is trimmed, not stopped. SensorControl has no
        "replace", and each enable/disable/set_interval re-sends the masks
        (stop, extended mask, start). Its enabled sets are also the stream's
        decode table, so the masks must go through it: set the interval and
        clear the sets on its (spherov2 0.13.0, name-mangled) state, then one
        enable() sends a single update. Unknown names are skipped with a
        warning; ['default'] leaves the stream untouched.
        """
        wanted = self.sensor_stream
        if not wanted or wanted in ([SENSOR_STREAM_KEEP_DEFAULT], [SENSOR_STREAM_OFF]):
            return
        robot = self.sphero.robot
        control = getattr(robot, 'sensor_control', None)
        if control is None or not hasattr(control, '_SensorControl__enabled_extended'):
            self.get_logger().warning(
                'No spherov2 SensorControl on this robot; leaving the sensor stream as is.')
            return
        supported = set(robot.sensors) | set(robot.extended_sensors)
        unknown = [s for s in wanted if s not in supported]
        if unknown:
            self.get_logger().warning(
                f'Ignoring unknown sensor_stream entries {unknown} for '
                f'{type(robot).__name__}; supported: {sorted(supported)}')
        known = [s for s in wanted if s in supported]
        if not known or self.sensor_stream_interval_ms <= 0:
            self.get_logger().warning(
                'No valid sensor_stream / sensor_stream_interval_ms; leaving '
                "spherov2's default sensor stream on.")
            return
        if not self.external_location and 'locator' not in known:
            self.get_logger().warning(
                "external_localization is off but 'locator' is not in sensor_stream: "
                'position will not update.')
        try:
            control._SensorControl__interval = int(self.sensor_stream_interval_ms)
            control._SensorControl__enabled.clear()
            control._SensorControl__enabled_extended.clear()
            control.enable(*known)
        except Exception as e:
            self.get_logger().error(f'Could not apply sensor_stream: {e}')
            return
        self.get_logger().info(
            f'Sensor stream: {known} every {self.sensor_stream_interval_ms} ms')

    def detach_disconnect_listener(self):
        """Unregister from the current robot, so tearing the dead link down
        can't re-trip the event."""
        remove = getattr(self.sphero.robot, 'remove_disconnect_listener', None)
        if remove is not None:
            remove(self._on_ble_disconnect)

    def mark_link_down(self, reason):
        """Declare the link down: publish 'reconnecting' now and keep repeating it.

        main() blocks in the reconnect loop without spinning (scan, connect,
        lock wait, backoff), so a daemon thread repeats 'reconnecting' every
        BLE_RECONNECTING_REPEAT seconds until stop_reconnect_beacon().
        """
        if self.ble_link_down:
            return
        self.ble_link_down = True
        self._startup_complete = False
        self.get_logger().error(
            f'BLE link to {self.sphero_name} declared DOWN ({reason}); '
            f'handing off to reconnect.')
        self.publish_connection_state('reconnecting')
        stop = threading.Event()
        self._beacon_stop = stop

        def _beacon():
            while not stop.wait(BLE_RECONNECTING_REPEAT):
                self.publish_connection_state('reconnecting')

        threading.Thread(target=_beacon, name='sphero-reconnect-beacon',
                         daemon=True).start()

    def stop_reconnect_beacon(self):
        """Stop the 'reconnecting' repeater (no-op if not running)."""
        if self._beacon_stop is not None:
            self._beacon_stop.set()
            self._beacon_stop = None

    def _ble_liveness_probe(self):
        """Periodic real round-trip to detect a dead BLE link.

        Sensors swallow per-read exceptions, so they can't surface a drop. This
        probe calls `Sphero.check_link()`, a response-bearing read with no
        visible side effect. (It used to re-apply the main LED, but on a BOLT
        `set_main_led` floods the whole matrix, wiping any drawn graphic every
        second.) A `ToyDisconnectedError` (spherov2 >= 0.13.0) is definitive and
        trips immediately; any other link-dead error needs
        `BLE_LIVENESS_FAIL_THRESHOLD` consecutive failures. Tripping calls
        `mark_link_down`, which main() acts on. The round trip is skipped while
        sensor-stream packets keep arriving (see `BLE_STREAM_FRESH_SEC`).
        """
        # main() owns the reconnect lifecycle once the link is declared down;
        # don't probe a connection that's being torn down / rebuilt. Nor while
        # the quiet gate holds (the disconnect listener still catches drops).
        if self.ble_link_down or self._quiet_hold:
            return
        # A recent sensor-stream packet proves the link is up: no round trip.
        last = self._last_stream_rx
        if last is not None and time.monotonic() - last < BLE_STREAM_FRESH_SEC:
            self._ble_fail_streak = 0
            return
        try:
            self.sphero.check_link()
            # A successful round trip means the link is alive.
            self._ble_fail_streak = 0
        except Exception as exc:
            definitive = (ToyDisconnectedError is not None
                          and isinstance(exc, ToyDisconnectedError))
            if not definitive and not _is_link_dead_error(exc):
                # Transient (e.g. packet-decode collision): treat as alive.
                self._ble_fail_streak = 0
                self.get_logger().warning(
                    f'Liveness probe transient error (ignored): {exc}',
                    throttle_duration_sec=5.0)
                return
            self._ble_fail_streak += 1
            self.get_logger().warning(
                f'BLE liveness probe failed '
                f'({self._ble_fail_streak}/{BLE_LIVENESS_FAIL_THRESHOLD}): {exc}')
            if definitive or self._ble_fail_streak >= BLE_LIVENESS_FAIL_THRESHOLD:
                self.mark_link_down(
                    f'{self._ble_fail_streak} probe failure(s): {exc}')

    def rebind_connection(self, robot, api):
        """Swap a freshly reconnected robot/api into the live node.

        Command callbacks read `self.sphero.api` on every call, so replacing the
        handles (plus the state's api/toy refs) between calls is sufficient for
        all command + sensor paths to use the new link. Clears the down/streak
        flags so the liveness probe resumes.
        """
        self.sphero.robot = robot
        self.sphero.api = api
        self.sphero.state.set_api(api)
        self.sphero.state.set_toy(robot)
        self._ble_fail_streak = 0
        self.ble_link_down = False
        self._ble_lost_evt.clear()
        self._attach_disconnect_listener()
        self._last_stream_rx = None
        self._attach_stream_listener()
        # Stream, LEDs, battery: now, or when this host's connects are done.
        self._quiet_gate_enter()
        self.stop_reconnect_beacon()
        self.publish_connection_state('connected')
        self.get_logger().info(
            f'Rebound live BLE connection for {self.sphero_name}; resuming.')

    def publish_ble_lost(self, last_error):
        """Publish a terminal `ble_lost` device_error after reconnect exhaustion.

        Mirrors the initial-connect failure format/topic so the webserver's
        existing device_error_callback receives it. Published repeatedly with
        spin to ensure delivery before the process exits.
        """
        if self._device_error_pub is None:
            self._device_error_pub = self.create_publisher(
                String, f'{self.topic_prefix}/device_error', 10)

        error_msg = String()
        error_msg.data = json.dumps({
            'error': 'ble_lost',
            'sphero_name': self.sphero_name,
            'message': str(last_error)
        })
        self.stop_reconnect_beacon()
        for _ in range(10):
            self.publish_connection_state('disconnected')
            self._device_error_pub.publish(error_msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)
        self.get_logger().error(
            f'Published ble_lost device_error for {self.sphero_name}; exiting.')

    def cleanup(self):
        """Clean up resources before shutdown."""
        self.get_logger().info('Cleaning up Sphero instance device controller...')
        try:
            self.sphero.cleanup()
            self.get_logger().info('Sphero cleanup complete')
        except Exception as e:
            self.get_logger().error(f'Error during cleanup: {e}')


def _reconnect(node, sphero_name):
    """Try BLE_RECONNECT_ATTEMPTS fresh scan+connects and rebind the node.

    Blocks without spinning; node.mark_link_down() already started the
    'reconnecting' beacon. Returns (context_manager, None) on success, or
    (None, last_error) once all attempts are exhausted.
    """
    reconnect_error = None
    for attempt in range(1, BLE_RECONNECT_ATTEMPTS + 1):
        # Backoff first: gives a power-cycled unit time to come back
        # and avoids hammering the adapter immediately after a drop.
        time.sleep(BLE_RECONNECT_BACKOFF)
        attempt_cm = None
        try:
            with ble_connect_lock(label=f'{sphero_name}-reconnect'):
                new_robot = scanner.find_toy(toy_name=sphero_name)
                attempt_cm = SpheroEduAPI(toy=new_robot)
                new_api = attempt_cm.__enter__()
                # Still under the lock: stop the full stream __enter__ started.
                quiet_sensor_stream(new_robot, node.sensor_stream,
                                    node.sensor_stream_interval_ms)
            node.rebind_connection(new_robot, new_api)
            print(f"Reconnected to {sphero_name} "
                  f"(attempt {attempt}/{BLE_RECONNECT_ATTEMPTS}).")
            return attempt_cm, None
        except Exception as re_err:
            reconnect_error = re_err
            print(f"✗ Reconnect attempt {attempt}/"
                  f"{BLE_RECONNECT_ATTEMPTS} for {sphero_name} "
                  f"failed: {re_err}")
            if attempt_cm is not None:
                try:
                    attempt_cm.__exit__(None, None, None)
                except Exception:
                    pass
    return None, reconnect_error


def main(args=None):
    """Main entry point for the Sphero instance device controller node."""
    rclpy.init(args=args)
    node = None
    temp_node = None
    beacon = None
    shutdown_requested = False

    def signal_handler(_sig, _frame):
        nonlocal shutdown_requested
        print("\nShutdown signal received. Shutting down...")
        shutdown_requested = True

    # Register signal handler for SIGINT (Ctrl+C) and SIGTERM (the webserver /
    # worker agent removing the robot), so either path publishes 'disconnected'.
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        # Create a temporary node to read the sphero_name parameter
        temp_node = rclpy.create_node('temp_param_node')
        temp_node.declare_parameter('sphero_name', '')  # No default - REQUIRED parameter
        sphero_name = temp_node.get_parameter('sphero_name').value

        if not sphero_name:
            raise ValueError(
                "The 'sphero_name' parameter is required but was not provided. "
                "Please launch with: ros2 run sphero_instance_controller sphero_instance_device_controller_node.py "
                "--ros-args -p sphero_name:=<YOUR_SPHERO_NAME>"
            )

        print(f"Sphero name from parameter: {sphero_name}")
        # The node reads these too; needed here for quiet_sensor_stream.
        temp_node.declare_parameter('sensor_stream', SENSOR_STREAM)
        temp_node.declare_parameter('sensor_stream_interval_ms', SENSOR_STREAM_INTERVAL_MS)
        stream_sensors = list(temp_node.get_parameter('sensor_stream').value)
        stream_interval_ms = temp_node.get_parameter('sensor_stream_interval_ms').value

        # Sanitize topic name for publishing error status
        topic_name_safe = sphero_name.replace("-", "_")
        topic_prefix = f'sphero/{topic_name_safe}'

        # Report 'initializing' (until this host's connect lock is ours) then
        # 'connecting' on /status, repeated until the node takes over /status.
        spawn_id = os.environ.get(ENV_SPAWN_ID) or None
        beacon = PreConnectBeacon(
            temp_node.create_publisher(String, f'{topic_prefix}/status', 10),
            sphero_name, spawn_id)
        beacon.set_stage('initializing')

        # Scan + connect, serialized per host (see ble_connect_lock) with retry.
        # BlueZ rejects parallel connects on one adapter, so only one process on a
        # host runs find_toy + the SpheroEduAPI connect at a time. We enter the
        # SpheroEduAPI context MANUALLY so the connection can outlive the lock: the
        # lock covers scan+connect only, then is released before the spin loop.
        print(f"Scanning for Sphero robot: {sphero_name}...")
        robot = None
        api = None
        cm = None                # the SpheroEduAPI context manager, kept for __exit__
        connected = False
        last_error = None
        error_code = 'connect_failed'
        max_attempts = 3
        retry_backoff = 2.5      # seconds between attempts

        # Until the initial connect ends, mark this process as pending so that
        # robots already connected on this host stay BLE-quiet (quiet gate).
        with pending_connect_marker(label=sphero_name):
            for attempt in range(1, max_attempts + 1):
                attempt_cm = None
                try:
                    with ble_connect_lock(label=sphero_name):
                        beacon.set_stage('connecting')
                        # Scan (contends on the adapter too, so inside the lock).
                        robot = scanner.find_toy(toy_name=sphero_name)
                        # Manual __enter__ = the BLE connect. Kept open past the lock.
                        attempt_cm = SpheroEduAPI(toy=robot)
                        api = attempt_cm.__enter__()
                        # Still under the lock: stop the full stream __enter__
                        # started, so these writes can't overlap a neighbour.
                        quiet_sensor_stream(robot, stream_sensors, stream_interval_ms)
                        # Success: hand off the context manager and leave the lock.
                        cm = attempt_cm
                        connected = True
                    print(f"[ble-gate] t={_epoch():.3f} Connected to {sphero_name} "
                          f"(attempt {attempt}/{max_attempts})", flush=True)
                    break
                except Exception as connect_error:
                    last_error = connect_error
                    # Distinguish scan failure (no robot yet) from connect failure.
                    error_code = 'toy_not_found' if robot is None else 'connect_failed'
                    print(f"✗ Attempt {attempt}/{max_attempts} for {sphero_name} "
                          f"failed: {connect_error}")
                    # Close any half-open connection from this attempt.
                    if attempt_cm is not None:
                        try:
                            attempt_cm.__exit__(None, None, None)
                        except Exception:
                            pass
                    robot = None
                    api = None
                    if attempt < max_attempts:
                        print(f"Retrying {sphero_name} in {retry_backoff}s...")
                        time.sleep(retry_backoff)

        beacon.stop()
        if not connected:
            # All attempts failed - publish error status via temp_node before exiting.
            print(f"[ble-gate] t={_epoch():.3f} ✗ Failed to connect to Sphero {sphero_name} after "
                  f"{max_attempts} attempts: {last_error}")

            error_pub = temp_node.create_publisher(String, f'{topic_prefix}/device_error', 10)
            error_msg = String()
            error_msg.data = json.dumps({
                'error': error_code,
                'sphero_name': sphero_name,
                'message': str(last_error),
                'spawn_id': spawn_id,
            })

            # Publish multiple times and spin to ensure delivery
            for _ in range(10):
                error_pub.publish(error_msg)
                rclpy.spin_once(temp_node, timeout_sec=0.05)
                time.sleep(0.05)

            print(f"Published error status for {sphero_name} - exiting")
            # Clean exit without raising exception
            temp_node.destroy_node()
            temp_node = None
            rclpy.shutdown()
            return  # Exit cleanly

        # Connected. Destroy temporary node before creating controller node.
        temp_node.destroy_node()
        temp_node = None

        # Spin OUTSIDE the lock; guarantee the connection closes on shutdown/exception.
        # `cm` is the live context manager; it gets swapped on a successful
        # runtime reconnect, and the outer finally always __exit__s the latest one.
        try:
            # Create the node
            node = SpheroInstanceDeviceController(robot, api, sphero_name)

            # Spin until shutdown requested. The liveness probe (a node timer)
            # sets node.ble_link_down when the link dies; we break out to run the
            # reconnect lifecycle, then resume spinning on success.
            while rclpy.ok() and not shutdown_requested:
                rclpy.spin_once(node, timeout_sec=0.1)

                if node._ble_lost_evt.is_set():
                    node.mark_link_down('spherov2 disconnect listener')

                if node.ble_link_down:
                    # The live link is dead. Tear down the dead context manager
                    # cleanly before attempting fresh scan+connect under the lock.
                    print(f"BLE link to {sphero_name} down; starting reconnect "
                          f"({BLE_RECONNECT_ATTEMPTS} attempts).")
                    node.detach_disconnect_listener()
                    try:
                        cm.__exit__(None, None, None)
                    except Exception:
                        pass
                    cm = None  # closed; never __exit__ it again if _reconnect raises
                    cm, reconnect_error = _reconnect(node, sphero_name)

                    if cm is None:
                        # All reconnect attempts exhausted. Report the terminal
                        # error ('disconnected' + ble_lost) and exit cleanly.
                        print(f"✗ Failed to reconnect to {sphero_name} after "
                              f"{BLE_RECONNECT_ATTEMPTS} attempts: {reconnect_error}")
                        node.publish_ble_lost(reconnect_error)
                        break

            if shutdown_requested and rclpy.ok():
                # Deliberate shutdown (Ctrl-C / SIGTERM from the dashboard):
                # tell the webserver now, before the link is closed.
                node.stop_reconnect_beacon()
                for _ in range(3):
                    node.publish_connection_state('disconnected')
                    time.sleep(0.05)
        finally:
            # Manual __exit__ mirrors the manual __enter__ done under the lock.
            if cm is not None:
                try:
                    cm.__exit__(None, None, None)
                except Exception:
                    pass

    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

    finally:
        if beacon is not None:
            beacon.stop()
        # Clean up temporary node if it exists
        if temp_node:
            temp_node.destroy_node()

        # Clean up the controller node
        if node:
            node.stop_reconnect_beacon()
            node.cleanup()
            node.destroy_node()

        # Shutdown ROS 2
        if rclpy.ok():
            rclpy.shutdown()

        print("Goodbye!")


if __name__ == '__main__':
    main()
