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
import random
import os
import fcntl
from contextlib import contextmanager

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped

from sphero_instance_controller.msg import SpheroSensor

from spherov2 import scanner
from spherov2.sphero_edu import SpheroEduAPI

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
        self.declare_parameter('heartbeat_rate', 0)  # Default: 5 seconds
        self.declare_parameter('external_localization', False) # Default: turned OFF

        self.sensor_rate = self.get_parameter('sensor_rate').value
        self.heartbeat_rate = self.get_parameter('heartbeat_rate').value
        self.external_location = self.get_parameter('external_localization').value


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

        # Create timers
        self.sensor_timer = self.create_timer(self.sensor_period, self.publish_sensors)

        self.liveness_timer = self.create_timer(
            BLE_LIVENESS_PERIOD, self._ble_liveness_probe)

        if self.heartbeat_rate > 0:
            self.heartbeat_timer = self.create_timer(self.heartbeat_rate, self.publish_heartbeat)

        # Log initialization info
        self._log_initialization()

        # Initial front and back LEDs
        self.sphero.set_led(0, 255, 0, 'front')
        self.sphero.set_led(255, 0, 0, 'back')
        self.sphero.set_led(0, 0, 0, 'main')

        # Set the arrow symbol to indicate heading
        red = 128 * random.randint(0,2)
        green = 128 * random.randint(0,2)
        blue = 128 * random.randint(0,2)
        self.sphero.set_matrix('arrow_right', None, red, green, blue)

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

        self.obstacle_pub = self.create_publisher(
            String, f'{self.topic_prefix}/obstacle', 10)

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
        """Handle spin commands."""
        try:
            data = json.loads(msg.data)
            angle = int(data.get('angle', 360))
            duration = float(data.get('duration', 1.0))

            success = self.sphero.spin(angle, duration)
            if success:
                self.get_logger().info(f'Spinning {angle}deg over {duration}s')

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
        try:
            # Update sensors from device
            self.sphero.update_sensors()

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
        try:
            battery_percentage = self.sphero.get_battery_percentage()

            # Create heartbeat message
            heartbeat_data = {
                'sphero_name': self.sphero_name,
                'timestamp': self.get_clock().now().to_msg().sec,
                'connection_state': 'connected',
                'battery': {
                    'percentage': battery_percentage,
                    'health': 'GOOD' if battery_percentage > 10 else 'DEAD',
                    'status': 'FULL' if battery_percentage >= 100 else 'DISCHARGING'
                },
                'is_healthy': self.sphero.is_healthy()
            }

            # Publish heartbeat as JSON
            heartbeat_msg = String()
            heartbeat_msg.data = json.dumps(heartbeat_data)
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

    # ===== Runtime BLE Liveness / Reconnect Support =====

    def _ble_liveness_probe(self):
        """Periodic real round-trip to detect a dead BLE link.

        Sensors swallow per-read exceptions, so they can't surface a drop. This
        probe issues a real BLE write: it re-applies the CURRENT main-LED color
        via `set_main_led`. `get_main_led()` is a cached dict lookup that never
        round-trips, but `set_main_led` goes through spherov2's ToyUtil ->
        adapter.write path, which raises (BleakError / disconnect / timeout) on a
        dead link. Re-applying the existing color makes the write side-effect a
        no-op visually. A SINGLE failure never trips reconnect: only
        `BLE_LIVENESS_FAIL_THRESHOLD` consecutive link-dead failures set
        `ble_link_down`, which main() acts on.
        """
        # main() owns the reconnect lifecycle once the link is declared down;
        # don't probe a connection that's being torn down / rebuilt.
        if self.ble_link_down:
            return
        try:
            # Re-apply the current main color (visual no-op) as a real round-trip.
            current = self.sphero.api.get_main_led()  # cached value
            if current is None:
                from spherov2.types import Color
                current = Color(0, 0, 0)
            self.sphero.api.set_main_led(current)
            # A successful write means the link is alive.
            self._ble_fail_streak = 0
        except Exception as exc:
            if not _is_link_dead_error(exc):
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
            if self._ble_fail_streak >= BLE_LIVENESS_FAIL_THRESHOLD:
                self.ble_link_down = True
                self.get_logger().error(
                    f'BLE link to {self.sphero_name} declared DOWN after '
                    f'{self._ble_fail_streak} consecutive probe failures; '
                    f'handing off to reconnect.')

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
        for _ in range(10):
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


def main(args=None):
    """Main entry point for the Sphero instance device controller node."""
    rclpy.init(args=args)
    node = None
    temp_node = None
    shutdown_requested = False

    def signal_handler(_sig, _frame):
        nonlocal shutdown_requested
        print("\nKeyboard interrupt detected. Shutting down...")
        shutdown_requested = True

    # Register signal handler for SIGINT (Ctrl+C)
    signal.signal(signal.SIGINT, signal_handler)

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

        # Sanitize topic name for publishing error status
        topic_name_safe = sphero_name.replace("-", "_")
        topic_prefix = f'sphero/{topic_name_safe}'

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

        for attempt in range(1, max_attempts + 1):
            attempt_cm = None
            try:
                with ble_connect_lock(label=sphero_name):
                    # Scan (contends on the adapter too, so inside the lock).
                    robot = scanner.find_toy(toy_name=sphero_name)
                    # Manual __enter__ = the BLE connect. Kept open past the lock.
                    attempt_cm = SpheroEduAPI(toy=robot)
                    api = attempt_cm.__enter__()
                    # Success: hand off the context manager and leave the lock.
                    cm = attempt_cm
                    connected = True
                print(f"Connected to {sphero_name} (attempt {attempt}/{max_attempts})")
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

        if not connected:
            # All attempts failed - publish error status via temp_node before exiting.
            print(f"✗ Failed to connect to Sphero {sphero_name} after "
                  f"{max_attempts} attempts: {last_error}")

            error_pub = temp_node.create_publisher(String, f'{topic_prefix}/device_error', 10)
            error_msg = String()
            error_msg.data = json.dumps({
                'error': error_code,
                'sphero_name': sphero_name,
                'message': str(last_error)
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

                if node.ble_link_down:
                    # The live link is dead. Tear down the dead context manager
                    # cleanly before attempting fresh scan+connect under the lock.
                    print(f"BLE link to {sphero_name} down; starting reconnect "
                          f"({BLE_RECONNECT_ATTEMPTS} attempts).")
                    try:
                        cm.__exit__(None, None, None)
                    except Exception:
                        pass
                    cm = None

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
                                cm = attempt_cm
                            node.rebind_connection(new_robot, new_api)
                            print(f"Reconnected to {sphero_name} "
                                  f"(attempt {attempt}/{BLE_RECONNECT_ATTEMPTS}).")
                            break
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
                            cm = None

                    if node.ble_link_down:
                        # Still down => all reconnect attempts exhausted. Report
                        # the terminal error and break to clean process exit.
                        print(f"✗ Failed to reconnect to {sphero_name} after "
                              f"{BLE_RECONNECT_ATTEMPTS} attempts: {reconnect_error}")
                        node.publish_ble_lost(reconnect_error)
                        break
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
        # Clean up temporary node if it exists
        if temp_node:
            temp_node.destroy_node()

        # Clean up the controller node
        if node:
            node.cleanup()
            node.destroy_node()

        # Shutdown ROS 2
        if rclpy.ok():
            rclpy.shutdown()

        print("Goodbye!")


if __name__ == '__main__':
    main()
