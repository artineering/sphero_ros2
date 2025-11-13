#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Sphero Controller Node for Battleship Game.

This node interfaces with Sphero robots using the spherov2 API and provides
ROS2 topic-based control for various Sphero commands.
"""

import time
import json
import signal

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import BatteryState
from sphero_package.msg import SpheroSensor

from spherov2 import scanner
from spherov2.sphero_edu import SpheroEduAPI, EventType
from spherov2.types import Color
from spherov2.commands.sensor import Sensor, CollisionDetectionMethods

from sphero_package.core.sphero.state import (
    SpheroState,
    SpheroConnectionState
)
from sphero_package.core.sphero.matrix_patterns import get_pattern
from sphero_package.spherov2_collision_patch import apply_collision_patch

# Apply collision detection patch for 16-byte collision responses
apply_collision_patch()


class SpheroControllerNode(Node):
    """
    ROS2 node for controlling Sphero robots.

    Provides topic-based interface for:
    - LED control
    - Movement (roll, spin, heading, speed)
    - Matrix display (for BOLT)
    - Sensor data publishing
    - Stop command
    """

    def __init__(self, robot, api, toy_name: str = "Sphero"):
        """
        Initialize the Sphero controller node.

        Args:
            robot: The Sphero robot object from scanner
            api: The SpheroEduAPI object
            toy_name: Name of the toy for identification
        """
        super().__init__('sphero_controller_node')

        self.robot = robot
        self.api = api
        self.toy_name = toy_name

        # Declare and get ROS parameters
        self.declare_parameter('sensor_rate', 10.0)  # Default: 10 Hz
        self.declare_parameter('heartbeat_rate', 5.0)  # Default: 5 seconds

        self.sensor_rate = self.get_parameter('sensor_rate').value
        self.heartbeat_rate = self.get_parameter('heartbeat_rate').value

        # Calculate sensor timer period (1/frequency)
        self.sensor_period = 1.0 / self.sensor_rate if self.sensor_rate > 0 else 0.1

        # Initialize Sphero state
        self.sphero_state = SpheroState(
            toy_name=toy_name,
            connection_state=SpheroConnectionState.CONNECTED
        )
        # Set API handle and toy so state can query device directly
        self.sphero_state.set_api(api)
        self.sphero_state.set_toy(robot)

        # Internal state (for backward compatibility)
        self._current_heading = 0
        self._current_speed = 0
        self._is_moving = False
        self._collision_mode = None
        self._collision_event_handler = None

        # Create subscribers for Sphero commands using JSON over String
        self.led_sub = self.create_subscription(
            String,
            'sphero/led',
            self.led_callback,
            10
        )

        self.roll_sub = self.create_subscription(
            String,
            'sphero/roll',
            self.roll_callback,
            10
        )

        self.spin_sub = self.create_subscription(
            String,
            'sphero/spin',
            self.spin_callback,
            10
        )

        self.heading_sub = self.create_subscription(
            String,
            'sphero/heading',
            self.heading_callback,
            10
        )

        self.speed_sub = self.create_subscription(
            String,
            'sphero/speed',
            self.speed_callback,
            10
        )

        self.stop_sub = self.create_subscription(
            String,
            'sphero/stop',
            self.stop_callback,
            10
        )

        self.reset_aim_sub = self.create_subscription(
            String,
            'sphero/reset_aim',
            self.reset_aim_callback,
            10
        )

        self.matrix_sub = self.create_subscription(
            String,
            'sphero/matrix',
            self.matrix_callback,
            10
        )

        self.collision_sub = self.create_subscription(
            String,
            'sphero/collision',
            self.collision_callback,
            10
        )

        self.stabilization_sub = self.create_subscription(
            String,
            'sphero/stabilization',
            self.stabilization_callback,
            10
        )

        # Create publishers for sensor data and status
        self.sensor_pub = self.create_publisher(
            SpheroSensor,
            'sphero/sensors',
            10
        )

        self.state_pub = self.create_publisher(
            String,
            'sphero/state',
            10
        )

        self.battery_pub = self.create_publisher(
            BatteryState,
            'sphero/battery',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            'sphero/status',
            10
        )

        self.tap_pub = self.create_publisher(
            String,
            'sphero/tap',
            10
        )

        self.obstacle_pub = self.create_publisher(
            String,
            'sphero/obstacle',
            10
        )

        # Create timer for periodic sensor publishing (configurable rate)
        self.sensor_timer = self.create_timer(self.sensor_period, self.publish_sensors)

        # Create heartbeat timer for battery health (configurable rate)
        self.heartbeat_timer = self.create_timer(self.heartbeat_rate, self.publish_heartbeat)

        self.get_logger().info(f'Sphero Controller Node initialized for {toy_name}')
        self.get_logger().info('Parameters:')
        self.get_logger().info(f'  - sensor_rate: {self.sensor_rate} Hz (period: {self.sensor_period:.3f}s)')
        self.get_logger().info(f'  - heartbeat_rate: {self.heartbeat_rate} seconds')
        self.get_logger().info('Subscribed to:')
        self.get_logger().info('  - sphero/led')
        self.get_logger().info('  - sphero/roll')
        self.get_logger().info('  - sphero/spin')
        self.get_logger().info('  - sphero/heading')
        self.get_logger().info('  - sphero/speed')
        self.get_logger().info('  - sphero/stop')
        self.get_logger().info('  - sphero/reset_aim')
        self.get_logger().info('  - sphero/matrix')
        self.get_logger().info('  - sphero/collision')
        self.get_logger().info('Publishing to:')
        self.get_logger().info(f'  - sphero/sensors (battleship_game/SpheroSensor, {self.sensor_rate} Hz)')
        self.get_logger().info(f'  - sphero/state (complete state JSON, {self.sensor_rate} Hz)')
        self.get_logger().info(f'  - sphero/battery (sensor_msgs/BatteryState, every {self.heartbeat_rate}s)')
        self.get_logger().info(f'  - sphero/status (health heartbeat JSON, every {self.heartbeat_rate}s)')

        # Initial LED indicator (green = ready)
        self.api.set_main_led(Color(r=0, g=255, b=0))

    def led_callback(self, msg: String):
        """
        Handle LED color commands.

        Expected JSON format:
        {
            "red": 255,
            "green": 0,
            "blue": 0,
            "led": "main"  # Optional: "main", "front", "back". Default: "main"
        }

        Available LEDs:
        - "main": Main LED (dome/body)
        - "front": Front LED
        - "back": Back LED
        """
        try:
            data = json.loads(msg.data)
            red = data.get('red', 0)
            green = data.get('green', 0)
            blue = data.get('blue', 0)
            led_type = data.get('led', 'main').lower()

            # Validate RGB values
            red = max(0, min(255, int(red)))
            green = max(0, min(255, int(green)))
            blue = max(0, min(255, int(blue)))

            color = Color(r=red, g=green, b=blue)

            # Set LED based on type
            if led_type == 'main':
                self.api.set_main_led(color)
                self.get_logger().info(f'Main LED set to RGB({red}, {green}, {blue})')
            elif led_type == 'front':
                if hasattr(self.api, 'set_front_led'):
                    self.api.set_front_led(color)
                    self.get_logger().info(f'Front LED set to RGB({red}, {green}, {blue})')
                else:
                    self.get_logger().warning('Front LED not supported on this Sphero model')
            elif led_type == 'back':
                if hasattr(self.api, 'set_back_led'):
                    self.api.set_back_led(color)
                    self.get_logger().info(f'Back LED set to RGB({red}, {green}, {blue})')
                else:
                    self.get_logger().warning('Back LED not supported on this Sphero model')
            else:
                self.get_logger().warning(
                    f'Unknown LED type: {led_type}. Supported types: main, front, back. '
                    f'Defaulting to main LED.'
                )
                self.api.set_main_led(color)
                self.get_logger().info(f'Main LED set to RGB({red}, {green}, {blue})')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in LED command: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error in LED callback: {str(e)}')

    def roll_callback(self, msg: String):
        """
        Handle roll commands.

        Expected JSON format:
        {
            "heading": 90,
            "speed": 128,
            "duration": 2.0
        }
        """
        try:
            data = json.loads(msg.data)
            heading = int(data.get('heading', 0))
            speed = int(data.get('speed', 100))
            duration = float(data.get('duration', 0))

            # Normalize heading to 0-359
            heading = heading % 360

            # Clamp speed to 0-255
            speed = max(0, min(255, speed))

            self._current_heading = heading
            self._current_speed = speed
            self._is_moving = speed > 0

            # Handle duration-based vs indefinite rolling
            if duration > 0:
                # Duration-based: use roll() with specified duration
                self.api.roll(heading, speed, duration)
                self.get_logger().info(
                    f'Rolling at heading {heading}deg with speed {speed} for {duration}s'
                )
            else:
                # Indefinite rolling: set heading and speed separately
                # This keeps the Sphero moving continuously until another command
                self.api.set_heading(heading)
                self.api.set_speed(speed)
                self.get_logger().info(
                    f'Rolling continuously at heading {heading}deg with speed {speed} (indefinite)'
                )

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in roll command: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error in roll callback: {str(e)}')

    def spin_callback(self, msg: String):
        """
        Handle spin commands.

        Expected JSON format:
        {
            "angle": 360,
            "duration": 1.0
        }
        """
        try:
            data = json.loads(msg.data)
            angle = int(data.get('angle', 360))
            duration = float(data.get('duration', 1.0))

            self.get_logger().info(f'Spinning {angle}deg over {duration}s')

            # Spin by rotating heading
            initial_heading = self._current_heading
            target_heading = (initial_heading + angle) % 360

            self.api.spin(angle, duration)
            self._current_heading = target_heading

            self.get_logger().info(f'Spin complete, new heading: {target_heading}deg')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in spin command: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error in spin callback: {str(e)}')

    def heading_callback(self, msg: String):
        """
        Handle heading commands.

        Expected JSON format:
        {
            "heading": 180
        }
        """
        try:
            data = json.loads(msg.data)
            heading = int(data.get('heading', 0))

            # Normalize heading to 0-359
            heading = heading % 360

            self.api.set_heading(heading)
            self._current_heading = heading

            self.get_logger().info(f'Heading set to {heading}deg')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in heading command: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error in heading callback: {str(e)}')

    def speed_callback(self, msg: String):
        """
        Handle speed commands.

        Expected JSON format:
        {
            "speed": 150,
            "duration": 3.0
        }
        """
        try:
            data = json.loads(msg.data)
            speed = int(data.get('speed', 0))
            duration = float(data.get('duration', 0))

            # Clamp speed to 0-255
            speed = max(0, min(255, speed))

            self.api.set_speed(speed)
            self._current_speed = speed
            self._is_moving = speed > 0

            self.get_logger().info(f'Speed set to {speed}')

            # If duration is specified, stop after duration
            if duration > 0:
                time.sleep(duration)
                self.api.set_speed(0)
                self._current_speed = 0
                self._is_moving = False
                self.get_logger().info('Duration complete, stopped')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in speed command: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error in speed callback: {str(e)}')

    def stop_callback(self, msg: String):
        """
        Handle stop commands.

        Expected JSON format:
        {} (empty or any data)
        """
        try:
            self.api.set_speed(0)
            self._current_speed = 0
            self._is_moving = False

            self.get_logger().info('Sphero stopped')

        except Exception as e:
            self.get_logger().error(f'Error in stop callback: {str(e)}')

    def reset_aim_callback(self, msg: String):
        """
        Handle reset aim (reset to origin) commands.

        This resets:
        - The Sphero's heading to 0 degrees (recalibrates orientation)
        - The position coordinates to (0, 0)
        - Position offset in SpheroState (device location will be subtracted by this offset going forward)

        Expected JSON format:
        {} (empty or any data)
        """
        try:
            # Query current location from device BEFORE resetting
            current_location = self.api.get_location()
            device_x = current_location.get('x', 0.0)
            device_y = current_location.get('y', 0.0)

            # Reset the physical heading
            self.api.reset_aim()
            self._current_heading = 0

            # Set position offset in SpheroState - future position will be (device - offset)
            # This makes the current device location become (0, 0) in our coordinate system
            self.sphero_state._position_offset_x = device_x
            self.sphero_state._position_offset_y = device_y

            # Reset the tracked position coordinates to origin
            self.sphero_state.position.x = 0.0
            self.sphero_state.position.y = 0.0

            # Reset position tracking for noise reduction
            self.sphero_state._last_published_position = (0.0, 0.0)

            self.get_logger().info(
                f'Sphero reset to origin: heading=0°, position=(0, 0). '
                f'Device offset: ({device_x:.2f}, {device_y:.2f})'
            )

        except Exception as e:
            self.get_logger().error(f'Error in reset_aim callback: {str(e)}')

    def matrix_callback(self, msg: String):
        """
        Handle LED matrix commands (for Sphero BOLT).

        Expected JSON format:
        {
            "pattern": "smile",  // or "custom"
            "matrix": [0, 1, 0, ...],  // 64 values for custom pattern
            "red": 255,
            "green": 255,
            "blue": 255,
            "duration": 2.0
        }
        """
        try:
            data = json.loads(msg.data)
            pattern = data.get('pattern', '')
            custom_matrix = data.get('matrix', [])
            red = int(data.get('red', 255))
            green = int(data.get('green', 255))
            blue = int(data.get('blue', 255))
            duration = float(data.get('duration', 0))

            # Check if Sphero supports matrix
            if not hasattr(self.api, 'set_matrix_pixel'):
                self.get_logger().warning('This Sphero does not support LED matrix')
                return

            # Use predefined pattern or custom matrix
            matrix_data = get_pattern(pattern)
            if matrix_data is not None:
                # Pattern found
                pass
            elif custom_matrix and len(custom_matrix) == 64:
                matrix_data = custom_matrix
            else:
                self.get_logger().error('Invalid pattern or matrix data')
                return

            # Set matrix pixels
            for i, brightness in enumerate(matrix_data):
                if brightness > 0:
                    row = i // 8
                    col = i % 8
                    self.api.set_matrix_pixel(row, col, Color(r=red, g=green, b=blue))

            self.get_logger().info(f'Matrix pattern "{pattern}" displayed')

            # Clear after duration
            if duration > 0:
                time.sleep(duration)
                self.api.clear_matrix()
                self.get_logger().info('Matrix cleared')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in matrix command: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error in matrix callback: {str(e)}')

    def collision_callback(self, msg: String):
        """
        Handle collision messages using spherov2's built-in collision detection.
        Expected JSON format:
        {
            "action": "start" | "stop",
            "mode": "tap" | "obstacle"
        }
        """
        try:
            data = json.loads(msg.data)
            action = data.get('action', '')

            # Debug: Log received data
            self.get_logger().info(f'[DEBUG] Collision command received: {json.dumps(data)}')

            if action == 'start':
                self._collision_mode = data.get('mode', 'obstacle')

                # Get optional sensitivity parameter (default to HIGH)
                # Map sensitivity levels to threshold values
                # Lower threshold = more sensitive (detects lighter impacts)
                sensitivity_str = data.get('sensitivity', 'HIGH').upper()
                self.get_logger().info(f'[DEBUG] Sensitivity from data: {data.get("sensitivity")}')
                self.get_logger().info(f'[DEBUG] Sensitivity after upper: {sensitivity_str}')

                threshold_map = {
                    'SUPER_HIGH': 50,    # Most sensitive - very light touch
                    'VERY_HIGH': 80,     # Very sensitive
                    'HIGH': 100,         # High sensitivity (default)
                    'MEDIUM': 130,       # Medium sensitivity
                    'LOW': 160,          # Low sensitivity
                    'VERY_LOW': 200      # Least sensitive - hard impact only
                }
                threshold = threshold_map.get(sensitivity_str, 100)
                self.get_logger().info(f'[DEBUG] Mapped to threshold: {threshold}')

                try:
                    # Configure collision detection with threshold
                    self.get_logger().info(f'Configuring collision detection with {sensitivity_str} sensitivity (threshold={threshold})...')
                    self.robot.configure_collision_detection(
                        CollisionDetectionMethods.ACCELEROMETER_BASED_DETECTION,
                        x_threshold=threshold,  # X-axis threshold
                        y_threshold=threshold,  # Y-axis threshold
                        x_speed=100,            # X-axis speed threshold
                        y_speed=100,            # Y-axis speed threshold
                        dead_time=10            # Dead time between detections (in 10ms units)
                    )
                    self.get_logger().info('Collision detection configured')

                    # Enable collision detection notifications
                    self.get_logger().info('Enabling collision notifications...')
                    self.robot.add_collision_detected_notify_listener(self._hardware_collision_callback)
                    self.get_logger().info('Collision notifications enabled')

                    # Store collision event handler reference so hardware callback can trigger it
                    self.get_logger().info('Registering collision event handler...')
                    self._collision_event_handler = self._on_collision_detected
                    self.api.register_event(EventType.on_collision, self._collision_event_handler)
                    self.get_logger().info('Collision event handler registered')

                    self.get_logger().info(
                        f'Started collision detection in "{self._collision_mode}" mode '
                        f'with {sensitivity_str} sensitivity'
                    )
                except Exception as e:
                    self.get_logger().error(f'Error starting collision detection: {e}')
                    import traceback
                    self.get_logger().error(f'Traceback: {traceback.format_exc()}')

            elif action == 'stop':
                # Remove collision detection listener
                try:
                    self.robot.remove_collision_detected_notify_listener(self._hardware_collision_callback)
                except Exception as e:
                    self.get_logger().warning(f'Error removing collision listener: {e}')

                # Unregister collision event handler
                try:
                    self.api.register_event(EventType.on_collision, None)
                    self._collision_event_handler = None
                except Exception as e:
                    self.get_logger().warning(f'Error unregistering collision handler: {e}')

                self._collision_mode = None
                self.get_logger().info('Stopped collision detection')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in collision command: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error in collision callback: {str(e)}')

    def _hardware_collision_callback(self, collision_data):
        """
        Hardware-level callback invoked directly by spherov2 when collision packet arrives.
        This triggers the SpheroEduAPI event system.

        Args:
            collision_data: CollisionDetected object with collision details
        """
        self.get_logger().info('[DEBUG] Hardware collision callback invoked!')
        self.get_logger().info(f'[DEBUG] Collision data: {collision_data}')

        # Trigger the SpheroEduAPI collision event
        # This will call _on_collision_detected with the API object
        try:
            # Manually trigger the event by calling the registered handler
            if hasattr(self, '_collision_event_handler') and self._collision_event_handler:
                self._collision_event_handler(self.api)
        except Exception as e:
            self.get_logger().error(f'Error triggering collision event: {e}')

    def _on_collision_detected(self, api):
        """
        Callback invoked by spherov2 when a collision is detected.

        Args:
            api: The SpheroEduAPI object (required signature for register_event)
        """
        self.get_logger().info('[DEBUG] Collision callback invoked!')

        if self._collision_mode is None:
            self.get_logger().warning('[DEBUG] Collision mode is None, ignoring')
            return

        self.get_logger().info(f'[DEBUG] Processing collision in mode: {self._collision_mode}')

        try:
            # Query current motion state to determine if we're moving
            is_moving = self.sphero_state.query_property('motion', 'is_moving')
            if is_moving is None:
                is_moving = self._is_moving  # Fallback to internal state

            # Tap mode: Detect impacts when stationary
            if self._collision_mode == 'tap' and not is_moving:
                self.get_logger().info('Tap detected!')

                # Flash LED red for visual feedback
                self._flash_red_led()

                tap_msg = String()
                tap_msg.data = json.dumps({
                    'type': 'tap',
                    'timestamp': self.get_clock().now().to_msg().sec
                })
                self.tap_pub.publish(tap_msg)

            # Obstacle mode: Detect collisions when in motion (or always if moving state unknown)
            elif self._collision_mode == 'obstacle':
                self.get_logger().info('Obstacle collision detected!')

                # Flash LED red for visual feedback
                self._flash_red_led()

                obstacle_msg = String()
                obstacle_msg.data = json.dumps({
                    'type': 'obstacle',
                    'timestamp': self.get_clock().now().to_msg().sec
                })
                self.obstacle_pub.publish(obstacle_msg)

        except Exception as e:
            self.get_logger().error(f'Error in collision handler: {str(e)}')

    def _flash_red_led(self):
        """
        Flash the main LED red for 1.5 seconds to indicate collision detection.
        The LED will then return to green.
        """
        try:
            # Set LED to red
            self.api.set_main_led(Color(r=255, g=0, b=0))

            # Wait for 1.5 seconds
            time.sleep(1.5)

            # Return LED to green (ready state)
            self.api.set_main_led(Color(r=0, g=255, b=0))

        except Exception as e:
            self.get_logger().error(f'Error flashing LED: {str(e)}')

    def stabilization_callback(self, msg: String):
        """
        Handle stabilization enable/disable commands.
        Expected JSON format:
        {
            "enable": true | false
        }
        """
        try:
            data = json.loads(msg.data)
            enable = data.get('enable', True)

            # Set stabilization on the Sphero
            self.api.set_stabilization(enable)

            status = "enabled" if enable else "disabled"
            self.get_logger().info(f'Stabilization {status}')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in stabilization command: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error in stabilization callback: {str(e)}')

    def publish_sensors(self):
        """
        Publish sensor data periodically by querying the Sphero device.

        This method delegates sensor querying to the SpheroState object,
        which manages all device communication internally.

        Published topics:
        - sphero/state: Complete state as JSON (includes all sensor data)
        - sphero/sensors: battleship_game/SpheroSensor message format

        Note: Battery state (sensor_msgs/BatteryState) is published separately
              in publish_heartbeat() to avoid excessive publishing.

        Rate controlled by ROS parameter 'sensor_rate' (Hz).
        """
        try:
            # Update state from device (handles all sensor queries internally)
            self.sphero_state.update_from_device()

            # Update backward compatibility variables
            self._current_heading = self.sphero_state.get('motion', 'heading', 0)
            self._current_speed = self.sphero_state.get('motion', 'speed', 0)
            self._is_moving = self.sphero_state.get('motion', 'is_moving', False)

            # Publish complete state as JSON
            state_msg = String()
            state_msg.data = json.dumps(self.sphero_state.to_dict())
            self.state_pub.publish(state_msg)

            # Publish sensor data using SpheroSensor message format
            sensor_msg = self.sphero_state.to_sphero_sensor_msg()
            if sensor_msg is not None:
                sensor_msg.timestamp = self.get_clock().now().to_msg()
                self.sensor_pub.publish(sensor_msg)

        except Exception as e:
            self.get_logger().error(f'Error publishing sensors: {str(e)}', throttle_duration_sec=5.0)

    def publish_heartbeat(self):
        """
        Publish heartbeat with battery health information at configured rate.

        This provides a periodic health check of the Sphero, including:
        - Battery percentage and health status (sensor_msgs/BatteryState)
        - Connection state
        - Overall system health
        - Uptime information

        Published topics:
        - sphero/status: JSON heartbeat message with health summary
        - sphero/battery: Standard sensor_msgs/BatteryState message

        Note: This is the ONLY place where sensor_msgs/BatteryState is published
              to avoid duplication with high-frequency sensor updates.
        """
        try:
            # Create heartbeat message with battery health
            heartbeat_data = {
                'toy_name': self.toy_name,
                'timestamp': self.get_clock().now().to_msg().sec,
                'connection_state': self.sphero_state.connection_state.value,
                'battery': {
                    'percentage': self.sphero_state.battery.percentage,
                    'voltage': self.sphero_state.battery.voltage,
                    'health': 'GOOD' if self.sphero_state.battery.percentage > 10 else 'DEAD',
                    'status': 'FULL' if self.sphero_state.battery.percentage >= 100 else 'DISCHARGING'
                },
                'is_healthy': self.sphero_state.is_healthy(),
                'uptime': time.time() - self.sphero_state.timestamp
            }

            # Publish heartbeat as JSON string
            heartbeat_msg = String()
            heartbeat_msg.data = json.dumps(heartbeat_data)
            self.status_pub.publish(heartbeat_msg)

            # Also publish dedicated battery state message
            battery_msg = self.sphero_state.battery.to_battery_state_msg()
            if battery_msg is not None:
                battery_msg.header.stamp = self.get_clock().now().to_msg()
                battery_msg.header.frame_id = f'sphero_{self.toy_name}'
                self.battery_pub.publish(battery_msg)

            # Log warning if battery is low
            if self.sphero_state.battery.percentage < 20:
                self.get_logger().warning(
                    f'Low battery: {self.sphero_state.battery.percentage}%',
                    throttle_duration_sec=30.0
                )
            elif self.sphero_state.battery.percentage < 10:
                self.get_logger().error(
                    f'Critical battery: {self.sphero_state.battery.percentage}%',
                    throttle_duration_sec=30.0
                )

            # Log heartbeat at info level (throttled to avoid spam)
            self.get_logger().info(
                f'Heartbeat: {self.toy_name} - Battery: {self.sphero_state.battery.percentage}% - '
                f'Connection: {self.sphero_state.connection_state.value} - '
                f'Healthy: {self.sphero_state.is_healthy()}',
                throttle_duration_sec=30.0
            )

        except Exception as e:
            self.get_logger().error(f'Error publishing heartbeat: {str(e)}')

    def cleanup(self):
        """Clean up resources before shutdown."""
        self.get_logger().info('Cleaning up Sphero controller...')
        try:
            # Disable collision detection
            if self._collision_mode is not None:
                try:
                    Sensor.enable_collision_detected_notify(self.robot, False)
                    self.api.register_event(EventType.on_collision, None)
                except Exception as e:
                    self.get_logger().warning(f'Error disabling collision detection: {e}')

            # Stop movement
            self.api.set_speed(0)

            # Turn off LED
            self.api.set_main_led(Color(r=0, g=0, b=0))

            # Clear matrix if supported
            if hasattr(self.api, 'clear_matrix'):
                self.api.clear_matrix()

            self.get_logger().info('Sphero cleanup complete')

        except Exception as e:
            self.get_logger().error(f'Error during cleanup: {e}')


def main(args=None):
    """Main entry point for the Sphero controller node."""
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
        # Create a temporary node to read the toy_name parameter
        temp_node = rclpy.create_node('temp_param_node')
        temp_node.declare_parameter('toy_name', 'SB-3660')  # Default value
        toy_name = temp_node.get_parameter('toy_name').value

        print(f"Toy name from parameter: {toy_name}")

        # Destroy temporary node before creating controller node
        temp_node.destroy_node()
        temp_node = None

        # Scan for Sphero
        print(f"Scanning for Sphero robot: {toy_name}...")
        robot = scanner.find_toy(toy_name=toy_name)

        with SpheroEduAPI(toy=robot) as api:
            print(f"Connected to {toy_name}")

            # Create the node
            node = SpheroControllerNode(robot, api, toy_name)

            # Spin until shutdown requested
            while rclpy.ok() and not shutdown_requested:
                rclpy.spin_once(node, timeout_sec=0.1)

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
