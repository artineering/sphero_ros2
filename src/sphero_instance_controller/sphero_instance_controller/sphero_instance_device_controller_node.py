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

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import BatteryState
from sphero_instance_controller.msg import SpheroSensor

from spherov2 import scanner
from spherov2.sphero_edu import SpheroEduAPI

from sphero_instance_controller.core.sphero import Sphero
from sphero_instance_controller.spherov2_collision_patch import apply_collision_patch

# Apply collision detection patch for 16-byte collision responses
apply_collision_patch()


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
        super().__init__('sphero_instance_device_controller_node')

        self.sphero_name = sphero_name
        self.topic_prefix = f'sphero/{sphero_name}'

        # Initialize Sphero core class
        self.sphero = Sphero(robot, api, sphero_name)

        # Declare and get ROS parameters
        self.declare_parameter('sensor_rate', 10.0)  # Default: 10 Hz
        self.declare_parameter('heartbeat_rate', 5.0)  # Default: 5 seconds

        self.sensor_rate = self.get_parameter('sensor_rate').value
        self.heartbeat_rate = self.get_parameter('heartbeat_rate').value

        # Calculate sensor timer period (1/frequency)
        self.sensor_period = 1.0 / self.sensor_rate if self.sensor_rate > 0 else 0.1

        # Create subscribers for Sphero commands
        self._create_subscribers()

        # Create publishers for sensor data and status
        self._create_publishers()

        # Create timers
        self.sensor_timer = self.create_timer(self.sensor_period, self.publish_sensors)
        self.heartbeat_timer = self.create_timer(self.heartbeat_rate, self.publish_heartbeat)

        # Log initialization info
        self._log_initialization()

        # Initial LED indicator (green = ready)
        self.sphero.set_led(0, 255, 0)

    def _create_subscribers(self):
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

        self.stop_sub = self.create_subscription(
            String, f'{self.topic_prefix}/stop', self.stop_callback, 10)

        self.reset_aim_sub = self.create_subscription(
            String, f'{self.topic_prefix}/reset_aim', self.reset_aim_callback, 10)

        self.matrix_sub = self.create_subscription(
            String, f'{self.topic_prefix}/matrix', self.matrix_callback, 10)

        self.collision_sub = self.create_subscription(
            String, f'{self.topic_prefix}/collision', self.collision_callback, 10)

        self.stabilization_sub = self.create_subscription(
            String, f'{self.topic_prefix}/stabilization', self.stabilization_callback, 10)

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
        self.get_logger().info(f'Sphero Instance Device Controller initialized for {self.sphero_name}')
        self.get_logger().info('Parameters:')
        self.get_logger().info(f'  - sphero_name: {self.sphero_name}')
        self.get_logger().info(f'  - topic_prefix: {self.topic_prefix}')
        self.get_logger().info(f'  - sensor_rate: {self.sensor_rate} Hz (period: {self.sensor_period:.3f}s)')
        self.get_logger().info(f'  - heartbeat_rate: {self.heartbeat_rate} seconds')

    # ===== Command Callbacks =====

    def led_callback(self, msg: String):
        """Handle LED color commands."""
        try:
            data = json.loads(msg.data)
            red = data.get('red', 0)
            green = data.get('green', 0)
            blue = data.get('blue', 0)
            led_type = data.get('led', 'main').lower()

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

        # Destroy temporary node before creating controller node
        temp_node.destroy_node()
        temp_node = None

        # Scan for Sphero
        print(f"Scanning for Sphero robot: {sphero_name}...")
        robot = scanner.find_toy(toy_name=sphero_name)

        with SpheroEduAPI(toy=robot) as api:
            print(f"Connected to {sphero_name}")

            # Create the node
            node = SpheroInstanceDeviceController(robot, api, sphero_name)

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
