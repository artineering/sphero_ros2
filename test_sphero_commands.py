#!/usr/bin/env python3
"""
Test script to send multiple ROS2 commands to Sphero.

This script demonstrates sending various commands to control a Sphero robot
including LED control, roll commands, stop commands, and more.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import sys


class SpheroCommandTester(Node):
    """Node to test Sphero commands."""

    def __init__(self, sphero_name='SB-1234'):
        super().__init__('sphero_command_tester')
        self.sphero_name = sphero_name
        self.topic_prefix = f'sphero/{sphero_name}'

        # Create publishers for different command topics
        self.led_pub = self.create_publisher(String, f'{self.topic_prefix}/led', 10)
        self.roll_pub = self.create_publisher(String, f'{self.topic_prefix}/roll', 10)
        self.stop_pub = self.create_publisher(String, f'{self.topic_prefix}/stop', 10)
        self.heading_pub = self.create_publisher(String, f'{self.topic_prefix}/heading', 10)
        self.speed_pub = self.create_publisher(String, f'{self.topic_prefix}/speed', 10)
        self.spin_pub = self.create_publisher(String, f'{self.topic_prefix}/spin', 10)
        self.matrix_pub = self.create_publisher(String, f'{self.topic_prefix}/matrix', 10)
        self.raw_motor_pub = self.create_publisher(String, f'{self.topic_prefix}/raw_motor', 10)

        # Wait for publishers to be ready
        time.sleep(0.5)
        self.get_logger().info(f'Sphero Command Tester ready for {sphero_name}')

    def send_led(self, red, green, blue, led_type='main'):
        """Send LED command."""
        msg = String()
        msg.data = json.dumps({
            'red': red,
            'green': green,
            'blue': blue,
            'type': led_type
        })
        self.led_pub.publish(msg)
        self.get_logger().info(f'LED: R={red} G={green} B={blue}')

    def send_roll(self, heading, speed, duration=0):
        """Send roll command."""
        msg = String()
        msg.data = json.dumps({
            'heading': heading,
            'speed': speed,
            'duration': duration
        })
        self.roll_pub.publish(msg)
        self.get_logger().info(f'Roll: heading={heading}° speed={speed} duration={duration}s')

    def send_stop(self):
        """Send stop command."""
        msg = String()
        msg.data = json.dumps({})
        self.stop_pub.publish(msg)
        self.get_logger().info('Stop command sent')

    def send_heading(self, heading):
        """Send heading command."""
        msg = String()
        msg.data = json.dumps({'heading': heading})
        self.heading_pub.publish(msg)
        self.get_logger().info(f'Heading: {heading}°')

    def send_speed(self, speed):
        """Send speed command."""
        msg = String()
        msg.data = json.dumps({'speed': speed})
        self.speed_pub.publish(msg)
        self.get_logger().info(f'Speed: {speed}')

    def send_spin(self, angle, duration=1.0):
        """Send spin command."""
        msg = String()
        msg.data = json.dumps({
            'angle': angle,
            'duration': duration
        })
        self.spin_pub.publish(msg)
        self.get_logger().info(f'Spin: {angle}° over {duration}s')

    def send_matrix(self, pattern='smile', red=255, green=255, blue=255):
        """Send matrix command."""
        msg = String()
        msg.data = json.dumps({
            'pattern': pattern,
            'red': red,
            'green': green,
            'blue': blue
        })
        self.matrix_pub.publish(msg)
        self.get_logger().info(f'Matrix: {pattern} R={red} G={green} B={blue}')

    def send_raw_motor(self, left_mode, left_speed, right_mode, right_speed):
        """Send raw motor command."""
        msg = String()
        msg.data = json.dumps({
            'left_mode': left_mode,
            'left_speed': left_speed,
            'right_mode': right_mode,
            'right_speed': right_speed
        })
        self.raw_motor_pub.publish(msg)
        self.get_logger().info(f'Raw Motor: L={left_mode}:{left_speed} R={right_mode}:{right_speed}')


def test_basic_commands(tester):
    """Test basic LED and motion commands."""
    print("\n=== TEST 1: Basic LED Colors ===")
    tester.send_led(255, 0, 0)  # Red
    time.sleep(1)
    tester.send_led(0, 255, 0)  # Green
    time.sleep(1)
    tester.send_led(0, 0, 255)  # Blue
    time.sleep(1)
    tester.send_led(255, 255, 255)  # White
    time.sleep(1)


def test_roll_commands(tester):
    """Test roll commands in different directions."""
    print("\n=== TEST 2: Roll Commands ===")

    # Roll forward
    tester.send_led(255, 0, 0)  # Red for forward
    tester.send_roll(0, 100, 2.0)
    time.sleep(2.5)

    # Roll right
    tester.send_led(0, 255, 0)  # Green for right
    tester.send_roll(90, 100, 2.0)
    time.sleep(2.5)

    # Roll backward
    tester.send_led(0, 0, 255)  # Blue for backward
    tester.send_roll(180, 100, 2.0)
    time.sleep(2.5)

    # Roll left
    tester.send_led(255, 255, 0)  # Yellow for left
    tester.send_roll(270, 100, 2.0)
    time.sleep(2.5)

    tester.send_stop()


def test_square_pattern(tester):
    """Test square movement pattern."""
    print("\n=== TEST 3: Square Pattern ===")

    for i in range(4):
        heading = i * 90
        tester.send_led(255, 255, 255)
        tester.send_roll(heading, 120, 2.0)
        time.sleep(2.5)

    tester.send_stop()


def test_spin(tester):
    """Test spin command."""
    print("\n=== TEST 4: Spin ===")

    tester.send_led(255, 0, 255)  # Magenta
    tester.send_spin(360, 2.0)  # Full rotation in 2 seconds
    time.sleep(2.5)

    tester.send_spin(720, 3.0)  # Two rotations in 3 seconds
    time.sleep(3.5)


def test_speed_variations(tester):
    """Test different speed levels."""
    print("\n=== TEST 5: Speed Variations ===")

    speeds = [50, 100, 150, 200]

    for speed in speeds:
        tester.get_logger().info(f'Testing speed: {speed}')
        tester.send_roll(0, speed, 2.0)
        time.sleep(2.5)

    tester.send_stop()


def test_raw_motor_circle(tester):
    """Test circular motion with raw motors."""
    print("\n=== TEST 6: Raw Motor Circle ===")

    tester.send_led(0, 255, 255)  # Cyan
    # Left motor slower than right for right turn
    tester.send_raw_motor('forward', 80, 'forward', 120)
    time.sleep(5.0)

    tester.send_stop()


def test_matrix_patterns(tester):
    """Test matrix LED patterns (if available)."""
    print("\n=== TEST 7: Matrix Patterns ===")

    patterns = ['smile', 'heart', 'arrow', 'x']

    for pattern in patterns:
        tester.send_matrix(pattern, 255, 255, 255)
        time.sleep(2.0)


def test_continuous_roll(tester):
    """Test continuous rolling (no duration)."""
    print("\n=== TEST 8: Continuous Roll ===")

    tester.send_led(255, 165, 0)  # Orange
    tester.send_roll(45, 100, 0)  # Continuous roll at 45 degrees
    time.sleep(3.0)
    tester.send_stop()


def run_all_tests(tester):
    """Run all test sequences."""
    print("\n" + "="*50)
    print("STARTING SPHERO COMMAND TESTS")
    print("="*50)

    test_basic_commands(tester)
    time.sleep(1)

    test_roll_commands(tester)
    time.sleep(1)

    test_square_pattern(tester)
    time.sleep(1)

    test_spin(tester)
    time.sleep(1)

    test_speed_variations(tester)
    time.sleep(1)

    test_raw_motor_circle(tester)
    time.sleep(1)

    test_matrix_patterns(tester)
    time.sleep(1)

    test_continuous_roll(tester)

    print("\n" + "="*50)
    print("ALL TESTS COMPLETED")
    print("="*50)


def main():
    """Main entry point."""
    rclpy.init()

    # Get Sphero name from command line or use default
    sphero_name = sys.argv[1] if len(sys.argv) > 1 else 'SB-1234'

    tester = SpheroCommandTester(sphero_name)

    try:
        # Run all tests
        run_all_tests(tester)

    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        tester.send_stop()

    finally:
        tester.send_stop()
        tester.send_led(0, 0, 0)  # Turn off LED
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
