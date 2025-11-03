#!/usr/bin/env python3
"""
Example script to send a state machine configuration.

Usage:
    python3 send_config.py <config_file.json>
"""

import sys
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 send_config.py <config_file.json>")
        sys.exit(1)

    config_file = sys.argv[1]

    # Load configuration
    try:
        with open(config_file, 'r') as f:
            config = json.load(f)
    except FileNotFoundError:
        print(f"Error: File '{config_file}' not found")
        sys.exit(1)
    except json.JSONDecodeError as e:
        print(f"Error: Invalid JSON in '{config_file}': {e}")
        sys.exit(1)

    # Initialize ROS2
    rclpy.init()
    node = Node('state_machine_config_sender')

    # Create publisher
    pub = node.create_publisher(String, '/state_machine/config', 10)

    # Wait for subscriber
    print("Waiting for state machine controller to be ready...")
    import time
    time.sleep(2)

    # Send configuration
    msg = String()
    msg.data = json.dumps(config)
    pub.publish(msg)

    print(f"Configuration sent: {config.get('name', 'unnamed')}")
    print(f"  - States: {len(config.get('states', []))}")
    print(f"  - Transitions: {len(config.get('transitions', []))}")
    print(f"  - Initial state: {config.get('initial_state', 'unknown')}")

    # Cleanup
    time.sleep(1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
