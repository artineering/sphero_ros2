#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Example task sender for Sphero Task Controller.

This script demonstrates how to send tasks programmatically to the task controller.

Created with assistance from Claude Code (Anthropic)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import sys


class TaskSender(Node):
    """Simple task sender node."""

    def __init__(self):
        super().__init__('task_sender')
        self.task_pub = self.create_publisher(String, 'sphero/task', 10)
        self.status_sub = self.create_subscription(
            String,
            'sphero/task/status',
            self.status_callback,
            10
        )
        # Wait for publisher to be ready
        time.sleep(0.5)

    def send_task(self, task_type: str, parameters: dict, task_id: str = None):
        """Send a task to the task controller."""
        task = {
            'task_type': task_type,
            'parameters': parameters
        }
        if task_id:
            task['task_id'] = task_id

        msg = String()
        msg.data = json.dumps(task)
        self.task_pub.publish(msg)
        self.get_logger().info(f'Sent task: {task_type} (ID: {task_id or "auto"})')

    def status_callback(self, msg: String):
        """Print task status updates."""
        try:
            status = json.loads(msg.data)
            self.get_logger().info(
                f"Task {status['task_id']}: {status['status']} "
                f"({status['task_type']})"
            )
        except:
            pass


def print_menu():
    """Print the task menu."""
    print("\n" + "="*50)
    print("Sphero Task Sender - Example Tasks")
    print("="*50)
    print("1. Move to (100, 100)")
    print("2. Square patrol")
    print("3. Circle (10 seconds)")
    print("4. Rainbow LED sequence (looping)")
    print("5. Matrix emoji sequence")
    print("6. Spin twice")
    print("7. Custom demo sequence")
    print("8. Send all tasks in sequence")
    print("9. Stop (immediate)")
    print("0. Exit")
    print("="*50)


def main(args=None):
    rclpy.init(args=args)
    sender = TaskSender()

    print("\n\033[1;32mSphero Task Sender Ready!\033[0m")
    print("You can send multiple tasks consecutively.")
    print("Tasks will be queued and executed in order.")

    # Start spinning in background thread
    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(sender,), daemon=True)
    spin_thread.start()

    try:
        while True:
            print_menu()
            choice = input("\nEnter choice (0-9): ").strip()

            if choice == '1':
                sender.send_task('move_to', {'x': 100, 'y': 100, 'speed': 100})

            elif choice == '2':
                sender.send_task('patrol', {
                    'waypoints': [
                        {'x': 0, 'y': 0},
                        {'x': 100, 'y': 0},
                        {'x': 100, 'y': 100},
                        {'x': 0, 'y': 100}
                    ],
                    'speed': 80,
                    'loop': True
                })

            elif choice == '3':
                sender.send_task('circle', {
                    'radius': 50,
                    'speed': 100,
                    'duration': 10.0
                })

            elif choice == '4':
                sender.send_task('led_sequence', {
                    'sequence': [
                        {'red': 255, 'green': 0, 'blue': 0},
                        {'red': 255, 'green': 127, 'blue': 0},
                        {'red': 255, 'green': 255, 'blue': 0},
                        {'red': 0, 'green': 255, 'blue': 0},
                        {'red': 0, 'green': 0, 'blue': 255},
                        {'red': 75, 'green': 0, 'blue': 130},
                        {'red': 148, 'green': 0, 'blue': 211}
                    ],
                    'interval': 0.5,
                    'loop': True
                })

            elif choice == '5':
                sender.send_task('matrix_sequence', {
                    'sequence': [
                        {'pattern': 'smile', 'red': 255, 'green': 255, 'blue': 0},
                        {'pattern': 'heart', 'red': 255, 'green': 0, 'blue': 0},
                        {'pattern': 'star', 'red': 0, 'green': 255, 'blue': 255},
                        {'pattern': 'checkmark', 'red': 0, 'green': 255, 'blue': 0}
                    ],
                    'interval': 2.0,
                    'loop': False
                })

            elif choice == '6':
                sender.send_task('spin', {'rotations': 2, 'speed': 100})

            elif choice == '7':
                sender.send_task('custom', {
                    'commands': [
                        {'type': 'led', 'red': 255, 'green': 0, 'blue': 0, 'duration': 1.0},
                        {'type': 'roll', 'heading': 0, 'speed': 100, 'duration': 2.0},
                        {'type': 'led', 'red': 0, 'green': 255, 'blue': 0, 'duration': 1.0},
                        {'type': 'roll', 'heading': 90, 'speed': 100, 'duration': 2.0},
                        {'type': 'matrix', 'pattern': 'smile', 'red': 255, 'green': 255, 'blue': 0, 'duration': 2.0},
                        {'type': 'stop', 'duration': 0.5}
                    ]
                })

            elif choice == '8':
                print("\nSending multiple tasks in sequence...")
                sender.send_task('led_sequence', {
                    'sequence': [{'red': 255, 'green': 0, 'blue': 0}],
                    'interval': 0.5
                }, task_id='task_1')
                time.sleep(0.1)

                sender.send_task('circle', {'duration': 5.0}, task_id='task_2')
                time.sleep(0.1)

                sender.send_task('square', {'side_length': 100}, task_id='task_3')
                time.sleep(0.1)

                sender.send_task('stop', {}, task_id='task_4')
                print("All tasks queued!")

            elif choice == '9':
                sender.send_task('stop', {'delay': 0.0})
                print("\033[1;31mSTOP command sent! This will cancel the current task.\033[0m")

            elif choice == '0':
                print("\n\033[1;33mExiting...\033[0m")
                break

            else:
                print("\033[1;31mInvalid choice! Please enter 0-9.\033[0m")

            # Small delay to allow status message to be received
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\033[1;33mShutting down...\033[0m")
    finally:
        sender.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
