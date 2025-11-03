#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Task Executor Node for State Machine.

This node subscribes to state machine task commands and executes them
by publishing to the appropriate Sphero control topics.
"""

import json
import time
from typing import Dict, Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TaskExecutorNode(Node):
    """
    Executes state machine tasks by translating them to Sphero commands.

    Subscribes to:
    - /state_machine/task_command: Task commands from state machine

    Publishes to:
    - /sphero/led: LED color commands
    - /sphero/roll: Roll/movement commands
    - /sphero/heading: Heading commands
    - /sphero/speed: Speed commands
    - /sphero/stop: Stop commands
    - /sphero/matrix: LED matrix commands
    """

    def __init__(self):
        """Initialize the task executor node."""
        super().__init__('task_executor')

        # Subscribe to state machine task commands
        self.task_sub = self.create_subscription(
            String,
            '/state_machine/task_command',
            self.task_callback,
            10
        )

        # Create publishers for Sphero commands
        self.led_pub = self.create_publisher(String, '/sphero/led', 10)
        self.roll_pub = self.create_publisher(String, '/sphero/roll', 10)
        self.heading_pub = self.create_publisher(String, '/sphero/heading', 10)
        self.speed_pub = self.create_publisher(String, '/sphero/speed', 10)
        self.stop_pub = self.create_publisher(String, '/sphero/stop', 10)
        self.matrix_pub = self.create_publisher(String, '/sphero/matrix', 10)

        # Publisher for task execution feedback
        self.feedback_pub = self.create_publisher(
            String,
            '/state_machine/task_feedback',
            10
        )

        self.get_logger().info('Task Executor Node initialized')
        self.get_logger().info('Ready to execute state machine tasks')

    def task_callback(self, msg: String):
        """
        Handle incoming task commands from state machine.

        Expected JSON format:
        {
            "state": "state_name",
            "task_type": "task_type",
            "params": {...},
            "timestamp": 123.456
        }
        """
        try:
            task = json.loads(msg.data)
            task_type = task.get('task_type', 'none')
            params = task.get('params', {})
            state = task.get('state', 'unknown')

            self.get_logger().info(f'Executing task from state "{state}": {task_type}')

            # Execute the task based on type
            success = self.execute_task(task_type, params)

            # Publish feedback
            self.publish_feedback(state, task_type, success)

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse task JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Error executing task: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def execute_task(self, task_type: str, params: Dict[str, Any]) -> bool:
        """
        Execute a task based on its type.

        Args:
            task_type: Type of task to execute
            params: Task parameters

        Returns:
            True if task executed successfully, False otherwise
        """
        try:
            if task_type == 'none':
                # No-op task
                self.get_logger().debug('No-op task')
                return True

            elif task_type == 'set_led':
                return self.execute_set_led(params)

            elif task_type == 'roll':
                return self.execute_roll(params)

            elif task_type == 'heading':
                return self.execute_heading(params)

            elif task_type == 'speed':
                return self.execute_speed(params)

            elif task_type == 'stop':
                return self.execute_stop(params)

            elif task_type == 'matrix':
                return self.execute_matrix(params)

            else:
                self.get_logger().warning(f'Unknown task type: {task_type}')
                return False

        except Exception as e:
            self.get_logger().error(f'Error executing task {task_type}: {e}')
            return False

    def execute_set_led(self, params: Dict[str, Any]) -> bool:
        """Execute LED color change task."""
        color = params.get('color', 'white').lower()

        # Color name to RGB mapping
        color_map = {
            'red': (255, 0, 0),
            'green': (0, 255, 0),
            'blue': (0, 0, 255),
            'yellow': (255, 255, 0),
            'cyan': (0, 255, 255),
            'magenta': (255, 0, 255),
            'white': (255, 255, 255),
            'orange': (255, 165, 0),
            'purple': (128, 0, 128),
            'pink': (255, 192, 203),
            'off': (0, 0, 0)
        }

        # Check if direct RGB values provided
        if 'red' in params and 'green' in params and 'blue' in params:
            red = params['red']
            green = params['green']
            blue = params['blue']
        elif color in color_map:
            red, green, blue = color_map[color]
        else:
            self.get_logger().warning(f'Unknown color: {color}, defaulting to white')
            red, green, blue = 255, 255, 255

        msg = String()
        msg.data = json.dumps({'red': red, 'green': green, 'blue': blue})
        self.led_pub.publish(msg)

        self.get_logger().info(f'LED: RGB({red}, {green}, {blue})')
        return True

    def execute_roll(self, params: Dict[str, Any]) -> bool:
        """Execute roll/movement task."""
        heading = params.get('heading', 0)
        speed = params.get('speed', 100)
        duration = params.get('duration', 0.0)

        msg = String()
        msg.data = json.dumps({
            'heading': heading,
            'speed': speed,
            'duration': duration
        })
        self.roll_pub.publish(msg)

        self.get_logger().info(f'Roll: heading={heading}, speed={speed}, duration={duration}')
        return True

    def execute_heading(self, params: Dict[str, Any]) -> bool:
        """Execute heading change task."""
        heading = params.get('heading', 0)

        msg = String()
        msg.data = json.dumps({'heading': heading})
        self.heading_pub.publish(msg)

        self.get_logger().info(f'Heading: {heading}')
        return True

    def execute_speed(self, params: Dict[str, Any]) -> bool:
        """Execute speed change task."""
        speed = params.get('speed', 0)
        duration = params.get('duration', 0.0)

        msg = String()
        msg.data = json.dumps({'speed': speed, 'duration': duration})
        self.speed_pub.publish(msg)

        self.get_logger().info(f'Speed: {speed}, duration={duration}')
        return True

    def execute_stop(self, params: Dict[str, Any]) -> bool:
        """Execute stop task."""
        msg = String()
        msg.data = json.dumps({})
        self.stop_pub.publish(msg)

        self.get_logger().info('Stop command sent')
        return True

    def execute_matrix(self, params: Dict[str, Any]) -> bool:
        """Execute LED matrix display task."""
        pattern = params.get('pattern', 'smile')
        red = params.get('red', 255)
        green = params.get('green', 255)
        blue = params.get('blue', 255)
        duration = params.get('duration', 0)

        msg = String()
        msg.data = json.dumps({
            'pattern': pattern,
            'red': red,
            'green': green,
            'blue': blue,
            'duration': duration
        })
        self.matrix_pub.publish(msg)

        self.get_logger().info(f'Matrix: pattern={pattern}, RGB({red}, {green}, {blue})')
        return True

    def publish_feedback(self, state: str, task_type: str, success: bool):
        """Publish task execution feedback."""
        feedback = {
            'state': state,
            'task_type': task_type,
            'success': success,
            'timestamp': time.time()
        }

        msg = String()
        msg.data = json.dumps(feedback)
        self.feedback_pub.publish(msg)


def main(args=None):
    """Main entry point for the task executor node."""
    rclpy.init(args=args)

    node = TaskExecutorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down task executor...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
