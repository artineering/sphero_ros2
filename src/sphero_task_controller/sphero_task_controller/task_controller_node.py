#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Sphero Task Controller Node.

This node accepts high-level tasks from a dedicated topic and executes them
by publishing appropriate commands to the sphero_controller_node.

Task Types Supported:
- move_to: Move to a specific location
- patrol: Patrol between waypoints
- circle: Move in a circle
- square: Move in a square pattern
- led_sequence: Execute LED color sequence
- matrix_sequence: Display matrix patterns in sequence
- spin: Spin in place
- custom: Custom command sequence

Author: Siddharth Vaghela
Created with assistance from Claude Code (Anthropic)
"""

import json
import time
import math
from enum import Enum
from typing import Dict, List, Any, Optional
from dataclasses import dataclass, field

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String


class TaskStatus(Enum):
    """Task execution status."""
    PENDING = "pending"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


class TaskType(Enum):
    """Supported task types."""
    MOVE_TO = "move_to"
    PATROL = "patrol"
    CIRCLE = "circle"
    SQUARE = "square"
    LED_SEQUENCE = "led_sequence"
    MATRIX_SEQUENCE = "matrix_sequence"
    SPIN = "spin"
    STOP = "stop"
    CUSTOM = "custom"


@dataclass
class Task:
    """Represents a task to be executed."""
    task_id: str
    task_type: str
    parameters: Dict[str, Any]
    status: TaskStatus = TaskStatus.PENDING
    created_at: float = field(default_factory=time.time)
    started_at: Optional[float] = None
    completed_at: Optional[float] = None
    error_message: Optional[str] = None

    def to_dict(self) -> dict:
        """Convert task to dictionary."""
        return {
            'task_id': self.task_id,
            'task_type': self.task_type,
            'parameters': self.parameters,
            'status': self.status.value,
            'created_at': self.created_at,
            'started_at': self.started_at,
            'completed_at': self.completed_at,
            'error_message': self.error_message
        }


class SpheroTaskController(Node):
    """
    ROS2 node for high-level task control of Sphero robots.

    Subscribes to:
        - /sphero/task (std_msgs/String): Task commands in JSON format
        - /sphero/state (std_msgs/String): Sphero state feedback

    Publishes to:
        - /sphero/led (std_msgs/String): LED commands
        - /sphero/roll (std_msgs/String): Roll commands
        - /sphero/heading (std_msgs/String): Heading commands
        - /sphero/speed (std_msgs/String): Speed commands
        - /sphero/stop (std_msgs/String): Stop commands
        - /sphero/matrix (std_msgs/String): Matrix commands
        - /sphero/task/status (std_msgs/String): Task status updates
    """

    def __init__(self):
        """Initialize the task controller node."""
        super().__init__('sphero_task_controller')

        # Callback group for reentrant callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Task queue and current task
        self.task_queue: List[Task] = []
        self.current_task: Optional[Task] = None
        self.task_history: List[Task] = []

        # Sphero state
        self.current_state: Dict[str, Any] = {}
        self.current_position = {'x': 0.0, 'y': 0.0}
        self.current_heading = 0

        # Publishers for Sphero commands
        self.led_pub = self.create_publisher(String, 'sphero/led', 10)
        self.roll_pub = self.create_publisher(String, 'sphero/roll', 10)
        self.heading_pub = self.create_publisher(String, 'sphero/heading', 10)
        self.speed_pub = self.create_publisher(String, 'sphero/speed', 10)
        self.stop_pub = self.create_publisher(String, 'sphero/stop', 10)
        self.matrix_pub = self.create_publisher(String, 'sphero/matrix', 10)

        # Publisher for task status
        self.task_status_pub = self.create_publisher(String, 'sphero/task/status', 10)

        # Subscribers
        self.task_sub = self.create_subscription(
            String,
            'sphero/task',
            self.task_callback,
            10,
            callback_group=self.callback_group
        )

        self.state_sub = self.create_subscription(
            String,
            'sphero/state',
            self.state_callback,
            10,
            callback_group=self.callback_group
        )

        # Timer for task execution
        self.task_timer = self.create_timer(
            0.1,  # 10 Hz
            self.task_execution_loop,
            callback_group=self.callback_group
        )

        # Configuration
        self.default_speed = 100
        self.position_tolerance = 10.0  # cm
        self.heading_tolerance = 5  # degrees

        self.get_logger().info('Sphero Task Controller initialized')
        self.get_logger().info('Listening for tasks on /sphero/task')

    def task_callback(self, msg: String):
        """
        Handle incoming task messages.

        Expected JSON format:
        {
            "task_id": "unique_id",
            "task_type": "move_to|patrol|circle|...",
            "parameters": {
                // Task-specific parameters
            }
        }
        """
        try:
            task_data = json.loads(msg.data)

            # Validate required fields
            if 'task_type' not in task_data:
                self.get_logger().error('Task missing required field: task_type')
                return

            # Generate task ID if not provided
            if 'task_id' not in task_data:
                task_data['task_id'] = f"task_{int(time.time() * 1000)}"

            # Create task object
            task = Task(
                task_id=task_data['task_id'],
                task_type=task_data['task_type'],
                parameters=task_data.get('parameters', {})
            )

            # Add to queue
            self.task_queue.append(task)
            self.get_logger().info(
                f'Added task {task.task_id} ({task.task_type}) to queue. '
                f'Queue length: {len(self.task_queue)}'
            )

            # Publish status
            self.publish_task_status(task)

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in task message: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing task: {e}')

    def state_callback(self, msg: String):
        """Handle Sphero state updates."""
        try:
            self.current_state = json.loads(msg.data)

            # Update position
            if 'position' in self.current_state:
                self.current_position = self.current_state['position']

            # Update heading
            if 'motion' in self.current_state and 'heading' in self.current_state['motion']:
                self.current_heading = self.current_state['motion']['heading']

        except json.JSONDecodeError:
            pass

    def task_execution_loop(self):
        """Main task execution loop."""
        # Check if next task in queue is a STOP task that should cancel current task
        if self.current_task is not None and len(self.task_queue) > 0:
            next_task = self.task_queue[0]
            if next_task.task_type.lower() == TaskType.STOP.value:
                # Check if STOP has delay parameter
                delay = next_task.parameters.get('delay', 0.0)
                if delay == 0.0:
                    # Immediate stop - cancel current task
                    self.get_logger().info(f'Cancelling task {self.current_task.task_id} due to STOP command')
                    self.current_task.status = TaskStatus.CANCELLED
                    self.current_task.completed_at = time.time()
                    self.publish_task_status(self.current_task)
                    self.task_history.append(self.current_task)
                    self.current_task = None

        # Start next task if no task is currently running
        if self.current_task is None and len(self.task_queue) > 0:
            self.current_task = self.task_queue.pop(0)
            self.current_task.status = TaskStatus.RUNNING
            self.current_task.started_at = time.time()
            self.get_logger().info(f'Starting task {self.current_task.task_id}')
            self.publish_task_status(self.current_task)

        # Execute current task
        if self.current_task is not None:
            try:
                completed = self.execute_task(self.current_task)

                if completed:
                    self.current_task.status = TaskStatus.COMPLETED
                    self.current_task.completed_at = time.time()
                    duration = self.current_task.completed_at - self.current_task.started_at
                    self.get_logger().info(
                        f'Task {self.current_task.task_id} completed in {duration:.2f}s'
                    )
                    self.publish_task_status(self.current_task)
                    self.task_history.append(self.current_task)
                    self.current_task = None

            except Exception as e:
                self.get_logger().error(f'Task {self.current_task.task_id} failed: {e}')
                self.current_task.status = TaskStatus.FAILED
                self.current_task.error_message = str(e)
                self.current_task.completed_at = time.time()
                self.publish_task_status(self.current_task)
                self.task_history.append(self.current_task)
                self.current_task = None

    def execute_task(self, task: Task) -> bool:
        """
        Execute a task based on its type.

        Returns:
            bool: True if task is completed, False if still running
        """
        task_type = task.task_type.lower()

        if task_type == TaskType.MOVE_TO.value:
            return self.execute_move_to(task)
        elif task_type == TaskType.PATROL.value:
            return self.execute_patrol(task)
        elif task_type == TaskType.CIRCLE.value:
            return self.execute_circle(task)
        elif task_type == TaskType.SQUARE.value:
            return self.execute_square(task)
        elif task_type == TaskType.LED_SEQUENCE.value:
            return self.execute_led_sequence(task)
        elif task_type == TaskType.MATRIX_SEQUENCE.value:
            return self.execute_matrix_sequence(task)
        elif task_type == TaskType.SPIN.value:
            return self.execute_spin(task)
        elif task_type == TaskType.STOP.value:
            return self.execute_stop(task)
        elif task_type == TaskType.CUSTOM.value:
            return self.execute_custom(task)
        else:
            raise ValueError(f'Unknown task type: {task_type}')

    def execute_move_to(self, task: Task) -> bool:
        """Move to a specific position."""
        target_x = task.parameters.get('x', 0)
        target_y = task.parameters.get('y', 0)
        speed = task.parameters.get('speed', self.default_speed)

        # Calculate distance and heading
        dx = target_x - self.current_position['x']
        dy = target_y - self.current_position['y']
        distance = math.sqrt(dx**2 + dy**2)

        # Check if we've reached the target
        if distance < self.position_tolerance:
            self.send_stop()
            return True

        # Calculate heading to target
        target_heading = int(math.degrees(math.atan2(dy, dx))) % 360

        # Send roll command
        self.send_roll(target_heading, speed)
        return False

    def execute_patrol(self, task: Task) -> bool:
        """Patrol between waypoints."""
        waypoints = task.parameters.get('waypoints', [])
        speed = task.parameters.get('speed', self.default_speed)
        loop = task.parameters.get('loop', False)

        if 'current_waypoint_index' not in task.parameters:
            task.parameters['current_waypoint_index'] = 0

        current_idx = task.parameters['current_waypoint_index']

        if current_idx >= len(waypoints):
            if loop:
                task.parameters['current_waypoint_index'] = 0
                return False
            else:
                self.send_stop()
                return True

        # Move to current waypoint
        waypoint = waypoints[current_idx]
        dx = waypoint['x'] - self.current_position['x']
        dy = waypoint['y'] - self.current_position['y']
        distance = math.sqrt(dx**2 + dy**2)

        if distance < self.position_tolerance:
            # Reached waypoint, move to next
            task.parameters['current_waypoint_index'] += 1
            return False

        target_heading = int(math.degrees(math.atan2(dy, dx))) % 360
        self.send_roll(target_heading, speed)
        return False

    def execute_circle(self, task: Task) -> bool:
        """Move in a circular pattern."""
        radius = task.parameters.get('radius', 50)  # cm
        speed = task.parameters.get('speed', self.default_speed)
        duration = task.parameters.get('duration', 10.0)  # seconds

        if 'start_time' not in task.parameters:
            task.parameters['start_time'] = time.time()

        elapsed = time.time() - task.parameters['start_time']

        if elapsed >= duration:
            self.send_stop()
            return True

        # Rotate heading continuously
        heading = int((elapsed * 36) % 360)  # 10 degrees per second
        self.send_roll(heading, speed)
        return False

    def execute_square(self, task: Task) -> bool:
        """Move in a square pattern."""
        side_length = task.parameters.get('side_length', 100)  # cm
        speed = task.parameters.get('speed', self.default_speed)

        if 'waypoints' not in task.parameters:
            # Generate square waypoints
            start_x = self.current_position['x']
            start_y = self.current_position['y']
            task.parameters['waypoints'] = [
                {'x': start_x + side_length, 'y': start_y},
                {'x': start_x + side_length, 'y': start_y + side_length},
                {'x': start_x, 'y': start_y + side_length},
                {'x': start_x, 'y': start_y}
            ]
            task.parameters['current_waypoint_index'] = 0

        # Use patrol logic for square
        return self.execute_patrol(task)

    def execute_led_sequence(self, task: Task) -> bool:
        """Execute a sequence of LED colors."""
        sequence = task.parameters.get('sequence', [])
        interval = task.parameters.get('interval', 1.0)  # seconds
        loop = task.parameters.get('loop', False)

        if 'current_index' not in task.parameters:
            task.parameters['current_index'] = 0
            task.parameters['last_change_time'] = time.time()

        current_idx = task.parameters['current_index']

        if current_idx >= len(sequence):
            if loop:
                task.parameters['current_index'] = 0
                return False
            return True

        # Check if it's time to change color
        if time.time() - task.parameters['last_change_time'] >= interval:
            color = sequence[current_idx]
            self.send_led(color['red'], color['green'], color['blue'])
            task.parameters['current_index'] += 1
            task.parameters['last_change_time'] = time.time()

        return False

    def execute_matrix_sequence(self, task: Task) -> bool:
        """Execute a sequence of matrix patterns."""
        sequence = task.parameters.get('sequence', [])
        interval = task.parameters.get('interval', 2.0)  # seconds
        loop = task.parameters.get('loop', False)

        if 'current_index' not in task.parameters:
            task.parameters['current_index'] = 0
            task.parameters['last_change_time'] = time.time()

        current_idx = task.parameters['current_index']

        if current_idx >= len(sequence):
            if loop:
                task.parameters['current_index'] = 0
                return False
            return True

        # Check if it's time to change pattern
        if time.time() - task.parameters['last_change_time'] >= interval:
            pattern = sequence[current_idx]
            self.send_matrix(
                pattern.get('pattern', 'smile'),
                pattern.get('red', 255),
                pattern.get('green', 255),
                pattern.get('blue', 255)
            )
            task.parameters['current_index'] += 1
            task.parameters['last_change_time'] = time.time()

        return False

    def execute_spin(self, task: Task) -> bool:
        """Spin in place."""
        rotations = task.parameters.get('rotations', 1)
        speed = task.parameters.get('speed', 100)

        if 'start_time' not in task.parameters:
            task.parameters['start_time'] = time.time()
            task.parameters['start_heading'] = self.current_heading

        # Estimate time for rotation (rough approximation)
        rotation_time = (360 * rotations) / 90  # Assumes ~90 deg/sec at speed 100

        if time.time() - task.parameters['start_time'] >= rotation_time:
            self.send_stop()
            return True

        # Keep spinning
        self.send_roll(self.current_heading + 10, speed)
        return False

    def execute_stop(self, task: Task) -> bool:
        """Stop the Sphero and cancel any currently running task."""
        self.send_stop()

        # If there was a task running before this stop command, it should be cancelled
        # This is a special case - stop should execute immediately
        return True

    def execute_custom(self, task: Task) -> bool:
        """Execute custom command sequence."""
        commands = task.parameters.get('commands', [])

        if 'current_command_index' not in task.parameters:
            task.parameters['current_command_index'] = 0
            task.parameters['command_start_time'] = time.time()

        current_idx = task.parameters['current_command_index']

        if current_idx >= len(commands):
            return True

        command = commands[current_idx]
        duration = command.get('duration', 1.0)

        # Check if command duration has elapsed
        if time.time() - task.parameters['command_start_time'] >= duration:
            task.parameters['current_command_index'] += 1
            task.parameters['command_start_time'] = time.time()
            return False

        # Execute current command
        cmd_type = command.get('type')
        if cmd_type == 'led':
            self.send_led(command['red'], command['green'], command['blue'])
        elif cmd_type == 'roll':
            self.send_roll(command['heading'], command['speed'])
        elif cmd_type == 'matrix':
            self.send_matrix(command['pattern'], command.get('red', 255),
                           command.get('green', 255), command.get('blue', 255))
        elif cmd_type == 'stop':
            self.send_stop()

        return False

    # Command sending methods
    def send_led(self, red: int, green: int, blue: int):
        """Send LED color command."""
        msg = String()
        msg.data = json.dumps({'red': red, 'green': green, 'blue': blue})
        self.led_pub.publish(msg)

    def send_roll(self, heading: int, speed: int, duration: float = 0.0):
        """Send roll command."""
        msg = String()
        msg.data = json.dumps({'heading': heading % 360, 'speed': speed, 'duration': duration})
        self.roll_pub.publish(msg)

    def send_heading(self, heading: int):
        """Send heading command."""
        msg = String()
        msg.data = json.dumps({'heading': heading % 360})
        self.heading_pub.publish(msg)

    def send_speed(self, speed: int):
        """Send speed command."""
        msg = String()
        msg.data = json.dumps({'speed': speed})
        self.speed_pub.publish(msg)

    def send_stop(self):
        """Send stop command."""
        msg = String()
        msg.data = json.dumps({})
        self.stop_pub.publish(msg)

    def send_matrix(self, pattern: str, red: int = 255, green: int = 255, blue: int = 255):
        """Send matrix pattern command."""
        msg = String()
        msg.data = json.dumps({
            'pattern': pattern,
            'red': red,
            'green': green,
            'blue': blue,
            'duration': 0
        })
        self.matrix_pub.publish(msg)

    def publish_task_status(self, task: Task):
        """Publish task status update."""
        # Include queue information
        status_dict = task.to_dict()
        status_dict['queue_length'] = len(self.task_queue)
        status_dict['has_current_task'] = self.current_task is not None
        status_dict['total_pending'] = len(self.task_queue) + (1 if self.current_task is not None else 0)

        msg = String()
        msg.data = json.dumps(status_dict)
        self.task_status_pub.publish(msg)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    try:
        node = SpheroTaskController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
