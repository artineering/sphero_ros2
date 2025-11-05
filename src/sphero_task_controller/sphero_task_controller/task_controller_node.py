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
    # High-level tasks
    MOVE_TO = "move_to"
    PATROL = "patrol"
    CIRCLE = "circle"
    SQUARE = "square"
    LED_SEQUENCE = "led_sequence"
    MATRIX_SEQUENCE = "matrix_sequence"
    SPIN = "spin"
    STOP = "stop"
    CUSTOM = "custom"
    # Basic/immediate commands
    SET_LED = "set_led"
    ROLL = "roll"
    HEADING = "heading"
    SPEED = "speed"
    MATRIX = "matrix"


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
        - /sphero/reset_aim (std_msgs/String): Reset position and heading to origin

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

        self.reset_aim_sub = self.create_subscription(
            String,
            'sphero/reset_aim',
            self.reset_aim_callback,
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
            self.get_logger().info(f'Task parameters: {json.dumps(task.parameters, indent=2)}')

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

    def reset_aim_callback(self, msg: String):
        """Reset position and heading to origin when reset_aim command is received."""
        self.current_position = {'x': 0.0, 'y': 0.0}
        self.current_heading = 0
        self.get_logger().info('Task controller reset to origin: heading=0°, position=(0, 0)')

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

            # Update current position from state before starting task
            if 'position' in self.current_state:
                self.current_position = self.current_state['position'].copy()
                self.get_logger().info(
                    f'Starting task {self.current_task.task_id} at position: '
                    f'x={self.current_position.get("x", 0.0):.2f}, '
                    f'y={self.current_position.get("y", 0.0):.2f}'
                )
            else:
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

        # Log first execution of task
        if 'first_execution_logged' not in task.parameters:
            task.parameters['first_execution_logged'] = True
            self.get_logger().info(
                f'EXECUTE_TASK: {task.task_id} type={task_type}, '
                f'parameters={json.dumps(task.parameters, default=str)}'
            )

        # High-level tasks
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
        # Basic/immediate commands
        elif task_type == TaskType.SET_LED.value:
            return self.execute_basic_set_led(task)
        elif task_type == TaskType.ROLL.value:
            return self.execute_basic_roll(task)
        elif task_type == TaskType.HEADING.value:
            return self.execute_basic_heading(task)
        elif task_type == TaskType.SPEED.value:
            return self.execute_basic_speed(task)
        elif task_type == TaskType.MATRIX.value:
            return self.execute_basic_matrix(task)
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

        # Debug logging
        self.get_logger().debug(
            f'MOVE_TO: target=({target_x}, {target_y}), '
            f'current=({self.current_position["x"]:.2f}, {self.current_position["y"]:.2f}), '
            f'distance={distance:.2f}cm, speed={speed}'
        )

        # Check if we've reached the target
        if distance < self.position_tolerance:
            self.get_logger().info(f'Reached target position (within {self.position_tolerance}cm)')
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
            self.get_logger().info(f'PATROL: Starting patrol with {len(waypoints)} waypoints, speed={speed}, loop={loop}')

        current_idx = task.parameters['current_waypoint_index']

        if current_idx >= len(waypoints):
            if loop:
                self.get_logger().info('PATROL: Completed loop, restarting')
                task.parameters['current_waypoint_index'] = 0
                return False
            else:
                self.get_logger().info('PATROL: All waypoints reached, stopping')
                self.send_stop()
                return True

        # Move to current waypoint
        waypoint = waypoints[current_idx]
        dx = waypoint['x'] - self.current_position['x']
        dy = waypoint['y'] - self.current_position['y']
        distance = math.sqrt(dx**2 + dy**2)

        # Debug logging
        self.get_logger().debug(
            f'PATROL: waypoint[{current_idx}]=({waypoint["x"]}, {waypoint["y"]}), '
            f'current=({self.current_position["x"]:.2f}, {self.current_position["y"]:.2f}), '
            f'distance={distance:.2f}cm, speed={speed}'
        )

        if distance < self.position_tolerance:
            # Reached waypoint, move to next
            self.get_logger().info(f'PATROL: Reached waypoint {current_idx}, moving to next')
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
            self.get_logger().info(f'CIRCLE: Starting circle, radius={radius}cm, speed={speed}, duration={duration}s')

        elapsed = time.time() - task.parameters['start_time']

        if elapsed >= duration:
            self.get_logger().info(f'CIRCLE: Completed after {elapsed:.2f}s')
            self.send_stop()
            return True

        # Rotate heading continuously
        heading = int((elapsed * 36) % 360)  # 10 degrees per second
        self.get_logger().debug(f'CIRCLE: elapsed={elapsed:.2f}s, heading={heading}°, speed={speed}')
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
            waypoints = [
                {'x': start_x + side_length, 'y': start_y},
                {'x': start_x + side_length, 'y': start_y + side_length},
                {'x': start_x, 'y': start_y + side_length},
                {'x': start_x, 'y': start_y}
            ]
            task.parameters['waypoints'] = waypoints
            task.parameters['current_waypoint_index'] = 0
            self.get_logger().info(
                f'SQUARE: Generated square from ({start_x:.2f}, {start_y:.2f}), '
                f'side_length={side_length}cm, speed={speed}'
            )
            self.get_logger().info(f'SQUARE: Waypoints: {waypoints}')

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
        """
        Send roll command to Sphero.

        Args:
            heading: Direction to move (0-359 degrees)
            speed: Speed to move (0-255)
            duration: Duration in seconds (0.0 = continuous/indefinite movement)

        Note: When duration=0.0, the Sphero will continue moving at the specified
        heading and speed until a new command is received. This is the correct
        behavior for navigation tasks like move_to and patrol.
        """
        msg = String()
        cmd_data = {'heading': heading % 360, 'speed': speed, 'duration': duration}
        msg.data = json.dumps(cmd_data)
        self.roll_pub.publish(msg)

        # Debug logging
        self.get_logger().debug(
            f'ROLL CMD: heading={heading % 360}°, speed={speed}, duration={duration}s'
        )

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

    # Basic/immediate command execution methods
    def execute_basic_set_led(self, task: Task) -> bool:
        """Execute basic LED command - completes immediately."""
        params = task.parameters
        color = params.get('color', 'white').lower()

        # Color name to RGB mapping
        color_map = {
            'red': (255, 0, 0), 'green': (0, 255, 0), 'blue': (0, 0, 255),
            'yellow': (255, 255, 0), 'cyan': (0, 255, 255), 'magenta': (255, 0, 255),
            'white': (255, 255, 255), 'orange': (255, 165, 0), 'purple': (128, 0, 128),
            'pink': (255, 192, 203), 'off': (0, 0, 0)
        }

        if 'red' in params and 'green' in params and 'blue' in params:
            red, green, blue = params['red'], params['green'], params['blue']
        elif color in color_map:
            red, green, blue = color_map[color]
        else:
            red, green, blue = 255, 255, 255

        self.send_led(red, green, blue)
        self.get_logger().info(f'SET_LED: RGB({red}, {green}, {blue})')
        return True  # Immediate completion

    def execute_basic_roll(self, task: Task) -> bool:
        """Execute basic roll command."""
        params = task.parameters
        heading = params.get('heading', 0)
        speed = params.get('speed', 100)
        duration = params.get('duration', 0.0)

        if duration > 0:
            # Duration-based roll: wait for specified duration
            if 'start_time' not in params:
                self.send_roll(heading, speed, duration)
                self.get_logger().info(f'ROLL: heading={heading}, speed={speed}, duration={duration}')
                task.parameters['start_time'] = time.time()
                return False  # Not complete yet

            elapsed = time.time() - task.parameters['start_time']
            return elapsed >= duration
        else:
            # Indefinite roll (duration=0): "start rolling" - complete immediately
            self.send_roll(heading, speed, duration)
            self.get_logger().info(f'ROLL: heading={heading}, speed={speed}, duration={duration} (indefinite)')
            return True

    def execute_basic_heading(self, task: Task) -> bool:
        """Execute basic heading command - completes immediately."""
        heading = task.parameters.get('heading', 0)
        self.send_heading(heading)
        self.get_logger().info(f'HEADING: {heading}')
        return True

    def execute_basic_speed(self, task: Task) -> bool:
        """Execute basic speed command - completes immediately."""
        speed = task.parameters.get('speed', 0)
        self.send_speed(speed)
        self.get_logger().info(f'SPEED: {speed}')
        return True

    def execute_basic_matrix(self, task: Task) -> bool:
        """Execute basic matrix command - completes immediately."""
        params = task.parameters
        pattern = params.get('pattern', 'smile')
        red = params.get('red', 255)
        green = params.get('green', 255)
        blue = params.get('blue', 255)

        self.send_matrix(pattern, red, green, blue)
        self.get_logger().info(f'MATRIX: pattern={pattern}, RGB({red}, {green}, {blue})')
        return True

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
