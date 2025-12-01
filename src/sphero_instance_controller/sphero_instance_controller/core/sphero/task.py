#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Sphero Task Execution System.

This module provides high-level task management and execution for Sphero robots,
including movement patterns, LED sequences, and complex behaviors.
"""

import time
import math
import random
from enum import Enum
from typing import Dict, List, Any, Optional, Callable
from dataclasses import dataclass, field

from .sphero import Sphero


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
    COLLISION = "collision"
    REFLECT = "reflect"
    JUMPING_BEAN = "jumping_bean"


@dataclass
class TaskDescriptor:
    """Describes a task to be executed."""
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


class TaskExecutorBase:
    """
    Base class for task executors.

    This abstract class defines the common interface and queue management
    for task execution. Subclasses implement hardware-specific or topic-based
    command execution.
    """

    def __init__(self,
                 position_callback: Optional[Callable[[], Dict[str, float]]] = None,
                 heading_callback: Optional[Callable[[], int]] = None):
        """
        Initialize the base task executor.

        Args:
            position_callback: Callback to get current position {'x': float, 'y': float}
            heading_callback: Callback to get current heading (0-359 degrees)
        """
        self.position_callback = position_callback
        self.heading_callback = heading_callback

        # Configuration
        self.default_speed = 100
        self.position_tolerance = 10.0  # cm
        self.heading_tolerance = 5  # degrees

        # Task queue management
        self.task_queue: List[TaskDescriptor] = []
        self.current_task: Optional[TaskDescriptor] = None
        self.task_history: List[TaskDescriptor] = []

    def get_current_position(self) -> Dict[str, float]:
        """Get current position from callback."""
        if self.position_callback:
            return self.position_callback()
        return {'x': 0.0, 'y': 0.0}

    def get_current_heading(self) -> int:
        """Get current heading from callback."""
        if self.heading_callback:
            return self.heading_callback()
        return 0

    # Abstract methods to be implemented by subclasses
    def _send_raw_motor_command(self, left_mode: str, left_speed: int, right_mode: str, right_speed: int):
        """Send raw motor command - to be implemented by subclass."""
        raise NotImplementedError("Subclass must implement _send_raw_motor_command")

    def _send_roll_command(self, heading: int, speed: int, duration: float = 0):
        """Send roll command - to be implemented by subclass."""
        raise NotImplementedError("Subclass must implement _send_roll_command")

    def _send_stop_command(self):
        """Send stop command - to be implemented by subclass."""
        raise NotImplementedError("Subclass must implement _send_stop_command")

    def _send_led_command(self, red: int, green: int, blue: int, led_type: str = 'main'):
        """Send LED command - to be implemented by subclass."""
        raise NotImplementedError("Subclass must implement _send_led_command")

    def _send_heading_command(self, heading: int):
        """Send heading command - to be implemented by subclass."""
        raise NotImplementedError("Subclass must implement _send_heading_command")

    def _send_speed_command(self, speed: int):
        """Send speed command - to be implemented by subclass."""
        raise NotImplementedError("Subclass must implement _send_speed_command")

    def _send_spin_command(self, angle: int, duration: float = 1.0):
        """Send spin command - to be implemented by subclass."""
        raise NotImplementedError("Subclass must implement _send_spin_command")

    def _send_matrix_command(self, pattern: str = None, red: int = 255, green: int = 255, blue: int = 255):
        """Send matrix command - to be implemented by subclass."""
        raise NotImplementedError("Subclass must implement _send_matrix_command")

    def _send_stabilization_command(self, enable: bool):
        """Send stabilization command - to be implemented by subclass."""
        raise NotImplementedError("Subclass must implement _send_stabilization_command")

    def _send_collision_detection_command(self, action: str, mode: str = 'obstacle', sensitivity: str = 'HIGH'):
        """Send collision detection command - to be implemented by subclass."""
        raise NotImplementedError("Subclass must implement _send_collision_detection_command")

    def add_task(self, task: TaskDescriptor):
        """Add a task to the queue."""
        self.task_queue.append(task)

    def process_tasks(self) -> Optional[TaskDescriptor]:
        """
        Process task queue - should be called periodically.

        Returns:
            Current task being executed, or None
        """
        # Check if next task in queue is a STOP task that should cancel current task
        if self.current_task is not None and len(self.task_queue) > 0:
            next_task = self.task_queue[0]
            if next_task.task_type.lower() == TaskType.STOP.value:
                delay = next_task.parameters.get('delay', 0.0)
                if delay == 0.0:
                    # Immediate stop - cancel current task
                    self.current_task.status = TaskStatus.CANCELLED
                    self.current_task.completed_at = time.time()
                    self.task_history.append(self.current_task)
                    self.current_task = None

        # Start next task if no task is currently running
        if self.current_task is None and len(self.task_queue) > 0:
            self.current_task = self.task_queue.pop(0)
            self.current_task.status = TaskStatus.RUNNING
            self.current_task.started_at = time.time()

        # Execute current task
        if self.current_task is not None:
            try:
                completed = self.execute_task(self.current_task)

                if completed:
                    self.current_task.status = TaskStatus.COMPLETED
                    self.current_task.completed_at = time.time()
                    self.task_history.append(self.current_task)
                    self.current_task = None

            except Exception as e:
                self.current_task.status = TaskStatus.FAILED
                self.current_task.error_message = str(e)
                self.current_task.completed_at = time.time()
                self.task_history.append(self.current_task)
                self.current_task = None

        return self.current_task

    def execute_task(self, task: TaskDescriptor) -> bool:
        """
        Execute a task based on its type.

        Returns:
            bool: True if task is completed, False if still running
        """
        task_type = task.task_type.lower()

        # High-level tasks
        if task_type == TaskType.MOVE_TO.value:
            return self._execute_move_to(task)
        elif task_type == TaskType.PATROL.value:
            return self._execute_patrol(task)
        elif task_type == TaskType.CIRCLE.value:
            return self._execute_circle(task)
        elif task_type == TaskType.SQUARE.value:
            return self._execute_square(task)
        elif task_type == TaskType.LED_SEQUENCE.value:
            return self._execute_led_sequence(task)
        elif task_type == TaskType.MATRIX_SEQUENCE.value:
            return self._execute_matrix_sequence(task)
        elif task_type == TaskType.SPIN.value:
            return self._execute_spin(task)
        elif task_type == TaskType.STOP.value:
            return self._execute_stop(task)
        elif task_type == TaskType.CUSTOM.value:
            return self._execute_custom(task)
        # Basic/immediate commands
        elif task_type == TaskType.SET_LED.value:
            return self._execute_basic_set_led(task)
        elif task_type == TaskType.ROLL.value:
            return self._execute_basic_roll(task)
        elif task_type == TaskType.HEADING.value:
            return self._execute_basic_heading(task)
        elif task_type == TaskType.SPEED.value:
            return self._execute_basic_speed(task)
        elif task_type == TaskType.MATRIX.value:
            return self._execute_basic_matrix(task)
        elif task_type == TaskType.COLLISION.value:
            return self._execute_basic_collision(task)
        elif task_type == TaskType.REFLECT.value:
            return self._execute_basic_reflect(task)
        elif task_type == TaskType.JUMPING_BEAN.value:
            return self._execute_jumping_bean(task)
        else:
            raise ValueError(f'Unknown task type: {task_type}')

    # ===== High-level Task Execution =====

    def _execute_move_to(self, task: TaskDescriptor) -> bool:
        """Move to a specific position."""
        target_x = task.parameters.get('x', 0)
        target_y = task.parameters.get('y', 0)
        speed = task.parameters.get('speed', self.default_speed)

        current_pos = self.get_current_position()

        # Calculate distance and heading
        dx = target_x - current_pos['x']
        dy = target_y - current_pos['y']
        distance = math.sqrt(dx**2 + dy**2)

        # Check if we've reached the target
        if distance < self.position_tolerance:
            self._send_stop_command()
            return True

        # Calculate heading to target
        target_heading = int(math.degrees(math.atan2(dy, dx))) % 360

        # Send roll command only if heading changed or first call
        if 'last_heading' not in task.parameters or task.parameters['last_heading'] != target_heading:
            self._send_roll_command(target_heading, speed)
            task.parameters['last_heading'] = target_heading

        return False

    def _execute_patrol(self, task: TaskDescriptor) -> bool:
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
                self._send_stop_command()
                return True

        # Move to current waypoint
        waypoint = waypoints[current_idx]
        current_pos = self.get_current_position()

        dx = waypoint['x'] - current_pos['x']
        dy = waypoint['y'] - current_pos['y']
        distance = math.sqrt(dx**2 + dy**2)

        if distance < self.position_tolerance:
            # Reached waypoint, move to next
            task.parameters['current_waypoint_index'] += 1
            # Clear last heading so new waypoint gets fresh command
            if 'last_heading' in task.parameters:
                del task.parameters['last_heading']
            return False

        target_heading = int(math.degrees(math.atan2(dy, dx))) % 360

        # Send roll command only if heading changed or first call for this waypoint
        if 'last_heading' not in task.parameters or task.parameters['last_heading'] != target_heading:
            self._send_roll_command(target_heading, speed)
            task.parameters['last_heading'] = target_heading

        return False

    def _execute_circle(self, task: TaskDescriptor) -> bool:
        """
        Move in a circular pattern using differential motor speeds.

        The circle motion is achieved by setting different speeds for left and right motors.
        The speed differential is calculated based on the desired radius and base speed.

        For a Sphero with wheel separation d (5.0 cm), to achieve circular motion
        with radius r at average speed v:
        - Outer wheel speed: v_outer = v * (r + d/2) / r
        - Inner wheel speed: v_inner = v * (r - d/2) / r
        """
        radius = task.parameters.get('radius', 50)  # cm (radius of circle path)
        speed = task.parameters.get('speed', self.default_speed)  # average speed
        duration = task.parameters.get('duration', 10.0)  # seconds
        direction = task.parameters.get('direction', 'ccw').lower()  # 'cw' or 'ccw'

        # Initialize on first call - send motor command ONCE
        if 'start_time' not in task.parameters:
            task.parameters['start_time'] = time.time()

            # Sphero wheel separation (distance between left and right motors)
            wheel_separation = 5.0  # cm

            # Calculate differential speeds for circular motion
            # Clamp radius to avoid division by very small numbers
            effective_radius = max(radius, wheel_separation)

            # Calculate speed ratios
            outer_ratio = (effective_radius + wheel_separation / 2) / effective_radius
            inner_ratio = (effective_radius - wheel_separation / 2) / effective_radius

            # Calculate motor speeds (0-255 range)
            outer_speed = int(min(255, speed * outer_ratio))
            inner_speed = int(min(255, speed * inner_ratio))

            # Determine which motor is inner/outer based on direction
            if direction == 'cw':  # Clockwise: right motor is inner
                left_speed = outer_speed
                right_speed = inner_speed
            else:  # Counter-clockwise (default): left motor is inner
                left_speed = inner_speed
                right_speed = outer_speed

            # Send motor command ONCE at the start
            self._send_raw_motor_command('forward', left_speed, 'forward', right_speed)
            return False  # Task started, still running

        # Check if duration has elapsed
        elapsed = time.time() - task.parameters['start_time']
        if elapsed >= duration:
            self._send_stop_command()
            return True  # Task completed

        # Still running - motors already set, just wait
        return False

    def _execute_square(self, task: TaskDescriptor) -> bool:
        """Move in a square pattern."""
        side_length = task.parameters.get('side_length', 100)  # cm
        speed = task.parameters.get('speed', self.default_speed)

        if 'waypoints' not in task.parameters:
            # Generate square waypoints
            current_pos = self.get_current_position()
            start_x = current_pos['x']
            start_y = current_pos['y']

            waypoints = [
                {'x': start_x + side_length, 'y': start_y},
                {'x': start_x + side_length, 'y': start_y + side_length},
                {'x': start_x, 'y': start_y + side_length},
                {'x': start_x, 'y': start_y}
            ]
            task.parameters['waypoints'] = waypoints
            task.parameters['current_waypoint_index'] = 0

        # Use patrol logic for square
        return self._execute_patrol(task)

    def _execute_led_sequence(self, task: TaskDescriptor) -> bool:
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
            self._send_led_command(color['red'], color['green'], color['blue'])
            task.parameters['current_index'] += 1
            task.parameters['last_change_time'] = time.time()

        return False

    def _execute_matrix_sequence(self, task: TaskDescriptor) -> bool:
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
            self._send_matrix_command(
                pattern=pattern.get('pattern', 'smile'),
                red=pattern.get('red', 255),
                green=pattern.get('green', 255),
                blue=pattern.get('blue', 255)
            )
            task.parameters['current_index'] += 1
            task.parameters['last_change_time'] = time.time()

        return False

    def _execute_spin(self, task: TaskDescriptor) -> bool:
        """Spin in place."""
        rotations = task.parameters.get('rotations', 1)
        speed = task.parameters.get('speed', 100)

        if 'start_time' not in task.parameters:
            task.parameters['start_time'] = time.time()
            task.parameters['start_heading'] = self.get_current_heading()
            # Use spin command which handles rotation internally
            angle = int(360 * rotations)
            duration = (360 * rotations) / 90  # Estimate: ~90 deg/sec
            self._send_spin_command(angle, duration)
            task.parameters['rotation_time'] = duration
            return False

        # Check if rotation time has elapsed
        if time.time() - task.parameters['start_time'] >= task.parameters['rotation_time']:
            return True  # Completed

        # Still spinning - command already sent
        return False

    def _execute_stop(self, task: TaskDescriptor) -> bool:
        """Stop the Sphero."""
        self._send_stop_command()
        return True

    def _execute_custom(self, task: TaskDescriptor) -> bool:
        """Execute custom command sequence."""
        commands = task.parameters.get('commands', [])

        if 'current_command_index' not in task.parameters:
            task.parameters['current_command_index'] = 0
            task.parameters['command_start_time'] = time.time()
            task.parameters['command_executed'] = False

        current_idx = task.parameters['current_command_index']

        if current_idx >= len(commands):
            return True

        command = commands[current_idx]
        duration = command.get('duration', 1.0)

        # Execute command only once per sequence step
        if not task.parameters['command_executed']:
            cmd_type = command.get('type')
            if cmd_type == 'led':
                self._send_led_command(command['red'], command['green'], command['blue'])
            elif cmd_type == 'roll':
                self._send_roll_command(command['heading'], command['speed'])
            elif cmd_type == 'matrix':
                self._send_matrix_command(
                    pattern=command['pattern'],
                    red=command.get('red', 255),
                    green=command.get('green', 255),
                    blue=command.get('blue', 255)
                )
            elif cmd_type == 'stop':
                self._send_stop_command()
            task.parameters['command_executed'] = True

        # Check if command duration has elapsed
        if time.time() - task.parameters['command_start_time'] >= duration:
            task.parameters['current_command_index'] += 1
            task.parameters['command_start_time'] = time.time()
            task.parameters['command_executed'] = False  # Reset for next command
            return False

        return False

    # ===== Basic Command Execution =====

    def _execute_basic_set_led(self, task: TaskDescriptor) -> bool:
        """Execute basic LED command."""
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

        self._send_led_command(red, green, blue)
        return True

    def _execute_basic_roll(self, task: TaskDescriptor) -> bool:
        """Execute basic roll command."""
        params = task.parameters
        heading = params.get('heading', 0)
        speed = params.get('speed', 100)
        duration = params.get('duration', 0.0)

        if duration > 0:
            # Duration-based roll
            if 'start_time' not in params:
                self._send_roll_command(heading, speed, duration)
                task.parameters['start_time'] = time.time()
                return False

            elapsed = time.time() - task.parameters['start_time']
            return elapsed >= duration
        else:
            # Indefinite roll
            self._send_roll_command(heading, speed, duration)
            return True

    def _execute_basic_heading(self, task: TaskDescriptor) -> bool:
        """Execute basic heading command."""
        heading = task.parameters.get('heading', 0)
        self._send_heading_command(heading)
        return True

    def _execute_basic_speed(self, task: TaskDescriptor) -> bool:
        """Execute basic speed command."""
        speed = task.parameters.get('speed', 0)
        self._send_speed_command(speed)
        return True

    def _execute_basic_matrix(self, task: TaskDescriptor) -> bool:
        """Execute basic matrix command."""
        params = task.parameters
        pattern = params.get('pattern', 'smile')
        red = params.get('red', 255)
        green = params.get('green', 255)
        blue = params.get('blue', 255)

        self._send_matrix_command(pattern=pattern, red=red, green=green, blue=blue)
        return True

    def _execute_basic_collision(self, task: TaskDescriptor) -> bool:
        """Execute collision detection command."""
        params = task.parameters
        action = params.get('action', 'start')
        mode = params.get('mode', 'obstacle')
        sensitivity = params.get('sensitivity', 'HIGH')

        self._send_collision_detection_command(action, mode, sensitivity)
        return True

    def _execute_basic_reflect(self, task: TaskDescriptor) -> bool:
        """Execute reflect command - reverses heading with random offset."""
        params = task.parameters
        offset_min = params.get('offset_min', -45)
        offset_max = params.get('offset_max', 45)
        speed = params.get('speed', 80)

        current_heading = self.get_current_heading()
        reverse_heading = (current_heading + 180) % 360

        # Add random offset
        offset = random.randint(offset_min, offset_max)
        new_heading = (reverse_heading + offset) % 360

        self._send_roll_command(new_heading, speed, duration=0.0)
        return True

    def _execute_jumping_bean(self, task: TaskDescriptor) -> bool:
        """Execute jumping bean behavior."""
        params = task.parameters
        duration = params.get('duration', 10.0)
        flip_interval = params.get('flip_interval', 0.1)
        speed = params.get('speed', 200)

        # Disable stabilization
        self._send_stabilization_command(False)

        # Calculate flips
        num_flips = int(duration / flip_interval)
        current_heading = random.randint(0, 359)
        current_speed = speed
        flip_count = 0

        for i in range(num_flips):
            self._send_roll_command(current_heading, current_speed, flip_interval)
            flip_count += 1
            current_speed = -current_speed

            if flip_count % 10 == 0:
                current_heading = random.randint(0, 359)

            time.sleep(flip_interval)

        # Stop and re-enable stabilization
        self._send_stop_command()
        self._send_stabilization_command(True)

        return True
