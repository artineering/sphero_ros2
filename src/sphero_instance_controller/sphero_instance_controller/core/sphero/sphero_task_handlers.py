#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Sphero task handlers.

Each handler implements one entry in the Sphero TaskType catalog. Handlers are
plain functions with signature ``(executor, task) -> bool``: return True when
the task is complete, False to keep it running on the next
``process_tasks()`` tick.

The handlers call back into the executor for two things:
  * ``executor._send_*`` — the abstract command-emission methods that
    DirectTaskExecutor / TopicTaskExecutor implement.
  * ``executor.get_current_position()`` / ``executor.get_current_heading()``
    and the ``executor.default_speed`` / ``executor.position_tolerance``
    config knobs.

These functions are registered on a SpheroTaskExecutorBase instance via
``executor.register_handler(name, fn)`` from
``SpheroTaskExecutorBase._register_default_handlers``.
"""

import math
import random
import time
from enum import Enum

from sphero_instance_controller.core.common.task import TaskDescriptor, TaskStatus


# Default angular rate for duration-based spins: one full rotation per second.
# When a spin task supplies an explicit ``duration``, the swept ``angle`` is
# derived as ``SPIN_DEG_PER_SEC * duration`` so the Sphero keeps spinning for
# the whole requested time instead of crawling.
SPIN_DEG_PER_SEC = 360.0


class TaskType(Enum):
    """Catalog of Sphero task types."""
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
    CALIBRATE_COMPASS = "calibrate_compass"


# ===== High-level task handlers =====

def execute_move_to(executor, task: TaskDescriptor) -> bool:
    """Move to a specific (x, y) target with stall detection."""
    target_x = task.parameters.get('x', 0)
    target_y = task.parameters.get('y', 0)
    speed = task.parameters.get('speed', executor.default_speed)

    stall_timeout = task.parameters.get('stall_timeout', 5.0)
    stall_distance_threshold = task.parameters.get('stall_distance_threshold', 2.0)

    current_pos = executor.get_current_position()
    dx = target_x - current_pos['x']
    dy = target_y - current_pos['y']
    distance = math.sqrt(dx ** 2 + dy ** 2)

    print(f"[MOVE_TO] current distance = {distance}cm")

    if distance < executor.position_tolerance:
        executor._send_stop_command()
        return True

    if 'command_sent' not in task.parameters:
        target_heading = int(math.degrees(math.atan2(dy, dx))) % 360
        executor._send_roll_command(target_heading, speed)
        task.parameters['command_sent'] = True
        task.parameters['last_distance'] = distance
        task.parameters['last_progress_time'] = time.time()
        task.parameters['stalled'] = False
        return False

    last_distance = task.parameters.get('last_distance', distance)
    last_progress_time = task.parameters.get('last_progress_time', time.time())
    distance_change = last_distance - distance

    if distance_change > stall_distance_threshold:
        task.parameters['last_distance'] = distance
        task.parameters['last_progress_time'] = time.time()
        task.parameters['stalled'] = False
    else:
        time_since_progress = time.time() - last_progress_time
        if time_since_progress >= stall_timeout:
            if not task.parameters.get('stalled', False):
                print(f"[MOVE_TO] STALLED: No progress for {stall_timeout}s (distance: {distance}cm)")
                task.parameters['stalled'] = True
                executor._send_stop_command()
                task.status = TaskStatus.FAILED
                task.error_message = f"Stalled: No progress for {stall_timeout}s"
            return True

    return False


def execute_patrol(executor, task: TaskDescriptor) -> bool:
    """Patrol between waypoints, optionally looping."""
    waypoints = task.parameters.get('waypoints', [])
    speed = task.parameters.get('speed', executor.default_speed)
    loop = task.parameters.get('loop', False)

    if 'current_waypoint_index' not in task.parameters:
        task.parameters['current_waypoint_index'] = 0

    current_idx = task.parameters['current_waypoint_index']

    if current_idx >= len(waypoints):
        if loop:
            task.parameters['current_waypoint_index'] = 0
            if 'command_sent' in task.parameters:
                del task.parameters['command_sent']
            return False
        else:
            executor._send_stop_command()
            return True

    waypoint = waypoints[current_idx]
    task.parameters['x'] = waypoint['x']
    task.parameters['y'] = waypoint['y']
    task.parameters['speed'] = speed

    current_pos = executor.get_current_position()
    dx = waypoint['x'] - current_pos['x']
    dy = waypoint['y'] - current_pos['y']
    distance = math.sqrt(dx ** 2 + dy ** 2)

    if distance < executor.position_tolerance:
        task.parameters['current_waypoint_index'] += 1
        if 'command_sent' in task.parameters:
            del task.parameters['command_sent']
        return False

    return execute_move_to(executor, task)


def execute_circle(executor, task: TaskDescriptor) -> bool:
    """
    Move in a circular pattern using differential motor speeds.

    For a Sphero with wheel separation d, to achieve a circle of radius r at
    average speed v: outer wheel = v * (r + d/2) / r, inner = v * (r - d/2) / r.
    """
    radius = task.parameters.get('radius', 50)
    speed = task.parameters.get('speed', executor.default_speed)
    duration = task.parameters.get('duration', 10.0)
    direction = task.parameters.get('direction', 'ccw').lower()

    if 'start_time' not in task.parameters:
        task.parameters['start_time'] = time.time()

        wheel_separation = 5.0
        effective_radius = max(radius, wheel_separation)
        outer_ratio = (effective_radius + wheel_separation / 2) / effective_radius
        inner_ratio = (effective_radius - wheel_separation / 2) / effective_radius
        outer_speed = int(min(255, speed * outer_ratio))
        inner_speed = int(min(255, speed * inner_ratio))

        if direction == 'cw':
            left_speed = outer_speed
            right_speed = inner_speed
        else:
            left_speed = inner_speed
            right_speed = outer_speed

        executor._send_raw_motor_command('forward', left_speed, 'forward', right_speed)
        return False

    elapsed = time.time() - task.parameters['start_time']
    if elapsed >= duration:
        executor._send_stop_command()
        return True

    return False


def execute_square(executor, task: TaskDescriptor) -> bool:
    """Move in a square pattern (uses patrol logic over generated waypoints)."""
    side_length = task.parameters.get('side_length', 100)

    if 'waypoints' not in task.parameters:
        current_pos = executor.get_current_position()
        start_x = current_pos['x']
        start_y = current_pos['y']
        task.parameters['waypoints'] = [
            {'x': start_x + side_length, 'y': start_y},
            {'x': start_x + side_length, 'y': start_y + side_length},
            {'x': start_x, 'y': start_y + side_length},
            {'x': start_x, 'y': start_y},
        ]
        task.parameters['current_waypoint_index'] = 0

    return execute_patrol(executor, task)


def execute_led_sequence(executor, task: TaskDescriptor) -> bool:
    """Cycle through a sequence of LED colors at a fixed interval."""
    sequence = task.parameters.get('sequence', [])
    interval = task.parameters.get('interval', 1.0)
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

    if time.time() - task.parameters['last_change_time'] >= interval:
        color = sequence[current_idx]
        executor._send_led_command(color['red'], color['green'], color['blue'])
        task.parameters['current_index'] += 1
        task.parameters['last_change_time'] = time.time()

    return False


def execute_matrix_sequence(executor, task: TaskDescriptor) -> bool:
    """Cycle through a sequence of matrix patterns at a fixed interval."""
    sequence = task.parameters.get('sequence', [])
    interval = task.parameters.get('interval', 2.0)
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

    if time.time() - task.parameters['last_change_time'] >= interval:
        pattern = sequence[current_idx]
        executor._send_matrix_command(
            pattern=pattern.get('pattern', 'smile'),
            red=pattern.get('red', 255),
            green=pattern.get('green', 255),
            blue=pattern.get('blue', 255),
        )
        task.parameters['current_index'] += 1
        task.parameters['last_change_time'] = time.time()

    return False


def execute_spin(executor, task: TaskDescriptor) -> bool:
    """Spin in place for an explicit ``duration`` or by N ``rotations``.

    If ``duration`` > 0 it takes precedence: the Sphero spins for exactly that
    many seconds, with the swept ``angle`` derived as
    ``SPIN_DEG_PER_SEC * duration`` (one rotation per second). Otherwise the
    legacy ``rotations`` path is used: ``angle = 360 * rotations`` swept over
    ``(360 * rotations) / 90`` seconds. ``speed`` is accepted for back-compat
    but is not used in the angle math.
    """
    duration = task.parameters.get('duration', 0.0)
    rotations = task.parameters.get('rotations', 1)
    speed = task.parameters.get('speed', 100)

    if 'start_time' not in task.parameters:
        task.parameters['start_time'] = time.time()
        task.parameters['start_heading'] = executor.get_current_heading()
        if duration > 0:
            angle = int(SPIN_DEG_PER_SEC * duration)
            effective_duration = duration
        else:
            angle = int(360 * rotations)
            effective_duration = (360 * rotations) / 90
        executor._send_spin_command(angle, effective_duration)
        task.parameters['rotation_time'] = effective_duration
        return False

    if time.time() - task.parameters['start_time'] >= task.parameters['rotation_time']:
        return True

    return False


def execute_stop(executor, task: TaskDescriptor) -> bool:
    """Stop the Sphero."""
    executor._send_stop_command()
    return True


def execute_calibrate_compass(executor, task: TaskDescriptor) -> bool:
    """Trigger a BOLT compass (magnetometer) calibration; the robot spins in place."""
    executor._send_calibrate_compass_command()
    return True


def execute_custom(executor, task: TaskDescriptor) -> bool:
    """Execute a custom timed sequence of mixed commands."""
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

    if not task.parameters['command_executed']:
        cmd_type = command.get('type')
        if cmd_type == 'led':
            executor._send_led_command(command['red'], command['green'], command['blue'])
        elif cmd_type == 'roll':
            executor._send_roll_command(command['heading'], command['speed'])
        elif cmd_type == 'matrix':
            executor._send_matrix_command(
                pattern=command['pattern'],
                red=command.get('red', 255),
                green=command.get('green', 255),
                blue=command.get('blue', 255),
            )
        elif cmd_type == 'stop':
            executor._send_stop_command()
        task.parameters['command_executed'] = True

    if time.time() - task.parameters['command_start_time'] >= duration:
        task.parameters['current_command_index'] += 1
        task.parameters['command_start_time'] = time.time()
        task.parameters['command_executed'] = False
        return False

    return False


# ===== Basic / immediate command handlers =====

def execute_set_led(executor, task: TaskDescriptor) -> bool:
    """Set the main LED to a named color or explicit RGB."""
    params = task.parameters
    color = params.get('color', 'white').lower()

    color_map = {
        'red': (255, 0, 0), 'green': (0, 255, 0), 'blue': (0, 0, 255),
        'yellow': (255, 255, 0), 'cyan': (0, 255, 255), 'magenta': (255, 0, 255),
        'white': (255, 255, 255), 'orange': (255, 165, 0), 'purple': (128, 0, 128),
        'pink': (255, 192, 203), 'off': (0, 0, 0),
    }

    if 'red' in params and 'green' in params and 'blue' in params:
        red, green, blue = params['red'], params['green'], params['blue']
    elif color in color_map:
        red, green, blue = color_map[color]
    else:
        red, green, blue = 255, 255, 255

    led_type = str(params.get('led_type', params.get('type', 'main'))).lower()
    if led_type not in ('main', 'front', 'back'):
        led_type = 'main'
    executor._send_led_command(red, green, blue, led_type)
    return True


def execute_roll(executor, task: TaskDescriptor) -> bool:
    """Roll at a heading + speed, optionally for a fixed duration."""
    params = task.parameters
    heading = params.get('heading', 0)
    speed = params.get('speed', 100)
    duration = params.get('duration', 0.0)

    if duration > 0:
        if 'start_time' not in params:
            executor._send_roll_command(heading, speed, duration)
            task.parameters['start_time'] = time.time()
            return False

        elapsed = time.time() - task.parameters['start_time']
        return elapsed >= duration
    else:
        # duration <= 0 => roll indefinitely (Sphero keeps moving until it is
        # told to stop). The task therefore stays running and continues to
        # occupy the DRIVE lane until cancelled. Emit the command once.
        if not params.get('_rolling'):
            executor._send_roll_command(heading, speed, duration)
            task.parameters['_rolling'] = True
        return False


def execute_heading(executor, task: TaskDescriptor) -> bool:
    """Set heading without moving."""
    heading = task.parameters.get('heading', 0)
    executor._send_heading_command(heading)
    return True


def execute_speed(executor, task: TaskDescriptor) -> bool:
    """Set speed without changing direction."""
    speed = task.parameters.get('speed', 0)
    executor._send_speed_command(speed)
    return True


def execute_matrix(executor, task: TaskDescriptor) -> bool:
    """Display one BOLT matrix pattern."""
    params = task.parameters
    pattern = params.get('pattern', 'smile')
    red = params.get('red', 255)
    green = params.get('green', 255)
    blue = params.get('blue', 255)

    executor._send_matrix_command(pattern=pattern, red=red, green=green, blue=blue)
    return True


def execute_collision(executor, task: TaskDescriptor) -> bool:
    """Configure collision detection."""
    params = task.parameters
    action = params.get('action', 'start')
    mode = params.get('mode', 'obstacle')
    sensitivity = params.get('sensitivity', 'HIGH')

    executor._send_collision_detection_command(action, mode, sensitivity)
    return True


def execute_reflect(executor, task: TaskDescriptor) -> bool:
    """Reverse heading with a random offset."""
    params = task.parameters
    offset_min = params.get('offset_min', -45)
    offset_max = params.get('offset_max', 45)
    speed = params.get('speed', 80)

    current_heading = executor.get_current_heading()
    reverse_heading = (current_heading + 180) % 360
    offset = random.randint(offset_min, offset_max)
    new_heading = (reverse_heading + offset) % 360

    executor._send_roll_command(new_heading, speed, duration=0.0)
    return True


def execute_jumping_bean(executor, task: TaskDescriptor) -> bool:
    """Disable stabilization and rapidly flip heading/speed for a duration."""
    params = task.parameters
    duration = params.get('duration', 10.0)
    flip_interval = params.get('flip_interval', 0.1)
    speed = params.get('speed', 200)

    executor._send_stabilization_command(False)

    num_flips = int(duration / flip_interval)
    current_heading = random.randint(0, 359)
    current_speed = speed
    flip_count = 0

    for _ in range(num_flips):
        executor._send_roll_command(current_heading, current_speed, flip_interval)
        flip_count += 1
        current_speed = -current_speed

        if flip_count % 10 == 0:
            current_heading = random.randint(0, 359)

        time.sleep(flip_interval)

    executor._send_stop_command()
    executor._send_stabilization_command(True)

    return True
