#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dynamic State Machine Controller for Sphero Robot.

This node creates a configurable state machine that can be dynamically configured
via ROS2 topics. Each state has entry conditions and associated tasks.
"""

import json
import time
from typing import Dict, Any, List, Optional, Callable
from enum import Enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sphero_package.msg import SpheroSensor

from statemachine import StateMachine, State
from statemachine.exceptions import TransitionNotAllowed


class ConditionType(Enum):
    """Types of entry conditions for states."""
    ALWAYS = "always"
    TIMER = "timer"
    TOPIC_VALUE = "topic_value"
    CUSTOM = "custom"


class DynamicState:
    """
    Represents a dynamically configured state.

    Attributes:
        name: Unique identifier for the state
        entry_condition: Condition that must be met before entering state
        entry_condition_params: Parameters for the entry condition
        tasks: List of tasks to execute when state is active
    """

    def __init__(self, name: str, config: Dict[str, Any]):
        self.name = name
        self.config = config

        # Entry condition configuration
        self.entry_condition_type = config.get('entry_condition', {}).get('type', 'always')
        self.entry_condition_params = config.get('entry_condition', {}).get('params', {})

        # Task configuration - support both single task (backward compatibility) and task array
        if 'tasks' in config:
            # New format: array of tasks
            self.tasks = config['tasks']
        elif 'task' in config:
            # Old format: single task - convert to array
            self.tasks = [config['task']]
        else:
            # No tasks
            self.tasks = []

        # State metadata
        self.description = config.get('description', '')
        self.timeout = config.get('timeout', None)

        # Runtime data
        self.entry_time = None
        self.condition_met = False
        self.task_completed = False


class DynamicStateMachineController(Node):
    """
    ROS2 node that manages a dynamically configurable state machine.

    The state machine can be configured via the '/state_machine/config' topic.
    Each state has:
    - Entry conditions that must be satisfied
    - Associated tasks to execute
    - Transitions to other states
    """

    def __init__(self):
        """Initialize the dynamic state machine controller."""
        super().__init__('state_machine_controller')

        # State machine configuration
        self.sm_config: Optional[Dict[str, Any]] = None
        self.sm_states: Dict[str, DynamicState] = {}
        self.sm_transitions: List[Dict[str, Any]] = []
        self.sm_current_state: Optional[str] = None
        self.state_machine: Optional[StateMachine] = None

        # Condition monitoring data
        self.sensor_topic_values: Dict[str, Any] = {}
        self.state_timers: Dict[str, float] = {}

        # Subscribers for configuration and sensor data
        self.config_sub = self.create_subscription(
            String,
            '/state_machine/config',
            self.config_callback,
            10
        )

        # Subscriber for Sphero sensor data used in conditions
        self.sensor_sub = self.create_subscription(
            SpheroSensor,
            '/sphero/sensors',
            self.sensor_data_callback,
            10
        )

        # Publishers for state machine status and events
        self.status_pub = self.create_publisher(
            String,
            '/state_machine/status',
            10
        )

        self.event_pub = self.create_publisher(
            String,
            '/state_machine/events',
            10
        )

        self.task_pub = self.create_publisher(
            String,
            '/state_machine/task_command',
            10
        )

        # Timer for periodic state machine updates
        self.update_timer = self.create_timer(0.1, self.update_callback)

        self.get_logger().info('Dynamic State Machine Controller initialized')
        self.get_logger().info('Subscribed to:')
        self.get_logger().info('  - /state_machine/config (configuration)')
        self.get_logger().info('  - /sphero/sensors (sensor data for conditions)')
        self.get_logger().info('Waiting for configuration on /state_machine/config')

        # Publish initial status
        self.publish_status()

    def config_callback(self, msg: String):
        """
        Handle incoming state machine configuration.

        Expected JSON format:
        {
            "name": "my_state_machine",
            "initial_state": "idle",
            "states": [
                {
                    "name": "idle",
                    "description": "Waiting state",
                    "entry_condition": {
                        "type": "always",
                        "params": {}
                    },
                    "task": {
                        "type": "set_led",
                        "params": {"color": "blue"}
                    }
                },
                {
                    "name": "moving",
                    "description": "Moving state",
                    "entry_condition": {
                        "type": "timer",
                        "params": {"duration": 5.0}
                    },
                    "task": {
                        "type": "roll",
                        "params": {"heading": 0, "speed": 100}
                    },
                    "timeout": 10.0
                }
            ],
            "transitions": [
                {
                    "source": "idle",
                    "destination": "moving",
                    "trigger": "start_moving"
                },
                {
                    "source": "moving",
                    "destination": "idle",
                    "trigger": "stop_moving"
                }
            ]
        }
        """
        try:
            config = json.loads(msg.data)
            self.get_logger().info(f'Received state machine configuration: {config.get("name", "unnamed")}')

            # Validate configuration
            if not self.validate_config(config):
                self.get_logger().error('Invalid configuration received')
                return

            # Build the state machine
            self.build_state_machine(config)

            # Publish configuration success event
            self.publish_event('configuration_loaded', {
                'name': config.get('name', 'unnamed'),
                'num_states': len(self.sm_states),
                'num_transitions': len(self.sm_transitions)
            })

            self.get_logger().info(f'State machine configured with {len(self.sm_states)} states')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse configuration JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing configuration: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def validate_config(self, config: Dict[str, Any]) -> bool:
        """Validate the state machine configuration."""
        if 'states' not in config or not config['states']:
            self.get_logger().error('Configuration must have at least one state')
            return False

        if 'initial_state' not in config:
            self.get_logger().error('Configuration must specify initial_state')
            return False

        # Check that initial state exists
        state_names = [s['name'] for s in config['states']]
        if config['initial_state'] not in state_names:
            self.get_logger().error(f'Initial state "{config["initial_state"]}" not found in states')
            return False

        # Validate transitions reference existing states
        if 'transitions' in config:
            for trans in config['transitions']:
                if trans['source'] not in state_names:
                    self.get_logger().error(f'Transition source "{trans["source"]}" not found')
                    return False
                if trans['destination'] not in state_names:
                    self.get_logger().error(f'Transition destination "{trans["destination"]}" not found')
                    return False

        return True

    def build_state_machine(self, config: Dict[str, Any]):
        """Build the state machine from configuration."""
        self.sm_config = config
        self.sm_states = {}
        self.sm_transitions = config.get('transitions', [])

        # Create state objects
        for state_config in config['states']:
            state_name = state_config['name']
            self.sm_states[state_name] = DynamicState(state_name, state_config)
            self.get_logger().info(f'Created state: {state_name}')

        # Set initial state
        self.sm_current_state = config['initial_state']
        self.sm_states[self.sm_current_state].entry_time = time.time()

        self.get_logger().info(f'State machine ready. Initial state: {self.sm_current_state}')

        # Execute initial state task
        self.execute_state_task(self.sm_current_state)

    def sensor_data_callback(self, msg: SpheroSensor):
        """
        Handle incoming Sphero sensor data for condition evaluation.

        Converts SpheroSensor message to a dictionary for easy condition checking.
        Available sensor keys:
        - orientation: pitch, roll, yaw
        - accelerometer: accel_x, accel_y, accel_z
        - gyroscope: gyro_x, gyro_y, gyro_z
        - position: x, y
        - velocity: velocity_x, velocity_y
        - battery: battery_percentage
        """
        try:
            # Convert SpheroSensor message to dictionary for condition evaluation
            self.sensor_topic_values = {
                'pitch': msg.pitch,
                'roll': msg.roll,
                'yaw': msg.yaw,
                'accel_x': msg.accel_x,
                'accel_y': msg.accel_y,
                'accel_z': msg.accel_z,
                'gyro_x': msg.gyro_x,
                'gyro_y': msg.gyro_y,
                'gyro_z': msg.gyro_z,
                'x': msg.x,
                'y': msg.y,
                'velocity_x': msg.velocity_x,
                'velocity_y': msg.velocity_y,
                'battery_percentage': msg.battery_percentage
            }
        except Exception as e:
            self.get_logger().error(f'Failed to process sensor data: {e}')

    def update_callback(self):
        """Periodic update to check conditions and transitions."""
        if self.sm_current_state is None or not self.sm_states:
            return

        # Check if current state has timed out
        current = self.sm_states[self.sm_current_state]
        if current.timeout is not None and current.entry_time is not None:
            elapsed = time.time() - current.entry_time
            if elapsed > current.timeout:
                self.get_logger().warning(f'State {self.sm_current_state} timed out after {elapsed:.1f}s')
                self.publish_event('state_timeout', {
                    'state': self.sm_current_state,
                    'elapsed': elapsed
                })

        # Check for available transitions
        self.check_transitions()

        # Publish status periodically (every second)
        if int(time.time() * 10) % 10 == 0:
            self.publish_status()

    def check_transitions(self):
        """Check if any transitions should be triggered."""
        for transition in self.sm_transitions:
            if transition['source'] == self.sm_current_state:
                dest_state = transition['destination']

                # Check if destination state's entry condition is met
                if self.check_entry_condition(dest_state):
                    trigger = transition.get('trigger', 'auto')
                    self.get_logger().info(f'Transition triggered: {self.sm_current_state} -> {dest_state} (trigger: {trigger})')
                    self.transition_to_state(dest_state)
                    break

    def check_entry_condition(self, state_name: str, is_initial: bool = False) -> bool:
        """
        Check if a state's entry condition is satisfied.

        Args:
            state_name: Name of the state to check
            is_initial: True if this is the initial state (ignore entry conditions)

        Returns:
            True if condition is met, False otherwise
        """
        if state_name not in self.sm_states:
            return False

        # Initial state can always be entered
        if is_initial:
            return True

        state = self.sm_states[state_name]
        condition_type = state.entry_condition_type
        params = state.entry_condition_params

        if condition_type == 'always' or condition_type == ConditionType.ALWAYS.value:
            return True

        elif condition_type == 'timer' or condition_type == ConditionType.TIMER.value:
            # Check if enough time has passed in current state
            if self.sm_current_state is None:
                self.get_logger().debug(f'Timer condition for {state_name}: No current state')
                return False

            current = self.sm_states[self.sm_current_state]
            if current.entry_time is None:
                self.get_logger().debug(f'Timer condition for {state_name}: No entry time')
                return False

            duration = params.get('duration', 0.0)
            elapsed = time.time() - current.entry_time
            result = elapsed >= duration
            self.get_logger().debug(f'Timer condition for {state_name}: elapsed={elapsed:.2f}s, duration={duration}s, met={result}')
            return result

        elif condition_type == 'topic_value' or condition_type == ConditionType.TOPIC_VALUE.value:
            # Check if a specific topic value meets criteria
            key = params.get('key', '')
            operator = params.get('operator', '==')
            value = params.get('value')

            if key not in self.sensor_topic_values:
                return False

            actual_value = self.sensor_topic_values[key]

            if operator == '==':
                return actual_value == value
            elif operator == '!=':
                return actual_value != value
            elif operator == '>':
                return actual_value > value
            elif operator == '<':
                return actual_value < value
            elif operator == '>=':
                return actual_value >= value
            elif operator == '<=':
                return actual_value <= value
            else:
                self.get_logger().warning(f'Unknown operator: {operator}')
                return False

        else:
            self.get_logger().warning(f'Unknown condition type: {condition_type}')
            return False

    def transition_to_state(self, new_state: str):
        """
        Transition to a new state.

        Args:
            new_state: Name of the state to transition to
        """
        if new_state not in self.sm_states:
            self.get_logger().error(f'Cannot transition to unknown state: {new_state}')
            return

        old_state = self.sm_current_state

        # Update state
        self.sm_current_state = new_state
        self.sm_states[new_state].entry_time = time.time()
        self.sm_states[new_state].condition_met = True
        self.sm_states[new_state].task_completed = False

        # Publish transition event
        self.publish_event('state_transition', {
            'from': old_state,
            'to': new_state,
            'timestamp': time.time()
        })

        self.get_logger().info(f'Transitioned from {old_state} to {new_state}')

        # Execute new state's task
        self.execute_state_task(new_state)

        # Publish updated status
        self.publish_status()

    def execute_state_task(self, state_name: str):
        """
        Execute all tasks associated with a state.

        Args:
            state_name: Name of the state whose tasks to execute
        """
        if state_name not in self.sm_states:
            return

        state = self.sm_states[state_name]

        if not state.tasks:
            self.get_logger().info(f'No tasks to execute for state {state_name}')
            state.task_completed = True
            return

        self.get_logger().info(f'Executing {len(state.tasks)} task(s) for state {state_name}')

        # Execute each task in the array
        for idx, task in enumerate(state.tasks):
            task_type = task.get('type', 'none')
            task_params = task.get('params', {})

            self.get_logger().info(f'  Task {idx + 1}/{len(state.tasks)}: {task_type}')

            # Publish task command
            task_command = {
                'state': state_name,
                'task_type': task_type,
                'params': task_params,
                'task_index': idx,
                'total_tasks': len(state.tasks),
                'timestamp': time.time()
            }

            msg = String()
            msg.data = json.dumps(task_command)
            self.task_pub.publish(msg)

            # Publish task execution event
            self.publish_event('task_executed', task_command)

        state.task_completed = True

    def publish_status(self):
        """Publish current state machine status."""
        if self.sm_current_state is None:
            status = {
                'configured': False,
                'current_state': None,
                'timestamp': time.time()
            }
        else:
            current = self.sm_states[self.sm_current_state]
            elapsed = time.time() - current.entry_time if current.entry_time else 0

            status = {
                'configured': True,
                'name': self.sm_config.get('name', 'unnamed') if self.sm_config else 'unnamed',
                'current_state': self.sm_current_state,
                'state_description': current.description,
                'time_in_state': elapsed,
                'condition_met': current.condition_met,
                'task_completed': current.task_completed,
                'num_states': len(self.sm_states),
                'num_transitions': len(self.sm_transitions),
                'timestamp': time.time()
            }

        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

    def publish_event(self, event_type: str, data: Dict[str, Any]):
        """
        Publish a state machine event.

        Args:
            event_type: Type of event
            data: Event data
        """
        event = {
            'event_type': event_type,
            'data': data,
            'timestamp': time.time()
        }

        msg = String()
        msg.data = json.dumps(event)
        self.event_pub.publish(msg)

        self.get_logger().info(f'Event: {event_type} - {data}')


def main(args=None):
    """Main entry point for the state machine controller."""
    rclpy.init(args=args)

    node = DynamicStateMachineController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down state machine controller...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
