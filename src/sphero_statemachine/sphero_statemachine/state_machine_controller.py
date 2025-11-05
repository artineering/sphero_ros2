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
import importlib

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sphero_package.msg import SpheroSensor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from statemachine import StateMachine, State
from statemachine.exceptions import TransitionNotAllowed


class ConditionType(Enum):
    """Types of entry conditions for states."""
    ALWAYS = "always"
    TIMER = "timer"
    TOPIC_VALUE = "topic_value"
    CUSTOM = "custom"


class TransitionConditionType(Enum):
    """Types of transition conditions."""
    AUTO = "auto"  # Automatic based on state entry conditions
    TOPIC_VALUE = "topic_value"  # Based on a ROS topic value
    TOPIC_MESSAGE = "topic_message"  # Based on receiving any message on a topic
    TIMER = "timer"  # Time-based from source state


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

        # Dynamic topic monitoring for transitions
        self.topic_subscriptions: Dict[str, Any] = {}  # topic_name -> subscription
        self.topic_values: Dict[str, Any] = {}  # topic_name -> latest value
        self.topic_message_types: Dict[str, Any] = {}  # topic_name -> message type class
        self.topic_last_received: Dict[str, float] = {}  # topic_name -> timestamp

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
                    "trigger": "timer_based",
                    "condition": {
                        "type": "timer",
                        "duration": 5.0
                    }
                },
                {
                    "source": "moving",
                    "destination": "idle",
                    "trigger": "topic_trigger",
                    "condition": {
                        "type": "topic_value",
                        "topic": "/user_command",
                        "msg_type": "std_msgs/String",
                        "field_path": "data",
                        "operator": "==",
                        "value": "stop"
                    }
                },
                {
                    "source": "idle",
                    "destination": "alert",
                    "trigger": "message_received",
                    "condition": {
                        "type": "topic_message",
                        "topic": "/emergency",
                        "msg_type": "std_msgs/Bool",
                        "timeout": 1.0
                    }
                }
            ]
        }

        Transition Condition Types:
        - auto: Use destination state's entry condition (default, backward compatible)
        - timer: Transition after duration in source state
        - topic_value: Transition when topic value matches condition
        - topic_message: Transition when message received on topic
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

                # Validate transition condition if present
                if 'condition' in trans:
                    if not self.validate_transition_condition(trans['condition']):
                        self.get_logger().error(f'Invalid transition condition: {trans}')
                        return False

        return True

    def validate_transition_condition(self, condition: Dict[str, Any]) -> bool:
        """
        Validate a transition condition configuration.

        Args:
            condition: Condition configuration dictionary

        Returns:
            True if valid, False otherwise
        """
        condition_type = condition.get('type', 'auto')

        # Validate based on condition type
        if condition_type in ['topic_value', 'topic_message']:
            # These require topic and msg_type
            if 'topic' not in condition:
                self.get_logger().error(f'Condition type {condition_type} requires "topic" parameter')
                return False
            if 'msg_type' not in condition:
                self.get_logger().error(f'Condition type {condition_type} requires "msg_type" parameter')
                return False

            # topic_value also requires operator and value
            if condition_type == 'topic_value':
                if 'operator' not in condition:
                    self.get_logger().error('Condition type topic_value requires "operator" parameter')
                    return False
                if 'value' not in condition:
                    self.get_logger().error('Condition type topic_value requires "value" parameter')
                    return False

                # Validate operator
                valid_operators = ['==', '!=', '>', '<', '>=', '<=']
                if condition['operator'] not in valid_operators:
                    self.get_logger().error(
                        f'Invalid operator "{condition["operator"]}". Must be one of: {valid_operators}'
                    )
                    return False

        elif condition_type == 'timer':
            # Timer condition requires duration
            if 'duration' not in condition:
                self.get_logger().error('Condition type timer requires "duration" parameter')
                return False

        elif condition_type != 'auto':
            self.get_logger().warning(f'Unknown condition type: {condition_type}')
            return False

        return True

    def build_state_machine(self, config: Dict[str, Any]):
        """Build the state machine from configuration."""
        self.sm_config = config
        self.sm_states = {}
        self.sm_transitions = config.get('transitions', [])

        # Clean up old topic subscriptions
        for topic_name in list(self.topic_subscriptions.keys()):
            self.unsubscribe_from_topic(topic_name)

        # Create state objects
        for state_config in config['states']:
            state_name = state_config['name']
            self.sm_states[state_name] = DynamicState(state_name, state_config)
            self.get_logger().info(f'Created state: {state_name}')

        # Set up topic subscriptions for transition conditions
        for transition in self.sm_transitions:
            condition = transition.get('condition', {})
            condition_type = condition.get('type', 'auto')

            if condition_type in ['topic_value', 'topic_message']:
                topic_name = condition.get('topic')
                msg_type = condition.get('msg_type')
                field_path = condition.get('field_path')

                if topic_name and msg_type:
                    self.subscribe_to_topic(topic_name, msg_type, field_path)
                else:
                    self.get_logger().warning(
                        f'Transition condition {condition_type} missing required parameters: topic and msg_type'
                    )

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

    def subscribe_to_topic(self, topic_name: str, msg_type: str, field_path: Optional[str] = None):
        """
        Dynamically subscribe to a ROS topic for transition monitoring.

        Args:
            topic_name: The topic to subscribe to
            msg_type: Message type as string (e.g., 'std_msgs/String', 'geometry_msgs/Twist')
            field_path: Optional dot-notation path to extract specific field (e.g., 'data', 'linear.x')
        """
        if topic_name in self.topic_subscriptions:
            self.get_logger().info(f'Already subscribed to topic: {topic_name}')
            return

        try:
            # Parse message type string (e.g., 'std_msgs/String' -> std_msgs.msg.String)
            parts = msg_type.split('/')
            if len(parts) != 2:
                self.get_logger().error(f'Invalid message type format: {msg_type}. Expected "package/Type"')
                return

            package_name, type_name = parts
            module_path = f'{package_name}.msg'

            # Import the message type
            try:
                msg_module = importlib.import_module(module_path)
                msg_class = getattr(msg_module, type_name)
            except (ImportError, AttributeError) as e:
                self.get_logger().error(f'Failed to import message type {msg_type}: {e}')
                return

            # Store message type information
            self.topic_message_types[topic_name] = {
                'class': msg_class,
                'field_path': field_path
            }

            # Create callback that stores the received message
            def topic_callback(msg):
                self.topic_last_received[topic_name] = time.time()

                # Extract specific field if specified
                if field_path:
                    try:
                        value = msg
                        for field in field_path.split('.'):
                            value = getattr(value, field)
                        self.topic_values[topic_name] = value
                    except AttributeError as e:
                        self.get_logger().error(f'Failed to extract field {field_path} from {topic_name}: {e}')
                        self.topic_values[topic_name] = msg
                else:
                    # Store the entire message
                    self.topic_values[topic_name] = msg

                self.get_logger().debug(f'Received message on {topic_name}: {self.topic_values[topic_name]}')

            # Create subscription with default QoS
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )

            subscription = self.create_subscription(
                msg_class,
                topic_name,
                topic_callback,
                qos
            )

            self.topic_subscriptions[topic_name] = subscription
            self.get_logger().info(f'Subscribed to topic: {topic_name} (type: {msg_type})')

        except Exception as e:
            self.get_logger().error(f'Failed to subscribe to topic {topic_name}: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def unsubscribe_from_topic(self, topic_name: str):
        """
        Unsubscribe from a dynamically created topic subscription.

        Args:
            topic_name: The topic to unsubscribe from
        """
        if topic_name in self.topic_subscriptions:
            self.destroy_subscription(self.topic_subscriptions[topic_name])
            del self.topic_subscriptions[topic_name]
            if topic_name in self.topic_values:
                del self.topic_values[topic_name]
            if topic_name in self.topic_message_types:
                del self.topic_message_types[topic_name]
            if topic_name in self.topic_last_received:
                del self.topic_last_received[topic_name]
            self.get_logger().info(f'Unsubscribed from topic: {topic_name}')

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

                # Check if this transition's condition is met
                if self.check_transition_condition(transition):
                    trigger = transition.get('trigger', 'auto')
                    self.get_logger().info(f'Transition triggered: {self.sm_current_state} -> {dest_state} (trigger: {trigger})')
                    self.transition_to_state(dest_state)
                    break

    def check_transition_condition(self, transition: Dict[str, Any]) -> bool:
        """
        Check if a transition's condition is satisfied.

        Args:
            transition: Transition configuration dictionary

        Returns:
            True if condition is met, False otherwise
        """
        condition = transition.get('condition', {})
        condition_type = condition.get('type', 'auto')
        dest_state = transition['destination']

        # AUTO: Check destination state's entry condition (original behavior)
        if condition_type == 'auto' or condition_type == TransitionConditionType.AUTO.value:
            return self.check_entry_condition(dest_state)

        # TIMER: Time-based from current state
        elif condition_type == 'timer' or condition_type == TransitionConditionType.TIMER.value:
            if self.sm_current_state is None:
                return False

            current = self.sm_states[self.sm_current_state]
            if current.entry_time is None:
                return False

            duration = condition.get('duration', 0.0)
            elapsed = time.time() - current.entry_time
            result = elapsed >= duration

            if result:
                self.get_logger().debug(
                    f'Timer transition condition met: {self.sm_current_state} -> {dest_state} '
                    f'(elapsed={elapsed:.2f}s, duration={duration}s)'
                )
            return result

        # TOPIC_VALUE: Check if topic value meets criteria
        elif condition_type == 'topic_value' or condition_type == TransitionConditionType.TOPIC_VALUE.value:
            topic_name = condition.get('topic')
            operator = condition.get('operator', '==')
            expected_value = condition.get('value')

            if not topic_name:
                self.get_logger().warning('TOPIC_VALUE condition missing topic name')
                return False

            if topic_name not in self.topic_values:
                return False

            actual_value = self.topic_values[topic_name]

            try:
                if operator == '==':
                    result = actual_value == expected_value
                elif operator == '!=':
                    result = actual_value != expected_value
                elif operator == '>':
                    result = actual_value > expected_value
                elif operator == '<':
                    result = actual_value < expected_value
                elif operator == '>=':
                    result = actual_value >= expected_value
                elif operator == '<=':
                    result = actual_value <= expected_value
                else:
                    self.get_logger().warning(f'Unknown operator: {operator}')
                    return False

                if result:
                    self.get_logger().debug(
                        f'Topic value condition met: {topic_name} {operator} {expected_value} '
                        f'(actual={actual_value})'
                    )
                return result

            except Exception as e:
                self.get_logger().error(f'Error comparing topic values: {e}')
                return False

        # TOPIC_MESSAGE: Check if message was received recently
        elif condition_type == 'topic_message' or condition_type == TransitionConditionType.TOPIC_MESSAGE.value:
            topic_name = condition.get('topic')
            timeout = condition.get('timeout', None)  # Optional: message must be received within timeout

            if not topic_name:
                self.get_logger().warning('TOPIC_MESSAGE condition missing topic name')
                return False

            if topic_name not in self.topic_last_received:
                return False

            # If timeout specified, check if message was received recently enough
            if timeout is not None:
                elapsed = time.time() - self.topic_last_received[topic_name]
                result = elapsed <= timeout
                if result:
                    self.get_logger().debug(
                        f'Topic message condition met: {topic_name} received within {timeout}s '
                        f'(elapsed={elapsed:.2f}s)'
                    )
                return result
            else:
                # No timeout, just check if we ever received a message
                self.get_logger().debug(f'Topic message condition met: {topic_name} received')
                return True

        else:
            self.get_logger().warning(f'Unknown transition condition type: {condition_type}')
            return False

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
