#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
State Machine Core Logic for Sphero Robot.

This module provides a configurable state machine that can be dynamically configured.
Each state has entry conditions and associated tasks.
"""

import time
import importlib
from typing import Dict, Any, List, Optional, Callable
from enum import Enum
from dataclasses import dataclass, field


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


@dataclass
class DynamicState:
    """
    Represents a dynamically configured state.

    Attributes:
        name: Unique identifier for the state
        config: Full configuration dictionary
        entry_condition_type: Type of condition to enter state
        entry_condition_params: Parameters for the entry condition
        tasks: List of tasks to execute when state is active
        description: Human-readable description
        timeout: Optional timeout in seconds
    """
    name: str
    config: Dict[str, Any]
    entry_condition_type: str = 'always'
    entry_condition_params: Dict[str, Any] = field(default_factory=dict)
    tasks: List[Dict[str, Any]] = field(default_factory=list)
    description: str = ''
    timeout: Optional[float] = None

    # Runtime data
    entry_time: Optional[float] = None
    condition_met: bool = False
    task_completed: bool = False

    def __post_init__(self):
        """Initialize state from config."""
        # Entry condition configuration
        self.entry_condition_type = self.config.get('entry_condition', {}).get('type', 'always')
        self.entry_condition_params = self.config.get('entry_condition', {}).get('params', {})

        # Task configuration - support both single task (backward compatibility) and task array
        if 'tasks' in self.config:
            # New format: array of tasks
            self.tasks = self.config['tasks']
        elif 'task' in self.config:
            # Old format: single task - convert to array
            self.tasks = [self.config['task']]
        else:
            # No tasks
            self.tasks = []

        # State metadata
        self.description = self.config.get('description', '')
        self.timeout = self.config.get('timeout', None)


class StateMachine:
    """
    Core state machine logic for Sphero robot.

    Manages states, transitions, and condition evaluation.
    This class is ROS-independent and can be used standalone.
    """

    def __init__(
        self,
        logger: Optional[Callable] = None,
        topic_subscribe_callback: Optional[Callable] = None,
        topic_unsubscribe_callback: Optional[Callable] = None
    ):
        """
        Initialize the state machine.

        Args:
            logger: Optional logging function (e.g., node.get_logger().info)
            topic_subscribe_callback: Callback to subscribe to topics (topic_name, msg_type, field_path)
            topic_unsubscribe_callback: Callback to unsubscribe from topics (topic_name)
        """
        self.logger = logger or self._default_logger
        self.topic_subscribe_callback = topic_subscribe_callback
        self.topic_unsubscribe_callback = topic_unsubscribe_callback

        # State machine configuration
        self.config: Optional[Dict[str, Any]] = None
        self.states: Dict[str, DynamicState] = {}
        self.transitions: List[Dict[str, Any]] = []
        self.current_state: Optional[str] = None

        # Condition monitoring data
        self.sensor_topic_values: Dict[str, Any] = {}
        self.state_timers: Dict[str, float] = {}

        # Dynamic topic monitoring for transitions
        self.topic_values: Dict[str, Any] = {}
        self.topic_last_received: Dict[str, float] = {}

    def _default_logger(self, message: str):
        """Default logger that prints to console."""
        print(f"[StateMachine] {message}")

    def configure(self, config: Dict[str, Any]) -> bool:
        """
        Configure the state machine from a configuration dictionary.

        Args:
            config: State machine configuration

        Returns:
            True if configuration successful, False otherwise
        """
        self.logger(f'Configuring state machine: {config.get("name", "unnamed")}')

        # Validate configuration
        if not self.validate_config(config):
            self.logger('ERROR: Invalid configuration')
            return False

        # Build the state machine
        self._build_state_machine(config)

        self.logger(f'State machine configured with {len(self.states)} states')
        return True

    def validate_config(self, config: Dict[str, Any]) -> bool:
        """
        Validate the state machine configuration.

        Args:
            config: Configuration dictionary

        Returns:
            True if valid, False otherwise
        """
        if 'states' not in config or not config['states']:
            self.logger('ERROR: Configuration must have at least one state')
            return False

        if 'initial_state' not in config:
            self.logger('ERROR: Configuration must specify initial_state')
            return False

        # Check that initial state exists
        state_names = [s['name'] for s in config['states']]
        if config['initial_state'] not in state_names:
            self.logger(f'ERROR: Initial state "{config["initial_state"]}" not found in states')
            return False

        # Validate transitions reference existing states
        if 'transitions' in config:
            for trans in config['transitions']:
                if trans['source'] not in state_names:
                    self.logger(f'ERROR: Transition source "{trans["source"]}" not found')
                    return False
                if trans['destination'] not in state_names:
                    self.logger(f'ERROR: Transition destination "{trans["destination"]}" not found')
                    return False

                # Validate transition condition if present
                if 'condition' in trans:
                    if not self._validate_transition_condition(trans['condition']):
                        self.logger(f'ERROR: Invalid transition condition: {trans}')
                        return False

        return True

    def _validate_transition_condition(self, condition: Dict[str, Any]) -> bool:
        """Validate a transition condition configuration."""
        condition_type = condition.get('type', 'auto')

        # Validate based on condition type
        if condition_type in ['topic_value', 'topic_message']:
            # These require topic and msg_type
            if 'topic' not in condition:
                self.logger(f'ERROR: Condition type {condition_type} requires "topic" parameter')
                return False
            if 'msg_type' not in condition:
                self.logger(f'ERROR: Condition type {condition_type} requires "msg_type" parameter')
                return False

            # topic_value also requires operator and value
            if condition_type == 'topic_value':
                if 'operator' not in condition:
                    self.logger('ERROR: Condition type topic_value requires "operator" parameter')
                    return False
                if 'value' not in condition:
                    self.logger('ERROR: Condition type topic_value requires "value" parameter')
                    return False

                # Validate operator
                valid_operators = ['==', '!=', '>', '<', '>=', '<=']
                if condition['operator'] not in valid_operators:
                    self.logger(f'ERROR: Invalid operator "{condition["operator"]}". Must be one of: {valid_operators}')
                    return False

        elif condition_type == 'timer':
            # Timer condition requires duration
            if 'duration' not in condition:
                self.logger('ERROR: Condition type timer requires "duration" parameter')
                return False

        elif condition_type != 'auto':
            self.logger(f'WARNING: Unknown condition type: {condition_type}')
            return False

        return True

    def _build_state_machine(self, config: Dict[str, Any]):
        """Build the state machine from configuration."""
        self.config = config
        self.states = {}
        self.transitions = config.get('transitions', [])

        # Clean up old topic subscriptions
        if self.topic_unsubscribe_callback:
            for topic_name in list(self.topic_values.keys()):
                self.topic_unsubscribe_callback(topic_name)

        self.topic_values.clear()
        self.topic_last_received.clear()

        # Create state objects
        for state_config in config['states']:
            state_name = state_config['name']
            self.states[state_name] = DynamicState(state_name, state_config)
            self.logger(f'Created state: {state_name}')

        # Set up topic subscriptions for transition conditions
        if self.topic_subscribe_callback:
            for transition in self.transitions:
                condition = transition.get('condition', {})
                condition_type = condition.get('type', 'auto')

                if condition_type in ['topic_value', 'topic_message']:
                    topic_name = condition.get('topic')
                    msg_type = condition.get('msg_type')
                    field_path = condition.get('field_path')

                    if topic_name and msg_type:
                        self.topic_subscribe_callback(topic_name, msg_type, field_path)
                    else:
                        self.logger(f'WARNING: Transition condition {condition_type} missing required parameters')

        # Set initial state
        self.current_state = config['initial_state']
        self.states[self.current_state].entry_time = time.time()

        self.logger(f'State machine ready. Initial state: {self.current_state}')

    def update_sensor_data(self, sensor_data: Dict[str, Any]):
        """
        Update sensor data for condition evaluation.

        Args:
            sensor_data: Dictionary of sensor values
        """
        self.sensor_topic_values.update(sensor_data)

    def update_topic_value(self, topic_name: str, value: Any):
        """
        Update a topic value for transition conditions.

        Args:
            topic_name: Name of the topic
            value: Latest value from the topic
        """
        self.topic_values[topic_name] = value
        self.topic_last_received[topic_name] = time.time()

    def process(self) -> Optional[Dict[str, Any]]:
        """
        Process state machine logic (check conditions, transitions, timeouts).

        Returns:
            Dictionary with status and any events, or None if not configured
        """
        if self.current_state is None or not self.states:
            return None

        events = []

        # Check if current state has timed out
        current = self.states[self.current_state]
        if current.timeout is not None and current.entry_time is not None:
            elapsed = time.time() - current.entry_time
            if elapsed > current.timeout:
                events.append({
                    'type': 'state_timeout',
                    'state': self.current_state,
                    'elapsed': elapsed
                })

        # Check for available transitions
        transition_event = self._check_transitions()
        if transition_event:
            events.append(transition_event)

        return {
            'current_state': self.current_state,
            'events': events,
            'time_in_state': time.time() - current.entry_time if current.entry_time else 0
        }

    def _check_transitions(self) -> Optional[Dict[str, Any]]:
        """Check if any transitions should be triggered."""
        for transition in self.transitions:
            if transition['source'] == self.current_state:
                dest_state = transition['destination']

                # Check if this transition's condition is met
                if self._check_transition_condition(transition):
                    trigger = transition.get('trigger', 'auto')
                    self.logger(f'Transition triggered: {self.current_state} -> {dest_state} (trigger: {trigger})')

                    # Perform transition
                    old_state = self.current_state
                    self.transition_to_state(dest_state)

                    return {
                        'type': 'state_transition',
                        'from': old_state,
                        'to': dest_state,
                        'timestamp': time.time()
                    }

        return None

    def _check_transition_condition(self, transition: Dict[str, Any]) -> bool:
        """Check if a transition's condition is satisfied."""
        condition = transition.get('condition', {})
        condition_type = condition.get('type', 'auto')
        dest_state = transition['destination']

        # AUTO: Check destination state's entry condition (original behavior)
        if condition_type == 'auto' or condition_type == TransitionConditionType.AUTO.value:
            return self._check_entry_condition(dest_state)

        # TIMER: Time-based from current state
        elif condition_type == 'timer' or condition_type == TransitionConditionType.TIMER.value:
            if self.current_state is None:
                return False

            current = self.states[self.current_state]
            if current.entry_time is None:
                return False

            duration = condition.get('duration', 0.0)
            elapsed = time.time() - current.entry_time
            return elapsed >= duration

        # TOPIC_VALUE: Check if topic value meets criteria
        elif condition_type == 'topic_value' or condition_type == TransitionConditionType.TOPIC_VALUE.value:
            topic_name = condition.get('topic')
            operator = condition.get('operator', '==')
            expected_value = condition.get('value')

            if not topic_name or topic_name not in self.topic_values:
                return False

            actual_value = self.topic_values[topic_name]

            try:
                return self._compare_values(actual_value, operator, expected_value)
            except Exception as e:
                self.logger(f'ERROR: Error comparing topic values: {e}')
                return False

        # TOPIC_MESSAGE: Check if message was received recently
        elif condition_type == 'topic_message' or condition_type == TransitionConditionType.TOPIC_MESSAGE.value:
            topic_name = condition.get('topic')
            timeout = condition.get('timeout', None)

            if not topic_name or topic_name not in self.topic_last_received:
                return False

            # If timeout specified, check if message was received recently enough
            if timeout is not None:
                elapsed = time.time() - self.topic_last_received[topic_name]
                return elapsed <= timeout
            else:
                # No timeout, just check if we ever received a message
                return True

        else:
            self.logger(f'WARNING: Unknown transition condition type: {condition_type}')
            return False

    def _check_entry_condition(self, state_name: str, is_initial: bool = False) -> bool:
        """Check if a state's entry condition is satisfied."""
        if state_name not in self.states:
            return False

        # Initial state can always be entered
        if is_initial:
            return True

        state = self.states[state_name]
        condition_type = state.entry_condition_type
        params = state.entry_condition_params

        if condition_type == 'always' or condition_type == ConditionType.ALWAYS.value:
            return True

        elif condition_type == 'timer' or condition_type == ConditionType.TIMER.value:
            # Check if enough time has passed in current state
            if self.current_state is None:
                return False

            current = self.states[self.current_state]
            if current.entry_time is None:
                return False

            duration = params.get('duration', 0.0)
            elapsed = time.time() - current.entry_time
            return elapsed >= duration

        elif condition_type == 'topic_value' or condition_type == ConditionType.TOPIC_VALUE.value:
            # Check if a specific topic value meets criteria
            key = params.get('key', '')
            operator = params.get('operator', '==')
            value = params.get('value')

            if key not in self.sensor_topic_values:
                return False

            actual_value = self.sensor_topic_values[key]
            return self._compare_values(actual_value, operator, value)

        else:
            self.logger(f'WARNING: Unknown condition type: {condition_type}')
            return False

    def _compare_values(self, actual: Any, operator: str, expected: Any) -> bool:
        """Compare two values using the given operator."""
        if operator == '==':
            return actual == expected
        elif operator == '!=':
            return actual != expected
        elif operator == '>':
            return actual > expected
        elif operator == '<':
            return actual < expected
        elif operator == '>=':
            return actual >= expected
        elif operator == '<=':
            return actual <= expected
        else:
            self.logger(f'WARNING: Unknown operator: {operator}')
            return False

    def transition_to_state(self, new_state: str) -> bool:
        """
        Transition to a new state.

        Args:
            new_state: Name of the state to transition to

        Returns:
            True if transition successful, False otherwise
        """
        if new_state not in self.states:
            self.logger(f'ERROR: Cannot transition to unknown state: {new_state}')
            return False

        old_state = self.current_state

        # Update state
        self.current_state = new_state
        self.states[new_state].entry_time = time.time()
        self.states[new_state].condition_met = True
        self.states[new_state].task_completed = False

        self.logger(f'Transitioned from {old_state} to {new_state}')
        return True

    def get_current_state_tasks(self) -> List[Dict[str, Any]]:
        """
        Get tasks for the current state.

        Returns:
            List of task dictionaries
        """
        if self.current_state and self.current_state in self.states:
            return self.states[self.current_state].tasks
        return []

    def mark_tasks_completed(self):
        """Mark current state's tasks as completed."""
        if self.current_state and self.current_state in self.states:
            self.states[self.current_state].task_completed = True

    def get_status(self) -> Dict[str, Any]:
        """
        Get current state machine status.

        Returns:
            Dictionary with status information
        """
        if self.current_state is None:
            return {
                'configured': False,
                'current_state': None,
                'timestamp': time.time()
            }

        current = self.states[self.current_state]
        elapsed = time.time() - current.entry_time if current.entry_time else 0

        return {
            'configured': True,
            'name': self.config.get('name', 'unnamed') if self.config else 'unnamed',
            'current_state': self.current_state,
            'state_description': current.description,
            'time_in_state': elapsed,
            'condition_met': current.condition_met,
            'task_completed': current.task_completed,
            'num_states': len(self.states),
            'num_transitions': len(self.transitions),
            'timestamp': time.time()
        }
