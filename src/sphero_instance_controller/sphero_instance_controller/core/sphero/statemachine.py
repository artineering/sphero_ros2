#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
State Machine Core Logic for Sphero Robot.

Each state declares its own ``exits[]`` list. Every exit pairs a ``condition``
with a ``destination`` state. Each tick, the state machine evaluates the
current state's exits in declared order; the first whose condition is True
fires the transition. A state with no ``exits`` is a leaf state and the
machine remains there.
"""

import time
from typing import Dict, Any, List, Optional, Callable, Iterable
from enum import Enum
from dataclasses import dataclass, field


class ConditionType(Enum):
    """Types of conditions usable for entry and exit."""
    ALWAYS = "always"
    TIMER = "timer"
    TOPIC_VALUE = "topic_value"
    TOPIC_MESSAGE = "topic_message"


_VALID_OPERATORS = ('==', '!=', '>', '<', '>=', '<=')
_ENTRY_CONDITION_TYPES = ('always', 'timer', 'topic_value')
_EXIT_CONDITION_TYPES = ('always', 'timer', 'topic_value', 'topic_message')


@dataclass
class ExitSpec:
    """A single exit from a state: a condition paired with a destination."""
    condition: Dict[str, Any]
    destination: str


@dataclass
class DynamicState:
    """A dynamically configured state.

    Schema (config dict):
        name: str
        description: str (optional)
        entry_condition: {type, ...} (optional, defaults to {type: 'always'})
        exits: [{condition: {type, ...}, destination: str}, ...] (optional)
        tasks: [...] (optional)
        timeout: float (optional)

    A state with no ``exits`` (empty list or missing key) is a leaf state.
    """
    name: str
    config: Dict[str, Any]
    entry_condition: Dict[str, Any] = field(default_factory=lambda: {'type': 'always'})
    exits: List[ExitSpec] = field(default_factory=list)
    tasks: List[Dict[str, Any]] = field(default_factory=list)
    description: str = ''
    timeout: Optional[float] = None

    entry_time: Optional[float] = None
    condition_met: bool = False
    task_completed: bool = False

    def __post_init__(self):
        self.entry_condition = self.config.get('entry_condition', {'type': 'always'})

        self.exits = [
            ExitSpec(
                condition=exit_cfg.get('condition', {}),
                destination=exit_cfg.get('destination'),
            )
            for exit_cfg in self.config.get('exits', [])
        ]

        if 'tasks' in self.config:
            self.tasks = self.config['tasks']
        elif 'task' in self.config:
            self.tasks = [self.config['task']]
        else:
            self.tasks = []

        self.description = self.config.get('description', '')
        self.timeout = self.config.get('timeout', None)

    def isLeafState(self) -> bool:
        """Return True if this state has no exits and will never transition out."""
        return len(self.exits) == 0


class StateMachine:
    """
    Core state machine logic for Sphero robot.

    ROS-independent — all interaction with the outside world happens through
    injectable callbacks (logger, topic subscribe/unsubscribe).
    """

    def __init__(
        self,
        logger: Optional[Callable] = None,
        topic_subscribe_callback: Optional[Callable] = None,
        topic_unsubscribe_callback: Optional[Callable] = None,
    ):
        """
        Args:
            logger: Optional logging function (e.g. ``node.get_logger().info``).
            topic_subscribe_callback: Called with ``(topic_name, msg_type, field_path)``
                for every topic referenced by an entry/exit condition.
            topic_unsubscribe_callback: Called with ``(topic_name)`` to unsubscribe
                when the configuration changes.
        """
        self.logger = logger or self._default_logger
        self.topic_subscribe_callback = topic_subscribe_callback
        self.topic_unsubscribe_callback = topic_unsubscribe_callback

        self.config: Optional[Dict[str, Any]] = None
        self.states: Dict[str, DynamicState] = {}
        self.current_state: Optional[str] = None

        # Pause / resume state. While paused, process() returns a status dict
        # with no events; on resume, every entry_time / topic_last_received is
        # shifted forward by the paused duration so elapsed-time math is correct.
        self.paused: bool = False
        self._pause_started_at: Optional[float] = None

        # Sensor data is kept for backward compatibility with the ROS2 controller
        # node, which calls update_sensor_data(). It is no longer evaluated by the
        # condition engine — exit/entry conditions read from topic_values only.
        self.sensor_topic_values: Dict[str, Any] = {}

        self.topic_values: Dict[str, Any] = {}
        self.topic_last_received: Dict[str, float] = {}

    def _default_logger(self, message: str):
        print(f"[StateMachine] {message}")

    # ------------------------------------------------------------------ config

    def configure(self, config: Dict[str, Any]) -> bool:
        """Configure the state machine. Returns True on success."""
        self.logger(f'Configuring state machine: {config.get("name", "unnamed")}')

        if not self.validate_config(config):
            self.logger('ERROR: Invalid configuration')
            return False

        self._build_state_machine(config)
        self.logger(f'State machine configured with {len(self.states)} states')
        return True

    def validate_config(self, config: Dict[str, Any]) -> bool:
        """Validate the configuration. Returns True if valid."""
        if not isinstance(config, dict):
            self.logger('ERROR: Configuration must be a dictionary')
            return False

        if 'states' not in config or not config['states']:
            self.logger('ERROR: Configuration must have at least one state')
            return False

        if 'initial_state' not in config:
            self.logger('ERROR: Configuration must specify initial_state')
            return False

        state_names = [s['name'] for s in config['states']]
        if config['initial_state'] not in state_names:
            self.logger(f'ERROR: Initial state "{config["initial_state"]}" not found in states')
            return False

        if 'transitions' in config:
            self.logger(
                'ERROR: Top-level "transitions" is no longer supported. '
                'Move each transition into the source state\'s `exits[]` array '
                'as {"condition": {...}, "destination": "..."}.'
            )
            return False

        for state_cfg in config['states']:
            name = state_cfg.get('name', '<unnamed>')

            entry = state_cfg.get('entry_condition')
            if entry is not None:
                if not self._validate_condition(
                    entry,
                    allowed_types=_ENTRY_CONDITION_TYPES,
                    label=f'state "{name}" entry_condition',
                ):
                    return False

            for i, exit_cfg in enumerate(state_cfg.get('exits', [])):
                label = f'state "{name}" exits[{i}]'
                if not isinstance(exit_cfg, dict):
                    self.logger(f'ERROR: {label} must be a dict')
                    return False
                if 'condition' not in exit_cfg:
                    self.logger(f'ERROR: {label} missing "condition"')
                    return False
                if 'destination' not in exit_cfg:
                    self.logger(f'ERROR: {label} missing "destination"')
                    return False
                if exit_cfg['destination'] not in state_names:
                    self.logger(
                        f'ERROR: {label} destination "{exit_cfg["destination"]}" '
                        f'is not a known state'
                    )
                    return False
                if not self._validate_condition(
                    exit_cfg['condition'],
                    allowed_types=_EXIT_CONDITION_TYPES,
                    label=f'{label}.condition',
                ):
                    return False

        return True

    def _validate_condition(
        self,
        condition: Any,
        allowed_types: Iterable[str],
        label: str,
    ) -> bool:
        """Validate a condition dict. ``allowed_types`` gates which types are accepted."""
        if not isinstance(condition, dict):
            self.logger(f'ERROR: {label} must be a dict')
            return False

        cond_type = condition.get('type')
        if cond_type is None:
            self.logger(f'ERROR: {label} missing "type"')
            return False

        if cond_type not in allowed_types:
            self.logger(
                f'ERROR: {label} unknown or disallowed type "{cond_type}". '
                f'Allowed: {list(allowed_types)}'
            )
            return False

        if cond_type == 'always':
            return True

        if cond_type == 'timer':
            if 'duration' not in condition:
                self.logger(f'ERROR: {label} type "timer" requires "duration"')
                return False
            return True

        if cond_type == 'topic_value':
            for required in ('topic', 'operator', 'value'):
                if required not in condition:
                    self.logger(f'ERROR: {label} type "topic_value" requires "{required}"')
                    return False
            if condition['operator'] not in _VALID_OPERATORS:
                self.logger(
                    f'ERROR: {label} invalid operator "{condition["operator"]}". '
                    f'Must be one of {list(_VALID_OPERATORS)}'
                )
                return False
            return True

        if cond_type == 'topic_message':
            if 'topic' not in condition:
                self.logger(f'ERROR: {label} type "topic_message" requires "topic"')
                return False
            return True

        # Should be unreachable thanks to allowed_types check above.
        self.logger(f'ERROR: {label} unhandled condition type "{cond_type}"')
        return False

    def _build_state_machine(self, config: Dict[str, Any]):
        """Build state objects, wire subscriptions, set initial state."""
        self.config = config

        if self.topic_unsubscribe_callback:
            for topic_name in list(self.topic_values.keys()):
                self.topic_unsubscribe_callback(topic_name)

        self.states = {}
        self.topic_values.clear()
        self.topic_last_received.clear()

        for state_cfg in config['states']:
            name = state_cfg['name']
            self.states[name] = DynamicState(name, state_cfg)
            self.logger(f'Created state: {name}')

        if self.topic_subscribe_callback:
            seen = set()
            for state in self.states.values():
                for cond in self._iter_state_conditions(state):
                    ctype = cond.get('type')
                    if ctype in ('topic_value', 'topic_message'):
                        topic = cond.get('topic')
                        msg_type = cond.get('msg_type')
                        field_path = cond.get('field_path')
                        if topic and msg_type and topic not in seen:
                            seen.add(topic)
                            self.topic_subscribe_callback(topic, msg_type, field_path)
                        elif topic and not msg_type:
                            self.logger(
                                f'WARNING: state "{state.name}" condition on topic "{topic}" '
                                f'is missing "msg_type"; cannot subscribe.'
                            )

        self.paused = False
        self._pause_started_at = None

        self.current_state = config['initial_state']
        self.states[self.current_state].entry_time = time.time()
        self.states[self.current_state].condition_met = True

        self.logger(f'State machine ready. Initial state: {self.current_state}')

    @staticmethod
    def _iter_state_conditions(state: DynamicState) -> Iterable[Dict[str, Any]]:
        if state.entry_condition:
            yield state.entry_condition
        for exit_spec in state.exits:
            if exit_spec.condition:
                yield exit_spec.condition

    # --------------------------------------------------------------- updaters

    def update_sensor_data(self, sensor_data: Dict[str, Any]):
        """Backward-compat sensor data sink. No longer consumed by the engine."""
        self.sensor_topic_values.update(sensor_data)

    def update_topic_value(self, topic_name: str, value: Any):
        """Record a topic's latest value and timestamp for condition evaluation."""
        self.topic_values[topic_name] = value
        self.topic_last_received[topic_name] = time.time()

    # -------------------------------------------------------------- pause/clear

    def pause(self) -> bool:
        """Pause ticking. Returns False if no current state or already paused."""
        if self.current_state is None or self.paused:
            return False
        self.paused = True
        self._pause_started_at = time.time()
        self.logger('State machine paused')
        return True

    def resume(self) -> bool:
        """Resume ticking; shift entry/timestamp clocks so paused time doesn't count."""
        if not self.paused:
            return False
        paused_duration = time.time() - (self._pause_started_at or time.time())
        for state in self.states.values():
            if state.entry_time is not None:
                state.entry_time += paused_duration
        for topic in list(self.topic_last_received.keys()):
            self.topic_last_received[topic] += paused_duration
        self.paused = False
        self._pause_started_at = None
        self.logger(f'State machine resumed (paused for {paused_duration:.2f}s)')
        return True

    def clear(self) -> None:
        """Drop the loaded state machine and unsubscribe from all condition topics."""
        if self.topic_unsubscribe_callback:
            for topic_name in list(self.topic_values.keys()):
                self.topic_unsubscribe_callback(topic_name)
        self.config = None
        self.states = {}
        self.current_state = None
        self.paused = False
        self._pause_started_at = None
        self.sensor_topic_values.clear()
        self.topic_values.clear()
        self.topic_last_received.clear()
        self.logger('State machine cleared')

    # ------------------------------------------------------------------- tick

    def process(self) -> Optional[Dict[str, Any]]:
        """Tick the state machine. Returns a status dict (with events) or None."""
        if self.current_state is None or not self.states:
            return None

        current = self.states[self.current_state]

        if self.paused:
            return {
                'current_state': self.current_state,
                'events': [],
                'paused': True,
                'time_in_state': time.time() - current.entry_time if current.entry_time else 0,
            }

        events = []

        if current.timeout is not None and current.entry_time is not None:
            elapsed = time.time() - current.entry_time
            if elapsed > current.timeout:
                events.append({
                    'type': 'state_timeout',
                    'state': self.current_state,
                    'elapsed': elapsed,
                })

        transition_event = self._check_transitions()
        if transition_event:
            events.append(transition_event)

        return {
            'current_state': self.current_state,
            'events': events,
            'time_in_state': time.time() - current.entry_time if current.entry_time else 0,
        }

    def _check_transitions(self) -> Optional[Dict[str, Any]]:
        """Evaluate the current state's exits and fire the first whose condition is met."""
        if self.current_state is None:
            return None
        current = self.states[self.current_state]
        if current.isLeafState():
            return None

        for exit_spec in current.exits:
            if self._evaluate_condition(exit_spec.condition):
                old_state = self.current_state
                dest = exit_spec.destination
                self.logger(f'Exit fired: {old_state} -> {dest}')
                self.transition_to_state(dest)
                return {
                    'type': 'state_transition',
                    'from': old_state,
                    'to': dest,
                    'timestamp': time.time(),
                }
        return None

    def _evaluate_condition(self, condition: Dict[str, Any]) -> bool:
        """Evaluate a flat condition dict ({type, ...params}) against current runtime state."""
        if not condition:
            return False

        cond_type = condition.get('type')

        if cond_type == 'always':
            return True

        if cond_type == 'timer':
            if self.current_state is None:
                return False
            current = self.states[self.current_state]
            if current.entry_time is None:
                return False
            duration = condition.get('duration', 0.0)
            return time.time() - current.entry_time >= duration

        if cond_type == 'topic_value':
            topic = condition.get('topic')
            operator = condition.get('operator', '==')
            expected = condition.get('value')
            if not topic or topic not in self.topic_values:
                return False
            try:
                return self._compare_values(self.topic_values[topic], operator, expected)
            except Exception as e:
                self.logger(f'ERROR: Error comparing topic values for "{topic}": {e}')
                return False

        if cond_type == 'topic_message':
            topic = condition.get('topic')
            timeout = condition.get('timeout', None)
            if not topic or topic not in self.topic_last_received:
                return False
            if timeout is not None:
                return time.time() - self.topic_last_received[topic] <= timeout
            return True

        self.logger(f'WARNING: Unknown condition type: {cond_type}')
        return False

    def _compare_values(self, actual: Any, operator: str, expected: Any) -> bool:
        if operator == '==':
            return actual == expected
        if operator == '!=':
            return actual != expected
        if operator == '>':
            return actual > expected
        if operator == '<':
            return actual < expected
        if operator == '>=':
            return actual >= expected
        if operator == '<=':
            return actual <= expected
        self.logger(f'WARNING: Unknown operator: {operator}')
        return False

    # ----------------------------------------------------------- manual moves

    def transition_to_state(self, new_state: str) -> bool:
        """Force a transition to ``new_state``. Returns True on success."""
        if new_state not in self.states:
            self.logger(f'ERROR: Cannot transition to unknown state: {new_state}')
            return False

        old_state = self.current_state
        self.current_state = new_state
        self.states[new_state].entry_time = time.time()
        self.states[new_state].condition_met = True
        self.states[new_state].task_completed = False

        self.logger(f'Transitioned from {old_state} to {new_state}')
        return True

    # ------------------------------------------------------------------ tasks

    def get_current_state_tasks(self) -> List[Dict[str, Any]]:
        if self.current_state and self.current_state in self.states:
            return self.states[self.current_state].tasks
        return []

    def mark_tasks_completed(self):
        if self.current_state and self.current_state in self.states:
            self.states[self.current_state].task_completed = True

    # ----------------------------------------------------------------- status

    def get_status(self) -> Dict[str, Any]:
        if self.current_state is None:
            return {
                'configured': False,
                'current_state': None,
                'paused': False,
                'timestamp': time.time(),
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
            'is_leaf_state': current.isLeafState(),
            'num_states': len(self.states),
            'num_exits': len(current.exits),
            'paused': self.paused,
            'timestamp': time.time(),
        }
