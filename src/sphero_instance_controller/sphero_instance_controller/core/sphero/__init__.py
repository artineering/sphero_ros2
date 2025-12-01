"""Sphero core functionality."""

from .sphero import Sphero
from .state import SpheroState, SpheroConnectionState
from .matrix_patterns import get_pattern
from .task import TaskExecutorBase, TaskDescriptor, TaskStatus, TaskType
from .direct_task_executor import DirectTaskExecutor
from .topic_task_executor import TopicTaskExecutor
from .statemachine import StateMachine, DynamicState, ConditionType, TransitionConditionType

# Backwards compatibility
TaskExecutor = DirectTaskExecutor

__all__ = [
    'Sphero',
    'SpheroState',
    'SpheroConnectionState',
    'get_pattern',
    'TaskExecutorBase',
    'DirectTaskExecutor',
    'TopicTaskExecutor',
    'TaskExecutor',  # Backwards compatibility alias
    'TaskDescriptor',
    'TaskStatus',
    'TaskType',
    'StateMachine',
    'DynamicState',
    'ConditionType',
    'TransitionConditionType'
]
