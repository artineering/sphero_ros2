"""Sphero core functionality."""

from sphero_instance_controller.core.common.task import (
    TaskExecutorBase,
    TaskDescriptor,
    TaskStatus,
)

from .sphero import Sphero
from .state import SpheroState, SpheroConnectionState
from .matrix_patterns import get_pattern
from .sphero_task_handlers import TaskType
from .sphero_task_executor import SpheroTaskExecutorBase
from .direct_task_executor import DirectTaskExecutor
from .topic_task_executor import TopicTaskExecutor
from .statemachine import StateMachine, DynamicState, ExitSpec, ConditionType

# Backwards compatibility
TaskExecutor = DirectTaskExecutor

__all__ = [
    'Sphero',
    'SpheroState',
    'SpheroConnectionState',
    'get_pattern',
    'TaskExecutorBase',
    'SpheroTaskExecutorBase',
    'DirectTaskExecutor',
    'TopicTaskExecutor',
    'TaskExecutor',  # Backwards compatibility alias
    'TaskDescriptor',
    'TaskStatus',
    'TaskType',
    'StateMachine',
    'DynamicState',
    'ExitSpec',
    'ConditionType',
]
