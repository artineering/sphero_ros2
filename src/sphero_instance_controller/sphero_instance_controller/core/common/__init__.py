"""Robot-agnostic core primitives shared across robot families."""

from .task import TaskExecutorBase, TaskDescriptor, TaskStatus

__all__ = [
    'TaskExecutorBase',
    'TaskDescriptor',
    'TaskStatus',
]
