#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot-agnostic task execution primitives.

Provides the generic queue, lifecycle, and dispatch machinery used to drive
arbitrary tasks on a robot. The dispatch is registry-based: each robot family
registers handlers for the task types it understands, keyed by task_type
string. The base class itself contains no robot-specific knowledge.
"""

import time
from enum import Enum
from typing import Any, Callable, Dict, List, Optional
from dataclasses import dataclass, field


class TaskStatus(Enum):
    """Task execution status."""
    PENDING = "pending"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


@dataclass
class TaskDescriptor:
    """Describes a task to be executed."""
    task_id: str
    task_type: str
    parameters: Dict[str, Any]
    status: TaskStatus = TaskStatus.PENDING
    created_at: float = field(default_factory=time.time)
    start_at: Optional[float] = None  # absolute epoch; None => start immediately
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
            'start_at': self.start_at,
            'started_at': self.started_at,
            'completed_at': self.completed_at,
            'error_message': self.error_message,
        }


# Handler signature: (executor, task) -> bool
# Returns True when the task is finished, False to keep it running.
TaskHandler = Callable[['TaskExecutorBase', TaskDescriptor], bool]


class TaskExecutorBase:
    """
    Robot-agnostic task executor.

    Owns a task queue, drives lifecycle (pending -> running -> completed /
    failed / cancelled), and dispatches to registered handler callables keyed
    by task_type string. Subclasses (per robot family) register handlers in
    `_register_default_handlers()` or via `register_handler()`.

    Override `cancel_task_type` if your robot wants a different sentinel
    task name to interrupt the in-flight task.
    """

    cancel_task_type: str = 'stop'

    def __init__(self):
        self.task_queue: List[TaskDescriptor] = []
        self.current_task: Optional[TaskDescriptor] = None
        self.task_history: List[TaskDescriptor] = []
        self._handlers: Dict[str, TaskHandler] = {}
        self._register_default_handlers()

    def _register_default_handlers(self) -> None:
        """Subclass hook. Register handlers on ``self`` at construction time."""
        pass

    def register_handler(self, task_type: str, handler: TaskHandler) -> None:
        """Register a handler for a given task_type (case-insensitive)."""
        self._handlers[task_type.lower()] = handler

    def add_task(self, task: TaskDescriptor) -> None:
        """Append a task to the back of the queue."""
        self.task_queue.append(task)

    def process_tasks(self) -> Optional[TaskDescriptor]:
        """
        Drive the queue forward by one tick.

        Returns the currently-running task (or None if idle).
        """
        # Cancel-current-on-next-stop hook: if the next queued task matches the
        # cancel sentinel and has zero delay, kill the in-flight task before
        # picking up the next one.
        if self.current_task is not None and self.task_queue:
            next_task = self.task_queue[0]
            if next_task.task_type.lower() == self.cancel_task_type.lower():
                delay = next_task.parameters.get('delay', 0.0)
                if delay == 0.0:
                    self.current_task.status = TaskStatus.CANCELLED
                    self.current_task.completed_at = time.time()
                    self.task_history.append(self.current_task)
                    self.current_task = None

        # Promote next pending task to running, unless it is scheduled for a
        # future shared-start instant (synchronized start). Comparison uses the
        # wall clock (time.time()) intentionally: the shared NTP-synced frame is
        # exactly what synchronizes starts across units.
        if self.current_task is None and self.task_queue:
            head = self.task_queue[0]
            if head.start_at is None or time.time() >= head.start_at:
                self.current_task = self.task_queue.pop(0)
                self.current_task.status = TaskStatus.RUNNING
                self.current_task.started_at = time.time()

        # Tick the current task.
        if self.current_task is not None:
            try:
                completed = self.execute_task(self.current_task)
                if completed:
                    if self.current_task.status == TaskStatus.RUNNING:
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
        Dispatch a task to its registered handler.

        Returns True when the task finishes, False to keep it running on the
        next process_tasks() tick. Raises ValueError for unknown task types.
        """
        handler = self._handlers.get(task.task_type.lower())
        if handler is None:
            raise ValueError(f'Unknown task type: {task.task_type}')
        return handler(self, task)
