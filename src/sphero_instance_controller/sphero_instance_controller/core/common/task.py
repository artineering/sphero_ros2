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
from typing import Any, Callable, Dict, FrozenSet, List, Optional
from dataclasses import dataclass, field


class TaskStatus(Enum):
    """Task execution status."""
    PENDING = "pending"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


# ----- Lane model (concurrent per-actuator execution) -----
# A task occupies one or more lanes. Tasks in disjoint lanes run in parallel;
# tasks sharing a lane are serialized (FIFO within a lane). Lane assignment per
# task_type lives beside the handler registry (see sphero_task_executor.py); the
# base class is robot-agnostic and only carries the resolved `lanes` set.
LANE_DRIVE = 'drive'
LANE_LED = 'led'
LANE_MATRIX = 'matrix'
LANE_CONFIG = 'config'

# The three slotted lanes that participate in gating / exclusivity. CONFIG is a
# slot (so it flows through the same promote->tick->complete path) but is
# deliberately excluded here so it never blocks, and is never blocked by, the
# three real lanes.
LANES = (LANE_DRIVE, LANE_LED, LANE_MATRIX)
EXCLUSIVE_LANES = frozenset(LANES)  # "whole robot" set (custom / jumping_bean)
DEFAULT_LANE = LANE_DRIVE

# Slot dict keys: the three real lanes plus CONFIG.
_SLOT_LANES = LANES + (LANE_CONFIG,)


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
    # The set of lanes this task occupies. Default = the single DRIVE lane,
    # which keeps lane-unaware tasks (and the generic test handlers) serial.
    lanes: FrozenSet[str] = field(default_factory=lambda: frozenset({DEFAULT_LANE}))

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
            'lanes': sorted(self.lanes),
            'lane': (
                'none' if not self.lanes
                else next(iter(self.lanes)) if len(self.lanes) == 1
                else 'multi'
            ),
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
        # One running slot per lane. Tasks in disjoint lanes run concurrently.
        self.current_tasks: Dict[str, Optional[TaskDescriptor]] = {
            ln: None for ln in _SLOT_LANES
        }
        self.task_history: List[TaskDescriptor] = []
        self._handlers: Dict[str, TaskHandler] = {}
        self._register_default_handlers()

    # ----- Back-compat single-slot view -----
    # Existing callers / tests treat the executor as having one `current_task`.
    # The getter resolves to the DRIVE slot (or any occupied slot); the setter
    # clears every lane then places the value under each of its lanes. This lets
    # the serial DRIVE stream and the force-state tests keep working unchanged.
    @property
    def current_task(self) -> Optional['TaskDescriptor']:
        drive = self.current_tasks[LANE_DRIVE]
        if drive is not None:
            return drive
        return next((t for t in self.current_tasks.values() if t is not None), None)

    @current_task.setter
    def current_task(self, value: Optional['TaskDescriptor']) -> None:
        for ln in self.current_tasks:
            self.current_tasks[ln] = None
        if value is not None:
            for ln in value.lanes:
                if ln in self.current_tasks:
                    self.current_tasks[ln] = value

    def running_tasks(self) -> List['TaskDescriptor']:
        """Distinct, non-None running tasks across all lane slots."""
        seen = []
        for t in self.current_tasks.values():
            if t is not None and t not in seen:
                seen.append(t)
        return seen

    def _register_default_handlers(self) -> None:
        """Subclass hook. Register handlers on ``self`` at construction time."""
        pass

    def _is_modifier(self, task: TaskDescriptor) -> bool:
        """Whether ``task`` is a modifier (runs inline, reserves no lane).

        Robot-agnostic default: nothing is a modifier, so generic tasks keep the
        owner promote->tick path. Robot subclasses override this to classify
        single-shot setpoint pokes (heading / speed / set_led / matrix).
        """
        return False

    def register_handler(self, task_type: str, handler: TaskHandler) -> None:
        """Register a handler for a given task_type (case-insensitive)."""
        self._handlers[task_type.lower()] = handler

    def add_task(self, task: TaskDescriptor) -> None:
        """Append a task to the back of the queue."""
        self.task_queue.append(task)

    def process_tasks(self) -> Optional[List[TaskDescriptor]]:
        """
        Drive the queue forward by one tick, per-lane.

        Phases: (1) handle any stop/cancel sentinel at the queue head, (2)
        promote due queued tasks into free lanes (per-lane start_at gating),
        (3) tick each running task.

        Returns the list of running tasks, or ``None`` when nothing is running
        (back-compat with the original single-slot contract, which returned the
        current task or ``None``).
        """
        # ----- Phase 1: stop / cancel sentinel handling (Decision D3). -----
        self._handle_stop_sentinels()

        # ----- Phase 2: per-lane promotion with per-lane start_at gating. -----
        self._promote_due()

        # ----- Phase 3: tick each distinct running task. -----
        for task in self.running_tasks():
            try:
                completed = self.execute_task(task)
                if completed:
                    if task.status == TaskStatus.RUNNING:
                        task.status = TaskStatus.COMPLETED
                    task.completed_at = time.time()
                    self.task_history.append(task)
                    self._clear_task(task)
            except Exception as e:
                task.status = TaskStatus.FAILED
                task.error_message = str(e)
                task.completed_at = time.time()
                self.task_history.append(task)
                self._clear_task(task)

        # ----- Phase 4: re-promote into lanes freed by Phase 3 completions. -----
        # A task that finished this tick frees its lane(s); its lane-follower
        # should start in the same tick (serial-within-lane has no idle gap).
        self._promote_due()

        running = self.running_tasks()
        return running if running else None

    # ----- Stop / promotion phases -----

    def _handle_stop_sentinels(self) -> None:
        """
        Phase 1. Consume stop/halt sentinels.

        Targeted (`target`) and panic (`scope == 'all'` / ``halt``) sentinels
        are consumed wherever they sit in the queue. A leading *bare* stop
        (zero delay, no target/scope) cancels only the DRIVE slot and is left in
        place so it falls through to promotion (so the physical stop is emitted).
        A delayed bare stop is an ordinary queued task and is left untouched.
        """
        cancel = self.cancel_task_type.lower()

        # First pass: scan the whole queue for targeted / panic sentinels.
        i = 0
        while i < len(self.task_queue):
            s = self.task_queue[i]
            if s.task_type.lower() != cancel:
                i += 1
                continue
            params = s.parameters
            target = params.get('target')
            is_panic = params.get('scope') == 'all' or s.task_type.lower() == 'halt'

            if target is not None:
                self._cancel_by_id(target)
                self.task_queue.pop(i)
                self._complete_sentinel(s)
                continue
            if is_panic:
                for t in self.running_tasks():
                    self._cancel_task(t)
                self.task_queue.clear()
                self._complete_sentinel(s)
                return  # queue is empty now
            i += 1

        # Second pass: a leading bare stop (delay 0) cancels the DRIVE slot.
        # When a DRIVE task is running, `_cancel_task` (via `_stop_lane`) now
        # emits the physical DRIVE stop, so we CONSUME the sentinel here (pop +
        # complete) to avoid it falling through to promotion -> `execute_stop`,
        # which would emit a second, redundant physical stop. When no DRIVE task
        # is running there is nothing to cancel, so we leave the sentinel in
        # place and let it fall through to promotion (its handler emits the
        # one physical stop). Anything else at the head is left as-is.
        if not self.task_queue:
            return
        head = self.task_queue[0]
        if head.task_type.lower() == cancel and head.parameters.get('delay', 0.0) == 0.0:
            drive = self.current_tasks[LANE_DRIVE]
            if drive is not None:
                self._cancel_task(drive)
                self.task_queue.pop(0)
                self._complete_sentinel(head)

    def _promote_due(self) -> None:
        """
        Phase 2 / 4. Promote due queued tasks into free lanes (per-lane FIFO).

        Scan front-to-back. A task promotes only if every one of its lanes is
        free AND not reserved this tick. Any task that cannot promote (busy
        lane, already-reserved lane, or not yet due per ``start_at``) reserves
        its OWN lanes, so a follower needing those lanes waits (preserving FIFO
        within a lane) while a task in a disjoint lane proceeds.
        """
        now = time.time()
        reserved = set()
        i = 0
        while i < len(self.task_queue):
            t = self.task_queue[i]
            lanes = self._slot_lanes(t.lanes)

            busy = any(self.current_tasks[ln] is not None for ln in lanes)
            not_due = t.start_at is not None and now < t.start_at
            if busy or (reserved & lanes) or not_due:
                reserved |= lanes
                i += 1
                continue

            # Modifier: single-shot live poke. Run it inline this tick — it
            # reserves no lane (empty lane set), so it never blocks an owner or a
            # follower and never enters `current_tasks`. start_at gating above
            # still applies (synchronized start preserved).
            if self._is_modifier(t):
                self.task_queue.pop(i)
                t.status = TaskStatus.RUNNING
                t.started_at = now
                try:
                    self.execute_task(t)
                    if t.status == TaskStatus.RUNNING:
                        t.status = TaskStatus.COMPLETED
                except Exception as e:
                    t.status = TaskStatus.FAILED
                    t.error_message = str(e)
                t.completed_at = time.time()
                self.task_history.append(t)
                # Do not slot, do not reserve lanes, do not advance i.
                continue

            # Promote.
            self.task_queue.pop(i)
            t.status = TaskStatus.RUNNING
            t.started_at = now
            for ln in lanes:
                self.current_tasks[ln] = t
            reserved |= lanes
            # Do not advance i: a new task may now sit at this index.

    # ----- Lane / cancel helpers -----

    def _slot_lanes(self, lanes: FrozenSet[str]) -> FrozenSet[str]:
        """Restrict a task's lanes to lanes that exist as slots."""
        return frozenset(ln for ln in lanes if ln in self.current_tasks)

    def _clear_task(self, task: TaskDescriptor) -> None:
        """Vacate every lane slot currently holding ``task``."""
        for ln in self.current_tasks:
            if self.current_tasks[ln] is task:
                self.current_tasks[ln] = None

    def _stop_lane(self, lane: str) -> None:
        """
        Robot-agnostic hook: emit a PHYSICAL stop for the given lane.

        Called by ``_cancel_task`` for each lane slot a cancelled task occupied,
        so cancellation (panic halt, targeted stop, bare stop) actually halts
        hardware rather than only vacating scheduler slots. The base class has no
        actuators, so this is a no-op; robot subclasses override it.
        """
        pass

    def _cancel_task(self, task: TaskDescriptor) -> None:
        """Mark a running task CANCELLED, file to history, vacate its lanes.

        Emits a physical stop per occupied lane via ``_stop_lane`` BEFORE
        vacating the slots, so the lane set is still known.
        """
        task.status = TaskStatus.CANCELLED
        task.completed_at = time.time()
        self.task_history.append(task)
        occupied = [ln for ln in self.current_tasks
                    if self.current_tasks[ln] is task]
        for ln in occupied:
            self._stop_lane(ln)
        self._clear_task(task)

    def _cancel_by_id(self, task_id: str) -> None:
        """Cancel a task by task_id, whether it is running or still queued."""
        for t in self.running_tasks():
            if t.task_id == task_id:
                self._cancel_task(t)
                return
        for idx, t in enumerate(self.task_queue):
            if t.task_id == task_id:
                t.status = TaskStatus.CANCELLED
                t.completed_at = time.time()
                self.task_history.append(t)
                self.task_queue.pop(idx)
                return

    def _complete_sentinel(self, sentinel: TaskDescriptor) -> None:
        """File a consumed targeted/halt sentinel as COMPLETED."""
        sentinel.status = TaskStatus.COMPLETED
        sentinel.completed_at = time.time()
        self.task_history.append(sentinel)

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
