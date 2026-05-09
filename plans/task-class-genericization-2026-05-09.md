# Robot-agnostic Task class + plugin-registry dispatch

## Context

`core/sphero/task.py` carries two concerns mashed together:

1. **Generic queue & lifecycle infrastructure** — `TaskDescriptor` (a plain dataclass), `TaskStatus`, queue management, `add_task` / `process_tasks` / `execute_task`. None of this depends on Sphero.
2. **Sphero-specific dispatch** — a 16-arm `if/elif` chain in `execute_task()` (task.py:222–258) hard-coded to Sphero task types (`roll`, `set_led`, `matrix`, `jumping_bean`, …); 18 `_execute_*` and `_send_*` methods built around Sphero kinematics (heading + 2D position, wheel separation, BOLT matrix, collision-detection modes).

The path `core/sphero/task.py` makes it look like the whole thing is Sphero-owned. The dataclass and queue mechanics are not — they're useful for any robot. We want to:

- Make `TaskDescriptor` / `TaskStatus` and the queue/lifecycle live in a robot-agnostic location so a future robot package can import them without depending on Sphero.
- Replace the if/elif dispatch with a **handler registry** so each robot registers its own task types without modifying base-class code.
- Keep all 16 existing Sphero task types and their semantics **bit-for-bit identical** — no behavior change, no breakage of state-machine configs already in flight, no new test failures in the 115-test SM suite.

Decisions locked in (from clarifying questions):
- Registry approach (handlers register by string `task_type`).
- New `core/common/` subpackage hosts the generic pieces.
- Sphero handlers move into a dedicated handlers module — same logic, just relocated and re-shaped from methods to functions.

There are only **three** import sites for `Task*` symbols outside `task.py` itself, so the migration surface is small:
- `core/sphero/__init__.py` (re-exports)
- `core/sphero/direct_task_executor.py` (`from .task import TaskExecutorBase`)
- `core/sphero/topic_task_executor.py` (`from .task import TaskExecutorBase`)
- `sphero_instance_task_controller_node.py` (`from sphero_instance_controller.core.sphero.task import TaskDescriptor, TaskStatus`)

Plus one cruft import inside `task.py` itself: `from .sphero import Sphero` (line 17) is unused — `Task*` never references `Sphero`. Drops out.

---

## Architecture before / after

**Before:**
```
core/sphero/task.py
├── TaskStatus, TaskType (enums)
├── TaskDescriptor (dataclass)
└── TaskExecutorBase
    ├── task_queue / current_task / task_history
    ├── add_task / process_tasks
    ├── execute_task — 16-arm if/elif on task_type
    ├── _send_*  — 10 abstract Sphero methods
    ├── _execute_*  — 16 Sphero handler methods
    └── position_callback / heading_callback / get_current_position / get_current_heading
```

**After:**
```
core/common/
├── __init__.py                      ← re-exports the generic pieces
└── task.py
    ├── TaskStatus
    ├── TaskDescriptor
    └── TaskExecutorBase             ← purely generic; registry-based dispatch
        ├── task_queue / current_task / task_history
        ├── _handlers: Dict[str, Callable[[Self, TaskDescriptor], bool]]
        ├── register_handler(task_type, fn)
        ├── add_task / process_tasks
        ├── execute_task              ← dispatches via self._handlers
        └── cancel_task_type = 'stop' ← single configurable knob for the
                                        "next-task-cancels-current" hook

core/sphero/
├── sphero_task_executor.py          ← NEW
│   └── SpheroTaskExecutorBase(TaskExecutorBase)
│       ├── position_callback / heading_callback / get_current_position / get_current_heading
│       ├── default_speed / position_tolerance / heading_tolerance
│       ├── 10 abstract _send_* methods (unchanged signatures)
│       └── _register_default_handlers()  ← registers all 16 Sphero handlers
├── sphero_task_handlers.py          ← NEW
│   └── 16 handler functions, signature (executor, task) -> bool
│       (the existing _execute_* bodies, lifted verbatim and rewritten
│        to call executor._send_*, executor.get_current_position, etc.)
├── direct_task_executor.py          ← extends SpheroTaskExecutorBase (unchanged otherwise)
├── topic_task_executor.py           ← extends SpheroTaskExecutorBase (unchanged otherwise)
└── task.py                          ← DELETED (or stub re-exports for one release if needed)
```

---

## Generic `TaskExecutorBase` shape

```python
# core/common/task.py

class TaskExecutorBase:
    cancel_task_type: str = 'stop'   # next-task-with-this-type cancels current task

    def __init__(self):
        self.task_queue: List[TaskDescriptor] = []
        self.current_task: Optional[TaskDescriptor] = None
        self.task_history: List[TaskDescriptor] = []
        self._handlers: Dict[str, Callable[['TaskExecutorBase', TaskDescriptor], bool]] = {}
        self._register_default_handlers()

    def _register_default_handlers(self) -> None:
        """Subclass hook. Override to register handlers at construction time."""
        pass

    def register_handler(
        self,
        task_type: str,
        handler: Callable[['TaskExecutorBase', TaskDescriptor], bool],
    ) -> None:
        self._handlers[task_type.lower()] = handler

    def add_task(self, task: TaskDescriptor): ...
    def process_tasks(self) -> Optional[TaskDescriptor]: ...   # logic preserved
    def execute_task(self, task: TaskDescriptor) -> bool:
        handler = self._handlers.get(task.task_type.lower())
        if handler is None:
            raise ValueError(f'Unknown task type: {task.task_type}')
        return handler(self, task)
```

Notes:
- Dispatch is via `self._handlers` — populated in `_register_default_handlers()` (subclass hook) and/or after construction via `register_handler()`.
- Stop-cancels-current logic in `process_tasks()` consults `self.cancel_task_type` (default `'stop'`) — keeps Sphero behavior identical but lets a different robot opt out / rename.
- `position_callback`, `heading_callback`, `default_speed`, `position_tolerance`, `heading_tolerance` are **dropped** from this class — they're Sphero-shaped (2D + heading + cm/deg). They move to `SpheroTaskExecutorBase`.
- `TaskType` enum moves to **Sphero** layer (it's a Sphero-specific catalog) — see below.

## Sphero layer

```python
# core/sphero/sphero_task_executor.py

from sphero_instance_controller.core.common.task import TaskExecutorBase, TaskDescriptor, TaskStatus
from . import sphero_task_handlers as h

class SpheroTaskExecutorBase(TaskExecutorBase):
    cancel_task_type = 'stop'

    def __init__(self, position_callback=None, heading_callback=None):
        self.position_callback = position_callback
        self.heading_callback = heading_callback
        self.default_speed = 100
        self.position_tolerance = 10.0
        self.heading_tolerance = 5
        super().__init__()  # invokes _register_default_handlers

    def _register_default_handlers(self):
        self.register_handler('move_to',         h.execute_move_to)
        self.register_handler('patrol',          h.execute_patrol)
        self.register_handler('circle',          h.execute_circle)
        self.register_handler('square',          h.execute_square)
        self.register_handler('led_sequence',    h.execute_led_sequence)
        self.register_handler('matrix_sequence', h.execute_matrix_sequence)
        self.register_handler('spin',            h.execute_spin)
        self.register_handler('stop',            h.execute_stop)
        self.register_handler('custom',          h.execute_custom)
        self.register_handler('set_led',         h.execute_set_led)
        self.register_handler('roll',            h.execute_roll)
        self.register_handler('heading',         h.execute_heading)
        self.register_handler('speed',           h.execute_speed)
        self.register_handler('matrix',          h.execute_matrix)
        self.register_handler('collision',       h.execute_collision)
        self.register_handler('reflect',         h.execute_reflect)
        self.register_handler('jumping_bean',    h.execute_jumping_bean)

    def get_current_position(self) -> Dict[str, float]: ...   # unchanged
    def get_current_heading(self) -> int: ...                  # unchanged

    # 10 abstract _send_* methods, signatures unchanged
    def _send_raw_motor_command(...): raise NotImplementedError
    def _send_roll_command(...): raise NotImplementedError
    # ...
```

```python
# core/sphero/sphero_task_handlers.py

# 16 functions, each one is the existing _execute_* body lifted verbatim.
# Difference: signature is (executor, task) -> bool; references like
# self._send_roll_command(...) become executor._send_roll_command(...).

def execute_move_to(executor, task: TaskDescriptor) -> bool:
    target_x = task.parameters.get('x', 0)
    # ... (verbatim from current task.py:262–323, with self -> executor)

def execute_patrol(executor, task): ...
# ... 14 more
```

Behavior is identical — same parameter reads, same `_send_*` calls, same return semantics, same stall-detection / waypoint-cycling / sequence-stepping logic. Pure relocation + signature reshape.

`TaskType` enum: keep it for backwards-compat callers (and the existing dropdown templates in `app.js`) but move it to `core/sphero/sphero_task_handlers.py` (or a small `core/sphero/task_types.py`) — it's a Sphero catalog of strings, not a generic concept. Re-export from `core/sphero/__init__.py` so any external code that did `from ...core.sphero import TaskType` keeps working.

---

## Phase 1 — Create `core/common/`

**New files:**
- `src/sphero_instance_controller/sphero_instance_controller/core/common/__init__.py` — re-export `TaskExecutorBase`, `TaskDescriptor`, `TaskStatus`.
- `src/sphero_instance_controller/sphero_instance_controller/core/common/task.py` — generic `TaskStatus`, `TaskDescriptor`, `TaskExecutorBase` (shape above).

`TaskDescriptor.to_dict()` is preserved as-is. `TaskStatus` enum unchanged.

## Phase 2 — Split Sphero pieces out of `core/sphero/task.py`

**New files:**
- `core/sphero/sphero_task_executor.py` — `SpheroTaskExecutorBase` (shape above).
- `core/sphero/sphero_task_handlers.py` — 16 handler functions + `TaskType` enum (re-located).

**Deletion:** `core/sphero/task.py` — content fully migrated; remove the file.

## Phase 3 — Update `Direct` / `Topic` executors

Two-line change per file:
```python
# direct_task_executor.py — before
from .task import TaskExecutorBase
class DirectTaskExecutor(TaskExecutorBase): ...

# direct_task_executor.py — after
from .sphero_task_executor import SpheroTaskExecutorBase
class DirectTaskExecutor(SpheroTaskExecutorBase): ...
```

`__init__` signatures already accept `position_callback` / `heading_callback` and pass them to `super().__init__` — preserved.

Same change in `topic_task_executor.py`.

## Phase 4 — Fix imports

**`core/sphero/__init__.py`** — adjust source modules:
```python
from sphero_instance_controller.core.common.task import (
    TaskExecutorBase, TaskDescriptor, TaskStatus,
)
from .sphero_task_handlers import TaskType
from .sphero_task_executor import SpheroTaskExecutorBase
from .direct_task_executor import DirectTaskExecutor
from .topic_task_executor import TopicTaskExecutor
# Backwards compatibility
TaskExecutor = DirectTaskExecutor
__all__ = [..., 'SpheroTaskExecutorBase', ...]   # add new symbol
```

**`sphero_instance_task_controller_node.py:23`** — change to:
```python
from sphero_instance_controller.core.common.task import TaskDescriptor, TaskStatus
```

(Or keep importing from `core.sphero` since `__init__.py` still re-exports them — equally fine. Prefer the direct `core.common` import to make the boundary explicit at consumption sites too.)

## Phase 5 — Tests

The state-machine suite (`test/test_statemachine.py`, 115 tests) does **not** exercise `TaskExecutorBase` — it only checks that the SM emits the right `task_emit` events. Should stay green untouched.

Add a new file `src/sphero_instance_controller/test/test_task_executor.py` covering the new abstractions:

- **A. Generic queue / lifecycle** (uses a `RecordingTaskExecutor` test double — subclass of `TaskExecutorBase` that registers a handful of trivial handlers and records calls):
  - `test_add_task_appends_to_queue`
  - `test_process_tasks_picks_up_pending`
  - `test_process_tasks_marks_completed_when_handler_returns_true`
  - `test_process_tasks_keeps_running_when_handler_returns_false`
  - `test_handler_exception_marks_task_failed`
  - `test_unknown_task_type_raises_value_error`
  - `test_to_dict_round_trip`
- **B. Registry semantics**:
  - `test_register_handler_lowercases_task_type`
  - `test_register_handler_overwrites_existing`
  - `test_default_handlers_registered_via_subclass_hook`
- **C. Cancel-on-stop hook**:
  - `test_cancel_task_type_cancels_current_when_next_is_stop`
  - `test_cancel_task_type_with_delay_does_not_cancel`
  - `test_custom_cancel_task_type_via_class_attr` — set `cancel_task_type = 'abort'`, confirm it's that one that cancels.
- **D. Sphero handler regression smoke** (uses a mock executor with all `_send_*` methods recording calls):
  - For each of the 16 task types, fire one canonical task and assert the right `_send_*` was called with the right args. Catches any silent breakage during the handler relocation.

Target: ~25 new tests in `test_task_executor.py`. SM suite stays at 115.

---

## Out of scope (deferred)

- **Decorator-based registration** (`@task_handler('roll')` at module import time). Cleaner if there are many independently-shipped handler modules, but for a single Sphero handler module the explicit list in `_register_default_handlers()` is more debuggable. Add the decorator if/when a second robot lands.
- **Async handlers / coroutines.** Today handlers return `bool`; that's fine.
- **`TaskType` redesign / pruning.** The 16 types stay exactly as-is. Don't split `patrol`/`circle`/`square`, don't simplify `custom`. Out of scope per "Keep as-is" decision.
- **Promoting `core/common` to its own ROS package.** Stays inside `sphero_instance_controller` for now; can lift later when an actual second robot package exists.
- **Migrating the older `sphero_task_controller` / `sphero_statemachine` packages** (which duplicate `TaskType` and dispatch). Those are pre-multi-robot legacy and will be dealt with separately.

---

## Critical files

- **New:** `src/sphero_instance_controller/sphero_instance_controller/core/common/__init__.py`
- **New:** `src/sphero_instance_controller/sphero_instance_controller/core/common/task.py` — generic `TaskStatus`, `TaskDescriptor`, `TaskExecutorBase` (registry-based).
- **New:** `src/sphero_instance_controller/sphero_instance_controller/core/sphero/sphero_task_executor.py` — `SpheroTaskExecutorBase` with `position`/`heading` callbacks, 10 abstract `_send_*` methods, registers Sphero handlers.
- **New:** `src/sphero_instance_controller/sphero_instance_controller/core/sphero/sphero_task_handlers.py` — 16 handler functions + `TaskType` enum.
- **New:** `src/sphero_instance_controller/test/test_task_executor.py` — ~25 tests (groups A–D above).
- **Delete:** `src/sphero_instance_controller/sphero_instance_controller/core/sphero/task.py` — content migrated, remove file.
- **Edit:** `src/sphero_instance_controller/sphero_instance_controller/core/sphero/direct_task_executor.py` — base class swap.
- **Edit:** `src/sphero_instance_controller/sphero_instance_controller/core/sphero/topic_task_executor.py` — base class swap.
- **Edit:** `src/sphero_instance_controller/sphero_instance_controller/core/sphero/__init__.py` — fix exports.
- **Edit:** `src/sphero_instance_controller/sphero_instance_controller/sphero_instance_task_controller_node.py:23` — import from `core.common`.
- **No edits:** `statemachine.py`, controller node, websocket server, UI files. Wire format & emitted task dicts unchanged.

---

## Verification

1. **Build:**
   ```bash
   cd /home/svaghela/ros2_ws_2
   colcon build --packages-select sphero_instance_controller
   source install/setup.bash
   ```

2. **Existing test suite (regression check):**
   ```bash
   pytest src/sphero_instance_controller/test/test_statemachine.py -v
   ```
   Expect **115/115 green** — no SM behavior changed.

3. **New task tests:**
   ```bash
   pytest src/sphero_instance_controller/test/test_task_executor.py -v
   ```
   Expect ~25 green covering queue lifecycle, registry, cancel-on-stop, and per-handler smoke.

4. **Live click-test against SB-3660** (full chain — confirms zero regressions on the wire):
   - Bring up the multirobot dashboard and one Sphero instance.
   - Load the patrol composite-state config (existing `LOAD NESTED` button) and confirm the LEDs cycle N/E/S/W. This exercises `set_led` handler emissions through the new registry.
   - Issue a manual `roll` and `set_matrix` from the per-instance UI. Confirm the robot responds identically to the pre-refactor build.
   - Halt → confirm `stop` task cancels in-flight task immediately (`cancel_task_type` path).

5. **ROS-only spot check** — confirm the published task JSON wire format is unchanged:
   ```bash
   ros2 topic echo /sphero/SB_3660/task --field data | head -3
   ```
   Should still be `{"task_id": "...", "task_type": "set_led", "parameters": {...}}`.

---

## Notes on plan-file location

Per the workspace memory note, the executable plan file lives in `/home/svaghela/ros2_ws_2/plans/`. After approval, this plan content will be saved as `plans/task-class-genericization-2026-05-09.md` in the workspace before any source files are touched. The current `~/.claude/plans/` file is the working draft used by plan mode.
