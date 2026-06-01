# Fix stop/halt not emitting a physical stop (robot keeps rolling after halt)

**Created:** 2026-06-01T18:13:01Z
**Status:** Pending Approval

## Task Description
A confirmed live-hardware bug: `stop`/`halt` cancels tasks in the scheduler but
never emits a PHYSICAL stop, so a robot running an indefinite `roll` keeps
rolling after halt. Route physical stops through the cancellation path via an
overridable lane-scoped hook.

## Analysis (root cause confirmed)
In `core/common/task.py` (`TaskExecutorBase`):
- `_cancel_task` (line 289) / `_cancel_by_id` (296) / the panic loop in
  `_handle_stop_sentinels` (198) only set `status=CANCELLED` and vacate lane
  slots via `_clear_task` (283). They emit NO hardware command.
- The ONLY physical-stop path today is a bare `stop` sentinel falling through to
  promotion, where `execute_stop` calls `executor._send_stop_command()`.
- Panic halt (`scope:'all'`/`halt`) cancels + clears + RETURNS — no stop
  sentinel ever promoted → no `_send_stop_command()` → motors keep running.
- Targeted stop (`target`) → `_cancel_by_id` → `_cancel_task` → no physical cmd.

Architecture constraint: `_cancel_task` lives in robot-agnostic base; the
physical `_send_*` live in Sphero subclasses. Need an overridable hook on base.

### Matrix-clear investigation (important finding)
- `_send_matrix_command(...)` (topic/direct) routes to `Sphero.set_matrix`
  (`core/sphero/sphero.py:347`). `set_matrix(pattern=None, custom_matrix=None)`
  returns `False` early (matrix_data is None) and does NOT blank the device.
- There is no `'clear'` pattern in `matrix_patterns.py`. The only true blank
  primitive is `Sphero.clear_matrix()` / `api.clear_matrix()` (sphero.py:397).
- The established clear CONVENTION in the codebase is the `/api/matrix/clear`
  route (`sphero_instance_websocket_server.py:611`) which calls
  `publish_matrix_command('', 0, 0, 0)` — empty pattern + all-zero color.
- BUT the device controller `matrix_callback`
  (`sphero_instance_device_controller_node.py:358`) does NOT honor that clear
  convention: empty pattern + empty custom_matrix → `set_matrix` no-ops. So the
  matrix clear path is ALSO latently broken on hardware.

Decision: `_stop_lane(LANE_MATRIX)` will emit the established clear convention
`_send_matrix_command(pattern=None, red=0, green=0, blue=0)` (no new topic), and
I will make the device controller honor that convention by calling
`clear_matrix()` when pattern is empty and no custom matrix is provided. This is
the minimal fix that makes the existing clear mechanism actually blank the
device, consistent with `/api/matrix/clear`.

## Detailed Plan

### Step 1: Base hook (robot-agnostic no-op)
- File: `core/common/task.py`
- Add `def _stop_lane(self, lane: str) -> None: pass` to `TaskExecutorBase`.
- Expected: existing fake/recording executors keep working unchanged.

### Step 2: Funnel physical stops through `_cancel_task`
- File: `core/common/task.py` (`_cancel_task`, ~line 289)
- Capture the slot-lanes the task occupies BEFORE `_clear_task` vacates them,
  then call `self._stop_lane(ln)` for each. Lane-scoped automatically:
  - bare stop cancels DRIVE only → only DRIVE stop emitted
  - targeted running stop → `_cancel_task` → that task's lanes stopped
  - halt loops `_cancel_task` over all running → all occupied lanes stopped
- Queued-task cancel in `_cancel_by_id` has no lane slot → nothing emitted
  (correct). No change needed there.

### Step 3: Sphero subclass override
- File: `core/sphero/sphero_task_executor.py` (`SpheroTaskExecutorBase`, so both
  topic + direct executors inherit; `_send_*` are declared/reachable here).
- Implement `_stop_lane(self, lane)` mapping:
  - `LANE_DRIVE` → `self._send_stop_command()`
  - `LANE_LED` → `self._send_led_command(0, 0, 0)` (off convention)
  - `LANE_MATRIX` → `self._send_matrix_command(pattern=None, red=0, green=0, blue=0)`
    (the `/api/matrix/clear` convention)
  - `LANE_CONFIG` → no-op
- `LANE_*` already imported in this module.

### Step 4: Bare-stop double-stop decision
- Choice: (a) KEEP the fall-through. The bare-stop sentinel still promotes and
  `execute_stop` runs, but it is no longer the source of the physical DRIVE
  stop — `_cancel_task(drive)` now emits it. Need EXACTLY ONE physical drive
  stop (existing `test_bare_stop_cancels_drive_only` asserts `('stop', {}) in`
  sends; a stricter count test will be added). To guarantee exactly one:
  - Verify whether the promoted bare-stop sentinel currently also emits a stop
    via `execute_stop`. If it does, that would double-emit. So choose (b):
    CONSUME the leading bare stop after `_cancel_task(drive)` (pop it + complete
    via `_complete_sentinel`) since the physical stop is now handled by
    cancellation. This keeps exactly one DRIVE stop and removes the now-redundant
    promote→execute_stop tick. Cleaner and test-stable.
- Final decision recorded after reading `execute_stop` in `sphero_task_handlers`.

### Step 5: Make the device controller honor the matrix clear convention
- File: `sphero_instance_device_controller_node.py` (`matrix_callback`, ~358)
- When `pattern` is empty AND no `custom_matrix` provided, call
  `self.sphero.clear_matrix()` instead of `set_matrix(None,...)` (which no-ops).
- Surgical: only add the clear branch; existing pattern path unchanged.

### Step 6: Tests
- File: `test/test_task_executor.py` (RecordingSphero records `_send_*`).
- Add:
  - panic halt while a drive roll runs → `('stop', {})` in sends (regression)
  - targeted stop of running drive → `('stop', {})` in sends
  - bare stop → EXACTLY ONE `('stop', {})`; no `('led',...)` / `('matrix',...)`
    stop emitted
  - halt with LED+MATRIX+DRIVE running → stop emitted for all three lanes
    (`('stop',{})`, `('led',{0,0,0})`, `('matrix',{None,0,0,0})`)
  - base `_stop_lane` default no-op: a `RecordingExecutor` halt does not raise
- Keep ALL existing tests green (notably `test_bare_stop_cancels_drive_only`,
  `test_halt_scope_all_cancels_everything`, `test_targeted_stop_by_name`).

## Expected Outcomes
- Halt / targeted-stop / bare-stop all emit the correct lane-scoped physical
  stop. Indefinite roll halts physically.
- Matrix clear convention actually blanks the device.

## Potential Risks & Considerations
- Double-stop on bare stop if fall-through kept; mitigated by consuming the
  sentinel (Step 4b).
- Device-node matrix change is verified by inspection only (no hardware in CI).
- Must not change lane semantics elsewhere; `_stop_lane` is additive.

## Testing Plan
- `python3 -m py_compile` changed files.
- `python3 -m pytest test/test_task_executor.py -q` → 0 failed (report count).
- `colcon build --packages-select sphero_instance_controller`.

## Approval Status
- [ ] Waiting for user approval
- [ ] Approved
- [ ] Executed
