# Synchronized Task Start (start_offset / now)

**Created:** 2026-05-30T00:00:00Z
**Status:** Pending Approval

## Task Description

Upgrade the Sphero task controller so a broadcast can send the SAME task to all
deployed Spheros at once with a shared start reference, causing them to begin
executing in sync. The task JSON gains two OPTIONAL fields:

```json
{ "task_type": "...", "parameters": {...},
  "now": 1780175594.74,   // coordinator's time.time() at send (epoch seconds)
  "start_offset": 5.0 }   // seconds
```

Controller computes `target = now + start_offset` (absolute epoch in the
NTP-synced wall-clock frame) and does NOT begin executing the task until its own
`time.time() >= target`. Absent fields or a past target => execute immediately
(full back-compat).

## Analysis (verified against current code)

- `sphero_instance_task_controller_node.py::task_callback` (line 276) parses the
  JSON, builds `TaskDescriptor(task_id, task_type, parameters)` and calls
  `self.task_executor.add_task(task)`. This is the single place that sees `now`
  and `start_offset`.
- `TaskDescriptor` is a dataclass in
  `core/common/task.py` (lines 27-50): fields `task_id, task_type, parameters,
  status, created_at, started_at, completed_at, error_message` plus `to_dict()`.
- The executor lifecycle lives in `TaskExecutorBase.process_tasks()` in the SAME
  file `core/common/task.py` (lines 92-134). (Note: `TaskStatus`/`TaskExecutorBase`
  are defined in `core/common/task.py`; the task controller imports
  `TaskDescriptor, TaskStatus` from there and `TopicTaskExecutor` from
  `core/sphero/topic_task_executor.py`, which subclasses through
  `SpheroTaskExecutorBase` down to `TaskExecutorBase`. So editing
  `process_tasks` in the base covers the topic executor used by the node.)
- `process_tasks()` promotes the head of the queue exactly once per tick:
  lines 112-115 `if self.current_task is None and self.task_queue:` -> `pop(0)`,
  set `RUNNING`, set `started_at`. **This is the exact promotion point to gate.**
  It does NOT spin/block; it returns after one tick and the node's 100 Hz timer
  (`task_execution_loop`, 0.01s) re-invokes it. So gating = "skip promotion this
  tick, leave the task at the head of the queue, return None (idle)".
- `task_execution_loop` (node, line 349) diffs `previous_task` vs `current_task`
  and publishes status on transitions. A gated task is still in
  `task_queue[0]`, `current_task` stays None => loop stays idle, no churn. Good.
- Websocket server `/api/task` (line 669) -> `publish_task_command` (line 451)
  does `json.dumps(data)` of the raw POST body straight to `<prefix>/task`.
  Extra fields pass through untouched. **No change needed.**
- State-machine controller publishes to `<prefix>/task` but never sets `now` /
  `start_offset`, so its tasks hit the absent-field path => immediate.
  **No change needed.**

## Detailed Plan

### Step 1: Add `start_at` to TaskDescriptor

- File: `src/sphero_instance_controller/sphero_instance_controller/core/common/task.py`
- Add one optional field to the dataclass (after `created_at`, before the
  runtime-populated `started_at` so default ordering stays clean):
  ```python
  start_at: Optional[float] = None   # absolute epoch; None => start immediately
  ```
- Add it to `to_dict()` so status messages and the UI can read the scheduled
  instant:
  ```python
  'start_at': self.start_at,
  ```
- Semantics: `start_at is None` => no gating (immediate). `start_at` set =>
  executor must wait until `time.time() >= start_at` to promote.

### Step 2: Gate promotion in TaskExecutorBase.process_tasks()

- File: same `core/common/task.py`, function `process_tasks` (lines 92-134).
- Change ONLY the promotion block (currently lines 112-115). Add a not-yet-due
  guard so a future-dated head stays pending instead of being promoted:
  ```python
  # Promote next pending task to running, unless it is scheduled for a
  # future shared-start instant (synchronized start).
  if self.current_task is None and self.task_queue:
      head = self.task_queue[0]
      if head.start_at is None or time.time() >= head.start_at:
          self.current_task = self.task_queue.pop(0)
          self.current_task.status = TaskStatus.RUNNING
          self.current_task.started_at = time.time()
  ```
- Behavior / edge cases this covers:
  - `start_at is None` (absent fields, SM tasks): promotes immediately —
    identical to today.
  - `start_at` already in the past (late arrival, `start_offset <= 0`):
    `time.time() >= start_at` true on the first tick => immediate. No special
    case needed.
  - Future `start_at`: head stays in `task_queue[0]`, `current_task` stays None,
    `process_tasks` returns None each tick (idle), node loop does no work. When
    `time.time()` crosses `start_at`, the very next tick (<=10 ms later, 100 Hz
    timer) promotes it. Sub-tick jitter <=10 ms across units — fine for this use.
  - Clock concerns: pure `time.time()` comparison, no monotonic clock — this is
    intentional, the shared wall-clock (NTP) frame is exactly what we want. If a
    unit's clock steps backward after arrival, the task simply waits a bit
    longer; acceptable and self-correcting.
- **Head-of-queue gating (documented behavior):** a future-dated head blocks
  later queued tasks until it fires. This is the simple, intended model: the
  broadcast path queues a single synchronized task onto an idle controller, so
  head gating is sufficient. Tasks queued behind a gated head wait — by design,
  preserving FIFO order. (Note: the existing `stop` cancel-sentinel check at
  lines 101-109 runs only when `current_task is not None`; a gated task has no
  current_task, so a `stop` queued behind a waiting task does not pre-empt it —
  consistent with current "stop cancels the in-flight task" semantics. Will note
  this in the plan discussion; no behavior change requested.)

### Step 3: Compute start_at in task_callback

- File: `src/sphero_instance_controller/sphero_instance_controller/sphero_instance_task_controller_node.py`
- In `task_callback` (line 276), after building the `TaskDescriptor` (lines
  302-306), compute and assign `start_at` from the optional fields:
  ```python
  # Synchronized start (optional): anchor to coordinator's `now` so units
  # that receive late still fire at the same absolute target.
  now = task_data.get('now')
  start_offset = task_data.get('start_offset')
  if now is not None and start_offset is not None:
      target = float(now) + float(start_offset)
      if target > time.time():
          task.start_at = target
      # else: target already past => leave start_at None (immediate)
  ```
- Rationale for assigning on the descriptor rather than passing through
  `add_task`: keeps `add_task`'s signature untouched (used by SM path and tests)
  and matches the existing pattern of mutating the descriptor before queueing.
- Logging: extend the existing "Added task ..." info log to mention the schedule
  when `task.start_at` is set, e.g. append
  `f' scheduled in {task.start_at - time.time():.2f}s'`. Minimal, single line.

### Step 4: Status reporting for the waiting state (minimal)

- Goal: let the UI show a countdown for a gated task without adding a new status
  enum value or new publisher.
- The cleanest minimal addition that reuses `publish_task_status`:
  - In `task_callback`, the existing call `self.publish_task_status(task)`
    (line 318) already fires on arrival. Because Step 1 adds `start_at` to
    `to_dict()`, that status message now carries `start_at` while the task's
    `status` is still `"pending"`. The UI computes the countdown as
    `start_at - now`. No new status value, no new code path.
- That covers the requirement ("publish a scheduled/waiting status so the UI can
  show a countdown") with zero extra publishing: a pending task with a non-null
  `start_at` IS the "waiting/scheduled" signal. When it later promotes to
  RUNNING, `task_execution_loop` publishes the transition as it does today.
- Optional (only if you want an explicit label): add a `TaskStatus.SCHEDULED`
  enum and set it on the gated head — but this touches more surface (enum,
  to_dict consumers, the loop's transition diff). **Recommendation: do NOT add
  the enum.** The `pending + start_at` combination is sufficient and surgical.

### Step 5: Tests

- File: `src/sphero_instance_controller/test/test_task_executor.py` (exists).
- Add unit tests at the executor level (no ROS needed):
  - Task with `start_at = time.time() + long`: `process_tasks()` returns None and
    leaves the task in `task_queue` (not promoted) across several ticks.
  - Task with `start_at` in the past: promoted on first tick.
  - Task with `start_at = None`: promoted on first tick (regression guard).
  - `to_dict()` includes `start_at`.
- Will read the existing test file first to match its style/fixtures before
  adding.

## Files Touched (summary)

- `core/common/task.py` — add `start_at` field + `to_dict` entry; gate promotion
  in `process_tasks` (one block).
- `sphero_instance_task_controller_node.py` — compute `start_at` from
  `now`/`start_offset` in `task_callback`; one extra log line.
- `test/test_task_executor.py` — gating + back-compat tests.
- NO change: websocket server, state-machine controller, `topic_task_executor.py`,
  `sphero_task_executor.py`, message contract on the wire.

## Expected Outcomes

- Broadcast with `now`+`start_offset` => all NTP-synced units begin the task at
  the same absolute epoch `target` (within one 10 ms tick).
- Tasks without the fields, and all SM-generated tasks => immediate (unchanged).
- A scheduled-but-not-started task is observable on `<prefix>/task/status` as
  `status: "pending"` with a non-null `start_at` for UI countdown.

## Potential Risks & Considerations

- **Clock sync dependency:** correctness relies on NTP across workers (verified
  per project notes). Off-sync clocks => off-sync starts; out of scope to fix
  here, but worth a one-line code comment.
- **Head-of-queue blocking:** a future-dated head holds the queue. Documented and
  intended for the single-broadcast-task use case. If a future need requires
  per-task independent scheduling, that's a larger change (re-order/scan queue) —
  explicitly deferred.
- **Type safety:** `now`/`start_offset` cast via `float(...)`; malformed values
  raise inside the existing `try/except` in `task_callback` and are logged — no
  crash, task simply not queued. Acceptable.
- **No new dependencies, no message/IDL changes** (these are String/JSON topics).

## Testing Plan

- Unit: run `colcon test --packages-select sphero_instance_controller` (or pytest
  on `test_task_executor.py`) — new gating tests pass, existing pass.
- Build: `colcon build --packages-select sphero_instance_controller` then
  `source install/setup.bash`.
- Manual (single unit):
  - Immediate (back-compat):
    `ros2 topic pub --once /sphero/<name>/task std_msgs/String '{data: "{\"task_type\":\"...\",\"parameters\":{}}"}'`
    => starts now.
  - Scheduled: publish with
    `"now": <python time.time()>, "start_offset": 5.0` and watch
    `ros2 topic echo /sphero/<name>/task/status` show `pending` + `start_at`,
    then a `running` transition ~5 s later.
  - Past target: `start_offset: -2.0` => immediate.
- Manual (sync, multi-unit): broadcast the same payload to several units' `/task`
  topics; confirm motion begins together (visual / status timestamps).

## Approval Status
- [x] Waiting for user approval
- [ ] Approved
- [ ] Executed
