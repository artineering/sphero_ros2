# Add `calibrate_compass` Sphero Task Type (End-to-End)

**Created:** 2026-06-06T19:35:59Z
**Status:** Pending Approval

## Task Description
Implement a new one-shot Sphero task type `calibrate_compass` and wire it through the
full pipeline: task -> handler -> executor `_send_*` -> ROS topic -> device-controller
callback -> `Sphero.calibrate_compass()` -> `spherov2` `SpheroEduAPI.calibrate_compass()`.
It is a DRIVE-lane owner (the robot physically spins), not a modifier. Mirror existing
one-shot command patterns (`spin` for the topic/device wiring, `reset_aim` for the
device-side wrapper + callback shape).

## Analysis (current state, verified by reading the code)

- `core/sphero/sphero.py` `reset_aim()` (line 295) is the wrapper template: try/except,
  `print` on error, return `bool`. `calibrate_compass()` takes no args and is BOLT-only.
- `core/sphero/sphero_task_executor.py` holds the abstract `_send_*` interface (lines
  153-187), `MODIFIER_TASK_TYPES` (line 36), `TASK_LANES` (line 46), and
  `_register_default_handlers` (line 103).
- `core/sphero/sphero_task_handlers.py` has the `TaskType` enum (line 38) and the
  `(executor, task) -> bool` handlers. `execute_stop`/`execute_heading` are the
  simplest one-shot templates (call one `_send_*`, return True).
- `core/sphero/topic_task_executor.py` does **NOT** create publishers itself. It calls
  `self.command_publisher(<command_name>, params)` (a callback). The actual topic
  routing lives in the task controller node's `publish_command()` (line 226) +
  `_create_publishers()` (line 121). `_send_spin_command` is the no-/few-arg template:
  `self.command_publisher('spin', {...})`.
- `core/sphero/direct_task_executor.py` implements each `_send_*` by calling
  `self.sphero.*` directly.
- `sphero_instance_device_controller_node.py`: subscriptions created in
  `_create_subscribers()` (line 231); `reset_aim_sub` at line 263, `reset_aim_callback`
  at line 463 (calls `self.sphero.reset_aim()`, logs).
- `test/test_task_executor.py`: `RecordingSphero` (line 103) records every `_send_*`
  call into `ex.sends`; `lane_task()` (line 664) resolves lanes via `lanes_for`;
  lane/modifier assertions follow `test_modifier_empty_lane_and_never_reserves`
  (line 1103). Current suite: **77 tests**.

### DEVIATION FROM SPEC (important — must flag)
The spec (file #4) says: in `topic_task_executor.py`, create a publisher for
`{topic_prefix}/calibrate_compass` in `__init__` and publish a `String` with `'{}'`,
mirroring a `reset_aim` publisher.

That does not match this codebase:
1. `TopicTaskExecutor` has no publishers and no `topic_prefix`; it only has a
   `command_publisher` callback. The publishers live in the **task controller node**.
2. There is **no** `reset_aim` publisher in `TopicTaskExecutor` and **no**
   `_send_reset_aim_command` anywhere — `reset_aim` never flows through the
   task/executor pipeline; it is published directly by the node's UI/reset path.

So the faithful adaptation that preserves the spec's INTENT (publish an empty-payload
command on a dedicated `{prefix}/calibrate_compass` topic that the device controller
subscribes to) is to mirror the real **`spin`** wiring:
- `TopicTaskExecutor._send_calibrate_compass_command` -> `self.command_publisher('calibrate_compass', {})`.
- Add a `calibrate_compass_pub` to the task controller node's `_create_publishers()`
  and a branch in `publish_command()` routing `'calibrate_compass'` to it.

This still publishes a `String` with an empty JSON object (`json.dumps({})` == `'{}'`)
to `{topic_prefix}/calibrate_compass`, exactly as the device controller will expect.

> NOTE: file #4 in the spec does not mention editing the task controller node, but in
> this architecture that edit is REQUIRED for the topic path to actually reach a
> publisher (otherwise `publish_command` logs "Unknown command topic"). This is the
> only structural deviation; I will implement it this way unless you prefer otherwise.

## Detailed Plan

### Step 1: `core/sphero/sphero.py` — add wrapper
- Add `calibrate_compass(self) -> bool` right after `reset_aim` (after line 325),
  mirroring its structure:
  - Guard `if not hasattr(self.api, 'calibrate_compass'): print(...); return False`
    (non-BOLT graceful path).
  - `self.api.calibrate_compass()` inside try; return True.
  - `except Exception as e: print(f"Error calibrating compass: {e}"); return False`.
- Verify: method present, no syntax error.

### Step 2: `core/sphero/sphero_task_handlers.py` — enum + handler
- Add `CALIBRATE_COMPASS = "calibrate_compass"` to `TaskType` (after `JUMPING_BEAN`).
- Add `def execute_calibrate_compass(executor, task) -> bool:` near the other one-shot
  handlers; docstring notes it triggers a BOLT compass calibration (robot spins);
  body: `executor._send_calibrate_compass_command(); return True`.
- Verify: handler importable.

### Step 3: `core/sphero/sphero_task_executor.py` — abstract method, lane, registration
- Add abstract `_send_calibrate_compass_command(self)` raising NotImplementedError,
  next to the other `_send_*` abstracts (after `_send_spin_command`, line 175 area).
- Add `'calibrate_compass': frozenset({LANE_DRIVE}),` to `TASK_LANES`
  (do NOT add to `MODIFIER_TASK_TYPES`).
- Register: `self.register_handler('calibrate_compass', h.execute_calibrate_compass)`.
- Verify: `lanes_for('calibrate_compass') == frozenset({LANE_DRIVE})`,
  `is_modifier('calibrate_compass') is False`.

### Step 4: `core/sphero/direct_task_executor.py` — direct impl
- Add `def _send_calibrate_compass_command(self):` -> `self.sphero.calibrate_compass()`,
  mirroring the other `_send_*` direct methods.

### Step 5: `core/sphero/topic_task_executor.py` — topic impl
- Add `def _send_calibrate_compass_command(self):` ->
  `self.command_publisher('calibrate_compass', {})` (mirrors `_send_spin_command`).

### Step 6: `sphero_instance_task_controller_node.py` — route the new command topic
  (REQUIRED by this architecture; see DEVIATION note)
- In `_create_publishers()`: add `self.calibrate_compass_pub = self.create_publisher(
  String, f'{self.topic_prefix}/calibrate_compass', 10)`.
- In `publish_command()`: add
  `elif topic_name == 'calibrate_compass': self.calibrate_compass_pub.publish(msg)`.
- (Log line in `_log_initialization` optional; will add one publishing-topic line to
  match the existing list style.)

### Step 7: `sphero_instance_device_controller_node.py` — subscription + callback
- In `_create_subscribers()` next to `reset_aim_sub` (line 263): add
  `self.calibrate_compass_sub = self.create_subscription(String,
  f'{self.topic_prefix}/calibrate_compass', self.calibrate_compass_callback, 10)`.
- Add `calibrate_compass_callback(self, msg)` next to `reset_aim_callback`:
  - log "Calibrating compass (robot will spin)..."
  - `# NOTE: this BLOCKS this callback thread until calibration completes (~seconds).`
  - `success = self.sphero.calibrate_compass()`
  - log "Compass calibrated" on success else
    "Compass calibration not supported (BOLT only) or failed".

### Step 8: Tests in `test/test_task_executor.py`
- Add `_send_calibrate_compass_command` to `RecordingSphero` so the concrete class
  stays instantiable: `def _send_calibrate_compass_command(self): self._record('calibrate_compass')`.
- Add a lane/modifier test:
  `lanes_for('calibrate_compass') == frozenset({LANE_DRIVE})` and
  `is_modifier('calibrate_compass') is False`.
- Add a handler test: `ex = RecordingSphero(); ex.add_task(lane_task('calibrate_compass'));
  ex.process_tasks(); assert 'calibrate_compass' in _send_names(ex)` and the task
  completes (one-shot, returns True). This mirrors `test_stop` / the lane tests and
  records the `_send_*` call exactly once via `RecordingSphero`.

## Expected Outcomes
- `colcon build --packages-select sphero_instance_controller` is clean.
- `pytest test/test_task_executor.py -q` passes; new total = **79** (77 + 2 new tests;
  the `RecordingSphero` method addition is not itself a test).
- A `{"task_type":"calibrate_compass"}` task: promotes into the DRIVE lane ->
  `execute_calibrate_compass` -> `_send_calibrate_compass_command` ->
  topic path publishes `'{}'` to `{prefix}/calibrate_compass` (or direct path calls
  `sphero.calibrate_compass()`) -> device `calibrate_compass_callback` ->
  `self.sphero.calibrate_compass()` -> `api.calibrate_compass()`.

## Potential Risks & Considerations
- The device callback BLOCKS (spherov2 spins until the magnetometer notify fires). For
  v1 this is accepted; flagged in code comment + operator log. It will stall that node's
  executor callback group for a few seconds — acceptable per spec.
- Non-BOLT units: `hasattr` guard returns False gracefully; callback logs the BOLT-only
  message. No exception leaks.
- DRIVE-lane assignment means a running `roll` will queue/block the calibration in the
  same lane (correct, since both physically drive the robot).
- The task-controller-node edit is outside the spec's listed files but mandatory here;
  flagged above.

## Testing Plan
1. `cd /home/svaghela/sphero_ros2 && colcon build --packages-select sphero_instance_controller`
2. `python3 -m pytest src/sphero_instance_controller/test/test_task_executor.py -q`
3. Static trace of the call path (report verbatim in final summary).

## Approval Status
- [ ] Waiting for user approval
- [ ] Approved
- [ ] Executed
```
