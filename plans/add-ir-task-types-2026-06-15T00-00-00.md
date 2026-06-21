# Add IR Robot-to-Robot Task Types to Sphero Task Pipeline

**Created:** 2026-06-15T00:00:00
**Status:** Pending Approval

## Task Description
Surface the six already-implemented `Sphero` core IR methods
(`start_ir_broadcast/follow/evade`, `stop_ir_broadcast/follow/evade`) as task
types so the webserver can dispatch them per-unit:

- `ir_broadcast` {near:int, far:int}
- `ir_follow` {near:int, far:int}
- `ir_evade` {near:int, far:int}
- `ir_broadcast_stop` {}
- `ir_follow_stop` {}
- `ir_evade_stop` {}

## Analysis
Tracing an existing task end-to-end (`set_led`, `roll`, `calibrate_compass`):

1. `TaskType` enum in `sphero_task_handlers.py` (informational catalog).
2. Handler fns `(executor, task) -> bool` in `sphero_task_handlers.py`.
3. Registered in `SpheroTaskExecutorBase._register_default_handlers()`.
4. Lane assignment in `TASK_LANES` (`sphero_task_executor.py`).
5. Abstract `_send_*` in `SpheroTaskExecutorBase`.
6. `DirectTaskExecutor._send_*` -> `self.sphero.<method>()` directly.
7. `TopicTaskExecutor._send_*` -> `self.command_publisher(topic, params)`.
8. Task controller node (`sphero_instance_task_controller_node.py`):
   creates publishers and routes in `publish_command()`.
9. Device controller node (`sphero_instance_device_controller_node.py`):
   subscribes per-topic and the callback calls `self.sphero.<method>()`.

TopicTaskExecutor IS used: the task controller node instantiates it
(line 74) and is the running node that consumes web-dispatched tasks. So the
full topic path (publisher + subscriber + dispatch) must be wired.

Persistence: `execute_roll` with `duration<=0` emits the command once
(guarded by a `_rolling` flag) and returns `False` forever so it stays
resident and holds its lane until cancelled. The IR start handlers mirror
this. Stop handlers emit once and return `True` (like `execute_stop`).

Topic transport choice: rather than 6 new topics, use ONE `ir` command
topic carrying an `action` field (`broadcast`/`follow`/`evade`/`*_stop`)
plus `near`/`far` — mirroring how `motion` and `collision` multiplex
start/stop on one topic. This is the minimal, consistent wiring.

## Detailed Plan

### Step 1: TaskType enum entries
- File: `core/sphero/sphero_task_handlers.py`
- Add six enum members (IR_BROADCAST, IR_FOLLOW, IR_EVADE, and the three
  _STOP) to the `TaskType` catalog.

### Step 2: Handlers
- File: `core/sphero/sphero_task_handlers.py`
- `execute_ir_broadcast/follow/evade`: read `near`/`far` (default 0),
  emit `_send_ir_*_command(near, far)` once guarded by an `_ir_active`
  flag, return `False` (resident).
- `execute_ir_broadcast_stop/follow_stop/evade_stop`: emit
  `_send_ir_*_stop_command()` once, return `True`.

### Step 3: Registration
- File: `core/sphero/sphero_task_executor.py`
- Register all six in `_register_default_handlers()`.

### Step 4: Lane assignment (TASK_LANES)
- File: `core/sphero/sphero_task_executor.py`
- `ir_broadcast` -> `frozenset({LANE_CONFIG})` (does not drive).
- `ir_follow`, `ir_evade` -> `frozenset({LANE_DRIVE})` (firmware drives
  the robot; must own the drive lane so a concurrent roll/heading can't
  fight them).
- `ir_broadcast_stop` -> `frozenset({LANE_CONFIG})` (mirror start).
- `ir_follow_stop`, `ir_evade_stop` -> `frozenset({LANE_DRIVE})` (mirror
  start so the stop replaces/releases the resident follow/evade owner).

### Step 5: Abstract methods
- File: `core/sphero/sphero_task_executor.py`
- Add abstract `_send_ir_broadcast_command(near, far)`,
  `_send_ir_follow_command(near, far)`, `_send_ir_evade_command(near, far)`,
  and `_send_ir_broadcast_stop_command()`, `_send_ir_follow_stop_command()`,
  `_send_ir_evade_stop_command()` (raise NotImplementedError).

### Step 6: DirectTaskExecutor
- File: `core/sphero/direct_task_executor.py`
- Implement the six methods calling `self.sphero.start_ir_broadcast(...)`
  etc. directly.

### Step 7: TopicTaskExecutor
- File: `core/sphero/topic_task_executor.py`
- Implement the six methods publishing to topic `'ir'` with payload
  `{'action': 'broadcast'|'follow'|'evade'|'broadcast_stop'|..., 'near', 'far'}`.

### Step 8: Task controller node (publisher side)
- File: `sphero_instance_task_controller_node.py`
- `_create_publishers`: add `self.ir_pub` on `{topic_prefix}/ir`.
- `publish_command`: add `elif topic_name == 'ir':` branch -> `self.ir_pub.publish(msg)`.
- Add the `/ir` line to the `_log_initialization` publishing list.

### Step 9: Device controller node (subscriber + dispatch)
- File: `sphero_instance_device_controller_node.py`
- Add `self.ir_sub` subscription on `{topic_prefix}/ir` -> `self.ir_callback`.
- Add `ir_callback(msg)`: parse `action`/`near`/`far`, dispatch to the
  matching `self.sphero.start_ir_*` / `stop_ir_*` method.
- Add `/ir` to the subscriber log list.

## Expected Outcomes
- Six new task types dispatchable via the web UI per unit.
- DirectTaskExecutor path works for the device controller's direct use.
- TopicTaskExecutor path (task controller -> device controller) fully wired.
- IR follow/evade own the drive lane; broadcast is on config lane.

## Potential Risks & Considerations
- `near`/`far` are clamped 0-7 inside the Sphero core methods, so handlers
  pass through raw ints.
- IR start handlers are resident; they are released by their `_stop`
  counterpart or any task that preempts their lane (scheduler will call
  `_stop_lane(LANE_DRIVE)` -> `_send_stop_command()` for follow/evade on
  cancel, which stops motors but does not turn off IR follow/evade firmware).
  This matches existing `roll` behavior; the explicit `_stop` task is the
  clean release. Noted for the user.
- Single `ir` topic multiplexes all six (consistent with `motion`/`collision`).

## Testing Plan
- `colcon build --packages-select sphero_instance_controller`.
- Verify import / no syntax errors.
- Live test via web UI (per user; do not commit).

## Addendum (approved): IR-aware lane cleanup

A bare motor stop does NOT stop the ir_follow/ir_evade firmware (it re-drives on
the next IR cycle and fights a preempting drive task). So on cancel/preempt of
an `ir_follow`/`ir_evade` DRIVE owner, the executor must emit the matching IR
stop (`stop_ir_follow`/`stop_ir_evade`), not just a motor stop.

Implementation: `SpheroTaskExecutorBase._cancel_task` is overridden to consult
`_IR_DRIVE_CANCEL_STOP` ({'ir_follow': '_send_ir_follow_stop_command',
'ir_evade': '_send_ir_evade_stop_command'}); if the cancelled task is one of
these it issues the IR stop, then delegates to `super()._cancel_task` (which
runs the existing `_stop_lane(DRIVE)` motor stop + slot vacate). All cancel
paths (panic halt, targeted stop, bare stop, lane preemption) funnel through
`_cancel_task`, so this covers them all consistently. `ir_broadcast` does not
drive and needs no special cleanup beyond `ir_broadcast_stop`.

## Approval Status
- [x] Waiting for user approval
- [x] Approved
- [x] Executed
