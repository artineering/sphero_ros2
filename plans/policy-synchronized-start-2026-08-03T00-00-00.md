# Policy-level synchronized start (FleetPolicy `now` + `start_offset`)

**Created:** 2026-08-03T00:00:00
**Status:** Pending Approval

## Task Description
Make synchronized-start available for EVERY fleet-policy cue, not just blink. Add
policy-level `now` (coordinator epoch) and `start_offset` (seconds) to
`FleetPolicy.msg` so that every member Sphero begins its cue at the SAME shared
absolute instant (`now + start_offset`), regardless of when each unit receives
the broadcast or loads its state-machine config. Absent / zero timing fields
mean "start immediately" (backward compatible with today's broadcasters and the
webapp).

Reuse the DIRECT-task-path idiom already in the codebase; do NOT invent a
parallel timing mechanism. Layering rule (project_control_stack_layering):
ticking stays a state machine; no timing is smuggled into the device controller.

## Analysis (grounded in current code)

### The mechanism that already exists (direct task path) — REUSE it
- `_compute_start_at(now, start_offset)` resolves an absolute epoch anchored to
  the SENDER's clock: `target = float(now) + float(start_offset)`, returns None
  for immediate (or when the target is already past). File:
  `src/sphero_instance_controller/sphero_instance_controller/sphero_instance_task_controller_node.py:352-362`.
- `TaskDescriptor.start_at: Optional[float]` — absolute epoch, `None => start
  immediately`: `src/sphero_instance_controller/sphero_instance_controller/core/common/task.py:57`.
- The executor GATE on `start_at` lives in `_promote_due`:
  `not_due = t.start_at is not None and now < t.start_at`
  (`core/common/task.py:283`), applied per lane before promotion (and again in
  the Phase-4 re-promote, `:201-204`). A task with a future `start_at` waits in
  the queue until the shared instant, then promotes — so all units release
  together.
- The direct path stamps `start_at` in `_build_task` (`:390-391`) and threads
  `now`/`start_offset` at `task_callback:428` (single) and `_handle_bundle:528`
  (bundle). `_compute_start_at` reads them via dict `.get()`, so ABSENT keys ->
  `None` -> immediate.

### Where policy cues flow today — and why they DON'T share a clock
`policy_callback` (`:561-602`) fans a `FleetPolicy` out to four sub-appliers.
The cues split into TWO transport paths, which matters for the design:

1. SM-expressed cues (ticking) -> published as SM configs to `sm_config_pub`:
   - Similarity BEHAVIOR **blink** -> `_publish_blink_sm` (`:725-760`), a
     two-state timer ping-pong on the surface lane (led/matrix).
   - **Proximity** -> `_publish_proximity_sm` (`:848-880`), a two-state timer
     ping-pong on the `drive` lane.

2. One-shot TASK cues -> enqueued straight onto this unit's executor via
   `_enqueue_policy_task` (`:719-723`), which calls `_build_task(item, None)` ->
   `start_at = None` (immediate):
   - Similarity IDENTITY **steady** render (`set_led`/`matrix`) at `:637-639`.
   - Similarity BEHAVIOR **spin** -> `_apply_spin` (`:768-793`), a finite `spin`
     task on the DRIVE lane.
   - Surface-blank tasks (`:634-635`, `:683-684`).

**Correction to the task framing (surfaced honestly):** the brief states "ALL
current cues (similarity blink/spin, proximity) are expressed as per-unit SMs."
That is not what the code shows — only **blink** and **proximity** are SMs;
**steady-identity render** and **spin** are one-shot TASKS enqueued directly
(`:637-639`, `:788-792`). This split is the crux of the design: gating only SM
activation would leave steady-render and spin unsynchronized. See the design
decision below.

### Why the SM path drifts (no shared wall-clock anchor)
When an SM config arrives, `config_callback` (`:198-264`) resolves the lane
(`:241`), calls `sm.configure(config)` (`:248`), then IMMEDIATELY fires the
initial state's tasks via `execute_current_state_tasks(lane)` (`:257`).
`configure()` -> `_build_state_machine` -> `_enter_initial` (depth 0,
`core/sphero/statemachine.py:366-367`) sets `current_state = initial` and
`entry_time = time.time()` (`statemachine.py:434`). Two consequences, both
seeded from *whenever that unit loaded the config*:
- The FIRST cue fire happens at config-load time (`:257`).
- Every subsequent `timer` exit is measured from `entry_time`
  (`statemachine.py:627-634`), so the ping-pong PHASE is seeded from load time
  too.
Different units load at different times -> first fire and timer phase drift ->
cues fall out of phase across the fleet. There is no epoch anywhere on the SM
path.

### Message + lane facts to build on
- `FleetPolicy.msg` today: `header, policy_id, active, members[],
  enable_similarity, enable_proximity, enable_common_fate, SimilarityCue
  similarity, ProximityCue proximity` (`src/multirobot_msgs/msg/FleetPolicy.msg:1-9`).
  No timing fields. Registered in `src/multirobot_msgs/CMakeLists.txt:10-18`.
- `SimilarityCue.msg` uses `float32` for RATES (`blink_hz:13`,
  `spin_duration_s:16`) — fine for small magnitudes, but NOT for an epoch (see
  Decision D1).
- SM controller already has the Phase-2 LANE model: one `StateMachine` per lane
  in `self.state_machines` (`sphero_instance_statemachine_controller_node.py:74-81`),
  ticked every 10 Hz in `update_callback` (`:479-505`); `_resolve_lane` (`:268`)
  maps absent/unknown lane -> reserved `default`. Config carries an optional
  `lane` id (`:241`). All lanes derive from ONE policy broadcast, so one epoch
  covers every lane a policy activates.
- Migration point: the live SM-config publishers omit any epoch and MUST keep
  working. `sphero_instance_websocket_server.py:197` publishes
  `state_machine/config` (webapp path); it will omit the new fields -> immediate
  start unchanged. (The task controller also publishes SM configs; it WILL add
  the epoch when a policy carries one.)
- The `FleetPolicy` producer (webapp/webserver that builds `/fleet/policy`) is
  NOT committed in this repo — no `.py`/`.js` references a `FleetPolicy`
  publisher (only the msg def + the task-controller subscriber). So stamping
  `now`/`start_offset` at broadcast time is an out-of-tree integration point
  (Open Question 1).

## Recommended design (the crux): COMBINATION of (a) + (b)

Because the cues split across two transports, one gate cannot cover them:

- **(a) SM entry-gating** for the SM-expressed cues (blink, proximity). The SM
  controller WITHHOLDS first activation of a lane's machine until wall-clock >=
  the shared epoch. Because `entry_time` is stamped at activation
  (`statemachine.py:434`), gating activation aligns BOTH the first fire AND the
  timer phase for every SM cue — for free, with no change to the StateMachine
  core.

- **(b) `start_at` stamping** for the one-shot TASK cues (steady-identity
  render, spin, blank). Pass the resolved epoch into `_enqueue_policy_task` ->
  `_build_task(item, start_at)` so the EXISTING executor gate
  (`core/common/task.py:283`) does the waiting. Zero new machinery — `_build_task`
  already stamps `start_at` (`:390-391`).

**Why not (a) alone:** steady render + spin never become SMs (`:637-639`,
`:788-792`); they would fire immediately and drift. **Why not (b) alone:** an SM
config is not a task and never passes through `_promote_due`; stamping the
SM-FIRED tasks with `start_at` would gate only their FIRST fire, not the timer
phase, and would fight the SM's own cadence. (a)+(b) apply to DISJOINT task
sources, so there is **no double-gating** (see below).

### No double-gating (explicit)
- SM-fired tasks (built in `_dispatch_tasks_for_state:545-549`) keep
  `start_at = None` — they are gated ONCE, at SM activation, by mechanism (a).
  This step does NOT add `start_at` to SM-fired tasks.
- Policy one-shot tasks (steady/spin/blank) are gated by mechanism (b) only;
  they are never SM-fired.
The two mechanisms never touch the same task, so a cue is gated exactly once.

## Detailed Plan

### Step 1: FleetPolicy.msg — add `now` + `start_offset`
- Add two fields to `src/multirobot_msgs/msg/FleetPolicy.msg`:
  `float64 now` (coordinator epoch, seconds since Unix epoch) and
  `float64 start_offset` (seconds after `now` at which the cue fires).
  Semantics: `now <= 0.0` (unset) OR `start_offset <= 0` => start immediately.
- Append them (do not reorder existing fields) so any positional assumptions in
  out-of-tree producers are unaffected; default 0.0 => immediate (backward
  compatible with every existing broadcast).
- No `CMakeLists.txt` change needed — `FleetPolicy.msg` is already registered
  (`src/multirobot_msgs/CMakeLists.txt:15`).
- **Verify:** `colcon build --packages-select multirobot_msgs` succeeds;
  `ros2 interface show multirobot_msgs/msg/FleetPolicy` lists `float64 now` and
  `float64 start_offset`.

### Step 2: policy_callback — resolve the shared epoch once, thread it down
- In `policy_callback` (`:561`), after confirming membership/active, normalize
  the msg fields to the dict-idiom `_compute_start_at` expects (which checks
  `is not None`): `now = msg.now if msg.now > 0.0 else None`;
  `start_offset = msg.start_offset if msg.start_offset > 0.0 else None`. This is
  the ONE subtlety versus the direct path: msg floats default to `0.0` (never
  `None`), so the immediate-start sentinel must be `> 0.0`, not `is not None`.
- Compute `start_at = self._compute_start_at(now, start_offset)` ONCE and pass
  it into `_apply_similarity(msg.similarity, start_at)` and
  `_apply_proximity(msg.proximity, members, start_at)`. Clears
  (`_clear_similarity`/`_clear_proximity`) need no epoch.
- **Verify:** unit-test that a `FleetPolicy` with `now=0.0` yields
  `start_at=None` (immediate) and one with `now=T, start_offset=3.0` yields
  `T+3.0`; log line shows the resolved delay.

### Step 3: One-shot TASK cues honor `start_at` (mechanism b)
- Thread `start_at` through `_apply_similarity` (`:604`) into every one-shot
  enqueue it performs: the steady render (`:637-639`), the surface-blank tasks
  (`:634-635`), and `_apply_spin` (`:768`).
- Give `_enqueue_policy_task` (`:719`) an optional `start_at` param and pass it
  to `_build_task(item, start_at)` (already stamps `start_at`, `:390-391`).
  Default `None` keeps every current caller immediate.
- Spin: pass `start_at` through `_apply_spin` so the finite DRIVE spin promotes
  only at the shared instant (executor gate, `core/common/task.py:283`).
- Design note: a surface-BLANK on cue-change should fire immediately (clear old
  visual now), while the NEW render waits for the epoch. Keep blanks at
  `start_at=None`; apply `start_at` only to the new render/spin. (Confirm in
  Open Question 3 — the alternative is to blank-at-epoch too.)
- **Verify:** offline, build a steady-identity policy with `start_offset=2s`;
  assert the enqueued `set_led`/`matrix`/`spin` TaskDescriptors carry
  `start_at ≈ now+2`, and that `_promote_due` withholds them until then while a
  disjoint-lane task proceeds.

### Step 4: SM config contract — carry the epoch (mechanism a plumbing)
- In `_publish_blink_sm` (`:725`) and `_publish_proximity_sm` (`:848`), add the
  resolved `start_at` to the published SM config JSON as a single key,
  `"start_at": <float epoch>` (omit the key when `start_at is None`). Prefer the
  pre-resolved absolute epoch over shipping `now`+`start_offset` separately, so
  the SM controller does not re-run offset math and both lanes anchor to the
  identical instant computed once in Step 2.
- Thread `start_at` into `_apply_similarity`'s blink branch (`:640-643`) and
  `_apply_proximity` (`:803`, `:835`).
- **Verify:** offline, assert the JSON published to `sm_config_pub` for a
  timed policy contains `start_at`, and that a policy with no timing omits it.

### Step 5: SM controller — withhold activation until the epoch (mechanism a)
Approach: a per-lane PENDING buffer in
`sphero_instance_statemachine_controller_node.py`. This aligns `entry_time` to
the epoch naturally and needs NO change to the StateMachine core.
- In `config_callback` (`:198`), read `start_at = config.get('start_at')`. If it
  is present AND in the future (`start_at > time.time()`), do NOT call
  `sm.configure()` / `execute_current_state_tasks` yet; instead stash
  `self._pending_config[lane] = (config, start_at)` (new dict keyed like
  `self.state_machines`) and log the scheduled delay. If absent or already past,
  configure immediately exactly as today (backward-compatible path).
- Loading a pending config for a lane REPLACES any prior pending config on that
  lane; a fresh non-timed config or a `clear` control on that lane drops the
  pending entry (so a revoke mid-wait cancels a not-yet-started cue).
- In `update_callback` (`:479`), BEFORE ticking, flush ready pending lanes: for
  each `(config, start_at)` with `time.time() >= start_at`, pop it, call
  `sm.configure(config)` + `execute_current_state_tasks(lane)` (the same two
  calls `config_callback` makes today at `:248,:257`), then let the normal tick
  proceed. This stamps `entry_time` at ~epoch (within one 10 Hz tick, ~<=100 ms),
  so first fire AND timer phase align across units.
- Multi-lane: because Step 2 computes ONE epoch and Steps 4 stamps it on every
  lane's config, a blink-on-`led` + proximity-on-`drive` policy has both lanes
  pending on the SAME `start_at` and both flush on the same tick -> fleet-aligned.
- **Verify:** offline, feed two lane-tagged configs (led + drive) each with the
  same future `start_at`; assert neither activates before the epoch and both
  activate on the first tick at/after it; assert a config with no `start_at`
  activates immediately (current behavior); assert a `clear` on a pending lane
  drops it without ever activating.

### Step 6: Build + verify end-to-end (worktree; do NOT commit)
- Build `multirobot_msgs`, then `sphero_instance_controller`; source install.
- `ros2 interface show` FleetPolicy (Step 1). `py_compile` both controller
  nodes. Offline unit tests from Steps 2-5.
- **Verify:** all offline tests pass; nodes import; no regression in the
  no-timing (immediate) path.

## Decisions

- **D1 — `now` MUST be `float64` (not `float32`).** A Unix epoch is ~1.75e9.
  `float32` carries ~7 significant decimal digits, so at that magnitude the
  representable step is ~128 s — an epoch is unusable in 32 bits. `float64`
  (~15-16 digits) resolves sub-microsecond. `start_offset` is small, but make it
  `float64` too for symmetry with the direct path's Python floats and to keep
  `now + start_offset` in one precision domain. (Contrast the existing
  `float32` RATE fields in `SimilarityCue.msg:13,16`, which are small
  magnitudes and fine.)
- **D2 — Combination (a)+(b), not either alone.** Cues split across SM-config
  and one-shot-task transports (Analysis); a single gate cannot cover both.
  Justification and no-double-gating argument above.
- **D3 — Ship a resolved absolute `start_at` on the SM config**, not
  `now`+`start_offset`. The epoch is computed once in `policy_callback`; both
  lanes and the executor tasks then anchor to the identical instant, and the SM
  controller stays offset-math-free.
- **D4 — Immediate-start sentinel is `now <= 0.0`** (msg floats default to 0.0,
  never None), normalized in `policy_callback` before reusing the existing
  `_compute_start_at`. Keeps ONE resolution function for both paths.
- **D5 — Gate SM activation via a per-lane pending buffer in the controller,
  not by changing StateMachine core.** Stamping `entry_time` at flush time
  aligns first fire and timer phase together; the core stays untouched and the
  `default`-lane / websocket path is unaffected.

## Open Questions

1. **FleetPolicy producer (out of tree).** No committed node publishes
   `/fleet/policy`; the webapp/webserver that builds it must stamp
   `now = time.time()` at broadcast and expose a `start_offset` control (the
   webapp already has a task-bundle `start_offset` UI, `static/js/app.js:1314`).
   Confirm which component owns the `FleetPolicy` publisher so `now` is stamped
   on the coordinator's clock. Until then, `now=0.0` -> immediate (no behavior
   change).
2. **Default `start_offset` for policies.** Direct-task broadcasts default to a
   countdown (`app.js` start-in-Ns). Should policies default to immediate
   (`0`) or to a small lead (e.g. 2-3 s) so late-joining units reliably make the
   window? Propose default 0 (immediate) with an operator-set offset for
   synchronized demos.
3. **Blank-on-change timing.** Should the surface-blank when a cue changes fire
   immediately (proposed, clears the old visual now) or wait for the epoch with
   the new render? Proposed: blank now, render at epoch.
4. **Clock skew budget.** With NTP the residual skew is typically < a few ms;
   the SM flush granularity is one 10 Hz tick (~<=100 ms). Confirm a
   `start_offset` floor (e.g. >= 0.5 s) so the target is comfortably in the
   future for the last-arriving unit (mirrors `_compute_start_at`'s
   `target > time.time()` guard, `:360`).

## Testing Plan

### Offline unit tests (no hardware)
- **Epoch resolution:** `_compute_start_at(None|0.0, ...)` -> None;
  `_compute_start_at(T, 3.0)` -> `T+3.0`; past target -> None. Include the msg
  normalization (`now=0.0` -> immediate) from Step 2.
- **Executor gate (mechanism b):** a `TaskDescriptor` with future `start_at`
  stays in `task_queue` across `process_tasks()` ticks until the instant, then
  promotes; a disjoint-lane task promotes meanwhile (asserts no cross-lane
  block). Reuses `core/common/task.py:283`.
- **SM entry-gating (mechanism a):** drive the SM controller's pending-buffer
  logic with two lane-tagged configs (`led` + `drive`) sharing one future
  `start_at`: assert neither activates before the epoch, both activate on the
  first tick at/after it (so both `entry_time`s land within one tick -> phase
  aligned), a no-`start_at` config activates immediately, and a `clear` on a
  pending lane cancels it.

### Bench test (fleet, no commit)
- Publish a `FleetPolicy` with `now = time.time()`, `start_offset ≈ 3.0`,
  `enable_similarity` blink (and separately proximity).
- Watch across units: `sphero/<name>/state_machine/status` (per-lane
  `current_state`, `time_in_state`) and `sphero/<name>/task`. Confirm the first
  fire and the ping-pong phase are simultaneous across units (within NTP skew +
  one 10 Hz tick), NOT staggered by receive time.
- Repeat with steady-identity + spin (`start_offset ≈ 2.0`): confirm the
  `set_led`/`matrix` render and the `spin` task promote at the same instant on
  every unit (executor gate).
- Confirm the no-timing path (`now=0.0`) fires immediately as before, and a
  revoke mid-wait cancels a not-yet-started cue.

## Risks & Considerations

- **CLOCK ASSUMPTION (primary).** The sender-anchored epoch requires NTP-synced
  clocks across the distributed BLE worker Pis (10.0.0.11-14 each run Sphero
  trees on separate hosts; see project_distributed_ble_workers). This is the
  SAME assumption the existing direct-path `_compute_start_at` already makes
  (`:352-362`). If a worker's clock is skewed, its units fire early/late by the
  skew. Mitigation: ensure NTP/chrony sync; enforce a `start_offset` floor
  (Open Q4) so the target stays in the future for every unit.
- **10 Hz flush granularity.** SM activation flushes on the update tick, so the
  first fire lands within ~<=100 ms of the epoch. Uniform across units (all tick
  at 10 Hz), so it does not cause relative drift, but it bounds absolute
  precision; note it for demos needing tighter sync.
- **Shared-node change (SM controller).** The pending-buffer touches a node
  shared by every cue and by the webapp SM-config path. Backward compat rests on
  "absent `start_at` -> configure immediately" — verify the websocket publisher
  (`sphero_instance_websocket_server.py:197`) omits `start_at` and still
  activates instantly.
- **Revoke during the wait.** A `FleetPolicy active=false` (or dropped
  membership) arriving while a lane is pending must drop the pending config
  (Step 5) so a cancelled cue never fires late. Covered, but must be tested.
- **Two resolution call-sites.** `policy_callback` now resolves an epoch the
  same way `task_callback`/`_handle_bundle` do; keep it funneled through the one
  `_compute_start_at` (plus the `> 0.0` msg normalization) so the two paths
  cannot diverge.
- **Layering.** All timing stays in the task controller (resolution) and the SM
  controller (gating); the device controller is untouched
  (project_control_stack_layering).

## Resolved decisions (user, 2026-08-03)
- OQ1 -> **Receiver-side only.** Implement the msg field + task/SM-controller
  gating; bench-test by stamping `now = time.time()` in the `ros2 topic pub`
  command. Producer integration deferred (no in-tree FleetPolicy publisher).
- OQ2 -> **Default `start_offset` = immediate (0).** Operator sets an explicit
  offset for synchronized demos.
- OQ3 -> **Blank now, render at epoch.** Surface-blank fires immediately at
  `start_at=None`; only the new render/spin carries the epoch.
- OQ4 -> **Enforce a ~0.5 s floor** on a nonzero `start_offset` so the target is
  safely future for the last-arriving unit; `0` still means immediate.

## Approval Status
- [x] Waiting for user approval
- [x] Approved (2026-08-03; all four open questions resolved above)
- [x] Executed (2026-08-03; offline-verified, 215 tests pass; NOT committed — pending live fleet test)
