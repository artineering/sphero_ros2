# Phase 2 - Proximity cue (per-unit, localization-fed clustering)

**Created:** 2026-07-18T00:00:00
**Status:** Pending Approval

## Task Description
Implement the Proximity Gestalt cue of the fleet-policy broadcast. Proximity now
means: policy members actively CLUSTER to a target inter-robot spacing so an
observer perceives them as one group (this SUPERSEDES the earlier "Stop"
reaction / threshold_cm / reaction{FLASH,STOP} design, which is dropped).
Motion is via `move_to`-style rolling on the DRIVE lane. Coordination is
PER-UNIT and localization-fed: each member's task controller subscribes to the
group's positions, computes its own target, and drives itself. The periodic
control loop is expressed as a per-unit STATE MACHINE (layering rule:
project_control_stack_layering), not an ad-hoc timer.

Foundational sub-piece: the state machine controller gains a LANE model (a map
of {lane -> StateMachine} ticked concurrently), so a Similarity-BLINK SM and a
Proximity-cadence SM coexist on one unit without contending for a single slot.
This retires the Phase-1 single-SM-slot clobber limitation (decision B).

## Analysis (grounded in current code)
Anchors (worktree paths under src/sphero_instance_controller/sphero_instance_controller):
- Localization contract: `/localization/<name_safe>/position`, `geometry_msgs/PoseStamped`.
  Existing subscribe: `sphero_instance_device_controller_node.py:236-238` (device
  controller, only when external_location). Field-frame ground truth per unit.
- `move_to` handler: `core/sphero/sphero_task_handlers.py:71` (`execute_move_to`) -
  x/y target in cm, speed, stall detection; issues roll toward
  `atan2(dy,dx)`, completes within `executor.position_tolerance`. DRIVE lane
  (`core/sphero/sphero_task_executor.py:47`, registered `:127`).
- Task controller owns policy: `sphero_instance_task_controller_node.py`
  `policy_callback:544`, `_apply_similarity:581`, own position from
  `sphero/<name>/state` (`state_callback:699`, `get_current_position:271`).
  SM hand-off publishers added in Phase 1: `sm_config_pub` /`sm_control_pub`
  (`/state_machine/config`, `/state_machine/control`).
- State machine controller ingests config: `sphero_instance_statemachine_controller_node.py:98-104`
  (`config_sub` on `{prefix}/state_machine/config`); fires a state's `tasks[]` on
  ENTRY only, and SKIPS unchanged path levels (`execute_tasks_for_entered_branch`
  ~`:426`). Conditions available: always | timer | topic_value | topic_message
  (`core/sphero/statemachine.py:19-29`, timer at `:627`).
- FleetPolicy today carries only `SimilarityCue similarity`
  (`src/multirobot_msgs/msg/FleetPolicy.msg`). No proximity sub-message yet.

### Key structural constraint (drives the design)
The SM fires STATIC task dicts on entry and does not re-fire on a self-loop. A
proximity target is DYNAMIC (recomputed each tick from live positions), so it
CANNOT be encoded in static SM params. Therefore:
- SM provides CADENCE only: a two-state timer ping-pong (`step_a`<->`step_b`,
  each `timer(1/tick_hz)` -> the other), mirroring the Phase 1 blink idiom. Each
  entry fires one `proximity_step` task.
- A NEW task handler `execute_proximity_step` does the per-unit computation at
  execution time, reading live member positions from the executor, and issues a
  single roll/stop step toward the computed target. Motion stays on DRIVE.

## Detailed Plan

### Step 0: State machine controller LANES (foundational, orthogonal plumbing)
Rationale: Similarity-BLINK and Proximity-cadence are both state machines that
must coexist on one unit. Today `sphero_instance_statemachine_controller_node`
holds ONE `self.state_machine` (`:55`) and one config topic, so a second SM
clobbers the first (Phase-1 decision B). Fix by giving the SM controller a lane
model that MIRRORS the task executor's actuator partition.

- Hold `self.state_machines: Dict[str, StateMachine]` keyed by lane instead of a
  single machine. `update_callback` (`:398`) advances EVERY lane's SM each tick;
  each fires its own tasks to `sphero/<name>/task` independently. Status/events
  publish per lane.
- Lane-scope the contract: `state_machine/config` and `state_machine/control`
  JSON payloads carry a `lane` id. Config loads/replaces the SM on that lane;
  control (pause/resume/clear) targets one lane (plus `scope:"all"` to clear
  every lane).
- Lane taxonomy = MIRROR the executor lanes (`core/common/task.py:32-43`:
  drive / led / matrix / config). Justification: an SM's ultimate effect is the
  executor lane its fired tasks land on; if two concurrent SMs fire into the
  SAME executor lane they serialize/fight downstream, defeating concurrency. So
  SM lanes must correspond to executor actuator domains. Similarity-BLINK ->
  `led` or `matrix` (whichever surface); Proximity-cadence -> `drive`. Optionally
  warn if two SM lanes would fire into the same executor lane.
- Backward compat: an absent `lane` maps to a reserved `default` lane (its own
  slot), so existing single-SM publishers keep working unchanged. MIGRATION
  CHECKED: the live ROS publisher is `sphero_instance_websocket_server.py:197`,
  driven by the multirobot_webserver webapp (frontend `static/js/app.js:1252` ->
  Flask `/api/state_machine/config` `:683`). It omits `lane` -> lands on
  `default` -> unchanged behavior. (The task controller also publishes SM configs
  and WILL tag lanes.) Optional later: add a lane selector to the webapp.
- RETIRES Phase-1 decision B (single-SM-slot clobber): a policy blink SM now
  lives on its own lane and never clobbers a user SM on another lane.

### Step 1: Messages
- New `src/multirobot_msgs/msg/ProximityCue.msg`:
  `float32 target_spacing_cm`, `uint8 anchor` (0=CENTROID,1=FIELD_POINT),
  `float32 anchor_x`, `float32 anchor_y` (used when FIELD_POINT),
  `float32 max_speed`, `float32 tick_hz`, `float32 min_separation_cm`.
- Add `ProximityCue proximity` to `FleetPolicy.msg`; register in
  `src/multirobot_msgs/CMakeLists.txt` (both msgs already listed pattern).

### Step 2: Task controller - position ingestion + provider
- On an active proximity policy, subscribe to `/localization/<m>/position`
  (PoseStamped) for every member `m` in `msg.members` (raw->safe name). Maintain
  `self._member_positions = {name_safe: {'x','y','t'}}`. Unsubscribe on revoke.
- Inject a `member_positions_callback` into `TopicTaskExecutor` (alongside the
  existing `position_callback`/`heading_callback`) so the handler can read the
  group snapshot. (Executor ctor: `core/sphero/topic_task_executor.py:24`.)

### Step 3: proximity_step task + handler
- Register `proximity_step` (DRIVE lane) in `TASK_LANES` /
  `_register_default_handlers` (`core/sphero/sphero_task_executor.py:47,127`).
- `execute_proximity_step(executor, task)` in `sphero_task_handlers.py`:
  1. Read member set + params from task.parameters (target_spacing, anchor,
     min_sep, max_speed).
  2. Compute anchor: FIELD_POINT -> (anchor_x,anchor_y) [identical across units,
     no divergence]; CENTROID -> mean of member positions (see consistency).
  3. Vector from own pos (executor.get_current_position()) toward anchor.
  4. Min-separation floor: if nearest neighbor < min_sep, cancel/repel the
     inbound component so units settle at ~target_spacing instead of piling up.
  5. Field-bounds clamp on the target.
  6. If within deadband (position_tolerance) and spacing satisfied -> `stop`,
     return True; else issue one `_send_roll_command(heading, speed<=max_speed)`,
     return True (single-shot; SM cadence re-issues next tick).

### Step 4: Wire proximity into policy_callback (task controller)
- Extend `policy_callback:544`: if `msg.enable_proximity` -> `_apply_proximity(
  msg.proximity, members)`; else `_clear_proximity()`. Remove the phase-2
  deferred warning.
- `_apply_proximity`: (re)subscribe member positions; publish a proximity SM
  config (two-state timer ping-pong, tasks=[proximity_step-with-params]) to
  `sm_config_pub` tagged `lane: "drive"`. Track `self._policy_proximity_active`.
- Lane tagging (depends on Step 0): the task controller tags every SM config it
  publishes with a lane so cues never contend: Similarity-BLINK -> `lane` =
  its surface (`led`/`matrix`); Proximity-cadence -> `lane: "drive"`. Update the
  Phase-1 `_publish_blink_sm` / `_clear_blink_sm` to include the surface lane.
- With SM lanes, Similarity-BLINK (led/matrix lane) and Proximity (drive lane)
  run as two concurrent SMs on one unit - no contention. Steady Similarity (no
  SM) + Proximity is likewise fine.

### Step 5: Teardown / revoke
- `_clear_proximity`: publish `stop` task on DRIVE lane, `sm control clear`,
  unsubscribe member positions, reset state. Revoke path (`active:false` /
  non-member) already funnels through policy_callback.
- Keep Proximity on DRIVE and Similarity on LED/MATRIX (disjoint lanes) so a
  combined policy composes.

### Step 6: Build + verify (worktree; do NOT commit)
- Build `multirobot_msgs` then `sphero_instance_controller`. `ros2 interface
  show` ProximityCue/FleetPolicy. py_compile + import nodes. Offline unit-test
  the centroid/min-sep/clamp math with synthetic member snapshots.
- SM-lanes: offline-test that two lane-tagged configs load into two SMs and both
  tick; that absent-`lane` lands on `default`; that a lane-scoped clear targets
  only its lane. Confirm a Similarity-BLINK (led lane) + Proximity (drive lane)
  policy runs both SMs concurrently on one unit.

## Consistency solution (REQUIRED - per-unit must not diverge)
Because every unit computes independently, they must derive the SAME target:
1. Same member set: taken verbatim from the latched `members[]` broadcast -
   identical on every unit.
2. Deterministic ordering: sort members by callsign before any slot/index use,
   so any positional assignment is identical everywhere.
3. Identical computation: all units run the same anchor formula.
   - FIELD_POINT anchor is a static constant -> zero divergence (recommended
     default while bringing the loop up).
   - CENTROID anchor = mean of latest member positions. Async position streams
     mean each unit has a slightly different snapshot, so centroids differ by a
     small, bounded delta.
4. Convergence argument for CENTROID: centroid-seeking is a damped consensus -
   moving each unit toward the group mean is a contraction that reduces position
   variance; the min-separation floor + a deadband (`position_tolerance`) + a
   `max_speed` cap prevent limit cycles/chatter from async jitter. Target spacing
   emerges as the equilibrium between centroid attraction and min-sep repulsion.
   Add explicit damping (step a fraction toward target, not the whole way) if
   bench testing shows oscillation.
5. Slot/formation (optional): if a precise ring/grid is wanted, assign slot =
   sorted-callsign index; deterministic by (2). Default plan uses NO explicit
   slots (attraction + min-sep floor) which self-organizes to ~target_spacing.

## Decisions locked (folded in)
- Q1 Anchor = BOTH, selectable per policy: keep `anchor{CENTROID|FIELD_POINT}` +
  `anchor_x`/`anchor_y`; broadcaster picks. FIELD_POINT = trivially-consistent
  bring-up path; CENTROID = damped-consensus math. Support both.
- Q2 = SM controller gains LANES (Step 0). Similarity-BLINK and Proximity-cadence
  coexist as concurrent per-lane SMs. This is foundational for Phase 2.
- Localization rate ~10 Hz (kinect_field_tracking) -> proximity tick_hz default
  2-5 Hz (well under the position update rate).
- No-fix unit: idles (no motion) AND is excluded from the centroid computation
  so a missing position does not drag the group anchor.
- Ticking cadence = per-unit two-state timer SM (layering rule).

## Defaults to confirm with the user
- Minimum-separation floor ~15 cm (from chassis-less Sphero diameter) so 16
  units cluster without pile-up. EXACT value + real field-bounds extent still
  TBD from the user before a live trial.
- ProximityCue params: target_spacing_cm, anchor{CENTROID|FIELD_POINT}+xy,
  max_speed, tick_hz (2-5 Hz), min_separation_cm. (Replaces the superseded
  threshold_cm / reaction{FLASH,STOP}.)
- SM lane taxonomy = mirror executor lanes (drive/led/matrix/config) with a
  `default` lane for absent-`lane` backward compat - confirm this taxonomy.

## Open Questions (still need user before coding)
1. RESOLVED - anchor = BOTH, selectable per policy.
2. RESOLVED - SM controller lanes (Step 0); Similarity-BLINK and Proximity run
   on separate SM lanes.
3. OPEN - exact `min_separation_cm` (~15 cm proposed) and the real field-bounds
   extent of the kinect field frame (needed before a live trial).
4. RESOLVED - localization ~10 Hz -> tick_hz 2-5 Hz.
5. RESOLVED - no-fix unit idles and is excluded from the centroid.
6. OPEN - confirm the SM lane taxonomy (mirror executor drive/led/matrix/config
   + `default` for backward compat), and confirm no current publisher of
   `state_machine/config` breaks under the lane-scoped contract (migration
   check: which nodes/UI publish SM configs today?).

## Testing Plan
- Offline: unit-test centroid, min-sep repulsion, bounds clamp, deadband stop
  with synthetic snapshots (no hardware).
- Bench: publish a proximity policy to `/fleet/policy`; watch `sphero/<name>/task`,
  `.../task/status`, `.../state_machine/status`, and `/localization/*/position`.
  Verify members converge to ~target_spacing, hold, and do not pile up; verify
  revoke stops motion and clears the SM; verify Proximity + steady Similarity
  compose (disjoint lanes).

## Risks & Considerations
- Async snapshots -> centroid divergence (mitigated by damping/deadband; FIELD_
  POINT sidesteps entirely).
- 16 chassis-less units clustering -> collision/pile-up (min-sep floor + bounds).
- SM-lanes plumbing touches a shared node (state machine controller). Existing
  single-SM publishers must keep working via the `default` lane (migration check,
  open Q6). Per-lane ticking multiplies SM `process()` calls per cycle - cheap,
  but validate no cross-lane shared-state assumptions in StateMachine (each lane
  gets its own StateMachine instance, so topic stores are already per-instance).
- DRIVE-lane (executor) contention: proximity vs a user roll task serialize FIFO
  on DRIVE - intended, but a running proximity loop blocks other DRIVE tasks
  while active.

## Approval Status
- [x] Waiting for user approval (pending open questions 3 and 6)
- [x] Approved (2026-08-03; Q3 field-bounds/min-sep deferred to pre-live-trial, FIELD_POINT default for bring-up; Q6 migration check confirmed in plan)
- [ ] Executed
