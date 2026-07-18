# Route Similarity cue through the state machine (ticking) + task layers

**Created:** 2026-07-18T00:00:00
**Status:** Pending Approval

## Task Description
Rework the fleet-policy Similarity cue so it is applied through the proper
control stack instead of the device controller calling the Sphero API directly.
Per user direction: long-running / ticking behavior must be expressed as a STATE
MACHINE. The state machine fires tasks (task controller), which arbitrate on
lanes and reach the device via command topics.

## Architecture (confirmed from source)
- StateMachine controller subscribes `sphero/<name>/state_machine/config` (JSON).
  On configure it fires the initial state's tasks; on each transition it fires
  the entered state's `tasks[]` by publishing to `sphero/<name>/task`.
  Control topic `state_machine/control` accepts pause/resume/clear.
- State model: `states[]` + `initial_state`; each state has `tasks[]`, `exits[]`
  (condition -> destination), optional `timeout`, `entry_condition`, `sub_machine`.
  Conditions: always | timer | topic_value | topic_message. Ticking is the 10 Hz
  `process()` loop firing timer exits.
- Task controller -> TopicTaskExecutor -> command topics (`led`, `matrix`, ...).
  Lane model arbitrates (drive/led/matrix/config). `_stop_lane` blanks a surface
  on cancel. `_send_matrix_command` currently forwards ONLY `pattern` (no custom
  64-matrix); device `matrix_callback` already accepts a `matrix` field.
- CURRENT (to remove) direct-API cue lives in the device controller.

## Design

### Similarity cue -> state machine config (uniform representation)
Consumer (see decision A) builds an SM config from the cue and loads it via
`state_machine/config`. Surface routing by content is expressed as the task type
used in the state(s): RGB-only -> `set_led`; pattern/custom-matrix -> `matrix`.

- IDENTITY (steady): single leaf state `identity`, entry task = render task, no
  exits. Fires once; hardware holds the value.
- BEHAVIOR (blink): two-state timer loop.
    initial_state: "on"
    on:  tasks=[render task],       exits=[timer(half_period) -> "off"]
    off: tasks=[blank task],        exits=[timer(half_period) -> "on"]
  half_period = 1/(2*max(blink_hz,0.1)). Ticking handled by the SM loop.

### Task-layer additions
- Extend `_send_matrix_command` (abstract + TopicTaskExecutor + DirectTaskExecutor)
  and the `matrix` task handler to accept + forward `custom_matrix` so a
  custom-64 cue can render through the task path (device already supports it).

### Teardown (revoke / surface switch)
`state_machine clear()` does NOT blank hardware. On revoke (active:false) or a
re-published cue that changes surface: publish a one-shot blank task
(set_led 0,0,0 / matrix clear) to `sphero/<name>/task`, then send
`state_machine/control` clear. (See decision C for exact ownership.)

### Remove direct-API cue from device controller
Delete `/fleet/policy` sub, `policy_callback`, `_apply_similarity`,
`_clear_similarity`, `_clear_sim_surface`, `_cancel_blink_timer`, blink timer +
`_sim_surface` state, similarity bits of `_clear_all_policy_cues`. Device
controller returns to pure actuator.

## Open Decisions (need user sign-off)
A. Policy consumer: STATE MACHINE controller subscribes `/fleet/policy` and
   builds/loads the SM config [recommended, SM layer owns ticking behavior];
   OR a coordinator fans policy into `state_machine/config` per Sphero.
B. Single-SM-slot clobber: the SM controller holds ONE `self.state_machine`. A
   Similarity policy would REPLACE any user-loaded SM (and vice versa). Options:
   (b1) accept clobber [simplest]; (b2) reserve a dedicated policy SM channel /
   second SM instance; (b3) guard so policy only loads when no user SM is active.
C. Teardown ownership + steady persistence: how revoke blanks the surface, and
   whether a steady IDENTITY should persist against later task writes (needs
   priority the stack does not currently have).
D. Scope: Similarity only (proximity/common-fate remain deferred no-ops)?

## Build + Verify (worktree; do NOT commit)
- Rebuild `sphero_instance_controller`; py_compile + import all three nodes.
- Re-derive the five cases as `/fleet/policy` publishes; confirm each produces
  the expected SM config, transitions, fired tasks, lane occupancy, and device
  render. Confirm blink cadence and revoke blanking.

## Resolved Decisions
- A: TASK CONTROLLER owns `/fleet/policy`; it publishes SM config/control to the
  state machine controller for blink cues (user-approved new coupling).
- B: accept single-SM-slot clobber this phase (documented).
- C: teardown enqueues a blank task; blink also clears the SM. Steady identity
  not protected against later task writes (no priority in the stack).
- D: Similarity only; proximity/common-fate remain deferred no-ops.

## Approval Status
- [x] Approved
- [x] Executed (worktree; not committed — pending hardware test)
