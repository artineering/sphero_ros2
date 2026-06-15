# Owner/Modifier Lane Model for the Concurrent Task Executor

**Created:** 2026-06-01T21-00-00Z
**Status:** Pending Approval
**Scope:** DESIGN ONLY — no code edits in this pass.

## Task Description

The concurrent task executor uses a coarse lane model in which every motion
task occupies the single exclusive `DRIVE` lane. As a result `heading` and
`speed` — which are *live setpoint pokes* meant to steer an active `roll` — are
serialized behind / rejected against the roll instead of applying immediately.
A bundle of `{roll, heading}` is rejected as a "lane conflict on ['drive']".
The same coarse treatment applies to LEDs and the matrix.

We are formalizing two task *classes* — **Owners** and **Modifiers** — so that
single-shot setpoint pokes (`heading`, `speed`, `set_led`, `matrix`) run
immediately and concurrently with the continuous owner that holds the lane,
without evicting it. We are also making `set_led` honor the `led_type`
(`main`/`front`/`back`) parameter, which is currently silently ignored.

## Analysis (current state, verified)

### Where the model lives
- `core/common/task.py`
  - Lane constants `LANE_DRIVE/LED/MATRIX/CONFIG`, `LANES`, `EXCLUSIVE_LANES`,
    `DEFAULT_LANE`, `_SLOT_LANES`.
  - `TaskDescriptor.lanes` (a `FrozenSet[str]`, default `{DRIVE}`).
  - `current_tasks`: one running slot per lane (`drive/led/matrix/config`).
  - `process_tasks()` four-phase tick: (1) `_handle_stop_sentinels`,
    (2) `_promote_due`, (3) tick each running task, (4) `_promote_due` again.
  - `_promote_due()`: promotes a queued task only if **every** one of its lanes
    is free and unreserved this tick and it is due (`start_at`). Otherwise it
    reserves its own lanes (FIFO).
  - `_handle_stop_sentinels()`: targeted/panic/bare-stop semantics; bare stop
    cancels DRIVE only and emits a physical stop via `_cancel_task` ->
    `_stop_lane`.

- `core/sphero/sphero_task_executor.py`
  - `TASK_LANES` map (task_type -> lane frozenset) and `lanes_for()`.
  - Today **every** drive verb (incl. `heading`, `speed`, `stop`) maps to
    `{DRIVE}`; `set_led`/`led_sequence` -> `{LED}`; `matrix`/`matrix_sequence`
    -> `{MATRIX}`; `collision` -> `{CONFIG}`; `custom`/`jumping_bean` ->
    `EXCLUSIVE_LANES` (all three).
  - `_stop_lane()` routes per-lane physical stop (motors / LED off / matrix
    clear).

### Handler shapes (verified in `sphero_task_handlers.py`)
- **Single-shot** (emit one command, `return True` first tick):
  `heading`, `speed`, `set_led`, `matrix`, `collision`, `reflect`, `stop`.
- **Continuous** (`return False`, occupy lane across ticks):
  `roll` (duration=0), `roll` (timed), `circle`, `square`, `patrol`, `move_to`,
  `spin`, `led_sequence`, `matrix_sequence`, `custom`, `jumping_bean`.
  (`reflect` is single-shot: it emits one indefinite roll then returns True, so
  it does NOT hold the lane — it is a drive *modifier*, see classification.)

### The `set_led` bug (verified)
`execute_set_led` resolves RGB then calls `executor._send_led_command(r, g, b)`
with **no** `led_type`. But the whole stack already supports it:
- `_send_led_command(self, red, green, blue, led_type='main')` in both
  `direct_task_executor.py` and `topic_task_executor.py`.
- Device controller `led_callback` reads `type`/`led` and calls
  `self.sphero.set_led(r, g, b, led_type)`.
- `sphero.py:set_led` supports `'main'`/`'front'`/`'back'`.
So front/back LED are wired everywhere EXCEPT the task handler. Fixing
`execute_set_led` to read `led_type`/`type` and pass it through is the only
change needed to reach front/back from a task.

### Bundle validation (verified in `sphero_instance_task_controller_node.py`)
`_handle_bundle()` rejects a bundle if **any** two sub-tasks share **any**
lane (`seen_lanes & t.lanes`). Because `roll` and `heading` both map to
`{DRIVE}`, `{roll, heading}` is rejected. This is the bug at the bundle layer.

### Status surface (verified)
`publish_task_status()` emits `running_lanes = {lane: task_id|None}` straight
from `current_tasks`. Modifiers that never reserve a slot will not appear here
(they complete same-tick), which is correct.

---

## The Model: OWNERS vs MODIFIERS

### Definitions
- **Owner** — a continuous task that *reserves* its lane(s) for its lifetime.
  At most one owner per lane. Owner-vs-owner in the same lane = conflict
  (rejected in a bundle; serialized FIFO when streamed singly).
- **Modifier** — a single-shot task that applies a live setpoint to an
  actuator. A modifier **executes immediately**, in the same tick it becomes
  due, **even if that actuator's lane is currently owned**, and it **never
  reserves a slot, never conflicts, and never blocks** anything (owner or
  follower). It completes the same tick.

### Key mechanism decision: modifiers do NOT use lane slots

Because a modifier is single-shot and must never wait on a busy lane nor block
a follower, the cleanest implementation is: **a modifier carries an empty lane
set (`frozenset()`) and is executed inline at promotion time, completing the
same tick, without ever being placed in `current_tasks`.**

This falls out of the existing `_promote_due` invariants almost for free:
- `_slot_lanes(frozenset())` == `frozenset()`, so `busy = any(... for ln in
  lanes)` is `any([])` == `False`, `reserved & lanes` is empty, and the task is
  never blocked by a busy/reserved lane.
- The only gate left is `start_at` (synchronized start), which we WANT to keep.

The minimal change to `_promote_due` is: when a due task is a **modifier**,
run it inline immediately and file it to history instead of placing it in
`current_tasks`. Concretely, inside the promote loop, after the due/busy/
reserved checks pass:

```
if self._is_modifier(t):
    self.task_queue.pop(i)
    t.status = TaskStatus.RUNNING
    t.started_at = now
    try:
        self.execute_task(t)            # single-shot: returns True
        t.status = TaskStatus.COMPLETED
    except Exception as e:
        t.status = TaskStatus.FAILED
        t.error_message = str(e)
    t.completed_at = time.time()
    self.task_history.append(t)
    # do NOT touch current_tasks; do NOT add lanes to `reserved`
    # do NOT advance i (a new task may shift into this index)
    continue
```

Owners keep the existing promote path (reserve `current_tasks`, tick in
Phase 3).

**Why inline-at-promote and not "give modifiers a slot then complete it":**
giving a modifier a transient slot would (a) make it appear in `running_lanes`
for one tick, (b) risk it blocking a same-lane follower in the brief window,
and (c) require modifiers to share a slot key with the owner — none of which we
want. Inline execution with no slot is strictly simpler and matches the
"never reserve, never block" requirement exactly.

### How a task is tagged owner vs modifier

Add a module-level `MODIFIER_TASK_TYPES` frozenset in
`sphero_task_executor.py` (beside `TASK_LANES`), and a tiny predicate. Two
candidate tagging strategies; the design picks **(A)** for minimal surface:

- **(A) Type-set predicate (chosen).** `SpheroTaskExecutorBase` exposes
  `is_modifier(task_type) -> bool` checking membership in
  `MODIFIER_TASK_TYPES`. The base `TaskExecutorBase._is_modifier(task)` returns
  `False` (robot-agnostic default → all generic test tasks remain owners, so
  the generic A/B/C tests are untouched). `SpheroTaskExecutorBase` overrides
  `_is_modifier(task)` to consult the set. `lanes_for()` returns
  `frozenset()` for modifier types so descriptors built via the node get the
  empty lane set automatically.

- (B) A boolean flag on `TaskDescriptor` (`is_modifier`). Rejected: it pushes
  classification into every construction site and duplicates the source of
  truth that already lives next to `TASK_LANES`.

**Single source of truth:** `MODIFIER_TASK_TYPES` + `TASK_LANES`. A modifier's
`TASK_LANES` entry becomes `frozenset()` (no lane). Owners keep their lane
entry. `lanes_for()` and `is_modifier()` both read from this one place.

---

## Final Lane Taxonomy

Lanes (owner-reservable slots) stay exactly three real lanes + CONFIG:
- `LANE_DRIVE` — the single drive owner.
- `LANE_LED` — the main-LED owner (`led_sequence`).
- `LANE_MATRIX` — the matrix owner (`matrix_sequence`).
- `LANE_CONFIG` — slotless config (`collision`, and `stabilization` if/when a
  task type is added); never blocks / blocked.

**No new lanes for front/back LED.** Front/back LEDs are touched ONLY by the
`set_led` modifier (which reserves nothing), so they need no owner lane. Main
LED keeps its lane solely because `led_sequence` (an owner) animates it.

**Modifiers carry `frozenset()` (no lane).** They never reserve, never
conflict. For status display they simply don't appear in `running_lanes`
(they complete same-tick), which is the correct UX — there is no persistent
"heading lane" to show.

---

## Owner / Modifier Classification Table (ALL task types)

| task_type        | class    | shape       | lane(s) (`TASK_LANES`) | rationale |
|------------------|----------|-------------|------------------------|-----------|
| `roll`           | Owner    | continuous  | `{DRIVE}`              | holds drive until stopped/timed-out |
| `move_to`        | Owner    | continuous  | `{DRIVE}`              | closed-loop drive to a point |
| `patrol`         | Owner    | continuous  | `{DRIVE}`              | multi-waypoint drive |
| `square`         | Owner    | continuous  | `{DRIVE}`              | patrol over generated waypoints |
| `circle`         | Owner    | continuous  | `{DRIVE}`              | timed differential drive |
| `spin`           | Owner    | continuous  | `{DRIVE}`              | timed in-place rotation |
| `jumping_bean`   | Owner    | continuous* | `EXCLUSIVE_LANES`      | whole-robot exclusive; unchanged |
| `custom`         | Owner    | continuous  | `EXCLUSIVE_LANES`      | mixed timed sequence; exclusive; unchanged |
| `led_sequence`   | Owner    | continuous  | `{LED}`                | animates main LED over time |
| `matrix_sequence`| Owner    | continuous  | `{MATRIX}`             | animates matrix over time |
| `heading`        | **Modifier** | single-shot | `frozenset()`      | live heading setpoint; steers active roll |
| `speed`          | **Modifier** | single-shot | `frozenset()`      | live speed setpoint; modulates active roll |
| `set_led`        | **Modifier** | single-shot | `frozenset()`      | one-shot LED poke (main/front/back), param-aware |
| `matrix`         | **Modifier** | single-shot | `frozenset()`      | one-shot single matrix pattern |
| `reflect`        | **Modifier** | single-shot | `frozenset()`      | emits one indefinite roll then returns True; a live drive poke, not a lane holder |
| `stop`           | Special  | single-shot | `{DRIVE}`              | sentinel; bare stop = DRIVE-lane cancel; targeted/halt handled in Phase 1 (unchanged) |
| `collision`      | Config   | single-shot | `{CONFIG}`             | config; slotless; never blocks/blocked (unchanged) |

\* `jumping_bean` blocks synchronously inside one handler call (busy-loops with
`time.sleep`); it is an exclusive owner and is out of scope for this change.

### Notes on the two judgment calls
- **`reflect` is a modifier.** It is single-shot (one `_send_roll_command` then
  `return True`) and it *changes the live drive heading*. Under the old model it
  occupied DRIVE for one tick. Making it a modifier means it can re-aim an
  active `roll` without evicting it — consistent with "live drive poke." If the
  team prefers `reflect` to remain an owner-style DRIVE one-shot (taking over
  the lane), flip it back to `{DRIVE}` owner; this is the one debatable entry.
  **Recommendation: modifier**, since it behaves like `heading`+`speed` fused.
- **`stop` stays special, not a modifier.** Its cancel semantics are entirely
  in Phase 1 (`_handle_stop_sentinels`); it keeps `{DRIVE}` so the bare-stop
  fall-through and physical-stop-on-cancel paths are unchanged.

---

## Bundle-Validation Change

In `_handle_bundle()` (`sphero_instance_task_controller_node.py`), replace the
"any shared lane = reject" check with **owner-vs-owner same-lane only**:

- Modifiers (empty lane set) are always accepted and never counted toward
  `seen_lanes`.
- Lane disjointness is enforced **only among owners**.

New logic (conceptual):
```
seen_owner_lanes = set()
for t in tasks:
    if not t.lanes:            # modifier: always allowed, occupies no lane
        continue
    conflict = seen_owner_lanes & t.lanes
    if conflict:
        reject(f"two owners contend for {sorted(conflict)}")
        return
    seen_owner_lanes |= t.lanes
```

Outcomes:
- `{roll, heading, speed, set_led(front), matrix}` -> ACCEPTED (one DRIVE owner
  + four modifiers, no owner lane reused).
- `{roll, circle}` -> REJECTED (two DRIVE owners).
- `{led_sequence, led_sequence}` -> REJECTED (two LED owners).
- `{custom, set_led}` -> still REJECTED: `custom` is `EXCLUSIVE_LANES`, so it
  reuses DRIVE/LED/MATRIX vs... no, it is the only owner here, but pairing an
  exclusive owner with a modifier is fine at *validation* (modifier has no
  lane). However at *runtime* the modifier will fire immediately and the
  exclusive owner reserves all three lanes — the modifier still runs (it never
  checks lanes). This is acceptable: a one-shot LED poke during a `custom`
  exclusive sequence is harmless. If the team wants exclusive owners to also
  suppress modifiers, that is a follow-up; **not** in scope here.

`_build_task` already returns the modifier's empty lane set via
`lanes_for()` (now returning `frozenset()` for modifier types), so no change is
needed there beyond the `TASK_LANES` edits. The explicit `lane`/`lanes`
override path in `_build_task` is preserved.

---

## `execute_set_led` Param Fix

Change the final emit in `execute_set_led` to honor the LED target:

```
led_type = str(params.get('led_type', params.get('type', 'main'))).lower()
if led_type not in ('main', 'front', 'back'):
    led_type = 'main'
executor._send_led_command(red, green, blue, led_type)
```

- Accepts `led_type` (preferred) and `type` (alias used by the web/device
  layer), defaulting to `'main'`.
- Validates against `{main, front, back}`, falling back to `main` on anything
  else (matches the lenient device-side behavior).
- Front and back are independent physical LEDs (`sphero.py` routes each), so
  a bundle can drive `set_led(front=red)` and `set_led(back=blue)` together;
  both are modifiers with no lane, so they coexist fine.

**Back-compat:** existing `execute_set_led` callers that pass no `led_type`
still target `main`. The two existing smoke tests already assert
`led_type='main'` in the recorded send, so they pass unchanged.

---

## Status / `running_lanes` Impact

- `running_lanes` continues to reflect only owner slots (`current_tasks`).
  Modifiers complete same-tick and never occupy a slot, so they never appear —
  correct, since there is no persistent "heading/speed/set_led" lane.
- `publish_task_status` is still called for a modifier when it is enqueued and
  again when it completes (via the `previous`/`current` diff in
  `task_execution_loop`)... **caveat:** because a modifier never enters
  `running_tasks()`, the `task_execution_loop` started/finished diff (which
  snapshots `running_tasks()`) will NOT observe it. The enqueue-time
  `publish_task_status(t)` in `_handle_bundle` / `task_callback` still fires, so
  the dashboard sees the modifier as accepted. If per-modifier completion
  telemetry is desired, that is a small follow-up (publish from the inline
  branch); **for this change, enqueue-time status is sufficient** and matches
  the "fire-and-forget poke" semantics.
- `to_dict()['lane']` for a modifier (empty lane set) currently yields `'multi'`
  via `next(iter(...))` on an empty set → that path is `len(self.lanes) == 1`
  false, so it returns `'multi'`. We should special-case empty -> `'none'` (or
  `'modifier'`) in `to_dict` for clarity. Minor; include in the change.

---

## Back-Compat Analysis

| Surface | Effect |
|---------|--------|
| Generic `TaskExecutorBase` + A/B/C tests | UNCHANGED. Base `_is_modifier` returns `False`, so all generic handlers stay owners with default `{DRIVE}` lane; promotion path identical. |
| Single `/api/task` for an owner (`roll`, `led_sequence`, …) | UNCHANGED. Same lane, same promote/tick path. |
| Single `/api/task` for a modifier (`heading`, `speed`, `set_led`, `matrix`, `reflect`) | Now runs inline at promote and completes same tick (it already completed same tick before, since single-shot). Observable behavior identical for a lone modifier on a free lane; the ONLY new behavior is it now also runs when its actuator lane is owned. |
| SM-generated bundles | Bundles that were valid stay valid; bundles that were wrongly rejected (`roll`+`heading`) now accepted. No previously-accepted bundle becomes rejected (owner-vs-owner was already rejected). |
| `stop` / bare-stop / targeted / halt / physical-stop-on-cancel | UNCHANGED. `stop` keeps `{DRIVE}`; Phase 1 logic and `_stop_lane` untouched. All `TestPhysicalStopOnCancel` tests pass as-is. |
| `set_led` smoke tests (assert `led_type='main'`) | PASS — default path still emits `main`. |

### Tests that may need updating (enumerate + why)
1. `TestConcurrentLanes.test_per_lane_start_at_gating_independence` uses
   `set_led` as the "LED lane" task and asserts the LED is emitted while the
   gated DRIVE roll waits. Under the new model `set_led` is a modifier (no
   lane), so the assertion `ex.current_tasks[LANE_DRIVE] is None` still holds
   and the LED still emits. **It should still pass**, but the test's mental
   model ("LED lane") shifts to "modifier"; review for intent. Likely no edit.
2. `test_bundle_per_task_start_offsets_stagger_lanes` and
   `test_per_lane_fifo_same_lane_gated_blocks_follower` and
   `test_lane_conflict_queues_within_lane` and `test_targeted_stop_by_name`
   and `test_bare_stop_cancels_drive_only` all use `set_led` as a stand-in
   "LED-lane" task. With `set_led` now a modifier (no slot), these tests'
   `current_tasks[LANE_LED]` expectations change:
   - `test_per_lane_fifo_same_lane_gated_blocks_follower` builds **two
     `set_led`** tasks and asserts neither promotes and `ex.sends == []`. Under
     the modifier model, a `set_led` with a future `start_at` is gated (good),
     but the second `set_led` is NOT blocked by the first (modifiers don't
     share a lane). The first is gated by `start_at`, the second is due and is
     a modifier so it WOULD fire. **This test must be rewritten** to use an
     owner (`led_sequence`) for the same-lane-FIFO assertion. It was testing
     LED-lane FIFO, which is now an owner-only property.
   - `test_lane_conflict_queues_within_lane` uses `set_led` only as a parallel
     LED emit; its core assertion is about two `roll` owners serializing — that
     still holds. The `set_led` emit assertion still holds (modifier fires).
     **Likely passes; verify.**
   - `test_targeted_stop_by_name`, `test_bare_stop_cancels_drive_only`,
     `test_halt_*` use `led_sequence` (owner) for the LED lane, so they are
     UNCHANGED.
   **Net:** the only mandatory rewrite is the same-lane-FIFO test that used two
   `set_led`s; switch it to two `led_sequence` owners (or two `roll` owners).

### New tests to add
1. **roll + heading concurrent (the headline fix).** Start an indefinite
   `roll` (owner, DRIVE). Enqueue a `heading` modifier. One tick: assert
   `_send_heading_command` emitted, roll STILL running on DRIVE
   (`current_tasks[DRIVE] is roll`, not cancelled), heading filed COMPLETED in
   history. Same for `speed`.
2. **modifier fires while its actuator lane is owned.** Start `led_sequence`
   (owner, LED). Enqueue `set_led` modifier. Assert the LED poke emits and the
   `led_sequence` owner is untouched.
3. **param-aware `set_led` front/back.** `set_led(led_type='front', color=...)`
   -> recorded `('led', {... led_type:'front'})`; same for `back`; and
   `type='back'` alias resolves; invalid `led_type='foo'` falls back to `main`.
4. **bundle owner+modifiers accepted.** Feed
   `{roll, heading, speed, set_led(front), matrix}` through the bundle
   validator; assert all five enqueued (not rejected).
5. **two-owner bundle rejected.** `{roll, circle}` rejected;
   `{led_sequence, led_sequence}` rejected.
6. **modifier has empty lane set / never reserves.** Assert
   `SpheroTaskExecutorBase.lanes_for('heading') == frozenset()` and
   `is_modifier('heading')` is True; after running a lone `set_led`,
   `current_tasks[LANE_LED] is None` (never slotted).
7. **modifier respects `start_at`.** A `heading` modifier with a future
   `start_at` does NOT emit until due (synchronized start preserved).
8. **modifier does not block a same-tick owner behind it.** Queue order
   `[set_led (modifier), roll (owner)]`: in one tick BOTH run (modifier inline,
   roll promotes into DRIVE) — proving modifiers never reserve/hold up
   followers.

---

## Detailed Plan (implementation steps, for the approved follow-up)

### Step 1: `sphero_task_executor.py` — classification source of truth
- Add `MODIFIER_TASK_TYPES = frozenset({'heading','speed','set_led','matrix','reflect'})`.
- Change `TASK_LANES` entries for those five to `frozenset()`.
- `lanes_for()` unchanged in shape (still reads `TASK_LANES`, returns the empty
  set for modifiers; keep the default `{DRIVE}` for truly-unknown types).
- Add `is_modifier(task_type)` static/classmethod.
- Files: `core/sphero/sphero_task_executor.py`.
- Verify: `lanes_for('heading') == frozenset()`, `is_modifier('roll') is False`.

### Step 2: `task.py` — modifier-aware promotion
- Add `_is_modifier(task) -> bool` on `TaskExecutorBase` returning `False`.
- In `_promote_due`, when a due/unblocked task `_is_modifier`, execute it
  inline (run handler, set COMPLETED/FAILED, file to history), do NOT slot it,
  do NOT add its (empty) lanes to `reserved`, do NOT advance `i`.
- Fix `to_dict()` empty-lane -> `'none'`.
- Files: `core/common/task.py`.
- Verify: generic tests still pass (base `_is_modifier` False).

### Step 3: `sphero_task_executor.py` — override `_is_modifier`
- Override `_is_modifier(self, task)` to return `is_modifier(task.task_type)`.
- Files: `core/sphero/sphero_task_executor.py`.

### Step 4: `sphero_task_handlers.py` — param-aware `execute_set_led`
- Read `led_type`/`type`, validate against `{main,front,back}`, pass through.
- Files: `core/sphero/sphero_task_handlers.py`.

### Step 5: `sphero_instance_task_controller_node.py` — bundle validation
- Replace "any shared lane" check with owner-only disjointness (skip empty-lane
  modifiers).
- Files: `sphero_instance_task_controller_node.py`.

### Step 6: Tests
- Rewrite the same-lane-FIFO test that used two `set_led`s to use owners.
- Add new tests (1)-(8) above.
- Files: `test/test_task_executor.py` (+ a bundle-validation test, which may
  need a lightweight harness around `_handle_bundle` or a unit test of the new
  validation predicate factored out as a pure function — recommend extracting
  the owner-disjointness check into a small testable helper).
- Verify: `colcon test --packages-select sphero_instance_controller`.

### Step 7: Build + run tests
- `colcon build --packages-select sphero_instance_controller`
- `source install/setup.bash`
- `colcon test --packages-select sphero_instance_controller --pytest-args -q`

## Expected Outcomes
- `roll`+`heading`/`speed` bundles accepted; modifiers steer a live roll.
- `set_led` reaches front/back LEDs.
- Two owners in one lane still rejected (bundle) / serialized (stream).
- All existing tests pass except the one same-lane-FIFO test (rewritten) and
  any intentionally adjusted LED-lane tests.

## Potential Risks & Considerations
- **`reflect` reclassification** is the one debatable call; flag for sign-off.
- **Modifier during an EXCLUSIVE owner (`custom`/`jumping_bean`).** Modifiers
  will still fire (they ignore lanes). Decided acceptable; note as a known,
  intentional behavior, not a bug.
- **Modifier completion telemetry** is enqueue-time only (not via the running
  diff). Acceptable for fire-and-forget; follow-up if richer status is wanted.
- **Ordering within a tick:** a modifier inline-executes during `_promote_due`,
  which runs BEFORE Phase-3 owner ticks on the first promotion. For a bundle
  with a synchronized `start_at`, the modifier and the owner both become due the
  same tick; the owner promotes (slotted) and is ticked in Phase 3, the modifier
  runs inline in Phase 2 — both apply in the same tick at the synced instant, as
  required.

## Testing Plan
- Unit: the 8 new tests + rewritten FIFO test (see above).
- Manual (single robot):
  - `ros2 topic pub <prefix>/task` with a bundle
    `{"tasks":[{"task_type":"roll",...},{"task_type":"heading","parameters":{"heading":90}}]}`
    and confirm via `ros2 topic echo <prefix>/task_status` that it is accepted
    and `running_lanes.drive` shows the roll while heading completes.
  - `set_led` with `parameters:{"led_type":"front","color":"red"}` and confirm
    the device-controller log reports "Front LED set".

## Approval Status
- [ ] Waiting for user approval
- [ ] Approved
- [ ] Executed
