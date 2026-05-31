# Concurrent Task Execution — Implementation Plan (Option A: resource-slotted lanes)

**Created:** 2026-05-31T00-00-00Z
**Status:** Pending Approval
**Scope:** `sphero_instance_controller` task executor core + node + tests
**Builds on:** `plans/concurrent-tasks-design-2026-05-31T00-00-00Z.md` (design, Option A chosen)
**Author role:** ROS2 SME

## Task Description

Implement concurrent, per-actuator-lane task execution on a single Sphero instance.
The chosen engine is **Option A**: one running slot per lane (DRIVE / LED / MATRIX /
CONFIG), serial within a lane, parallel across lanes, reusing the existing tick /
lifecycle / `start_at` / cancel machinery. Add a thin `tasks:[...]` bundle payload with a
shared `start_at`. Two user decisions are baked in:

1. **Targeted stop by task name** (changed from the proposal's DRIVE-only stop): `stop` may
   carry a `target` naming a specific task to cancel, across any lane.
2. **Scope (v1 vs v2) is decided from this plan** — both increments are specified below with
   effort/risk and a recommendation.

All line numbers below were read from branch `deploy` and are accurate as of writing.

---

## Analysis (current state, verified)

- `core/common/task.py`
  - `TaskDescriptor` (`:27-52`): fields `task_id, task_type, parameters, status,
    created_at, start_at, started_at, completed_at, error_message`; `to_dict()` (`:40`).
  - `TaskExecutorBase.__init__` (`:75-80`): `task_queue`, single `current_task`,
    `task_history`, `_handlers`.
  - `add_task` (`:90-92`): FIFO append.
  - `process_tasks` (`:94-141`): (1) cancel-on-stop hook `:103-111`; (2) promotion + `start_at`
    gate `:117-122`; (3) tick current `:125-139`; returns `current_task` `:141`.
  - `execute_task` (`:143-153`): handler lookup by `task_type.lower()`, `ValueError` if unknown.
- `core/sphero/sphero_task_executor.py`
  - `_register_default_handlers` (`:41-58`): the 17 handler registrations — the natural home
    for a parallel **lane map**.
- `core/sphero/sphero_task_handlers.py`
  - `execute_stop` (`:305-308`): one-shot stop.
  - `execute_custom` (`:311-351`): multi-actuator mini-interpreter (LED/roll/matrix/stop).
  - `execute_jumping_bean` (`:452-479`): **blocking `time.sleep` loop** — freezes the loop.
- `sphero_instance_task_controller_node.py`
  - `task_callback` (`:276-337`): single-task ingest; auto `task_id` at `:298-299`;
    `now`/`start_offset` → `start_at` at `:308-316`; `add_task` + status.
  - `task_execution_loop` (`:363-396`): captures `previous_task` (`:366`), single-slot
    identity diff (`:372`).
  - `publish_task_status` (`:398-411`): emits task dict + `queue_length`/`has_current_task`/
    `total_pending`.
- `test/test_task_executor.py`: `RecordingExecutor` (`:66`) registers generic lane-less
  handlers (`one_shot`/`two_shot`/`boom`); `RecordingSphero` (`:100`) records `_send_*`.
  Suites: `TestQueueLifecycle`, `TestSynchronizedStart`, `TestRegistry`, `TestCancelOnStop`,
  `TestSpheroHandlerSmoke`. **All must keep passing** (see Back-compat checklist).

Key facts that make the design low-risk:
- Each actuator is a **separate ROS topic** (design §1.3), so parallel emission needs no new
  plumbing — only the scheduler above `execute_task` changes.
- Handler multi-tick state lives in **`task.parameters`**, not the executor. Two tasks in
  different lanes never share an actuator → **no cross-lane state collision**. The only shared
  executor reads (`get_current_position/heading`, `default_speed`, `position_tolerance`) are
  read-only.

---

## Design decisions (resolved in this plan)

### D1. Task name: reuse `task_id` (do NOT add a separate `name` field)

**Decision: reuse the existing `task_id` as the caller-specified stable name.** Justification:
- `task_id` already exists on `TaskDescriptor`, is already accepted in the `/task` payload
  (`task_callback` reads `task_data['task_id']` at `:303`, auto-generates at `:298-299`), is
  already in `to_dict()` and already surfaced in `/task/status` and the web UI.
- Adding a parallel `name` field would create two near-identical identity concepts, duplicate
  the auto-generation fallback, and force every status/UI consumer to learn a second key.
- The only gap is that callers rarely set `task_id` today. We close that by documenting it and
  by having the `tasks:[...]` bundle accept a per-sub-task `task_id` (or `name` alias →
  normalized to `task_id` at ingest, see API §M3). Auto-generation stays as the fallback.

So "targeted stop by name" means **stop by `task_id`**. A `name` alias is accepted on input
purely for ergonomics and is copied into `task_id`.

### D2. `stop` with NO target — DEFAULT (FLAGGED for user)

`stop` is doubly overloaded today: it is the DRIVE handler (`execute_stop`) AND the queue
cancel sentinel (`cancel_task_type='stop'`, `task.py:73`). Under lanes, a bare `stop`
(no `target`, no `scope`) needs a defined meaning. Two candidates:

- **Option D2-a (RECOMMENDED): bare `stop` = stop the DRIVE lane only.** Cancels the running
  DRIVE task, emits the physical stop, leaves LED/MATRIX/CONFIG running. This matches the
  physical meaning of "stop" (stop moving) and is the least-surprising default. It also keeps
  the existing single-lane test semantics identical (in the tests every task shares one lane,
  so "DRIVE-lane stop" == "stop the running task" == today's behavior).
- **Option D2-b: bare `stop` = cancel ALL lanes + drain queue (panic).** Simpler mental model
  ("stop means stop everything") but surprising for someone who only wanted to stop motion
  while a light show continues, and a heavier default.

**>>> OPEN DECISION FOR USER: choose D2-a (DRIVE-only, recommended) or D2-b (stop-all).** The
plan is written assuming **D2-a**. Switching to D2-b changes only the bare-`stop` branch in
`process_tasks` and one test; everything else is identical.

Regardless of D2 choice, we add an **explicit panic halt**:
- `{"task_type":"stop","parameters":{"scope":"all"}}` (alias `task_type:"halt"`) cancels every
  lane's running task and drains the queue. This is the unambiguous panic button.

### D3. Targeted stop vs the cancel sentinel — EXTEND, keep back-compat

The existing zero-delay cancel sentinel (`process_tasks` `:103-111`) is **kept and
generalized**, not replaced. New decision tree for a `stop` task reaching the head of the
queue (evaluated in `process_tasks`, before per-lane promotion):

1. `stop` with `parameters.target == "<task_id>"` → **targeted cancel**: find that task in any
   lane slot (or in the queue) and cancel only it; clear its lane slot; file CANCELLED to
   history. The `stop` token itself is consumed (filed COMPLETED) and does NOT additionally
   stop DRIVE.
2. `stop` with `parameters.scope == "all"` (or `task_type=="halt"`) → **halt**: cancel every
   lane slot, mark each CANCELLED, drain the queue, file the halt COMPLETED.
3. `stop` with neither, and `delay == 0.0` → **default (D2-a)**: lane-scoped cancel of the
   DRIVE slot's running task, exactly as the old sentinel but DRIVE-scoped, then the `stop`
   runs as the DRIVE one-shot (emit physical stop).
4. `stop` with `delay > 0.0` → not a sentinel; it queues into the DRIVE lane like any task
   (preserves `test_stop_with_delay_does_not_cancel_current`).

`cancel_task_type` class attr (`task.py:73`) stays and is still honored: the sentinel match
uses `task_type.lower() == self.cancel_task_type.lower()`. Custom sentinels (the
`AbortingExecutor` test) keep working — an `abort` with no target/scope falls into branch 3
and cancels the single (lane-less default) slot.

---

## SCOPE OPTIONS — pick v1 or v2

### v1-minimal (RECOMMENDED) — effort: Medium, risk: Low

- Three real lanes **DRIVE / LED / MATRIX**; **CONFIG = "always admit, no slot"** (collision/
  stabilization fire-and-forget; they tick once and complete).
- `custom` and `jumping_bean` run **EXCLUSIVE**: they occupy ALL lanes (a sentinel lane set
  meaning "whole robot"); nothing else promotes while one is running, and they only promote
  when every lane is free. This defers their multi-lane refactor and quarantines
  `jumping_bean`'s blocking sleep (it only blocks itself, same as today's serial behavior).
- Lane map, per-lane shared-queue scheduler, targeted/halt/DRIVE stop, `tasks:[...]` bundle,
  `lane`/`running_lanes` status, full test suite.
- Delivers the actual goal ("roll + blink + animate together") with maximal reuse.

### v2-full — effort: High, risk: Medium

Everything in v1 PLUS:
- Decompose `custom` at **ingest** into per-lane sub-tasks (split its `commands` list by
  actuator into a DRIVE timeline, an LED timeline, a MATRIX timeline) so a custom routine runs
  concurrently across lanes instead of exclusively. This is a non-trivial interpreter rewrite
  and changes `custom`'s observable timing semantics → needs new tests + careful review.
- Refactor `execute_jumping_bean` from the blocking `time.sleep` loop into a **two-phase**
  handler (stash `next_flip_time`, `flip_count`, `current_heading/speed` in `task.parameters`;
  emit one roll per due interval; return `True` when `duration` elapses) so it can share DRIVE
  + CONFIG lanes without freezing the loop. Fixes the latent 100 Hz-freeze bug.

**Recommendation: ship v1 first.** It delivers the headline capability with low risk and
contains the two hazardous handlers behind an "exclusive" rule. Schedule v2 as a follow-up
once v1 is validated on hardware, because v2's `custom` decomposition and `jumping_bean`
refactor are the only Medium-risk pieces and they are independent of the lane engine.

The rest of this plan specifies v1 concretely and marks **[v2]** on the deltas v2 adds.

---

## Detailed Plan

### Step 1 — Data model: lanes + per-lane slots — `core/common/task.py`

- Action:
  - Add a `Lane` concept. Use plain lowercase strings (`'drive'`, `'led'`, `'matrix'`,
    `'config'`) plus a special multi-lane marker. Define module constants:
    `LANE_DRIVE='drive'`, `LANE_LED='led'`, `LANE_MATRIX='matrix'`, `LANE_CONFIG='config'`,
    `LANES = (LANE_DRIVE, LANE_LED, LANE_MATRIX)` (the three slotted lanes), and
    `EXCLUSIVE_LANES = frozenset(LANES)` (the "whole robot" set used by custom/jumping_bean).
    `DEFAULT_LANE = LANE_DRIVE`.
  - `TaskDescriptor` gains `lanes: frozenset[str] = field(default_factory=lambda: frozenset({LANE_DRIVE}))`
    — the SET of lanes the task occupies (a normal task has one; custom/jumping_bean have all).
    Add `lanes` to `to_dict()` as a sorted list (`'lanes': sorted(self.lanes)`), plus a derived
    scalar `'lane'` = the single lane if `len==1` else `'multi'` (status convenience).
    `task_id` is unchanged and is the stable caller name (Decision D1).
  - `TaskExecutorBase.__init__`: replace `self.current_task` with
    `self.current_tasks: Dict[str, Optional[TaskDescriptor]] = {ln: None for ln in LANES}`.
    Keep one shared `self.task_queue` (preserves FIFO fairness + queue introspection) and one
    `self.task_history`.
  - **Back-compat property** `current_task`:
    ```python
    @property
    def current_task(self):
        return self.current_tasks[LANE_DRIVE] or next(
            (t for t in self.current_tasks.values() if t is not None), None)
    @current_task.setter
    def current_task(self, value):  # only used by tests that force state
        # clear all lanes, then place value in its lane(s)
    ```
    Rationale: existing tests read AND write `current_task` (e.g.
    `TestCancelOnStop.test_custom_cancel_task_type_via_class_attr` sets
    `ex.current_task = None` at test line 419). The setter must clear all lane slots; setting a
    non-None value places it under each of its `lanes`. This keeps every existing test working
    without edits.
  - Add `lane_of(self, task) -> frozenset[str]`: returns `task.lanes`. Lane assignment is
    decided at **ingest/registration time** (Step 2), not here, so the base class stays
    robot-agnostic — `TaskDescriptor` simply carries the resolved `lanes` set. Default for a
    descriptor with no explicit lanes = `{DEFAULT_LANE}` (= drive), which makes the generic
    `RecordingExecutor` tests serial (all tasks land in the single drive lane).
  - Add helper `running_tasks(self) -> List[TaskDescriptor]`: distinct non-None slot values.
- Files: `core/common/task.py`.
- Expected outcome: data model supports N lane slots while every existing `current_task`
  read/write still resolves to a single task → existing tests untouched.

### Step 2 — Lane map (task_type → lanes) — `core/sphero/sphero_task_executor.py`

- Action:
  - Add a module-level dict next to `_register_default_handlers`, mapping each registered
    `task_type` to its lane set (from the design taxonomy):
    ```
    TASK_LANES = {
      'move_to': {DRIVE}, 'patrol': {DRIVE}, 'square': {DRIVE}, 'circle': {DRIVE},
      'spin': {DRIVE}, 'roll': {DRIVE}, 'heading': {DRIVE}, 'speed': {DRIVE},
      'reflect': {DRIVE}, 'stop': {DRIVE},
      'set_led': {LED}, 'led_sequence': {LED},
      'matrix': {MATRIX}, 'matrix_sequence': {MATRIX},
      'collision': {CONFIG},
      'custom': EXCLUSIVE_LANES,        # v1: all three lanes (exclusive)
      'jumping_bean': EXCLUSIVE_LANES,  # v1: all three lanes (exclusive)
    }
    ```
  - Add `SpheroTaskExecutorBase.lanes_for(task_type) -> frozenset[str]`: returns
    `TASK_LANES.get(task_type.lower(), {DEFAULT_LANE})`. This is the single source of truth;
    kept beside the handler registry so they're maintained together (a comment notes "add a
    lane entry whenever you register a handler").
  - **CONFIG note:** CONFIG tasks (`collision`) are slotless in v1 ("always admit"). The
    scheduler treats a CONFIG-only task as never blocked and never blocking: it ticks once and
    completes (all CONFIG handlers are one-shot). Implementation detail in Step 3.
- Files: `core/sphero/sphero_task_executor.py`.
- Expected outcome: any Sphero task resolves to a lane set; generic test handlers (no entry)
  default to DRIVE → serial.

### Step 3 — Scheduler rewrite: `process_tasks` — `core/common/task.py`

Rewrite `process_tasks` (currently `:94-141`) to the per-lane model. New tick order:

1. **Sentinel / stop handling (generalized cancel hook).** While the queue head is the
   `cancel_task_type` sentinel, apply Decision D3's decision tree:
   - `target` present → find the task with that `task_id` among `current_tasks` slots (and the
     queue); if found, set CANCELLED, `completed_at`, file to history, clear its lane slot(s)
     (or remove from queue). Pop the sentinel, file it COMPLETED. (No physical DRIVE stop.)
   - `scope=='all'` / `halt` → for each occupied lane: CANCELLED + history + clear; clear the
     whole `task_queue`; pop + COMPLETE the sentinel.
   - else if `delay == 0.0` → DRIVE-scoped (D2-a): if `current_tasks[DRIVE]` set, CANCELLED +
     history + clear DRIVE slot; then let the sentinel fall through to normal DRIVE promotion
     so `execute_stop` emits the physical stop. (Preserves
     `test_stop_with_no_delay_cancels_current_task`: generic tasks are DRIVE-lane.)
   - else (`delay>0`) → leave it; it's a normal queued DRIVE task.
   Keep `cancel_task_type` honored so `AbortingExecutor` works (its `abort` with no
   target/scope/delay hits the `delay==0.0` branch and cancels the single occupied slot).
2. **Per-lane promotion with per-lane `start_at` gating.** Scan `task_queue` once, front to
   back. For each queued task `t` with lane set `L`:
   - Skip if any lane in `L` is already occupied (`current_tasks[ln] is not None`).
     For an **exclusive** task (`L == EXCLUSIVE_LANES`), it promotes only when ALL of
     DRIVE/LED/MATRIX slots are free; conversely, while an exclusive task runs, every lane is
     occupied so nothing else promotes.
   - Apply the **per-lane `start_at` gate**: if `t.start_at is not None and time.time() <
     t.start_at`, this task is not yet due. Mark its lanes as "reserved-but-gated" for THIS
     tick **only for the lanes it would occupy**, so a gated task does NOT block tasks behind
     it that need DIFFERENT lanes. **CRITICAL:** a future-gated DRIVE head must NOT block a due
     LED task. Implementation: keep a local `gated_lanes` set; when we encounter a gated task,
     add its lanes to `gated_lanes` and `continue`; a later task may still promote if its lanes
     are disjoint from BOTH occupied lanes AND `gated_lanes`. (Adding to `gated_lanes` keeps
     per-lane FIFO: a due LED task behind a gated LED task still waits, but a due LED task
     behind a gated DRIVE task proceeds.)
   - If not skipped and due: pop `t` from the queue, set RUNNING, stamp `started_at`, and
     occupy every lane in `L` (`current_tasks[ln] = t`). CONFIG-only tasks: since CONFIG is not
     in `LANES`, they don't consume a slot — handle by promoting them into a transient
     "run-once now" path (tick immediately in step 3 via a small CONFIG list), OR simpler:
     include `'config'` in `current_tasks` as a slot that is cleared the same tick it completes
     (all CONFIG handlers are one-shot, so it occupies for exactly one tick). **Chosen: add
     `LANE_CONFIG` to the slot dict** but NOT to the gating/exclusivity set, so a CONFIG task
     never blocks DRIVE/LED/MATRIX and vice-versa, yet still flows through the identical
     promote→tick→complete path (minimal special-casing).
   - Note FIFO fairness: scanning front-to-back and skipping lane-busy tasks gives "first
     queued task whose lanes are free runs first," preserving FIFO within each lane.
3. **Tick each occupied lane's handler.** For each distinct running task (dedupe multi-lane
   tasks so an exclusive task ticks once, not three times), run the existing two-phase logic
   (`execute_task` → on `True`: COMPLETED unless handler preset status; `completed_at`;
   history; clear all its lane slots; on exception: FAILED + message + clear). This is the
   verbatim `:125-139` body, now executed per distinct running task. Two-phase handler state
   stays in `task.parameters`; **confirmed no cross-lane collision** because concurrent tasks
   are in disjoint lanes and never touch the same actuator or `parameters` dict.
4. **Return value:** return `self.running_tasks()` (a `List[TaskDescriptor]`). The
   `current_task` property still works for old single-slot callers/tests.

- Files: `core/common/task.py`.
- Expected outcome: parallel lanes tick independently; gated DRIVE head never blocks a due LED
  task; cancel sentinel + targeted stop + halt all handled; serial DRIVE stream behaves exactly
  as before.

### Step 4 — Node ingest + loop + status — `sphero_instance_task_controller_node.py`

- **`task_callback` (`:276-337`) — accept single task OR a bundle, resolve lanes:**
  - Branch on payload key:
    - **(a) single** (`'task_type'` present, no `'tasks'`): build one `TaskDescriptor` as today
      (`:302-306`); set `task.lanes = self.task_executor.lanes_for(task.task_type)` (or honor an
      explicit `task_data['lane']`/`lanes` override if present); compute `start_at` from
      `now`/`start_offset` (`:308-316`) unchanged; `add_task`; status. Auto `task_id` fallback
      unchanged (`:298-299`).
    - **(b) bundle** (`'tasks'` is a list): compute ONE shared `start_at` from top-level
      `now`/`start_offset`. For each sub-task: normalize `name`→`task_id` (alias), auto-generate
      `task_id` if absent, resolve `lanes`, set the shared `start_at` on each. **Validate lane
      disjointness:** if two sub-tasks share a lane (or any sub-task is exclusive alongside
      others), **reject the whole bundle** with a logged error (v1 keeps semantics obvious).
      Then `add_task` each; publish one status per sub-task (or a single bundle status — see
      below). The shared `start_at` makes all lanes fire on the same NTP instant (rides the
      existing broadcast `now`/`start_offset` mechanism unchanged).
  - **Targeted stop ingest:** a `{"task_type":"stop","parameters":{"target":"<task_id>"}}` (or
    `scope:"all"` / `task_type:"halt"`) is just a normal single task whose `parameters` carry
    `target`/`scope`; `add_task`s it; the scheduler (Step 3) interprets it. Resolve its lanes to
    DRIVE (harmless; the sentinel path consumes it before promotion for target/halt cases).
- **`task_execution_loop` (`:363-396`) — diff the running SET, not one slot:**
  - Replace `previous_task = self.task_executor.current_task` (`:366`) with
    `previous = set(id(t) for t in self.task_executor.running_tasks())` plus a snapshot map
    `{id: task}`. Call `process_tasks()` (now returns a list). Compute `current = {id(t):t}`.
    For each task that left the running set (`in previous, not in current`) → it finished/
    cancelled → `publish_task_status` + the duration log (`:378-381`). For each task newly in
    the set → `publish_task_status` + the "Starting task" log (`:387-396`), updating
    `current_position` from state as today. This generalizes the single-slot identity diff to a
    set diff; per-lane starts/finishes are reported independently.
- **`publish_task_status` (`:398-411`) — add lane fields:**
  - `status_dict` still starts from `task.to_dict()` (now includes `lanes`/`lane`).
  - Add `status_dict['running_lanes'] = {ln: (t.task_id if t else None) for ln, t in
    self.task_executor.current_tasks.items()}` so the dashboard can show which lanes are busy.
  - Keep `queue_length`; redefine `has_current_task` = `any(running_tasks())`; `total_pending`
    = `len(task_queue) + len(running_tasks())`. (Additive; old consumers ignore new keys.)
- Files: `sphero_instance_task_controller_node.py`.
- Expected outcome: node ingests single + bundle + targeted-stop payloads; loop reports each
  lane transition; status carries `lane`/`lanes`/`running_lanes`.

### Step 5 — Message / API shape (`/api/task` + broadcast) — minimal & back-compat

Three accepted shapes (web layer NOT implemented here — note shape, don't build UI):

- **(a) single task (unchanged):** `{ "task_type":"roll", "parameters":{...} }` (+ optional
  `task_id`/`name`, optional `now`/`start_offset`). Lane inferred from `task_type`. Back-compat
  for every existing caller, SM-generated task, and the broadcast feature.
- **(b) concurrent bundle:**
  ```jsonc
  {
    "tasks": [
      {"task_type":"roll",   "name":"r1", "parameters":{"heading":0,"speed":100}},
      {"task_type":"set_led","name":"l1", "parameters":{"color":"red"}},
      {"task_type":"matrix", "name":"m1", "parameters":{"pattern":"smile"}}
    ],
    "now": 1735600000.0,   // optional (coordinator/broadcast)
    "start_offset": 3.0    // optional → ONE shared start_at for all sub-tasks
  }
  ```
  Reject if two sub-tasks share a lane or any is exclusive in a multi-item bundle.
- **(c) targeted stop / halt:**
  - `{ "task_type":"stop", "parameters":{"target":"r1"} }` → cancel only task `r1`.
  - `{ "task_type":"stop", "parameters":{"scope":"all"} }` (or `"task_type":"halt"`) → panic.
  - `{ "task_type":"stop" }` → bare default = DRIVE-only (D2-a) [or stop-all if user picks D2-b].
- The Flask `/api/task` relay forwards the JSON onto `/sphero/<name>/task` unchanged — the
  bundle/target keys ride through transparently; only the node parses them. No web code in this
  plan; flag that `multirobot_webserver` static JS that renders `/task/status` will see new
  `lane`/`running_lanes` keys (additive, safe to ignore) and could later show per-lane state.

### Step 6 — [v2 only] custom decomposition + jumping_bean refactor

- **[v2]** `custom` ingest decomposition in `task_callback`: split `commands` by actuator into
  up to three lane-tagged sub-tasks sharing one `start_at`, replacing the single exclusive
  `custom`. Remove `custom` from `EXCLUSIVE_LANES`; it becomes a bundle expansion. New tests
  for cross-lane custom timing.
- **[v2]** `execute_jumping_bean` two-phase rewrite (`handlers:452-479`): stash
  `next_flip_time`, `flip_count`, `current_heading`, `current_speed`, `end_time` in
  `task.parameters`; per tick, if `time.time() >= next_flip_time` emit one roll + advance;
  return `True` when `time.time() >= end_time`, then emit stop + re-enable stabilization. Keeps
  DRIVE+CONFIG lanes; no `time.sleep`. Update `test_jumping_bean_*` to drive via clock ticks.
- Files: `sphero_instance_task_controller_node.py`, `core/sphero/sphero_task_handlers.py`,
  `core/sphero/sphero_task_executor.py` (drop custom/jumping_bean from EXCLUSIVE), tests.

### Step 7 — Build & test

- Commands:
  - `colcon build --packages-select sphero_instance_controller`
  - `source install/setup.bash`
  - `colcon test --packages-select sphero_instance_controller --event-handlers console_direct+`
    (or `pytest src/sphero_instance_controller/test/test_task_executor.py -v`).
- Expected outcome: all existing + new tests green.

---

## Lane assignment table (taxonomy — v1)

| task_type | lane set (v1) | notes |
|---|---|---|
| move_to, patrol, square, circle, spin, roll, heading, speed, reflect, stop | `{DRIVE}` | serial within DRIVE |
| set_led, led_sequence | `{LED}` | |
| matrix, matrix_sequence | `{MATRIX}` | |
| collision | `{CONFIG}` | slotless: never blocks/blocked, one-shot |
| custom | `EXCLUSIVE` (all 3) | v1 exclusive; **[v2]** decomposed per-lane |
| jumping_bean | `EXCLUSIVE` (all 3) | v1 exclusive; **[v2]** two-phase, DRIVE+CONFIG |
| (unmapped / generic test types) | `{DRIVE}` (default) | keeps existing tests serial |

---

## Back-compat checklist (must stay green or be explicitly changed)

1. **Serial DRIVE stream** → identical (one DRIVE lane + shared queue). ✅ no change to
   `TestQueueLifecycle`.
2. **`current_task` reads & writes** → preserved via property + setter; `RecordingExecutor`
   tests and the `ex.current_task = None` reset (test `:419`) keep working. ✅
3. **Cancel sentinel** (`TestCancelOnStop`, `:354-427`) → generalized hook, generic tasks are
   DRIVE-lane, bare `stop`/`abort` (delay 0) cancels the single occupied slot. ✅
   `test_stop_with_delay_does_not_cancel_current` → `delay>0` still queues, no cancel. ✅
   `test_custom_cancel_task_type_via_class_attr` → custom sentinel honored. ✅
4. **`start_at` gating** (`TestSynchronizedStart`, `:247-311`) → per-lane gate reuses the same
   comparison; single-lane generic tasks behave exactly as before, INCLUDING
   `test_future_head_blocks_later_queued_tasks` (gated DRIVE head + DRIVE task behind → behind
   still waits because same lane). ✅
5. **Sphero handler smoke** (`TestSpheroHandlerSmoke`, `:435-654`) → handlers unchanged in v1
   (jumping_bean/custom untouched until v2); each single task lands in one lane and ticks as
   before. ✅
6. **SM-generated tasks & single `/api/task`** → shape (a) unchanged; lanes inferred. ✅
7. **Synchronized-start / broadcast** (`now`/`start_offset`) → unchanged for shape (a);
   bundle (b) applies ONE shared `start_at` to all sub-tasks. ✅
8. **[v2 only]** `test_jumping_bean_*` and `test_custom_*` CHANGE (rewritten for two-phase /
   decomposed semantics) — the only intentionally-modified existing tests, and only in v2.

---

## Test Plan (new tests in `test/test_task_executor.py`)

New suite `TestConcurrentLanes` (uses `RecordingSphero` so lanes resolve via `TASK_LANES`):

1. **Parallel lanes tick independently:** queue `roll` (DRIVE), `set_led` (LED), `matrix`
   (MATRIX); one `process_tasks()` → all three `_send_*` emitted in the same tick; three lanes
   occupied; `running_tasks()` has 3.
2. **Per-lane start_at gating independence (THE critical test):** a DRIVE task gated 5s in the
   future queued AHEAD of a due `set_led`; one tick → LED promotes and emits NOW, DRIVE stays
   pending; after `clock.advance(5)` DRIVE promotes. Asserts the gated DRIVE head does NOT
   block the due LED task.
3. **Per-lane FIFO within a lane still gates:** two LED tasks, first gated 5s; the second LED
   task waits (same lane) → confirms `gated_lanes` blocks same-lane followers.
4. **Lane conflict queues within a lane:** two `roll` tasks → second waits until first
   completes (serial DRIVE), LED/MATRIX unaffected.
5. **Targeted stop by name:** start `roll`(id=`r1`,DRIVE) + `led_sequence`(id=`l1`,LED);
   submit `stop target=r1` → only `r1` CANCELLED, `l1` still RUNNING; `r1` cleared from DRIVE
   slot, in history CANCELLED.
6. **Halt / stop scope=all:** with DRIVE+LED+MATRIX running, `stop scope=all` → all three
   CANCELLED, queue drained.
7. **Bare stop default (D2-a):** DRIVE `roll` + LED `set_led` running; bare `stop` → DRIVE
   cancelled + physical stop emitted, LED still running. (If user picks D2-b, this test asserts
   all lanes cancelled instead.)
8. **Exclusive custom/jumping_bean:** while a DRIVE task runs, a queued `custom` does NOT
   promote until DRIVE (and all lanes) free; once promoted, a queued `set_led` does NOT promote
   until custom completes.
9. **Bundle ingest reject on lane clash:** (node-level, light) two `set_led` sub-tasks in one
   `tasks:[...]` → rejected. (Can be a thin unit test on a bundle-validation helper extracted
   from `task_callback` to keep it node-free.)
10. **Back-compat serial behavior:** a stream of `roll` tasks behaves one-at-a-time, FIFO,
    identical to pre-change (regression guard).
11. **[v2]** custom decomposed across lanes; **[v2]** jumping_bean two-phase emits flips on
    clock ticks without `time.sleep`, re-enables stabilization at end.

---

## Risks & mitigations

- **BLE command-rate saturation across 3 lanes/tick:** up to 3 lanes can emit per 10ms tick.
  Steady state is low (immediate handlers emit once then complete; `led_sequence`/
  `matrix_sequence` are interval-gated at 1s/2s). Mitigation: ship v1 as-is; if hardware
  profiling shows backpressure, add a per-lane "last command de-dupe / min-interval" guard on
  the emission side (out of scope for v1 unless observed). FLAG for hardware validation.
- **Handler-state assumptions:** state lives in `task.parameters`; cross-lane tasks never share
  an actuator or dict → safe. The only shared executor reads are read-only config/state.
  Verified against every handler's state keys in the design (§1.2).
- **`jumping_bean` blocking `time.sleep`** (`handlers:452-479`): freezes the 100Hz loop today.
  v1 quarantines it (exclusive → only blocks itself, same as current serial behavior) and
  records a TODO; **[v2]** fixes it via two-phase. Do NOT silently regress.
- **Web UI / status contract ripple:** `/task/status` gains `lanes`/`lane`/`running_lanes`;
  `has_current_task`/`total_pending` redefined over the running set. Additive — old JS ignores
  new keys. Flag a follow-up web task to render per-lane state (not in this plan).
- **`current_task` property/setter correctness:** the setter is only exercised by tests that
  force state; must clear all slots then place by lane. Covered by keeping existing tests
  green (they ARE the regression guard).

---

## Files touched

- `core/common/task.py` — Lane constants, `TaskDescriptor.lanes` + `to_dict`, `current_tasks`
  slots, `current_task` compat property/setter, `running_tasks()`, `process_tasks` rewrite.
- `core/sphero/sphero_task_executor.py` — `TASK_LANES` map, `lanes_for()`.
- `sphero_instance_task_controller_node.py` — `task_callback` single/bundle/targeted-stop
  ingest + lane resolution, `task_execution_loop` set-diff, `publish_task_status` lane fields.
- `core/sphero/sphero_task_handlers.py` — **[v2 only]** `jumping_bean` two-phase.
- `test/test_task_executor.py` — `TestConcurrentLanes` suite; **[v2]** updated custom/jb tests.
- (web `multirobot_webserver/*` — NOT touched; status-shape note only.)

## Approval Status
- [ ] Waiting for user approval
- [ ] Approved
- [ ] Executed

## OPEN DECISIONS FOR USER
1. **Scope:** v1-minimal (recommended) vs v2-full.
2. **Bare `stop` (no target/scope) default:** D2-a DRIVE-only (recommended) vs D2-b stop-all.
