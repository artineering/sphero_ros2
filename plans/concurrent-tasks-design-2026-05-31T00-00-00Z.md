# Concurrent Task Execution on a Single Sphero — Design Proposal

**Created:** 2026-05-31T00-00-00Z
**Status:** Design Exploration (NO implementation in this document)
**Scope:** `sphero_instance_controller` task executor core
**Author role:** ROS2 SME

## Task Description

Today a Sphero instance task controller runs exactly ONE task at a time. The executor
holds a single `current_task` slot; any newly-arriving task queues behind the running one
(`core/common/task.py::process_tasks`, lines 94-141). We want a robot to be able to do
several physically-independent things at once — e.g. **roll while blinking its main LED
while animating its 8x8 matrix**. Concurrency must respect physical actuator limits: two
tasks that drive the same actuator (two motion tasks, two LED tasks) still cannot run
together.

This document: (1) builds the actuator-resource taxonomy from the actual registered
handlers, (2) presents three executor designs with honest trade-offs, (3) specifies
status/lifecycle and cancel semantics under concurrency, (4) covers back-compat with the
serial model, the `stop` cancel sentinel, and the just-added `start_at` synchronized start,
(5) proposes a minimal API/message shape, and (6) bounds scope and risk. **No code is
written here.**

---

## 1. Current execution model (verified against source)

All line numbers are real, read from the repo on branch `deploy`.

### 1.1 The single-slot tick loop — `core/common/task.py`

- `TaskExecutorBase` (`task.py:60`) owns three lists and a handler registry:
  `task_queue` (`:76`), `current_task: Optional[TaskDescriptor]` (`:77`), `task_history`
  (`:78`), `_handlers` (`:79`).
- `add_task` (`:90`) appends to the back of `task_queue` (FIFO).
- `process_tasks` (`:94`) is the heart — it runs **one tick** and does three things in order:
  1. **Cancel-on-stop hook** (`:103-111`): if `current_task` is set AND the head of the
     queue is the `cancel_task_type` (`'stop'`, `:73`) with `delay == 0.0`, it marks the
     running task `CANCELLED`, files it to history, and clears the slot. This is the only
     pre-emption path today.
  2. **Promotion / `start_at` gate** (`:117-122`): if the slot is empty and the queue head
     is due (`head.start_at is None or time.time() >= head.start_at`), pop it, set
     `RUNNING`, stamp `started_at`. A future-dated head **blocks the whole queue** (strict
     FIFO; head-of-line). This is what synchronizes broadcast starts across units.
  3. **Tick the current task** (`:125-139`): call `execute_task` → handler. If it returns
     `True` (done), stamp `COMPLETED`/`completed_at`, file to history, clear slot. On
     exception, mark `FAILED` with the message. Returns `self.current_task`.
- `execute_task` (`:143`) looks up the handler by `task_type.lower()` and calls it; unknown
  type raises `ValueError` (caught by the tick → `FAILED`).

### 1.2 The two-phase handler contract — `core/sphero/sphero_task_handlers.py`

- Handler signature `(executor, task) -> bool` (`task.py:55-57`): **`False` = keep running,
  `True` = done**.
- Multi-tick handlers stash state **inside `task.parameters`** keyed by string. Examples:
  `move_to` uses `command_sent`, `last_distance`, `last_progress_time`, `stalled`
  (`handlers:83-105`); `circle`/`spin` use `start_time` (`:166`, `:286`); `led_sequence`/
  `matrix_sequence` use `current_index`, `last_change_time` (`:219-221`, `:246-248`);
  `patrol`/`square` use `current_waypoint_index`, generated `waypoints` (`:120`, `:198`);
  `custom` uses `current_command_index`, `command_start_time`, `command_executed`
  (`:315-318`).
- **Critical implication for concurrency:** handler state lives in the per-task
  `parameters` dict, NOT in the executor. So two DIFFERENT tasks ticked in the same loop do
  not collide on handler state — *as long as they don't drive the same actuator*. The
  collision risk is the physical actuator and the shared `executor.get_current_*` /
  `default_speed` knobs, not the state dict.
- **One handler that is NOT cooperative:** `execute_jumping_bean` (`:452-479`) calls
  `time.sleep(flip_interval)` in a Python `for` loop and blocks the whole loop for its full
  `duration` (default 10s). Under any concurrent model this would freeze every other lane.
  It must be flagged and refactored to two-phase, or barred from concurrency (see §6).

### 1.3 Command emission is per-actuator topics — `topic_task_executor.py` + node

This is the fact that makes parallel actuation *physically possible*: **each actuator is a
separate ROS topic.** `TopicTaskExecutor._send_*` (`topic_task_executor.py:39-112`) maps to
distinct command channels, and `publish_command` in the node
(`sphero_instance_task_controller_node.py:226-272`) routes each to its own publisher:

| `_send_*` method | command key | node topic (`sphero/<name>/…`) |
|---|---|---|
| `_send_raw_motor_command` | `raw_motor` | `/raw_motor` |
| `_send_roll_command` | `motion` (action=roll) | `/roll` |
| `_send_stop_command` | `motion` (action=stop) | `/stop` |
| `_send_spin_command` | `spin` | `/spin` |
| `_send_heading_command` | `heading` | `/heading` |
| `_send_speed_command` | `speed` | `/speed` |
| `_send_led_command` | `led` | `/led` |
| `_send_matrix_command` | `matrix` | `/matrix` |
| `_send_stabilization_command` | `stabilization` | `/stabilization` |
| `_send_collision_detection_command` | `collision` | `/collision` |

Drive, LED, and matrix are **already three independent command paths**. A concurrent
executor doesn't need new plumbing on the emission side; it needs the *scheduler* above
`execute_task` to allow more than one task to tick per loop.

### 1.4 The node loop — `sphero_instance_task_controller_node.py`

- `task_callback` (`:276-337`) ingests one task per `/task` message, optionally computes
  `start_at` from `now + start_offset` (`:310-316`), `add_task`s it, publishes status.
- `task_execution_loop` (`:363-396`) runs at **100 Hz** (timer period `0.01`, `:81-85`),
  captures `previous_task`, calls `process_tasks()`, and if the slot identity changed,
  publishes status for the finished and/or newly-started task. **This whole "did the single
  slot change?" diff is single-slot-specific** and is the main node-side thing concurrency
  changes.
- `publish_task_status` (`:398-411`) emits the task dict plus `queue_length`,
  `has_current_task`, `total_pending` — all derived from the single slot + queue.

---

## 2. Actuator-resource taxonomy

Every registered handler (`sphero_task_executor.py:41-58`) classified by the physical
resource(s) it controls. "Lane" = an actuator that can be driven by at most one task at a
time.

| task_type | Handler | Resource lane(s) | Notes / conflicts |
|---|---|---|---|
| `move_to` | `execute_move_to` | **DRIVE** | roll + stop; reads position |
| `patrol` | `execute_patrol` | **DRIVE** | delegates to `move_to` |
| `square` | `execute_square` | **DRIVE** | delegates to `patrol` |
| `circle` | `execute_circle` | **DRIVE** | raw_motor + stop |
| `spin` | `execute_spin` | **DRIVE** | spin + (implicit) heading |
| `roll` | `execute_roll` | **DRIVE** | roll |
| `heading` | `execute_heading` | **DRIVE** | sets heading (orientation) |
| `speed` | `execute_speed` | **DRIVE** | sets speed |
| `reflect` | `execute_reflect` | **DRIVE** | one-shot roll at reversed heading |
| `stop` | `execute_stop` | **DRIVE** (+ cancel sentinel) | stops motion; also the queue cancel token |
| `set_led` | `execute_set_led` | **LED** | main LED RGB |
| `led_sequence` | `execute_led_sequence` | **LED** | timed LED cycle |
| `matrix` | `execute_matrix` | **MATRIX** | one 8x8 frame |
| `matrix_sequence` | `execute_matrix_sequence` | **MATRIX** | timed 8x8 cycle |
| `collision` | `execute_collision` | **CONFIG** (sensor) | collision detection on/off; not an actuator |
| `jumping_bean` | `execute_jumping_bean` | **DRIVE + CONFIG (stabilization)** ⚠️ | **blocking sleep loop**; toggles stabilization AND rolls |
| `custom` | `execute_custom` | **MULTI (DRIVE / LED / MATRIX)** ⚠️ | per-command-step touches whichever actuator the step names |

### Lane model

Four logical lanes:

- **DRIVE** — motors / heading / speed / roll / spin / stop. The big one.
- **LED** — main RGB LED.
- **MATRIX** — 8x8 display.
- **CONFIG** — collision detection, stabilization. Mostly fire-and-forget config; could be
  its own lane or be treated as "lane-less, always admit."

### Can-run-concurrently matrix

| | DRIVE | LED | MATRIX | CONFIG |
|---|---|---|---|---|
| **DRIVE** | ✗ conflict | ✓ parallel | ✓ parallel | ✓ parallel |
| **LED** | ✓ | ✗ conflict | ✓ | ✓ |
| **MATRIX** | ✓ | ✓ | ✗ conflict | ✓ |
| **CONFIG** | ✓ | ✓ | ✓ ~ | ~ |

✓ = different actuators, physically independent → may run at once.
✗ = same actuator → second task conflicts with the first (must queue / pre-empt / reject).

### The two flagged special cases

- **`jumping_bean`** declares **DRIVE + CONFIG (stabilization)**. It is also the only
  blocking handler (`time.sleep` loop, `handlers:452-479`). Even in the serial model it
  freezes the 100 Hz loop for ~10 s. Under concurrency it would freeze *all* lanes. It must
  occupy BOTH DRIVE and CONFIG (a multi-lane task) AND be refactored to two-phase, or be
  declared non-concurrent (runs exclusive, blocks the lane model) for v1.
- **`custom`** is a mini-interpreter (`handlers:311-351`): each timed step emits an `led`,
  `roll`, `matrix`, or `stop`. Across its lifetime it may touch DRIVE, LED, and MATRIX. It
  cannot be cleanly assigned to one lane. Options: declare it a **multi-lane (DRIVE+LED+
  MATRIX) exclusive** task that grabs all three lanes for its duration (simplest, safe), or
  leave `custom` as a serial-only legacy escape hatch. Recommend: multi-lane-exclusive.

**Conclusion:** the taxonomy is clean for the three core lanes (DRIVE / LED / MATRIX). Only
`custom` and `jumping_bean` are multi-lane; both are handled by "a task may declare a SET of
lanes and must hold all of them."

---

## 3. Design options

All three share one new concept: **a task is tagged with the set of lanes it occupies.** The
lane set is derived from `task_type` via a static map (the taxonomy table above), with
`custom`/`jumping_bean` mapping to multi-lane sets. The options differ in how the scheduler
admits and ticks tasks.

### Option A — Resource-slotted executor (one running slot per lane) ★ recommended

**Mechanism.** Replace the single `current_task` with a dict of lane → slot:
`current: Dict[Lane, Optional[TaskDescriptor]]` plus a queue *per lane* (or one shared queue
filtered by lane at promotion). Each `process_tasks` tick iterates the lanes; for each lane
it runs the same promote-then-tick logic that exists today, scoped to that lane. Serial
within a lane, parallel across lanes. A multi-lane task (`custom`, `jumping_bean`) occupies
several lane slots at once and only promotes when ALL its lanes are free.

- Promotion per lane reuses the exact `start_at` gate (`task.py:117-122`) unchanged.
- Tick per lane reuses the exact two-phase contract and lifecycle (`:125-139`) unchanged —
  it just runs N times per loop, once per occupied lane.
- `stop` semantics become lane-scoped by default (stops DRIVE), with an explicit "stop all"
  variant (see §4).

**Pros.**
- Smallest conceptual delta: it's literally "the existing single-slot loop, replicated per
  lane." The handler contract, `start_at` gate, history, and `FAILED`/`CANCELLED` paths are
  reused verbatim. Lowest risk of subtle lifecycle bugs.
- Physically honest: lanes map 1:1 to actuators. Impossible to drive one actuator with two
  tasks.
- Natural back-compat: if every incoming task is DRIVE (today's common case) behavior is
  identical to serial. A single shared queue degrades gracefully.
- Easy minimal first increment: ship DRIVE+LED+MATRIX, treat CONFIG as "always admit," defer
  multi-lane.

**Cons.**
- Need a lane registry mapping task_type → lane(s). One static dict; low cost but must be
  kept in sync when handlers are added.
- Multi-lane tasks (custom/jumping_bean) need the "acquire all lanes or wait" rule — a small
  amount of extra logic.
- Per-lane queues vs one shared queue is a decision (recommend: one shared `task_queue`,
  promotion scans for the first task whose lanes are all free; preserves FIFO-ish fairness
  and keeps `task_queue` introspection working for status).

**Back-compat / migration.** `current_task` becomes a derived/compat property (e.g. "the
DRIVE slot, else any occupied slot") so existing node code and tests that read
`current_task` keep working during migration. The node's slot-change diff
(`node:366-396`) is rewritten to diff the *set* of running tasks.

**Complexity.** Low–Medium. Most code is reused; new code is the lane map + per-lane
iteration + multi-lane acquire.

### Option B — Multi-current set with conflict arbitration

**Mechanism.** Keep a single `running: List[TaskDescriptor]` set plus the conflict matrix
(§2). On each tick, attempt to promote the queue head: admit it only if its lane set is
disjoint from every running task's lane set; otherwise apply a policy (queue / reject /
pre-empt). Tick every running task each loop.

**Pros.**
- More general than A: the conflict relation is data, not structural. Could express richer
  rules later (e.g. "spin conflicts with heading but two heading-only tasks coalesce").
- Single queue + single running set is conceptually one level of indirection less than
  per-lane slots if you think in sets rather than lanes.

**Cons.**
- The admission policy (queue vs reject vs pre-empt on conflict) is a genuine design surface
  with more edge cases than A's "the lane is busy, wait." More ways to get FIFO/fairness and
  pre-emption semantics subtly wrong.
- `start_at` gating interacts awkwardly: a future-dated head currently blocks the *whole*
  queue (`task.py:119`). With a running set you must decide whether a gated head blocks
  admission of non-conflicting tasks behind it. More semantics to define and test.
- Harder to reason about "what is the robot doing right now" than A's explicit lanes.

**Back-compat.** Same compat-property trick needed; the conflict matrix must reduce to
"everything conflicts with everything" to reproduce serial mode, which is exactly A's lanes
collapsed — i.e. B's general form buys little over A for this hardware.

**Complexity.** Medium–High (policy surface).

### Option C — Composite "task group" (sub-tasks tagged by lane, started together)

**Mechanism.** Introduce a composite task: one descriptor carrying a list of sub-tasks, each
tagged with a lane, all sharing one `start_at`. The executor expands the group at promotion
into per-lane running tasks that start atomically together. Could layer on top of A or B for
the *scheduling*, but it's primarily an **API/grouping** construct.

**Pros.**
- Best ergonomics for the actual use case ("roll + blink + animate, together") and for the
  broadcast feature: one payload = one synchronized multi-actuator behavior with a single
  shared start instant.
- Atomic group start fits the `start_at` synchronized-start model perfectly: the *group* has
  one `start_at`; all lanes fire on the same shared NTP instant across all units.
- Group-level lifecycle ("the whole gesture is done when all lanes finish") is meaningful for
  SM/HRI choreography.

**Cons.**
- Doesn't by itself answer "what happens when a second, separate task arrives mid-group" —
  it still needs an underlying lane/conflict scheduler (A or B) to arbitrate. So C is not an
  alternative to A; it's a *layer on top*.
- Adds a new message shape (nested sub-tasks) and group lifecycle/status — more than the
  minimal increment needs.

**Complexity.** Medium as an API layer; depends on A/B underneath.

### Recommendation

**Adopt Option A (resource-slotted, DRIVE/LED/MATRIX lanes) as the engine, and add a thin
slice of Option C (a `tasks: [...]` list payload with a shared `start_at`) as the *request
shape* for "do these together."** This pairing:
- reuses the existing tick/lifecycle/`start_at` machinery almost verbatim (lowest risk),
- is physically honest about actuators,
- and gives the broadcast/synchronized-start feature exactly what it wants: one payload, one
  shared start, multiple lanes firing together.

Option B's generality isn't justified — on this hardware "conflict" *is* "same actuator,"
which lanes already express. Full group lifecycle (C's heavy form) is deferred.

---

## 4. Status / lifecycle semantics under concurrency

### Per-task lifecycle — unchanged

Each task keeps its own `TaskStatus` (`task.py:18-24`), `started_at`, `completed_at`,
`error_message`. The two-phase contract and the promote→tick→complete/fail flow
(`task.py:117-139`) run **per lane**, identical to today. A handler's internal state still
lives in its own `task.parameters`; since concurrent tasks are in *different* lanes they
never share an actuator and don't collide.

### `process_tasks` return value

Today it returns the single `current_task` (`task.py:141`). Under A it should return the
**set/list of currently-running tasks** (e.g. `List[TaskDescriptor]`, one per occupied lane).
For back-compat, keep a `current_task` property returning the DRIVE slot (or first occupied)
so old callers/tests don't break. The node's loop is updated to diff the *set* of running
tasks between ticks instead of the single slot identity (`node:366-396`).

### `publish_task_status`

Today it emits one task dict + `queue_length`/`has_current_task`/`total_pending`
(`node:398-411`). Under concurrency, publish **one status message per task** on a lane
transition (start/finish/cancel), each carrying its own `task_id` and adding a `lane` field
plus the per-lane occupancy summary, e.g.:
- `lane`: which lane this task occupies (`drive`/`led`/`matrix`/`config`/`multi`).
- `running_lanes`: map of lane → running `task_id` (or null), so the dashboard can show "3
  lanes busy."
- keep `queue_length` (shared queue) for compat.

The web layer already keys task status by `task_id`; adding `lane` is additive and
backward-compatible (old consumers ignore it).

### `stop` / cancel semantics — the important decision

Today `stop` is overloaded: it's both a DRIVE handler (`execute_stop`, `handlers:305-308`)
AND the queue cancel sentinel (`cancel_task_type = 'stop'`, `task.py:73`; cancel hook
`:103-111`). Under lanes this must be disambiguated:

- **Default `stop` = stop the DRIVE lane only.** It cancels the running DRIVE task and emits
  the stop command. LED/MATRIX keep going. This matches the physical meaning ("stop moving")
  and is the least-surprising default. The cancel-on-stop hook (`task.py:103-111`) becomes
  lane-scoped: a zero-delay `stop` cancels the DRIVE slot's current task, not all lanes.
- **Add an explicit "stop all" / `halt`** (new task_type, or `stop` with
  `parameters.scope == 'all'`) that cancels every lane and clears the queue — the panic
  button. This preserves the old "stop kills the in-flight task" behavior for callers that
  want it, made explicit.
- Per-lane cancel generalizes: a new task arriving for a busy lane can pre-empt that lane's
  current task (policy: pre-empt vs queue — recommend **pre-empt within a lane** for
  immediate commands like `set_led`/`matrix`/`roll`, since they're idempotent
  one-shots; queue for multi-tick lane tasks is also acceptable and can be a per-lane policy
  knob, but keep v1 simple: newest task pre-empts its lane).

**Recommended v1 cancel rule:** a new task for lane L cancels (pre-empts) the task currently
in lane L and takes the slot; `stop` pre-empts DRIVE; `halt`/`stop scope=all` pre-empts every
lane and drains the queue.

---

## 5. Back-compat requirements (must keep working)

1. **Serial behavior:** a stream of DRIVE-only tasks must behave exactly as today (one at a
   time, FIFO). Under A with a single shared queue and one DRIVE lane, this is automatic.
2. **Cancel sentinel:** `cancel_task_type = 'stop'` (`task.py:73`) and the zero-delay
   cancel hook (`:103-111`) keep working, scoped to DRIVE as in §4. The existing tests
   `TestCancelOnStop` (test file `:354-427`) should still pass with DRIVE-lane scoping
   (they use a generic executor where all tasks share one lane → identical behavior).
3. **`start_at` gating:** the synchronized-start gate (`task.py:117-122`) is reused
   per-lane unchanged. `TestSynchronizedStart` (`:247-311`) must still pass. With a shared
   queue, a future-dated head still gates promotion; verify a gated DRIVE task does not
   wrongly block an LED task that is due now — this is the one new semantic to define and
   test (recommend: gating is per-lane, so a gated DRIVE head does NOT block a due LED task).
4. **SM-generated tasks & single `/api/task` posts:** a single task with no lane info is
   classified by its `task_type` into its lane and runs as before. No payload change required
   for existing callers. The broadcast payload (`now`/`start_offset`, see broadcast plan
   `plans/broadcast-task-2026-05-30T21-24-50Z.md`) rides through unchanged.
5. **Existing tests:** `RecordingExecutor` (test `:66`) registers generic handlers with no
   lanes; under A it must default such tasks to a single lane so all current tests keep
   their serial semantics. Provide a "default lane" for unmapped types.

---

## 6. API / message shape

Keep it minimal and additive. Two accepted shapes on `/sphero/<name>/task` (and
`/api/task`):

**(a) Single task (today's shape) — unchanged, still serial within its lane:**
```jsonc
{ "task_type": "roll", "parameters": { "heading": 0, "speed": 100 } }
```
The executor derives the lane from `task_type`. No new fields required. Optional explicit
`"lane": "drive"` override is allowed but unnecessary.

**(b) Concurrent set (thin Option C) — a list sharing one start:**
```jsonc
{
  "tasks": [
    { "task_type": "roll",   "parameters": { "heading": 0, "speed": 100 } },
    { "task_type": "set_led","parameters": { "color": "red" } },
    { "task_type": "matrix", "parameters": { "pattern": "smile" } }
  ],
  "now": 1735600000.0,        // optional, from coordinator (broadcast)
  "start_offset": 3.0         // optional → shared start_at for ALL sub-tasks
}
```
Rules:
- Each sub-task is classified into its lane. If two sub-tasks in one payload map to the SAME
  lane → reject the payload (or accept and serialize them in that lane — recommend **reject
  with a clear error** in v1 to keep semantics obvious).
- `now`/`start_offset` (if present) compute ONE `start_at` applied to every sub-task, so all
  lanes fire together on the shared NTP instant — this is exactly how the broadcast feature
  already stamps a single `now` for all units (`plans/broadcast-task-…md`). The result:
  N units × M lanes all start at the same absolute instant.
- Back-compat: a payload with `task_type` (not `tasks`) is shape (a). `task_callback`
  (`node:276-337`) branches on which key is present.

This is the smallest surface that expresses "do these together with a shared start" while
leaving the single-task path untouched.

---

## 7. Scope, minimal increment, and risk

### Minimal first increment (recommended v1)

1. **Three lanes only: DRIVE / LED / MATRIX.** CONFIG (collision, stabilization) treated as
   "always admit, no slot." Defer formal CONFIG lane.
2. **Lane map** from `task_type` → lane, with a default lane for unmapped/generic types
   (keeps existing tests serial).
3. **Per-lane slot + single shared queue** (Option A). Promotion scans the queue for the
   first task whose lane(s) are free; tick all occupied lanes each loop.
4. **`stop` scoped to DRIVE**, add explicit **`halt` / stop-all**.
5. **Defer multi-lane tasks:** for v1, run `custom` and `jumping_bean` as **exclusive**
   (they grab all three lanes; nothing else runs while they do). This sidesteps `custom`'s
   multi-actuator interpreter and `jumping_bean`'s blocking sleep without having to refactor
   them yet. Note `jumping_bean`'s blocking sleep as a known limitation/TODO.
6. **API:** accept the `tasks: [...]` list payload (shape b) in addition to the single-task
   shape (a). Wire the shared `start_at`.
7. **Status:** add `lane` + `running_lanes` to status messages; node diffs the running SET.

This delivers "roll + blink + animate together" — the actual goal — with maximal reuse and
minimal new surface.

### Key risks

- **Handler state collisions:** LOW for cross-lane tasks (state is per-`task.parameters`).
  The real risk is two tasks in the SAME lane — prevented by the lane-slot invariant. The
  shared executor knobs (`get_current_position/heading`, `default_speed`,
  `position_tolerance`, `sphero_task_executor.py:33-35,60-68`) are read-only config or
  read-only state, safe to share across lanes.
- **Conflicting commands:** prevented structurally — only one task per actuator lane.
  CONFIG (`stabilization`, `collision`) is the soft spot: `jumping_bean` toggles
  stabilization; if it runs concurrently with a future stabilization task they'd fight.
  Mitigated by making `jumping_bean` exclusive in v1.
- **100 Hz loop now ticks several handlers:** at most ~4 lanes → ~4 handler calls per 10 ms
  tick. Negligible CPU. The genuine hazard is **BLE command-rate on the device controller**:
  three lanes can each emit a command every tick (LED + matrix + roll at up to 100 Hz). The
  device controller / BLE link can be saturated. Mitigation: the immediate-command handlers
  already mostly emit ONCE then complete (`set_led`, `matrix`, `roll` indefinite); the
  *sequence* handlers are interval-gated (`led_sequence` interval default 1 s,
  `matrix_sequence` 2 s, `handlers:216,243`). So steady-state emission is low. Still,
  recommend a per-lane command-rate guard / de-dupe on the emission side if profiling shows
  BLE backpressure.
- **`start_at` per-lane semantics:** new behavior (a gated DRIVE head must not block a due
  LED task). Must be explicitly designed and unit-tested, or the broadcast/sync feature
  could stall a lane. This is the single most important new test.
- **`jumping_bean` blocking sleep** (`handlers:452-479`): already a latent bug (freezes the
  100 Hz loop ~10 s even today); under concurrency it freezes all lanes. v1 makes it
  exclusive (so it only blocks itself), but the proper fix is a two-phase refactor — call
  out as a TODO, do not silently regress.
- **Test compat:** `RecordingExecutor`/`RecordingSphero` tests assume single-slot semantics.
  The default-lane mapping must reproduce serial behavior so the existing 60+ tests pass
  unchanged; add new tests for cross-lane parallelism, per-lane stop, and per-lane `start_at`
  gating.

---

## 8. Files that a future implementation would touch (for reference — NOT edited here)

- `src/sphero_instance_controller/sphero_instance_controller/core/common/task.py`
  — lane-aware slots/queue, per-lane promote+tick, `current_task` compat property.
- `src/sphero_instance_controller/sphero_instance_controller/core/sphero/sphero_task_executor.py`
  — lane map (task_type → lane), multi-lane/exclusive classification.
- `src/sphero_instance_controller/sphero_instance_controller/core/sphero/sphero_task_handlers.py`
  — `jumping_bean` two-phase refactor (deferred TODO); no change for v1 if made exclusive.
- `src/sphero_instance_controller/sphero_instance_controller/sphero_instance_task_controller_node.py`
  — `task_callback` accepts `tasks: [...]`; loop diffs running SET; `publish_task_status`
  adds `lane`/`running_lanes`.
- `src/sphero_instance_controller/test/test_task_executor.py`
  — new tests: cross-lane parallelism, per-lane stop vs halt, per-lane `start_at` gating;
  verify all existing tests still pass.

---

## Approval Status
- [ ] Waiting for user approval
- [ ] Approved
- [ ] Executed (design only — no implementation in this document)
