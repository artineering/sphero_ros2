# StateMachine: state-owned exits — 2026-05-08

## Context

`StateMachine` (`src/sphero_instance_controller/sphero_instance_controller/core/sphero/statemachine.py`, 545 lines) has a structural asymmetry: `DynamicState` owns `entry_condition_type` / `entry_condition_params`, but state-leaving lives entirely in `transitions[].condition`. The state has no concept of "I'm ready to leave."

**Redesign:** each state owns an `exits: []` list. Each entry pairs a `condition` with a `destination`. The top-level `transitions` array is removed entirely. A state with no `exits` (empty or absent) is a **leaf state** — once entered, the state machine stays there forever. Exposed via `DynamicState.isLeafState() -> bool`. **Breaks every existing config** — accepted tradeoff for cleaner semantics.

After the implementation, write unit tests for the **basic state mechanism only** — skip ROS2-callback wiring and the tasks API.

---

## Phase 1 — Redesign the class

### Schema

```json
{
  "name": "moving",
  "entry_condition": {"type": "always"},
  "exits": [
    {"condition": {"type": "topic_message", "topic": "/halt"},
     "destination": "stop"},
    {"condition": {"type": "topic_value", "topic": "/cmd",
                   "operator": "==", "value": "idle"},
     "destination": "idle"}
  ],
  "tasks": [...]
}
```

- `exits` is optional; empty or absent → leaf state.
- Each exit entry has `condition` (same shape as `entry_condition`) + `destination` (state name).
- Top-level `transitions` array is removed.
- `entry_condition` is preserved on `DynamicState` (parsed and stored) but at runtime it's only meaningful for the initial-state guard, which currently short-circuits to `True` anyway. Don't re-evaluate it for non-initial states this pass.

### Tick semantics

`process()` per tick:
- If `current_state.isLeafState()` → no transition.
- Else iterate `current_state.exits` in declared order; fire the first whose `condition` evaluates to `True`. On fire, call `transition_to_state(exit.destination)` and emit a `state_transition` event.
- If no exit fires → no transition.

### Condition types

Both `entry_condition` and exit `condition` use the same evaluator:
- `always` — always True.
- `timer` — `time.time() - state.entry_time >= params.duration`.
- `topic_value` — `topic_values[topic]` compared to `value` via `operator` (`==,!=,>,<,>=,<=`).
- `topic_message` — message received on `topic`; with `timeout`, must be within `<= timeout` seconds; without `timeout`, sticky once received.

Refactor: extract `_evaluate_condition(condition_dict, state) -> bool` from the existing `_check_entry_condition`. Reuse for both entry and exit. Entry usage stays restricted to `always` / `timer` / `topic_value` (validation rejects `topic_message` for entry).

### Class changes (`statemachine.py`)

- **Delete** `TransitionConditionType` enum (lines 25-30).
- **`DynamicState`** (lines 34-80):
  - Add `exits: List[ExitSpec]` field, parsed in `__post_init__` from `state.get('exits', [])`.
  - Add `isLeafState(self) -> bool` returning `len(self.exits) == 0`.
  - Keep `entry_condition_type` / `entry_condition_params` (initial-state book-keeping).
- **New** `@dataclass ExitSpec`: `condition_type: str`, `condition_params: Dict[str, Any]`, `destination: str`.
- **New** `_evaluate_condition(condition_type, params, state) -> bool` — shared evaluator for the 4 types.
- **`_check_transitions`** (lines 328-350): rewrite per tick semantics above.
- **Delete** `_check_transition_condition` (lines 352-410).
- **`_check_entry_condition`** (lines 412-455): becomes a thin wrapper around `_evaluate_condition`.
- **`validate_config`** (lines 149-230):
  - Remove top-level `transitions` validation.
  - Validate `exits[]` per state: each entry has `condition` + `destination`; destination must be a known state; condition must be valid (same rules as `entry_condition` plus `topic_message`).
  - Reject configs that still carry top-level `transitions` with an error message pointing at the migration.
- **`_build_state_machine`** (lines 232-272): subscriptions discovered from each state's `entry_condition` (when `topic_value`) and each state's `exits[].condition` (when `topic_value` / `topic_message`). No more transition-level discovery.
- **`get_status`** (lines 516-544): no schema change. Drop `num_transitions` (no top-level transitions) and add `num_exits` for the current state if useful.

---

## Phase 2 — Tests (basic state mechanism only)

**Scope:** unit tests for the state mechanism — config parsing, leaf detection, condition evaluation, exit firing, transitions, lifecycle. **Skip** ROS2 subscribe/unsubscribe callback wiring and the tasks API (`get_current_state_tasks` / `mark_tasks_completed`).

### Files

- `src/sphero_instance_controller/test/test_statemachine.py` — new.
- `src/sphero_instance_controller/test/conftest.py` — new, `sys.path` shim:
  `sys.path.insert(0, str(Path(__file__).resolve().parents[1]))`.
- No CMake / package.xml changes.

### Time control

`monkeypatch.setattr('sphero_instance_controller.core.sphero.statemachine.time.time', clock)` where `clock` is a tiny callable holder (`clock.now`, `clock.advance(dt)`, `clock.set(t)`, `__call__` returns `now`).

### Fixtures

- `clock`, `log` (list-backed logger), `sm(clock, log)` — fresh `StateMachine` with logger only (no subscribe callbacks since we're not testing ROS wiring).
- Inline configs: `LEAF_ONLY`, `TIMER_HOP`, `TOPIC_VALUE_HOP`, `TOPIC_MESSAGE_HOP`, `MULTI_EXIT`.

### Test groups

**A. Config validation**
- `test_validate_rejects_missing_states`
- `test_validate_rejects_unknown_initial_state`
- `test_validate_rejects_exit_unknown_destination`
- `test_validate_rejects_exit_missing_condition`
- `test_validate_rejects_exit_missing_destination`
- `test_validate_rejects_legacy_top_level_transitions`
- `test_validate_rejects_topic_message_on_entry_condition`
- `test_validate_accepts_leaf_state` (no `exits`)
- `test_validate_accepts_multi_exit_state`

**B. `configure` / parsing**
- `test_configure_sets_initial_state`
- `test_configure_stamps_entry_time`
- `test_configure_parses_exits_into_exitspec`
- `test_configure_invalid_returns_false`

**C. `isLeafState`**
- `test_state_with_no_exits_is_leaf`
- `test_state_with_empty_exits_array_is_leaf`
- `test_state_with_one_or_more_exits_is_not_leaf`

**D. Condition evaluation (`always`, `timer`, `topic_value`, `topic_message`)** — drive each through a real exit:
- `test_exit_always_fires_immediately`
- `test_exit_timer_blocks_before_duration`
- `test_exit_timer_fires_at_duration` (`>=` boundary)
- `test_exit_topic_value_per_operator` (parametrized over `== != > < >= <=`)
- `test_exit_topic_value_topic_never_received_does_not_fire`
- `test_exit_topic_value_compare_exception_does_not_fire`
- `test_exit_topic_message_does_not_fire_before_message`
- `test_exit_topic_message_fires_after_first_message`
- `test_exit_topic_message_within_timeout`
- `test_exit_topic_message_past_timeout_does_not_fire`
- `test_exit_topic_message_at_timeout_boundary` (`<=` inclusive)

**E. Multi-exit ordering**
- `test_multi_exit_picks_first_matching_in_declared_order`
- `test_multi_exit_skips_non_matching` (second exit wins when first's condition is False)
- `test_multi_exit_no_match_no_transition`

**F. Leaf-state behavior**
- `test_leaf_state_never_transitions`
- `test_leaf_state_process_returns_status_with_no_event`

**G. `transition_to_state`**
- `test_transition_to_state_happy_path`
- `test_transition_to_state_unknown_returns_false`
- `test_transition_to_state_resets_entry_time`

**H. State timeout** (`process`, lines 306-315)
- `test_state_timeout_emits_event_after_threshold` (`>` strict)
- `test_state_timeout_no_event_at_exact_threshold`
- `test_no_timeout_field_no_event`

**I. `process` lifecycle**
- `test_process_unconfigured_returns_none`
- `test_process_no_event_returns_status_dict`
- `test_process_emits_state_transition_event_shape` (`{type, from, to, timestamp}`)

**J. Updaters**
- `test_update_topic_value_stamps_last_received`
- `test_update_topic_value_overwrites`

**K. `get_status`**
- `test_get_status_unconfigured`
- `test_get_status_configured_basic_shape`
- `test_get_status_time_in_state_uses_clock`

### Out of scope for the test pass

- Subscribe / unsubscribe callback invocation (no ROS2 callback tests).
- Tasks API (`get_current_state_tasks`, `mark_tasks_completed`, legacy `task` vs `tasks` parsing).
- Sensor data path (`update_sensor_data` only mattered for the pre-redesign entry conditions during transitions; no longer relevant at runtime).
- Real Sphero, `colcon test` wiring, multi-threading.

---

## Verification

```
cd /home/svaghela/ros2_ws_2
pytest src/sphero_instance_controller/test/test_statemachine.py -v
```

All tests green. Optional `--cov=sphero_instance_controller.core.sphero.statemachine --cov-report=term-missing` to check coverage.

Smoke check (independent of tests): import `StateMachine`, configure with `LEAF_ONLY`, call `process()`, confirm no crash and `current_state` set.

---

## Deferred

- Migrating `src/sphero_statemachine/examples/*.json` to the new schema — separate task once the class lands.
- Tests for tasks API and ROS2 subscribe-callback wiring — separate task.
- Compound conditions (`any_of`, `all_of`) — only if needed during config migration.
