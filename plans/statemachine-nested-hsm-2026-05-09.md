# StateMachine: nested state machines (HSM) — 2026-05-09

## Context

Today the StateMachine is strictly flat: `current_state` is a single `Optional[str]`, `states` is a flat dict, `process()` only ticks the current state's exits. Adding hierarchical state-machine support.

Locked-in design:
- **Composite states** — `sub_machine` config field on a state.
- **Path-based active state** — runtime is a path from root to leaf.
- **Parent-first exit priority** — outer exits checked before inner; first match wins.
- **No-history default** — when a parent's exit fires, every nested level on the path is reset; re-entering a composite starts at its `initial_state`. History deferred.
- **Composite tasks allowed** — a composite state may carry its own `tasks` (fire on parent entry); its sub-states have their own tasks too.

## Phase 1 — StateMachine class

**File:** `src/sphero_instance_controller/sphero_instance_controller/core/sphero/statemachine.py`

- `DynamicState`: gain `sub_machine: Optional[StateMachine] = None` (the live nested instance, built by the parent SM during `_build_state_machine`). Add `isComposite()` helper.
- `StateMachine` ctor: optional `topic_values` / `topic_last_received` / `depth` for nested instances; nested SMs share the root's topic dicts by reference.
- `_build_state_machine`: for each state with a `sub_machine` config block, construct a child `StateMachine`, wire shared callbacks + dicts, recursively `configure()` it.
- `process()` parent-first walk:
  1. Unconfigured → None.
  2. Paused → paused status (root paused freezes whole tree).
  3. Check current state's timeout.
  4. Try this level's exits → if a transition fires, reset old sub-machine, transition, enter new sub-machine, emit event with full `path`, **stop**.
  5. Else if current state has a sub-machine → recurse `child.process()`; prepend this state's name to child events' `path`.
- `_reset_active_state()` (new) — wipes runtime data + `current_state=None` + `paused=False` recursively. Doesn't unsubscribe.
- `_enter_initial()` (new) — sets `current_state = config['initial_state']`, stamps `entry_time`, recurses into the entered state's sub-machine.
- `pause()` unchanged. `resume()` walks the tree to shift every `state.entry_time` (and root's `topic_last_received` once) by paused duration.
- `clear()` walks the tree depth-first, unsubscribing each level's condition topics, then resets root.
- `get_status()` adds `path: [...]` (full active path) and `sub_status` (recursive child status or None).
- `validate_config()` recurses for `sub_machine` blocks.

## Phase 2 — Controller node

**File:** `src/sphero_instance_controller/sphero_instance_controller/sphero_instance_statemachine_controller_node.py`

- On `state_transition` event: identify the path delta vs. the previous active path; for every state in the **entered branch** (newly active states that weren't on the previous path), execute its tasks once, root-down. Cache the previous path in the node.

## Phase 3 — UI

- `templates/index.html`: add `[PATH]` row → `<span id="sm-path">--</span>`.
- `static/js/app.js`: render `smStatus.path.join(' · ')` with fallback to `smStatus.current_state`.

## Phase 4 — Tests

~17 new tests across:
- O. Composite config / validation
- P. Path-based runtime
- Q. Parent-first exit priority
- R. No-history reset
- S. Pause/resume with nesting
- T. Clear with nesting
- U. Transition event path shape

Target: 90 + 17 = 107 green.

## Critical files

- `src/sphero_instance_controller/sphero_instance_controller/core/sphero/statemachine.py`
- `src/sphero_instance_controller/sphero_instance_controller/sphero_instance_statemachine_controller_node.py`
- `src/sphero_instance_controller/sphero_instance_controller/static/js/app.js`
- `src/sphero_instance_controller/sphero_instance_controller/templates/index.html`
- `src/sphero_instance_controller/test/test_statemachine.py`

## Verification

1. `pytest src/sphero_instance_controller/test/test_statemachine.py -v` → ~107 green.
2. `colcon build --packages-select sphero_instance_controller` → clean.
3. Live test: deploy a `patrol{N/E/S/W}` composite with a root `topic_message /halt` exit. Watch `[PATH]` cycle, then publish halt → exits to idle, then republish config and confirm patrol re-enters at N (no history).
