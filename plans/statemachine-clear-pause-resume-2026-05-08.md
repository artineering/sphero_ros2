# StateMachine: clear / pause / resume — 2026-05-08

## Context

The StateMachine has no runtime control today. Once a config is loaded the only way to stop or change behaviour is to publish a brand-new config — there's no pause, no resume, no clean way to drop a running state machine. The user wants three new control verbs:

- **Pause** — stop ticking but keep the config; in-flight timers freeze (pause time doesn't count toward elapsed).
- **Resume** — start ticking again; timers and `topic_message` age windows pick up where they left off.
- **Clear** — full reset to unconfigured: drop all states/exits, unsubscribe from any condition topics, clear topic-value caches.

User decisions:
- **Pause time doesn't count** — on resume, every `state.entry_time` and every `topic_last_received` entry is shifted forward by the paused duration so elapsed-time math stays correct.
- **Clear = full reset** — symmetric with pre-configure state; loading a new config required to do anything.

---

## Phase 1 — StateMachine class

**File:** `src/sphero_instance_controller/sphero_instance_controller/core/sphero/statemachine.py`

- New state: `self.paused: bool = False`, `self._pause_started_at: Optional[float] = None`.
- `pause() -> bool` — False if no `current_state` or already paused; otherwise sets `paused=True`, stamps `_pause_started_at`, logs.
- `resume() -> bool` — False if not paused; computes `paused_duration`; shifts every non-None `state.entry_time` and every `topic_last_received[topic]` forward by that duration; clears flag.
- `clear() -> None` — calls `topic_unsubscribe_callback` for every key in `topic_values`; resets `config=None, states={}, current_state=None, paused=False, _pause_started_at=None`; clears the three caches; logs.
- `process()` — after the unconfigured guard, if `self.paused`, return a status dict with `events=[]` and `paused=True`; skip transition + timeout checks.
- `get_status()` — add `'paused'` to both branches.

## Phase 2 — Controller node

**File:** `src/sphero_instance_controller/sphero_instance_controller/sphero_instance_statemachine_controller_node.py`

- Subscribe to `sphero/<name>/state_machine/control` (std_msgs/String, JSON `{"action": "pause"|"resume"|"clear"}`).
- `control_callback(msg)`: parse, dispatch to `state_machine.pause/resume/clear`, publish `sm_paused`/`sm_resumed`/`sm_cleared` event with `{success: bool}`.
- Update `_log_initialization` to list the new subscribed topic.

## Phase 3 — Websocket server (HTTP routes)

**File:** `src/sphero_instance_controller/sphero_instance_controller/sphero_instance_websocket_server.py`

- Constructor: add `self.sm_control_pub = self.create_publisher(String, f'{self.topic_prefix}/state_machine/control', 10)`.
- New method: `publish_sm_control(action: str)` — JSON-encodes `{"action": action}`, publishes.
- Three Flask routes mirroring `/api/state_machine/config`'s shape:
  - `POST /api/state_machine/pause`
  - `POST /api/state_machine/resume`
  - `POST /api/state_machine/clear`

  Each calls `publish_sm_control(...)` and returns `{success: True, message: ...}`.

## Phase 4 — Web UI

**Files:** `src/sphero_instance_controller/sphero_instance_controller/static/js/app.js`, `src/sphero_instance_controller/sphero_instance_controller/templates/index.html`.

### app.js
- `pauseStateMachine()`, `resumeStateMachine()`, `clearStateMachine()` — POST to the new endpoints; surface success/error via `showStateMachineMessage`.
- `updateStateMachineStatusDisplay()` — render `paused` (Yes/No) into `#sm-paused`; flip the pause/resume toggle button label and onclick based on `smStatus.paused`.

### index.html
- Rename the existing "Clear" button (which clears the editor textarea via `clearStateMachineConfig()`) → **"Clear Editor"**.
- Add Pause/Resume toggle button (`id="sm-pause-resume-btn"`, default label "Pause", onclick `pauseStateMachine()`) and "Clear State Machine" button (`onclick="clearStateMachine()"`).
- Add status row: `<p><strong>Paused:</strong> <span id="sm-paused">--</span></p>`.

## Phase 5 — Tests

**File:** `src/sphero_instance_controller/test/test_statemachine.py` (extends the existing 73-test suite).

New groups:
- **L. Pause / resume** — flag transitions, error returns for invalid states, `process()` no-op while paused, timer exit fires correctly only after resume, `entry_time` and `topic_last_received` shifted forward by paused duration.
- **M. Clear** — resets to unconfigured, unsubscribe callback called per topic, caches cleared, no-op when unconfigured, can configure again post-clear.
- **N. `get_status`** — `paused` field present in both branches.

Target: ~14 new tests → ~87 total.

---

## Out of scope

- SocketIO event parity for pause/resume/clear — HTTP-only this pass.
- Multirobot dashboard surfacing pause/clear at the fleet level — separate task.
- Persisting paused state across controller restarts — in-memory only.

## Critical files

- `src/sphero_instance_controller/sphero_instance_controller/core/sphero/statemachine.py`
- `src/sphero_instance_controller/sphero_instance_controller/sphero_instance_statemachine_controller_node.py`
- `src/sphero_instance_controller/sphero_instance_controller/sphero_instance_websocket_server.py`
- `src/sphero_instance_controller/sphero_instance_controller/static/js/app.js`
- `src/sphero_instance_controller/sphero_instance_controller/templates/index.html`
- `src/sphero_instance_controller/test/test_statemachine.py`

## Verification

1. `pytest src/sphero_instance_controller/test/test_statemachine.py -v` — expect ~87 green.
2. `colcon build --packages-select sphero_instance_controller` — clean build.
3. **Live click-test:** start full chain, load template, click Send (LED blue), wait 1.5s, Pause (LED stays blue past 3s), wait several seconds, Resume (LED goes green ~1.5s later — 3s total active), Clear (status panel returns to unconfigured shape).
4. **ROS-only spot check:** `ros2 topic pub --once /sphero/SB_3660/state_machine/control std_msgs/msg/String "data: '{\"action\":\"pause\"}'"` then echo status, confirm `paused: true`.
