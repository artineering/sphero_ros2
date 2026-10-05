# BLE link-state status reporting + matrix wipe fix

**Created:** 2026-10-05T00:00:00
**Status:** Executed; live-tested on the fleet 2026-10-05 (SB-33E3 on rpi4-node01)

## Task Description
1. Dashboard learns of a dead BLE link ~17 s late; target <= 3 s from power-off.
2. Issue #3: a graphic drawn on the BOLT matrix disappears after ~1 s.

## Analysis
### Problem 2 root cause
`_ble_liveness_probe` (device controller node) re-applies the cached main LED every
1 s via `api.set_main_led`. On a BOLT that is `IO.set_compressed_frame_player_one_color`,
which floods the whole 8x8 matrix with one color (usually black). Any graphic is wiped
within <= 1 s. Task completion never clears the matrix (`execute_matrix` is a modifier;
completion does not call `_stop_lane`).

### Problem 1 root cause
- `publish_heartbeat` hard-codes `connection_state: 'connected'`.
- `publish_sensors` keeps publishing cached values while the link is dead.
- `main()` stops spinning during the reconnect loop (dead `__exit__`, 2.5 s backoff,
  connect lock, 5 s scan, connect) so nothing is published for ~16 s.
- Webserver marks a robot running if any message arrived in the last 15 s.

## Interface contract (confirmed)
`/sphero/<name>/status` (std_msgs/String JSON, heartbeat shape), `connection_state` in
{connected, reconnecting, disconnected}:
- "connected" once at startup, on each heartbeat, and after a successful rebind.
- "reconnecting" immediately when the link is declared down, repeated ~1 s while reconnecting.
- "disconnected" when reconnects are exhausted (alongside `ble_lost`) and on a deliberate
  shutdown (SIGINT/SIGTERM). Terminal: the process exits after it.
- battery / is_healthy are only meaningful when connected (cached otherwise).
- No /sensors or /state while the link is down. No seq field.

## Detailed Plan
All paths under `src/sphero_instance_controller/sphero_instance_controller/`.

### Step 1: `core/sphero/sphero.py` - `Sphero.check_link()`
One response-bearing, side-effect-free round trip (`Power.get_battery_voltage(self.robot)`);
re-raises on failure.

### Step 2: probe uses `check_link()`
Replace `get_main_led`/`set_main_led` in `_ble_liveness_probe`. Fixes Problem 2.

### Step 3: faster detection
Defensive `ToyDisconnectedError` import; a single `ToyDisconnectedError` trips the
probe immediately. Other dead-link errors keep the 3-strike rule.

### Step 4: status state
`connection_state` attribute, `_status_payload()` helper, `publish_connection_state()`;
heartbeat uses the real state and is skipped while down; "connected" published at startup.

### Step 5: disconnect listener (spherov2 0.13.0)
Registered via `getattr(robot, 'add_disconnect_listener', None)`; callback only sets a
`threading.Event`. `detach_disconnect_listener()` before tearing down the dead link.

### Step 6: `mark_link_down(reason)` + reconnect beacon
Sets `ble_link_down`, publishes "reconnecting", starts a daemon thread publishing
"reconnecting" every 1 s until stopped.

### Step 7: `publish_sensors` guard
Return early if `ble_link_down` or `robot.is_connected` is False.

### Step 8-10: main() loop, rebind, exhaustion
Main loop promotes the listener event to `mark_link_down`; rebind stops the beacon and
publishes "connected"; exhaustion stops the beacon and publishes "disconnected" + ble_lost.

### Step 11: `_reconnect(node, sphero_name)` extraction (approved)
Reconnect block moved to a module function returning the new context manager or None.

### Step 12: deliberate shutdown
Handle SIGTERM like SIGINT; publish "disconnected" before tearing down the link.

### Step 13: clear before draw in `Sphero.set_matrix` (approved later)
After the pattern is validated, call `api.clear_matrix()` immediately before the
pixel writes (no sleep) so stale pixels from the previous graphic are gone. No
separate single-pixel API is exposed, so nothing else clears.

## Expected Outcomes
- "reconnecting" on /status within ~0.1-1 s of spherov2 seeing the drop.
- BOLT matrix graphics persist.

## Potential Risks & Considerations
- Power-off -> library detection depends on the BLE supervision timeout.
- Beacon thread must be stopped on every exit path.
- Node tests need a built + sourced workspace (generated msgs).

## Testing Plan
- New `test/test_ble_liveness.py` (probe classification, sensor guard, rebind, listener,
  beacon, shutdown/exhaustion state, probe never touches LEDs/matrix).
- New `test/test_sphero_matrix.py`: graphic B after A leaves only B's pixels; clear precedes pixel writes; invalid input does not blank.
- Existing tests in `src/sphero_instance_controller/test` still pass.
- Live (user): power off SB-33E3, time /status transitions from power-off; matrix graphic persists 30 s+.

## Approval Status
- [x] Waiting for user approval
- [x] Approved
- [x] Executed (2026-10-05; not committed)
