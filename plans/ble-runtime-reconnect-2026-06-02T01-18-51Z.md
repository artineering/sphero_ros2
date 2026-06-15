# Runtime BLE Reconnection + Terminal Error Reporting for Device Controller

**Created:** 2026-06-02T01:18:51Z
**Status:** Pending Approval

## Task Description
Add runtime BLE failure handling to the Sphero device controller node. If the
live BLE link drops while the controller is running, it must: detect the loss,
attempt to reconnect a fixed number of times (configurable, default 3), and —
if all reconnects fail — publish a `device_error` to the webserver and exit the
process cleanly (no infinite loop). If the Sphero comes back during the retry
window, it must reconnect and resume. The existing initial-connect retry,
per-host BLE lock, and all current behavior stay intact.

File: `src/sphero_instance_controller/sphero_instance_controller/sphero_instance_device_controller_node.py`

## Analysis (verified against the code + spherov2 on worker 10.0.0.11)

### How BLE failures surface
- `spherov2` `Toy._execute` (`~/.local/lib/python3.12/site-packages/spherov2/toy/__init__.py:85`)
  raises `RuntimeError('Use toys in context manager')` when the adapter is None
  (link gone / context exited).
- Live BLE drops surface as `bleak` exceptions from `BleakAdapter.write` /
  `write_gatt_char` (e.g. `BleakError`, dbus `EOFError`, disconnection errors),
  and `_wait_packet` can raise `concurrent.futures.TimeoutError` (10s) when the
  device stops responding.
- spherov2 exposes NO disconnect signal/callback we can subscribe to.
  `BleakAdapter` has no on_disconnect hook wired through. => failure-counting it is.

### Critical finding: sensors swallow errors
`SpheroState.update_from_device()` (`core/sphero/state.py:610+`) wraps EVERY
sensor read in `except Exception: pass`. So `publish_sensors` -> `update_sensors`
will NOT surface a dead link; sensors silently go stale. Therefore I cannot rely
on the sensor path for detection. I need a DEDICATED liveness probe that lets the
exception propagate.

### Detection design (robust, no false positives on transients)
Add a periodic liveness probe timer in the node (separate from the sensor timer).
Each tick it calls one lightweight, real round-trip Sphero API call
(`self.sphero.api.get_main_led()` — a getter that goes through `_execute` and
thus fails on a dead link) inside a try/except.
- On success: reset `self._ble_fail_streak = 0`.
- On a "link-dead" exception: increment `self._ble_fail_streak`.
- A SINGLE failure does NOT trigger reconnect. Only when the streak reaches a
  threshold (`liveness_fail_threshold`, default 3 consecutive probe failures)
  do we declare the link down. With a 1.0s probe period that's ~3s of sustained
  failure — well past a transient BleakError or one packet collision.
- "link-dead" is identified by message/type matching against the known
  signatures (`Use toys in context manager`, `BleakError`, `disconnect`,
  `EOFError`, `TimeoutError`, `not connected`). A `PacketDecodingException`
  (transient packet collision) is treated as NON-fatal (does not increment).

The probe and the failure flag are read by the spin loop in `main()`, which owns
the reconnect lifecycle (it already owns the context manager `cm` and the lock).

### Reconnect lifecycle (lives in main(), where cm + lock already live)
When the node signals "link down" (`node.ble_link_down` True):
1. Cleanly `cm.__exit__(None,None,None)` the dead `SpheroEduAPI`.
2. For attempt in 1..N (`SPHERO_BLE_RECONNECT_ATTEMPTS`, default 3):
   - sleep `backoff` (`SPHERO_BLE_RECONNECT_BACKOFF`, default 2.5s) — also gives
     the unit time to come back if it was power-cycled.
   - re-acquire `ble_connect_lock(label=...)` and re-run `scanner.find_toy` +
     `SpheroEduAPI(toy=robot)` + `__enter__` (SAME serialized path as initial
     connect).
   - on success: rebind the live api/robot into the running node via a new
     `node.rebind_connection(robot, api)` method (updates `node.sphero.api`,
     `node.sphero.robot`, `node.sphero.state` api/toy refs), clear the
     down/streak flags, and resume the spin loop. Log the success.
   - on failure: log attempt, close any half-open cm, loop.
3. If all N fail: call `node.publish_ble_lost(last_error)` which publishes a
   `device_error` with `error: 'ble_lost'` (reuse existing topic + format,
   published ~10x with spin_once for delivery), then break out of the spin loop
   and fall through to the existing `finally` cleanup -> process exits cleanly.
   The webserver's existing `poll()` check (`sphero_instance_websocket_server.py:355`)
   detects the exited process; the `device_error` topic also reaches its
   `device_error_callback` (line 303).

### Webserver relay (verified)
`device_error_callback` (`sphero_instance_websocket_server.py:303-320`) currently
ONLY emits to socketio + triggers fleet shutdown for `error == 'toy_not_found'`.
For `ble_lost` it will hit the callback but the `if error_type == 'toy_not_found'`
branch won't fire, so it will NOT emit `device_error` to dashboard clients for the
new code. HOWEVER, the process exit IS still detected by the `poll()` path which
triggers `_shutdown_after_delay`. So: the controller dies cleanly and the
webserver reaps it. To ALSO surface `ble_lost` in the dashboard UI (socketio emit)
is a 2-line change in that callback — but per the scope this is a WEBSERVER file,
not the device controller. I will NOT modify the webserver in this change; I will
report that a small web_expert follow-up is needed to emit `ble_lost` to the
dashboard UI (the controller-side reporting + clean exit is complete without it).

## Detailed Plan

### Step 1: Add env-configurable constants + node state
- File: `sphero_instance_device_controller_node.py`
- Add module-level reads of `SPHERO_BLE_RECONNECT_ATTEMPTS` (default 3),
  `SPHERO_BLE_RECONNECT_BACKOFF` (default 2.5), `SPHERO_BLE_LIVENESS_PERIOD`
  (default 1.0), `SPHERO_BLE_LIVENESS_FAIL_THRESHOLD` (default 3).
- In `__init__`: init `self._ble_fail_streak = 0`, `self.ble_link_down = False`,
  and create a liveness timer `self.create_timer(liveness_period, self._ble_liveness_probe)`.

### Step 2: Liveness probe + link-dead classification
- Add `_LINK_DEAD_MARKERS` helper / `_is_link_dead_error(exc)` classifier.
- Add `_ble_liveness_probe(self)`: if `self.ble_link_down` already set, no-op
  (main() is handling it). Else call `self.sphero.api.get_main_led()`; on success
  reset streak; on link-dead exc increment streak and, at threshold, set
  `self.ble_link_down = True` + log a warning. Transient/non-link errors logged
  + streak reset (treat as alive).

### Step 3: rebind + publish helpers on the node
- Add `rebind_connection(self, robot, api)`: update `self.sphero.robot`,
  `self.sphero.api`, `self.sphero.state.set_api(api)`, `self.sphero.state.set_toy(robot)`,
  reset streak + clear `ble_link_down`. (Command handlers read `self.sphero.api`
  live, so this is enough.)
- Add `publish_ble_lost(self, last_error)`: create/reuse a `device_error`
  publisher on `self.topic_prefix/device_error`, publish `{'error':'ble_lost',
  'sphero_name':..., 'message':str(last_error)}` ~10x with `spin_once`.

### Step 4: Reconnect loop in main()
- Refactor the spin loop: wrap the existing
  `while rclpy.ok() and not shutdown_requested: spin_once` so that when
  `node.ble_link_down` becomes True it breaks to a reconnect routine.
- Implement reconnect routine (exit dead cm, N attempts under lock, rebind on
  success, publish + exit on exhaustion). Keep the outer `finally: cm.__exit__`.
- Use a small helper `_reconnect(node, sphero_name)` returning (new_cm, robot, api)
  or None. Keep it inline in main() to avoid over-abstraction.

### Step 5: Build + tests
- `colcon build --packages-select sphero_instance_controller`
- `python3 -m pytest test/test_task_executor.py -q` (must stay 0 failed)

### Step 6: Hardware verification on worker 10.0.0.11
- Spawn one powered unit via agent (`POST :8181/spawn`), confirm connect + a
  command (`POST :5001/api/led green`).
- Power off the unit mid-run; tail `/var/log/sphero_worker_agent/<NAME>.log`;
  confirm: detection -> N reconnect attempts logged -> after N failures
  `device_error ble_lost` published -> process exits cleanly (no infinite loop).
- Repeat: power unit back ON during the retry window; confirm reconnect + a
  command works again.
- `DELETE /spawn/<NAME>` teardown. No persistent changes to workers.

## Expected Outcomes
- Runtime BLE drop is detected within ~3s (3 failed probes) without firing on a
  single transient.
- Up to N reconnects attempted under the existing per-host lock; success rebinds
  the live connection and resumes; commands work again.
- Exhaustion publishes `ble_lost` device_error and exits the process cleanly.
- All existing behavior (initial connect retry, lock, sensors, commands) intact.
- pytest stays green; colcon build succeeds.

## Potential Risks & Considerations
- The liveness probe adds one extra BLE round-trip/sec. `get_main_led()` is cheap;
  period is configurable. Risk: probe itself could collide with a command — but it
  shares the same serialized spherov2 send path, and a single failure won't trip
  the threshold.
- `get_main_led` availability: it's a standard SpheroEduAPI getter used already in
  `state.update_from_device` (`_get_led` calls it). Safe across Sphero/BOLT.
- Rebinding mid-flight: command callbacks read `self.sphero.api` each call, so a
  swap between calls is safe. No long-held api references elsewhere.
- Worker logs: confirm exact log path before tailing (agent may name it per unit).

## Testing Plan
- Unit: pytest test_task_executor.py (regression guard).
- Build: colcon build.
- Hardware: full drop / reconnect / exhaust+exit matrix on 10.0.0.11 as above.

## Approval Status
- [ ] Waiting for user approval
- [ ] Approved
- [ ] Executed
