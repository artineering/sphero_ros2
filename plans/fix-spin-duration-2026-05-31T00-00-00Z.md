# Fix execute_spin to honor explicit `duration`

**Created:** 2026-05-31T00:00:00Z
**Status:** Pending Approval

## Task Description
Fix a bug in `execute_spin` (sphero_task_handlers.py ~line 265). A task with
`{"task_type":"spin","parameters":{"duration":60,"speed":120}}` only spins ~4s
because `execute_spin` ignores the `duration` param. It reads `rotations`
(default 1), computes `duration = (360*rotations)/90` (= 4s), and gates on that.
The explicit `duration` does nothing.

## Analysis
Mechanics verified by reading the call chain:
- `execute_spin` -> `executor._send_spin_command(angle, duration)`
  (topic_task_executor.py:84) publishes `{'angle', 'duration'}` on `.../spin`.
- Device controller `spin_callback` (sphero_instance_device_controller_node.py:254)
  reads angle/duration -> `self.sphero.spin(angle, duration)`.
- `core/sphero/sphero.py::spin` (line 180) calls `api.spin(angle, duration)`,
  which in spherov2 rotates `angle` degrees over `duration` seconds.

Therefore the continuous-spin requirement is governed by the ratio
`angle / duration` (degrees per second). The old code's `angle = 360*rotations`
spread over a 60s `duration` would crawl. To keep a real spin for the full
requested time we must scale `angle` with `duration` using a fixed angular rate.

Sibling handlers `execute_roll` (line 359) and `execute_circle` (line 147)
already treat `duration` as: if `duration > 0`, run for that many seconds and
gate completion on `elapsed >= duration`. We will mirror that.

## Detailed Plan

### Step 1: Rewrite `execute_spin`
- File: `src/sphero_instance_controller/sphero_instance_controller/core/sphero/sphero_task_handlers.py` (~265)
- Logic:
  - Read `duration = task.parameters.get('duration', 0.0)`,
    `rotations = task.parameters.get('rotations', 1)`,
    `speed = task.parameters.get('speed', 100)` (speed kept for back-compat /
    forward semantics; not used in angle math — documented).
  - Introduce module-doc constant rate `SPIN_DEG_PER_SEC = 360.0`
    (one rotation per second) as the default angular rate, documented inline.
  - On first tick (`'start_time' not in task.parameters`):
    - record `start_time`, `start_heading`.
    - If `duration > 0`: `angle = int(SPIN_DEG_PER_SEC * duration)`;
      `effective_duration = duration`.
    - Else (back-compat): `angle = int(360 * rotations)`;
      `effective_duration = (360 * rotations) / 90`.
    - `executor._send_spin_command(angle, effective_duration)`.
    - store `task.parameters['rotation_time'] = effective_duration`.
    - return False.
  - On later ticks: return
    `time.time() - start_time >= task.parameters['rotation_time']`.
- The gate uses `rotation_time`, which now equals the explicit `duration` when
  given, else the derived value — so completion matches the actual intended
  duration.

### Step 2: Update the existing `test_spin` + add a duration test
- File: `src/sphero_instance_controller/test/test_task_executor.py` (~511)
- Keep existing `test_spin` (rotations=2 path) — still valid (angle 720, dur 8.0).
- Add `test_spin_with_duration`: task `spin` with `duration=60, speed=120`;
  assert a `spin` send with `angle=21600, duration=60.0`; assert task still
  running before 60s, completes after `clock.advance(60.5)`.

### Step 3: Verify
- `python3 -m py_compile sphero_task_handlers.py`
- `python3 -m pytest test/test_task_executor.py -q`
- `colcon build --packages-select sphero_instance_controller`

## Expected Outcomes
- `spin` with explicit `duration` spins continuously for that many seconds.
- `spin` without `duration` keeps the legacy `rotations` behavior.

## Final Parameter Contract for `spin` (for web UI fix)
- `duration` (float seconds, optional): if > 0, spin lasts exactly this long;
  angle derived as `360 * duration` (1 rotation/sec). Takes precedence.
- `rotations` (int, default 1): used ONLY when `duration` not given/<=0;
  angle = `360 * rotations`, internal time = `(360*rotations)/90` = 4s/rotation.
- `speed` (int, default 100): accepted, not used in angle math (documented).

Web UI note (`static/js/app.js:17`): default is `{ speed: 120, duration: 3 }`.
With the fix this now spins ~3s as intended (previously ignored, spun ~4s via
rotations default). No web change strictly required, but template is now honored.

## Potential Risks & Considerations
- Changing `_send_spin_command`'s `duration` arg to a large value with a large
  angle is the intended behavior for spherov2 `api.spin`; no API change.
- `SPIN_DEG_PER_SEC = 360` is a chosen sane constant; if a different visual spin
  speed is desired later, it is a single documented knob.

## Testing Plan
- Unit: py_compile + pytest (new + existing spin tests).
- Build: colcon build of the single package.
- Manual (optional, on hardware): publish a spin task with duration 60 and
  confirm it spins for ~60s.

## Approval Status
- [ ] Waiting for user approval
- [ ] Approved
- [ ] Executed
