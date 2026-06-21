# Re-add BOLT Robot-to-Robot IR Methods (Secondary Processor Fix)

**Created:** 2026-06-15T00:00:00
**Status:** Pending Approval

## Task Description
Re-add six robot-to-robot IR communication methods to the `Sphero` core class. Unlike the high-level `SpheroEduAPI` IR methods (which silently fail on BOLT because spherov2 binds them to the PRIMARY processor while BOLT's IR hardware lives on the SECONDARY processor), these methods call the low-level `spherov2.commands.sensor.Sensor` commands directly with `proc=Processors.SECONDARY`. This was empirically verified on real hardware.

## Analysis
- Target file: `src/sphero_instance_controller/sphero_instance_controller/core/sphero/sphero.py` (665 lines).
- `Sensor` is already imported (line 17). `Processors` is NOT imported yet.
- Verified library signatures in `/usr/local/lib/python3.12/dist-packages/spherov2/commands/sensor.py`:
  - `start_robot_to_robot_infrared_broadcasting(toy, far_code, near_code, proc=None)` (line 172)
  - `start_robot_to_robot_infrared_following(toy, far_code, near_code, proc=None)` (line 176)
  - `start_robot_to_robot_infrared_evading(toy, far_code, near_code, proc=None)` (line 202)
  - `stop_robot_to_robot_infrared_broadcasting(toy, proc=None)` (line 180)
  - `stop_robot_to_robot_infrared_following(toy, proc=None)` (line 198)
  - `stop_robot_to_robot_infrared_evading(toy, proc=None)` (line 206)
  - Confirmed arg order is **far_code first, then near_code**.
- Verified `Processors` is in `/usr/local/lib/python3.12/dist-packages/spherov2/controls/v2.py` (line 337) with `SECONDARY = 2`.
- Connected toy held as `self.robot` (line 49). Existing methods that call low-level commands already use `self.robot` (e.g. `set_raw_motors`, collision config).
- No existing IR methods in the file — clean add.
- Existing method style (from `roll`, `set_led`): docstring with Args/Returns, `try/except Exception as e` with `print(f"...: {e}")`, return True on success / False on failure.

## Detailed Plan

### Step 1: Add the Processors import
- Action: Add `from spherov2.controls.v2 import Processors` near the existing spherov2 imports (after line 17).
- Files: `.../core/sphero/sphero.py`
- Expected outcome: `Processors.SECONDARY` is available.

### Step 2: Add the six IR methods
- Action: Insert a new `# ===== Robot-to-Robot IR =====` section. Insertion point: after `_on_collision_detected` (line 555), before `# ===== Sensor Data =====` (line 557).
- The three `start_*` methods take `(self, near, far)`, clamp both to 0..7 via `max(0, min(7, int(x)))`, then call the matching `Sensor.start_*` with args `(self.robot, far, near, proc=Processors.SECONDARY)` — far first, near second.
- The three `stop_*` methods call the matching `Sensor.stop_*` with `(self.robot, proc=Processors.SECONDARY)`.
- Each method: docstring, try/except with `print(f"...: {e}")`, return True/False. No extra logic beyond the clamp.
- Files: `.../core/sphero/sphero.py`
- Expected outcome: six new methods present.

### Step 3: Build
- Commands: `colcon build --packages-select sphero_instance_controller`
- Expected outcome: clean build.

## Expected Outcomes
- `start_ir_broadcast`, `stop_ir_broadcast`, `start_ir_follow`, `stop_ir_follow`, `start_ir_evade`, `stop_ir_evade` added to the `Sphero` class.
- All route through `Sensor` directly on the SECONDARY processor, bypassing `self.api`.
- Package builds successfully.

## Potential Risks & Considerations
- Argument-order inversion (near/far) is the highest-risk detail; mapping is explicit per spec.
- Bypassing the high-level API means no `bound_value` clamp — handled by manual 0..7 clamp.
- These are only the wrapper methods on the core class; wiring into nodes/tasks/topics is out of scope.

## Testing Plan
- Build with colcon (Step 3).
- User live-tests IR follow/broadcast/evade on the real fleet (per Test-before-commit memory). Do NOT commit.

## Approval Status
- [ ] Waiting for user approval
- [ ] Approved
- [ ] Executed
