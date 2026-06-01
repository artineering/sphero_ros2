# Serialize BLE Scan+Connect Phase Per-Host (Fix Concurrency Bug)

**Created:** 2026-06-01T20-18-26Z
**Status:** Pending Approval

## Task Description
Fix a confirmed BLE concurrency bug in the Sphero device-controller. When a Pi runs
4 Sphero device-controller processes, all 4 call `scanner.find_toy()` +
`SpheroEduAPI(...).__enter__()` (the BLE connect) at nearly the same instant. BlueZ
rejects parallel connects on one adapter (`org.bluez.Error.InProgress`), so only ~2/4
connect; losers fail with `BleakDeviceNotFoundError` (scan starved) or
"failed to discover services, device disconnected".

Root cause is empirically confirmed: fully serialized one-at-a-time connect + retry =
4/4 stable. Fix is to serialize the scan+connect phase per host using a file lock.

File: `src/sphero_instance_controller/sphero_instance_controller/sphero_instance_device_controller_node.py`,
function `main()` (scan+connect region, ~lines 575-622).

## Analysis
Current code (verified):
- `temp_node` reads `sphero_name`, builds `topic_prefix`.
- `scanner.find_toy(toy_name=sphero_name)` in a try/except that publishes a
  `device_error` ('toy_not_found') and exits cleanly on failure.
- `temp_node.destroy_node()` then `with SpheroEduAPI(toy=robot) as api:` opens the
  connection for the whole node lifetime (spin loop inside the `with`).
- `finally` block cleans up `temp_node`, `node`, and `rclpy.shutdown()`.

Constraints:
- The lock must cover `find_toy` (scan contends too) THROUGH the connect/`__enter__`,
  then release immediately — NOT held during the spin loop.
- Because the connection must outlive the lock, we must enter the `SpheroEduAPI`
  context MANUALLY: `cm = SpheroEduAPI(toy=robot); api = cm.__enter__()` under the
  lock, release lock, run spin loop, and guarantee `cm.__exit__(None,None,None)` on
  shutdown/exception via try/finally.
- `device_error` publish needs `temp_node`, so we cannot destroy `temp_node` until
  after a successful connect (it is used to publish errors if all retries fail).

## Detailed Plan

### Step 1: Add imports
- Action: Add `import fcntl` and `import os` to the import block (top of file).
- Files: the node file.
- Expected outcome: file still compiles.

### Step 2: Add a per-host lock helper
- Action: Add a small module-level context manager / helper that:
  - Resolves lock path from env `SPHERO_BLE_CONNECT_LOCK`, default
    `/tmp/sphero_ble_connect.lock` (local to each Pi, NOT under NFS workspace).
  - Opens/creates the lockfile (`open(path, 'w')`).
  - Acquires an exclusive lock with a BOUNDED blocking wait: loop calling
    `fcntl.flock(fd, LOCK_EX | LOCK_NB)`; on `BlockingIOError` sleep ~0.25s and
    retry until ~90s elapse, then proceed anyway (attempt rather than hang forever).
  - Logs "waiting on" / "acquired" lines.
  - Releases (`fcntl.flock(fd, LOCK_UN)`) and closes fd on exit.
- Implement as a contextmanager so the critical section is obvious and release is
  guaranteed even on exception inside.
- Expected outcome: reusable, short critical section.

### Step 3: Replace the scan+connect region in main()
- Action: Replace current lines ~579-622 (the scan try/except + `with SpheroEduAPI`)
  with a retry loop (up to 3 attempts, ~2.5s backoff):
  - For each attempt:
    1. Acquire the per-host lock (Step 2 helper).
    2. `robot = scanner.find_toy(toy_name=sphero_name)`.
    3. `cm = SpheroEduAPI(toy=robot); api = cm.__enter__()` (the connect).
    4. On success: release lock, break out of retry loop with `cm`/`api`/`robot` set.
    5. On failure: if `cm` was created, call `cm.__exit__(None,None,None)` in a
       guarded try (close any half-open connection); release lock (handled by the
       contextmanager); log retry; `time.sleep(2.5)`; continue.
  - The lock is released by the contextmanager `__exit__` at the end of each attempt
    body, AFTER the successful connect (we set a flag / structure so release happens
    once connect succeeds but before the spin loop).
  - After the loop: if not connected, publish the existing `device_error` using
    `temp_node` (reuse 'toy_not_found' for find_toy failures, add 'connect_failed'
    for connect failures), then clean exit (mirror existing error path:
    publish x10, destroy temp_node, rclpy.shutdown(), return).
  - On success: destroy `temp_node`, set `temp_node = None`, then run the spin loop
    OUTSIDE the lock:
      `node = SpheroInstanceDeviceController(robot, api, sphero_name)`
      `while rclpy.ok() and not shutdown_requested: rclpy.spin_once(node, 0.1)`
- Lifetime: wrap the post-connect spin in try/finally that calls
  `cm.__exit__(None,None,None)` so the connection always closes on shutdown/exception.
  (Existing outer finally already handles node cleanup + rclpy.shutdown.)
- Files: the node file.
- Expected outcome: only one process scans+connects at a time; retries recover
  transient BlueZ failures; connection stays open for node lifetime.

### Step 4: Verify
- `python3 -m py_compile` the file.
- `colcon build --packages-select sphero_instance_controller`.
- Smoke import check if easy (import module — may require sourced install / deps).

## Expected Outcomes
- Scan+connect serialized per host via `/tmp/sphero_ble_connect.lock` (env override).
- Bounded blocking lock acquire (~90s) so no permanent deadlock.
- Up to 3 attempts, ~2.5s backoff, half-open connections closed between attempts.
- Lock released immediately after successful connect; NOT held during spin loop.
- Connection reliably closed on shutdown via manual `cm.__exit__`.
- Existing error-publish + clean-shutdown behavior preserved; new 'connect_failed'
  error code added for connect-phase failures.

## Potential Risks & Considerations
- `temp_node` must survive until after connect succeeds (needed for error publish).
  Current code destroys it before connect — we move the destroy to after success.
- Must not double-release / double-exit the context manager; guard with flags.
- `/tmp` is local per Pi (correct); must NOT use the NFS workspace path.
- spherov2 import at module load already happens; build does not exercise hardware.
- No hardware unit test exists; will not fabricate one.

## Testing Plan
- `python3 -m py_compile <file>` passes.
- `colcon build --packages-select sphero_instance_controller` succeeds (updates
  NFS/install so workers pick up the change).
- Optional: import smoke check.
- Real hardware validation (4 Spheros on one Pi → 4/4 connect) is operator-run; not
  performed here.

## Approval Status
- [ ] Waiting for user approval
- [ ] Approved
- [ ] Executed
