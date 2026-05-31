# Per-Task Start Offsets in Concurrent Task Bundles

**Created:** 2026-05-31T22:58:49Z
**Status:** Pending Approval

## Task Description
Allow each sub-task in a concurrent `tasks:[...]` bundle to optionally carry its
own `start_offset` (float seconds), ADDITIVE on top of the bundle's shared
`start_offset`. This staggers individual lanes after the fleet-synchronized
bundle start, while keeping the bundle-level offset as the fleet sync anchor.

## Analysis
Current `_handle_bundle()` in
`src/sphero_instance_controller/sphero_instance_controller/sphero_instance_task_controller_node.py`
computes ONE shared `start_at = _compute_start_at(now, start_offset)` and passes
it to every `_build_task(item, start_at)`. All lanes therefore start together.

The per-lane `start_at` gating in `core/common/task.py::process_tasks` already
supports independent per-lane start instants (verified by existing tests
`test_per_lane_start_at_gating_independence`,
`test_per_lane_fifo_same_lane_gated_blocks_follower`). So the only change needed
is at ingest: compute a per-item `start_at` instead of a single shared one.

`_build_task(item, start_at)` already accepts a `start_at` argument — no
signature change needed.

## Locked Contract
For each sub-task in a bundle:
```
item_start_offset   = float(item.get('start_offset', 0.0))
bundle_start_offset = float(task_data.get('start_offset', 0.0) or 0.0)
start_at = _compute_start_at(now, bundle_start_offset + item_start_offset)
```
- Sub-task with no `start_offset` (=0) starts at the bundle's synced instant.
- Sub-task with `start_offset: 2.0` starts 2s after that synced instant.
- `_compute_start_at` returns None when `now` is absent → per-task stagger only
  takes effect when the bundle carries `now` (the broadcast always sends it);
  otherwise all fall back to immediate exactly as today.

## Detailed Plan

### Step 1: Modify `_handle_bundle()`
- File: `src/sphero_instance_controller/sphero_instance_controller/sphero_instance_task_controller_node.py`
- Action: Replace the single shared `start_at` computation + the loop that
  passes it to every `_build_task` with per-item `start_at` computation. Read
  `bundle_start_offset` once (with `or 0.0` guard); inside the per-item loop
  compute `item_start_offset` and the per-item `start_at`, then call
  `_build_task(item, item_start_at)`.
- Update the closing log line: the bundle no longer has a single schedule time,
  so log per-task start delays / note that tasks are staggered.
- Unchanged: empty-list rejection, missing-`task_type` rejection, lane
  disjointness validation, per-task status publishing.

### Step 2: Add executor-level tests
- File: `src/sphero_instance_controller/test/test_task_executor.py`
- Action: Add a test in `TestConcurrentLanes` that builds a bundle-like scenario
  with differing per-task `start_at` values (DRIVE start_at=now, LED
  start_at=now+2) and asserts each lane gates independently: at t=now only DRIVE
  runs; at t=now+2 LED also runs. This validates the executor honors the
  per-task instants the node now produces. (Node-level `_handle_bundle` is not
  unit-tested directly — no node test file exists — so coverage lives at the
  executor level, matching existing per-lane gating tests.)

### Step 3: Verify
- `python3 -m py_compile` the changed node file.
- `python3 -m pytest test/test_task_executor.py -q` → must be 0 failed.
- `colcon build --packages-select sphero_instance_controller`.

## Expected Outcomes
- Bundles support per-task additive start offsets; lanes stagger after the
  synced fleet start.
- No change to single-task (shape a) or targeted-stop (shape c) paths.
- All existing tests stay green; new test passes.

## Potential Risks & Considerations
- `_compute_start_at` returns None for an offset whose target is already past;
  with `now` present and positive offsets this is fine. No change to that helper.
- The bundle-level offset still anchors all sub-tasks to the same `now`, so
  fleet sync is preserved.

## Testing Plan
- New executor test for independent per-lane gating with differing start_at.
- Full `test_task_executor.py` suite green.
- colcon build success (refreshes NFS/install).

## Approval Status
- [ ] Waiting for user approval
- [ ] Approved
- [ ] Executed
