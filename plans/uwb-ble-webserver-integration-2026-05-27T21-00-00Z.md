# Integrate sphero_uwb_positioning BLE node into multirobot_webserver

**Created:** 2026-05-27T21:00:00Z
**Status:** Approved + Executed

## Task Description
Wire the `sphero_uwb_positioning` BLE node into the `multirobot_webserver`
multi-robot workflow. The BLE node should publish per-tag-id positions
(decoupled from Sphero names), the webserver should own the tag pool and the
live `tag_id -> sphero` mapping, UWB should replace odometry as the source of
`FleetRobot.pose`, and the webserver should start the BLE node as a child
process (Foxglove-style) only after anchor positions are configured. A
combined launch file brings up the webserver. An explicit web-facing REST
contract is defined so a separate `web_expert` agent can build the UI.

## Analysis

### Current state (verified by reading code)
- **BLE node** (`ble_position_node.py`): builds a `tag_to_sphero` dict from
  paired `tag_ids` + `sphero_names` startup params, creates one PoseStamped
  publisher per tag on `sphero/{_name_safe(name)}/uwb/position` (depth-10
  VOLATILE / default QoS), runs trilateration + per-tag CV Kalman in cm.
  `_detection_callback` rejects tag_ids not in `tag_to_sphero`. `fake_mode`
  synthesizes positions only for `fake_tag_ids` that are also in
  `tag_to_sphero`. `_diagnostics_callback` keys status by tag+sphero name.
  `anchor_positions_cm` is a flat 8-float startup param reshaped to 4x2.
- **Webserver** (`multirobot_webapp.py`): plain Flask app (NOT a ROS node) +
  a `FleetNode` ROS node spun in a background thread. The front end
  (`static/js/app.js`) polls REST `/api/spheros` every 5s and
  `/api/aruco_slam/status` every 4s — there is **no live Socket.IO wired into
  the Python** (docs mention flask-socketio but it is not used). So the web
  contract will be REST endpoints in the existing `/api/*` style.
  - `FleetNode.add_robot(name, name_safe)` subscribes to
    `/sphero/{name_safe}/sensors` (`SpheroSensor`) and seeds an entry with
    x/y/heading/battery. `_on_sensor` writes x/y from `msg.x/msg.y`, heading
    from `msg.yaw`, battery from `msg.battery_percentage`.
    `_publish_fleet_state` emits `FleetRobot.pose = Point(x,y,0)` on the
    latched `/sphero_fleet/robots`.
  - `SpheroInstanceManager` launches per-Sphero WS servers and the ArUco SLAM
    node and the Foxglove bridge via `subprocess.Popen`
    (`start_foxglove_bridge` / `stop_foxglove_bridge` are the reuse pattern).
  - `name_safe` in the webapp is `name.replace('-', '_')`, matching the BLE
    node's `_name_safe`. With per-tag decoupling this consistency becomes moot
    (see Resolved Questions).
- **Messages**: `FleetRobot` has `geometry_msgs/Point pose` (no tag_id field
  today); `FleetState` is `Header + FleetRobot[] + bool aruco_slam_running`.
- **Units**: BLE node works in **cm**. The dashboard reads `pose.x/pose.y`
  with no conversion; ArUco/odometry pose units are currently whatever
  `SpheroSensor.x/y` are. See Resolved Questions for the conversion decision.

### Key design decisions (driven by requirements A-E)
1. BLE node publishes **one PoseStamped per tag id** on
   `uwb/tag/<id>/position` (relative topic; resolves to `/uwb/tag/<id>/position`
   at the node's default namespace). 16 tags (ids 1-16). No `sphero_names`,
   no `tag_to_sphero`. Anchor positions remain startup params.
2. **FleetNode owns the tag pool** (1-16) and the `tag_id -> sphero` map. On
   add-Sphero the operator selects a free tag id; FleetNode subscribes to
   that tag's topic and routes its position into that robot's `pose`. Battery
   + heading continue to come from `SpheroSensor`. Tag id freed on remove.
3. **SpheroInstanceManager owns the BLE node lifecycle** (start/stop child
   process), gated on anchor configuration, mirroring the Foxglove pattern.
4. **Combined launch file** starts the webserver (+ optional Foxglove). The
   BLE node is started on-demand by the webserver, not by the launch file.

## Detailed Plan

### Step 1: Refactor BLE node to publish per-tag-id, drop sphero_names
- **Action:**
  - Remove the `sphero_names` param and the `tag_ids`/`sphero_names`
    length-match + `tag_to_sphero` construction. Replace with a single
    `tag_ids` default of `[1..16]` (the active tag pool the node publishes
    for).
  - Change publisher creation: for each `tid` in `tag_ids`, create a
    `PoseStamped` publisher on relative topic `f"uwb/tag/{tid}/position"`
    (depth-10, default QoS — unchanged). Keep `self._pose_pubs: dict[int, Publisher]`.
  - `_detection_callback`: replace `if tag_id not in self.tag_to_sphero` with
    `if tag_id not in self._pose_pubs` (i.e. unknown/inactive tag). All
    trilateration/KF logic unchanged.
  - `_publish_callback`: iterate `self._pose_pubs` (or `tag_ids`) instead of
    `tag_to_sphero`; stale-warning text uses only the tag id (drop sphero name).
  - `_diagnostics_callback`: key `status.name` / values by tag id only
    (e.g. `uwb_tag_{tid}`), drop `sphero_name` KeyValue. `_name_safe` is then
    unused — remove it.
  - `fake_mode`: keep, but gate on `tid in self._pose_pubs` instead of
    `tag_to_sphero`. `fake_tag_ids` param stays.
  - `anchor_positions_cm` param: unchanged (startup-only, flat 8 floats -> 4x2).
- **Files:** `src/sphero_uwb_positioning/sphero_uwb_positioning/ble_position_node.py`
- **Commands:** none (build in Step 6).
- **Expected outcome:** Node publishes `/uwb/tag/<id>/position` for ids 1-16,
  needs no Sphero names, still reads anchors at startup.

### Step 2: Update BLE node config + (optionally) launch defaults
- **Action:** In `config/ble_uwb_config.yaml`, remove `sphero_names`, set
  `tag_ids: [1..16]`, keep `anchor_positions_cm` placeholder + other params.
  Note: the webserver will pass `tag_ids` and `anchor_positions_cm` as CLI
  `--ros-args -p` overrides, so the config is mainly for standalone runs.
  Leave `uwb_ble.launch.py` as-is (standalone/manual use); it is NOT part of
  the on-demand webserver flow.
- **Files:** `src/sphero_uwb_positioning/config/ble_uwb_config.yaml`
- **Expected outcome:** Standalone `ros2 launch sphero_uwb_positioning uwb_ble.launch.py`
  still works with the new per-tag scheme.

### Step 3: Add tag_id to FleetRobot message
- **Action:** Add `uint8 tag_id` to `FleetRobot.msg` (0 = unassigned). This
  lets the dashboard show which tag drives each robot and lets the front end
  render the assignment without a second lookup.
- **Files:** `src/multirobot_msgs/msg/FleetRobot.msg`
- **Commands:** rebuild `multirobot_msgs` (Step 6).
- **Expected outcome:** `FleetState` now carries the tag assignment per robot.
- **Note:** This is the only message change. If the user prefers zero message
  changes, the tag map can live only in the manager and be exposed via REST
  (`/api/uwb/tags`) — see Open Questions.

### Step 4: FleetNode owns the tag pool + UWB pose routing
- **Action:** In `FleetNode`:
  - Add `self.uwb_frame_to_meters` / unit handling (see Step 7) and a
    `self._tag_subs: dict[str, Subscription]` plus per-robot `tag_id`,
    `uwb_x`, `uwb_y` fields in the `robots[name]` dict.
  - Add a matching QoS for the tag subscription: depth-10, default
    reliability/durability (VOLATILE) to match the BLE publisher.
  - Change `add_robot(name, name_safe, tag_id)`: in addition to the existing
    `SpheroSensor` subscription, if `tag_id` is set, create a `PoseStamped`
    subscription on `/uwb/tag/<tag_id>/position` with callback
    `_on_uwb(name, msg)` that stores `uwb_x/uwb_y` (converted to FleetState
    units). Store `tag_id` in the robot entry.
  - `_on_sensor`: STOP writing `entry['x']/entry['y']` from `msg.x/msg.y`
    (UWB replaces odometry for pose). Keep writing `battery` and `heading`.
    Pose x/y now come exclusively from `_on_uwb`.
  - `_publish_fleet_state`: set `robot.pose = Point(x=uwb_x, y=uwb_y, z=0.0)`
    and `robot.tag_id = entry['tag_id']`.
  - `remove_robot(name)`: destroy both the sensor sub and the UWB tag sub;
    free the tag id back to the pool.
  - Add `assign_tag(name, tag_id)` / `free_tag(name)` helpers and a
    `free_tag_ids()` / `tag_assignments()` query used by the REST layer. Pool
    = ids 1-16 minus currently assigned. Guard against double-assignment.
- **Files:** `src/multirobot_webserver/multirobot_webserver/multirobot_webapp.py`
- **Expected outcome:** Each robot's `pose` is driven by its assigned UWB tag;
  battery/heading still from `SpheroSensor`; tag ids are pooled and recycled.

### Step 5: SpheroInstanceManager — BLE node lifecycle + tag-aware add/remove
- **Action:** In `SpheroInstanceManager`:
  - Add fields: `self.uwb_process: Optional[subprocess.Popen] = None`,
    `self.anchor_positions_cm: Optional[list[float]] = None` (8 floats),
    `self.uwb_tag_ids = list(range(1, 17))`.
  - Add `set_anchor_positions(coords)`: validate 4 (x,y) pairs -> store flat
    8-float list. Returns success/message. Anchors are NOT runtime-reconfig
    for a running node; if the node is already running, return a message that
    the operator must stop/restart positioning to apply new anchors.
  - Add `start_uwb()`: require `anchor_positions_cm` to be set first (else
    fail with message). Launch the BLE node as a child process, Foxglove-style:
    `ros2 run sphero_uwb_positioning ble_position_node --ros-args
    -p anchor_positions_cm:=[...] -p tag_ids:=[1,...,16]`
    (plus `-p fake_mode:=true` when requested for testing). `time.sleep(2)`,
    poll `process.poll()`, return success/message.
  - Add `stop_uwb()` mirroring `stop_foxglove_bridge`.
  - Add `is_uwb_running()`.
  - `add_sphero(sphero_name, tag_id)`: thread the selected `tag_id` through to
    `fleet_node.add_robot(sphero_name, sphero_name.replace('-','_'), tag_id)`.
    Validate that `tag_id` is in the free pool before launching; reject
    otherwise.
  - `remove_sphero`: unchanged except `fleet_node.remove_robot` frees the tag.
  - `shutdown_all`: also `stop_uwb()`.
- **Files:** `src/multirobot_webserver/multirobot_webserver/multirobot_webapp.py`
- **Expected outcome:** Webserver starts/stops the BLE node on demand, gated on
  anchor config; add-Sphero binds a chosen tag id.

### Step 6: Web-facing REST contract (Flask routes) — see contract section
- **Action:** Add Flask routes (REST, matching existing `/api/*` polling style;
  no Socket.IO). Modify the existing `POST /api/spheros` to accept an optional
  `tag_id`. Add `/api/uwb/tags`, `/api/uwb/anchors` (GET+POST),
  `/api/uwb/start`, `/api/uwb/stop`, `/api/uwb/status`. Full payload shapes in
  the "Web-Facing Contract" section below.
- **Files:** `src/multirobot_webserver/multirobot_webserver/multirobot_webapp.py`
- **Expected outcome:** A `web_expert` agent can build the UI against a stable
  documented contract; no front-end code is written in this plan.

### Step 7: Units — cm vs meters
- **Action:** Decide and implement one conversion point. Recommended: keep
  `FleetState`/`FleetRobot.pose` in **meters** (ROS convention; Foxglove and
  `/tf` expect meters). Convert cm->m **once** in `FleetNode._on_uwb`
  (`x_m = msg.pose.position.x / 100.0`). The BLE node and its
  `/uwb/tag/<id>/position` topics stay in cm (as documented in the node). Flag
  that the existing odometry path was previously feeding `pose` in unknown
  units; since UWB now replaces it, pose is unambiguously meters downstream.
- **Files:** `multirobot_webapp.py` (the `_on_uwb` conversion).
- **Expected outcome:** Dashboard/Foxglove see meters; UWB topics remain cm.
- **Open question:** confirm the dashboard currently expects meters (Open Q3).

### Step 8: Combined launch file
- **Action:** Add `multirobot_webserver.launch.py` that runs the
  `multirobot_webapp` entry point and (optionally, via a launch arg
  `foxglove:=true`) does nothing extra for the BLE node — the BLE node is
  started by the webserver on demand, not by the launch file. Because the
  webapp uses blocking `input()` for ArUco prompt and `app.run()`, the launch
  file runs it as a process with `output='screen'`. Note: the webapp already
  self-starts Foxglove in `main()`, so the combined launch may simply launch
  the webapp; document this rather than duplicating Foxglove.
- **Files:** `src/multirobot_webserver/launch/multirobot_webserver.launch.py`,
  `setup.py` already globs `launch/*.py` (no setup change needed).
- **Expected outcome:** `ros2 launch multirobot_webserver multirobot_webserver.launch.py`
  brings up the webserver; BLE node remains on-demand.

### Step 9: Dependencies + build
- **Action:** Add `<exec_depend>sphero_uwb_positioning</exec_depend>` to
  `multirobot_webserver/package.xml` (webserver now launches it as a child
  process). No new Python imports of the package are needed (it's a subprocess).
- **Files:** `src/multirobot_webserver/package.xml`
- **Commands (build order: msgs first, then dependents):**
  ```
  colcon build --packages-select multirobot_msgs
  source install/setup.bash
  colcon build --packages-select sphero_uwb_positioning multirobot_webserver
  source install/setup.bash
  ```
- **Expected outcome:** Clean build, FleetRobot regenerated with `tag_id`.

## Expected Outcomes
- BLE node publishes `/uwb/tag/<id>/position` (PoseStamped, cm) for ids 1-16,
  with no dependency on Sphero names.
- FleetNode owns the tag pool; operator-selected tag drives each robot's pose
  (UWB replaces odometry); battery/heading still from `SpheroSensor`; tags
  recycled on removal.
- Webserver starts/stops the BLE node on demand, gated on anchor config.
- A documented REST contract for tag assignment, anchor config, and node
  start/stop/status.
- A combined launch file for the webserver.

## Potential Risks & Considerations
- **Message change ripple:** adding `tag_id` to `FleetRobot` forces a rebuild
  of `multirobot_msgs` and any consumer (Foxglove layouts are field-tolerant;
  dashboard reads via REST, not the message). Low risk.
- **QoS mismatch:** BLE publisher uses default (VOLATILE, depth 10). FleetNode
  tag subscription must match (no TRANSIENT_LOCAL) or it will get no data.
- **Anchor reconfig:** explicitly NOT supported at runtime. Changing anchors
  requires stop/start of the BLE node. UI must surface this.
- **Tag-before-node race:** a robot can be added with a tag id before the BLE
  node is running; the subscription simply receives nothing until the node
  starts. Acceptable; document in contract (`/api/uwb/status`).
- **Blocking input() in main():** the existing ArUco `input()` prompt blocks
  startup; under `ros2 launch` this still works on an attached TTY but not when
  detached. Out of scope to change; note it.
- **Units:** if the dashboard actually expects cm, the cm->m conversion must
  move/disappear (Open Q3).

## Testing Plan
- **BLE node, fake mode (no hardware):**
  ```
  ros2 run sphero_uwb_positioning ble_position_node --ros-args \
    -p fake_mode:=true -p fake_tag_ids:=[1,2] -p tag_ids:=[1,2,3,4] \
    -p anchor_positions_cm:=[0.,0.,300.,0.,300.,300.,0.,300.]
  ros2 topic list | grep uwb            # expect /uwb/tag/1..4/position
  ros2 topic echo /uwb/tag/1/position   # expect moving PoseStamped (cm)
  ros2 topic echo /diagnostics          # expect uwb_tag_1 OK
  ```
- **FleetNode routing:** start webserver, configure anchors via
  `POST /api/uwb/anchors`, `POST /api/uwb/start` (with fake_mode for bench
  test), `GET /api/uwb/tags` to see free pool, add a Sphero with `tag_id`,
  then `ros2 topic echo /sphero_fleet/robots` and confirm that robot's
  `pose.x/pose.y` track the fake UWB position (in meters) and `tag_id` is set.
- **Tag recycling:** remove the Sphero, `GET /api/uwb/tags` shows the id free
  again.
- **Lifecycle gating:** `POST /api/uwb/start` before anchors set -> 400 with
  "anchors not configured".

## Web-Facing Contract (for a separate web_expert agent)

All endpoints are REST/JSON in the existing `/api/*` style; the front end
polls (current cadence: fleet 5s, aruco 4s). No Socket.IO. Add a `uwb` poll
(suggest 4s) for `/api/uwb/status` + `/api/uwb/tags`.

### 1. Tag assignment when adding a Sphero
- **Modified** `POST /api/spheros`
  - Request body (tag_id now optional but recommended):
    ```json
    { "sphero_name": "SB-3660", "tag_id": 3 }
    ```
  - On success `201`: existing shape plus the assigned tag echoed:
    ```json
    { "success": true, "message": "...", "instance": { "...": "...", "tag_id": 3 } }
    ```
  - Errors `400`: `{ "success": false, "message": "tag_id 3 already assigned" }`
    or `"tag_id 3 out of range (1-16)"`. If `tag_id` omitted, robot is added
    with `tag_id: 0` (no UWB pose) — UI should normally require selection.
- **New** `GET /api/uwb/tags` — reports the pool so the UI can populate the
  selector and disable taken ids:
  ```json
  {
    "success": true,
    "all": [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16],
    "free": [1,2,4,5,6,7,8,9,10,11,12,13,14,15,16],
    "assigned": { "3": "SB-3660" }
  }
  ```

### 2. Submitting the 4-anchor coordinate map (A0-A3, x/y in cm)
- **New** `POST /api/uwb/anchors`
  - Request body (centimeters, A0..A3 in order):
    ```json
    { "anchors_cm": [
        { "x": 0,   "y": 0   },
        { "x": 300, "y": 0   },
        { "x": 300, "y": 300 },
        { "x": 0,   "y": 300 }
    ] }
    ```
  - Success `200`: `{ "success": true, "message": "Anchor map stored" }`.
    If the BLE node is already running:
    `{ "success": true, "message": "Stored; stop/start positioning to apply" }`.
  - Error `400`: `{ "success": false, "message": "expected exactly 4 anchors" }`.
- **New** `GET /api/uwb/anchors` — returns the currently stored map (or null):
  ```json
  { "success": true, "anchors_cm": [ {"x":0,"y":0}, ... ], "configured": true }
  ```

### 3. Start / stop / status of the positioning node
- **New** `POST /api/uwb/start`
  - Optional body: `{ "fake_mode": false }` (bench testing).
  - Requires anchors configured first.
  - Success `200`: `{ "success": true, "message": "UWB positioning started" }`.
  - Error `400`: `{ "success": false, "message": "anchors not configured" }`
    or `{ "success": false, "message": "UWB positioning already running" }`.
- **New** `POST /api/uwb/stop`
  - Success `200`: `{ "success": true, "message": "UWB positioning stopped" }`.
  - Error `400` if not running.
- **New** `GET /api/uwb/status` — for polling:
  ```json
  {
    "success": true,
    "running": true,
    "anchors_configured": true,
    "fake_mode": false,
    "assigned_count": 1
  }
  ```
- **Hook-in note:** all of the above live alongside the existing
  `/api/aruco_slam/*` routes in `multirobot_webapp.py` and call new
  `SpheroInstanceManager` / `FleetNode` methods (Steps 4-5). The front end
  should mirror the existing ArUco start/stop/status UI pattern for the UWB
  node, add a tag-id `<select>` to the add-Sphero modal driven by
  `GET /api/uwb/tags`, and add an anchor-coordinate form posting to
  `/api/uwb/anchors`.

## Resolved Questions (addressed per the brief)
- **`name_safe` consistency:** Moot. The BLE node no longer uses Sphero names
  or `_name_safe`; it publishes per tag id. The webapp keeps
  `name.replace('-','_')` only for the `SpheroSensor` topic (battery/heading).
  `_name_safe` is removed from the BLE node.
- **`fake_mode` after dropping `sphero_names`:** retained; gated on
  `tid in self._pose_pubs` (i.e. in the active `tag_ids` pool) instead of the
  old `tag_to_sphero`. `fake_tag_ids` param stays.
- **Per-tag publisher creation:** driven solely by `tag_ids` (default 1-16).
- **QoS:** FleetNode's tag subscription uses depth-10 VOLATILE to match the
  BLE publisher's default profile.
- **Units:** convert cm->m once in `FleetNode._on_uwb`; topics stay cm,
  FleetState pose becomes meters (pending Open Q3 confirmation).

## Open Questions for the user
1. **FleetRobot.tag_id field (Step 3):** OK to add `uint8 tag_id` to
   `FleetRobot.msg` (forces a `multirobot_msgs` rebuild)? Or keep the message
   unchanged and expose the tag map only via `GET /api/uwb/tags`?
2. **Tag pool size:** confirm exactly 16 tags, ids 1-16 (the brief says 16;
   the old default was 4).
3. **Pose units:** does the existing dashboard / Foxglove layout expect
   **meters** (ROS convention, my recommendation) or **cm**? This decides
   whether the cm->m conversion in `_on_uwb` stays.
4. **Add-Sphero without a tag:** should `POST /api/spheros` REQUIRE a `tag_id`
   (reject if missing), or allow adding a robot with no UWB pose (`tag_id: 0`)?
5. **Anchor input units:** confirm the UI submits anchor coordinates in **cm**
   (matches the node's `anchor_positions_cm`). If the operator measures in mm
   or m, conversion belongs in the route.

## Approval Status
- [x] Waiting for user approval
- [x] Approved
- [x] Executed

## Execution Notes (2026-05-27)
- Final user decisions applied: REST polling (no Socket.IO); FleetState pose in
  METERS (cm->m in `FleetNode._on_uwb`); `uint8 tag_id` added to FleetRobot;
  tag_id REQUIRED on `POST /api/spheros`; 16 tags (ids 1-16); anchors in cm.
- **Deviation (forced by ROS):** topic is `uwb/tag_<id>/position`, NOT
  `uwb/tag/<id>/position`. ROS topic name tokens may not start with a digit;
  the original string crashed the node at startup. Applied consistently in the
  BLE node publisher and the FleetNode subscriber.
- Builds passed: multirobot_msgs, then sphero_uwb_positioning +
  multirobot_webserver.
- Smoke tests passed: BLE fake_mode publishes /uwb/tag_1..4/position (cm) +
  /diagnostics (uwb_tag_N); FleetNode routes tag 5 -> pose in meters,
  tag_id=5 on the wire in /sphero_fleet/robots; tag pool free/assign/recycle
  verified.
