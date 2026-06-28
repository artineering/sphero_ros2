# New Package: `kinect_field_tracking` (replaces deprecated aruco_slam kinect nodes)

**Created:** 2026-06-28T13:55:49Z
**Status:** Pending Approval

## Task Description
Create a new, self-contained ament_python package `kinect_field_tracking` that does field
calibration, callsign registration, and continuous Kalman-fused 10 Hz tracking of overhead
Kinect-v1-detected Spheros, publishing `/localization/<name_safe>/position` in a calibrated
`field` frame. It replaces the now-deprecated kinect_* nodes in `aruco_slam`.

**HARD CONSTRAINTS**
- Do NOT add to or modify `aruco_slam`. Validated helpers are PORTED BY COPY into the new
  package. No import of `aruco_slam` at runtime.
- Allowed edits OUTSIDE the new package: add `Register.srv` to `multirobot_msgs`; add a
  completion publish to `sphero_instance_device_controller_node.py` in
  `sphero_instance_controller`. Nothing else outside the new package.

---

## Analysis: ground-truth verification of the spec against current code

I read the deprecated `aruco_slam/aruco_slam/kinect_tracking.py` and `kinect_tracker_node.py`,
the device controller, `multirobot_msgs`, `SpheroSensor.msg`, and the FleetState publisher.
Findings (corrections to the spec are flagged **[CORRECTION]**):

- **Fleet roster topic CONFIRMED:** `/sphero_fleet/robots`, type `multirobot_msgs/FleetState`.
  Published by `multirobot_webserver` `FleetNode`. `FleetState{ Header header; FleetRobot[] robots;
  bool aruco_slam_running }`. `FleetRobot{ string name; string name_safe; string status;
  uint8 battery_percentage; geometry_msgs/Point pose; int16 heading; uint8 tag_id;
  float64 added_at; float64 last_seen }`.
  - **[CORRECTION]** The roster is published **LATCHED** (`TRANSIENT_LOCAL`, depth 1). The old
    kinect node subscribed with the default depth-10 **VOLATILE** QoS, so on late-join it would
    miss the latched roster until the next periodic republish. The new node MUST subscribe with
    `TRANSIENT_LOCAL` depth-1 to reliably get the roster on startup.
  - **[CORRECTION]** `FleetRobot` already carries `name_safe`; use `r.name_safe` directly instead
    of recomputing `name.replace('-','_')`. We keep a local `_name_safe()` only as a fallback.
  - **"Deployed" definition:** there is no explicit "deployed" flag; the practical definition is
    "names currently present in `FleetState.robots`," optionally filtered by `status`. See Open
    Questions.

- **`sphero/<name_safe>/led` CONFIRMED:** device controller `led_callback` accepts JSON
  `{type|led, red, green, blue}`; `type` in {`main`,`front`,`back`}. `main` on a BOLT fills/blanks
  the whole 8x8 matrix (one-colour fill). This is the channel we use; there is no
  `set_matrix_color`.

- **`sphero/<name_safe>/calibrate_compass` CONFIRMED:** `String` topic; `calibrate_compass_callback`
  (line 480) BLOCKS the callback thread (~seconds) and currently only logs. We will add a
  completion publish here (see Deliverable 3).

- **`sphero/<name_safe>/sensors` CONFIRMED:** type `sphero_instance_controller/msg/SpheroSensor`,
  fields: `pitch, roll, yaw` (deg), `accel_x/y/z` (G), `gyro_x/y/z` (deg/s), `x, y`,
  `velocity_x, velocity_y`, `battery_percentage`, `builtin_interfaces/Time timestamp`.
  - **[CORRECTION]** the battery field is `battery_percentage` (uint8), not `battery`. We do not
    use battery anyway.

- **Circular-position hazard CONFIRMED:** `_localization_callback` (line 328) calls
  `self.sphero.set_external_location(x, y)` from `/localization/<name_safe>/position` — i.e. our
  own output is injected back into the controller's reported `x,y`. We therefore MUST NOT fuse
  telemetry `x,y`. Use only velocity/orientation/accel/gyro.

- **Ported helpers CONFIRMED present** in `kinect_tracking.py`: `Intrinsics`, `backproject`,
  `project`, `KalmanFilter` (constant-velocity, 4-state), `Track`/`Tracker`, `_associate`
  (greedy NN + gate), `detect_blobs` (baseline subtraction + holes + connected components),
  `FreenectSource` (with `stop()` for stream serialization), `SimSource`, `capture_baseline`,
  and the RGB probe helpers `sample_track_patches`/`probe_hue_response`/`detect_lit_blob`.

- **Build systems:** `multirobot_msgs` is **ament_cmake** (rosidl). `sphero_instance_controller`
  is **ament_cmake** with Python entry points installed via `install(PROGRAMS ...)`. `aruco_slam`
  (the package being replaced) is **ament_python**. The new package will be **ament_python**
  (matches the code being ported and the spec).

- **OpenCV 4.6 legacy:** the field-rectangle detector uses `cvtColor`/`Canny`/`findContours`/
  `approxPolyDP`/`convexHull` only — none of these need `cv2.aruco.ArucoDetector`, so the legacy
  API is fine.

---

## Deliverable 1: Package file tree

```
src/kinect_field_tracking/
  package.xml                      # ament_python; deps below
  setup.py                         # console_scripts entry points (3 nodes)
  setup.cfg                        # [develop]/[install] script dirs (standard ament_python)
  resource/kinect_field_tracking   # ament resource marker (empty file)
  kinect_field_tracking/
    __init__.py
    # --- PORTED BY COPY from aruco_slam/kinect_tracking.py (no aruco_slam import) ---
    detection.py                   # Intrinsics, backproject, project, detect_blobs,
                                    #   capture_baseline, FreenectSource, SimSource
    tracking.py                    # KalmanFilter(CV) + greedy _associate (kept for reuse/tests);
                                    #   NOTE the live node uses the new FusionKF below, not this
    rgb_probe.py                   # sample_track_patches, probe_hue_response, detect_lit_blob
    # --- NEW modules ---
    geometry.py                    # plane fit (RANSAC/LSQ), corner back-projection,
                                    #   field-frame construction, contact-point radius-offset,
                                    #   camera->field 4x4 transform, point transforms (PURE)
    fusion.py                      # FusionKF: the 13-state per-callsign filter (PURE, no ROS)
    calibration.py                 # CalibrationResult dataclass + YAML load/save + heightmap I/O
    field_tracker_node.py          # THE node: state machine, services, 10Hz timer, TF, callbacks
  config/
    kinect_field_tracking.yaml     # all node params (defaults below)
  launch/
    kinect_field_tracking.launch.py
  test/
    test_geometry.py               # plane fit, field-frame, contact-point, transforms
    test_fusion.py                 # predict/update math, predict-only path, body->field rotation
    test_registration_logic.py     # pure registration helpers (target selection, status merge)
    test_detection_sim.py          # detect_blobs over SimSource (no hardware)
    test_copyright.py / test_flake8.py / test_pep257.py  # standard ament lint (optional)
```

Module split rationale: every file under `kinect_field_tracking/` that contains math is PURE
(no rclpy) so it is unit-testable without hardware or a ROS graph. Only `field_tracker_node.py`
touches rclpy.

**`setup.py` `entry_points`** (mirrors aruco_slam style):
```python
entry_points={'console_scripts': [
    'field_tracker_node = kinect_field_tracking.field_tracker_node:main',
]}
```
(Single node. Calibration and registration are services on that node, not separate executables.
Baseline/heightmap capture is a service too — see Deliverable: tracking.) `data_files` install
`config/*.yaml` and `launch/*.launch.py` to `share/`, exactly like aruco_slam.

**`package.xml` deps:** `rclpy`, `std_msgs`, `std_srvs`, `geometry_msgs`, `sensor_msgs`,
`visualization_msgs`, `tf2_ros`, `builtin_interfaces`, `multirobot_msgs`,
`sphero_instance_controller` (for the `SpheroSensor` msg), `python3-numpy`,
`python3-opencv` (exec). `freenect` is NOT a rosdep key (hand-built); document it as a runtime
prerequisite and import lazily (like the ported `FreenectSource`), so `source:=sim` needs no
hardware.

---

## Deliverable 2: `Register.srv` in `multirobot_msgs`

New file `src/multirobot_msgs/srv/Register.srv` (verbatim from spec):
```
string[] callsigns   # empty => register all currently-deployed Spheros
bool skip_compass    # true => skip the compass-cal phase (for re-registering a failed subset)
---
bool success
string[] registered  # turned solid green
string[] failed      # turned solid red
string message
```

`multirobot_msgs/CMakeLists.txt` change — add the srv to the existing generator call:
```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FleetRobot.msg"
  "msg/FleetState.msg"
  "srv/Register.srv"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)
```
No `package.xml` change needed (rosidl build/runtime + member_of_group already present; the srv
uses only built-in primitive types so no new DEPENDENCIES). Field calibration uses the built-in
`std_srvs/Trigger`, so no extra srv is required there.

---

## Deliverable 3: device-controller change (`calibrate_compass_done`)

File: `src/sphero_instance_controller/sphero_instance_controller/sphero_instance_device_controller_node.py`

- In `_create_publishers()` add:
  `self.calibrate_compass_done_pub = self.create_publisher(Bool, f'{self.topic_prefix}/calibrate_compass_done', 10)`
  (`from std_msgs.msg import Bool` at top).
- At the END of `calibrate_compass_callback` (line ~480), in a `finally`, publish the result so
  the new node's wait always unblocks even on failure:
  ```python
  finally:
      done = Bool(); done.data = bool(success)
      self.calibrate_compass_done_pub.publish(done)
  ```
  (`success` defaults to `False` before the try.) This is the ONLY behavioural change; the
  existing spin/log behaviour is untouched. **[DECISION needed]** `Bool` (success flag) vs `String`
  JSON — recommend `Bool`; simplest, and the registrar only needs "this callsign finished."

---

## Deliverable 4: Node architecture (threading, state machine, timer pausing)

**State machine** (enum on the node):
`UNCALIBRATED -> CALIBRATED -> REGISTERING -> TRACKING`
- Startup: if `kinect_field.yaml` exists -> load + re-broadcast static TF -> `CALIBRATED`
  (no detection). Else `UNCALIBRATED`. If `auto_calibrate:=true`, run calibration on startup.
- `~/calibrate` (Trigger): force fresh detect + overwrite YAML + re-broadcast TF. Allowed from
  any state; pauses the tracking timer for its duration.
- `~/register` (Register): requires `CALIBRATED` (or `TRACKING`); transitions to `REGISTERING`,
  runs the blocking pass, then `TRACKING`. Re-registration from `TRACKING` is subset-scoped.
- Tracking timer publishes only in `TRACKING` (and only for locked trackers).

**Callback groups / threading** (mirrors the old node, which worked):
- `MultiThreadedExecutor`.
- `timer_group = MutuallyExclusiveCallbackGroup()` — owns ONLY the 10 Hz tracking timer.
- `service_group = MutuallyExclusiveCallbackGroup()` — owns `~/calibrate`, `~/register`,
  `~/capture_baseline` (all long/blocking).
- `sub_group = ReentrantCallbackGroup()` — owns the FleetState sub, the per-callsign `sensors`
  subs, and the `calibrate_compass_done` subs (cheap, high-rate, must not be starved).
- **Timer pausing:** a `self._paused` flag. The tracking timer early-returns while paused; every
  service that needs the camera (calibrate, register, capture_baseline) sets `_paused=True` on
  entry and clears it in a `finally`. This enforces the USB2 depth-XOR-video rule: only one of
  {tracking-depth-grab, service RGB/depth grab} ever runs.
- **Stream serialization helper:** a small `_with_stream(kind)` context that calls
  `source.stop()` before switching between depth and video, grabs a couple of warm-up frames, then
  reads — exactly the discipline the old `_run_registration` used (`stop()` then 3 grabs).

---

## Deliverable 5: Field calibration math

1. **Capture (serialized):** `_paused=True`. `source.stop()`; grab N depth frames -> robust
   averaged depth (reuse `capture_baseline` averaging idea); `source.stop()`; grab M RGB frames ->
   median RGB. Never both streams at once.
2. **Rectangle detection (color-agnostic, OpenCV legacy):** `cvtColor(BGR2GRAY)` ->
   optional blur -> `Canny` -> `findContours(RETR_EXTERNAL, CHAIN_APPROX_SIMPLE)` -> for each
   contour `approxPolyDP(eps=0.02*peri, closed=True)`; keep those with exactly 4 vertices and
   `isContourConvex`; choose the largest by `contourArea`. Result: 4 corner pixels `(u,v)`.
   (Spec guarantees exactly one rectangle visible.)
3. **Ground-plane fit:** back-project ALL valid depth pixels (`backproject`) to camera-frame 3D;
   fit plane `n·X + d = 0` by RANSAC (inlier threshold ~15 mm) with a least-squares refit on
   inliers. Orient `n` toward the camera (z component negative in camera optical frame -> flip so
   "up" is consistent).
4. **Corner back-projection onto the plane:** for each corner pixel build the ray
   `dir = ((u-cx)/fx, (v-cy)/fy, 1)`; intersect with the fitted plane -> 4 metric 3D corners
   `C0..C3` in the camera frame (this is more accurate than reading noisy depth AT the corner
   pixel, which often sits on an edge/hole).
5. **Field-frame construction (origin = bottom-left of camera view; +x long edge; +y short edge;
   z up; right-handed):**
   - Pick the bottom-left corner: among the 4 corners, order by image pixel coords; "bottom" =
     largest `v` (image y grows downward), "left" = smallest `u`. Concretely: take the two
     corners with the largest `v` (bottom edge in the image), of those choose the smaller `u`
     -> `O` (origin). **[DECISION]** "bottom-left of the camera view" is interpreted in IMAGE
     space (the operator's overhead view); confirm this matches intent (Open Questions).
   - From `O`, its two adjacent rectangle corners give two edge vectors `e1`, `e2` (metric).
     `long = argmax(|e1|,|e2|)`, `short = the other`.
   - `x_hat = normalize(long edge from O)`; `up = plane normal (z up)`;
     `y_hat = normalize(cross(up, x_hat))` then re-pick sign so `y_hat` points along the short
     edge from `O` (dot > 0); `z_hat = cross(x_hat, y_hat)` (right-handed, ~= up).
   - Rotation `R_fc = [x_hat y_hat z_hat]^T` (camera->field), translation `t = -R_fc · O`.
     `T_field_from_cam = [[R_fc, t],[0,1]]`. The published static TF is parent `field` ->
     child `kinect_overhead` = inverse (`T_cam_from_field`), so robots tracked in camera and then
     expressed in field via `T_field_from_cam` are consistent with the TF tree.
6. **Outputs:** broadcast `field -> kinect_overhead` as a STATIC TF on `/tf_static`
   (`tf2_ros.StaticTransformBroadcaster`, latched). Publish the 4 field-frame corners as a
   `visualization_msgs/Marker` LINE_STRIP (and/or `geometry_msgs/PolygonStamped`) on
   `~/field_corners`, frame_id `field`, for Foxglove. Persist to `config/kinect_field.yaml`.
7. **Startup reuse:** if YAML present, load corners + transform and re-broadcast the static TF
   WITHOUT detection. `~/calibrate` forces a fresh detect + overwrite.

---

## Deliverable 6: Kalman model (the core design)

Per-callsign filter `FusionKF` in `fusion.py`, all quantities in the FIELD frame.
State (13): `[px, py, vx, vy, yaw, pitch, roll, ax, ay, az, gx, gy, gz]`
(position m or cm — see units note; velocity per-second; angles deg; accel field-frame; gyro deg/s).

**Prediction (constant-accel position, integrated yaw), dt = 0.1 s:**
- `px += vx·dt + 0.5·ax·dt²`,  `py += vy·dt + 0.5·ay·dt²`
- `vx += ax·dt`,  `vy += ay·dt`
- `yaw += gz·dt`  (gz = field-frame yaw rate; see body->field note)
- `pitch, roll, ax, ay, az, gx, gy, gz` carried (identity transition; driven by measurements)
- `F` = the corresponding 13x13 block matrix (linear; the only nonlinearity is the body->field
  rotation, handled OUTSIDE F at measurement time — see below — so the filter stays a linear KF,
  not an EKF).
- `Q` = block-diagonal-ish process noise: larger on accel/gyro (driven, noisy), moderate on
  vel, small on pos; tunable via config (`q_pos, q_vel, q_yaw, q_acc, q_gyro, q_angle`).

**Measurements / H / R:**
- **Camera position** (only when a blob is associated this cycle):
  `z = [px, py]`, `H_cam` picks `px,py`. `R_cam = diag(r_cam, r_cam)` (default from the old
  `r=100 mm²`, converted to chosen units).
- **Telemetry** (every cycle a `sensors` msg is available), split into independent updates so a
  missing field never blocks the rest:
  - velocity: measure field-frame `[vx, vy]` from body `[velocity_x, velocity_y]` rotated by
    `(yaw + body_to_field_yaw_offset)`; `H_vel` picks `vx,vy`; `R_vel = diag(r_vel,...)`.
  - orientation: `[yaw, pitch, roll]` (yaw with the offset applied to align magnetic-north
    telemetry yaw to field +x); `H_ori`; `R_ori`.
  - accel: field-frame `[ax, ay, az]` from body `[accel_x, accel_y, accel_z]` rotated by yaw
    (z passes through, planar arena); `H_acc`; `R_acc`.
  - gyro: `[gx, gy, gz]` (gz -> yaw-rate term); `H_gyro`; `R_gyro`.
- **DO NOT fuse telemetry position `x,y`** (circular via `set_external_location`).

**Body->field rotation:** body-frame velocity & accel are rotated into field using the filter's
current `yaw` plus the single global `body_to_field_yaw_offset` (default 0.0). Because Phase-0
compass calibration makes all robots' telemetry yaw magnetic-north-referenced, this offset is one
physical constant = angle(magnetic-north -> field +x). Rotation is applied to the MEASUREMENT
before the linear update (so H/R stay constant); this is the standard "rotate the measurement into
the state frame" trick and keeps the filter linear.

**Predict-only on blob loss:** every cycle: `predict()`; if a blob is associated apply the camera
update; ALWAYS apply whatever telemetry updates are available. With no camera, the filter
dead-reckons position from integrated accel/velocity and yaw from gyro — and STILL PUBLISHES.
=> missing blob = predict (+telemetry) only, still publish at 10 Hz.

**Trackers are created ONLY by registration** (`FusionKF` initialized at the registered field
position). The tracking loop never spawns trackers; association is greedy NN with a gating
distance to the LOCKED callsign set (port `_associate`, restricted to locked keys).

**Units note [DECISION]:** the ported pipeline is mm internally and the localization contract is
cm (old node divided by 10). Recommend the fusion filter and `/localization/.../position` work in
**cm** to match the existing contract consumers (`set_external_location`, FleetNode). Telemetry
velocity/accel units (`velocity_x` cm/s? G for accel) must be converted consistently; accel G->cm/s²
(×980.665). Confirm telemetry velocity units (Open Questions).

---

## Deliverable 7: Config params + YAML schema

**`config/kinect_field_tracking.yaml`** (node params, defaults):
```yaml
field_tracker_node:
  ros__parameters:
    source: sim                  # 'kinect' | 'sim'
    auto_calibrate: false        # run ~/calibrate on startup if no YAML
    field_yaml_path: ''          # default -> share/.../config/kinect_field.yaml
    heightmap_path: ''           # default -> alongside field_yaml (npy)
    publish_rate_hz: 10.0
    # intrinsics (Kinect v1 canonical)
    fx: 594.21
    fy: 591.04
    cx: 339.31
    cy: 242.74
    # foreground segmentation (heightmap baseline)
    fg_threshold_mm: 25.0        # |depth - heightmap| > this => foreground
    fg_height_max_mm: 300.0
    include_holes: true
    min_area: 40
    max_area: 6000
    sphere_radius_mm: 36.5       # BOLT radius for contact-point offset
    # association (cm)
    gate_cm: 22.0
    # calibration
    calib_depth_frames: 30
    calib_rgb_frames: 5
    ransac_thresh_mm: 15.0
    canny_lo: 50
    canny_hi: 150
    approx_eps_frac: 0.02
    # registration
    probe_red: 0
    probe_green: 255
    probe_blue: 0
    render_timeout_seconds: 6.0
    off_timeout_seconds: 4.0
    probe_poll_seconds: 0.2
    reset_settle_seconds: 0.5
    discovery_settle_seconds: 1.0
    compass_timeout_seconds: 25.0   # Phase-0 wait for all calibrate_compass_done
    blob_find_timeout_seconds: 5.0  # per-callsign blob confirm timeout
    # fusion / Kalman
    body_to_field_yaw_offset: 0.0
    r_cam: 1.0
    r_vel: 4.0
    r_ori: 2.0
    r_acc: 50.0
    r_gyro: 4.0
    q_pos: 0.04
    q_vel: 1.0
    q_yaw: 1.0
    q_angle: 1.0
    q_acc: 100.0
    q_gyro: 25.0
    # sim only
    sim_spheres: 4
    seed: 0
```

**`config/kinect_field.yaml`** (persisted calibration result schema):
```yaml
field_calibration:
  created: "2026-06-28T..."        # ISO timestamp
  frame_parent: field
  frame_child: kinect_overhead
  intrinsics: {fx: 594.21, fy: 591.04, cx: 339.31, cy: 242.74}
  corners_px: [[u0,v0],[u1,v1],[u2,v2],[u3,v3]]      # detected RGB pixels
  corners_cam_mm: [[x,y,z], ...]                     # back-projected 3D (camera frame)
  corners_field_cm: [[x,y], ...]                     # in field frame (z=0)
  origin_corner_index: <int>                         # which corner is bottom-left
  long_edge_len_cm: <float>
  short_edge_len_cm: <float>
  plane: {n: [a,b,c], d: <float>}                    # camera-frame ground plane
  T_field_from_cam: [[...4x4...]]                     # row-major
```

**Heightmap baseline:** a per-pixel npy (`heightmap.npy`, 640x480 float32), NOT a scalar — the
camera may not be perfectly overhead so the empty-arena depth varies per pixel.
**[DECISION]** Recommend a **separate `~/capture_baseline` (Trigger) service** rather than folding
it into calibration: the field rectangle is geometric and stable, but the empty-arena heightmap
must be recaptured whenever the arena is cleared, and it requires the arena to be EMPTY of robots
(calibration does not). Keeping them separate lets the operator recalibrate the field without
clearing robots, and recapture the baseline without re-detecting the rectangle. Calibration can
optionally also capture a heightmap if `auto_baseline:=true`. Persisted as npy; loaded on startup.

---

## Deliverable 8: Testing approach (NO hardware)

All pure modules are unit-tested with pytest; the node itself is exercised in `source:=sim`.
- `test_geometry.py`:
  - plane fit recovers a known synthetic plane (with noise + outliers) within tolerance.
  - corner back-projection: synthesize 4 pixels from a known field rectangle via `project`, then
    confirm back-projection + transform recovers the field corners.
  - field-frame construction: feed 4 corners in scrambled order -> assert origin = bottom-left,
    +x = long edge, +y = short edge, right-handed (det(R)≈+1), and round-trip a few points
    camera<->field.
  - contact-point radius-offset: a ray + plane + radius -> assert the contact point equals the
    ball-center-dropped-to-plane analytic value.
- `test_fusion.py`:
  - predict-only path: no camera, constant telemetry velocity -> position integrates linearly;
    covariance grows.
  - camera+telemetry update reduces covariance and pulls toward the measurement.
  - body->field rotation: body velocity (1,0) at yaw=90deg, offset=0 -> field velocity ≈ (0,1).
  - offset semantics: nonzero `body_to_field_yaml_offset` rotates as expected.
  - circular-fusion guard: a "telemetry position" input is ignored (no H row exists for it).
- `test_registration_logic.py` (pure helpers, no ROS):
  - target selection: empty `callsigns` -> all deployed; non-empty -> exactly that subset.
  - green/red status merge on re-registration: locked trackers for callsigns NOT in the request
    are preserved; only requested callsigns' green/red and the full status are updated.
  - `registration_complete` JSON shape.
- `test_detection_sim.py`: run `detect_blobs` over `SimSource` frames; assert blob count/positions
  track the sim ground truth within tolerance (ports the old sim idea; no Kinect).
- Manual (when hardware present, documented in README, not part of CI):
  `ros2 launch kinect_field_tracking ... source:=kinect`; `ros2 service call ~/capture_baseline`;
  `~/calibrate`; check `~/field_corners` + TF in Foxglove; `~/register`; `ros2 topic echo
  /localization/<name_safe>/position`; verify 10 Hz continuous publish and predict-only when a
  robot is occluded.

---

## Detailed Plan (execution order, each step verifiable)

### Step 1: `multirobot_msgs` — add `Register.srv`
- Files: create `src/multirobot_msgs/srv/Register.srv`; edit `CMakeLists.txt`.
- Verify: `colcon build --packages-select multirobot_msgs` then
  `ros2 interface show multirobot_msgs/srv/Register` prints the definition.

### Step 2: device controller — `calibrate_compass_done`
- File: edit `sphero_instance_device_controller_node.py` (import Bool, add publisher, finally-publish).
- Verify: `colcon build --packages-select sphero_instance_controller`; node still imports;
  topic `sphero/<name_safe>/calibrate_compass_done` appears when a controller runs.

### Step 3: scaffold `kinect_field_tracking` package
- Files: package.xml, setup.py, setup.cfg, resource marker, empty modules, config, launch.
- Verify: `colcon build --packages-select kinect_field_tracking` succeeds (empty node `main`).

### Step 4: port validated helpers (BY COPY)
- Files: `detection.py`, `tracking.py`, `rgb_probe.py` copied from `kinect_tracking.py`, trimmed
  to what we use; no `aruco_slam` import anywhere.
- Verify: `test_detection_sim.py` passes; `grep -r aruco_slam src/kinect_field_tracking` empty.

### Step 5: pure geometry + fusion + calibration I/O
- Files: `geometry.py`, `fusion.py`, `calibration.py`.
- Verify: `test_geometry.py`, `test_fusion.py` pass.

### Step 6: the node — calibration responsibility
- Field calibration service + startup load + static TF + corners Marker.
- Verify (sim): `~/calibrate` returns success on a synthetic rect; TF `field->kinect_overhead`
  on `/tf_static`; `kinect_field.yaml` written; restart loads it without re-detecting.

### Step 7: the node — registration responsibility
- FleetState sub (TRANSIENT_LOCAL), per-callsign led/calibrate_compass_done pubs/subs, the
  `Register` service: Phase 0 parallel compass + wait-for-done, Phase 1 LED reset, Phase 2 serial
  camera-confirmed probe -> contact point -> field coords -> FusionKF init, green/red, latched
  `~/registration_status`. Subset-scoped re-registration.
- Verify (sim): registration runs end-to-end with the sim source + a stubbed/sim fleet;
  `test_registration_logic.py` passes; `~/registration_status` latched JSON correct on late
  subscribe.

### Step 8: the node — 10 Hz tracking loop + fusion
- Per-callsign `sensors` subs, the timer: predict -> camera update (if associated) -> telemetry
  updates -> publish `/localization/<name_safe>/position` (frame_id `field`) for EVERY locked
  tracker every cycle; predict-only on blob loss.
- Verify (sim): continuous 10 Hz publish for all locked callsigns; occluding a sim blob still
  publishes (dead-reckoned); `ros2 topic hz` ≈ 10.

### Step 9: launch + config + docs
- `kinect_field_tracking.launch.py` (sim default, kinect overridable); both YAMLs; brief README.
- Verify: `ros2 launch kinect_field_tracking kinect_field_tracking.launch.py` brings the node up.

---

## Expected Outcomes
- A self-contained `kinect_field_tracking` package; `aruco_slam` untouched and removable.
- `multirobot_msgs/srv/Register` available; device controller emits `calibrate_compass_done`.
- `~/calibrate` (Trigger), `~/capture_baseline` (Trigger), `~/register` (Register) services.
- Calibrated `field` static TF + corners marker for Foxglove.
- Continuous 10 Hz `/localization/<name_safe>/position` in `field` for every registered callsign,
  Kalman-fused from camera + telemetry, dead-reckoning through blob loss.
- Full no-hardware test suite green.

## Potential Risks & Considerations
- **Cross-host telemetry reachability (INTEGRATION CHECK):** `sphero/<name>/sensors` and
  `calibrate_compass_done` originate on the distributed BLE worker Pis. The same cross-host gap
  bit heartbeat status before. Must verify these topics actually reach the rpi5 ROS graph
  (DDS discovery / `ROS_DOMAIN_ID` / multicast). If they don't, fusion degrades to camera-only
  and Phase-0 compass-done waits will time out. Flagged as a required pre-integration check, not
  solvable in code here.
- USB2 depth-XOR-video: any new code path that grabs a stream must go through the pause + `stop()`
  serialization or it will corrupt with "Invalid magic."
- LATCHED roster QoS mismatch (corrected above) — must use TRANSIENT_LOCAL to get the roster on
  startup.
- Plane fit / rectangle detection robustness to lighting and partial occlusion of the field edge.
- Yaw offset / compass: if compass cal is skipped, `body_to_field_yaw_offset` is no longer a single
  shared constant and field-frame velocity/accel rotation will be wrong per-robot.
- Units consistency (mm vs cm; accel G vs cm/s²; telemetry velocity units) — single biggest source
  of silent fusion bugs.

## Open Questions (please decide before implementation)
1. **"Deployed" definition:** all names in `FleetState.robots`, or only those with a particular
   `status` value (e.g. connected/online)? What are the valid `status` strings?
2. **Bottom-left in image space vs field/world?** I interpreted "bottom-left of the camera view"
   as image-pixel space (operator's overhead view). Confirm.
3. **`calibrate_compass_done` payload:** `Bool` (recommended) or `String` JSON?
4. **Baseline heightmap capture:** separate `~/capture_baseline` service (recommended) or a
   sub-step of `~/calibrate`?
5. **Working units for fusion + the localization contract:** cm (recommended, matches existing
   consumers) confirmed?
6. **Telemetry units:** what are `velocity_x/_y` units (cm/s? mm/s?) and is `accel_*` in G
   (×980.665 -> cm/s²)? Needed to wire R/Q and conversions correctly.
7. **Contact-point "drop vertically in field-z":** confirm ball center = blob-ray point stepped
   inward by `sphere_radius_mm` along the ray, then projected onto the ground plane along field +z
   (i.e. report the ground contact, z=0 in field).
8. **Compass-done timeout behaviour:** if some robots never report done within
   `compass_timeout_seconds`, proceed with the rest, or fail the whole register? (Recommend
   proceed + note in message.)

## Approval Status
- [x] Waiting for user approval
- [x] Approved (all 8 open questions resolved; gyro correction: NO change to state.py)
- [x] Executed (2026-06-28; build green, 20/20 tests pass; no git commit per instructions)
