# LED-Matrix Active-Marker Positioning as a Third Source (ArUco kept intact)

**Created:** 2026-05-29T00:31:07Z
**Revised:** 2026-05-29 (v2 — pivoted from passive chassis color to active LED-matrix markers)
**Status:** Pending Approval

## Task Description
Add an OpenCV vision pipeline as a **third** selectable positioning source, alongside the existing
**ArUco** robot-tracking and **UWB/BLE** sources. ArUco robot tracking is NOT replaced.

**Identity mechanism (revised):** robots have **no chassis** (HRI study uses bare Spheros), so passive
chassis color is out. Instead, identity is an **active, self-assigned marker on the Sphero's 8×8 LED
matrix**, which the robot keeps **facing up** toward an overhead camera. Identity =
**(matrix hue ∈ 8) × (matrix fill ∈ {filled, ring}) = 16**. Both primitives are **rotation-invariant**
(the ball yaws freely about vertical, so spatial/letter codes are ambiguous; a solid hue and a
concentric filled-vs-ring are not). Position = matrix-blob centroid in field cm. Reuse the existing
field-corner ArUco calibration homography (`FieldMapper`). **All three sources** (ArUco, Matrix, UWB)
publish the **identical neutral interface** `/localization/<name_safe>/position` so they are fully
interchangeable downstream — consumers read one topic regardless of the active source.

## Why this scheme (design rationale, decided with user)
- **Active emissive markers** beat passive color for bare balls: high contrast vs background,
  robust to ambient lighting, and identity is **programmable** (assign/reassign in software).
- **Matrix faces up** → an overhead camera sees it face-on (user-confirmed Spheros hold matrix up).
- **Resolution reality:** matrix is **1″ (2.54 cm)**. At 1920×1080 over a 300×200 cm field (~6 px/cm)
  the matrix projects to only **~15–16 px square → ~2 px per 8×8 cell**. Therefore:
  - Solid matrix **color**: trivially readable.
  - **Filled vs ring** (center-on vs center-off): a 2-zone decision, marginal-but-feasible at ~16 px.
  - Fine **X/O glyphs**: NOT reliable at ~2 px/cell, and orientation-ambiguous under yaw. **Dropped.**
- **8 hues × {filled, ring} = 16** hits the fleet exactly with only rotation-invariant signals and
  **no camera/hardware change**. (User selected this scheme.)

## Verified control + contract surface (this workspace)
- **Matrix command (per robot):** topic `sphero/<name_safe>/matrix`, `std_msgs/String` JSON
  `{pattern, matrix[64], red, green, blue, duration}` → `matrix_callback`
  (`sphero_instance_device_controller_node.py:357`) → `Sphero.set_matrix(...)` (`core/sphero/sphero.py:347`).
  - **Solid fill:** `custom_matrix = [1]*64` at the chosen RGB.
  - **Ring:** predefined `'circle'` pattern (`core/sphero/matrix_patterns.py:125`), or a custom
    border-on/center-off matrix, at the chosen RGB.
- **Aim LEDs (per robot):** topic `sphero/<name_safe>/led`, `Sphero.set_led(r,g,b,led_type)`
  (`core/sphero/sphero.py:78`) — front/back/main. Used optionally to enlarge the color signal (below).
- **Existing publishers (pre-migration):**
  - ArUco → `/aruco_slam/<name_safe>/position`, `PoseStamped`, `frame_id='field'`, cm, z=0, identity
    (`aruco_slam_node.py:90-93`). Keyed by **name**.
  - UWB → `/uwb/tag_<id>/position`, `PoseStamped`, `frame_id='sphero_arena'` (param), cm, z=0, identity
    (`ble_position_node.py:152-158, 305-312`). Keyed by **tag_id int**, no name map in the node.
- **Existing consumers:**
  - Device controller subscribes `/aruco_slam/<name_safe>/position` when `external_localization=true`
    (`sphero_instance_device_controller_node.py:116-122`). Topic built from `topic_name_safe`.
  - `FleetNode` subscribes `/uwb/tag_<id>/position`, keyed by tag_id, in `add_robot(...)`
    (`multirobot_webapp.py:62-93`); converts cm→m. `tag_id→name` map (`tag_assignments_map`) is built
    dynamically by the Flask REST API at robot-add time — it lives ONLY in the webserver.

### Neutral-topic decision (DECIDED — all sources share `/localization/<name_safe>/position`)
The standardized contract for **all three** sources: `/localization/<name_safe>/position`,
`geometry_msgs/PoseStamped`, `header.frame_id='field'`, `pose.position.{x,y}` in **field cm**, z=0,
identity orientation. `name_safe = name.replace('-','_')`.
- **Consumers become source-agnostic:** device controller and FleetNode each subscribe ONLY to
  `/localization/<name_safe>/position` — no per-source subscription swapping. Whichever source is the
  active publisher transparently feeds them. (This is simpler than the old shared-`/aruco_slam` swap.)
- **One active source at a time:** because all three now publish the same per-robot topic, exactly one
  may publish at once (else publishers fight). The source selector enforces this for ArUco/Matrix/UWB
  alike (UWB is no longer independently co-runnable — intended, per the interchangeable-source goal).
- **UWB re-keying (the one added cost):** the UWB node must publish by **name**, so it needs a
  `tag_id→name` map. Add a `tag_names` param (or `tag_name_map`) to `BlePositionNode` and have the
  webserver pass the current assignment when it launches/starts the UWB node (it already owns the map);
  a static map serves dev/launch. Also set UWB `frame_id` param to `'field'` to match the contract.

## Closed-loop identity (key difference from v1)
Identity is **assigned by the system, then read back** — there is no recognition bootstrapping:
1. A fixed table `robot_name -> (hue, fill)` (16 entries) is the ground truth.
2. On startup / on entering matrix-tracking mode, the node **publishes a matrix command to each
   robot** (`sphero/<name_safe>/matrix`) to display its assigned (hue, fill) marker.
3. Vision detects each emissive blob, classifies (hue, fill), looks up the name, emits the pose.
4. **Conflict note:** while matrix-tracking is active, the matrix is reserved for the ID marker and
   cannot simultaneously show application/stimulus content. Flagged as a research-design constraint
   (esp. for the HRI study) — see Open Questions.

## Detailed Plan

### Recommendation: new files INSIDE the existing `aruco_slam` package
Reuses `FieldMapper` + corner ArUco detection, publishes `/aruco_slam/...`. One extra console_script.
ArUco files stay put, untouched except a surgical capture extraction.

### Step 1: Shared camera capture module (with locks)
- New `aruco_slam/camera_capture.py`, class `CameraCapture`:
  - Wraps `cv2.VideoCapture(camera_id, cv2.CAP_V4L)` at 1920×1080.
  - Locks (default ON), tuned for emissive markers: `CAP_PROP_AUTO_WB=0` + fixed `WB_TEMPERATURE`;
    `CAP_PROP_AUTO_EXPOSURE=1` (manual) + **low fixed `CAP_PROP_EXPOSURE`** to keep bright LEDs from
    blooming to white (preserves hue); optional `CAP_PROP_AUTOFOCUS=0`.
  - `open()`, `read()->frame|None`, `release()`.
- Modify `aruco_slam/aruco_detector.py` to use `CameraCapture` (capture only; detection untouched).

### Step 2: Marker definitions + dynamic allocation + color-calibration module
- New `aruco_slam/matrix_marker.py`:
  - The `hue_name -> RGB` palette and the ordered 16-slot marker pool (8 hues × {filled, ring}).
    8-hue palette chosen for emissive separability (e.g. Red, Orange, Yellow, Green, Cyan, Blue,
    Magenta, Purple) — avoid White/near-off (bloom washout).
  - `marker_to_matrix_cmd(hue, fill)`: builds the `sphero/<name>/matrix` JSON (`[1]*64` for filled,
    `'circle'` for ring, at the hue RGB).
  - `assign_markers(name_to_marker, publisher_fn)`: publishes each robot's matrix command from the
    supplied `name→(hue,fill)` map (the map is built by the webserver at add-time, not hardcoded).
- **Dynamic allocation lives in the webserver** (Step 8): a 16-slot marker pool, allocated when a robot
  is added to the fleet (alongside its UWB tag_id), stored in a `marker_assignments_map` (name↔marker),
  freed on removal. The matrix node receives this map as a param/topic when the matrix source starts.
- New `aruco_slam/color_calibrator.py` (console entry `color_calibrator`):
  - Commands robots to cycle through each hue (filled), samples the observed emissive blob hue per
    color under current camera locks, writes learned hue centroids+radii to `config/color_model.yaml`.
  - **Separability guard:** warn if any two hue centroids overlap within radius; prompt to lower
    exposure / adjust palette. (Far easier than v1's achromatic case — emissive hues separate well.)

### Step 3: Matrix-marker detector (the new robot-ID front-end)
- New `aruco_slam/matrix_detector.py`, class `MatrixMarkerDetector`. Per frame, in order:
  1. **Capture** frame (locked WB, low exposure).
  2. **Mask** to the calibrated field quad (homography corners); mask out the 4 corner-marker regions.
  3. **Blob extraction:** threshold on brightness (emissive matrix is bright vs field) → binary mask;
     morphological open/close; connected components; filter by `min/max_blob_area_px`.
  4. **Hue classification:** per blob, mean Lab/HSV chroma → nearest learned hue centroid within radius.
  5. **Fill classification (filled vs ring):** compare center-region intensity to ring-region intensity
     within the blob; center-bright → filled, center-dark → ring. (~16 px; tune
     `ring_center_frac`, `ring_contrast_thresh`; readability validated in testing.)
  6. **ID lookup:** (hue, fill) -> robot name via the table.
  7. **Centroid:** blob pixel centroid.
  8. **camera_to_field:** `FieldMapper.camera_to_field_batch()` -> field cm.
  9. **Publish** per-robot PoseStamped on `/localization/<name_safe>/position`. Dedupe duplicate IDs
     (keep larger/brighter blob; log warning).
- **Main-LED color boost (ENABLED) — LED ownership split by channel:** the matrix node drives the
  **`main`** LED to each robot's hue (`sphero/<name>/led` with `type:"main"`) so the whole ball glows
  that hue, enlarging the color signal beyond the ~16 px matrix. The **`front`/`back` (aim) LEDs are
  owned by the state-machine layer for behavioral identity** (blink fast/slow). No contention: same
  topic, different `type` routes to different physical LEDs. Invariants:
  - **Tracking never depends on the LEDs** — the **matrix is authoritative** for (hue, fill); the
    detector classifies from the matrix blob alone, using `main`-LED glow only as an opportunistic
    boost. Behavioral blinking on front/back never affects tracking.
  - Matrix node writes ONLY `type:"main"`; state machine writes ONLY `type:"front"/"back"`.

### Step 4: Matrix-marker node (mirrors ArUco node's ROS surface)
- New `aruco_slam/matrix_slam_node.py`, class `MatrixSLAMNode`:
  - Reuses `FieldMapper` + ArUco corner detection for the 4 corners (same as ArUco node).
  - On start: receives the `name→(hue,fill)` map (param from the webserver; static config for dev),
    then `assign_markers(...)` publishes each robot's matrix marker and builds the reverse
    `(hue,fill)→name` lookup for detection.
  - Owns a `MatrixMarkerDetector`; loads the color model from config.
  - Publishes pose on the neutral `/localization/<name_safe>/position`, plus diagnostics
    `/aruco_slam/calibration_status`, `/aruco_slam/camera_feed`, `/aruco_slam/all_markers`
    (diagnostic topics may keep the `aruco_slam` prefix or move to `matrix_slam`; pose is the contract).
  - Params: `camera_id`, `field_width_cm`(300), `field_height_cm`(200), `corner_marker_ids`([4,5,6,7]),
    `color_model_path`, `robot_table`, `publish_rate_hz`, `show_visualization`, `drive_aim_leds`,
    blob/fill thresholds, camera-lock params.

### Step 5: Migrate existing publishers + consumers to the neutral `/localization/...` topic
- **ArUco node (`aruco_slam_node.py:90`):** change publish topic `/aruco_slam/<name_safe>/position` →
  `/localization/<name_safe>/position`. Message/frame/units already match. Tracking logic untouched.
- **UWB node (`ble_position_node.py:152-158`):** add `tag_names` param (`tag_id→name`); publish
  ONLY `/localization/<name_safe>/position` (the `/uwb/tag_<id>/position` topic is fully retired);
  set `frame_id='field'`.
- **Device controller (`sphero_instance_device_controller_node.py:118`):** change subscribed topic to
  `/localization/<self.topic_name_safe>/position`. One-line; callback unchanged.
- **FleetNode (`multirobot_webapp.py:62-93`):** subscribe `/localization/<name_safe>/position`
  (keyed by name) instead of `/uwb/tag_<id>/position`. `_on_uwb` cm→m logic reused (rename to
  `_on_localization`). `tag_assignments_map` stays for tag→name bookkeeping the UWB node needs.
- Add a 1-line comment in each marking the shared "localization position contract."

### Step 6: Config
- New `aruco_slam/config/matrix_positioning.yaml`:
```yaml
matrix_slam_node:
  ros__parameters:
    camera_id: 0
    field_width_cm: 300.0
    field_height_cm: 200.0
    corner_marker_ids: [4, 5, 6, 7]
    publish_rate_hz: 10.0
    show_visualization: true
    drive_aim_leds: false
    camera: {lock_white_balance: true, wb_temperature: 4600, lock_exposure: true, exposure: 120, lock_autofocus: true}
    blob: {min_blob_area_px: 80, max_blob_area_px: 1200, ring_center_frac: 0.45, ring_contrast_thresh: 0.4}
    color_model_path: "config/color_model.yaml"
    # robot_table is DEV-ONLY fallback; in production the webserver passes the live name→(hue,fill)
    # map (allocated at robot-add time). Format when used standalone:
    robot_table:                       # name -> (hue, fill)
      - {name: SB-3660, hue: Red,    fill: filled}
      - {name: SB-74FB, hue: Red,    fill: ring}
      - {name: SB-3716, hue: Orange, fill: filled}
      - {name: SB-58EF, hue: Orange, fill: ring}
```
- Generated `aruco_slam/config/color_model.yaml` (written by calibrator): learned per-hue centroids.

### Step 7: setup.py
- Add console_scripts: `matrix_slam_node = aruco_slam.matrix_slam_node:main`,
  `color_calibrator = aruco_slam.color_calibrator:main`.
- Add `data_files` to install `config/*.yaml` into `share/aruco_slam/config`.

### Step 8: Three-way source selection (ArUco / Matrix / UWB) — simplified by the neutral topic
Exactly one source publishes `/localization/...` at a time; consumers never change subscriptions.
- **Marker allocation at robot-add (mirrors UWB tag pool):** add a 16-slot marker pool (8 hues ×
  {filled, ring}) in the webserver. In `add_sphero()` / `FleetNode.add_robot(...)`, allocate the next
  free marker alongside the UWB tag_id, store in `marker_assignments_map` (name↔marker), free on
  removal, expose via the REST API (e.g. `GET /api/markers`) like `/api/uwb/tags`.
- **Process layer (`SpheroInstanceManager` in the webserver):** add `start_matrix_slam(camera_id)` /
  `stop_matrix_slam()` mirroring the existing aruco start/stop. Add a `positioning_source` selector
  `'aruco' | 'matrix' | 'uwb'`; selecting one starts that source's process and stops the others
  (enforces the single-active-publisher rule). When starting Matrix, pass the current
  `name→(hue,fill)` map; when starting UWB, pass the current `tag_id→name` map.
- **Default + switching:** initial source is **Matrix**, set via a launch-arg/param
  (`positioning_source`, default `'matrix'`) and overridable at runtime from the web UI. On startup the
  webserver brings up the Matrix source automatically.
- **Consumer layer:** NO subscription swapping needed. Device controller and FleetNode each subscribe
  once to `/localization/<name_safe>/position` and transparently follow whichever source is active.
  (FleetNode edit is just the topic change in Step 5 — may route to web_expert.)

### Step 9 (optional): standalone launch file
- New `aruco_slam/launch/matrix_positioning.launch.py` for dev/test. Webserver-managed launch stays prod.

## Files: reused vs modified vs new
**Reused UNCHANGED:** `aruco_slam/field_mapper.py`, `aruco_slam/aruco_marker.py`; Sphero
`set_matrix`/`set_led` + `matrix_patterns.py` (consumed via existing `sphero/<name>/matrix` topic).
**Modified (neutral-topic migration touches more files):**
- `aruco_slam/aruco_detector.py` — capture extraction (surgical).
- `aruco_slam/aruco_slam_node.py` — publish topic → `/localization/...` (contract only; tracking same).
- `aruco_slam/setup.py` — 2 entry points + config data_files.
- `sphero_uwb_positioning/.../ble_position_node.py` — `tag_names` map, publish `/localization/...`,
  `frame_id='field'`.
- `sphero_instance_controller/.../sphero_instance_device_controller_node.py` — subscribe `/localization/...`.
- `multirobot_webserver/.../multirobot_webapp.py` — FleetNode subscribe `/localization/...`; matrix
  start/stop; 3-way `positioning_source` selector; pass `tag_id→name` to UWB on start.
**New:** `aruco_slam/camera_capture.py`, `matrix_marker.py`, `matrix_detector.py`, `matrix_slam_node.py`,
`color_calibrator.py`, `config/matrix_positioning.yaml`, `config/color_model.yaml` (generated),
(optional) `launch/matrix_positioning.launch.py`.

## Calibration / operating workflow
1. **Field corners:** place ArUco corner markers (IDs 4,5,6,7); homography auto-calibrates as today.
2. **Camera locks** applied on startup (WB + low exposure to preserve hue, no bloom).
3. **Hue sampling:** `ros2 run aruco_slam color_calibrator` — robots cycle hues, tool learns
   emissive-hue centroids, writes `color_model.yaml`, warns if non-separable.
4. **Assign + run:** `matrix_slam_node` publishes each robot's (hue, fill) marker, then tracks. Verify
   on `/aruco_slam/camera_feed` and `ros2 topic echo /localization/<name_safe>/position`.

## Testing Plan
- Build: `colcon build --packages-select aruco_slam` then `source install/setup.bash`.
- ArUco regression: existing node still tracks; now publishes on `/localization/<name_safe>/position`.
- Neutral-topic wiring: device controller + FleetNode receive pose on `/localization/...` from each
  source in turn; UWB re-keyed by name (echo `/localization/<name_safe>/position` while UWB active).
- Corner calibration: `/aruco_slam/calibration_status` reports CALIBRATED.
- **Hue separability (CV-risk):** all 8 emissive hues classify correctly under locked exposure.
- **Filled-vs-ring readability (CV-risk):** confirm a filled and a ring marker are distinguished at the
  planned camera height (~16 px). If not, fall back to fewer-but-larger IDs, higher-res camera, or
  smaller field — log any reduced capacity, no silent caps.
- Single robot: echo its position; move to a known field point; check x,y cm.
- 16-robot scale: 16 distinct topics at ~10 Hz; per-blob area ≥ `min_blob_area_px`.
- Source switch: cycle aruco/matrix/uwb; only one source publishes `/localization/...` at a time;
  device controllers + dashboard follow transparently across all three with no subscription change.

## Risks & Considerations
- **Filled-vs-ring at ~16 px** is the main CV risk — validate early; levers above if it fails.
- **Hue washout from bloom** — mitigated by low locked exposure; the calibrator's separability guard
  catches palette collisions.
- **Matrix reserved for ID** while tracking → cannot show stimulus/app content simultaneously
  (esp. HRI study). Research-design constraint — confirm acceptable.
- **Yaw rotation** handled by using only rotation-invariant primitives (hue + concentric fill).
- **LED ownership by channel:** matrix node writes only `main` (hue boost); state machine writes only
  `front`/`back` (behavioral blinking). No topic contention. Detector never depends on any LED — the
  matrix is the authoritative ID (matrix-only fallback is the invariant).
- **Pi performance:** bright-blob threshold + CC at 1080p/10 Hz is lighter than v1's multi-mask;
  measure, downscale/ROI if needed.
- **Single-publisher constraint:** with the neutral topic, all three sources are mutually exclusive
  (only one publishes `/localization/...` at a time); the source selector enforces it. UWB can no
  longer co-run alongside a camera source — intended.
- **UWB tag→name coupling:** the dynamic `tag_id→name` map lives in the webserver; the UWB node now
  depends on receiving it. If UWB is launched standalone (not via webserver), it needs a static
  `tag_names` param or it cannot build `/localization/<name>/position`.
- **Frame-id unification:** UWB currently `frame_id='sphero_arena'`; standardize to `'field'`. Confirm
  nothing downstream relies on the old frame string.

## Open Questions (please confirm)
1. ~~**Matrix-for-stimulus conflict:**~~ **RESOLVED** — confirmed acceptable that the matrix is
   reserved for the ID marker while matrix-tracking is active (no simultaneous stimulus content).
2. ~~**Robot name table:**~~ **RESOLVED** — markers are NOT a static table; the webserver allocates a
   `(hue,fill)` marker dynamically when a robot is added to the fleet (mirrors UWB tag_id allocation),
   and passes the `name→(hue,fill)` map to the matrix node on start. Static config table is dev-only.
3. ~~**UWB tag→name source:**~~ **RESOLVED** (same pattern as Q2) — the webserver passes the live
   `tag_id→name` map to the UWB node on start; a static `tag_names` param serves standalone/dev launches.
4. ~~**Default source + switching:**~~ **RESOLVED** — default startup source is **Matrix**; switching
   is available **both** ways: a launch-arg / param sets the initial `positioning_source`, and the
   web UI can change it at runtime.
5. ~~**Aim-LED signal boost:**~~ **RESOLVED** — YES, aim LEDs default to each robot's hue as a color
   boost. Caveat captured: aim LEDs are SHARED with behavioral identity (blink fast/slow); the matrix
   stays authoritative for ID and tracking never depends on aim LEDs.
7. ~~**Aim-LED ownership/coordination:**~~ **RESOLVED** — split by LED channel on `sphero/<name>/led`:
   matrix node writes only `type:"main"` (hue boost); state-machine layer writes only
   `type:"front"/"back"` (behavioral blinking). No contention.
6. ~~**`/uwb/tag_<id>/position` retirement:**~~ **RESOLVED** — retire it fully. The UWB node publishes
   ONLY `/localization/<name_safe>/position` (name-keyed). No parallel tag-keyed topic.

## Approval Status — all 7 open questions resolved
1. Matrix reserved for ID during tracking — OK. 2. Markers allocated dynamically at fleet-add
(webserver), passed to matrix node. 3. UWB tag→name passed by webserver on start (+ static dev param).
4. Default source = Matrix; switch via launch-arg/param + web UI. 5. Main-LED hue boost ON. 6.
`/uwb/tag_<id>/position` retired; UWB publishes only `/localization/...`. 7. LED ownership split by
channel — matrix node `main`, state machine `front`/`back`.

- [x] ~~Waiting for user approval~~
- [x] **Approved** (2026-05-29)
- [x] **Executed** (2026-05-29) — ros2_expert built the vision pipeline + topic migration;
  web_expert wired the webserver selector + marker pool + UI. All 4 packages build. Pre-existing
  ArUco executable-name bug (`aruco_slam_node.py` → `aruco_slam_node`) fixed in the selector path.
  NOT YET runtime-tested with a live camera (headless Pi). State-machine blink behavior on front/back
  LEDs is intentionally out of scope (channel reserved).

## Interface Contract (shared — both SME agents build against this; do not diverge)
- **Position topic (all sources):** `/localization/<name_safe>/position`, `geometry_msgs/PoseStamped`,
  `header.frame_id='field'`, `pose.position.{x,y}` in cm, z=0, identity orientation.
  `name_safe = name.replace('-','_')`.
- **Matrix node:** executable `matrix_slam_node` (`ros2 run aruco_slam matrix_slam_node`). Key params:
  - `camera_id` (int, default 0)
  - `marker_assignments` (string JSON): `[{"name":"SB-3660","hue":"Red","fill":"filled"}, ...]`.
    Empty → fall back to dev `robot_table` from config yaml.
  - `color_model_path`, `field_width_cm`, `field_height_cm`, `corner_marker_ids`, `publish_rate_hz`,
    `show_visualization`, `drive_main_led` (bool, default true), camera-lock + blob/fill threshold params.
- **Hue palette (8, fixed names; emit RGB for matrix + main LED; calibrator refines observed centroids):**
  Red(255,0,0), Orange(255,80,0), Yellow(255,255,0), Green(0,255,0), Cyan(0,255,255), Blue(0,0,255),
  Magenta(255,0,255), Purple(140,0,255).
- **Fill:** `filled` = `[1]*64`; `ring` = `'circle'` pattern.
- **Marker pool allocation order (webserver):** filled across all 8 hues first (slots 0–7), then ring
  across all 8 hues (slots 8–15): slot0=(Red,filled) … slot7=(Purple,filled), slot8=(Red,ring) …
  slot15=(Purple,ring). Maximizes hue diversity for small fleets.
- **Main-LED boost:** matrix node publishes `sphero/<name_safe>/led` `type:"main"` at the robot's hue;
  NEVER writes `front`/`back` (state machine owns those).
- **UWB node:** publishes ONLY `/localization/<name_safe>/position`; params: existing `tag_ids` (int[])
  + new aligned `tag_names` (string[]) → builds `tag_id→name`; `frame_id='field'`.
- **Webserver:** `marker_assignments_map` (name↔marker) allocated in `add_sphero`/`add_robot` from the
  16-slot pool; `GET /api/markers`; `positioning_source` selector (default `'matrix'`) with
  `start_matrix_slam`/`stop_matrix_slam` mutually exclusive with aruco/uwb; on start pass
  `marker_assignments` JSON to the matrix node and `tag_ids`+`tag_names` to the UWB node.
</content>
</invoke>
