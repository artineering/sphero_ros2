# OpenCV Color-Based Positioning as a Third Source (ArUco kept intact)

**Created:** 2026-05-29T00:31:07Z
**Status:** Pending Approval

## Task Description
Add an OpenCV color-based positioning pipeline as a **third** selectable positioning source,
alongside the existing **ArUco** robot-tracking and **UWB/BLE** sources. ArUco robot tracking is
NOT replaced. Robot identity for color = (body color, accent color) pair from
{Black, Orange, White, BlueGray} (4x4 = 16). Position only (centroid x,y in field cm). Reuse the
existing field-corner ArUco calibration homography (`FieldMapper`). Critically: make sure BOTH
ArUco tracking and color tracking emit the same position-message interface so the three sources
(ArUco / Color / UWB) are interchangeable downstream with no consumer changes.

## Analysis

### The position-message interface to standardize on (verified)
- `aruco_slam_node.py` publishes per robot: `/aruco_slam/<name_safe>/position`,
  type `geometry_msgs/PoseStamped`, `header.frame_id='field'`, `pose.position.{x,y}` in **field cm**,
  z=0, identity orientation.
- Consumer `sphero_instance_device_controller_node.py` (line 120) subscribes to
  `/aruco_slam/<topic_name_safe>/position` (PoseStamped) when `external_location` is set.
- Webserver `FleetNode` reads pose ONLY from `/uwb/tag_<id>/position` (`_on_uwb`,
  multirobot_webapp.py 74-79, 139). It does NOT read the aruco topic today. So the dashboard
  currently shows UWB pose only; the device-controller path is the one wired to aruco.

### Decision on the shared topic for vision sources
ArUco and Color are both **camera/field-cm** sources and both already belong conceptually to the
`/aruco_slam/...` namespace and the device-controller consumer. To make them interchangeable with
zero downstream change, **the color node publishes the IDENTICAL topic name, type, frame, and
units** as the ArUco node: `/aruco_slam/<name_safe>/position`, PoseStamped, frame 'field', cm.
- This means ArUco and Color must NOT run simultaneously on the same topic (they would both publish
  to `/aruco_slam/<name>/position`). Source selection ensures only one camera source runs at a time.
- UWB stays on its own `/uwb/tag_<id>/position` topic, unchanged.

This is the minimal-change path and directly satisfies "make sure both aruco and color tracking fit
properly with the position messages" — both produce the exact same PoseStamped-in-field-cm contract.
(See Open Question 1 for an optional cleaner alternative: a neutral `/localization/<name>/position`
topic that all three sources publish and all consumers read. Recommended longer-term but a bigger
change; default plan keeps `/aruco_slam/...` to avoid touching the device controller.)

### Color separability decision: use **Lab**, not HSV
Colors are Black, Orange, White, BlueGray. Three (Black/White/BlueGray) are achromatic — HSV Hue is
unstable for them; they differ mainly in Value. Lab separates lightness (L) from chroma (a,b):
Orange is high-chroma and trivially separated; Black/White/BlueGray separate along L with near-zero
a,b. Lab's perceptual uniformity makes learned per-color centroids (nearest-centroid in Lab) robust
under mild lighting variation, especially with AWB/exposure locked. HSV kept only as an optional
Orange-chroma fallback test.

### Reusable assets
- `FieldMapper.camera_to_field()` / `camera_to_field_batch()` — REUSE UNCHANGED (shared by both
  ArUco and Color nodes).
- ArUco corner detection (`ArucoDetector` + `get_last_tag_centers`) — used by BOTH nodes for the 4
  corner markers. ArUco node ALSO keeps using it for robot markers (unchanged).
- Camera capture currently in `ArucoDetector` sets 1080p but does not lock WB/exposure. We extract
  capture into a shared module so both nodes share it and add the CV-risk camera locks there.

## Detailed Plan

### Recommendation: new files INSIDE the existing `aruco_slam` package (not a new package)
Both vision sources reuse `FieldMapper` + corner ArUco detection and publish the `/aruco_slam/...`
topics. Keeping color in-package = local homography reuse, identical topic namespace (zero
downstream change), one extra console_script. A new package would duplicate the homography or add a
cross-package dep for no benefit. ArUco files stay where they are, untouched except a surgical
capture extraction.

### Step 1: Shared camera capture module
- New file: `aruco_slam/camera_capture.py`, class `CameraCapture`:
  - Wraps `cv2.VideoCapture(camera_id, cv2.CAP_V4L)` at 1920x1080.
  - Adds configurable camera locks (default ON): `CAP_PROP_AUTO_WB=0` + fixed
    `CAP_PROP_WB_TEMPERATURE`; `CAP_PROP_AUTO_EXPOSURE=1` (manual) + fixed `CAP_PROP_EXPOSURE`;
    optional `CAP_PROP_AUTOFOCUS=0`. Essential for stable Lab values (achromatic mitigation).
  - Methods: `open()`, `read()->frame|None`, `release()`.
- Modify `aruco_slam/aruco_detector.py` to use `CameraCapture` for capture (surgical: capture only;
  ArUco detection untouched). ArUco node keeps working unchanged and now benefits from the locks.

### Step 2: Color calibration / sampling module
- New file: `aruco_slam/color_calibrator.py` (+ console entry point `color_calibrator`):
  - Operator clicks/drags a region per color (Black, Orange, White, BlueGray) on a live frame; the
    tool collects Lab pixels, fits centroid+radius per color, writes `config/color_model.yaml`.
  - Separability guard: warns if any two centroids overlap within their radii (achromatic-confusion
    check), prompting lighting/exposure adjustment.

### Step 3: Color detector (the new robot-ID front-end)
- New file: `aruco_slam/color_detector.py`, class `ColorDetector`. Algorithm per frame, in order:
  1. **Capture** frame (locked WB/exposure).
  2. **Preprocess:** BGR->Lab; light blur; mask out pixels outside the calibrated field quad
     (using homography corners) to reject background; mask out the 4 corner-marker regions.
  3. **Segment per color:** nearest-centroid classification in Lab -> 4 binary masks (each pixel to
     its closest learned color centroid within radius). Morphological open/close.
  4. **Body-blob extraction:** connected-components per body-color mask -> candidate donut regions;
     filter by `min/max_body_area_px`.
  5. **Accent assignment (blob-merge mitigation):**
     - Find accent-color sub-blobs within/overlapping each body region.
     - Two-tone: body region with exactly one differing-color accent sub-blob -> ID=(body,accent),
       position = centroid of the whole robot region (body+accent), not the accent alone.
     - Solid (accent==body): uniform donut, no differing accent sub-blob -> ID=(body,body).
     - Merged same-base-color robots: when N same-base bodies touch into one CC, do NOT trust the
       body CC count; count accent sub-blobs inside the merged component (including the solid/donut
       case), assign each to nearest enclosing body pixels, take per-partition centroid. This
       segments by accent within the merged body blob.
     - Solid-vs-two-tone rule: inside a body region, look for a contiguous accent-color sub-region
       >= `min_accent_area_frac` of the ring; if none and ring color variance is low -> solid.
  6. **ID lookup:** (body,accent) -> robot name via config table.
  7. **Centroid:** pixel centroid of each identified robot region.
  8. **camera_to_field:** `FieldMapper.camera_to_field_batch()` -> field cm.
  9. **Publish** per-robot PoseStamped on `/aruco_slam/<name_safe>/position` (identical contract).
     Dedupe duplicate IDs (keep larger/more-confident, log warning).

### Step 4: Color node (mirrors ArUco node's ROS surface)
- New file: `aruco_slam/color_slam_node.py`, class `ColorSLAMNode`:
  - Reuses `FieldMapper` + ArUco corner detection for the 4 corners (same as ArUco node).
  - Owns a `ColorDetector`; loads (body,accent)->name table and color model from config.
  - Publishes the SAME topics as the ArUco node: `/aruco_slam/<name_safe>/position` (PoseStamped,
    field cm), `/aruco_slam/calibration_status` (String), `/aruco_slam/camera_feed` (Image),
    `/aruco_slam/all_markers` (String).
  - Params: `camera_id`, `field_width_cm` (300), `field_height_cm` (200),
    `corner_marker_ids` ([4,5,6,7]), `color_model_path`, `robot_table`, `publish_rate_hz`,
    `show_visualization`, camera-lock params.

### Step 5: Make ArUco node conform to the shared contract (verification, minimal/no change)
- Audit `aruco_slam_node.py` against the standardized contract: topic name pattern, PoseStamped,
  frame 'field', cm, z=0, identity orientation. It already matches. Action: confirm and document;
  add a 1-line comment marking it as the shared "vision position contract" so the color node and any
  future source stay aligned. No behavioral change to ArUco tracking.

### Step 6: Config
- New `aruco_slam/config/color_positioning.yaml`:
```yaml
color_slam_node:
  ros__parameters:
    camera_id: 0
    field_width_cm: 300.0
    field_height_cm: 200.0
    corner_marker_ids: [4, 5, 6, 7]
    publish_rate_hz: 10.0
    show_visualization: true
    camera:
      lock_white_balance: true
      wb_temperature: 4600
      lock_exposure: true
      exposure: 250
      lock_autofocus: true
    blob:
      min_body_area_px: 400
      max_body_area_px: 8000
      min_accent_area_frac: 0.10
    color_model_path: "config/color_model.yaml"
    robot_table:                       # 16-entry (body,accent)->name
      - {body: Black,  accent: Black,  name: SB-3660}
      - {body: Black,  accent: Orange, name: SB-74FB}
      # ... 16 total
```
- Generated `aruco_slam/config/color_model.yaml` (written by calibrator):
```yaml
colors:
  Black:    {L: 28.0,  a: 128.0, b: 127.0, radius: 14.0}
  Orange:   {L: 62.0,  a: 160.0, b: 175.0, radius: 18.0}
  White:    {L: 235.0, a: 128.0, b: 128.0, radius: 12.0}
  BlueGray: {L: 140.0, a: 124.0, b: 118.0, radius: 12.0}
# OpenCV 8-bit Lab convention (L 0-255, a/b centered at 128)
```

### Step 7: setup.py
- Add console_scripts: `color_slam_node = aruco_slam.color_slam_node:main`,
  `color_calibrator = aruco_slam.color_calibrator:main`.
- Add `data_files` to install `config/*.yaml` into `share/aruco_slam/config`.

### Step 8: Three-way source selection (ArUco / Color / UWB)
Goal: three runtime-selectable sources; only one camera source (ArUco or Color) active at a time
since they share `/aruco_slam/...`; UWB independent.
- **Process layer (webserver `SpheroInstanceManager`):** it already starts/stops `aruco_slam` and
  the UWB BLE node. Add `start_color_slam(camera_id)` / `stop_color_slam()` mirroring the existing
  aruco start/stop. Add a `positioning_source` selector with values `'aruco' | 'color' | 'uwb'`.
  Selecting a source starts that source's process and stops the others (enforcing the
  one-camera-source rule).
- **Consumer layer:**
  - Device-controller path: unchanged — it consumes `/aruco_slam/<name>/position`, which BOTH ArUco
    and Color publish, so switching between ArUco and Color is transparent.
  - FleetNode (dashboard): today reads pose only from UWB. To reflect the selected source, add a
    `positioning_source` param + `set_positioning_source(src)` that swaps subscriptions
    (vision sources -> `/aruco_slam/<name_safe>/position`; uwb -> existing
    `/uwb/tag_<id>/position`), reusing the existing create/destroy-subscription pattern. UWB path
    left intact. (FleetNode edit may route to web_expert — Open Question 2.)

### Step 9 (optional): standalone launch file
- New `aruco_slam/launch/color_positioning.launch.py` for dev/test of the color node with config.
  Webserver-managed launching stays the production path. Recommend dev-only.

## Files: reused vs modified vs new

**Reused UNCHANGED:**
- `aruco_slam/field_mapper.py`, `aruco_slam/aruco_marker.py`.

**Modified:**
- `aruco_slam/aruco_detector.py` — extract capture into shared `CameraCapture` (surgical).
- `aruco_slam/aruco_slam_node.py` — verification + 1 comment marking the shared contract; no
  behavioral change (ArUco tracking preserved).
- `aruco_slam/setup.py` — 2 entry points + config data_files.
- `multirobot_webserver/.../multirobot_webapp.py` — add color start/stop, 3-way `positioning_source`,
  FleetNode subscription swap. (UWB and ArUco paths intact.)

**New:**
- `aruco_slam/camera_capture.py`, `color_calibrator.py`, `color_detector.py`, `color_slam_node.py`
- `aruco_slam/config/color_positioning.yaml`, `config/color_model.yaml` (generated)
- (optional) `aruco_slam/launch/color_positioning.launch.py`

**Contract confirmation:** ArUco and Color both publish `/aruco_slam/<name_safe>/position`,
PoseStamped, frame 'field', cm — identical. UWB stays on `/uwb/tag_<id>/position`. The
device-controller consumer is unchanged for all three; FleetNode gains a source-aware subscription.

## Calibration Workflow (user-facing)
1. Field corner calibration (reused): place ArUco corner markers (IDs 4,5,6,7); whichever vision
   node runs auto-calibrates the homography exactly as today.
2. Camera lock: color node applies WB/exposure/focus locks on startup for stable colors.
3. Per-session color sampling: `ros2 run aruco_slam color_calibrator`; click/drag a sample per color
   under current lighting; writes `color_model.yaml`; warns if colors aren't separable.
4. Verify: run `color_slam_node`; check IDs and field-cm positions on `/aruco_slam/camera_feed` and
   `ros2 topic echo /aruco_slam/<name>/position`.

## Testing Plan
- Build: `colcon build --packages-select aruco_slam` then `source install/setup.bash`.
- ArUco regression: run the existing ArUco node, confirm it still publishes positions unchanged.
- Corner calibration: `/aruco_slam/calibration_status` reports CALIBRATED.
- Color single robot: echo its position topic; move to a known field point; check x,y cm.
- Two-tone vs solid: confirm correct IDs for a solid and a two-tone robot.
- Blob-merge: push two same-base-color robots together; both still publish distinct positions.
- 16-robot scale (if available): 16 distinct topics at ~10 Hz.
- Source switch: cycle aruco/color/uwb at runtime; confirm only one camera source runs at a time,
  device controllers follow transparently across aruco<->color, dashboard pose follows the selected
  source, and UWB still works when selected.

## Potential Risks & Considerations
- Achromatic confusion (Black/White/BlueGray): mitigated by locked WB/exposure + learned Lab
  centroids + separability check; residual risk under glare/shadow.
- Solid-vs-two-tone depends on `min_accent_area_frac`; tune during calibration.
- Merged-blob accent partitioning is heuristic (nearest enclosing body); can mis-assign adjacent
  accents.
- 16 donuts on 300x200 at 1080p: verify per-robot pixel area meets `min_body_area_px` at the planned
  camera height.
- Performance on the Pi: per-frame multi-mask + CC at 1080p/10 Hz may need downscale/ROI; measure
  first.
- Shared-topic constraint: ArUco and Color must not run simultaneously (both publish
  `/aruco_slam/...`); the source selector enforces this.

## Open Questions (please confirm)
1. **Shared vs neutral topic:** default keeps both vision sources on `/aruco_slam/<name>/position`
   (zero device-controller change, but ArUco+Color can't co-run). Alternative: introduce a neutral
   `/localization/<name>/position` that all three sources publish and all consumers read (cleaner,
   future-proof, but touches the device controller + FleetNode). Keep shared `/aruco_slam` topic, or
   move to neutral topic?
2. **FleetNode change ownership:** to show selected-source pose on the dashboard, FleetNode (in
   `multirobot_webserver`) needs a source-aware subscription. Include that here, or split the web
   portion to web_expert? (Or is the device-controller path the only intended consumer, no dashboard
   change?)
3. **Robot name table:** I know only 4 names (SB-3660, SB-74FB, SB-3716, SB-58EF). Provide the full
   16 (body,accent)->name mapping, or confirm placeholders.
4. **"BlueGray":** confirm it is a distinct printed color (not shadowed white) — separability from
   White depends on it.
5. **Source priority/default:** with three sources, what is the default on startup (ArUco, Color, or
   UWB), and should switching be web-UI driven, launch-arg driven, or both?

## Approval Status
- [ ] Waiting for user approval
- [ ] Approved
- [ ] Executed
