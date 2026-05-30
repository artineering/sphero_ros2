# Matrix Source: Blue-Tape Arena Boundary + Single Moderate Exposure

**Created:** 2026-05-29T16:48:32Z
**Status:** Executed

## Task Description
Make the `aruco_slam` matrix positioning source stop depending on ArUco corner
markers (IDs 4,5,6,7) for field calibration. Instead detect a blue-tape rectangle
as the arena boundary and use its 4 ordered corners for the homography. Also fix
the too-dark camera exposure (currently locked at 120 -> frame mean ~18) by moving
to a single moderate exposure, and dim the matrix/main-LED brightness so hues do
not bloom to white at the brighter exposure. ONLY the matrix node is touched; the
standalone ArUco source is left untouched.

## Analysis
- `matrix_slam_node.py` currently instantiates `ArucoDetector(camera_id=...)` as
  `self.corner_detector`, calls `detect_markers`/`get_last_tag_centers` each frame
  for calibration AND for corner-exclusion circles in `_field_mask`.
- `FieldMapper.calibrate(Dict[int, center])` reads `self.corner_marker_ids` in
  order and calls `getPerspectiveTransform`. We will add
  `calibrate_from_corners(ordered_corners)` that takes the 4 ordered pixel corners
  directly. `camera_to_field` / `field_to_camera` / `field_corners` stay unchanged.
- `_field_mask` already fills the calibrated quad to 255 (restrict detection to
  the play area). We keep that; we drop the ArUco corner-exclusion circles. The
  blue boundary band is a large thin outline, well outside the matrix blob area
  band (min 80 / max 1200 px) and outside the saturated bright-blob V threshold
  reasoning, so it should not register as a robot blob. To be safe we erode the
  field mask inward by a small margin so the boundary band itself is excluded.
- `camera_capture.py` exposure default 120 is far too dark. Measured: manual
  exposure ~1000 -> mean ~118. Choose a fixed manual exposure of 1000 (keep
  `lock_exposure=True`, manual mode), exposed via the `exposure` param. Manual is
  preferred over auto for stable color classification.
- `matrix_marker.py` renders hue patterns at full 255 palette RGB. At brighter
  exposure these bloom to white. Add a brightness scale (default 0.4) applied to
  the palette RGB in `marker_to_matrix_cmd`, and apply the same scale to the
  main-LED color boost in `matrix_slam_node._assign_markers`.

## Detailed Plan

### Step 1: FieldMapper - add calibrate_from_corners
- File: `src/aruco_slam/aruco_slam/field_mapper.py`
- Action: Add method `calibrate_from_corners(self, ordered_corners)` taking a list
  of 4 (x,y) pixel points already ordered TL, TR, BR, BL. It builds
  `camera_corners = np.float32(ordered_corners)` and runs the same two
  `getPerspectiveTransform` calls + sets `self.calibrated = True`. Leave existing
  `calibrate()` untouched (still used by the standalone ArUco node).
- Also update `get_calibration_status_text()`? It references corner_marker_ids in
  the NOT-CALIBRATED branch. Leave it; the matrix node will pass its own status.
  (Minimal change: matrix node logs its own "arena boundary not found" message.)
- Expected outcome: FieldMapper can calibrate from a raw ordered quad.

### Step 2: New module - blue-tape boundary detector
- File (new): `src/aruco_slam/aruco_slam/boundary_detector.py`
- Action: Implement `BoundaryDetector` with params `(blue_lower_hsv,
  blue_upper_hsv, min_area_frac)`. Method `detect(frame) -> Optional[ndarray(4,2)]`:
  - HSV threshold for blue tape, morphology close to join the tape outline.
  - findContours (external), pick the largest by area; reject if area <
    `min_area_frac * frame_area`.
  - `approxPolyDP` with epsilon = 0.02 * arcLength; require exactly 4 vertices,
    else return None.
  - Order the 4 points TL, TR, BR, BL via sum/diff ordering.
  - Return float32 (4,2) ordered corners, else None.
- Add a module-level `order_corners(pts)` helper (sum/diff method).
- Expected outcome: returns 4 sensible ordered corners for a blue rectangle.

### Step 3: matrix_slam_node - swap ArUco corners for boundary detector
- File: `src/aruco_slam/aruco_slam/matrix_slam_node.py`
- Actions:
  - Remove `from .aruco_detector import ArucoDetector` import.
  - Add `from .boundary_detector import BoundaryDetector`.
  - Remove `corner_marker_ids` param usage from the matrix path. Keep declaring
    `field_width_cm`/`field_height_cm`. Drop `corner_marker_ids` declaration and
    the `corner_ids` plumbing into FieldMapper (FieldMapper keeps its default).
  - Add params: `blue_lower_hsv` (default [100,80,40]), `blue_upper_hsv`
    (default [130,255,255]), `boundary_min_area_frac` (default 0.1).
  - Replace `self.corner_detector = ArucoDetector(...)` with
    `self.boundary_detector = BoundaryDetector(lower, upper, min_area_frac)`.
  - In `process_frame`: remove `detect_markers`/`get_last_tag_centers`. When not
    calibrated, call `self.boundary_detector.detect(frame)`; if it returns 4
    corners, call `self.field_mapper.calibrate_from_corners(corners)` and log
    success, else log (throttled) "arena boundary not found (blue tape)".
  - `_field_mask`: keep filling the calibrated quad; remove the ArUco
    corner-exclusion circles. Erode the quad mask inward by a small margin
    (e.g. 15 px) so the blue boundary band is excluded from robot detection.
  - Update startup log: drop "Corner markers:" line, add a "Arena boundary:
    blue-tape detection" line.
  - Calibration status publish: publish "CALIBRATED ..." when calibrated else
    "NOT CALIBRATED - waiting for blue-tape arena boundary".
- Expected outcome: no ArUco usage in matrix node; calibration via blue tape.

### Step 4: camera_capture - moderate exposure default
- File: `src/aruco_slam/aruco_slam/camera_capture.py`
- Action: Change `exposure` default 120.0 -> 1000.0; update the docstring lines
  that say "LOW default ... preserving hue" to reflect a moderate exposure tuned
  for both tape and markers. Manual-exposure mode (CAP_PROP_AUTO_EXPOSURE=1) kept.
- Expected outcome: frame mean ~100-120 in this room.

### Step 5: Dim LED brightness
- File: `src/aruco_slam/aruco_slam/matrix_marker.py`
- Action: Add `brightness_scale: float = 0.4` param to `marker_to_matrix_cmd` and
  thread it through `assign_markers`. Scale red/green/blue by it (int, clamped).
- File: `src/aruco_slam/aruco_slam/matrix_slam_node.py`
- Action: Add param `led_brightness_scale` (default 0.4). Pass into
  `assign_markers`/`marker_to_matrix_cmd`. Apply the same scale to the main-LED
  RGB in `_assign_markers`.
- Expected outcome: markers/LEDs render dim; hue preserved, no white bloom.

### Step 6: Update config yaml + node param docs
- File: `src/aruco_slam/config/matrix_positioning.yaml`
- Action: Update `exposure: 120.0` -> `1000.0`; remove `corner_marker_ids`; add
  `blue_lower_hsv`, `blue_upper_hsv`, `boundary_min_area_frac`,
  `led_brightness_scale`. Update the low-exposure comment.
- Expected outcome: config matches new params; tunable without code edits.

### Step 7: Build, source, smoke test, synthetic-frame unit check
- Commands:
  - `colcon build --packages-select aruco_slam`
  - `source install/setup.bash`
  - Synthetic-frame python check of `BoundaryDetector` (blue rectangle -> 4
    ordered corners).
  - `fuser /dev/video0` then headless run:
    `ros2 run aruco_slam matrix_slam_node --ros-args -p camera_id:=0
     -p show_visualization:=false -p drive_main_led:=false
     -p "marker_assignments:='[{\"name\":\"SB-3660\",\"hue\":\"Red\",\"fill\":\"filled\"}]'"`
    for a few seconds; confirm opens camera, loops, logs "arena boundary not
    found" (not an ArUco warning), no crash.
- Expected outcome: clean build; boundary unit check passes; node runs headless.

## Expected Outcomes
- Matrix node calibrates from a blue-tape rectangle, no ArUco dependency.
- Camera renders usable brightness; markers/LEDs dimmed to preserve hue.
- New tunable params for blue HSV range, min boundary area fraction, LED scale.
- Standalone ArUco source untouched. Topic contract unchanged.

## Potential Risks & Considerations
- Blue tape vs Blue robot hue: boundary picked by largest-quad geometry + min
  area fraction; robot blobs excluded by area band + eroded mask margin. A Blue
  robot inside the arena is still detected (small filled/ring blob inside mask).
- Exposure 1000 is a starting estimate from the user's measurement; final value
  retunable via the `exposure` param.
- HSV blue range may need tuning for the specific tape; exposed as params.

## Testing Plan
- colcon build clean.
- Synthetic blue-rectangle frame -> BoundaryDetector returns 4 ordered corners.
- Headless node run on /dev/video0: opens, loops, sensible "boundary not found"
  log, no crash.
- Physical validation (user step): lay blue tape rectangle spanning most of the
  camera view; place a Sphero showing its assigned (hue,fill); confirm calibration
  and pose on /localization/<name_safe>/position.

## Approval Status
- [x] Waiting for user approval
- [x] Approved
- [x] Executed
