# Overhead tracking — algorithm design

How `overhead_tracking` turns a USB3 global-shutter camera into per-Sphero field
positions and headings. Companion to the code in `src/overhead_tracking/`; the
prototypes it was ported from are `scripts/blob_detect_timing.py`,
`scripts/kf_replay.py` and `scripts/goto_target.py` (results in
`doc/overhead_tracking_prototyping.md`).

**Output contract (do not change):** `/localization/<name_safe>/position`,
`geometry_msgs/PoseStamped`, `frame_id='field'`, position in **centimetres**
(origin bottom-left, +x along the long edge), published every processed frame
(`publish_rate_hz: 0`). Orientation is the EKF yaw (CCW from +x) once the robot's
LEDs gave a heading, identity before. Consumed by the device and task controllers,
the webserver and Foxglove. Never run alongside `kinect_field_tracking` (same topic).

---

## 1. Camera

| | |
|---|---|
| Mode | 1920x1200 MJPG at 60 fps (`camera_fps`), the full field of view |
| Controls | manual exposure (clamped to the 16.6 ms frame period), `gain 120`, AWB off, `exposure_dynamic_framerate=0`; applied 1.5 s after streaming starts and on every reopen (UVC discards them at stream start) |
| Robot size | ball ~34 px, LED pair ~26 px apart, blob ~700-900 px |

- **1280x720 is a 1:1 centre crop of 1920x1200, not a downscale** (measured: scale 1.00,
  offset 316,236). It discards 60 % of the arena for no extra detail.
- **`CAP_PROP_FPS` must be set**: OpenCV's V4L2 backend otherwise runs at 30 fps, which
  moves the exposure clamp. `~/state` reports the driver rate (sets the clamp) and the
  consume rate separately; only the driver rate drives the warning.
- **Capture keeps the raw MJPEG bytes** (`CAP_PROP_CONVERT_RGB=0`), copied once per
  frame, and decodes them to **gray only** on the grab thread
  (`imdecode(IMREAD_GRAYSCALE)`, ~6-7 ms; a full colour decode is ~22 ms, more than
  the frame budget). Each frame is a `Frame(seq, stamp, gray, jpeg)`: a sequence
  number unique per frame, its capture stamp, the gray image and the bytes.

Sources (`camera.py`, duck-typed, so the node is the same in all three):
`v4l2` (the camera), `replay` (a recorded `raw.mjpeg` + `frames.csv` stamps +
optional `cmds.csv` motion commands; realtime or lockstep) and `sim` (synthetic
gray frames, no colour).

---

## 2. Pipeline

```
 grab thread      Frame(seq, stamp, gray, jpeg) ──► wait_frame()
                         │ (publish_raw_jpeg: ~/camera/compressed, every frame)
 processing thread       ▼  60 Hz: every frame, 30 Hz: every 2nd by seq
   ColorWorker.submit(seq, jpeg) ──────────────► colour thread: half-res colour
   blobs.detect(gray, floor map)       ~5 ms     decode (+ HSV), ~7-8 ms
   px -> cm (homography)                               │
   ColorWorker.get(seq)  ◄──────── same frame only ────┘  (late -> no LED fix)
   LED pair heading for every UNSPLIT blob
   TrackManager.step(stamp, blobs)  (EKF per robot, gate, merges, ladder)
   publish poses, ~/detections, heading offsets
```

All timing uses frame stamps, never tick counts, so 60 and 30 Hz behave the same.
Services and subscriptions run on the ROS executor; what they share with the
processing thread is guarded by `_tracks_lock`.

**Detection** (`blobs.py`, `floor_map.py`). A robot is a blob brighter than
`floor_ratio` (3.2) x the LOCAL floor: the arena reads ~19 mid and ~8 in the
corners, robots are 3.4-5.6x their local floor, the blue tape <= 2.75x and floor
noise <= 2.5x. The floor map is a 61 px morphological opening of still frames
(robots and tape vanish), so per frame detection is one compare inside the arena
polygon grown by `blob_roi_margin_px`, connected components, fragments joined
within `blob_link_px`, and merged blobs (>= 1.6x the link-time ball area) split by
k-means. The map is built by `~/detect_arena`, `~/rebuild_floor`, lazily on the
first frame, and in the background when the in-arena floor level drifts more than
`floor_drift_frac` for `floor_drift_hold_sec`. No reference-frame subtraction.

**LED heading** (`color_decode.py`, `heading.py`). Robots run front green / back
red, main panel black. The colour worker decodes the frame's own bytes at half
resolution (`IMREAD_REDUCED_COLOR_2`, libjpeg DCT scaling) on its own thread while
the gray blobs are found, and converts it to HSV once (`hsv`) or
returns BGR (`led_color_space: bgr`, the default: converted per LED patch, identical headings, faster). Results are keyed
by frame seq and only returned for that frame, so a blob is never paired with
another frame's colour; a frame whose colour is not ready in 10 ms gets no LED
reading (`diag.color_late` counts them) and the EKF carries theta. Heading =
bearing red -> green, only with both LEDs found and at least half the nominal
separation apart. A crowded blob gets none: split, or another blob's centre
within `led_search_px` + `point_sep_px` / 2 on both axes, close enough for the
other robot's LEDs to fall in this blob's search window (`diag.led_crowded` counts them; touching
unsplit blobs swapped identities on `patrol_20261104_163022` without it).

**Filter** (`ekf.py`). Per robot an EKF on `[x, y, theta]`, driven by the robot's
`sphero/<ns>/motion_cmd` echoes (the only command feed: it covers roll, stop and
`/fleet/policy`): speed applied `lag_v_sec` (0.45 s) late, turns from CHANGES of
the commanded heading at <= `turn_rate_dps`, so the unknown aim offset cancels.
Until the first LED fix theta is unknown: no speed input, larger position noise,
identity orientation. `HeadingOffsetEstimator` learns `offset = field + cmd`
(Sphero headings are clockwise) from settled LED readings and publishes it on
`/localization/<ns>/heading_offset` (Float32 deg, latched, on > 1 deg change, <= 1 Hz);
reset on re-link, `reset_aim` and BLE reconnect.

**Association** (`track_manager.py`). One `TrackManager` over all robots; the
track set is fixed at link time. Per frame: predict to the frame stamp; gate on
`gate_sigma` Mahalanobis of the EKF covariance, clamped to
[`gate_min_cm`, `gate_max_cm`]; greedy claim on negative log-likelihood; crossing
pairs resolved by the cheaper joint assignment with the LED heading as the
tie-breaker (an unresolvable crossing marks both SUSPECT, sticky until re-link);
an unsplit blob >= `merge_area_frac` x ball area holding two robots makes both
MERGED (coast, clamped to one ball diameter); ladder in seconds HEALTHY ->
COASTING -> LOST (`lost_after_s`) -> UNRESOLVED (`unresolved_after_s`);
reacquire only on a mutually unambiguous nearest match.

**Publish.** Every processed frame: the pose extrapolated by `v * frame age`
(live only; lockstep replay stamps run behind the wall clock), `~/detections`
(JSON per robot: u, v, x/y cm, heading, area, split, status, misses, frame stamp),
`~/track_markers`. `~/state` (1 Hz) carries the camera stats, floor map, tracks and
`diag` (`processed_fps`, `process_ms` p50/p95 over the last 600 frames,
`detect_ms`, `track_ms`, `color_late`, `led_crowded`).

---

## 3. Operator workflow and linking

```
INIT -> IDLE/ARENA_READY -> ~/detect_spheros -> BLOBS_READY -> ~/link_spheros
     -> LINKING -> TRACKING            (~/reset -> ARENA_READY, arena kept)
```

- `~/detect_arena` is refused while TRACKING (a new homography teleports every
  track). Corners are manual (`manual_corners_px`) or tape hue (`arena_source: auto`).
- `~/detect_spheros` snapshots the blobs (biggest first, <= `max_robots`).
- `~/link_spheros` binds blobs to callsigns (requested, else fresh heartbeats):
  - `link_method: probe` — serial LED lit-diff: all dark, light one robot's white
    main panel, the blob that brightened is it, dark again, next.
  - `link_method: nudge` — identity by motion: roll one robot at `nudge_speed`
    (>= 50) for `nudge_duration_sec` (1.0 s; 0.5 s lost the race with the ~1 s
    BLE start latency) toward free space, stop, the one blob that moved is it.
    One robot at a time, no LED writes while moving. On a replay the recorded
    nudges in `cmds.csv` are watched instead.
  - `link_seed_csv` (replay only) — each robot takes the blob nearest its first
    row of a reference `track.csv`.
  - After every robot is done: tracking LEDs on linked robots (main black, which
    blanks the BOLT matrix, front green, back red), failed ones dim red. A linked
    robot whose `sphero/<ns>/status` goes back to `connected` gets the tracking
    LEDs again (the device controller redraws its arrow matrix on connect).
- No camera step while LINKING (probes flash robots, nudges move them); tracks
  catch up or reacquire on the first TRACKING frame.

---

## 4. Threads

| Thread | Work |
|---|---|
| Grab (source) | read MJPEG bytes, copy once, gray decode, `Frame` |
| Processing | wait for the next frame, decimate, detect, LED, TrackManager, publish |
| Colour worker | half-res colour decode (+ HSV) of the submitted frame |
| Floor rebuild | background map build on drift |
| Annotate worker | overlay on the gray frame, JPEG encode, `~/annotated/compressed` |
| Executor | services (MutuallyExclusive), subscriptions (Reentrant), 1 Hz state |

- `cv2.setNumThreads(1)`: OpenCV's own parallel-for would oversubscribe the cores.
- Frames are immutable fresh arrays: consumers hold references, the floor map and
  annotate queue keep views without copying.
- Per-robot publishers/subscriptions are created only from service threads.
- A lockstep replay is advanced by whoever pulls: the processing loop pulls only
  while a robot is tracked, so the frames before linking go to the services.
- 60 fps budget 16.6 ms. Measured numbers: see the step reports in `plans/`.

---

## 5. Parameters (non-obvious ones)

| Param | Default | Meaning |
|---|---|---|
| `track_rate_hz` | 60 | 60 = every frame, 30 = every second; anything else warns -> 60 |
| `publish_rate_hz` | 0 | 0 = every processed frame |
| `led_color_space` | bgr | colour worker output, `hsv` or `bgr` |
| `publish_raw_jpeg` | false | camera bytes on `~/camera/compressed` (full video in a bag) |
| `floor_ratio` | 3.2 | blob threshold x local floor |
| `gate_sigma` / `gate_min_cm` / `gate_max_cm` | 3 / 15 / 50 | association gate |
| `lost_after_s` / `unresolved_after_s` | 0.5 / 2.0 | miss ladder, frame time |
| `lag_v_sec` / `turn_rate_dps` / `cmd_speed_to_cms` | 0.45 / 180 / 0.61 | command model |
| `link_method` | probe | `probe` or `nudge` (no automatic fallback) |
| `nudge_speed` / `nudge_duration_sec` | 50 / 1.0 | nudge linking |
| `replay_path` / `replay_lockstep` / `replay_rate` / `replay_loop` | | `source: replay` |
| `link_seed_csv` | '' | replay only: seed links from a track.csv |

Full list with comments: `config/overhead_tracking.yaml`.

---

## 6. Interface summary

**Services** — `~/detect_arena`, `~/detect_spheros`, `~/rebuild_floor`, `~/reset`,
`~/reapply_camera_controls`, `~/save_annotated` (`std_srvs/Trigger`),
`~/link_spheros` (`multirobot_msgs/srv/Register`).

**Published** — `/localization/<ns>/position` (PoseStamped, cm),
`/localization/<ns>/heading_offset` (Float32 deg, latched), `~/detections`,
`~/state`, `~/link_status` (String JSON; the last two latched),
`~/annotated/compressed`, `~/camera/compressed` (opt-in, frame stamp),
`~/arena_corners`, `~/track_markers` (metres),
`sphero/<ns>/{led,matrix,heading,reset_aim,calibrate_compass,roll}` (live only;
a replay never publishes to `/sphero/*`).

**Subscribed** — `/sphero_fleet/robots`, `sphero/<ns>/motion_cmd`,
`sphero/<ns>/status`, `sphero/<ns>/reset_aim`, `sphero/<ns>/calibrate_compass_done`.
