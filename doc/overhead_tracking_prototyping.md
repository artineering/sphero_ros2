# Overhead tracking prototyping — 2026-10-05

Offline/standalone experiments run against the live overhead global-shutter
camera and the real fleet, ahead of changing the `overhead_tracking` node.
Everything here lives in `scripts/` and `recordings/`; the node itself was not
modified. Node design: [`overhead_tracking_algorithm.md`](overhead_tracking_algorithm.md).

Robot used for all driving tests: **SB-418F** (the only robot whose LED heading
was verified; SB-3660's LEDs never lit, SB-5B47 dropped its BLE link).

---

## 1. Recording and MATLAB export

- Camera: `/dev/video0` (USB3 global shutter), **1920x1200 MJPG @ 60 fps**,
  `auto_exposure=1`, `gain=120`, `white_balance_automatic=0`,
  `exposure_dynamic_framerate=0` — the `overhead_tracking.yaml` settings,
  applied after stream-on (UVC drops them otherwise).
- Recorded with `v4l2-ctl --stream-mmap --stream-to` (raw MJPEG, no re-encode):
  `recordings/overhead_20261005_145833.mjpeg`, 4304 frames, 71.7 s, steady
  59.9 fps. Auto-WB was on for the first ~30 s.
- `scripts/mjpeg2avi.py` remuxes the JPEGs byte-for-byte into an **MJPG AVI**,
  which MATLAB `VideoReader` reads on all platforms (4303 frames; last frame
  was truncated).
- Raw videos (94–302 MB each) are **not in git** (GitHub 100 MB limit, no LFS);
  they stay in `recordings/` on the Pi 5.

## 2. Arena

Corners from `~/overhead_field/overhead_arena.yaml` (calibrated 2026-09-28,
verified still aligned with the tape on 2026-10-05):

| Image corner | x px | y px |
|---|---|---|
| top-left | 318.5 | 60.0 |
| top-right | 1660.0 | 83.5 |
| bottom-right | 1680.5 | 1056.0 |
| bottom-left | 300.0 | 1065.0 |

Arena 243.84 x 182.88 cm (96 x 72 in). The homography `H` in the same file maps
pixels to field cm.

## 3. Blob detection (MATLAB pipeline ported to OpenCV)

`scripts/blob_detect_timing.py` — live camera or `--replay <raw.mjpeg>`.

MATLAB reference: `rgb2gray -> imbinarize -> bwconncomp(8) -> regionprops ->
inpolygon`.

| Version | Detect (mean) | Blobs | Notes |
|---|---|---|---|
| Straight port (BGR decode, Otsu, CC, centroid-in-polygon) | 13.6 ms | 9–19 per frame for 5 robots | 50.7 fps, drops frames |
| + fixed threshold 60, arena mask, min area 200 | 6.9 ms | exactly 5 (later 12) | dim corner robots missed |
| **Final** | **4.2–5.5 ms** | **12/12 in >99% of frames** | see below |

Final pipeline:
- **Grayscale decode**: the camera has no gray format (YUYV is 5 fps at full
  res), but `cv2.imdecode(..., IMREAD_GRAYSCALE)` on the raw MJPEG decodes only
  luma: ~6 ms, no colour conversion.
- **Per-pixel threshold = 3.2 x local floor**: lighting is uneven (floor ~19
  mid, ~8 corners). Against the local floor every robot is 3.4–5.6x, blue tape
  <= 2.75x, floor noise <= 2.5x. Floor map = morphological opening (61 px) of a
  still frame at startup.
- **Centroid decides arena membership** (like `inpolygon`), with 25 px slack —
  a robot against the wall has its centre on the calibrated edge.
- **`findContours` instead of full-frame labelling**: 0.9 ms vs 8.5 ms; area
  and centroid are exact pixel moments per blob (match `regionprops`).
- **Fragment joining**: a robot rolling dark-side-up breaks into sub-200 px
  pieces; each small piece joins its nearest piece within 25 px (big blobs never
  initiate a join, so two robots can't chain).
- **Touching-pair split**: blob >= 1.6x median area -> k-means into
  round(area/median) parts.

Rejected: Otsu inside the arena (lands at ~19, labels the floor); morphological
closing (doubles cost, causes false splits).

Replay results: slow motion run 1269/1270 frames correct, fast run 1466/1477.
Remaining misses: robot briefly dark-side-up at the dim top wall (3–4 frames).

## 4. Moving robots

`scripts/blob_motion_test.py` drives connected robots through a speed sweep
while detecting live and recording raw frames + per-blob CSV.

- Commands take **~1 s** to show up as motion.
- Motion blur never broke a blob up to ~10 px/frame (~1 m/s).
- Failures were all (a) robot clipped at the arena edge / dim, (b) touching
  robots, (c) fragmented dark-side-up robots — fixed in §3.

## 5. Kalman filter (EKF) test

`scripts/kf_record.py` records frames + `/sphero/<ns>/motion_cmd` echoes +
LED schedule; `scripts/kf_replay.py` runs the EKF offline (`--robots`, cached
`meas.pkl`).

Formulation (as specified): state `[px, py, theta]`, input `[v, omega]`,
measurement `[px, py, theta]` (theta from the LED pair, `heading.py`), angle
residual wrapped. v = speed x 0.61 cm/s (measured calibration); omega from
commanded-heading changes (aim offset cancels).

Results on SB-418F (41 s drive, 2471 frames, LED heading in 100% of frames):

| Filter | Lag v / omega | Innov pos | Innov heading | NIS | 0.5 s blind error (mean/p90) |
|---|---|---|---|---|---|
| As given | 0.45 / 0.45 s | 0.30 cm | 8.2° | 2.00 | 4.4 / 7.0 cm |
| omega lag 0 + 180°/s turn-rate limit | 0.45 / 0.00 s | 0.30 cm | 4.8° | 0.70 | 4.4 / 8.2 cm |
| No inputs | — | 0.63 cm | 5.0° | 0.75 | 16.6 / 21.9 cm |

- The speed input is what helps (blind error 16.6 -> 4.4 cm).
- LED heading = direction of travel (median difference 0.1°).
- With a measurement every frame the EKF just echoes the camera (0.001 cm);
  its value is prediction through dropouts.
- The fleet applies commands only ~every 0.5 s (10 Hz commands get queued).

## 6. Go-to-coordinate

`scripts/goto_target.py` (use `--fast`). Steps: find the robot by its lit
front (green) / back (red) LED pair -> position via homography, heading from
LEDs -> heading offset (command heading 0 at speed 0, read LED heading;
field = offset - cmd) -> drive -> stop -> optional correction pulses.

### Measured robot behaviour (SB-418F)

| Quantity | Value |
|---|---|
| Roll command -> motion | 0.6–1.1 s |
| Stop command applied (echo) | 0.7–1.15 s |
| Dead band | speed < 20 ignored; 25–36 unreliable (user) |
| Speed 30 cruise | 18.6–26.8 cm/s |
| Speed 50 cruise | 34.7–39.9 cm/s |
| Speed 100 | peaks ~78 cm/s; keeps accelerating 0.5–0.8 s after stop is sent |

### Stopping distance (stop sent -> at rest), 100 cm moves — `scripts/stop_sweep.py`

| Speed | Coast mean ± std | Best stop for 100 cm | Notes |
|---|---|---|---|
| 30 | 21.5 ± 2.2 cm | 78.5% | 5/12 trials never started moving |
| 50 | 42.9 ± 3.8 cm | 57% | all trials moved; ~±4 cm landing |
| 100 | 80.5 ± 5.8 cm | — | lands 86–111 cm whatever the stop point (5–21%) |

- Coast is a **fixed distance per speed**, not a fraction of the move.
- **LED re-assert writes during a move delay the stop** over BLE: coast std at
  speed 30 fell from 7.5 cm to 2.2 cm once LED writes were moved between trials.
- Per-trial heading recalibration doubled cross-track drift; one calibration
  per run is better.

### Current `--fast` algorithm

1. Clamp target one Sphero radius (3.65 cm) inside the walls.
2. Cruise at `--speed` (>= 50 enforced), one roll command.
3. Re-aim at 1/3 and 2/3 of the way if bearing error > 3° (one write each).
4. Send stop at **D − coast(speed)** (interpolated from the table above).
5. Abort if no progress for 2 s (lost link / stuck).
6. Correction pulses at speed 50, T = 0.6 s + error/gain, gain learned; stop
   pulsing after 2 pulses that move < 1 cm.

### Runs (5% of travel distance tolerance)

| Run | Move | Result | Time | Notes |
|---|---|---|---|---|
| 173306 | -> centre, speed 70 | FAIL 31.5 cm | 41.5 s | coast at speed 70 ~78 cm |
| 173623 | -> centre | PASS 1.45 cm | 27.9 s | 27 cm overshoot + 5 pulses |
| 175150 | -> start, "speed-0 brakes" idea | FAIL | 30 s | online offset refinement ran robot into a corner |
| 175336 | -> start | PASS 1.01 cm | 18.2 s | 4 of 5 pulses too short to move |
| 175804 | -> centre, 75% stop rule, speed 36 | PASS 0.66 cm | 10.2 s | one dead-time-aware pulse |
| 180740 | speed-dependent % rule, speed 50 | PASS 1.84 cm | 27.0 s | 31 cm overshoot; pulses bounce |
| 183241 | -> (0,0), measured coast | PASS 6.17 cm | 19.2 s | pinned in corner during pulses |
| 183435 | diagonal ~298 cm | PASS 14.1 cm | 10.4 s | no pulses |
| 183648 | short edge -> (243.84,0) | FAIL 13.7 cm | 19.8 s | grazed wall, slowed, stopped short |
| 183801 | -> (0,0) | PASS 11.3 cm | 11.4 s | slowed along wall |
| 184543 | -> centre (user frame) | PASS 4.35 cm | 20.0 s | speed-40 pulses overshoot |
| 184632 | -> top-left | FAIL | 40 s | BLE link died mid-run |
| 184907 | -> top-left | FAIL 14.7 cm | 6.8 s | short move: stopped before full speed; 0.98 s pulses never moved it |
| 185035 | -> centre | **PASS 5.36 cm** | **6.3 s** | single roll + stop, no pulses |

### Facing (`--face`)

Turns in place to point at a target and confirms with the LEDs; up to 3 tries.
Small in-place turns are not executed reliably (a 4.5° correction moved 0.3°),
so corrections swing 30° away and come back. Typical per-turn accuracy ±8°;
reached 0.5° (top-right) and 0.7° (bottom-left, 3rd try). The robot's heading
reference drifts 5–10° between runs, so every run re-measures it.

## 7. Conventions

- **User frame** (`goto_target.py` targets and printouts): origin at the
  **top-right** image corner, x leftward, y downward; (243.84, 182.88) is
  bottom-left. Headings: 0° = image-left, 90° = image-down.
- Internals, `log.csv`, the arena calibration and the tracker node stay in the
  calibration's **field frame** (origin bottom-left, x right, y up).
  user <-> field: `(243.84 - x, 182.88 - y)`.

## 8. Open issues / next steps

1. **Pulse dead time**: use ~1.0 s (measured start delay 0.8–1.1 s), not 0.6 s.
2. **Short moves**: the stop comes before full speed; base the stop on the
   measured speed (coast(v)) instead of the commanded speed.
3. **Wall stall rule** should only trigger when the robot is within a radius of
   a wall.
4. Stop timing jitter from BLE latency limits a single approach to ~±4–8 cm at
   speed 50; short moves with a 5% tolerance rely on pulses.
5. Port the final blob pipeline + grayscale-only decode into the
   `overhead_tracking` node (it currently decodes full BGR every frame and
   copies it for LED hue).
6. Decide whether the user-frame convention should apply fleet-wide (arena
   calibration / node) or stay in the scripts.

## 9. Files

Scripts (`scripts/`): `blob_detect_timing.py`, `blob_motion_test.py`,
`kf_record.py`, `kf_replay.py`, `goto_target.py`, `stop_sweep.py`,
`stop_probe.py`, `mjpeg2avi.py`.

Data (`recordings/`, raw `*.mjpeg`/`*.avi` excluded from git):
`motion_*` (blob motion tests), `kf_*` (EKF recordings: frames.csv, cmds.csv,
leds.csv, meas.pkl, kf_tracks.png), `stopsweep_*` (stop sweeps, per-frame
log.csv), `goto_*` (go-to runs: log.csv, result.jpg), `first_frame.jpg`.
