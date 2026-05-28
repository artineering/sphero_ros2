# BLE Receiver: UWB Trilateration + Kalman Filter

**Created:** 2026-05-23T16:02:37Z
**Status:** Pending Approval

---

## Task Description

Update `ble_position_node.py` so it (1) correctly parses the 9-byte UWB distance
payload the tag actually transmits, (2) runs 2D least-squares trilateration to
produce (x_cm, y_cm) from the anchor distances, and (3) applies a per-tag
constant-velocity Kalman filter to smooth the position before publishing.

---

## Analysis

### Arduino BLE payload (from UWB_MulticastTag.ino)

The tag writes an 11-byte `mfgData` buffer:

```
mfgData[0]  = 0xFF   ─┐ Company ID 0xFFFF (LE)
mfgData[1]  = 0xFF   ─┘
mfgData[2]  = TAG_ID
mfgData[3..4]   distance A0 (uint16 LE, 0xFFFF = invalid)
mfgData[5..6]   distance A1 (uint16 LE, 0xFFFF = invalid)
mfgData[7..8]   distance A2 (uint16 LE, 0xFFFF = invalid)
mfgData[9..10]  distance A3 (uint16 LE, 0xFFFF = invalid)
```

Bleak puts the company ID (0xFFFF) as the dict key and the *remaining* bytes as
the value. So `payload = mfg[0xFFFF]` is **9 bytes**:

```
payload[0]     TAG_ID
payload[1..2]  dist_A0 (uint16 LE)
payload[3..4]  dist_A1 (uint16 LE)
payload[5..6]  dist_A2 (uint16 LE)
payload[7..8]  dist_A3 (uint16 LE)
```

Current code treats payload as 5 bytes `[tag_id, x_cm_lo, x_cm_hi, y_cm_lo,
y_cm_hi]` — **wrong**.

### Current node structure (kept intact)

- `BlePositionNode.__init__` — declare params, create pubs/timers, start scanner
- `_scanner_main` / `_detection_callback` — BLE receive path
- `_fake_callback` — synthetic data generator
- `_publish_callback` — timer-driven publisher
- `_diagnostics_callback` — heartbeat diagnostics
- `TagSample` dataclass — shared state between detection and publish paths

---

## Detailed Plan

### Step 1 — Fix payload constants and parser

**Files:** `ble_position_node.py`

**Changes:**

- Replace `PAYLOAD_LEN = 5` with `PAYLOAD_LEN = 9`.
- Replace `INVALID_DIST = ??? ` (currently implicit) with `INVALID_DIST = 0xFFFF`.
- In `_detection_callback`, replace the two-line unpack:
  ```python
  x_cm, y_cm = struct.unpack("<hh", payload[1:5])
  ```
  with:
  ```python
  raw_dists = struct.unpack("<HHHH", payload[1:9])
  # raw_dists[i] is distance_cm to anchor i, or 0xFFFF if invalid
  ```
- Store `raw_dists` in `TagSample` (see Step 2 for dataclass change).

**Expected outcome:** Node parses the correct 9-byte payload; unknown/invalid
distances are preserved as `0xFFFF` sentinel values.

---

### Step 2 — Update TagSample dataclass

**Files:** `ble_position_node.py`

**Changes:**

Replace:
```python
@dataclass
class TagSample:
    tag_id: int
    x_cm: int
    y_cm: int
    rssi_dbm: int
    ble_address: str
    last_seen_monotonic: float
```

With:
```python
@dataclass
class TagSample:
    tag_id: int
    raw_dists: tuple            # (d0, d1, d2, d3) in cm; 0xFFFF = invalid
    x_cm: float                 # filtered position (set after KF update)
    y_cm: float
    rssi_dbm: int
    ble_address: str
    last_seen_monotonic: float
```

`x_cm` and `y_cm` remain as the published position — now they hold the Kalman
filtered estimate. `raw_dists` is stored for potential future logging/debug but
is not published directly.

---

### Step 3 — Declare new ROS2 parameters

**Files:** `ble_position_node.py`, `__init__` method only

**New parameters:**

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `anchor_positions_cm` | `list[float]` (8 values) | `[0.0, 0.0, 300.0, 0.0, 300.0, 300.0, 0.0, 300.0]` | Flat list `[x0,y0, x1,y1, x2,y2, x3,y3]` in cm. Corners of a 300×300 cm square — **placeholder; set real values in launch file.** |
| `process_noise_std_cm` | `float` | `5.0` | KF process noise std dev in cm/s (tunable). |
| `measurement_noise_std_cm` | `float` | `15.0` | KF measurement noise std dev in cm (tunable; reflects cm-scale UWB trilateration error). |

Parse `anchor_positions_cm` into a `(4, 2)` numpy array during `__init__`:
```python
ap = list(self.get_parameter("anchor_positions_cm").value)
self._anchors = np.array(ap).reshape(4, 2)  # shape (4, 2)
```

---

### Step 4 — Add 2D trilateration function

**Files:** `ble_position_node.py` — module-level function (not a method)

**Math sketch:**

Given N ≥ 3 anchors with known positions `(xi, yi)` and measured ranges `ri`,
the standard least-squares approach subtracts the last equation from all others
to linearize the quadratic system.

For the i-th anchor relative to anchor N:

```
(xi - xN)·x + (yi - yN)·y  =  0.5·(ri² - rN² - xi² + xN² - yi² + yN²)
                                                         ^-- all known constants
```

Stacking M = N-1 such equations gives `A x = b`, where A is (M×2) and b is
(M×1). Solve with `np.linalg.lstsq(A, b, rcond=None)`. This handles exactly-3
(unique solution) and 4-anchor cases (over-determined, minimum-residual).

**Assumptions:**
- Anchors and tag are all at z = 0 (planar). Distances are treated as 2D ranges
  even though UWB gives 3D slant ranges. For a flat arena this is acceptable;
  if significant height differences are introduced later, a z correction term
  must be added.

**Signature:**
```python
def _trilaterate(anchors: np.ndarray, dists: np.ndarray) -> tuple[float, float] | None:
    """anchors: (N, 2) float array; dists: (N,) float array.
    Returns (x_cm, y_cm) or None if solution fails."""
```

Returns `None` if fewer than 3 valid anchors are provided.

---

### Step 5 — Add per-tag Kalman filter class

**Files:** `ble_position_node.py` — small class defined at module level

**State vector:** `[x, y, vx, vy]` (cm and cm/s)

**Constant-velocity model (dt in seconds):**

```
F = [[1, 0, dt,  0],
     [0, 1,  0, dt],
     [0, 0,  1,  0],
     [0, 0,  0,  1]]
```

**Measurement matrix:**
```
H = [[1, 0, 0, 0],
     [0, 1, 0, 0]]
```

**Process noise Q** (discrete, additive, tuned by `process_noise_std_cm` σ_q):
```
Q = σ_q² · [[dt⁴/4, 0, dt³/2, 0],
             [0, dt⁴/4, 0, dt³/2],
             [dt³/2, 0, dt², 0],
             [0, dt³/2, 0, dt²]]
```
This is the standard piecewise-constant white-noise acceleration model.

**Measurement noise R:**
```
R = σ_r² · I₂
```
where σ_r = `measurement_noise_std_cm`.

**Defaults rationale:** σ_q = 5 cm/s (moderate dynamics, Sphero moves slowly),
σ_r = 15 cm (conservative UWB trilateration error at ~4 Hz update rate). Both
are exposed as ROS2 params for easy tuning.

**Class interface:**
```python
class TagKalmanFilter:
    def __init__(self, q_std: float, r_std: float) -> None: ...
    def update(self, z_x: float, z_y: float, dt: float) -> tuple[float, float]: ...
    # update() runs predict then correct; returns (x_est, y_est)
    # First call initializes state from measurement (no prediction step)
```

All math uses numpy — no external filter libraries.

---

### Step 6 — Wire trilateration and KF into `_detection_callback`

**Files:** `ble_position_node.py`

**Logic:**

```
parse raw_dists from payload (Step 1)
build valid_anchors, valid_dists arrays (drop 0xFFFF entries)
if len(valid_anchors) < 3: return   # keep previous estimate unchanged
(x_raw, y_raw) = _trilaterate(valid_anchors, valid_dists)
if (x_raw, y_raw) is None: return
dt = now - last_seen_monotonic of previous sample (or 0.25 as first-step default)
kf = self._kfilters.setdefault(tag_id, TagKalmanFilter(q_std, r_std))
x_filt, y_filt = kf.update(x_raw, y_raw, dt)
sample = TagSample(tag_id, raw_dists, x_filt, y_filt, rssi, addr, now_mono)
self._samples[tag_id] = sample
```

`self._kfilters: dict[int, TagKalmanFilter]` is initialized as `{}` in `__init__`.
`self._prev_mono: dict[int, float]` tracks the previous sample time for dt.

`q_std` and `r_std` are read once in `__init__` and stored as `self._q_std`,
`self._r_std`.

---

### Step 7 — Update `_fake_callback` (minimal change)

**Current behaviour:** generates `(x, y)` directly via sin/cos.

**Simplest fix (chosen):** keep the sin/cos position generator, but create a
`TagSample` with `raw_dists=(0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF)` (all invalid) and
**bypass the trilateration+KF pipeline** — set `x_cm`/`y_cm` directly from the
synthetic position. This is simpler than synthesizing fake distances and feeding
them through trilateration, and fake_mode exists only for UI/publish testing,
not for filter tuning.

A one-line comment will note this bypass.

---

### Step 8 — Update `_diagnostics_callback` KeyValues

**Files:** `ble_position_node.py`

Add `raw_dists` to the diagnostic key-value list so anchor distances are visible
in `ros2 topic echo /diagnostics`:

```
KeyValue(key="dist_A0_cm", value=str(sample.raw_dists[0]))
KeyValue(key="dist_A1_cm", value=str(sample.raw_dists[1]))
KeyValue(key="dist_A2_cm", value=str(sample.raw_dists[2]))
KeyValue(key="dist_A3_cm", value=str(sample.raw_dists[3]))
```

Existing `x_cm`, `y_cm` keys remain — they now reflect filtered position.

---

## New Parameters Summary

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `anchor_positions_cm` | `double_array` (8) | `[0.0,0.0, 300.0,0.0, 300.0,300.0, 0.0,300.0]` | Anchor XY positions in cm. Order: A0, A1, A2, A3. **Must be set to real values before use.** |
| `process_noise_std_cm` | `double` | `5.0` | KF process noise σ_q in cm/s. Higher = trust measurements more. |
| `measurement_noise_std_cm` | `double` | `15.0` | KF measurement noise σ_r in cm. Higher = smoother but laggier. |

---

## Trilateration Math (Least-Squares)

Given N valid anchors at `(xᵢ, yᵢ)` with measured distances `rᵢ`, each
satisfies `(x−xᵢ)² + (y−yᵢ)² = rᵢ²`. Expanding and subtracting the equation
for anchor N (reference) from each i < N removes the quadratic terms in x, y,
yielding the linear system `A·[x,y]ᵀ = b` where `Aᵢ = 2[xᵢ−xN, yᵢ−yN]` and
`bᵢ = rN²−rᵢ²+xᵢ²−xN²+yᵢ²−yN²`. With N=3 this is a unique 2×2 solve; with
N=4 it is over-determined and `np.linalg.lstsq` minimizes the residual in the
least-squares sense.

---

## Kalman Filter Summary

| Item | Value |
|------|-------|
| State | `[x, y, vx, vy]` (cm, cm/s) |
| Transition F | constant-velocity, parameterized by dt |
| Measurement H | `[[1,0,0,0],[0,1,0,0]]` — observe x, y only |
| Process noise Q | piecewise-white-noise acceleration model, σ_q = 5 cm/s (param) |
| Measurement noise R | `σ_r²·I₂`, σ_r = 15 cm (param) |
| Init P | `1000·I₄` (large uncertainty until first update) |
| Init state | first measurement, zero velocity |

---

## Expected Outcomes

- `ros2 topic echo /sphero/<name>/uwb/position` shows smoothed, physically
  plausible x/y positions.
- `ros2 topic echo /diagnostics` shows per-anchor distances alongside x/y.
- `fake_mode=true` continues to produce circular motion on the topic (KF
  bypassed for synthetic data).
- With fewer than 3 valid anchors in a frame, no position update is emitted
  (previous KF state is retained until next valid trilateration).

---

## Potential Risks & Considerations

1. **Planar assumption:** UWB distances are slant ranges. If the tag or anchors
   are at different heights, horizontal position error grows. Documented as a
   known assumption; no Z correction added now.
2. **First dt:** When a tag is first seen, there is no `prev_mono`. Use
   `dt = 0.25 s` (1/4 Hz) as a safe first-step default.
3. **KF divergence:** If the tag is stationary for a long time then suddenly
   moves, the velocity state may lag. σ_q can be raised in the launch file to
   increase responsiveness.
4. **Thread safety:** `_kfilters` and `_prev_mono` are accessed only inside
   `_detection_callback`, which runs in the BLE scanner thread. The KF state
   is consumed (already filtered) before storing in `TagSample` and then locked
   behind `_lock` — no additional locking needed for these dicts.
5. **numpy import:** Added at top of file. No new packages; numpy is a ROS2
   transitive dependency.

---

## Verification Steps

```bash
# 1. Build
cd ~/sphero_ros2
colcon build --packages-select sphero_uwb_positioning
source install/setup.bash

# 2. Run in fake_mode (no hardware needed)
ros2 run sphero_uwb_positioning ble_position_node \
  --ros-args \
  -p fake_mode:=true \
  -p fake_tag_ids:="[1,2]" \
  -p tag_ids:="[1,2]" \
  -p sphero_names:="[SB-3660,SB-74FB]"

# 3. Verify topic output
ros2 topic echo /sphero/SB_3660/uwb/position

# 4. Verify diagnostics (check dist_A0_cm..A3_cm keys)
ros2 topic echo /diagnostics

# 5. Run with real BLE hardware; set real anchor positions
ros2 run sphero_uwb_positioning ble_position_node \
  --ros-args \
  -p anchor_positions_cm:="[0.0,0.0,300.0,0.0,300.0,300.0,0.0,300.0]" \
  -p measurement_noise_std_cm:=15.0 \
  -p process_noise_std_cm:=5.0

# 6. Check trilateration residual (optional) — add a debug log line in
#    _detection_callback printing the lstsq residual norm when ≥ 4 anchors
#    are valid.
```

---

## Approval Status

- [x] Waiting for user approval
- [x] Approved
- [x] Executed (2026-05-23)
