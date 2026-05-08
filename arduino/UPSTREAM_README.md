# Arduino UWB Positioning

UWB real-time location system (RTLS) firmware for **4 anchors x 12 tags** using multicast Two-Way Ranging (TWR) sessions.

## Hardware

| Device | Board | UWB Module | Role |
|--------|-------|------------|------|
| Anchor | Arduino Portenta C33 + UWB Shield | Truesense DCU150 (SR150) | Controller (one-to-many) |
| Tag | Arduino Stella | Truesense DCU040 (nRF52840) | Responder (controlee) |

## Sketches

| File | Description |
|------|-------------|
| `UWB_MulticastAnchor.ino` | Anchor firmware — runs 2 multicast sessions (6 tags each), prints range measurements to Serial |
| `UWB_MulticastTag.ino` | Tag firmware — joins 4 sessions (one per anchor), computes 2D position, broadcasts via BLE |

## Addressing Scheme

- **Anchor MAC:** `{0xA0, ANCHOR_ID}` where ANCHOR_ID is 0-3
- **Tag MAC:** `{0x00, TAG_ID}` where TAG_ID is 1-12
- **Session ID:** `ANCHOR_ID * 100 + GROUP_INDEX + 1`

Tags are split into two groups:
- Group 0: Tags 1-6
- Group 1: Tags 7-12

## Position Computation

The tag computes its own 2D position using **linearized least-squares multilateration**.

```
     A3 (0,305)                          A2 (305,305)
      o─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─o
      |  ╲                             ╱ |
      |    ╲  d3                   d2 ╱  |
      |      ╲                     ╱     |
      |        ╲                 ╱       |
      |          ╲             ╱         |
      |            ╲    T    ╱           |
      |              ╲  *  ╱             |
      |                ╲╱                |
      |              ╱    ╲              |
      |            ╱        ╲            |
      |       d0 ╱            ╲ d1       |
      |        /                \        |
      |      ╱                    ╲      |
      |    ╱                        ╲    |
      |  ╱                            ╲  |
      |/                                \|
      o─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ o
     A0 (0,0)                            A1 (305,0)

     Each anchor measures slant distance dᵢ to the tag.
     The intersection of the 4 circles gives the tag position.
     Least-squares finds the best fit when circles don't
     intersect perfectly due to measurement noise.
```

### How it works

Each anchor *i* at known position (xᵢ, yᵢ) gives a circle equation from the measured horizontal distance dᵢ:

```
(x - xᵢ)² + (y - yᵢ)² = dᵢ²
```

This is nonlinear due to the x² and y² terms. The linearization trick is to subtract a reference anchor's equation from each of the others — the quadratic terms cancel:

```
2·(xᵢ - x₀)·x + 2·(yᵢ - y₀)·y = (xᵢ² - x₀²) + (yᵢ² - y₀²) - (dᵢ² - d₀²)
```

With 4 anchors this produces 3 linear equations for 2 unknowns (an overdetermined system **Ax = b**), solved via normal equations **(AᵀA)⁻¹ Aᵀb** with direct 2×2 matrix inversion.

### Height correction

Before multilateration, slant ranges are projected onto the ground plane:

```
       Anchor (25 cm)
         o
         |╲
   Δz    | ╲  d_slant (measured by UWB)
  20 cm  |  ╲
         |   ╲
  ·──────+────*── ground plane
              Tag (5 cm)
         |←─────→|
         d_horizontal (used for 2D position)

  d_horizontal = √(d_slant² - Δz²)
```

where Δz = 25 cm (anchor antenna at ~10 in) − 5 cm (tag at ~2 in) = 20 cm. Without this correction, ranges are systematically too large and the position drifts outward.

### Why least-squares

With exactly 3 anchors you get a unique solution. With 4, the circles won't intersect perfectly due to UWB measurement noise. Least-squares minimizes total squared error across all equations, giving a more robust estimate. The determinant check guards against degenerate (collinear anchor) configurations.

The final position is rounded to **10 cm** — the ranging resolution of the DCU040 module.

## BLE Position Advertising

Each tag broadcasts its computed position over BLE advertising (no pairing required):

- **Local name:** `UWB-T<id>` (e.g., `UWB-T1`)
- **Manufacturer data (7 bytes):**

| Byte | Content |
|------|---------|
| 0-1 | Company ID `0xFFFF` (BLE SIG test/dev, little-endian) |
| 2 | TAG_ID |
| 3-4 | X position in cm (`int16`, little-endian) |
| 5-6 | Y position in cm (`int16`, little-endian) |

The advertisement is only updated when the position changes to minimize BLE churn.

## Configuration

### Anchor

Set `ANCHOR_ID` (0-3) in `UWB_MulticastAnchor.ino`:

```cpp
#define ANCHOR_ID  0   // 0, 1, 2, or 3
```

### Tag

Set `TAG_ID` (1-12) and anchor positions in `UWB_MulticastTag.ino`:

```cpp
#define TAG_ID  1   // 1 through 12

// Anchor positions (cm) — 10 ft x 10 ft square, corners
#define ANCHOR_X  {   0.0f, 305.0f, 305.0f,   0.0f }
#define ANCHOR_Y  {   0.0f,   0.0f, 305.0f, 305.0f }

// Heights (cm): anchor antenna ~10 in, tag ~2 in
#define ANCHOR_HEIGHT_CM  25.0f
#define TAG_HEIGHT_CM      5.0f
```

## Flashing

1. Flash each Portenta C33 with `UWB_MulticastAnchor.ino`, changing `ANCHOR_ID` for each (0-3)
2. Flash each Stella with `UWB_MulticastTag.ino`, changing `TAG_ID` for each (1-12)
3. Power on **anchors first**, then tags — anchors drive the ranging schedule

## Session Budget

- 2 sessions per anchor (limit: 5 on SR150)
- 4 sessions per tag (limit: 5 on DCU040)

## License

Apache 2.0 — see [LICENSE](LICENSE).
