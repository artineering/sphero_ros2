# Anchor 4 Hz Ranging Rate

**Created:** 2026-05-23T16:30:00Z
**Status:** Pending Approval
**Hardware:** Arduino Portenta C33 + Portenta UWB Shield (SR150)
**Complexity:** Low

## Task Description

Change `rangingDuration` in `UWB_MulticastAnchor.ino` from 500 to 250 to deliver 4 position updates per second to each Stella tag.

## Analysis

### Units of `rangingDuration`

Confirmed from library source (`src/uwbapps/UWBAppParamList.hpp` in the Truesense PortentaUWBShield repo):

```cpp
/**
 * @brief Set the Ranging Duration parameter, in milliseconds
 * @param duration, default value is 200
 */
bool rangingDuration(uint32_t duration);
```

`rangingDuration` is in **milliseconds**. It sets the period of one complete ranging round (the full cycle time between successive callbacks for the same session).

### Current value → current rate

`rangingDuration(500)` → 500 ms per round → **2 Hz** per tag.

### Target value

250 ms per round → **4 Hz** per tag.

Each tag belongs to exactly one session (Group 0 or Group 1), so the per-tag update rate equals the per-session round rate. No cross-session arithmetic needed.

### Physical floor check

The library defaults `slotPerRR = 25` slots at 1200 µs/slot (FiRa SP3 DS-TWR standard slot duration):

```
25 slots × 1200 µs = 30 ms minimum physical cycle time (6 controlees)
```

250 ms is ~8× above the physical floor, so no risk of the stack rejecting the value or under-running.

## Detailed Plan

### Step 1: Edit one line in the anchor sketch

**File:** `arduino/UWB_MulticastAnchor/UWB_MulticastAnchor.ino`  
**Line 159** (inside the session-creation loop, applies to both sessions):

```cpp
// Before
sessions[g]->appParams.rangingDuration(500);

// After
sessions[g]->appParams.rangingDuration(250);
```

No other changes.

## Caveats

- **Both sessions change together.** The same loop body applies to both multicast sessions (g=0 and g=1), so both get 250 ms. This is correct — both groups should run at 4 Hz.
- **UWB radio duty cycle doubles.** Going from 500 ms to 250 ms cycles the radio twice as often; power draw at the anchor increases slightly, but anchors are mains-powered so this is not a concern.
- **Tag battery impact is neutral.** The tag radio wakes only to respond to the anchor's poll. The tag's on-air time per transaction is unchanged; the interval between transactions halves. Battery impact at the tag is modest (roughly proportional to the increase in radio wake events, estimated <10% additional draw for nRF52840).
- **STALE_TIMEOUT_MS on the tag is 2500 ms.** The tag firmware already declares a measurement stale after 2500 ms. At 4 Hz a measurement is refreshed every 250 ms, well within that window.

## Testing

### 1. Compile

```bash
/home/svaghela/sphero_ros2/bin/arduino-cli compile \
  --fqbn arduino:renesas_portenta:portenta_c33 \
  /home/svaghela/sphero_ros2/arduino/UWB_MulticastAnchor
```

Expected: no errors, flash usage similar to before.

### 2. Upload

```bash
/home/svaghela/sphero_ros2/bin/arduino-cli compile --upload \
  --fqbn arduino:renesas_portenta:portenta_c33 \
  -p /dev/ttyACM0 \
  /home/svaghela/sphero_ros2/arduino/UWB_MulticastAnchor
```

Repeat for each anchor, changing `-p` to the appropriate port.

### 3. Verify rate on the anchor serial monitor

```bash
/home/svaghela/sphero_ros2/bin/arduino-cli monitor \
  -p /dev/ttyACM0 -c baudrate=115200
```

With at least one tag in range, ranging lines (`Tag ID: X | Distance(cm): ...`) should print at ~4 Hz per tag (one line per 250 ms per tag).

### 4. Verify rate on the tag serial monitor

Flash a tag with `SERIAL_DEBUG 1` and connect via:

```bash
/home/svaghela/sphero_ros2/bin/arduino-cli monitor \
  -p /dev/ttyACM1 -c baudrate=115200
```

After 5 seconds the tag prints a STATS line:

```
STATS | ranging=4.0/s  bleAdv=...
```

`ranging` should read ~4.0/s (one callback per anchor per second × 4 anchors ≈ 16 total/s if all four anchors are active; or 4.0/s if only one anchor is present).

## Approval Status

- [x] Waiting for user approval
- [x] Approved
- [x] Executed
