# Arduino Stella Tag - Deployment Guide

**Version:** 1.0
**Date:** 2026-03-07
**Application:** UWB positioning for Sphero robot tracking

## Overview

This guide covers the physical deployment of Arduino Stella UWB tags, including battery installation, mounting to Sphero robots, operational procedures, and troubleshooting.

## Hardware Components

### Per Tag Assembly
- **Arduino Stella board** (flashed with tag_firmware_optimized.ino)
- **LiPo battery** (3.7V nominal, 1000-2000 mAh recommended)
- **Battery connector** (compatible with Stella power input)
- **Mounting hardware** (velcro, zip ties, or 3D-printed bracket)
- **Optional:** Battery voltage divider circuit (if not built into Stella)
- **Optional:** Enclosure for protection

## Battery Selection

### Recommended Specifications

**Chemistry:** LiPo (Lithium Polymer)
**Voltage:** 3.7V nominal (4.2V max, 3.0V min)
**Capacity:**
- **Minimum:** 500 mAh (2-3 hours operation)
- **Recommended:** 1000 mAh (6-8 hours operation)
- **Extended:** 2000 mAh (12-16 hours operation)

**Discharge Rate:** 1C or higher (not critical for this application)
**Form Factor:** Flat pouch preferred for easy mounting
**Connector:** Match Stella board power input (verify from datasheet)

### Battery Life Estimates

Based on power analysis from plan:

| Battery Capacity | Expected Runtime | Notes |
|------------------|------------------|-------|
| 500 mAh | 2-3 hours | Minimal, not recommended |
| 1000 mAh | 6-8 hours | Good for half-day experiments |
| 1500 mAh | 9-12 hours | Full day operation |
| 2000 mAh | 12-16 hours | Extended experiments |

**Assumptions:**
- Average current draw: ~150 mA with 50% duty cycle
- Sleep mode enabled (default in firmware)
- LED indicators enabled
- Serial debug disabled in production

**Power Saving Tips:**
- Disable serial debug: `#define ENABLE_SERIAL_DEBUG false`
- Reduce LED brightness: `#define LED_BRIGHTNESS 16`
- Increase ranging interval: `#define RANGING_INTERVAL_MS 500`

## Battery Installation

### Voltage Divider Circuit (If Required)

If Arduino Stella does not have built-in battery voltage sensing:

**Circuit:**
```
Battery+ ----[R1: 10kΩ]----+---- ADC Pin (A0)
                           |
                         [R2: 10kΩ]
                           |
                          GND
```

**Calculation:**
- Voltage divider ratio: (R1 + R2) / R2 = 2.0
- 4.2V battery → 2.1V at ADC (safe for 3.3V max)
- Update firmware: `#define VOLTAGE_DIVIDER 2.0`

**Note:** Verify Stella's ADC pin tolerance before connecting!

### Connection Procedure

1. **Verify Polarity**
   - Check battery connector polarity (red = +, black = -)
   - **DO NOT reverse polarity** - will damage Stella!

2. **Connect Battery**
   - If Stella has power switch, ensure it's OFF
   - Connect battery connector to Stella power input
   - Verify LED indicators show power

3. **First Power-On Test**
   - LED should blink blue (initialization)
   - Then turn green (if anchors visible) or yellow/red (if not)
   - If LED stays off or red: disconnect immediately and check connections

4. **Verify Operation**
   - Connect USB for serial monitoring (battery can remain connected)
   - Open Serial Monitor at 115200 baud
   - Verify heartbeat messages every 10 seconds
   - Check battery voltage reading: should be 3.7-4.2V for fresh battery

5. **Initial Charge Cycle**
   - Charge new batteries fully before first use
   - Use LiPo-compatible charger (balance charger recommended)
   - Never charge LiPo batteries unattended
   - Follow charger manufacturer's instructions

## Mounting to Sphero Robots

### Mounting Considerations

**Requirements:**
- **Secure attachment:** Tag must not disconnect during movement
- **UWB antenna clearance:** Metal should not block antenna
- **Weight distribution:** Minimize impact on Sphero balance
- **Accessibility:** Easy to access for battery changes
- **Protection:** Shield from collisions

**Recommended Positions:**
1. **Top-mounted:** On Sphero's clear shell (best UWB visibility)
2. **Side-mounted:** On equator ring (lower profile)
3. **Custom bracket:** 3D-printed holder for clean integration

### Mounting Methods

#### Option 1: Velcro Strips (Quick & Removable)

**Pros:** Easy to attach/detach, non-destructive
**Cons:** May loosen with vibration, adds thickness

**Procedure:**
1. Clean Sphero surface with isopropyl alcohol
2. Cut industrial-strength velcro to size (~2" × 1")
3. Attach hook side to Sphero shell
4. Attach loop side to Stella board or enclosure
5. Press firmly together
6. Verify secure attachment with shake test

#### Option 2: Zip Ties (Secure & Permanent)

**Pros:** Very secure, inexpensive
**Cons:** Semi-permanent, may require drilling

**Procedure:**
1. If Sphero has existing mounting points, use those
2. Otherwise, create small loops from thin wire or use cable tie mounts
3. Route zip ties around Sphero equator or through mounting points
4. Position Stella board with battery
5. Tighten zip ties securely (but not crushing wires)
6. Trim excess length

#### Option 3: 3D-Printed Bracket (Professional & Custom)

**Pros:** Custom fit, professional appearance, optimal antenna placement
**Cons:** Requires 3D printer, design time

**Procedure:**
1. Measure Sphero dimensions and Stella board size
2. Design bracket in CAD software (Fusion 360, OpenSCAD, etc.)
3. Include:
   - Secure Stella mounting points
   - Battery compartment
   - Snap-fit or screw attachment to Sphero
   - Cable routing channels
4. 3D print in PETG or ABS (durable materials)
5. Test fit and iterate design
6. Install on Sphero

**Design Files:** (To be created based on specific Sphero model)

### Antenna Orientation

**DW3000 UWB Antenna:**
- Stella has built-in or external UWB antenna
- Antenna should have clear line-of-sight in horizontal plane
- Avoid placing large metal objects directly in front of antenna
- Test ranging performance after mounting

**Best Practices:**
- Mount tag on top of Sphero (highest visibility)
- Keep antenna away from battery and large conductors
- Test in arena before permanent installation

## Pre-Deployment Testing

### Bench Test (Required Before Deployment)

**Setup:**
- All 4 anchors powered and running
- Tag with fresh battery
- Serial monitor connected (optional)

**Test Procedure:**

1. **Power-On Test**
   - [ ] LED blinks blue during initialization (2-3 seconds)
   - [ ] LED turns green when ranging starts (with anchors visible)
   - [ ] No solid red LED (system error)

2. **Ranging Test**
   - [ ] Place tag at various distances from anchors (1m, 5m, 10m)
   - [ ] Verify LED remains green (all anchors visible)
   - [ ] If serial connected: verify ranging data from all 4 anchors

3. **Movement Test**
   - [ ] Move tag around arena
   - [ ] Verify continuous ranging (LED stays green)
   - [ ] Check for dead zones or areas of poor reception

4. **Battery Test**
   - [ ] Verify battery voltage reading (if monitoring enabled)
   - [ ] Should read 3.7-4.2V for fresh LiPo
   - [ ] No low battery warning (orange LED)

5. **Uptime Test**
   - [ ] Run tag for 30 minutes continuously
   - [ ] Verify stable operation
   - [ ] Check for overheating (board should be warm but not hot)
   - [ ] Monitor battery voltage trend

### Multi-Tag Test (Before Full Deployment)

**Setup:**
- Start with 3 tags, then increase to 10-12
- All anchors operational
- Tags positioned in arena

**Test Procedure:**

1. **Simultaneous Operation**
   - [ ] Power on all tags
   - [ ] Verify each tag initializes (blue → green LED)
   - [ ] No tags show error state (solid red)

2. **Ranging Success Rate**
   - [ ] Monitor for 5 minutes
   - [ ] All tags should maintain green LED (or yellow if NLOS)
   - [ ] If serial available: verify > 90% success rate

3. **Interference Test**
   - [ ] Observe LED status on all tags
   - [ ] Tags should not interfere with each other (stagger prevents this)
   - [ ] If interference occurs: increase `TAG_STAGGER_MS` in firmware

4. **ROS2 Integration**
   - [ ] Verify ROS2 localization node receives data from all tags
   - [ ] Check position estimates are reasonable
   - [ ] Verify no duplicate TAG_IDs causing conflicts

## Operational Procedures

### Daily Startup Procedure

1. **Pre-Flight Checks**
   - [ ] Verify all anchors are powered on
   - [ ] Check arena for obstructions
   - [ ] Verify ROS2 localization node is running

2. **Tag Power-On Sequence**
   - Power on tags one at a time (or in small batches)
   - Wait 5 seconds between tags for initialization
   - Verify each tag shows green or yellow LED
   - If tag shows red: troubleshoot before continuing

3. **System Verification**
   - Check ROS2 topic for position data: `ros2 topic echo /uwb/positions`
   - Verify all expected TAG_IDs are publishing
   - Place tags in known positions and verify accuracy

4. **Sphero Integration**
   - Attach tags to Sphero robots
   - Power on Spheros
   - Verify tags continue ranging despite Sphero movement
   - Start experiment/operation

### During Operation

**Monitoring:**
- **LED Status:** Periodically check tag LEDs
  - **Green:** All good, 4 anchors visible
  - **Yellow:** Partial ranging, 2-3 anchors visible (acceptable)
  - **Red:** Poor ranging or no anchors (investigate)
  - **Orange:** Low battery warning (replace soon)
  - **Fast red blink:** Critical battery (replace immediately)

- **ROS2 Position Data:** Monitor position estimates
  - Check for outliers or jumps in position
  - Verify position tracks Sphero movement
  - Watch for tags going offline

**Battery Management:**
- Monitor battery voltage from heartbeat messages (if serial connected)
- Typical voltage drop during operation:
  - 4.2V → 3.9V: 80-100% (first hour)
  - 3.9V → 3.7V: 50-80% (normal operation)
  - 3.7V → 3.5V: 20-50% (nearing low battery)
  - 3.5V → 3.3V: Low battery warning (orange LED)
  - < 3.3V: Critical, replace battery

### Shutdown Procedure

1. **Tag Shutdown**
   - Remove tags from Spheros
   - Power off tags (disconnect battery or use power switch)
   - No special shutdown sequence required (data is not stored)

2. **Battery Charging**
   - Check battery voltage if possible
   - Charge depleted batteries with LiPo charger
   - Store batteries at ~3.8V (50% charge) for long-term storage
   - Never store fully discharged batteries

3. **System Shutdown**
   - Stop ROS2 localization node
   - Power off anchors
   - Log any issues for next session

## Troubleshooting

### LED Status Meanings

| LED Pattern | Meaning | Action Required |
|-------------|---------|-----------------|
| Solid blue | Initializing | Wait 2-3 seconds |
| Solid green | All 4 anchors visible | Normal operation |
| Slow blink yellow | 2-3 anchors visible | Check anchor positions, acceptable |
| Fast blink red | 0-1 anchors visible | Check anchors, investigate NLOS |
| Blink orange | Low battery | Replace battery soon |
| Fast blink red | Critical battery | Replace battery immediately |
| Solid red | System error | Power cycle, check serial output |
| Off | No power | Check battery connection |

### Common Issues

#### Tag Shows Red LED (No Anchors Visible)

**Possible Causes:**
1. Anchors are not powered on
2. Tag is too far from anchors (> 50m)
3. NLOS blockage (metal walls, equipment)
4. Incorrect MAC addresses or session IDs
5. UWB initialization failed

**Solutions:**
- Verify anchors are running (check their LEDs/serial output)
- Move tag closer to anchors
- Remove obstructions
- Check serial output for errors
- Reflash firmware if persistent

#### Tag Shows Yellow LED (Partial Ranging)

**Possible Causes:**
1. Some anchors are out of range
2. NLOS to some anchors
3. Interference or multipath
4. One or more anchors failed

**Solutions:**
- Verify all 4 anchors are operational
- Check anchor placement for optimal coverage
- This is often acceptable for positioning (3 anchors minimum)

#### Tag Shows Orange LED (Low Battery)

**Meaning:** Battery voltage below 3.3V

**Action:**
- Complete current task if critical
- Replace battery as soon as possible
- Tag will continue operating but may shut down unexpectedly

#### Ranging Success Rate Low

**Symptoms:** Frequent errors in serial output, inconsistent position data

**Solutions:**
- Increase `TAG_STAGGER_MS` to reduce collisions
- Reduce number of simultaneous tags
- Check for interference sources (WiFi, other UWB devices)
- Verify anchor placement optimizes coverage

#### Tag Stops Responding

**Symptoms:** LED off or frozen, no serial output, no position data

**Solutions:**
- Power cycle tag (disconnect and reconnect battery)
- Check battery voltage (may be depleted)
- Reflash firmware if persistent
- Hardware failure - replace board

#### Overheating

**Symptoms:** Board very hot to touch (> 60°C)

**Causes:**
- Continuous high-power operation
- Poor ventilation in enclosure
- Battery overcharge (damaged cell)

**Solutions:**
- Verify `ENABLE_SLEEP` is true in firmware
- Improve ventilation
- Check battery health
- Reduce LED brightness
- Consider larger heat sinking

#### Position Jumps / Outliers

**Symptoms:** ROS2 position data shows sudden jumps or outliers

**Causes:**
- NLOS ranging (multipath reflections)
- Anchor clock drift
- Interference from other tags
- Kalman filter issues in ROS2 node

**Solutions:**
- Addressed in ROS2 localization node (not tag firmware)
- Verify ranging data is consistent on tag serial output
- Check for metal reflectors in arena

## Maintenance

### Daily Maintenance
- Visual inspection of tags and mounting
- Check LED status on all tags
- Verify battery connections secure

### Weekly Maintenance
- Charge all batteries fully
- Check for loose mounting hardware
- Inspect cables for wear

### Monthly Maintenance
- Test battery capacity (charge cycle + runtime test)
- Verify firmware versions on all tags
- Check UWB antenna connections
- Clean boards if dusty

### Battery Care
- **Charging:** Use LiPo-compatible balance charger
- **Storage:** Store at 3.8V (50% charge) in cool, dry place
- **Safety:** Never charge damaged or swollen batteries
- **Replacement:** Replace batteries showing < 70% capacity
- **Disposal:** Dispose of old batteries at proper recycling facility

## Performance Expectations

### Normal Operation

**Ranging Performance:**
- **Update Rate:** 3-4 Hz per tag (varies with TAG_ID stagger)
- **Range:** 1-50 meters (optimal: 5-30 meters)
- **Accuracy:** 10-30 cm (depends on anchor geometry and NLOS)
- **Success Rate:** > 90% with all 4 anchors visible

**Power Consumption:**
- **Active:** ~150-200 mA during ranging
- **Sleep:** ~50-100 mA (soft sleep with delay)
- **Average:** ~100-150 mA with 50% duty cycle

**Battery Life:**
- **1000 mAh:** 6-8 hours
- **2000 mAh:** 12-16 hours

**Startup Time:**
- **Initialization:** 2-3 seconds (blue LED)
- **First ranging:** 3-5 seconds after init
- **Full operation:** < 10 seconds from power-on

### Environmental Limits

**Operating Temperature:**
- **Recommended:** 15-30°C (room temperature)
- **Extended:** 0-40°C
- **Avoid:** < 0°C (battery performance degrades), > 50°C (component damage)

**Humidity:**
- **Recommended:** 20-60% RH
- **Avoid:** Condensation, direct water exposure

**Vibration/Shock:**
- Tags should tolerate normal Sphero operation
- Verify mounting is secure
- Avoid dropping or impacts

## Safety

### Battery Safety

**WARNING:** LiPo batteries can be dangerous if mishandled!

**Safe Practices:**
- Never puncture, crush, or short-circuit battery
- Never charge unattended
- Never charge damaged or swollen batteries
- Use proper LiPo charger with balance function
- Store in LiPo-safe bag or container
- Keep away from flammable materials
- Dispose properly (do not throw in trash)

**Emergency:**
- If battery swells: carefully disconnect and dispose
- If battery catches fire: use Class D fire extinguisher or sand (NOT water!)

### Electrical Safety

- Verify battery polarity before connecting
- Do not modify circuits while powered
- Avoid short circuits
- Use proper fuses if available

### Operational Safety

- Tags should not interfere with Sphero motor control
- Verify tags are securely mounted (won't fall during operation)
- Keep arena clear of spectators during robot operation

## Appendix A: Configuration Reference

### Default Configuration (tag_firmware_optimized.ino)

```cpp
#define TAG_ID              1       // SET THIS UNIQUELY (1-12)
#define NUM_ANCHORS         4
#define SERIAL_BAUD         115200
#define PREAMBLE_BASE       9

#define RANGING_INTERVAL_MS 250
#define TAG_STAGGER_MS      10
#define ENABLE_SLEEP        true
#define SLEEP_MARGIN_MS     50

#define ENABLE_BATTERY_MON  true
#define BATTERY_ADC_PIN     A0
#define VOLTAGE_DIVIDER     2.0
#define BATTERY_LOW_MV      3300
#define BATTERY_CRIT_MV     3100

#define ENABLE_LED_STATUS   true
#define LED_BRIGHTNESS      32

#define ENABLE_SERIAL_DEBUG true
#define HEARTBEAT_INTERVAL  10000
```

### Tuning for Different Scenarios

**Long Battery Life (12+ hours):**
```cpp
#define RANGING_INTERVAL_MS 500     // Slower updates
#define ENABLE_SERIAL_DEBUG false   // Disable debug
#define LED_BRIGHTNESS      16      // Dimmer LED
```

**High Update Rate (6-8 Hz):**
```cpp
#define RANGING_INTERVAL_MS 125     // Faster updates
#define TAG_STAGGER_MS      5       // Shorter stagger
// Note: May increase collisions with many tags
```

**Minimal Interference (many tags):**
```cpp
#define TAG_STAGGER_MS      20      // Longer stagger
#define RANGING_INTERVAL_MS 300     // Longer interval
```

## Appendix B: Diagnostic Serial Output

### Startup Output
```
========================================
# Stella Tag 1 — OPTIMIZED FIRMWARE v1.0
========================================
# Battery voltage: 3.85V (85%)
# Ranging interval: 260 ms
# Initializing UWB...
# UWB ready.
# Session: anchor=1 sessionId=0x10001 preamble=9
# Session: anchor=2 sessionId=0x20001 preamble=10
# Session: anchor=3 sessionId=0x30001 preamble=11
# Session: anchor=4 sessionId=0x40001 preamble=12
# All sessions started. Tag is ranging.
========================================
```

### Ranging Output
```
Tag 1 | A1 | 142 cm | session 0x10001 | MAC: 1:1
Tag 1 | A2 | 238 cm | session 0x20001 | MAC: 1:1
Tag 1 | A3 | 189 cm | session 0x30001 | MAC: 1:1
Tag 1 | A4 | 315 cm | session 0x40001 | MAC: 1:1
```

### Heartbeat Output
```
----------------------------------------
# Tag 1 | Uptime: 125s | Anchors: 4/4 | Battery: 3.82V (80%)
#   A1: OK   | ranges=498 errors=2 (99.6%)
#   A2: OK   | ranges=501 errors=0 (100.0%)
#   A3: OK   | ranges=495 errors=5 (99.0%)
#   A4: OK   | ranges=499 errors=1 (99.8%)
----------------------------------------
```

## Support & Resources

**Firmware Location:**
- `/home/svaghela/ros2_ws_2/arduino/tag_firmware/tag_firmware_optimized.ino`

**Documentation:**
- `TAG_FLASHING_GUIDE.md` - Firmware installation
- `TAG_DEPLOYMENT_GUIDE.md` - This document
- `TAG_PERFORMANCE_REPORT.md` - Performance analysis

**Libraries:**
- StellaUWB Library (Truesense)
- Arduino Stella Board Support

**External Resources:**
- Arduino Stella documentation
- DW3000 UWB transceiver datasheet
- STM32H747 reference manual

---

**Document Version:** 1.0
**Last Updated:** 2026-03-07
**Maintained by:** Arduino Expert Agent
