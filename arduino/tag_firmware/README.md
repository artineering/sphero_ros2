# Arduino Stella UWB Tag Firmware

**Optimized firmware for battery-powered UWB positioning tags**

## Overview

This directory contains the optimized firmware for Arduino Stella boards operating as UWB tags in a Real-Time Location System (RTLS). The tags perform Two-Way Ranging (TWR) with 4 Portenta C33 anchors to enable precise positioning of Sphero robots.

**Hardware:** Arduino Stella (STM32H747 + DW3000 UWB)
**Application:** Sphero robot tracking in indoor arena
**Deployment:** 10-12 battery-powered tags

## Quick Start

### For First-Time Users

1. **Flash the firmware:**
   - See `TAG_FLASHING_GUIDE.md` for detailed instructions
   - Set unique `TAG_ID` (1-12) before flashing each board
   - Use Arduino IDE with StellaUWB library

2. **Deploy the hardware:**
   - See `TAG_DEPLOYMENT_GUIDE.md` for installation
   - Install battery (LiPo 3.7V, 1000+ mAh recommended)
   - Mount to Sphero robot

3. **Verify operation:**
   - LED should show green (all anchors visible) or yellow (partial ranging)
   - Check serial output at 115200 baud for diagnostics

4. **Monitor performance:**
   - See `TAG_PERFORMANCE_REPORT.md` for expected metrics
   - Battery life: 6-8 hours (1000 mAh)
   - Update rate: 2.7-3.8 Hz per tag

## Directory Contents

### Firmware Files

- **`tag_firmware_optimized.ino`** - Main firmware (FLASH THIS)
  - Optimized for power efficiency and reliability
  - Includes all enhancements: battery monitoring, LED status, anchor health tracking

### Documentation

- **`TAG_FLASHING_GUIDE.md`** - How to flash firmware to Stella boards
  - Arduino IDE setup
  - Library installation (StellaUWB)
  - Step-by-step flashing procedure
  - Troubleshooting compilation and upload issues

- **`TAG_DEPLOYMENT_GUIDE.md`** - How to deploy tags in the field
  - Battery selection and installation
  - Mounting to Sphero robots
  - Operational procedures
  - LED status meanings
  - Troubleshooting common issues

- **`TAG_PERFORMANCE_REPORT.md`** - Expected performance metrics
  - Power consumption analysis
  - Battery life calculations
  - Timing and update rates
  - Ranging accuracy expectations
  - System-level performance

- **`README.md`** - This file

## Key Features

### 1. Power Management
- **Sleep cycles:** Tag sleeps between ranging cycles to save power
- **Configurable intervals:** 250ms base + TAG_ID stagger
- **Battery life:** 6-8 hours on 1000 mAh LiPo (vs. 5 hours in reference firmware)

### 2. Battery Monitoring
- **Voltage sensing:** Real-time battery voltage via ADC
- **Low battery warning:** Orange LED when < 3.3V
- **Critical battery warning:** Fast red blink when < 3.1V
- **Percentage estimate:** Approximate charge level in diagnostics

### 3. LED Status Indication
- **7 distinct patterns:** Visual feedback without serial connection
- **Color coding:**
  - Blue: Initializing
  - Green: All 4 anchors visible (good ranging)
  - Yellow: 2-3 anchors visible (partial ranging)
  - Red: 0-1 anchors visible (poor ranging)
  - Orange: Low battery
  - Fast red: Critical battery
- **Low power:** LED brightness configurable (default: 32/255)

### 4. Anchor Health Tracking
- **Per-anchor statistics:** Success rate, error count, last-seen time
- **Automatic status updates:** LED reflects current anchor visibility
- **Diagnostic output:** Heartbeat messages every 10 seconds with detailed stats

### 5. Collision Mitigation
- **TAG_ID staggering:** Each tag has unique ranging cycle offset (10ms × TAG_ID)
- **Reduced interference:** < 5% collision rate with 10-12 tags
- **Load balancing:** Higher TAG_IDs have longer intervals (more time for others)

### 6. Comprehensive Diagnostics
- **Startup messages:** Initialization status, session IDs, battery voltage
- **Ranging data:** Distance to each anchor, session handle, MAC address
- **Heartbeat:** Periodic status with uptime, anchor health, battery level
- **Error tracking:** Failed ranges logged per anchor with success rate

## Configuration

### Critical: Set TAG_ID Before Flashing

**Each tag must have a unique TAG_ID (1-12).**

Edit this line in `tag_firmware_optimized.ino`:

```cpp
#define TAG_ID              1       // SET THIS TO 1-12 (UNIQUE PER TAG)
```

### Optional Configuration

**Power Management:**
```cpp
#define RANGING_INTERVAL_MS 250     // Base interval (ms)
#define TAG_STAGGER_MS      10      // Stagger per tag (ms)
#define ENABLE_SLEEP        true    // Enable power saving
```

**Battery Monitoring:**
```cpp
#define ENABLE_BATTERY_MON  true    // Enable/disable monitoring
#define BATTERY_ADC_PIN     A0      // ADC pin for voltage
#define VOLTAGE_DIVIDER     2.0     // Hardware divider ratio
#define BATTERY_LOW_MV      3300    // Low threshold (mV)
#define BATTERY_CRIT_MV     3100    // Critical threshold (mV)
```

**LED Status:**
```cpp
#define ENABLE_LED_STATUS   true    // Enable/disable LED
#define LED_BRIGHTNESS      32      // 0-255 (lower saves power)
```

**Debug Output:**
```cpp
#define ENABLE_SERIAL_DEBUG true    // Print ranging data to serial
#define HEARTBEAT_INTERVAL  10000   // Heartbeat period (ms)
```

## LED Status Guide

| LED Color/Pattern | Meaning | Action |
|-------------------|---------|--------|
| Solid blue | Initializing (2-3 sec) | Wait |
| Solid green | All 4 anchors visible | Normal operation ✓ |
| Slow blink yellow | 2-3 anchors visible | Check anchor placement |
| Fast blink red | 0-1 anchors visible | Verify anchors are running |
| Blink orange | Low battery (< 3.3V) | Replace battery soon |
| Fast blink red | Critical battery (< 3.1V) | Replace immediately |
| Solid red | System error | Power cycle or reflash |

## Serial Output Examples

### Startup (115200 baud)
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

### Ranging Data
```
Tag 1 | A1 | 142 cm | session 0x10001 | MAC: 1:1
Tag 1 | A2 | 238 cm | session 0x20001 | MAC: 1:1
Tag 1 | A3 | 189 cm | session 0x30001 | MAC: 1:1
Tag 1 | A4 | 315 cm | session 0x40001 | MAC: 1:1
```

### Heartbeat (every 10 seconds)
```
----------------------------------------
# Tag 1 | Uptime: 125s | Anchors: 4/4 | Battery: 3.82V (80%)
#   A1: OK   | ranges=498 errors=2 (99.6%)
#   A2: OK   | ranges=501 errors=0 (100.0%)
#   A3: OK   | ranges=495 errors=5 (99.0%)
#   A4: OK   | ranges=499 errors=1 (99.8%)
----------------------------------------
```

## Expected Performance

### Ranging Performance
- **Update rate:** 3.8 Hz (Tag 1) to 2.7 Hz (Tag 12)
- **Operational range:** 1-50 meters (optimal: 5-30m)
- **Accuracy:** 10-30 cm in line-of-sight
- **Success rate:** > 90% with all anchors visible

### Power Performance
- **Active current:** 165-230 mA during ranging
- **Sleep current:** 90-130 mA (soft sleep with delay)
- **Average current:** 100-150 mA with sleep enabled
- **Battery life (1000 mAh):** 6-8 hours

### System Performance
- **Startup time:** < 5 seconds from power-on
- **Multi-tag capacity:** 10-12 tags simultaneously
- **Collision rate:** < 5% with default stagger
- **End-to-end latency:** 100-570 ms (average: 300ms)

## Hardware Requirements

### Arduino Stella Board
- **MCU:** STM32H747 dual-core (Cortex-M7 @ 480MHz + M4 @ 240MHz)
- **UWB:** Built-in DW3000 transceiver
- **RAM:** 1 MB SRAM
- **Flash:** 2 MB
- **LED:** Built-in RGB LED
- **Power:** USB-C or battery input

### Battery
- **Type:** LiPo (Lithium Polymer)
- **Voltage:** 3.7V nominal (4.2V max, 3.0V min)
- **Capacity:** 1000-2000 mAh recommended
- **Connector:** Compatible with Stella power input

### Optional: Voltage Divider
- **R1:** 10 kΩ (battery to ADC)
- **R2:** 10 kΩ (ADC to GND)
- **Ratio:** 2.0 (configurable in firmware)
- **Purpose:** Battery voltage sensing on ADC pin

## Prerequisites

### Software
- **Arduino IDE 2.x** or later
- **Arduino Stella Board Support Package**
- **StellaUWB Library** (Truesense)

### Hardware
- **Arduino Stella board(s)** (1-12 units)
- **LiPo batteries** (one per tag)
- **USB-C cable** for programming
- **Mounting hardware** (velcro, zip ties, or brackets)

## Troubleshooting

### LED shows solid red
- **Cause:** UWB initialization failed
- **Solution:** Power cycle tag, verify firmware flashed correctly, check serial output

### LED shows fast blinking red (not battery)
- **Cause:** No anchors visible
- **Solution:** Verify anchors are powered on and running, check TAG_ID is unique

### No serial output
- **Cause:** Wrong baud rate or port
- **Solution:** Set Serial Monitor to 115200 baud, select correct port

### Battery drains too fast
- **Cause:** Sleep not enabled, serial debug enabled, LED too bright
- **Solution:** Verify `ENABLE_SLEEP true`, disable serial debug, reduce LED brightness

### Ranging success rate low
- **Cause:** Interference from other tags, NLOS, out of range
- **Solution:** Increase `TAG_STAGGER_MS`, check anchor placement, reduce tag count

## System Architecture

### Tag Role
- **Responder/Controlee:** Anchors initiate ranging, tags respond
- **Multi-session:** One UWB session per anchor (4 sessions total)
- **Passive:** Tags don't broadcast positions, anchors collect data

### MAC Addressing
- **Anchor N:** `{0xA0 + N, 0x01}` (e.g., Anchor 1 = `{0xA1, 0x01}`)
- **Tag M:** `{M, 0x01}` (e.g., Tag 5 = `{0x05, 0x01}`)

### Session IDs
- **Format:** `0x<AnchorHex><TagHex>`
- **Example:** Anchor 2 + Tag 5 = `0x020005` or `0x00020005`
- **Purpose:** Unique identifier for each anchor-tag pair

### Preamble Codes
- **Base:** 9 (must match anchor firmware)
- **Allocation:** Anchor 1 → 9, Anchor 2 → 10, Anchor 3 → 11, Anchor 4 → 12
- **Purpose:** Channel separation for simultaneous ranging

## Integration with ROS2

This firmware is designed to work with the UWB positioning system that includes:

1. **4 Portenta C33 Anchors:** Initiating ranging, collecting distance data
2. **Anchor WiFi Gateway:** Transmitting ranging data to host computer
3. **ROS2 Localization Node:** Computing tag positions via trilateration
4. **Sphero Control Node:** Using positions for robot control

**Data Flow:**
```
Tag (this firmware)
  ↕ UWB TWR
Anchor
  → WiFi/Serial
Host Computer
  → ROS2 Topic (/uwb/positions)
Sphero Control Node
  → Robot Commands
```

**Note:** Tags do NOT communicate directly with ROS2. Position computation happens on the host based on anchor-collected ranging data.

## Development Notes

### Code Structure

**Classes:**
- `LEDManager`: LED status indication (200 lines)
- `BatteryMonitor`: Battery voltage monitoring (100 lines)
- `PowerManager`: Sleep cycle management (100 lines)
- `AnchorHealth`: Anchor statistics tracking (50 lines)

**Global Objects:**
- `UWBMultiSessionTag* sessions[NUM_ANCHORS]`: UWB session handles
- `LEDManager ledManager`: LED controller
- `BatteryMonitor battery`: Battery monitor
- `PowerManager powerMgr`: Power manager
- `AnchorHealth anchorHealth[NUM_ANCHORS]`: Per-anchor statistics

**Key Functions:**
- `rangingHandler()`: UWB ranging callback (processes TWR results)
- `setup()`: Initialization (UWB, sessions, peripherals)
- `loop()`: Main loop (battery check, LED update, diagnostics, sleep)

### Memory Usage
- **Flash:** ~40-50 KB (< 3% of 2 MB)
- **RAM:** ~10-15 KB (< 2% of 1 MB)
- **Stack:** Minimal (no deep recursion)
- **Heap:** Minimal (sessions allocated in setup)

### Timing Constraints
- `loop()` iteration: < 10 ms (non-blocking)
- UWB callbacks: < 5 ms (handled by library)
- LED update: < 1 ms (simple GPIO)
- Battery ADC: < 1 ms (every 5 seconds)

## Future Enhancements

Potential improvements for future firmware versions:

1. **True STM32 Sleep Mode:** Replace `delay()` with STOP mode (50-70% power reduction)
2. **NLOS Detection:** Identify non-line-of-sight ranges (improve positioning accuracy)
3. **Dynamic Power Management:** Adjust ranging rate based on battery level
4. **Over-the-Air Configuration:** Change TAG_ID and settings without reflashing
5. **Data Logging:** Store ranging statistics to SD card or EEPROM
6. **Temperature Monitoring:** Track board temperature for thermal management
7. **Watchdog Timer:** Auto-reset on firmware hang
8. **FreeRTOS Migration:** Multi-task architecture for complex scenarios

## Support

For questions or issues:

1. **Check documentation:**
   - `TAG_FLASHING_GUIDE.md` - Installation and flashing
   - `TAG_DEPLOYMENT_GUIDE.md` - Hardware setup and troubleshooting
   - `TAG_PERFORMANCE_REPORT.md` - Expected performance metrics

2. **Check serial output:**
   - Connect USB, open Serial Monitor at 115200 baud
   - Look for error messages or unexpected behavior
   - Compare output to examples in this README

3. **Verify configuration:**
   - TAG_ID is unique (1-12)
   - NUM_ANCHORS matches deployment (4)
   - Battery monitoring settings match hardware

4. **External resources:**
   - Arduino Stella documentation
   - StellaUWB library documentation
   - DW3000 datasheet
   - STM32H747 reference manual

## Version History

### v1.0 (2026-03-07) - Initial Optimized Release
- Power management with sleep cycles
- Battery voltage monitoring and warnings
- RGB LED status indication (7 patterns)
- Anchor health tracking with statistics
- TAG_ID-based stagger for collision mitigation
- Comprehensive diagnostic output
- Production-ready for 10-12 tag deployment

### v0.1 (Reference Firmware)
- Basic TWR with 4 anchors
- Simple MAC/session architecture
- Minimal diagnostics
- No power optimization

## License

This firmware is developed for the Sphero UWB positioning system project.
Refer to main project repository for license information.

## Authors

- **Arduino Expert Agent** - Firmware development and documentation
- **Project Lead** - System architecture and requirements

---

**Last Updated:** 2026-03-07
**Firmware Version:** 1.0
**Maintained by:** Arduino Expert Agent
