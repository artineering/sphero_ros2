# Arduino Stella Tag - Flashing Guide

**Version:** 1.0
**Date:** 2026-03-07
**Firmware:** tag_firmware_optimized.ino

## Overview

This guide provides step-by-step instructions for flashing the optimized UWB tag firmware onto Arduino Stella boards. You will need to flash **10-12 individual tags**, each with a unique TAG_ID.

## Prerequisites

### Hardware Requirements
- **Arduino Stella board(s)** (DCU040 with STM32H747 + DW3000 UWB)
- **USB-C cable** for programming
- **Computer** running Windows, macOS, or Linux
- **LiPo battery** (3.7V, 1000+ mAh recommended) with appropriate connector
- Optional: **Battery voltage divider circuit** if not built into Stella

### Software Requirements
1. **Arduino IDE 2.x** or later
   - Download from: https://www.arduino.cc/en/software

2. **Arduino Stella Board Support Package**
   - Install via Arduino IDE Board Manager
   - Search for "Arduino Stella" or "STM32H747"

3. **StellaUWB Library**
   - Install via Arduino IDE Library Manager
   - Search for "StellaUWB" or "Truesense UWB"
   - Alternative: Manual installation from Truesense repository

4. **Firmware File**
   - `tag_firmware_optimized.ino` (from this repository)
   - Location: `/home/svaghela/ros2_ws_2/arduino/tag_firmware/`

## Installation Steps

### 1. Install Arduino IDE

Download and install Arduino IDE 2.x from the official website.

### 2. Add Stella Board Support

1. Open Arduino IDE
2. Go to **Tools → Board → Boards Manager**
3. Search for "**Arduino Stella**" or "**STM32H747**"
4. Click **Install** on the appropriate board package
5. Wait for installation to complete

**Note:** If Stella is not available, you may need to add a custom board URL:
- Go to **File → Preferences**
- Add board URL to "Additional Boards Manager URLs"
- Check Arduino documentation for the correct URL

### 3. Install StellaUWB Library

1. Go to **Tools → Manage Libraries** (or Sketch → Include Library → Manage Libraries)
2. Search for "**StellaUWB**"
3. Click **Install** on the StellaUWB library
4. Wait for installation to complete

**If not available in Library Manager:**
1. Download the library from Truesense repository
2. Extract to Arduino libraries folder:
   - Windows: `Documents\Arduino\libraries\`
   - macOS: `~/Documents/Arduino/libraries/`
   - Linux: `~/Arduino/libraries/`
3. Restart Arduino IDE

### 4. Open Firmware File

1. Open Arduino IDE
2. **File → Open**
3. Navigate to `/home/svaghela/ros2_ws_2/arduino/tag_firmware/`
4. Select `tag_firmware_optimized.ino`
5. Click **Open**

## Configuration

### CRITICAL: Set Unique TAG_ID

**Each Stella tag MUST have a unique TAG_ID (1-12).**

1. Locate the configuration section at the top of the firmware:

```cpp
// ============================================================
// USER CONFIGURATION — MUST SET TAG_ID BEFORE FLASHING
// ============================================================
#define TAG_ID              1       // Unique tag identifier (1–12)
```

2. Change `TAG_ID` to a unique value for this specific board:
   - **Tag 1:** `#define TAG_ID 1`
   - **Tag 2:** `#define TAG_ID 2`
   - **Tag 3:** `#define TAG_ID 3`
   - ... and so on up to 12

3. **Keep a log** of which TAG_ID was flashed to which board (use serial number or label)

### Optional Configuration

**Number of Anchors:**
```cpp
#define NUM_ANCHORS         4       // Match your deployment
```

**Power Management:**
```cpp
#define RANGING_INTERVAL_MS 250     // Base ranging cycle (250ms recommended)
#define TAG_STAGGER_MS      10      // Stagger per tag (10ms × TAG_ID)
#define ENABLE_SLEEP        true    // Enable power saving
```

**Battery Monitoring:**
```cpp
#define ENABLE_BATTERY_MON  true    // Enable if hardware supports
#define BATTERY_ADC_PIN     A0      // Adjust to your pin
#define VOLTAGE_DIVIDER     2.0     // Adjust to your circuit
```

**Debug Output:**
```cpp
#define ENABLE_SERIAL_DEBUG true    // Disable in production to save power
```

## Flashing Procedure

### For Each Tag (Repeat 10-12 times):

1. **Edit TAG_ID**
   - Open `tag_firmware_optimized.ino`
   - Set `TAG_ID` to next available number (1, 2, 3, ...)
   - Save the file (**Ctrl+S** or **Cmd+S**)

2. **Label the Physical Board**
   - Use a label maker or permanent marker
   - Write "**TAG X**" where X is the TAG_ID
   - Place label on the Stella board

3. **Connect Stella to Computer**
   - Use USB-C cable
   - Wait for computer to recognize device
   - Note: First connection may install drivers (Windows)

4. **Select Board and Port**
   - **Tools → Board → Arduino Stella** (or STM32H747)
   - **Tools → Port → (select the port showing Stella)**
     - Windows: `COMx`
     - macOS: `/dev/cu.usbmodemXXXX`
     - Linux: `/dev/ttyACMx`

5. **Compile Firmware**
   - Click **Verify** button (checkmark icon) or **Sketch → Verify/Compile**
   - Wait for compilation to complete
   - Check for errors in output window
   - Fix any errors before proceeding

6. **Upload Firmware**
   - Click **Upload** button (right arrow icon) or **Sketch → Upload**
   - Wait for upload process:
     - Compiling...
     - Uploading...
     - Success message
   - **Do not disconnect during upload**

7. **Verify Operation**
   - After upload completes, LED should:
     - Blink **blue** during initialization (~2-3 seconds)
     - Turn **solid green** when ranging starts (if 4 anchors visible)
     - Or turn **yellow** if only some anchors visible
     - Or turn **red** if no anchors visible

8. **Check Serial Output (Optional)**
   - **Tools → Serial Monitor**
   - Set baud rate to **115200**
   - You should see:
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

9. **Record in Log**
   - Tag ID: X
   - Board Serial Number (if available): _______
   - Flash Date: YYYY-MM-DD
   - Flash Status: ✓ Success / ✗ Failed
   - Notes: _______

10. **Disconnect and Move to Next Tag**
    - Safely disconnect USB cable
    - Set aside (with label visible)
    - Repeat steps 1-10 for next tag

## Troubleshooting

### Board Not Detected

**Symptoms:** Port not showing in Arduino IDE

**Solutions:**
1. Check USB cable (must be data cable, not charge-only)
2. Try different USB port
3. Install/update USB drivers (Windows)
4. Check if board is powered (LED indicators)
5. Press reset button on Stella while connecting

### Compilation Errors

**Symptoms:** Red error messages during compile

**Common Issues:**

1. **"StellaUWB.h not found"**
   - Solution: Install StellaUWB library (see Installation Steps)

2. **"UWBMultiSessionTag not declared"**
   - Solution: Update StellaUWB library to latest version

3. **"LEDR/LEDG/LEDB not defined"**
   - Solution: Verify correct board is selected (Stella, not generic STM32)

4. **Syntax errors**
   - Solution: Ensure you didn't accidentally modify the code

### Upload Fails

**Symptoms:** Upload starts but fails partway through

**Solutions:**
1. Verify correct board and port are selected
2. Try pressing and holding reset button, then release when upload starts
3. Check that no other program is using the serial port (close Serial Monitor)
4. Restart Arduino IDE
5. Try different USB cable or port

### LED Shows Red After Upload

**Symptoms:** LED is solid red or fast blinking red

**Meaning:**
- **Solid red:** System error (UWB initialization failed)
- **Fast blink red:** No anchors visible (or battery critical)

**Solutions:**
1. Verify anchors are powered on and running
2. Check TAG_ID is unique and valid (1-12)
3. Verify `NUM_ANCHORS` matches deployment
4. Check serial output for error messages
5. Reflash firmware

### No Serial Output

**Symptoms:** Serial Monitor shows nothing

**Solutions:**
1. Verify baud rate is set to **115200**
2. Select correct port in Serial Monitor
3. Try closing and reopening Serial Monitor
4. Press reset button on Stella
5. Verify `ENABLE_SERIAL_DEBUG` is set to `true`

## Flashing Checklist

Use this checklist for each tag:

- [ ] TAG_ID set correctly in code (1-12)
- [ ] Configuration reviewed (battery pin, voltage divider, etc.)
- [ ] Code compiled without errors
- [ ] Board labeled with TAG_ID
- [ ] USB cable connected
- [ ] Correct board selected (Arduino Stella)
- [ ] Correct port selected
- [ ] Upload successful
- [ ] LED shows blue → green (or yellow/red if anchors not visible)
- [ ] Serial output verified (optional)
- [ ] TAG_ID logged with board serial number
- [ ] Board disconnected and stored safely

## Mass Flashing Tips

When flashing multiple tags (10-12 units):

1. **Batch similar tags:** Flash all tags with same configuration first
2. **Use a spreadsheet:** Track TAG_ID, serial numbers, flash dates
3. **Label immediately:** Don't wait until after flashing all tags
4. **Test one first:** Fully test one tag before flashing all others
5. **Have spare boards:** In case of hardware failures
6. **Flash in pairs:** One person edits TAG_ID, another connects/uploads
7. **Double-check TAG_ID:** Most common mistake is duplicate TAG_IDs

## Next Steps

After flashing all tags:

1. **Battery Installation:** See `TAG_DEPLOYMENT_GUIDE.md`
2. **Bench Testing:** Verify all tags range with anchors
3. **Sphero Mounting:** Attach tags to Sphero robots
4. **System Integration:** Test with ROS2 localization node

## Support

For issues or questions:
- Check serial output for diagnostic messages
- Review `TAG_DEPLOYMENT_GUIDE.md` for operational details
- Consult StellaUWB library documentation
- Check Arduino Stella hardware documentation

---

**Document Version:** 1.0
**Last Updated:** 2026-03-07
**Maintained by:** Arduino Expert Agent
