# ✅ Arduino CLI Installation Complete

## Summary

Arduino CLI has been successfully installed and configured for flashing Portenta C33 and Arduino Stella boards.

---

## Installation Details

### Arduino CLI
- **Version:** 1.4.1
- **Location:** `/home/svaghela/ros2_ws_2/bin/arduino-cli`
- **Config:** `~/.arduino15/arduino-cli.yaml`
- **Added to PATH:** Yes (`~/.bashrc`)

### Board Cores Installed

1. **Arduino Portenta C33**
   - Core: `arduino:renesas_portenta@1.5.3`
   - FQBN: `arduino:renesas_portenta:portenta_c33`
   - For: Anchor firmware (WiFi/UDP)

2. **Arduino Stella**
   - Core: `arduino:mbed_stella@4.5.0`
   - FQBN: `arduino:mbed_stella:stella`
   - For: Tag firmware (battery-powered)

---

## Arduino Expert Agent Updated

**File:** `.claude/.agents/arduino_expert.md`

The Arduino Expert agent has been enhanced with:

✅ **Arduino CLI command reference** (comprehensive)
✅ **Board-specific FQBNs** for Portenta C33 and Stella
✅ **Batch flashing procedures** for 4 anchors and 10 tags
✅ **Troubleshooting commands** for debugging
✅ **Mandatory CLI usage** in all future plans

**Future Behavior:**
When you delegate Arduino tasks to the Arduino Expert agent, it will now automatically include Arduino CLI commands for compilation, uploading, and batch flashing in all plans.

---

## Quick Commands Reference

### Verify Installation
```bash
arduino-cli version
arduino-cli board listall
arduino-cli core list
```

### Compile Firmware
```bash
# Anchor firmware (Portenta C33)
arduino-cli compile --fqbn arduino:renesas_portenta:portenta_c33 arduino/anchor_firmware

# Tag firmware (Stella)
arduino-cli compile --fqbn arduino:mbed_stella:stella arduino/tag_firmware
```

### Upload to Board
```bash
# List connected boards
arduino-cli board list

# Upload anchor firmware
arduino-cli compile --upload --fqbn arduino:renesas_portenta:portenta_c33 -p /dev/ttyACM0 arduino/anchor_firmware

# Upload tag firmware
arduino-cli compile --upload --fqbn arduino:mbed_stella:stella -p /dev/ttyACM0 arduino/tag_firmware
```

### Serial Monitoring
```bash
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
```

---

## Documentation Created

**Main Setup Guide:** `arduino/ARDUINO_CLI_SETUP.md`

This comprehensive guide includes:
- Installation summary
- Quick start instructions
- Batch flashing scripts for 4 anchors and 10 tags
- Troubleshooting guide
- Next steps

---

## IMPORTANT: UWB Library Requirement

⚠️ **Note:** The UWB libraries (`PortentaUWBShield` and `StellaUWB`) are **not** available in the standard Arduino Library Manager. These are vendor-specific libraries.

### Current Status
- Firmware files include `#include <PortentaUWBShield.h>` and `#include <StellaUWB.h>`
- These libraries must be obtained from the hardware vendor (Truesense or Arduino)
- Compilation will fail until these libraries are installed

### How to Install UWB Libraries

**Option 1: Manual Installation**
1. Obtain the library ZIP files from the vendor
2. Extract to `~/Arduino/libraries/` (or `~/.arduino15/libraries/`)
3. Restart Arduino CLI or Arduino IDE

**Option 2: Include with Board Core** (if available)
Some board cores include the necessary libraries automatically. Check:
```bash
ls ~/.arduino15/packages/arduino/hardware/renesas_portenta/1.5.3/libraries/
ls ~/.arduino15/packages/arduino/hardware/mbed_stella/4.5.0/libraries/
```

**Option 3: From Source**
If you have access to the UWB library source code:
```bash
# Create library directory
mkdir -p ~/Arduino/libraries/PortentaUWBShield
mkdir -p ~/Arduino/libraries/StellaUWB

# Copy library files to appropriate directories
```

### Testing After Library Installation

Once the UWB libraries are installed:
```bash
# Test anchor firmware compilation
arduino-cli compile --fqbn arduino:renesas_portenta:portenta_c33 arduino/anchor_firmware

# Test tag firmware compilation
arduino-cli compile --fqbn arduino:mbed_stella:stella arduino/tag_firmware
```

Expected output: `Sketch uses X bytes... Global variables use Y bytes...` (success)

---

## Batch Flashing Scripts

Two scripts are provided in `arduino/ARDUINO_CLI_SETUP.md`:

1. **`flash_anchors.sh`** - Interactive script to flash 4 Portenta C33 anchors
   - Automatically configures ANCHOR_ID (1-4)
   - Prompts to connect each board
   - Auto-detects port or allows manual entry
   - Compiles and uploads

2. **`flash_tags.sh`** - Interactive script to flash 10 Stella tags
   - Automatically configures TAG_ID (1-10)
   - Prompts to connect each board
   - Auto-detects port or allows manual entry
   - Compiles and uploads

### Create the Scripts

```bash
cd /home/svaghela/ros2_ws_2/arduino

# Copy script templates from ARDUINO_CLI_SETUP.md
# Make them executable
chmod +x flash_anchors.sh
chmod +x flash_tags.sh
```

---

## Integration with Existing Workflow

### Current State

Your Sphero UWB Positioning System is **ready to deploy** except for the hardware flashing step:

✅ **Anchor firmware** - Created (`arduino/anchor_firmware/anchor_firmware.ino`)
✅ **Tag firmware** - Created (`arduino/tag_firmware/tag_firmware_optimized.ino`)
✅ **ROS2 node** - Created and built (`src/sphero_uwb_positioning/`)
✅ **Arduino CLI** - Installed and configured
✅ **Board cores** - Installed (Portenta C33, Stella)

⏳ **UWB libraries** - Needs to be obtained from vendor
⏳ **Hardware flashing** - Pending library installation

### Next Steps

1. **Obtain UWB libraries** from Arduino/Truesense vendor
2. **Install libraries** to Arduino libraries directory
3. **Test compilation** with Arduino CLI
4. **Flash hardware:**
   - 4 anchors (Portenta C33)
   - 10-12 tags (Stella)
5. **Deploy system** and test with ROS2 node

---

## Arduino Expert Agent Now Equipped

The Arduino Expert agent will now:
- ✅ Use Arduino CLI in all firmware plans
- ✅ Provide compilation commands
- ✅ Include batch flashing procedures
- ✅ Reference correct FQBNs for boards
- ✅ Include serial monitoring for testing

Example from future plans:
```markdown
## Step 5: Compile and Upload

### Compilation Test
arduino-cli compile --fqbn arduino:renesas_portenta:portenta_c33 arduino/new_firmware

### Upload to Hardware
arduino-cli compile --upload --fqbn arduino:renesas_portenta:portenta_c33 -p /dev/ttyACM0 arduino/new_firmware

### Monitor Output
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
```

---

## Files Created/Modified

### New Files
- `arduino/ARDUINO_CLI_SETUP.md` - Comprehensive setup and usage guide

### Modified Files
- `.claude/.agents/arduino_expert.md` - Updated with Arduino CLI commands and requirements
- `~/.bashrc` - Added Arduino CLI to PATH

### Configuration Files
- `~/.arduino15/arduino-cli.yaml` - Arduino CLI configuration
- Board cores installed in `~/.arduino15/packages/arduino/hardware/`

---

## Verification Checklist

Run these commands to verify the installation:

```bash
# 1. Check Arduino CLI is in PATH
arduino-cli version
# Expected: arduino-cli Version: 1.4.1 ...

# 2. List installed cores
arduino-cli core list
# Expected: arduino:renesas_portenta 1.5.3
#           arduino:mbed_stella 4.5.0

# 3. List available boards
arduino-cli board listall
# Expected: Arduino Portenta C33 arduino:renesas_portenta:portenta_c33
#           Arduino Stella       arduino:mbed_stella:stella

# 4. Try to compile (will fail until UWB libraries installed)
arduino-cli compile --fqbn arduino:renesas_portenta:portenta_c33 arduino/anchor_firmware
# Expected error: PortentaUWBShield.h: No such file or directory
# (This is expected until vendor libraries are installed)
```

---

## Summary

✅ **Arduino CLI:** Installed and configured
✅ **Board Cores:** Portenta C33 and Stella installed
✅ **Agent Updated:** Arduino Expert now uses CLI
✅ **Documentation:** Complete setup guide created
✅ **Scripts:** Batch flashing scripts provided
⚠️ **Pending:** UWB libraries (vendor-specific)

**Status:** Ready for hardware deployment once UWB libraries are obtained from vendor.

---

**Reference Documents:**
- Setup Guide: `arduino/ARDUINO_CLI_SETUP.md`
- System Overview: `UWB_SYSTEM_COMPLETE.md`
- Agent Definition: `.claude/.agents/arduino_expert.md`
