# Arduino CLI Setup Complete

## Installation Summary

✅ **Arduino CLI installed and configured**

### Installation Details

**Version:** 1.4.1
**Location:** `/home/svaghela/ros2_ws_2/bin/arduino-cli`
**Config:** `~/.arduino15/arduino-cli.yaml`
**PATH:** Added to `~/.bashrc`

### Installed Board Cores

1. **arduino:renesas_portenta@1.5.3**
   - For Arduino Portenta C33
   - FQBN: `arduino:renesas_portenta:portenta_c33`

2. **arduino:mbed_stella@4.5.0**
   - For Arduino Stella
   - FQBN: `arduino:mbed_stella:stella`

### Verification Commands

```bash
# Verify installation
arduino-cli version

# List installed cores
arduino-cli core list

# List available boards
arduino-cli board listall
```

Expected output:
```
arduino-cli  Version: 1.4.1 Commit: e39419312 Date: 2026-01-19T16:13:12Z

Board Name           FQBN
Arduino Portenta C33 arduino:renesas_portenta:portenta_c33
Arduino Stella       arduino:mbed_stella:stella
```

---

## Quick Start Guide

### 1. Compile Firmware

**Anchor Firmware (Portenta C33):**
```bash
cd /home/svaghela/ros2_ws_2
arduino-cli compile --fqbn arduino:renesas_portenta:portenta_c33 arduino/anchor_firmware
```

**Tag Firmware (Stella):**
```bash
cd /home/svaghela/ros2_ws_2
arduino-cli compile --fqbn arduino:mbed_stella:stella arduino/tag_firmware
```

### 2. List Connected Boards

```bash
arduino-cli board list
```

This will show:
- Port (e.g., `/dev/ttyACM0`)
- Protocol
- Board type (if recognized)
- FQBN

### 3. Upload to Board

**Single Upload:**
```bash
# Auto-detect port
arduino-cli upload --fqbn arduino:renesas_portenta:portenta_c33 arduino/anchor_firmware

# Specific port
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:renesas_portenta:portenta_c33 arduino/anchor_firmware
```

**Compile and Upload (one command):**
```bash
arduino-cli compile --upload --fqbn arduino:renesas_portenta:portenta_c33 -p /dev/ttyACM0 arduino/anchor_firmware
```

### 4. Monitor Serial Output

```bash
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
```

Press `Ctrl+C` to exit serial monitor.

---

## Batch Flashing Scripts

### Flash 4 Anchors (Interactive)

Create a script `flash_anchors.sh`:

```bash
#!/bin/bash
cd /home/svaghela/ros2_ws_2

for ANCHOR_ID in {1..4}; do
  echo "==================================="
  echo "Flashing Anchor $ANCHOR_ID"
  echo "==================================="

  # Update ANCHOR_ID in firmware
  sed -i "s/#define ANCHOR_ID.*/#define ANCHOR_ID $ANCHOR_ID/" arduino/anchor_firmware/anchor_firmware.ino

  # Show what was configured
  grep "#define ANCHOR_ID" arduino/anchor_firmware/anchor_firmware.ino

  # Wait for user to connect board
  echo "Connect Anchor $ANCHOR_ID to USB"
  read -p "Press Enter when ready..."

  # Detect port
  PORT=$(arduino-cli board list | grep "Portenta C33" | awk '{print $1}')

  if [ -z "$PORT" ]; then
    echo "ERROR: Portenta C33 not detected!"
    echo "Available ports:"
    arduino-cli board list
    read -p "Enter port manually (e.g., /dev/ttyACM0): " PORT
  fi

  echo "Using port: $PORT"

  # Compile and upload
  arduino-cli compile --upload --fqbn arduino:renesas_portenta:portenta_c33 -p $PORT arduino/anchor_firmware

  if [ $? -eq 0 ]; then
    echo "✓ Anchor $ANCHOR_ID flashed successfully"
  else
    echo "✗ Anchor $ANCHOR_ID failed to flash"
    exit 1
  fi

  echo ""
  echo "Disconnect Anchor $ANCHOR_ID"
  echo ""
done

echo "==================================="
echo "All 4 anchors flashed successfully!"
echo "==================================="
```

Make executable:
```bash
chmod +x flash_anchors.sh
./flash_anchors.sh
```

### Flash 10 Tags (Interactive)

Create a script `flash_tags.sh`:

```bash
#!/bin/bash
cd /home/svaghela/ros2_ws_2

for TAG_ID in {1..10}; do
  echo "==================================="
  echo "Flashing Tag $TAG_ID"
  echo "==================================="

  # Update TAG_ID in firmware
  sed -i "s/#define TAG_ID.*/#define TAG_ID $TAG_ID/" arduino/tag_firmware/tag_firmware_optimized.ino

  # Show what was configured
  grep "#define TAG_ID" arduino/tag_firmware/tag_firmware_optimized.ino

  # Wait for user to connect board
  echo "Connect Tag $TAG_ID (Stella) to USB"
  read -p "Press Enter when ready..."

  # Detect port
  PORT=$(arduino-cli board list | grep "Stella" | awk '{print $1}')

  if [ -z "$PORT" ]; then
    echo "ERROR: Arduino Stella not detected!"
    echo "Available ports:"
    arduino-cli board list
    read -p "Enter port manually (e.g., /dev/ttyACM0): " PORT
  fi

  echo "Using port: $PORT"

  # Compile and upload
  arduino-cli compile --upload --fqbn arduino:mbed_stella:stella -p $PORT arduino/tag_firmware

  if [ $? -eq 0 ]; then
    echo "✓ Tag $TAG_ID flashed successfully"
  else
    echo "✗ Tag $TAG_ID failed to flash"
    exit 1
  fi

  echo ""
  echo "Disconnect Tag $TAG_ID and label it!"
  echo ""
done

echo "==================================="
echo "All 10 tags flashed successfully!"
echo "==================================="
```

Make executable:
```bash
chmod +x flash_tags.sh
./flash_tags.sh
```

---

## Troubleshooting

### Board Not Detected

**Problem:** `arduino-cli board list` doesn't show your board

**Solutions:**
1. Check USB connection
2. Try a different USB port
3. Check device permissions:
   ```bash
   ls -l /dev/ttyACM*
   # If permission denied, add user to dialout group:
   sudo usermod -a -G dialout $USER
   # Log out and log back in
   ```
4. Check if board is in bootloader mode (double-tap reset button)

### Compilation Errors

**Problem:** Firmware won't compile

**Solutions:**
1. Check library dependencies:
   ```bash
   arduino-cli lib list
   ```
2. Use verbose output to see errors:
   ```bash
   arduino-cli compile --verbose --fqbn arduino:renesas_portenta:portenta_c33 arduino/anchor_firmware
   ```
3. Clean build cache:
   ```bash
   arduino-cli cache clean
   ```

### Upload Fails

**Problem:** Compilation succeeds but upload fails

**Solutions:**
1. Verify correct port:
   ```bash
   arduino-cli board list
   ```
2. Try manual bootloader mode (double-tap reset button, LED should pulse)
3. Check board is not being used by another program (serial monitor, etc.)
4. Try upload with verbose output:
   ```bash
   arduino-cli upload --verbose -p /dev/ttyACM0 --fqbn arduino:renesas_portenta:portenta_c33 arduino/anchor_firmware
   ```

### Libraries Missing

**Problem:** Code includes libraries that aren't found

**Note:** The UWB libraries (PortentaUWBShield, StellaUWB) are **not** available in the Arduino Library Manager. These are vendor-specific libraries that must be installed manually or are included with the board core.

**If you need additional libraries:**
```bash
# Search for library
arduino-cli lib search "LibraryName"

# Install library
arduino-cli lib install "LibraryName"

# List installed libraries
arduino-cli lib list
```

---

## Next Steps

1. **Test Compilation:**
   ```bash
   cd /home/svaghela/ros2_ws_2
   arduino-cli compile --fqbn arduino:renesas_portenta:portenta_c33 arduino/anchor_firmware
   arduino-cli compile --fqbn arduino:mbed_stella:stella arduino/tag_firmware
   ```

2. **Connect Hardware:**
   - Connect one Portenta C33 board via USB
   - Run `arduino-cli board list` to verify detection

3. **Configure Firmware:**
   - Edit WiFi credentials in `arduino/anchor_firmware/anchor_firmware.ino`
   - Set appropriate ANCHOR_ID or TAG_ID

4. **Flash and Test:**
   - Use the batch scripts above for multiple boards
   - Or flash individually with upload commands

5. **Monitor Output:**
   ```bash
   arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
   ```

---

## Arduino Expert Agent Updated

The Arduino Expert agent (`.claude/.agents/arduino_expert.md`) has been updated with:
- ✅ Arduino CLI command reference
- ✅ Board-specific FQBNs
- ✅ Batch flashing procedures
- ✅ Troubleshooting commands
- ✅ Required usage in all plans

When you delegate Arduino tasks to the Arduino Expert agent, it will now automatically include Arduino CLI commands in its plans and deliverables.

---

## Additional Resources

**Arduino CLI Documentation:**
- Official docs: https://arduino.github.io/arduino-cli/
- Command reference: https://arduino.github.io/arduino-cli/commands/

**Board Information:**
- Portenta C33: https://docs.arduino.cc/hardware/portenta-c33
- Stella: https://docs.arduino.cc/hardware/stella

**Installed Tools:**
```bash
# Show all installed tools
ls ~/.arduino15/packages/
```

---

**Setup Complete!** Arduino CLI is ready for flashing Portenta C33 anchors and Stella tags.
