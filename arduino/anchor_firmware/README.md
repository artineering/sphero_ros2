# UWB Anchor WiFi/UDP Firmware

Arduino firmware for Portenta C33 + UWB Shield anchors with WiFi/UDP transport.

## Quick Start

### 1. Install Arduino IDE and Libraries
- Install Arduino IDE 2.x
- Install board support: "Arduino Portenta C33"
- Install library: "PortentaUWBShield" by Truesense

### 2. Configure Firmware
Open `anchor_firmware.ino` and modify these lines:

```cpp
#define ANCHOR_ID       1                     // Change to 1, 2, 3, or 4
#define WIFI_SSID       "YourNetworkName"     // Your WiFi name
#define WIFI_PASS       "YourPassword"        // Your WiFi password
#define UDP_HOST        "192.168.1.100"       // Your ROS2 host IP
```

**Critical:** Each anchor must have a unique `ANCHOR_ID` (1-4).

### 3. Flash to Hardware
1. Connect Portenta C33 via USB-C
2. Select board: **Arduino Portenta C33**
3. Select port: **/dev/ttyACM0** (or similar)
4. Click **Upload**
5. Open Serial Monitor @ 115200 baud
6. Verify "WiFi connected!" message

### 4. Test UDP Connection
On your ROS2 host:
```bash
nc -u -l 5000
```

You should see JSON packets when tags are in range:
```json
{"anchor_id":1,"tag_id":1,"distance_cm":142,"timestamp_ms":12345}
```

## Documentation

| File | Purpose |
|------|---------|
| **anchor_firmware.ino** | Main firmware (flash this to anchors) |
| **WIFI_CONFIG_GUIDE.md** | Detailed WiFi configuration instructions |
| **TESTING_CHECKLIST.md** | Step-by-step testing procedure |
| **CHANGES_FROM_ORIGINAL.md** | What changed from serial version |

## LED Status

- **Red:** WiFi connection failed
- **Blue:** WiFi connecting
- **Green (solid):** Running, WiFi connected
- **Green (blinking):** Heartbeat (every 5 seconds)

## Hardware Requirements

- **Board:** Arduino Portenta C33
- **Shield:** Portenta UWB Shield
- **Power:** USB-C or 5V external (≥600mA recommended)
- **Network:** WiFi access point (2.4 GHz or 5 GHz)

## Network Requirements

- All 4 anchors and ROS2 host on same WiFi network
- Same subnet (e.g., 192.168.1.x)
- UDP port 5000 accessible
- Firewall allows UDP traffic

## Configuration Options

### Required Settings
```cpp
#define ANCHOR_ID       1           // UNIQUE per anchor (1-4)
#define WIFI_SSID       "..."       // WiFi network name
#define WIFI_PASS       "..."       // WiFi password
#define UDP_HOST        "..."       // ROS2 host IP
```

### Optional Settings
```cpp
#define NUM_TAGS        10          // Number of Stella tags
#define UDP_PORT        5000        // UDP destination port
#define WIFI_RETRY_MS   5000        // Reconnect interval
#define SERIAL_DEBUG                // Enable debug output (comment to disable)
```

## Troubleshooting

### WiFi Won't Connect
- Check LED: Red = failed, Blue = connecting, Green = connected
- Verify SSID and password (case-sensitive)
- Check Serial Monitor for error messages
- Ensure access point is in range (RSSI > -70 dBm)

### No UDP Packets
- Verify ROS2 host IP is correct
- Check firewall: `sudo ufw status`
- Test with: `nc -u -l 5000`
- Verify WiFi connected (Serial Monitor shows IP)

### Poor Range/Accuracy
- Check WiFi RSSI in heartbeat (should be > -70 dBm)
- Move anchor closer to WiFi access point
- Check UWB antennas are not obstructed
- Verify anchor positions are configured correctly

## Testing

Follow **TESTING_CHECKLIST.md** for comprehensive testing procedure.

Quick test:
```bash
# On ROS2 host
nc -u -l 5000

# Expected output (when tags in range):
{"anchor_id":1,"tag_id":1,"distance_cm":142,"timestamp_ms":12345}
```

## Integration with ROS2

After flashing all 4 anchors:

1. Configure ROS2 node (`rtls_config.yaml`):
   ```yaml
   transport: "udp"
   udp_port: 5000
   ```

2. Launch ROS2 localization:
   ```bash
   ros2 launch uwb_localization uwb_rtls.launch.py
   ```

3. Verify data:
   ```bash
   ros2 topic echo /uwb/ranges
   ```

## Source Information

- **Original firmware:** `/tmp/uwb_extracted/anchor_firmware.ino` (USB serial version)
- **Modified firmware:** This directory (WiFi/UDP version)
- **Plan document:** `/home/svaghela/ros2_ws_2/plans/uwb-anchor-wifi-firmware-2026-03-07T11-09-00.md`
- **WiFi integration plan:** `/home/svaghela/ros2_ws_2/uwb_wifi_foxglove_plan.md`

## Version History

- **v1.0** (2026-03-07): Initial WiFi/UDP implementation
  - Added WiFi connectivity (ESP32-C3 module)
  - Added UDP transmission
  - Added automatic reconnection
  - Added WiFi diagnostics in heartbeat
  - Preserved all UWB ranging functionality

## Support

For issues:
1. Check Serial Monitor output (115200 baud)
2. Follow TESTING_CHECKLIST.md
3. Review WIFI_CONFIG_GUIDE.md
4. Check LED status indicators
5. Verify network connectivity

## License

Same as original PortentaUWBShield examples.

## Hardware Specifications

### Portenta C33
- **MCU:** Renesas RA6M5 (Arm Cortex-M33, 200 MHz)
- **RAM:** 512 KB SRAM
- **Flash:** 2 MB
- **WiFi:** Built-in ESP32-C3 module
- **Power:** USB-C or 5V external

### Portenta UWB Shield
- **Chip:** DW3000 UWB transceiver
- **Interface:** SPI to Portenta C33
- **Frequency:** 3.5-6.5 GHz (UWB)
- **Range:** ~50 meters typical

## Power Consumption

| Mode | Current Draw |
|------|-------------|
| WiFi idle + UWB idle | ~280 mA |
| WiFi TX + UWB ranging | ~400 mA |
| Peak (WiFi connect) | ~450 mA |

**Recommendation:** Use power supply rated ≥600 mA.

## Files in This Directory

```
anchor_firmware/
├── anchor_firmware.ino          # Main firmware (flash this)
├── README.md                     # This file
├── WIFI_CONFIG_GUIDE.md         # Detailed configuration guide
├── TESTING_CHECKLIST.md         # Step-by-step testing
└── CHANGES_FROM_ORIGINAL.md    # Comparison with serial version
```

## Next Steps

1. **Configure:** Set ANCHOR_ID, WiFi credentials, ROS2 host IP
2. **Flash:** Upload to all 4 Portenta C33 boards
3. **Test:** Follow TESTING_CHECKLIST.md
4. **Deploy:** Place anchors at configured positions
5. **Integrate:** Launch ROS2 uwb_localization_node
6. **Visualize:** Connect Foxglove Studio to see tag positions

## Contact

For questions about the UWB RTLS system, see the main project documentation.
