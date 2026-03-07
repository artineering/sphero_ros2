# WiFi Anchor Configuration Guide

## Quick Start

Before flashing each anchor, you MUST configure these parameters in `anchor_firmware.ino`:

### Per-Anchor Configuration (UNIQUE for each anchor)

```cpp
#define ANCHOR_ID       1       // Change to 1, 2, 3, or 4
```

**Important:** Each of the 4 anchors must have a different ANCHOR_ID (1-4).

### Network Configuration (SAME for all anchors)

```cpp
#define WIFI_SSID       "YourNetworkName"     // Your WiFi network name
#define WIFI_PASS       "YourPassword"        // Your WiFi password
#define UDP_HOST        "192.168.1.100"       // Your ROS2 host IP address
#define UDP_PORT        5000                  // UDP port (default: 5000)
```

### Finding Your ROS2 Host IP Address

On your ROS2 computer, run:
```bash
hostname -I
```
or
```bash
ip addr show
```

Use the IP address that matches your WiFi network subnet (e.g., 192.168.1.x).

## Configuration Example

For a deployment with:
- WiFi Network: "MyLab5G"
- WiFi Password: "MySecurePass123"
- ROS2 Host IP: 192.168.1.50
- 4 Anchors

### Anchor 1 Configuration
```cpp
#define ANCHOR_ID       1
#define WIFI_SSID       "MyLab5G"
#define WIFI_PASS       "MySecurePass123"
#define UDP_HOST        "192.168.1.50"
#define UDP_PORT        5000
```

### Anchor 2 Configuration
```cpp
#define ANCHOR_ID       2       // <-- Only change this
#define WIFI_SSID       "MyLab5G"
#define WIFI_PASS       "MySecurePass123"
#define UDP_HOST        "192.168.1.50"
#define UDP_PORT        5000
```

### Anchor 3 Configuration
```cpp
#define ANCHOR_ID       3       // <-- Only change this
#define WIFI_SSID       "MyLab5G"
#define WIFI_PASS       "MySecurePass123"
#define UDP_HOST        "192.168.1.50"
#define UDP_PORT        5000
```

### Anchor 4 Configuration
```cpp
#define ANCHOR_ID       4       // <-- Only change this
#define WIFI_SSID       "MyLab5G"
#define WIFI_PASS       "MySecurePass123"
#define UDP_HOST        "192.168.1.50"
#define UDP_PORT        5000
```

## Additional Configuration Options

### Number of Tags
```cpp
#define NUM_TAGS        10      // Set to your actual number of Stella tags
```

### Debug Output
```cpp
#define SERIAL_DEBUG            // Comment out to disable serial debug output
```

When `SERIAL_DEBUG` is defined, the anchor will print JSON packets to the Serial Monitor (115200 baud) in addition to sending them via UDP. This is useful for debugging.

## Network Requirements

### WiFi Access Point
- All 4 anchors and the ROS2 host must be on the same WiFi network
- All devices must be on the same subnet (e.g., 192.168.1.x)
- 2.4 GHz or 5 GHz WiFi supported

### Firewall
- Ensure UDP port 5000 is not blocked on the ROS2 host
- The anchors send UDP packets to the host (no incoming connections required)

### Power
- WiFi adds ~80-150mA power consumption
- Use USB-C power or external 5V supply rated for at least 600mA

## Testing After Flashing

### 1. Serial Monitor Test
1. Open Arduino IDE Serial Monitor at 115200 baud
2. Expected output:
   ```
   # Anchor 1 — Initializing UWB...
   # UWB ready.
   # Session created: anchor=1 tag=1 sessionId=0x10001 preamble=9
   ...
   # All sessions started. Ranging active.
   # Connecting to WiFi: MyLab5G...
   # WiFi connected! IP: 192.168.1.123
   # RSSI: -45 dBm
   # UDP initialized.
   # heartbeat anchor=1 uptime_s=10 wifi=connected ip=192.168.1.123 rssi=-45dBm
   ```

### 2. UDP Packet Test
On your ROS2 host, listen for UDP packets:
```bash
nc -u -l 5000
```

When a Stella tag is in range, you should see JSON packets:
```json
{"anchor_id":1,"tag_id":1,"distance_cm":142,"timestamp_ms":12345}
{"anchor_id":1,"tag_id":2,"distance_cm":238,"timestamp_ms":12456}
```

Press Ctrl+C to stop.

### 3. Network Traffic Test
Verify packets are arriving from all 4 anchors:
```bash
sudo tcpdump -i any -n udp port 5000 -A
```

You should see packets from 4 different source IPs (the anchors).

## LED Status Indicators

- **Red (solid):** Initialization or WiFi connection failed
- **Blue (solid):** WiFi connection in progress
- **Green (solid):** Running, WiFi connected, UWB ranging active
- **Green (blinking):** Heartbeat (blinks every 5 seconds)

## Troubleshooting

### WiFi Won't Connect
- Verify SSID and password are correct (case-sensitive)
- Check that WiFi access point is powered on and in range
- Check LED: should turn blue during connection, then green on success
- Serial Monitor shows "# WiFi connection failed!" if credentials are wrong

### No UDP Packets Received
- Verify ROS2 host IP is correct and on the same subnet
- Check firewall settings on ROS2 host: `sudo ufw status`
- Test with: `nc -u -l 5000` on the ROS2 host
- Check anchor Serial Monitor for "# WiFi connected! IP: x.x.x.x"

### WiFi Signal Weak
- Check RSSI in heartbeat messages (should be > -70 dBm)
- Move anchor closer to WiFi access point
- Use 5 GHz WiFi if available (less congestion)

### Anchor Disconnects Randomly
- Check RSSI (if < -80 dBm, signal is too weak)
- Verify power supply is adequate (600mA minimum)
- Check for WiFi interference from other devices

## Advanced: Static IP Configuration

The current firmware uses DHCP. For static IP configuration, modify `connectWiFi()`:

```cpp
IPAddress staticIP(192, 168, 1, 101);  // Anchor 1
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 1, 1);

WiFi.config(staticIP, dns, gateway, subnet);
WiFi.begin(WIFI_SSID, WIFI_PASS);
```

Use unique IPs for each anchor (101, 102, 103, 104).

## Support

For issues, check:
1. Serial Monitor output (115200 baud)
2. LED indicators
3. Network connectivity with `ping` and `tcpdump`
4. ROS2 node logs: `ros2 launch uwb_localization uwb_rtls.launch.py`
