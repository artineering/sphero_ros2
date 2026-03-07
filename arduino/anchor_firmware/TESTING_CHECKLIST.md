# Anchor WiFi Firmware Testing Checklist

Use this checklist to verify each anchor after flashing the WiFi firmware.

## Pre-Flash Verification

- [ ] Verified ANCHOR_ID is unique (1, 2, 3, or 4)
- [ ] Verified WIFI_SSID matches your network name
- [ ] Verified WIFI_PASS is correct
- [ ] Verified UDP_HOST is the correct ROS2 host IP address
- [ ] Verified NUM_TAGS matches your deployment
- [ ] Selected correct board: Arduino Portenta C33
- [ ] Selected correct port: /dev/ttyACM* (Linux) or COM* (Windows)

## Phase 1: Single Anchor, No Tags (Bench Test)

### Anchor Hardware Setup
- [ ] Portenta C33 board with UWB Shield attached
- [ ] Connected to computer via USB-C cable
- [ ] Power LED is on

### Flash and Boot Test
- [ ] Firmware compiled without errors
- [ ] Firmware uploaded successfully
- [ ] Opened Serial Monitor at 115200 baud
- [ ] Saw "# Anchor X — Initializing UWB..."
- [ ] Saw "# UWB ready."
- [ ] Saw 10 session creation messages (or NUM_TAGS sessions)
- [ ] Saw "# All sessions started. Ranging active."

### WiFi Connection Test
- [ ] Saw "# Connecting to WiFi: [SSID]..."
- [ ] Saw "# WiFi connected! IP: x.x.x.x" within 10 seconds
- [ ] Saw "# RSSI: [value] dBm"
- [ ] Saw "# UDP initialized."
- [ ] LED turned green after WiFi connected
- [ ] Noted anchor IP address: ___________________

### WiFi Signal Strength
- [ ] RSSI is greater than -70 dBm (good signal)
- [ ] If RSSI < -70 dBm, moved anchor closer to access point

### Heartbeat Test
- [ ] Saw "# heartbeat anchor=X uptime_s=Y wifi=connected..."
- [ ] Heartbeat appears every 5 seconds
- [ ] Green LED blinks every 5 seconds
- [ ] WiFi status shows "connected" in heartbeat
- [ ] IP address appears in heartbeat

### UDP Socket Test
On ROS2 host, open terminal and run:
```bash
nc -u -l 5000
```
- [ ] nc command started (listening on UDP port 5000)
- [ ] No error messages (port not in use)

Keep nc running for next phase.

## Phase 2: Single Anchor, One Tag (Ranging Test)

### Tag Hardware Setup
- [ ] Powered on one Stella tag (any tag ID)
- [ ] Tag is within 10 meters of anchor
- [ ] Tag LED indicates it's running

### UDP Packet Reception
In the nc terminal:
- [ ] Saw JSON packets appearing (format: `{"anchor_id":1,"tag_id":X,"distance_cm":Y,"timestamp_ms":Z}`)
- [ ] anchor_id matches this anchor's ANCHOR_ID
- [ ] tag_id is between 1 and NUM_TAGS
- [ ] distance_cm is a reasonable value (not 0xFFFF or 0)
- [ ] timestamp_ms is increasing
- [ ] Packets arrive continuously (1-10 Hz)

### Serial Monitor (if SERIAL_DEBUG enabled)
- [ ] Serial Monitor shows same JSON packets as UDP
- [ ] Packet format is identical to UDP output

### Range Accuracy Test
- [ ] Moved tag to known distance (e.g., 1 meter = 100 cm)
- [ ] distance_cm value approximately matches (±20 cm tolerance)
- [ ] Moved tag to 2 meters, distance_cm increased
- [ ] Moved tag to 3 meters, distance_cm increased

### Multi-Tag Test (if available)
- [ ] Powered on second tag
- [ ] Saw packets with tag_id=2 (or different from first tag)
- [ ] Both tag_id values appear in UDP stream
- [ ] distance_cm values are independent per tag

Stop nc (Ctrl+C).

## Phase 3: WiFi Resilience Test

### Disconnection Test
- [ ] Anchor is running and ranging
- [ ] Restarted nc listener: `nc -u -l 5000`
- [ ] Unplugged WiFi access point or disabled WiFi
- [ ] Saw "# WiFi connection lost!" in Serial Monitor
- [ ] LED turned red (or stopped blinking green)
- [ ] Heartbeat shows "wifi=disconnected"

### Reconnection Test
- [ ] Reconnected WiFi access point or re-enabled WiFi
- [ ] Waited up to 5 seconds
- [ ] Saw "# WiFi disconnected, attempting reconnect..." in Serial Monitor
- [ ] Saw "# Connecting to WiFi..."
- [ ] Saw "# WiFi connected! IP: x.x.x.x"
- [ ] LED turned green
- [ ] Heartbeat shows "wifi=connected"
- [ ] UDP packets resumed in nc terminal

## Phase 4: Multi-Anchor Test (Full Deployment)

### Setup
- [ ] Flashed all 4 anchors with unique ANCHOR_ID (1, 2, 3, 4)
- [ ] All anchors have same WIFI_SSID, WIFI_PASS, UDP_HOST, UDP_PORT
- [ ] All anchors powered on and showing green LED
- [ ] Recorded IP addresses:
  - Anchor 1: ___________________
  - Anchor 2: ___________________
  - Anchor 3: ___________________
  - Anchor 4: ___________________

### UDP Reception from All Anchors
On ROS2 host:
```bash
nc -u -l 5000
```
- [ ] Saw packets with anchor_id=1
- [ ] Saw packets with anchor_id=2
- [ ] Saw packets with anchor_id=3
- [ ] Saw packets with anchor_id=4
- [ ] Packets from all 4 anchors are interleaved

### Multi-Tag Test
- [ ] Powered on multiple Stella tags (2-10)
- [ ] Saw packets with various tag_id values (1, 2, 3, ...)
- [ ] Each tag appears with measurements from all 4 anchors
- [ ] Packet rate scales with number of tags (more tags = more packets)

### Network Traffic Analysis
```bash
sudo tcpdump -i any -n udp port 5000 -c 20
```
- [ ] Saw packets from all 4 anchor IPs
- [ ] No packet errors or warnings
- [ ] Source ports vary (ephemeral ports from anchors)
- [ ] Destination port is 5000

## Phase 5: Integration with ROS2 Node

### Launch ROS2 Localization Node
```bash
cd ~/ros2_ws  # or wherever your workspace is
source install/setup.bash
ros2 launch uwb_localization uwb_rtls.launch.py
```
- [ ] Node started without errors
- [ ] Saw "Starting UWB Localization Node" message
- [ ] Transport mode is "udp" (check config or node output)

### Topic Verification
```bash
ros2 topic echo /uwb/ranges
```
- [ ] Saw UWBRange messages appearing
- [ ] anchor_id field is 1, 2, 3, or 4
- [ ] tag_id field is 1 through NUM_TAGS
- [ ] distance_m is a decimal value (in meters)
- [ ] timestamp is recent

### Position Estimation (if 4 anchors + calibration done)
```bash
ros2 topic echo /uwb/tag_1/pose2d
```
- [ ] Saw TagPose2D messages (requires ≥4 anchors + positions configured)
- [ ] x and y coordinates are reasonable (within arena bounds)
- [ ] num_anchors is 4 (or number of anchors in view)
- [ ] residual is small (<0.5 for good multilateration)

## Phase 6: Performance Test

### Long-Duration Test (1 hour)
- [ ] All 4 anchors running
- [ ] Multiple tags powered on
- [ ] ROS2 node running
- [ ] Started at: ___________________
- [ ] Ended at: ___________________
- [ ] No firmware crashes (check Serial Monitor)
- [ ] No ROS2 node crashes (check terminal)
- [ ] WiFi stayed connected on all anchors (check heartbeats)
- [ ] RSSI remained stable (check heartbeat logs)

### Packet Loss Analysis
In ROS2 node diagnostics (if implemented):
- [ ] Checked `/uwb/diagnostics` topic
- [ ] Packet loss rate < 5%
- [ ] All anchors show "connected" status

## Phase 7: Edge Cases

### Incorrect WiFi Credentials Test
- [ ] Flashed anchor with wrong WiFi password
- [ ] Saw "# WiFi connection failed!" in Serial Monitor
- [ ] LED is red (not green)
- [ ] Heartbeat shows "wifi=disconnected"
- [ ] No UDP packets arrive at ROS2 host
- [ ] Re-flashed with correct credentials
- [ ] WiFi connected successfully

### ROS2 Host Unreachable Test
- [ ] Anchor connected to WiFi (green LED)
- [ ] Changed UDP_HOST to non-existent IP (e.g., 192.168.1.254)
- [ ] Re-flashed anchor
- [ ] Anchor still shows "WiFi connected" (WiFi is OK)
- [ ] No UDP packets arrive (destination unreachable)
- [ ] Anchor does NOT crash (UDP send is non-blocking)
- [ ] Re-flashed with correct UDP_HOST

### Out-of-Range Tag Test
- [ ] Tag is in range, packets arriving
- [ ] Moved tag >50 meters away or powered it off
- [ ] Packets for that tag_id stopped appearing
- [ ] No "0xFFFF" or invalid distance values transmitted
- [ ] Anchor continues ranging other tags normally
- [ ] Brought tag back in range
- [ ] Packets resumed

### Power Cycle Test
- [ ] Anchor running and ranging
- [ ] Unplugged USB power
- [ ] Waited 5 seconds
- [ ] Reconnected USB power
- [ ] Anchor rebooted (saw init messages)
- [ ] WiFi reconnected automatically
- [ ] Ranging resumed
- [ ] UDP packets resumed

## Sign-Off

**Tester Name:** ___________________
**Date:** ___________________
**Anchor ID Tested:** ___________________

**Overall Result:**
- [ ] PASS — All critical tests passed, ready for deployment
- [ ] PASS with notes — Minor issues noted below, but functional
- [ ] FAIL — Critical issues, see notes below

**Notes/Issues:**
___________________________________________________________________________
___________________________________________________________________________
___________________________________________________________________________
___________________________________________________________________________

**Next Steps:**
- [ ] Proceed to deploy anchor in field
- [ ] Re-test after fixing issues
- [ ] Escalate to engineering

---

## Quick Reference Commands

### Listen for UDP packets:
```bash
nc -u -l 5000
```

### Capture UDP traffic:
```bash
sudo tcpdump -i any -n udp port 5000 -A
```

### Check ROS2 topics:
```bash
ros2 topic list
ros2 topic echo /uwb/ranges
ros2 topic echo /uwb/tag_1/pose2d
```

### Find ROS2 host IP:
```bash
hostname -I
```

### Check WiFi signal from anchor:
- Look for "rssi=" in heartbeat messages (Serial Monitor)
- Good: > -60 dBm
- Fair: -60 to -70 dBm
- Poor: < -70 dBm (may drop packets)
