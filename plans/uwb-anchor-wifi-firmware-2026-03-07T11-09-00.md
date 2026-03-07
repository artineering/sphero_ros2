# UWB Anchor WiFi/UDP Firmware Modification

**Created:** 2026-03-07T11:09:00
**Status:** Pending Approval
**Hardware:** Arduino Portenta C33 + Portenta UWB Shield
**Complexity:** Medium

## Task Description

Modify the existing UWB anchor firmware (currently using USB serial communication) to transmit range measurements via WiFi/UDP instead. The firmware will connect to a WiFi network and send JSON-formatted range data to a ROS2 host via UDP unicast.

## Hardware Requirements

- **Board:** Arduino Portenta C33
  - MCU: Renesas RA6M5 (Arm Cortex-M33, 200 MHz)
  - RAM: 512 KB SRAM
  - Flash: 2 MB
  - WiFi Module: Built-in ESP32-C3 (managed via WiFi.h library)
  - LED: RGB LED (LEDR, LEDG, LEDB - active low)

- **Shield:** Portenta UWB Shield
  - DW3000 UWB transceiver
  - SPI interface to Portenta C33
  - Already configured in existing firmware

- **Network Requirements:**
  - WiFi access point (2.4 GHz or 5 GHz)
  - All anchors and ROS2 host on same subnet
  - UDP port 5000 accessible (default)

- **Pin Assignments:** No changes required
  - UWB Shield uses dedicated SPI pins (managed by PortentaUWBShield library)
  - RGB LED pins: LEDR, LEDG, LEDB (already in use)
  - WiFi handled by ESP32-C3 via internal communication

- **Power Requirements:** No change from existing setup
  - USB-C power or external 5V supply
  - WiFi module adds ~80-150mA when transmitting

## Analysis

### Current State
The existing firmware (`/tmp/uwb_extracted/anchor_firmware.ino`):
- Uses `PortentaUWBShield.h` library for DW3000 UWB communication
- Creates TWR (Two-Way Ranging) sessions to multiple tags
- Outputs JSON-formatted range data via USB serial at 115200 baud
- Uses RGB LED for status indication (red=init, green=running)
- Implements heartbeat every 5 seconds
- Configurable: ANCHOR_ID (1-4), NUM_TAGS (default 10), preamble codes

### Required Changes
1. Add WiFi connectivity using ESP32-C3 module (WiFi.h library)
2. Replace Serial.print() with UDP transmission
3. Implement WiFi reconnection logic
4. Add WiFi diagnostics (RSSI, IP address)
5. Maintain serial debug output (optional via #ifdef)

### Hardware Constraints
- **RAM:** With 10 UWB sessions active, each ~1-2KB, plus WiFi stack (~40KB), total usage ~60-80KB - well within 512KB limit
- **Timing:** WiFi operations (connect, send) are blocking but short (<10ms per UDP send). UWB ranging callbacks fire asynchronously and will not be blocked.
- **Power:** WiFi adds power draw; ensure adequate power supply (recommend USB-C or >500mA external)

### Network Architecture
- 4 anchors send to the **same UDP port** (5000) on ROS2 host
- Each packet includes `anchor_id` field to identify source
- No acknowledgment protocol (UDP is fire-and-forget)
- Typical packet size: ~80 bytes JSON + UDP/IP/WiFi overhead = ~130 bytes total

## Detailed Plan

### Step 1: Add WiFi Configuration Defines

**Action:** Add user-configurable WiFi/UDP parameters to the top of the sketch

**Files:** `/tmp/uwb_extracted/anchor_firmware.ino` (modify)

**Code to Add (after existing config section):**
```cpp
// WiFi and UDP configuration
#define WIFI_SSID       "YourNetworkName"     // Change this to your WiFi SSID
#define WIFI_PASS       "YourPassword"        // Change this to your WiFi password
#define UDP_HOST        "192.168.1.100"       // ROS2 host IP address
#define UDP_PORT        5000                  // UDP destination port
#define WIFI_RETRY_MS   5000                  // Reconnect interval (ms)
#define SERIAL_DEBUG                          // Comment out to disable debug prints
```

**Expected Outcome:** Clear user configuration section for WiFi credentials

---

### Step 2: Add WiFi Library Includes and Global Objects

**Action:** Include WiFi libraries and declare global UDP objects

**Files:** `/tmp/uwb_extracted/anchor_firmware.ino` (modify)

**Code to Add (after #include <PortentaUWBShield.h>):**
```cpp
#include <WiFi.h>
#include <WiFiUdp.h>
```

**Code to Add (before session array declaration):**
```cpp
// WiFi and UDP objects
WiFiUDP udp;
bool wifiConnected = false;
unsigned long lastWifiAttempt = 0;
```

**Libraries Needed:** WiFi.h and WiFiUdp.h (built into Arduino Portenta C33 board support package)

**Expected Outcome:** WiFi libraries available, UDP object ready for use

---

### Step 3: Implement connectWiFi() Function

**Action:** Create a function to establish WiFi connection with timeout and LED feedback

**Files:** `/tmp/uwb_extracted/anchor_firmware.ino` (modify)

**Code to Add (before setup() function):**
```cpp
/**
 * Connect to WiFi with timeout and LED feedback
 * @return true if connected, false if timeout
 */
bool connectWiFi() {
  Serial.print("# Connecting to WiFi: ");
  Serial.print(WIFI_SSID);
  Serial.println("...");

#if defined(ARDUINO_PORTENTA_C33)
  // Blue LED = connecting
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, LOW);
#endif

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  // Wait up to 10 seconds for connection
  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 10000) {
    delay(100);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("# WiFi connected! IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("# RSSI: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");

#if defined(ARDUINO_PORTENTA_C33)
    // Green LED = connected
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, LOW);
    digitalWrite(LEDB, HIGH);
#endif

    wifiConnected = true;
    lastWifiAttempt = millis();
    return true;
  } else {
    Serial.println("# WiFi connection failed!");

#if defined(ARDUINO_PORTENTA_C33)
    // Red LED = failed
    digitalWrite(LEDR, LOW);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, HIGH);
#endif

    wifiConnected = false;
    lastWifiAttempt = millis();
    return false;
  }
}
```

**Expected Outcome:** WiFi connection with visual and serial feedback

**Timing Considerations:**
- Connection attempt: up to 10 seconds (blocking)
- Runs during setup() before ranging starts, so no UWB impact
- Reconnection runs in loop() with 5-second minimum interval

---

### Step 4: Modify setup() to Initialize WiFi

**Action:** Add WiFi connection and UDP initialization to setup()

**Files:** `/tmp/uwb_extracted/anchor_firmware.ino` (modify)

**Code to Add (after "All sessions started" message, before LED set to green):**
```cpp
  // Initialize WiFi
  connectWiFi();

  // Begin UDP (required for sending on some WiFi stacks)
  udp.begin(UDP_PORT);

  Serial.println("# UDP initialized.");
```

**Expected Outcome:** WiFi connected and UDP ready before ranging begins

---

### Step 5: Modify rangingHandler() for UDP Transmission

**Action:** Replace Serial.print() JSON building with UDP transmission, keep serial debug optional

**Files:** `/tmp/uwb_extracted/anchor_firmware.ino` (modify)

**Code to Replace (lines 52-63):**
```cpp
  for (int j = 0; j < rangingData.available(); j++) {
    if (twr[j].status == 0 && twr[j].distance != 0xFFFF) {
      // Build JSON message
      char buffer[128];
      snprintf(buffer, sizeof(buffer),
               "{\"anchor_id\":%d,\"tag_id\":%d,\"distance_cm\":%u,\"timestamp_ms\":%lu}",
               ANCHOR_ID, tag_id, twr[j].distance, millis());

      // Send via UDP if WiFi connected
      if (wifiConnected && WiFi.status() == WL_CONNECTED) {
        udp.beginPacket(UDP_HOST, UDP_PORT);
        udp.write((const uint8_t*)buffer, strlen(buffer));
        udp.endPacket();
      }

#ifdef SERIAL_DEBUG
      // Debug output to serial
      Serial.println(buffer);
#endif
    }
  }
```

**Expected Outcome:** Range measurements transmitted via UDP, optional serial debug

**Timing Considerations:**
- `udp.endPacket()` is typically non-blocking or <5ms
- Fires in UWB interrupt context - minimal blocking time is critical
- If timing issues arise, consider queueing packets for transmission in loop()

---

### Step 6: Add WiFi Reconnection Logic to loop()

**Action:** Monitor WiFi status and attempt reconnection if disconnected

**Files:** `/tmp/uwb_extracted/anchor_firmware.ino` (modify)

**Code to Add (inside loop(), before heartbeat section):**
```cpp
  // WiFi reconnection logic
  if (WiFi.status() != WL_CONNECTED && !wifiConnected) {
    if (millis() - lastWifiAttempt > WIFI_RETRY_MS) {
      Serial.println("# WiFi disconnected, attempting reconnect...");
      connectWiFi();
    }
  } else if (WiFi.status() == WL_CONNECTED && !wifiConnected) {
    // WiFi came back up
    wifiConnected = true;
  } else if (WiFi.status() != WL_CONNECTED && wifiConnected) {
    // WiFi just dropped
    Serial.println("# WiFi connection lost!");
    wifiConnected = false;
    lastWifiAttempt = millis();
  }
```

**Expected Outcome:** Automatic WiFi reconnection with 5-second retry interval

---

### Step 7: Enhance Heartbeat with WiFi Diagnostics

**Action:** Add WiFi status and RSSI to heartbeat messages

**Files:** `/tmp/uwb_extracted/anchor_firmware.ino` (modify)

**Code to Replace (lines 144-148):**
```cpp
  if (millis() - lastHeartbeat > 5000) {
    Serial.print("# heartbeat anchor=");
    Serial.print(ANCHOR_ID);
    Serial.print(" uptime_s=");
    Serial.print(millis() / 1000);

    // Add WiFi diagnostics
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print(" wifi=connected ip=");
      Serial.print(WiFi.localIP());
      Serial.print(" rssi=");
      Serial.print(WiFi.RSSI());
      Serial.print("dBm");
    } else {
      Serial.print(" wifi=disconnected");
    }
    Serial.println();

    lastHeartbeat = millis();

#if defined(ARDUINO_PORTENTA_C33)
    digitalWrite(LEDG, !digitalRead(LEDG));
#endif
  }
```

**Expected Outcome:** Heartbeat includes WiFi connection status and signal strength

---

### Step 8: Testing and Validation

**Action:** Flash one anchor and verify UDP transmission

**Hardware Setup:**
1. Connect Portenta C33 + UWB Shield to computer via USB-C
2. Ensure WiFi access point is powered and accessible
3. Ensure ROS2 host is on same network

**Testing Procedure:**

**Test 1: Serial Monitor Verification**
```bash
# Open Arduino IDE Serial Monitor at 115200 baud
# Expected output:
# Anchor 1 — Initializing UWB...
# UWB ready.
# Session created: anchor=1 tag=1 sessionId=0x10001 preamble=9
# ...
# All sessions started. Ranging active.
# Connecting to WiFi: YourNetworkName...
# WiFi connected! IP: 192.168.1.123
# RSSI: -45 dBm
# UDP initialized.
# heartbeat anchor=1 uptime_s=10 wifi=connected ip=192.168.1.123 rssi=-45dBm
```

**Test 2: UDP Packet Reception**
```bash
# On ROS2 host, listen on UDP port 5000
nc -u -l 5000

# Expected output (when tags are in range):
# {"anchor_id":1,"tag_id":1,"distance_cm":142,"timestamp_ms":12345}
# {"anchor_id":1,"tag_id":2,"distance_cm":238,"timestamp_ms":12456}
```

**Test 3: Network Traffic Inspection**
```bash
# On ROS2 host, capture UDP packets
sudo tcpdump -i any -n udp port 5000 -X

# Verify packets from anchor IP (192.168.1.123)
```

**Test 4: WiFi Reconnection**
```
1. Power off WiFi access point
2. Verify serial output: "# WiFi connection lost!"
3. Power on WiFi access point
4. Wait up to 5 seconds
5. Verify serial output: "# Connecting to WiFi..." then "# WiFi connected!"
6. Verify UDP packets resume
```

**Test 5: Multi-Anchor Operation**
```
1. Flash 4 anchors with unique ANCHOR_ID (1-4)
2. All with same WIFI_SSID, WIFI_PASS, UDP_HOST, UDP_PORT
3. Power all simultaneously
4. On ROS2 host: nc -u -l 5000
5. Verify packets from all 4 anchors (identified by anchor_id field)
```

---

## Pin Configuration

| Pin/Resource | Function | Notes |
|--------------|----------|-------|
| USB-C | Serial debug @ 115200 baud | Optional after WiFi working |
| LEDR | Status: Red | Init phase or WiFi failure |
| LEDG | Status: Green | Running / heartbeat blink |
| LEDB | Status: Blue | WiFi connecting |
| SPI (internal) | UWB Shield communication | Managed by PortentaUWBShield library |
| ESP32-C3 (internal) | WiFi module | Managed by WiFi.h library |

**No external wiring changes required** - all communication is internal or via WiFi.

---

## Timing Considerations

### WiFi Connection Timing
- **Initial connect:** Up to 10 seconds (blocking in setup(), before ranging starts)
- **Reconnection:** Triggered every 5 seconds if disconnected (non-blocking, runs in loop())
- **UDP send:** <5ms per packet (typically 1-2ms)

### UWB Ranging Timing
- **TWR sessions:** Asynchronous, managed by UWB stack
- **Ranging callback:** Fires when measurement completes
- **Callback duration:** Previously ~1ms (Serial.print), now ~2-5ms (UDP send)
- **Impact:** Minimal - UWB library handles buffering

### Task Priority (No RTOS in this firmware)
- Main loop runs continuously
- UWB callbacks preempt loop (interrupt-driven)
- WiFi operations in loop() are brief (<10ms)

### Critical Path Analysis
**Worst case in rangingHandler():**
- JSON formatting: ~100μs (snprintf)
- UDP transmission: ~2-5ms (WiFi stack)
- Serial debug: ~8ms (if enabled, 80 chars @ 115200 baud)
- **Total:** ~5-13ms per range measurement

**Measurement rate:** Typically 1-10 Hz per tag, so 10-100ms between callbacks - no contention.

---

## Expected Outcomes

1. **WiFi Connection**
   - Anchor connects to WiFi on boot within 10 seconds
   - IP address printed to serial monitor
   - Blue LED during connection, green when connected

2. **UDP Transmission**
   - Range measurements sent to ROS2 host via UDP
   - JSON format identical to original serial output
   - No acknowledgment required (UDP unicast)

3. **Reconnection**
   - Automatic reconnection if WiFi drops
   - 5-second retry interval
   - Visual and serial feedback

4. **Diagnostics**
   - Heartbeat every 5 seconds with WiFi status and RSSI
   - Serial debug output (if SERIAL_DEBUG defined)
   - LED indicates WiFi status

5. **Multi-Anchor Support**
   - 4 anchors send to same UDP port
   - `anchor_id` field distinguishes sources
   - No packet collisions (UDP handles sequencing)

---

## Potential Risks & Considerations

### Hardware Risks
- **Power consumption:** WiFi adds 80-150mA. Ensure USB-C or external power supply can provide >500mA total.
- **Heat:** WiFi module generates heat. Portenta C33 has thermal management, but monitor temperature in enclosed deployments.
- **EMI:** WiFi (2.4/5GHz) and UWB (3.5-6.5GHz) are in different bands, minimal interference expected.

### Software Risks
- **Memory:** WiFi stack uses ~40KB heap. With 512KB SRAM available, no concern, but monitor if adding features.
- **Blocking:** `udp.endPacket()` may block briefly. If UWB ranging is disrupted, move UDP sends to loop() with a queue.
- **Packet loss:** UDP is unreliable. If critical, add sequence numbers or switch to TCP (higher latency).

### Network Risks
- **WiFi congestion:** 4 anchors + other devices may saturate WiFi. Use 5GHz if available, or wired Ethernet (future enhancement).
- **IP conflicts:** Ensure DHCP range accommodates all anchors, or use static IPs.
- **Firewall:** Ensure UDP port 5000 is open on ROS2 host.

### Timing Risks
- **WiFi reconnection:** During reconnection (up to 10s), ranging data is lost. Consider buffering in a queue (future enhancement).
- **ISR duration:** If `rangingHandler()` runs in interrupt context and UDP send is too slow, consider deferring to loop() via queue.

### Mitigation Strategies
- **Power:** Document minimum power requirements (5V @ 600mA recommended).
- **Packet loss:** ROS2 node should handle missing packets gracefully (already in design).
- **Debugging:** Keep `SERIAL_DEBUG` enabled during deployment phase, disable for production.
- **Buffering:** If packet loss is severe, implement a circular buffer in firmware to smooth out WiFi hiccups.

---

## Testing Plan

### Phase 1: Single Anchor, No Tags (Bench Test)
- [ ] Flash firmware with test WiFi credentials
- [ ] Verify WiFi connection via serial monitor
- [ ] Verify IP assignment
- [ ] Verify UDP socket opens (no errors)
- [ ] Check heartbeat messages include WiFi RSSI

### Phase 2: Single Anchor, One Tag (Ranging Test)
- [ ] Power on one Stella tag
- [ ] Wait for ranging to establish
- [ ] On ROS2 host: `nc -u -l 5000`
- [ ] Verify JSON packets arrive with correct `anchor_id` and `tag_id`
- [ ] Move tag to different distances, verify `distance_cm` changes
- [ ] Check timestamps increment correctly

### Phase 3: WiFi Resilience Test
- [ ] With anchor running and ranging active
- [ ] Disconnect WiFi access point
- [ ] Verify serial output: "WiFi connection lost"
- [ ] Verify red LED indicates failure
- [ ] Reconnect WiFi access point
- [ ] Verify reconnection within 5 seconds
- [ ] Verify UDP packets resume

### Phase 4: Multi-Anchor Test (Full Deployment)
- [ ] Flash 4 anchors with unique `ANCHOR_ID` (1-4)
- [ ] All configured with same `UDP_HOST` and `UDP_PORT`
- [ ] Power all anchors simultaneously
- [ ] On ROS2 host: `nc -u -l 5000`
- [ ] Verify packets from all 4 anchors (check `anchor_id` field)
- [ ] Power on multiple tags
- [ ] Verify interleaved packets (anchor_id varies, tag_id varies)

### Phase 5: Integration with ROS2 Node
- [ ] Launch ROS2 uwb_localization_node with `transport: "udp"`
- [ ] `ros2 topic echo /uwb/ranges`
- [ ] Verify UWBRange messages published with correct fields
- [ ] Check position calculation and publishing to `/uwb/tag_X/pose2d`
- [ ] Verify Foxglove visualization shows tag positions

### Phase 6: Performance Test
- [ ] 4 anchors + 10 tags running for 1 hour
- [ ] Monitor packet loss (ROS2 diagnostics)
- [ ] Monitor WiFi stability (check heartbeat RSSI logs)
- [ ] Check for memory leaks (monitor free heap in heartbeat if added)
- [ ] Verify no firmware crashes or hangs

### Phase 7: Edge Cases
- [ ] Power cycle anchor during ranging (recovery test)
- [ ] Incorrect WiFi credentials (verify red LED, error message)
- [ ] ROS2 host unreachable (UDP sends fail silently, no crash)
- [ ] Network congestion (saturate WiFi, check packet loss)
- [ ] Out-of-range tags (verify no crashes on 0xFFFF distance)

---

## Debugging Strategy

### Serial Debug Output
- **Baud rate:** 115200
- **Format:** Lines prefixed with `#` are diagnostics, JSON lines are data
- **Key messages:**
  - `# WiFi connected! IP: X.X.X.X` → WiFi OK
  - `# WiFi connection failed!` → Check credentials or AP availability
  - `# WiFi disconnected, attempting reconnect...` → Network interruption
  - `# heartbeat ...` → Periodic status

### LED Indicators
- **Red:** Initialization or WiFi failure
- **Green (solid):** Running, WiFi connected
- **Green (blinking):** Heartbeat (toggles every 5s)
- **Blue:** WiFi connection in progress

### Network Debugging
**On ROS2 host:**
```bash
# Listen for UDP packets
nc -u -l 5000

# Capture and display packets
sudo tcpdump -i any -n udp port 5000 -A

# Check if port is already in use
sudo lsof -i :5000
```

**On anchor (via Serial Monitor):**
- Check `WiFi.localIP()` - should be on same subnet as ROS2 host
- Check `WiFi.RSSI()` - should be > -70 dBm for reliable operation
- If `RSSI < -80 dBm`, move anchor closer to AP or use external antenna

### Logic Analyzer (Advanced)
If timing issues arise:
- **Pin D0 (TX):** Monitor serial output timing
- **GPIO (if available):** Toggle a GPIO at start/end of `rangingHandler()` to measure callback duration

### Checkpoints
1. **Firmware compiles:** No syntax errors
2. **WiFi connects:** IP assigned within 10 seconds
3. **UDP socket opens:** No error messages
4. **Packets visible on network:** `tcpdump` shows traffic
5. **JSON parsing works:** ROS2 node receives valid messages
6. **Multi-anchor works:** All 4 anchors sending to same port
7. **Reconnection works:** Anchor recovers from WiFi drop

---

## Approval Status

- [ ] **Waiting for user approval**
- [ ] Approved
- [ ] Executed

---

## Notes for Implementation

1. **File Location:** The source file `/tmp/uwb_extracted/anchor_firmware.ino` is in `/tmp`, which is temporary. After modification, it should be saved to a permanent location (e.g., `/home/svaghela/ros2_ws_2/arduino/anchor_firmware/` or similar).

2. **Library Dependencies:** WiFi.h and WiFiUdp.h are built into the Arduino Portenta C33 board support package. No additional library installation required. PortentaUWBShield library must already be installed (as it's used in current firmware).

3. **Configuration Per Anchor:** Each anchor must be flashed with:
   - Unique `ANCHOR_ID` (1, 2, 3, 4)
   - Same `WIFI_SSID` and `WIFI_PASS`
   - Same `UDP_HOST` (ROS2 host IP)
   - Same `UDP_PORT` (5000)
   - Same `NUM_TAGS` (10 or actual count)

4. **ROS2 Host IP:** The user must determine the ROS2 host's IP address before flashing. Use `hostname -I` or `ip addr show` on the ROS2 host.

5. **Future Enhancements:**
   - Add packet sequence numbers for loss detection
   - Implement circular buffer for WiFi reconnection resilience
   - Add OTA (Over-The-Air) firmware update capability
   - Support static IP configuration (currently DHCP only)
   - Add WiFi signal strength threshold warnings

6. **Compatibility:** This plan preserves all existing UWB functionality. The only changes are to the transport layer. Users can switch back to serial by commenting out WiFi code and reverting `rangingHandler()`.

---

## References

- **WiFi.h API:** https://www.arduino.cc/reference/en/libraries/wifi/
- **WiFiUDP API:** https://www.arduino.cc/reference/en/libraries/wifi/wifiudp/
- **Portenta C33 Documentation:** https://docs.arduino.cc/hardware/portenta-c33
- **PortentaUWBShield Library:** https://github.com/TrueSense/PortentaUWBShield
- **UWB WiFi Plan:** `/home/svaghela/ros2_ws_2/uwb_wifi_foxglove_plan.md` (Task 1)
- **Original Firmware:** `/tmp/uwb_extracted/anchor_firmware.ino`
