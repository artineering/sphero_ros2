# Arduino Stella Tag Firmware for UWB Positioning System

**Created:** 2026-03-07T17:03:19Z
**Status:** Pending Approval
**Hardware:** Arduino Stella (STM32H747 + built-in DW3000 UWB)
**Complexity:** Medium
**Target:** 10-12 battery-powered tags for Sphero robot tracking

## Task Description

Create optimized firmware for Arduino Stella boards that will serve as UWB tags tracked by 4 Portenta C33 anchors. The tags are battery-powered, attached to Sphero robots, and must operate reliably with minimal power consumption while performing Two-Way Ranging (TWR) with multiple anchors.

## Hardware Requirements

### Board Specifications
- **Board:** Arduino Stella (DCU040)
  - **MCU:** STM32H747 dual-core (Cortex-M7 @ 480MHz + Cortex-M4 @ 240MHz)
  - **UWB:** Built-in DW3000 transceiver (integrated, not a shield)
  - **RAM:** 1 MB SRAM (512KB M7 + 288KB M4 + shared RAM)
  - **Flash:** 2 MB
  - **NO WiFi module** (critical difference from Portenta C33)

### Power Requirements
- **Source:** Battery-powered (likely LiPo 3.7V)
- **Operational current:** ~150-200 mA during active UWB ranging
- **Sleep current:** Target < 5 mA between ranging cycles
- **Battery life goal:** 4-6 hours continuous operation

### Additional Components
- **LED:** Built-in RGB LED for status indication
- **Power monitoring:** Battery voltage ADC if available
- **Antenna:** Built-in or external UWB antenna

### Pin Assignments
| Pin/Peripheral | Function | Notes |
|----------------|----------|-------|
| Built-in DW3000 | UWB transceiver | Internal SPI connection |
| RGB LED | Status indication | Red/Green/Yellow patterns |
| ADC (if available) | Battery monitoring | Optional voltage sensing |
| USB Serial | Debug output | 115200 baud |

## Analysis

### Current Reference Firmware Assessment

**Strengths of `/tmp/uwb_extracted/tag_firmware.ino`:**
- Clean MAC addressing scheme (Tag M → {M, 0x01})
- Proper session ID matching with anchors (0x<AnchorHex><TagHex>)
- Multi-session architecture for 4 anchors
- Uses StellaUWB library (correct for Stella hardware)
- Preamble code coordination with anchors (PREAMBLE_BASE + offset)

**Issues to Address:**
1. **No power management** - Runs continuously at full power
2. **No battery monitoring** - Cannot detect low battery
3. **Minimal status feedback** - Only serial debug (no LEDs)
4. **Tag acts as responder** - Anchors initiate ranging (correct architecture)
5. **No robust error handling** - Doesn't handle anchor dropout gracefully
6. **No interference mitigation** - 10-12 tags may collide
7. **Fixed 10ms delay in loop** - Not optimized for power

### Power Analysis

**Active Ranging Mode:**
- M7 core @ 480 MHz: ~100 mA
- DW3000 UWB TX/RX: ~50-100 mA
- LED indicators: ~5-10 mA
- **Total active:** ~150-200 mA

**Sleep Mode (between ranging cycles):**
- M7 in sleep mode: ~5-10 mA
- DW3000 in idle/sleep: ~1-2 mA
- **Total sleep:** ~5-15 mA

**Battery Life Calculation (1000 mAh LiPo):**
- Continuous active (200 mA): ~5 hours
- With 50% duty cycle (100ms ranging, 100ms sleep): ~7-8 hours
- With 25% duty cycle (100ms ranging, 300ms sleep): ~10-12 hours

**Recommendation:** Implement sleep between ranging cycles, target 100ms ranging interval (25% duty cycle).

### Timing Considerations

**TWR Timing:**
- Each anchor-tag TWR exchange: ~10-20ms
- 4 anchors × 20ms = 80ms minimum per ranging cycle
- Stagger delay between sessions: 50ms (current firmware)
- **Total cycle time:** ~250ms (allows margin)

**With 10-12 Tags Operating:**
- Time-division on anchors (anchors initiate)
- Each tag needs unique preamble or time slot
- Current preamble strategy: anchors use different preambles (9-12)
- Tags differentiated by MAC address
- **Collision risk:** Medium (anchors manage scheduling)

**Recommended Ranging Interval:**
- 100ms cycle: Too fast, high collision risk with 10+ tags
- 250ms cycle: Good balance (4 Hz position update)
- 500ms cycle: Conservative, low power, lower update rate
- **Selection:** 250ms base + TAG_ID stagger (10ms × TAG_ID)

### Library Compatibility

**StellaUWB Library:**
- Designed for Arduino Stella with built-in DW3000
- `UWBMultiSessionTag` class for tag role
- Different from `PortentaUWBShield` used by anchors
- Same MAC/session architecture
- **Assumption:** Library supports sleep modes (needs verification)

## Detailed Plan

### Step 1: Enhanced Configuration Section

**Action:** Expand configuration defines for power management and deployment

**Files Modified:**
- `/tmp/uwb_extracted/tag_firmware.ino` (or create new file)

**Configuration Parameters:**
```cpp
// ============================================================
// USER CONFIGURATION — MUST SET TAG_ID BEFORE FLASHING
// ============================================================
#define TAG_ID              1       // Unique tag identifier (1–12)
#define NUM_ANCHORS         4       // Total number of anchors
#define SERIAL_BAUD         115200  // Debug serial baud rate
#define PREAMBLE_BASE       9       // Must match anchor firmware

// Power Management
#define RANGING_INTERVAL_MS 250     // Base ranging cycle interval
#define TAG_STAGGER_MS      10      // Stagger per tag (TAG_ID × 10ms)
#define ENABLE_SLEEP        true    // Enable sleep between ranging cycles
#define SLEEP_MARGIN_MS     50      // Wake before next cycle

// Battery Monitoring
#define ENABLE_BATTERY_MON  true    // Enable battery voltage monitoring
#define BATTERY_LOW_MV      3300    // Low battery threshold (mV)
#define BATTERY_CRIT_MV     3100    // Critical battery threshold (mV)

// LED Status Indication
#define ENABLE_LED_STATUS   true    // Enable LED feedback
#define LED_BRIGHTNESS      32      // LED PWM value (0-255, low for power)

// Debug Options
#define ENABLE_SERIAL_DEBUG true    // Print ranging data to serial
#define HEARTBEAT_INTERVAL  10000   // Heartbeat message interval (ms)
```

**Expected Outcome:**
- Clear, documented configuration section
- All key parameters user-configurable
- Power-optimized defaults

### Step 2: Add LED Status Manager

**Action:** Implement RGB LED status indication system

**New Code Section:**
```cpp
// LED status states
enum LEDStatus {
  LED_INIT,           // Initializing (blue pulse)
  LED_RANGING_GOOD,   // All 4 anchors visible (green solid)
  LED_RANGING_PARTIAL,// 2-3 anchors visible (yellow slow blink)
  LED_RANGING_POOR,   // 0-1 anchors visible (red fast blink)
  LED_BATTERY_LOW,    // Battery low (orange blink)
  LED_BATTERY_CRIT,   // Battery critical (red fast blink)
  LED_ERROR           // System error (red solid)
};

class LEDManager {
private:
  LEDStatus currentStatus;
  unsigned long lastUpdate;
  bool blinkState;

public:
  void begin() {
    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    turnOff();
  }

  void setStatus(LEDStatus status) {
    currentStatus = status;
    lastUpdate = millis();
    blinkState = false;
  }

  void update() {
    // Update LED based on status and blink pattern
    // Called from main loop
  }

  void turnOff() {
    digitalWrite(LEDR, HIGH);  // Active LOW on Stella
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, HIGH);
  }

  void setColor(bool r, bool g, bool b) {
    digitalWrite(LEDR, !r);  // Invert for active LOW
    digitalWrite(LEDG, !g);
    digitalWrite(LEDB, !b);
  }
};
```

**Expected Outcome:**
- Visual status feedback without serial connection
- Low power LED operation (PWM at reduced brightness)
- Non-blocking blink patterns

### Step 3: Implement Battery Monitor

**Action:** Add battery voltage monitoring and status tracking

**New Code Section:**
```cpp
class BatteryMonitor {
private:
  int adcPin;
  float voltageDividerRatio;  // Hardware-specific
  unsigned long lastCheck;
  float currentVoltage;
  bool lowBattery;
  bool criticalBattery;

public:
  BatteryMonitor(int pin, float dividerRatio)
    : adcPin(pin), voltageDividerRatio(dividerRatio),
      currentVoltage(0), lowBattery(false), criticalBattery(false) {}

  void begin() {
    pinMode(adcPin, INPUT);
    lastCheck = 0;
  }

  void update() {
    // Read ADC every 5 seconds
    if (millis() - lastCheck > 5000) {
      int rawADC = analogRead(adcPin);
      currentVoltage = (rawADC / 4095.0) * 3.3 * voltageDividerRatio;

      criticalBattery = (currentVoltage * 1000 < BATTERY_CRIT_MV);
      lowBattery = (currentVoltage * 1000 < BATTERY_LOW_MV);

      lastCheck = millis();
    }
  }

  float getVoltage() { return currentVoltage; }
  bool isLow() { return lowBattery; }
  bool isCritical() { return criticalBattery; }
};
```

**Hardware Assumption:**
- Stella may have built-in battery voltage sensing (need to verify)
- If not, external voltage divider required (e.g., 2:1 ratio for 3.7V LiPo)

**Expected Outcome:**
- Real-time battery status monitoring
- Warning when battery is low
- Possible safe shutdown on critical battery

### Step 4: Add Anchor Health Tracking

**Action:** Track last-seen time for each anchor to detect dropouts

**New Code Section:**
```cpp
struct AnchorHealth {
  unsigned long lastSeen;     // millis() of last successful range
  uint32_t rangeCount;        // Total successful ranges
  uint32_t errorCount;        // Failed ranges
  bool isHealthy;             // Seen in last 1 second
};

AnchorHealth anchorHealth[NUM_ANCHORS];

void updateAnchorHealth(int anchorIndex, bool success) {
  if (success) {
    anchorHealth[anchorIndex].lastSeen = millis();
    anchorHealth[anchorIndex].rangeCount++;
  } else {
    anchorHealth[anchorIndex].errorCount++;
  }

  // Mark healthy if seen in last 1 second
  anchorHealth[anchorIndex].isHealthy =
    (millis() - anchorHealth[anchorIndex].lastSeen < 1000);
}

int countHealthyAnchors() {
  int count = 0;
  for (int i = 0; i < NUM_ANCHORS; i++) {
    if (anchorHealth[i].isHealthy) count++;
  }
  return count;
}
```

**Expected Outcome:**
- Real-time tracking of anchor visibility
- LED status reflects anchor health (4=green, 2-3=yellow, 0-1=red)
- Diagnostic data for debugging

### Step 5: Enhance Ranging Callback with Tracking

**Action:** Modify `rangingHandler()` to update anchor health and provide rich debug info

**Modified Code:**
```cpp
void rangingHandler(UWBRangingData &rangingData) {
  if (rangingData.measureType() != (uint8_t)uwb::MeasurementType::TWO_WAY) {
    return;
  }

  RangingMeasures twr = rangingData.twoWayRangingMeasure();
  uint32_t sessionId = rangingData.sessionHandle();

  // Extract anchor ID from session handle (upper 16 bits)
  int anchorId = (sessionId >> 16) & 0xFF;
  int anchorIndex = anchorId - 1;  // Zero-indexed

  for (int j = 0; j < rangingData.available(); j++) {
    bool success = (twr[j].status == 0 && twr[j].distance != 0xFFFF);

    // Update health tracking
    if (anchorIndex >= 0 && anchorIndex < NUM_ANCHORS) {
      updateAnchorHealth(anchorIndex, success);
    }

    if (success && ENABLE_SERIAL_DEBUG) {
      Serial.print("Tag ");
      Serial.print(TAG_ID);
      Serial.print(" | A");
      Serial.print(anchorId);
      Serial.print(" | ");
      Serial.print(twr[j].distance);
      Serial.print(" cm | session 0x");
      Serial.println(sessionId, HEX);
    }
  }
}
```

**Expected Outcome:**
- Anchor health updates on every ranging attempt
- Rich debug output with anchor identification
- Foundation for LED status updates

### Step 6: Implement Power Management

**Action:** Add sleep mode between ranging cycles

**New Code Section:**
```cpp
#include "mbed.h"  // For STM32 sleep functions

class PowerManager {
private:
  unsigned long lastRangingCycle;
  unsigned long nextWakeTime;

public:
  void begin() {
    lastRangingCycle = millis();
    calculateNextWake();
  }

  void calculateNextWake() {
    // Stagger based on TAG_ID to reduce collisions
    unsigned long baseInterval = RANGING_INTERVAL_MS;
    unsigned long stagger = TAG_ID * TAG_STAGGER_MS;
    nextWakeTime = lastRangingCycle + baseInterval + stagger;
  }

  void sleepUntilNextCycle() {
    if (!ENABLE_SLEEP) return;

    unsigned long now = millis();
    if (now < nextWakeTime - SLEEP_MARGIN_MS) {
      unsigned long sleepDuration = nextWakeTime - SLEEP_MARGIN_MS - now;

      // Enter STM32 sleep mode (not deep sleep to keep UWB sessions alive)
      // Note: This is a simplified approach; real implementation needs
      // to verify UWB stack behavior during sleep
      delay(sleepDuration);  // Replace with actual sleep API

      lastRangingCycle = millis();
      calculateNextWake();
    }
  }

  unsigned long getNextWakeTime() { return nextWakeTime; }
};
```

**Important Note:**
The StellaUWB library may not support sleep modes while sessions are active. This needs verification. Possible approaches:
1. **Soft sleep:** Use `delay()` - saves minimal power
2. **CPU sleep:** STM32 sleep mode - moderate savings (~30-40%)
3. **Deep sleep:** Stop UWB sessions, reinitialize - maximum savings but complex

**Recommendation:** Start with soft sleep, test power consumption, upgrade if needed.

**Expected Outcome:**
- Reduced power consumption during idle periods
- Staggered ranging cycles to reduce tag-tag interference
- Configurable sleep enable/disable

### Step 7: Update setup() Function

**Action:** Initialize all new systems in proper sequence

**Modified setup():**
```cpp
LEDManager ledManager;
BatteryMonitor battery(A0, 2.0);  // Adjust pin and ratio per hardware
PowerManager powerMgr;

void setup() {
  // Initialize LED first for visual feedback
  if (ENABLE_LED_STATUS) {
    ledManager.begin();
    ledManager.setStatus(LED_INIT);
  }

  // Initialize battery monitor
  if (ENABLE_BATTERY_MON) {
    battery.begin();
  }

  // Serial for debugging
  Serial.begin(SERIAL_BAUD);
  delay(1000);  // Allow serial to connect

  Serial.print("# Stella Tag ");
  Serial.print(TAG_ID);
  Serial.println(" — Initializing UWB...");

  // Initialize anchor health tracking
  for (int i = 0; i < NUM_ANCHORS; i++) {
    anchorHealth[i].lastSeen = 0;
    anchorHealth[i].rangeCount = 0;
    anchorHealth[i].errorCount = 0;
    anchorHealth[i].isHealthy = false;
  }

  // Build this tag's MAC address: {TAG_ID, 0x01}
  uint8_t tagAddr[] = {(uint8_t)TAG_ID, 0x01};
  UWBMacAddress tagMac(UWBMacAddress::Size::SHORT, tagAddr);

  UWB.registerRangingCallback(rangingHandler);
  UWB.begin();

  while (UWB.state() != 0) {
    delay(10);
  }
  Serial.println("# UWB ready.");

  // Create sessions for each anchor (same as original)
  for (int a = 1; a <= NUM_ANCHORS; a++) {
    uint8_t anchorAddr[] = {(uint8_t)(0xA0 + a), 0x01};
    UWBMacAddress anchorMac(UWBMacAddress::Size::SHORT, anchorAddr);

    uint32_t sessionId = ((uint32_t)a << 16) | (uint32_t)TAG_ID;
    uint8_t preamble = PREAMBLE_BASE + ((a - 1) % 4);

    sessions[a - 1] = new UWBMultiSessionTag(sessionId, tagMac, anchorMac, preamble);
    UWBSessionManager.addSession(*sessions[a - 1]);
    sessions[a - 1]->init();

    Serial.print("# Session: anchor=");
    Serial.print(a);
    Serial.print(" sessionId=0x");
    Serial.print(sessionId, HEX);
    Serial.print(" preamble=");
    Serial.println(preamble);
  }

  // Start all sessions with stagger
  for (int a = 0; a < NUM_ANCHORS; a++) {
    sessions[a]->start();
    delay(50);
  }

  Serial.println("# All sessions started. Tag is ranging.");

  // Initialize power manager
  powerMgr.begin();

  // LED to green once running
  if (ENABLE_LED_STATUS) {
    ledManager.setStatus(LED_RANGING_GOOD);
  }
}
```

**Expected Outcome:**
- Orderly initialization with visual feedback
- All systems online and tracking
- Ready for power-optimized operation

### Step 8: Update loop() Function

**Action:** Add battery monitoring, LED updates, health-based status, and sleep

**Modified loop():**
```cpp
void loop() {
  // Update battery status
  if (ENABLE_BATTERY_MON) {
    battery.update();

    // Override LED status for battery warnings
    if (battery.isCritical()) {
      ledManager.setStatus(LED_BATTERY_CRIT);
    } else if (battery.isLow()) {
      ledManager.setStatus(LED_BATTERY_LOW);
    }
  }

  // Update LED status based on anchor health
  if (ENABLE_LED_STATUS) {
    int healthyCount = countHealthyAnchors();

    if (!battery.isLow() && !battery.isCritical()) {
      if (healthyCount >= 4) {
        ledManager.setStatus(LED_RANGING_GOOD);
      } else if (healthyCount >= 2) {
        ledManager.setStatus(LED_RANGING_PARTIAL);
      } else {
        ledManager.setStatus(LED_RANGING_POOR);
      }
    }

    ledManager.update();  // Non-blocking LED blink handler
  }

  // Heartbeat with diagnostics
  static unsigned long lastHeartbeat = 0;
  if (ENABLE_SERIAL_DEBUG && millis() - lastHeartbeat > HEARTBEAT_INTERVAL) {
    Serial.print("# Tag ");
    Serial.print(TAG_ID);
    Serial.print(" | Uptime: ");
    Serial.print(millis() / 1000);
    Serial.print("s | Anchors: ");
    Serial.print(countHealthyAnchors());
    Serial.print("/");
    Serial.print(NUM_ANCHORS);

    if (ENABLE_BATTERY_MON) {
      Serial.print(" | Battery: ");
      Serial.print(battery.getVoltage(), 2);
      Serial.print("V");
    }

    Serial.println();

    // Detailed anchor stats
    for (int i = 0; i < NUM_ANCHORS; i++) {
      Serial.print("#   A");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(anchorHealth[i].isHealthy ? "OK" : "LOST");
      Serial.print(" | ranges=");
      Serial.print(anchorHealth[i].rangeCount);
      Serial.print(" errors=");
      Serial.println(anchorHealth[i].errorCount);
    }

    lastHeartbeat = millis();
  }

  // Sleep until next ranging cycle (if enabled)
  powerMgr.sleepUntilNextCycle();

  delay(10);  // Small delay to prevent tight loop
}
```

**Expected Outcome:**
- Comprehensive system status monitoring
- Automatic LED status based on conditions
- Power-optimized operation with sleep
- Rich diagnostic output for debugging

## Pin Configuration

**Arduino Stella Built-in Peripherals:**
| Peripheral | Function | Notes |
|------------|----------|-------|
| DW3000 UWB | UWB ranging | Internal SPI, pre-configured |
| RGB LED (R) | Status - Red | Active LOW, GPIO pin (verify in docs) |
| RGB LED (G) | Status - Green | Active LOW |
| RGB LED (B) | Status - Blue | Active LOW |
| USB Serial | Debug output | 115200 baud, /dev/ttyACMx |
| ADC A0 (?) | Battery voltage | If available, verify pin mapping |

**Note:** Arduino Stella pin mapping needs verification from official documentation. LED pins and battery ADC pin may differ.

## Timing Analysis

### Per-Tag Ranging Cycle (250ms base + stagger)

| Tag ID | Stagger | Total Interval | Update Rate |
|--------|---------|----------------|-------------|
| 1      | 10ms    | 260ms          | 3.85 Hz     |
| 2      | 20ms    | 270ms          | 3.70 Hz     |
| 3      | 30ms    | 280ms          | 3.57 Hz     |
| ...    | ...     | ...            | ...         |
| 10     | 100ms   | 350ms          | 2.86 Hz     |
| 12     | 120ms   | 370ms          | 2.70 Hz     |

**Collision Analysis:**
- With staggering, tags initiate at different times
- Anchors are responders, handle multiple tags via session IDs
- DW3000 channel access via ALOHA-like protocol
- **Expected success rate:** 85-95% with 10 tags, 80-90% with 12 tags

### Task Priorities (if using FreeRTOS)

Current firmware doesn't explicitly use FreeRTOS tasks, runs in `loop()`. If migrating to RTOS:

| Task | Priority | Stack | Period | Function |
|------|----------|-------|--------|----------|
| UWB ranging | 3 (high) | 2048 | Event-driven | Handle UWB callbacks |
| LED update | 1 (low) | 512 | 50ms | Update LED patterns |
| Battery monitor | 1 (low) | 512 | 5000ms | Read battery ADC |
| Serial debug | 1 (low) | 1024 | 10000ms | Print diagnostics |

**Recommendation:** Keep simple `loop()` architecture unless timing issues arise. RTOS adds complexity.

## Expected Outcomes

### Functional Requirements
- [x] Each tag performs TWR with 4 anchors
- [x] Configurable TAG_ID (1-12)
- [x] 250ms ranging cycle with stagger
- [x] LED status indication (green/yellow/red)
- [x] Battery monitoring with low/critical warnings
- [x] Robust to anchor dropouts
- [x] Power-optimized with sleep mode
- [x] Serial debug output
- [x] 30-50m ranging capability

### Performance Targets
- **Power consumption:** < 150 mA average (50% duty cycle)
- **Battery life:** 6-8 hours on 1000 mAh LiPo
- **Ranging success rate:** > 90% with all anchors visible
- **Position update rate:** 3-4 Hz per tag
- **Startup time:** < 5 seconds from power-on to ranging

### Deployment
- **Number of units:** 10-12 tags
- **Configuration:** Flash each tag with unique TAG_ID (1-12)
- **Installation:** Attach to Sphero robots, power on
- **Operation:** Autonomous ranging, no user intervention
- **Monitoring:** LED status, optional serial debug

## Potential Risks & Considerations

### Hardware Risks
1. **Battery voltage monitoring unavailable:**
   - Mitigation: Verify Stella schematic, add external voltage divider if needed
   - Fallback: Disable battery monitoring, use runtime-based warnings

2. **UWB antenna performance:**
   - Risk: Multipath, NLOS in indoor arena
   - Mitigation: Anchor placement optimization, NLOS detection in ROS2 node

3. **Heat generation:**
   - Risk: Continuous UWB operation generates heat
   - Mitigation: Monitor temperature, ensure adequate ventilation on Sphero

### Software Risks
1. **UWB library doesn't support sleep:**
   - Mitigation: Start with delay()-based "soft sleep", measure power
   - Escalation: Contact StellaUWB library maintainers for sleep API

2. **Session initialization failures:**
   - Risk: Tags fail to establish sessions with some anchors
   - Mitigation: Robust error handling, retry logic, LED error indication

3. **Tag-tag interference with 10-12 units:**
   - Risk: Packet collisions reduce ranging success rate
   - Mitigation: Stagger ranging cycles, monitor success rate, adjust intervals

4. **Memory exhaustion:**
   - Risk: 4 sessions + buffers may exceed available RAM
   - Mitigation: Monitor heap usage, optimize buffer sizes, remove debug strings in production

### Timing Risks
1. **Ranging cycle overrun:**
   - Risk: 4 anchors × 20ms = 80ms, exceeds 100ms sleep window
   - Mitigation: Measure actual TWR timing, adjust intervals dynamically

2. **Anchor scheduling conflicts:**
   - Risk: Anchors can't handle 10-12 tags simultaneously
   - Mitigation: Monitor anchor CPU usage, increase anchor ranging timeout

### Environmental Risks
1. **Arena reflections cause NLOS:**
   - Mitigation: Post-processing in ROS2 node (residual-based filtering)

2. **Sphero motor noise interferes with UWB:**
   - Mitigation: Shielding, ferrite beads, measure SNR during operation

## Testing Plan

### Phase 1: Single Tag Bench Test (1 hour)
**Setup:**
- 1 Stella tag flashed with TAG_ID=1
- 4 anchors powered and ranging
- Tag powered from USB or battery
- Serial monitor at 115200 baud

**Tests:**
1. **Initialization:**
   - [ ] LED turns blue during init
   - [ ] UWB stack initializes (state == 0)
   - [ ] 4 sessions created (A1-A4)
   - [ ] All sessions start successfully
   - [ ] LED turns green when ranging starts

2. **Ranging:**
   - [ ] Ranging callback fires for all 4 anchors
   - [ ] Distance measurements appear in serial output
   - [ ] Anchor health shows 4/4 healthy
   - [ ] LED remains green

3. **Anchor Dropout:**
   - [ ] Power off anchor 1
   - [ ] Wait 1 second
   - [ ] Anchor health shows 3/4 healthy
   - [ ] LED turns yellow (partial ranging)
   - [ ] Power on anchor 1
   - [ ] Health recovers to 4/4
   - [ ] LED turns green

4. **Battery Monitoring:**
   - [ ] Heartbeat shows battery voltage
   - [ ] Simulate low voltage (if possible)
   - [ ] LED turns orange on low battery
   - [ ] LED turns red on critical battery

5. **Power Consumption:**
   - [ ] Measure current with multimeter
   - [ ] Verify < 200 mA during active ranging
   - [ ] Verify reduced current during sleep (if implemented)

### Phase 2: Multi-Tag Test (2 hours)
**Setup:**
- Flash 3 tags with TAG_ID = 1, 2, 3
- 4 anchors running
- Position tags at different locations
- Monitor each tag via serial (USB hub or multiplexer)

**Tests:**
1. **Simultaneous Operation:**
   - [ ] All 3 tags initialize successfully
   - [ ] No session conflicts (unique session IDs)
   - [ ] Each tag ranges with all 4 anchors
   - [ ] Ranging success rate > 90% for each tag

2. **Stagger Verification:**
   - [ ] Tag 1 heartbeat at T+260ms intervals
   - [ ] Tag 2 heartbeat at T+270ms intervals
   - [ ] Tag 3 heartbeat at T+280ms intervals
   - [ ] Confirm stagger reduces collisions

3. **Interference Test:**
   - [ ] Add more tags (up to 10-12 if available)
   - [ ] Monitor ranging success rate
   - [ ] Verify > 85% success with all tags
   - [ ] Adjust stagger if needed

### Phase 3: Sphero Integration Test (1 hour)
**Setup:**
- Attach 1 tag to Sphero robot
- Ensure tag is powered (battery or Sphero power)
- Drive Sphero around arena

**Tests:**
1. **Mobility:**
   - [ ] Tag operates reliably while Sphero moves
   - [ ] No mechanical disconnections
   - [ ] UWB antenna maintains good radiation pattern

2. **Arena Coverage:**
   - [ ] Tag ranges successfully at all arena positions
   - [ ] No dead zones
   - [ ] LED status reflects anchor visibility

3. **ROS2 Integration:**
   - [ ] Anchors receive ranging data
   - [ ] ROS2 node computes tag position
   - [ ] Position tracks Sphero movement
   - [ ] Latency < 1 second

### Phase 4: Endurance Test (4-8 hours)
**Setup:**
- All tags powered on battery
- Ranging continuously
- Monitor battery status

**Tests:**
1. **Battery Life:**
   - [ ] Record start voltage and time
   - [ ] Monitor voltage over time
   - [ ] Record end voltage and time
   - [ ] Calculate actual battery life
   - [ ] Verify > 4 hours on 1000 mAh LiPo

2. **Reliability:**
   - [ ] Tags operate continuously without crashes
   - [ ] No memory leaks (heap size stable)
   - [ ] Ranging success rate remains > 90%

3. **Heat:**
   - [ ] Measure tag temperature after 1 hour
   - [ ] Verify < 50°C (safe for LiPo)

## Debugging Strategy

### Debug Output Levels

**Level 1: Startup (always enabled):**
```
# Stella Tag 1 — Initializing UWB...
# UWB ready.
# Session: anchor=1 sessionId=0x10001 preamble=9
# Session: anchor=2 sessionId=0x20001 preamble=10
# Session: anchor=3 sessionId=0x30001 preamble=11
# Session: anchor=4 sessionId=0x40001 preamble=12
# All sessions started. Tag is ranging.
```

**Level 2: Ranging (enable via ENABLE_SERIAL_DEBUG):**
```
Tag 1 | A1 | 142 cm | session 0x10001
Tag 1 | A2 | 238 cm | session 0x20001
Tag 1 | A3 | 189 cm | session 0x30001
Tag 1 | A4 | 315 cm | session 0x40001
```

**Level 3: Heartbeat (every 10 seconds):**
```
# Tag 1 | Uptime: 125s | Anchors: 4/4 | Battery: 3.82V
#   A1: OK | ranges=498 errors=2
#   A2: OK | ranges=501 errors=0
#   A3: OK | ranges=495 errors=5
#   A4: OK | ranges=499 errors=1
```

### Serial Debug Configuration

**Baud rate:** 115200 (standard)
**Format:** Lines prefixed with `#` are metadata, data lines are measurements
**Log rotation:** Not implemented (continuous stream)
**Remote logging:** Not supported (tags have no WiFi)

### LED Debug Patterns

| Pattern | Color | Meaning |
|---------|-------|---------|
| Solid blue | Blue | Initializing |
| Solid green | Green | Ranging good (4/4 anchors) |
| Slow blink yellow | Yellow | Partial ranging (2-3 anchors) |
| Fast blink red | Red | Poor ranging (0-1 anchors) |
| Blink orange | Orange | Battery low |
| Fast blink red | Red | Battery critical |
| Solid red | Red | System error |

### Hardware Debugging Tools

1. **Multimeter:**
   - Measure battery voltage at connector
   - Measure current consumption (in series with battery)
   - Verify 3.3V rail on Stella

2. **Logic Analyzer:**
   - Not typically needed (UWB is internal SPI)
   - Use if debugging UWB initialization failures

3. **Oscilloscope:**
   - Verify power rail stability
   - Check for noise on battery line

4. **UWB Sniffer (if available):**
   - Capture UWB packets
   - Verify TWR exchange timing
   - Debug collision issues

### Common Issues & Solutions

| Issue | Symptom | Solution |
|-------|---------|----------|
| Tag doesn't initialize | LED stuck on blue | Check USB power, re-flash firmware |
| No ranging data | LED red, no serial output | Verify anchors are running, check MAC addresses |
| Partial ranging | LED yellow | Check anchor placement, NLOS paths |
| Battery drains fast | < 2 hours runtime | Enable sleep, reduce LED brightness |
| Collisions with other tags | Low success rate | Increase stagger interval |
| Tag crashes | No LED, no serial | Check heap usage, reduce session count |

## Approval Checklist

Before approving this plan, verify:
- [ ] Pin assignments compatible with Stella hardware
- [ ] Power consumption estimates are reasonable
- [ ] Timing analysis accounts for all overhead
- [ ] LED status patterns are clear and useful
- [ ] Battery monitoring approach is viable (or marked optional)
- [ ] Sleep mode implementation is compatible with StellaUWB library
- [ ] MAC/session scheme matches anchor firmware
- [ ] Error handling is comprehensive
- [ ] Debug outputs are sufficient for troubleshooting
- [ ] Testing plan covers all key scenarios
- [ ] Documentation is complete for deployment

## Documentation Deliverables

### 1. Firmware File
**File:** `tag_firmware_optimized.ino`
- Complete Arduino sketch with all features
- Heavily commented configuration section
- Inline documentation for all classes and functions

### 2. Flashing Guide
**File:** `TAG_FLASHING_GUIDE.md`
- Prerequisites (Arduino IDE, Stella board support)
- Step-by-step flashing instructions
- How to set TAG_ID for each unit (edit sketch before upload)
- Verification procedure (serial output check)
- Troubleshooting common issues

### 3. Deployment Guide
**File:** `TAG_DEPLOYMENT_GUIDE.md`
- Battery selection and installation
- Mechanical mounting to Sphero
- Power-on procedure
- LED status interpretation
- Expected battery life
- Charging procedure

### 4. Performance Report
**File:** `TAG_PERFORMANCE_REPORT.md`
- Measured power consumption (active, sleep, average)
- Actual battery life (various LiPo capacities)
- Ranging success rates (1 tag, 10 tags, 12 tags)
- Latency measurements
- Known limitations

## Approval Status

- [ ] **Waiting for user approval**
- [ ] **Approved** - Proceed with implementation
- [ ] **Executed** - Firmware complete and tested

---

## Notes

**Key Decisions Made:**
1. **Power management:** Delay-based soft sleep initially, verify before RTOS migration
2. **Ranging interval:** 250ms base + 10ms stagger per TAG_ID
3. **LED status:** Priority to battery > anchor health
4. **Architecture:** Stay with simple loop(), avoid FreeRTOS complexity unless needed
5. **Battery monitoring:** Optional feature (hardware-dependent)

**Questions for User:**
1. Do you have Stella hardware datasheets/pinout for battery ADC?
2. What LiPo capacity are you planning to use?
3. Do you need to flash all 10-12 tags, or start with a subset?
4. Any specific mechanical mounting constraints for the Spheros?

**Dependencies:**
- StellaUWB library (must be installed in Arduino IDE)
- Arduino Stella board support package
- Anchor firmware already flashed and operational
- ROS2 localization node running on host

**Timeline Estimate:**
- Firmware development: 2-3 hours
- Single tag testing: 1 hour
- Multi-tag testing: 2 hours
- Documentation: 1-2 hours
- **Total:** 6-8 hours

**Risk Level:** Medium
- Hardware unknowns (battery ADC, sleep API)
- Multi-tag interference (mitigated by stagger)
- Power optimization (may require iteration)
