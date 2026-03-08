/**
 * UWB RTLS Tag Firmware - OPTIMIZED VERSION
 * ==========================================
 * Platform: Arduino Stella (STM32H747 + built-in DW3000 UWB)
 * Library:  StellaUWB (Truesense)
 *
 * FEATURES:
 *   - Power management with sleep modes
 *   - Battery voltage monitoring with warnings
 *   - RGB LED status indication
 *   - Anchor health tracking
 *   - Collision mitigation via staggered ranging
 *   - Comprehensive diagnostics
 *
 * Each Stella tag creates TWR sessions to all configured anchors.
 * The tag acts as responder/controlee — anchors initiate ranging.
 *
 * BEFORE FLASHING:
 *   1. Set TAG_ID (1-12) uniquely for each Stella board
 *   2. Verify NUM_ANCHORS matches your deployment
 *   3. Configure battery monitoring pin (A0) if available
 *   4. Adjust power settings as needed
 */

#include "StellaUWB.h"

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
#define BATTERY_ADC_PIN     A0      // ADC pin for battery voltage
#define VOLTAGE_DIVIDER     2.0     // Hardware voltage divider ratio
#define BATTERY_LOW_MV      3300    // Low battery threshold (mV)
#define BATTERY_CRIT_MV     3100    // Critical battery threshold (mV)

// LED Status Indication
#define ENABLE_LED_STATUS   true    // Enable LED feedback
#define LED_BRIGHTNESS      32      // LED PWM value (0-255, low for power)

// Debug Options
#define ENABLE_SERIAL_DEBUG true    // Print ranging data to serial
#define HEARTBEAT_INTERVAL  10000   // Heartbeat message interval (ms)

// ============================================================
// MAC address scheme (must match anchor firmware):
//   Anchor N  → {0xA0 + N, 0x01}
//   Tag    M  → {M, 0x01}
//
// Session ID scheme (must match anchor firmware):
//   0x<AnchorHex><TagHex>  e.g. Anchor 2 + Tag 5 = 0x020005
// ============================================================

// ============================================================
// LED STATUS MANAGER
// ============================================================
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
  int blinkInterval;

public:
  void begin() {
    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    turnOff();
    currentStatus = LED_INIT;
    lastUpdate = 0;
    blinkState = false;
    blinkInterval = 500;
  }

  void setStatus(LEDStatus status) {
    if (currentStatus != status) {
      currentStatus = status;
      lastUpdate = millis();
      blinkState = false;

      // Set blink interval based on status
      switch (status) {
        case LED_RANGING_PARTIAL:
          blinkInterval = 1000; // Slow blink
          break;
        case LED_RANGING_POOR:
        case LED_BATTERY_CRIT:
          blinkInterval = 250;  // Fast blink
          break;
        case LED_BATTERY_LOW:
          blinkInterval = 500;  // Medium blink
          break;
        default:
          blinkInterval = 500;
          break;
      }
    }
  }

  void update() {
    unsigned long now = millis();

    switch (currentStatus) {
      case LED_INIT:
        // Blue pulse during initialization
        if (now - lastUpdate > 500) {
          blinkState = !blinkState;
          setColor(false, false, blinkState);
          lastUpdate = now;
        }
        break;

      case LED_RANGING_GOOD:
        // Solid green
        setColor(false, true, false);
        break;

      case LED_RANGING_PARTIAL:
        // Slow blink yellow (red + green)
        if (now - lastUpdate > blinkInterval) {
          blinkState = !blinkState;
          setColor(blinkState, blinkState, false);
          lastUpdate = now;
        }
        break;

      case LED_RANGING_POOR:
        // Fast blink red
        if (now - lastUpdate > blinkInterval) {
          blinkState = !blinkState;
          setColor(blinkState, false, false);
          lastUpdate = now;
        }
        break;

      case LED_BATTERY_LOW:
        // Blink orange (red + green dim)
        if (now - lastUpdate > blinkInterval) {
          blinkState = !blinkState;
          setColor(blinkState, blinkState/2, false);
          lastUpdate = now;
        }
        break;

      case LED_BATTERY_CRIT:
        // Fast blink red
        if (now - lastUpdate > blinkInterval) {
          blinkState = !blinkState;
          setColor(blinkState, false, false);
          lastUpdate = now;
        }
        break;

      case LED_ERROR:
        // Solid red
        setColor(true, false, false);
        break;
    }
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

// ============================================================
// BATTERY MONITOR
// ============================================================
class BatteryMonitor {
private:
  int adcPin;
  float voltageDividerRatio;
  unsigned long lastCheck;
  float currentVoltage;
  bool lowBattery;
  bool criticalBattery;

public:
  BatteryMonitor(int pin, float dividerRatio)
    : adcPin(pin), voltageDividerRatio(dividerRatio),
      currentVoltage(0), lowBattery(false), criticalBattery(false) {
    lastCheck = 0;
  }

  void begin() {
    pinMode(adcPin, INPUT);
    lastCheck = 0;
    // Initial read
    update();
  }

  void update() {
    // Read ADC every 5 seconds to save power
    if (millis() - lastCheck > 5000) {
      int rawADC = analogRead(adcPin);

      // Convert ADC reading to voltage
      // STM32 ADC is 12-bit (0-4095) with 3.3V reference
      currentVoltage = (rawADC / 4095.0) * 3.3 * voltageDividerRatio;

      // Update battery status flags
      int voltageMillivolts = (int)(currentVoltage * 1000);
      criticalBattery = (voltageMillivolts < BATTERY_CRIT_MV);
      lowBattery = (voltageMillivolts < BATTERY_LOW_MV);

      lastCheck = millis();
    }
  }

  float getVoltage() { return currentVoltage; }
  bool isLow() { return lowBattery; }
  bool isCritical() { return criticalBattery; }
  int getPercentage() {
    // Rough LiPo percentage estimate (3.0V to 4.2V range)
    float voltage = currentVoltage;
    if (voltage >= 4.2) return 100;
    if (voltage <= 3.0) return 0;
    return (int)((voltage - 3.0) / 1.2 * 100);
  }
};

// ============================================================
// ANCHOR HEALTH TRACKING
// ============================================================
struct AnchorHealth {
  unsigned long lastSeen;     // millis() of last successful range
  uint32_t rangeCount;        // Total successful ranges
  uint32_t errorCount;        // Failed ranges
  bool isHealthy;             // Seen in last 1 second
};

AnchorHealth anchorHealth[NUM_ANCHORS];

void updateAnchorHealth(int anchorIndex, bool success) {
  if (anchorIndex < 0 || anchorIndex >= NUM_ANCHORS) return;

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

// ============================================================
// POWER MANAGER
// ============================================================
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

      // Soft sleep using delay() - saves minimal power but keeps UWB sessions alive
      // For deeper sleep, would need to verify UWB stack behavior
      if (sleepDuration > 10) {
        delay(sleepDuration);
      }

      lastRangingCycle = millis();
      calculateNextWake();
    }
  }

  unsigned long getNextWakeTime() { return nextWakeTime; }
  unsigned long getCurrentInterval() {
    return RANGING_INTERVAL_MS + (TAG_ID * TAG_STAGGER_MS);
  }
};

// ============================================================
// GLOBAL OBJECTS
// ============================================================
UWBMultiSessionTag* sessions[NUM_ANCHORS];
LEDManager ledManager;
BatteryMonitor battery(BATTERY_ADC_PIN, VOLTAGE_DIVIDER);
PowerManager powerMgr;

// ============================================================
// RANGING CALLBACK
// ============================================================
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
      Serial.print(sessionId, HEX);
      Serial.println();
    }
  }
}

// ============================================================
// SETUP
// ============================================================
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

  Serial.println("========================================");
  Serial.print("# Stella Tag ");
  Serial.print(TAG_ID);
  Serial.println(" — OPTIMIZED FIRMWARE v1.0");
  Serial.println("========================================");

  if (ENABLE_BATTERY_MON) {
    Serial.print("# Battery voltage: ");
    Serial.print(battery.getVoltage(), 2);
    Serial.print("V (");
    Serial.print(battery.getPercentage());
    Serial.println("%)");
  }

  Serial.print("# Ranging interval: ");
  Serial.print(RANGING_INTERVAL_MS + (TAG_ID * TAG_STAGGER_MS));
  Serial.println(" ms");

  Serial.println("# Initializing UWB...");

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

  // Create sessions for each anchor
  for (int a = 1; a <= NUM_ANCHORS; a++) {
    uint8_t anchorAddr[] = {(uint8_t)(0xA0 + a), 0x01};
    UWBMacAddress anchorMac(UWBMacAddress::Size::SHORT, anchorAddr);

    // Session ID must match what the anchor uses
    uint32_t sessionId = ((uint32_t)a << 16) | (uint32_t)TAG_ID;

    // Preamble must match the corresponding anchor
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
  Serial.println("========================================");

  // Initialize power manager
  powerMgr.begin();

  // LED to green once running
  if (ENABLE_LED_STATUS) {
    ledManager.setStatus(LED_RANGING_GOOD);
  }
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  // Update battery status
  if (ENABLE_BATTERY_MON) {
    battery.update();

    // Override LED status for battery warnings (highest priority)
    if (battery.isCritical()) {
      ledManager.setStatus(LED_BATTERY_CRIT);
    } else if (battery.isLow()) {
      ledManager.setStatus(LED_BATTERY_LOW);
    }
  }

  // Update LED status based on anchor health (if not battery warning)
  if (ENABLE_LED_STATUS) {
    if (!battery.isLow() && !battery.isCritical()) {
      int healthyCount = countHealthyAnchors();

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
    Serial.println("----------------------------------------");
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
      Serial.print("V (");
      Serial.print(battery.getPercentage());
      Serial.print("%)");
    }

    Serial.println();

    // Detailed anchor stats
    for (int i = 0; i < NUM_ANCHORS; i++) {
      Serial.print("#   A");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(anchorHealth[i].isHealthy ? "OK  " : "LOST");
      Serial.print(" | ranges=");
      Serial.print(anchorHealth[i].rangeCount);
      Serial.print(" errors=");
      Serial.print(anchorHealth[i].errorCount);

      if (anchorHealth[i].rangeCount > 0) {
        float successRate = 100.0 * anchorHealth[i].rangeCount /
                           (anchorHealth[i].rangeCount + anchorHealth[i].errorCount);
        Serial.print(" (");
        Serial.print(successRate, 1);
        Serial.print("%)");
      }

      Serial.println();
    }

    Serial.println("----------------------------------------");

    lastHeartbeat = millis();
  }

  // Sleep until next ranging cycle (if enabled)
  powerMgr.sleepUntilNextCycle();

  delay(10);  // Small delay to prevent tight loop
}
