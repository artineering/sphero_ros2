/**
 * UWB RTLS Anchor Firmware - WiFi/UDP Version
 * ============================================
 * Platform: Arduino Portenta C33 + Portenta UWB Shield
 * Library:  PortentaUWBShield (Truesense)
 *
 * Each anchor creates TWR sessions to all configured tags (Stella boards).
 * Range measurements are transmitted over WiFi/UDP to a ROS2 host as JSON packets:
 *   {"anchor_id":1,"tag_id":3,"distance_cm":142,"timestamp_ms":12345}
 *
 * CONFIGURATION:
 *   - Set ANCHOR_ID (1-4) uniquely per anchor before flashing
 *   - Set NUM_TAGS to the number of Stella tags in your deployment
 *   - Configure WiFi credentials (WIFI_SSID, WIFI_PASS)
 *   - Set UDP_HOST to your ROS2 host IP address
 *   - Preamble codes are auto-assigned per session to avoid collisions
 */

#include <PortentaUWBShield.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ============================================================
// USER CONFIGURATION — change these per anchor before flashing
// ============================================================
#define ANCHOR_ID       1       // Unique anchor identifier (1–4)
#define NUM_TAGS        10      // Total number of Stella tags
#define SERIAL_BAUD     115200
#define PREAMBLE_BASE   9       // Base preamble code (valid: 9–12 for ch9)

// WiFi and UDP configuration
#define WIFI_SSID       "YourNetworkName"     // Change this to your WiFi SSID
#define WIFI_PASS       "YourPassword"        // Change this to your WiFi password
#define UDP_HOST        "192.168.1.100"       // ROS2 host IP address
#define UDP_PORT        5000                  // UDP destination port
#define WIFI_RETRY_MS   5000                  // Reconnect interval (ms)
#define SERIAL_DEBUG                          // Comment out to disable debug prints

// ============================================================
// MAC address scheme (SHORT, 2-byte):
//   Anchor N  → {0xA0 + N, 0x01}  e.g. Anchor 1 = {0xA1, 0x01}
//   Tag    M  → {0xT0 + M, 0x01}  e.g. Tag 3    = {0x03, 0x01}
//
// Session ID scheme:
//   0x<AnchorHex><TagHex>  e.g. Anchor 1 + Tag 3 = 0x010003
// ============================================================

// WiFi and UDP objects
WiFiUDP udp;
bool wifiConnected = false;
unsigned long lastWifiAttempt = 0;

// Storage for session objects — must persist beyond setup()
UWBMultiSessionAnchor* sessions[NUM_TAGS];

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

// Ranging callback — fires for every TWR measurement
void rangingHandler(UWBRangingData &rangingData) {
  if (rangingData.measureType() != (uint8_t)uwb::MeasurementType::TWO_WAY) {
    return;
  }

  RangingMeasures twr = rangingData.twoWayRangingMeasure();
  uint32_t sessionHandle = rangingData.sessionHandle();

  // Extract tag_id from session handle (lower 16 bits)
  int tag_id = sessionHandle & 0xFF;

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
}

void setup() {
  Serial.begin(SERIAL_BAUD);

  // Status LED
#if defined(ARDUINO_PORTENTA_C33)
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  digitalWrite(LEDR, HIGH);  // off (active low)
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);
  digitalWrite(LEDR, LOW);   // red on during init
#endif

  // Register callback and initialize UWB stack
  UWB.registerRangingCallback(rangingHandler);
  UWB.begin();

  Serial.print("# Anchor ");
  Serial.print(ANCHOR_ID);
  Serial.println(" — Initializing UWB...");

  // Wait for UWB stack to be ready
  while (UWB.state() != 0) {
    delay(10);
  }
  Serial.println("# UWB ready.");

  // Build anchor MAC address: {0xA0 + ANCHOR_ID, 0x01}
  uint8_t anchorAddr[] = {(uint8_t)(0xA0 + ANCHOR_ID), 0x01};
  UWBMacAddress anchorMac(UWBMacAddress::Size::SHORT, anchorAddr);

  // Create a TWR session for each tag
  for (int t = 1; t <= NUM_TAGS; t++) {
    uint8_t tagAddr[] = {(uint8_t)t, 0x01};
    UWBMacAddress tagMac(UWBMacAddress::Size::SHORT, tagAddr);

    // Session ID encodes anchor + tag: e.g. 0x010003 for anchor 1, tag 3
    uint32_t sessionId = ((uint32_t)ANCHOR_ID << 16) | (uint32_t)t;

    // Preamble code: use PREAMBLE_BASE offset by anchor to reduce interference
    // Valid codes for channel 9: 9, 10, 11, 12 (short codes) or 25-32 (long)
    uint8_t preamble = PREAMBLE_BASE + ((ANCHOR_ID - 1) % 4);

    sessions[t - 1] = new UWBMultiSessionAnchor(sessionId, anchorMac, tagMac, preamble);

    UWBSessionManager.addSession(*sessions[t - 1]);
    sessions[t - 1]->init();

    Serial.print("# Session created: anchor=");
    Serial.print(ANCHOR_ID);
    Serial.print(" tag=");
    Serial.print(t);
    Serial.print(" sessionId=0x");
    Serial.print(sessionId, HEX);
    Serial.print(" preamble=");
    Serial.println(preamble);
  }

  // Start all sessions
  for (int t = 0; t < NUM_TAGS; t++) {
    sessions[t]->start();
    delay(50);  // Small stagger to avoid bus contention
  }

  Serial.println("# All sessions started. Ranging active.");

  // Initialize WiFi
  connectWiFi();

  // Begin UDP (required for sending on some WiFi stacks)
  udp.begin(UDP_PORT);

  Serial.println("# UDP initialized.");

#if defined(ARDUINO_PORTENTA_C33)
  digitalWrite(LEDR, HIGH);  // red off
  digitalWrite(LEDG, LOW);   // green on = running
#endif
}

void loop() {
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

  // Heartbeat every 5 seconds
  static unsigned long lastHeartbeat = 0;
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
  delay(10);
}
