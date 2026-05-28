/*
 * UWB_MulticastTag.ino
 *
 * Platform : Arduino Stella (Truesense DCU040 / nRF52840)
 * Library  : StellaUWB, ArduinoBLE
 *
 * Role     : Multicast Responder / Controlee — joins four multicast
 *            sessions (one per anchor) and broadcasts raw distance
 *            measurements over BLE advertising.  Position computation
 *            is deferred to the receiver so the tag stays independent
 *            of anchor geometry.
 *
 * -- CONFIGURATION -----------------------------------------------------
 *   Change ONLY the defines in the "CHANGE THESE" section below.
 *
 * -- ADDRESSING SCHEME -------------------------------------------------
 *   Tag    MAC      : { 0x00, TAG_ID }
 *   Anchor MAC      : { 0xA0, ANCHOR_ID }    ANCHOR_ID in [0..3]
 *   Group assignment : group = (TAG_ID - 1) / TAGS_PER_GROUP
 *     Tags  1- 6 -> Group 0
 *     Tags  7-12 -> Group 1
 *   Session ID      : ANCHOR_ID * 100 + GROUP_INDEX + 1
 *
 * -- BLE ADVERTISING FORMAT --------------------------------------------
 *   Local name : "UWB-T<id>"
 *   Manufacturer data (11 bytes):
 *     [0..1]  Company ID 0xFFFF (little-endian, BLE SIG test/dev)
 *     [2]     TAG_ID
 *     [3..4]  distance to A0 in cm (uint16, little-endian, 0xFFFF = invalid)
 *     [5..6]  distance to A1 in cm (uint16, little-endian, 0xFFFF = invalid)
 *     [7..8]  distance to A2 in cm (uint16, little-endian, 0xFFFF = invalid)
 *     [9..10] distance to A3 in cm (uint16, little-endian, 0xFFFF = invalid)
 *
 * -- SESSION BUDGET ----------------------------------------------------
 *   4 sessions per tag     (limit: 5 on DCU040)
 *   2 sessions per anchor  (limit: 5 on SR150)
 * ----------------------------------------------------------------------
 */

#include "StellaUWB.h"
#include <ArduinoBLE.h>

// ===================== CHANGE THESE ==================================
#define TAG_ID            1        // 1 through 12
#define SERIAL_DEBUG      0        // 1 = verbose serial output, 0 = silent (saves power)
// =====================================================================

#define NUM_ANCHORS     4
#define TAGS_PER_GROUP  6
#define GROUP_INDEX     (((TAG_ID) - 1) / TAGS_PER_GROUP)

#define STALE_TIMEOUT_MS  2500
#define INVALID_DIST      0xFFFF
#define BLE_UPDATE_MIN_MS 100

// -- State ------------------------------------------------------------

static MulticastResponder* sessions[NUM_ANCHORS] = { nullptr };

struct AnchorRange {
  uint16_t distance_cm;
  bool     valid;
  uint32_t timestamp;
};

static AnchorRange ranges[NUM_ANCHORS];
static uint16_t prevBle[NUM_ANCHORS];

static volatile uint32_t rangingCount = 0;
static uint32_t bleAdvCount = 0;
static uint32_t bleSkipThrottle = 0;
static uint32_t bleSkipNoChange = 0;

// -- BLE advertising --------------------------------------------------

static void updateBLEAdvertising() {
  static unsigned long lastBleUpdate = 0;
  if (millis() - lastBleUpdate < BLE_UPDATE_MIN_MS) { bleSkipThrottle++; return; }

  lastBleUpdate = millis();

  uint16_t cur[NUM_ANCHORS];
  for (int a = 0; a < NUM_ANCHORS; a++)
    cur[a] = ranges[a].valid ? ranges[a].distance_cm : INVALID_DIST;

  BLE.poll();
  BLE.stopAdvertise();

  uint8_t mfgData[11];
  mfgData[0] = 0xFF;
  mfgData[1] = 0xFF;
  mfgData[2] = (uint8_t)TAG_ID;
  for (int a = 0; a < NUM_ANCHORS; a++) {
    mfgData[3 + a * 2]     = (uint8_t)(cur[a] & 0xFF);
    mfgData[3 + a * 2 + 1] = (uint8_t)((cur[a] >> 8) & 0xFF);
  }

  BLEAdvertisingData advData;
  advData.setManufacturerData(mfgData, sizeof(mfgData));

  char name[16];
  snprintf(name, sizeof(name), "UWB-T%d", TAG_ID);
  advData.setLocalName(name);

  BLE.setAdvertisingData(advData);
  BLE.advertise();
  bleAdvCount++;
  delay(10);

  for (int a = 0; a < NUM_ANCHORS; a++)
    prevBle[a] = cur[a];
}

// -- Ranging callback -------------------------------------------------

void rangingHandler(UWBRangingData &rangingData) {
  if (rangingData.measureType() != (uint8_t)uwb::MeasurementType::TWO_WAY)
    return;

  uint32_t sessionHandle = rangingData.sessionHandle();
  int anchorId = (int)(sessionHandle / 100);

  if (anchorId < 0 || anchorId >= NUM_ANCHORS)
    return;

  RangingMeasures twr = rangingData.twoWayRangingMeasure();

  for (int j = 0; j < rangingData.available(); j++) {
    if (twr[j].status == 0 && twr[j].distance != 0xFFFF) {
      ranges[anchorId].distance_cm = (uint16_t)twr[j].distance;
      ranges[anchorId].valid       = true;
      ranges[anchorId].timestamp   = millis();
    }
  }
  rangingCount++;
}

// -- Setup ------------------------------------------------------------

void setup() {
  Serial.begin(115200);

  if (!BLE.begin()) {
    Serial.println("BLE: init failed!");
  } else {
    char name[16];
    snprintf(name, sizeof(name), "UWB-T%d", TAG_ID);
    BLE.setLocalName(name);

    uint8_t initMfg[11] = { 0xFF, 0xFF, (uint8_t)TAG_ID,
                             0xFF, 0xFF, 0xFF, 0xFF,
                             0xFF, 0xFF, 0xFF, 0xFF };
    BLEAdvertisingData initAdv;
    initAdv.setManufacturerData(initMfg, sizeof(initMfg));
    initAdv.setLocalName(name);
    BLE.setAdvertisingData(initAdv);
    BLE.advertise();

    Serial.print("BLE: advertising as ");
    Serial.println(name);
  }

  uint8_t tagAddrBytes[] = { 0x00, (uint8_t)TAG_ID };
  UWBMacAddress tagMac(UWBMacAddress::Size::SHORT, tagAddrBytes);

  UWB.begin();
  delay(1000);
  Serial.println("=========================================");
  Serial.print("  Multicast Tag T");
  Serial.print(TAG_ID);
  Serial.print("  MAC {0x00, 0x");
  if (TAG_ID < 0x10) Serial.print("0");
  Serial.print(TAG_ID, HEX);
  Serial.print("}  Group ");
  Serial.println(GROUP_INDEX);
  Serial.println("  Raw distance BLE advertising");
  Serial.println("=========================================");
  Serial.println("Waiting for UWB stack ...");

  while (UWB.state() != 0)
    delay(10);

  Serial.println("UWB stack ready.");

  for (int a = 0; a < NUM_ANCHORS; a++) {
    ranges[a].distance_cm = 0;
    ranges[a].valid       = false;
    ranges[a].timestamp   = 0;
    prevBle[a]            = INVALID_DIST;
  }

  // Phase 1: create and register all sessions (no SPI ranging traffic yet)
  for (int a = 0; a < NUM_ANCHORS; a++) {
    uint8_t anchorAddrBytes[] = { 0xA0, (uint8_t)a };
    UWBMacAddress anchorMac(UWBMacAddress::Size::SHORT, anchorAddrBytes);

    uint32_t sessionId = (uint32_t)(a * 100 + GROUP_INDEX + 1);

    Serial.print("  Creating session ");
    Serial.print(sessionId);
    Serial.print("  ->  Anchor A");
    Serial.print(a);
    Serial.print("  (MAC {0xA0, 0x0");
    Serial.print(a);
    Serial.println("})");

    sessions[a] = new MulticastResponder(sessionId, tagMac, anchorMac);
    UWBSessionManager.addSession(*sessions[a]);
    delay(200);
  }

  // Phase 2: init and start each session sequentially
  for (int a = 0; a < NUM_ANCHORS; a++) {
    Serial.print("  Starting session A");
    Serial.println(a);
    sessions[a]->init();
    delay(200);
    sessions[a]->start();
    delay(200);
  }

  UWB.registerRangingCallback(rangingHandler);

  Serial.println("-----------------------------------------");
  Serial.println("All responder sessions active.");
  Serial.println("-----------------------------------------");
}

// -- Loop -------------------------------------------------------------

void loop() {
  for (int a = 0; a < NUM_ANCHORS; a++) {
    if (ranges[a].valid && (millis() - ranges[a].timestamp > STALE_TIMEOUT_MS))
      ranges[a].valid = false;
  }

  updateBLEAdvertising();

  static unsigned long lastStats = 0;
  if (millis() - lastStats >= 5000) {
    float elapsed = (millis() - lastStats) / 1000.0;
    Serial.print("STATS | ranging=");
    Serial.print(rangingCount / elapsed, 1);
    Serial.print("/s  bleAdv=");
    Serial.print(bleAdvCount / elapsed, 1);
    Serial.print("/s  skipThrottle=");
    Serial.print(bleSkipThrottle);
    Serial.print("  skipNoChange=");
    Serial.println(bleSkipNoChange);
    rangingCount = 0;
    bleAdvCount = 0;
    bleSkipThrottle = 0;
    bleSkipNoChange = 0;
    lastStats = millis();
  }

#if SERIAL_DEBUG
  Serial.print("T");
  Serial.print(TAG_ID);
  Serial.print(" | ");

  int validCount = 0;
  for (int a = 0; a < NUM_ANCHORS; a++) {
    Serial.print("A");
    Serial.print(a);
    Serial.print("=");
    if (ranges[a].valid) {
      Serial.print(ranges[a].distance_cm);
      Serial.print("cm");
      validCount++;
    } else {
      Serial.print("---");
    }
    if (a < NUM_ANCHORS - 1) Serial.print("  ");
  }

  Serial.print("  [");
  Serial.print(validCount);
  Serial.print("/4]");

  Serial.println();
#endif

  delay(100);
}
