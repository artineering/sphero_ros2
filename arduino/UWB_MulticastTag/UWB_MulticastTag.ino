/*
 * UWB_MulticastTag.ino
 *
 * Platform : Arduino Stella (Truesense DCU040 / nRF52840)
 * Library  : StellaUWB, ArduinoBLE
 *
 * Role     : Multicast Responder / Controlee — joins four multicast
 *            sessions (one per anchor), computes 2D position via
 *            multilateration, and broadcasts the result over BLE
 *            advertising.
 *
 * ── CONFIGURATION ──────────────────────────────────────────────────
 *   Change ONLY the defines in the "CHANGE THESE" section below.
 *
 * ── ADDRESSING SCHEME ──────────────────────────────────────────────
 *   Tag    MAC      : { 0x00, TAG_ID }
 *   Anchor MAC      : { 0xA0, ANCHOR_ID }    ANCHOR_ID ∈ [0..3]
 *   Group assignment : group = (TAG_ID - 1) / TAGS_PER_GROUP
 *     Tags  1– 6 → Group 0
 *     Tags  7–12 → Group 1
 *   Session ID      : ANCHOR_ID * 100 + GROUP_INDEX + 1
 *
 * ── BLE ADVERTISING FORMAT ─────────────────────────────────────────
 *   Local name : "UWB-T<id>"
 *   Manufacturer data (7 bytes):
 *     [0..1] Company ID 0xFFFF (little-endian, BLE SIG test/dev)
 *     [2]    TAG_ID
 *     [3..4] x position in cm (int16, little-endian)
 *     [5..6] y position in cm (int16, little-endian)
 *
 * ── SESSION BUDGET ─────────────────────────────────────────────────
 *   4 sessions per tag     (limit: 5 on DCU040)  ✔
 *   2 sessions per anchor  (limit: 5 on SR150)   ✔
 * ───────────────────────────────────────────────────────────────────
 */

#include "StellaUWB.h"
#include <ArduinoBLE.h>
#include <math.h>

// ===================== CHANGE THESE ==================================
#define TAG_ID            1        // 1 through 12

// Anchor positions (cm) — 10 ft × 10 ft square, corners
//                        A0        A1        A2        A3
#define ANCHOR_X  {   0.0f, 305.0f, 305.0f,   0.0f }
#define ANCHOR_Y  {   0.0f,   0.0f, 305.0f, 305.0f }

// Heights (cm): anchor antenna ~10 in, tag ~2 in
#define ANCHOR_HEIGHT_CM  25.0f
#define TAG_HEIGHT_CM      5.0f
// =====================================================================

#define NUM_ANCHORS     4
#define TAGS_PER_GROUP  6
#define GROUP_INDEX     (((TAG_ID) - 1) / TAGS_PER_GROUP)

// UWB ranging resolution
#define POSITION_RES_CM   10
#define STALE_TIMEOUT_MS  2500

static const float anchorX[NUM_ANCHORS] = ANCHOR_X;
static const float anchorY[NUM_ANCHORS] = ANCHOR_Y;

// ── State ────────────────────────────────────────────────────────

static MulticastResponder* sessions[NUM_ANCHORS] = { nullptr };

struct AnchorRange {
  float    distance_cm;
  bool     valid;
  uint32_t timestamp;
};

static AnchorRange ranges[NUM_ANCHORS];

static int16_t posX_cm   = 0;
static int16_t posY_cm   = 0;
static bool    posValid  = false;
static int16_t prevBleX  = INT16_MIN;
static int16_t prevBleY  = INT16_MIN;

// ── Height-corrected horizontal distance ─────────────────────────

static float horizontalDistance(float slant_cm) {
  float dz = ANCHOR_HEIGHT_CM - TAG_HEIGHT_CM;
  float sq = slant_cm * slant_cm - dz * dz;
  return (sq > 0.0f) ? sqrtf(sq) : 0.0f;
}

// ── 2D multilateration (linearized least-squares) ────────────────
//
// Subtracts the first valid anchor's equation from the rest to
// eliminate the quadratic term, then solves the overdetermined
// 2×2 system via normal equations (A^T A)^-1 A^T b.

static bool computePosition(float* outX, float* outY) {
  int   vi[NUM_ANCHORS];
  float hd[NUM_ANCHORS];
  int   n = 0;

  for (int a = 0; a < NUM_ANCHORS; a++) {
    if (ranges[a].valid) {
      vi[n] = a;
      hd[n] = horizontalDistance(ranges[a].distance_cm);
      n++;
    }
  }

  if (n < 3) return false;

  int   ref  = vi[0];
  float x0   = anchorX[ref];
  float y0   = anchorY[ref];
  float d0sq = hd[0] * hd[0];

  float ata00 = 0, ata01 = 0, ata11 = 0;
  float atb0  = 0, atb1  = 0;

  for (int i = 1; i < n; i++) {
    int   idx = vi[i];
    float ax  = 2.0f * (anchorX[idx] - x0);
    float ay  = 2.0f * (anchorY[idx] - y0);
    float bi  = (anchorX[idx] * anchorX[idx] - x0 * x0)
              + (anchorY[idx] * anchorY[idx] - y0 * y0)
              - (hd[i] * hd[i] - d0sq);

    ata00 += ax * ax;
    ata01 += ax * ay;
    ata11 += ay * ay;
    atb0  += ax * bi;
    atb1  += ay * bi;
  }

  float det = ata00 * ata11 - ata01 * ata01;
  if (fabsf(det) < 1e-6f) return false;

  float inv = 1.0f / det;
  *outX = ( ata11 * atb0 - ata01 * atb1) * inv;
  *outY = (ata00 * atb1 - ata01 * atb0) * inv;
  return true;
}

static int16_t roundToRes(float v) {
  return (int16_t)(roundf(v / POSITION_RES_CM) * POSITION_RES_CM);
}

// ── BLE advertising ──────────────────────────────────────────────

static void updateBLEAdvertising() {
  if (posX_cm == prevBleX && posY_cm == prevBleY)
    return;

  BLE.stopAdvertise();

  uint8_t mfgData[7];
  mfgData[0] = 0xFF;
  mfgData[1] = 0xFF;
  mfgData[2] = (uint8_t)TAG_ID;
  mfgData[3] = (uint8_t)(posX_cm & 0xFF);
  mfgData[4] = (uint8_t)((posX_cm >> 8) & 0xFF);
  mfgData[5] = (uint8_t)(posY_cm & 0xFF);
  mfgData[6] = (uint8_t)((posY_cm >> 8) & 0xFF);

  BLEAdvertisingData advData;
  advData.setManufacturerData(mfgData, sizeof(mfgData));

  char name[16];
  snprintf(name, sizeof(name), "UWB-T%d", TAG_ID);
  advData.setLocalName(name);

  BLE.setAdvertisingData(advData);
  BLE.advertise();

  prevBleX = posX_cm;
  prevBleY = posY_cm;
}

// ── Ranging callback ─────────────────────────────────────────────

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
      ranges[anchorId].distance_cm = (float)twr[j].distance;
      ranges[anchorId].valid       = true;
      ranges[anchorId].timestamp   = millis();
    }
  }
}

// ── Setup ────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);

  if (!BLE.begin()) {
    Serial.println("BLE: init failed!");
  } else {
    char name[16];
    snprintf(name, sizeof(name), "UWB-T%d", TAG_ID);
    BLE.setLocalName(name);
    Serial.print("BLE: advertising as ");
    Serial.println(name);
  }

  uint8_t tagAddrBytes[] = { 0x00, (uint8_t)TAG_ID };
  UWBMacAddress tagMac(UWBMacAddress::Size::SHORT, tagAddrBytes);

  UWB.registerRangingCallback(rangingHandler);

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
  Serial.println("  2D multilateration + BLE position adv");
  Serial.println("=========================================");
  Serial.println("Waiting for UWB stack ...");

  while (UWB.state() != 0)
    delay(10);

  Serial.println("UWB stack ready.");

  for (int a = 0; a < NUM_ANCHORS; a++) {
    ranges[a].distance_cm = 0;
    ranges[a].valid       = false;
    ranges[a].timestamp   = 0;
  }

  for (int a = 0; a < NUM_ANCHORS; a++) {
    uint8_t anchorAddrBytes[] = { 0xA0, (uint8_t)a };
    UWBMacAddress anchorMac(UWBMacAddress::Size::SHORT, anchorAddrBytes);

    uint32_t sessionId = (uint32_t)(a * 100 + GROUP_INDEX + 1);

    Serial.print("  Joining session ");
    Serial.print(sessionId);
    Serial.print("  ->  Anchor A");
    Serial.print(a);
    Serial.print("  (MAC {0xA0, 0x0");
    Serial.print(a);
    Serial.println("})");

    sessions[a] = new MulticastResponder(sessionId, tagMac, anchorMac);

    UWBSessionManager.addSession(*sessions[a]);
    delay(100);
    sessions[a]->init();
    delay(100);
    sessions[a]->start();
    delay(100);
  }

  Serial.println("-----------------------------------------");
  Serial.println("All responder sessions active.");
  Serial.println("-----------------------------------------");
}

// ── Loop ─────────────────────────────────────────────────────────

void loop() {
  BLE.poll();

  for (int a = 0; a < NUM_ANCHORS; a++) {
    if (ranges[a].valid && (millis() - ranges[a].timestamp > STALE_TIMEOUT_MS))
      ranges[a].valid = false;
  }

  float x, y;
  if (computePosition(&x, &y)) {
    posX_cm  = roundToRes(x);
    posY_cm  = roundToRes(y);
    posValid = true;
    updateBLEAdvertising();
  } else {
    posValid = false;
  }

  Serial.print("T");
  Serial.print(TAG_ID);
  Serial.print(" | ");

  int validCount = 0;
  for (int a = 0; a < NUM_ANCHORS; a++) {
    Serial.print("A");
    Serial.print(a);
    Serial.print("=");
    if (ranges[a].valid) {
      Serial.print((int)ranges[a].distance_cm);
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

  if (posValid) {
    Serial.print("  pos=(");
    Serial.print(posX_cm);
    Serial.print(",");
    Serial.print(posY_cm);
    Serial.print(")cm");
  }

  Serial.println();

  delay(10);
}
