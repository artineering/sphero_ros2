/*
 * UWB_MulticastAnchor.ino
 *
 * Platform : Arduino Portenta C33 + UWB Shield (Truesense DCU150)
 * Library  : PortentaUWBShield
 *            https://github.com/Truesense-it/PortentaUWBShield
 *
 * Role     : UWB Ranging Controller (one-to-many) — acts as a fixed
 *            anchor that ranges to up to 12 Stella tags split across
 *            two multicast sessions (6 tags each).
 *
 *
 * ── CONFIGURATION ──────────────────────────────────────────────────
 *   Change ONLY the defines in the "CHANGE THESE" section below.
 *   - ANCHOR_ID    : unique per anchor (0–3)
 *
 * ── ADDRESSING SCHEME ──────────────────────────────────────────────
 *   Anchor MAC      : { 0xA0, ANCHOR_ID }
 *   Tag    MAC      : { 0x00, TAG_ID }        TAG_ID ∈ [1..12]
 *   Session ID      : ANCHOR_ID * 100 + GROUP_INDEX + 1
 *     Group 0 (tags  1– 6) → session ANCHOR_ID*100 + 1
 *     Group 1 (tags  7–12) → session ANCHOR_ID*100 + 2
 *
 * ── SESSION BUDGET ─────────────────────────────────────────────────
 *   2 sessions per anchor  (limit: 5 on SR150)  ✔
 *   4 sessions per tag     (limit: 5 on DCU040)  ✔
 * ───────────────────────────────────────────────────────────────────
 */

#include <PortentaUWBShield.h>

// ===================== CHANGE THESE ===============================
#define ANCHOR_ID       3                       // 0, 1, 2, or 3
// ==================================================================

// Uncomment to echo JSON lines to Serial for USB debugging
// #define SERIAL_DEBUG

#define NUM_TAGS        12
#define TAGS_PER_GROUP  6       // max 8 controlees per multicast session
#define NUM_GROUPS      ((NUM_TAGS + TAGS_PER_GROUP - 1) / TAGS_PER_GROUP)

// ── File-static state ────────────────────────────────────────────

// Heap-allocated session objects — survive beyond setup()
static UWBRangingOneToMany* sessions[NUM_GROUPS] = { nullptr };

// ── Ranging callback ─────────────────────────────────────────────
//
// Called asynchronously by the UWB stack each time a ranging round
// completes.  Within a multicast session the index j corresponds to
// the j-th controlee added to the destination list (i.e. the j-th
// tag in that group, 0-indexed).
//
void rangingHandler(UWBRangingData &rangingData) {
  uint8_t measType = rangingData.measureType();
  uint8_t twoWay = (uint8_t)uwb::MeasurementType::TWO_WAY;
  bool valueReceived = false;
  
  if (measType != twoWay) {
    Serial.println("incorrect measurement received");
    return;
  }
    

  uint32_t sessionHandle = rangingData.sessionHandle();

  // Determine which group produced this callback
  int groupIndex = (int)(sessionHandle % 100) - 1;   // 0 or 1
  int tagBase    = groupIndex * TAGS_PER_GROUP + 1;   // first TAG_ID in group

  RangingMeasures twr = rangingData.twoWayRangingMeasure();

  for (int j = 0; j < rangingData.available(); j++) {
    if (twr[j].status == 0 && twr[j].distance != 0xFFFF) {
      Serial.print("Tag ID: ");
      Serial.print(tagBase + j);
      Serial.print(" | Distance(cm): ");
      Serial.print(twr[j].distance);
      Serial.print(" | ");
      valueReceived |= true;
    }
  }
  if (valueReceived) {
    Serial.println("");
  }
}

bool bLED_ON = false;
unsigned int lastMillis = 0;

// ── Setup ────────────────────────────────────────────────────────
void setup() {
  #if defined(ARDUINO_PORTENTA_C33)
    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    digitalWrite(LEDR, HIGH);   // all LEDs off (active-low)
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, HIGH);
    digitalWrite(LEDR, LOW);    // red on during init
  #endif

  Serial.begin(115200);
  delay(1000);

  Serial.println("Starting UWB ...");
  // ── Anchor MAC address ──
  uint8_t anchorAddrBytes[] = { 0xA0, (uint8_t)ANCHOR_ID };
  UWBMacAddress anchorMac(UWBMacAddress::Size::SHORT, anchorAddrBytes);

  // Register callback before starting the stack
  UWB.registerRangingCallback(rangingHandler);

  UWB.begin();
  delay(3000);

  Serial.println("=========================================");
  Serial.print("  Multicast Anchor A");
  Serial.print(ANCHOR_ID);
  Serial.print("  MAC {0xA0, 0x0");
  Serial.print(ANCHOR_ID);
  Serial.println("}");

  Serial.print("Waiting for UWB state to stabilize...");
  while (UWB.state() != 0)
  {
    Serial.print(".");
    delay(10);
  }
  Serial.println("UWB stack ready.");

  // ── Create one multicast session per group ──
  for (int g = 0; g < NUM_GROUPS; g++) {

    int firstTag = g * TAGS_PER_GROUP + 1;
    int lastTag  = min(firstTag + TAGS_PER_GROUP - 1, NUM_TAGS);

    // Build destination list for this group
    UWBMacAddressList destList(UWBMacAddress::Size::SHORT);

    for (int t = firstTag; t <= lastTag; t++) {
      uint8_t tagAddrBytes[] = { 0x00, (uint8_t)t };
      UWBMacAddress tagMac(UWBMacAddress::Size::SHORT, tagAddrBytes);
      destList.add(tagMac);
    }

    uint32_t sessionId = (uint32_t)(ANCHOR_ID * 100 + g + 1);

    Serial.print("  Session ");
    Serial.print(sessionId);
    Serial.print("  →  T");
    Serial.print(firstTag);
    Serial.print("–T");
    Serial.println(lastTag);

    // Heap-allocate so the object outlives setup()
    sessions[g] = new UWBRangingOneToMany(sessionId, anchorMac, destList);
    sessions[g]->appParams.rangingDuration(250);
    UWBSessionManager.addSession(*sessions[g]);
    delay(100);
    sessions[g]->init();
    delay(100);
    sessions[g]->start();
    delay(100);
  }

  Serial.println("All multicast sessions active.");

  Serial.println("-----------------------------------------");
  Serial.print("Anchor A");
  Serial.print(ANCHOR_ID);
  Serial.println(" ready.");
  Serial.println("-----------------------------------------");

  digitalWrite(LEDR, HIGH);

  lastMillis = millis();
}

// ── Loop: WiFi watchdog + heartbeat ─────────────────────────────
void loop() {
  unsigned int now = millis();
  if ((now - lastMillis) > 500) {
    if (bLED_ON) {
      digitalWrite(LEDG, HIGH);
    } else {
      digitalWrite(LEDG, LOW);
    }
    bLED_ON = !bLED_ON;
    lastMillis = millis();
  }
  delay(10);
}
