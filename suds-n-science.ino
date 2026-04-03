/**
 * Interactive: Suds and Science
 * File: suds-n-science.ino
 * Description: On button press, scans for RFID pucks in a beaker. The set of
 *   detected pucks is matched against a combination table; a matching combo
 *   sends a UDP command to the BrightSign player and lights the corresponding LEDs.
 *
 * Author: Isai Sanchez
 * Date: 3-12-26
 * Hardware:
 *   - MCU:      Waveshare ESP32-S3-ETH
 *   - RFID:     SparkFun M7E Hecto simultaneous UHF reader (UART)
 *   - Ethernet: onboard W5500 (SPI) — handled entirely in udp.ino
 * Notes:
 *   - Continuous reading mode with Gen2 S1 / Target AB / Dynamic Q for
 *     reliable multi-tag detection without re-read suppression issues.
 *   - Puck identity is determined by 96-bit EPC matched against Config.h values.
 *   - After a scan, detected pucks are folded into a uint8_t bitmask and
 *     compared against COMBOS[]. An exact match sends the mapped UDP command.
 *   - Puck-to-LED mapping:
 *       Yellow LED (GPIO 1)  ← EPC: E2 80 68 94 00 00 40 33 56 39 29 0A
 *       Blue   LED (GPIO 2)  ← EPC: E2 80 68 94 00 00 40 33 56 38 ED 0A
 *       Green  LED (GPIO 3)  ← EPC: E2 80 68 94 00 00 50 33 56 39 2D 0A
 *       Red    LED (GPIO 15) ← EPC: E2 80 68 94 00 00 50 33 56 38 F1 0A
 *
 * (c) Thanksgiving Point Exhibits Electronics Team — 2025
 */

#include "src/SparkFun_UHF_RFID_Reader.h"
#include "src/Config.h"

// ─── Puck Combination → UDP Command Table ────────────────────────────────────
// Each entry maps a bitmask of required pucks (OR of PUCK_BIT_* constants) to
// a UDP command string sent to the BrightSign on an exact match.
// Order matters only if masks overlap — list more-specific combos first.
// To add a new scene: append a row; no other code changes required.
struct PuckCombo {
  uint8_t mask;
  const char *command;
};

static const PuckCombo COMBOS[] = {
  { config::PUCK_BIT_YELLOW | config::PUCK_BIT_BLUE | config::PUCK_BIT_RED, "lake" },
  { config::PUCK_BIT_YELLOW | config::PUCK_BIT_GREEN | config::PUCK_BIT_RED, "utah" },
  { config::PUCK_BIT_BLUE | config::PUCK_BIT_GREEN | config::PUCK_BIT_RED, "gato" },
  { config::PUCK_BIT_YELLOW | config::PUCK_BIT_BLUE | config::PUCK_BIT_GREEN, "other" },
};
static const uint8_t NUM_COMBOS = sizeof(COMBOS) / sizeof(COMBOS[0]);

// ─── Puck Registry ───────────────────────────────────────────────────────────
// PuckConfig::bit directly stores the PUCK_BIT_* constant so the bitmask
// accumulation loop never assumes anything about array ordering.
struct PuckConfig {
  const uint8_t *epc;
  uint8_t ledPin;
  uint8_t bit;
  const char *name;
  bool detected;
};

static PuckConfig pucks[config::NUM_PUCKS] = {
  { config::PUCK_EPC_YELLOW, config::PIN_LED_YELLOW, config::PUCK_BIT_YELLOW, "Yellow", false },
  { config::PUCK_EPC_BLUE, config::PIN_LED_BLUE, config::PUCK_BIT_BLUE, "Blue", false },
  { config::PUCK_EPC_GREEN, config::PIN_LED_GREEN, config::PUCK_BIT_GREEN, "Green", false },
  { config::PUCK_EPC_RED, config::PIN_LED_RED, config::PUCK_BIT_RED, "Red", false },
};

// ─── Tag Storage ─────────────────────────────────────────────────────────────
struct DetectedTag {
  byte epc[config::EPC_MAX_BYTES];
  uint8_t epcLen;
  int8_t rssi;
  uint32_t lastSeenMs;
};

DetectedTag tagInventory[config::MAX_TAGS];
int tagCount = 0;

// ─── Module Handle + Button State ────────────────────────────────────────────
RFID rfidModule;

volatile bool pressed = false;
volatile unsigned long lastPressTime = 0;

// ─── Interrupt Service Routine ───────────────────────────────────────────────
void IRAM_ATTR buttonISR() {
  unsigned long now = millis();
  if (now - lastPressTime > config::BUTTON_DEBOUNCE_MS) {
    pressed = true;
    lastPressTime = now;
  }
}

// ─── RFID Module Initialization ──────────────────────────────────────────────
bool initializeModule() {
  rfidModule.begin(Serial1, ThingMagic_M7E_HECTO);
  delay(200);

  // Drain any startup noise before the first command
  while (Serial1.available()) { Serial1.read(); }

  rfidModule.getVersion();

  if (rfidModule.msg[0] == ERROR_WRONG_OPCODE_RESPONSE) {
    // Module was left in continuous-read mode from a prior power cycle
    Serial.println(F("  Module was mid-read — sending stop..."));
    rfidModule.stopReading();
    delay(1500);
    rfidModule.getVersion();
  }

  if (rfidModule.msg[0] != ALL_GOOD) {
    Serial.println(F("  No response — retrying at 115200..."));
    Serial1.begin(115200, SERIAL_8N1, config::RXD1, config::TXD1);
    delay(100);
    rfidModule.getVersion();
    if (rfidModule.msg[0] != ALL_GOOD) return false;
  }

  Serial.print(F("  Firmware: "));
  for (uint8_t i = 5; i < rfidModule.msg[1] + 3; i++) {
    if (rfidModule.msg[i] < 0x10) Serial.print('0');
    Serial.print(rfidModule.msg[i], HEX);
    Serial.print(' ');
  }
  Serial.println();

  // Order of these calls matters — protocol must be set before antenna or power
  rfidModule.setTagProtocol();  // GEN2 (0x05)
  rfidModule.setAntennaPort();  // TX=1, RX=1
  rfidModule.setAntennaSearchList();
  rfidModule.setRegion(REGION_NORTHAMERICA);
  rfidModule.setReadPower(config::READ_POWER);

  Serial.print(F("  Read power: "));
  Serial.print(config::READ_POWER / 100);
  Serial.print('.');
  Serial.print(config::READ_POWER % 100);
  Serial.println(F(" dBm"));

  return true;
}

// ─── Gen2 Parameter Configuration ────────────────────────────────────────────
void configureGen2Parameters() {
  Serial.println(F("\nConfiguring Gen2 parameters:"));

  // S1 session: tag flags persist ~500ms–5s, naturally suppressing re-reads
  // within a single scan window without extra dedup logic.
  Serial.print(F("  Session S1 ......... "));
  printResult(rfidModule.setGen2Session(TMR_GEN2_SESSION_S1));

  // Target AB: inventory A-flagged tags first, then B, then repeat.
  // Ensures all tags get read even when the population is large.
  Serial.print(F("  Target AB .......... "));
  printResult(rfidModule.setGen2Target(TMR_GEN2_TARGET_AB));

  // Dynamic Q: reader auto-adjusts slot count based on observed collisions.
  // initQ=3 (8 slots) is a reasonable starting point for 2–4 tags.
  Serial.print(F("  Q Dynamic (init=3) . "));
  printResult(rfidModule.setGen2Q(TMR_SR_GEN2_Q_DYNAMIC, 3, true));

  Serial.print(F("  RFMode 250/M4/20 ... "));
  printResult(rfidModule.setGen2RFmode(TMR_GEN2_RFMODE_250_M4_20));
}

// ─── Multi-Tag Scan ───────────────────────────────────────────────────────────
void performScan() {
  Serial.println(F("\n>>> SCANNING <<<\n"));

  // Reset inventory and clear LEDs from any previous scan
  tagCount = 0;
  memset(tagInventory, 0, sizeof(tagInventory));
  for (int i = 0; i < config::NUM_PUCKS; i++) {
    pucks[i].detected = false;
    digitalWrite(pucks[i].ledPin, LOW);
  }

  rfidModule.startReading();

  uint32_t scanStart = millis();
  uint16_t totalReads = 0;
  uint16_t parseErrors = 0;
  uint8_t pucksFound = 0;

  // Poll the UART stream for tag records until the window expires.
  // Early exit once all known pucks are found — no benefit in continuing.
  while (millis() - scanStart < config::SCAN_WINDOW_MS) {

    if (rfidModule.check()) {
      uint8_t responseType = rfidModule.parseResponse();

      if (responseType == RESPONSE_IS_TAGFOUND) {
        totalReads++;

        uint8_t tagDataBytes = rfidModule.getTagDataBytes();
        uint8_t epcBytes = rfidModule.getTagEPCBytes();
        int8_t rssi = rfidModule.getTagRSSI();

        if (epcBytes == 0 || epcBytes > config::EPC_MAX_BYTES) {
          parseErrors++;
          continue;
        }

        // EPC starts at byte 31 + tagDataBytes in the raw message buffer.
        // This offset is defined by the M7E Hecto read response format.
        uint8_t epcStartIdx = 31 + tagDataBytes;

        // Deduplicate within this scan window — same tag may stream repeatedly
        if (findTagByEPC(&rfidModule.msg[epcStartIdx], epcBytes) >= 0) continue;
        if (tagCount >= config::MAX_TAGS) continue;

        DetectedTag *tag = &tagInventory[tagCount];
        memcpy(tag->epc, &rfidModule.msg[epcStartIdx], epcBytes);
        tag->epcLen = epcBytes;
        tag->rssi = rssi;
        tag->lastSeenMs = millis();
        tagCount++;

        Serial.print(F("  [NEW] Tag #"));
        Serial.print(tagCount);
        Serial.print(F(" | RSSI: "));
        Serial.print(rssi);
        Serial.print(F(" dBm | EPC: "));
        printEPC(tag->epc, tag->epcLen);

        int puckIdx = matchPuck(tag->epc, tag->epcLen);
        if (puckIdx >= 0 && !pucks[puckIdx].detected) {
          pucks[puckIdx].detected = true;
          pucksFound++;
          if (pucksFound >= config::NUM_PUCKS) break;
        }

      } else if (responseType == RESPONSE_IS_TEMPTHROTTLE) {
        Serial.println(F("  WARNING: Thermal throttling!"));
      } else if (responseType == ERROR_CORRUPT_RESPONSE) {
        parseErrors++;
      }
    }

    yield();
  }

  uint32_t elapsed = millis() - scanStart;

  rfidModule.stopReading();
  delay(50);
  while (Serial1.available()) { Serial1.read(); }  // flush residual bytes

  // ── Scan summary ─────────────────────────────────────────────────────────
  Serial.println(F("\n────────────────────────────────────────"));
  Serial.print(F("SCAN COMPLETE | "));
  Serial.print(tagCount);
  Serial.print(F(" tag(s) | "));
  Serial.print(pucksFound);
  Serial.print('/');
  Serial.print(config::NUM_PUCKS);
  Serial.print(F(" pucks | "));
  Serial.print(elapsed);
  Serial.println(F(" ms"));
  if (parseErrors > 0) {
    Serial.print(F("  Parse errors: "));
    Serial.println(parseErrors);
  }
  Serial.println(F("────────────────────────────────────────"));

  if (tagCount > 0) {
    Serial.println(F("\nAll detected tags:"));
    for (int i = 0; i < tagCount; i++) {
      char buf[8];
      snprintf(buf, sizeof(buf), "  #%d", i + 1);
      Serial.print(buf);
      snprintf(buf, sizeof(buf), " %4d", tagInventory[i].rssi);
      Serial.print(buf);
      Serial.print(F(" dBm | "));
      printEPC(tagInventory[i].epc, tagInventory[i].epcLen);
    }
  } else {
    Serial.println(F("\nNo tags detected."));
    Serial.println(F("  → Try increasing READ_POWER in Config.h."));
  }

  // ── Combination matching ─────────────────────────────────────────────────
  // Fold detected pucks into a single bitmask using each puck's stored bit value.
  uint8_t detectedMask = 0;
  for (int i = 0; i < config::NUM_PUCKS; i++) {
    if (pucks[i].detected) detectedMask |= pucks[i].bit;
  }

  // Look for an exact bitmask match in the combo table.
  // Exact matching means extra pucks in the beaker do not accidentally trigger
  // a subset combo — the visitor must place the correct set.
  const char *matchedCommand = nullptr;
  for (int i = 0; i < NUM_COMBOS; i++) {
    if (detectedMask == COMBOS[i].mask) {
      matchedCommand = COMBOS[i].command;
      break;
    }
  }

  if (matchedCommand != nullptr) {
    Serial.print(F("\nCombo match [0x"));
    if (detectedMask < 0x10) Serial.print('0');
    Serial.print(detectedMask, HEX);
    Serial.print(F("] → Sending: \""));
    Serial.print(matchedCommand);
    Serial.println('"');
    sendUdpCommand(matchedCommand);  // defined in udp.ino
  } else if (detectedMask != 0) {
    Serial.print(F("\nNo combo match for mask 0x"));
    if (detectedMask < 0x10) Serial.print('0');
    Serial.println(detectedMask, HEX);
  }

  // ── Light LEDs for all detected pucks ────────────────────────────────────
  // LEDs fire regardless of combo match, giving the visitor immediate feedback
  // on which pucks were actually read.
  if (pucksFound > 0) {
    Serial.print(F("\nLEDs ON: "));
    for (int i = 0; i < config::NUM_PUCKS; i++) {
      if (pucks[i].detected) {
        digitalWrite(pucks[i].ledPin, HIGH);
        Serial.print(pucks[i].name);
        Serial.print(' ');
      }
    }
    Serial.println();

    delay(config::LED_DISPLAY_MS);

    for (int i = 0; i < config::NUM_PUCKS; i++) {
      digitalWrite(pucks[i].ledPin, LOW);
    }
    Serial.println(F("LEDs OFF."));
  }

  Serial.println(F("\nPress button to scan again...\n"));
}

// ─── Tag Helpers ─────────────────────────────────────────────────────────────

// Returns inventory index of a tag matching the given EPC, or -1 if not found.
int findTagByEPC(byte *epc, uint8_t epcLen) {
  for (int i = 0; i < tagCount; i++) {
    if (tagInventory[i].epcLen == epcLen && memcmp(tagInventory[i].epc, epc, epcLen) == 0) return i;
  }
  return -1;
}

// Returns pucks[] index whose EPC matches, or -1 if the tag is unknown.
int matchPuck(byte *epc, uint8_t epcLen) {
  if (epcLen != config::EPC_LENGTH) return -1;
  for (int i = 0; i < config::NUM_PUCKS; i++) {
    if (memcmp(epc, pucks[i].epc, config::EPC_LENGTH) == 0) return i;
  }
  return -1;
}

// ─── Utility ─────────────────────────────────────────────────────────────────

void printEPC(byte *epc, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    if (epc[i] < 0x10) Serial.print('0');
    Serial.print(epc[i], HEX);
    if (i < len - 1) Serial.print(' ');
  }
  Serial.println();
}

void printResult(bool success) {
  Serial.println(success ? F("OK") : F("FAILED"));
}

/*
 * =======================
 *         MAIN
 * =======================
 */

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Serial.println(F("\n========================================"));
  Serial.println(F("  Suds and Science — RFID + UDP"));
  Serial.println(F("========================================\n"));

  for (int i = 0; i < config::NUM_PUCKS; i++) {
    pinMode(pucks[i].ledPin, OUTPUT);
    digitalWrite(pucks[i].ledPin, LOW);
  }
  pinMode(config::BUTTON_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(config::BUTTON_PIN), buttonISR, FALLING);

  Serial1.begin(config::RFID_BAUD, SERIAL_8N1, config::RXD1, config::TXD1);

  if (!initializeModule()) {
    Serial.println(F("FATAL: RFID module not responding. Check wiring and power."));
    while (1) { delay(1000); }
  }

  configureGen2Parameters();

  // initEthernet() and all ethernet state live in udp.ino
  initEthernet();

  // Brief LED self-test — confirms all four outputs are wired correctly
  Serial.println(F("\nLED test..."));
  for (int i = 0; i < config::NUM_PUCKS; i++) digitalWrite(pucks[i].ledPin, HIGH);
  delay(500);
  for (int i = 0; i < config::NUM_PUCKS; i++) digitalWrite(pucks[i].ledPin, LOW);

  Serial.println(F("\n────────────────────────────────────────"));
  Serial.println(F("Place pucks in beaker, then press button to scan."));
  Serial.println(F("────────────────────────────────────────\n"));
}

void loop() {
  updateLinkStatus();  // detect cable connect/disconnect events (defined in udp.ino)

  if (pressed) {
    Serial.println(F("\nButton pressed!"));
    pressed = false;
    performScan();
  }

  maintainEthernet();  // DHCP renewal housekeeping (defined in udp.ino)
}
