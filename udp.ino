/**
 * File: udp.ino
 * Description: Ethernet (W5500) initialization and UDP send utilities.
 *   This file contains no setup() or loop(). It is compiled as part of the
 *   same translation unit as suds-n-science.ino — Arduino concatenates all
 *   .ino files in the sketch directory before compilation, so functions and
 *   globals defined here are directly accessible from the main sketch.
 *
 * (c) Thanksgiving Point Exhibits Electronics Team — 2025
 */

#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include "src/Config.h"

// ─── Ethernet State ───────────────────────────────────────────────────────────
EthernetUDP udp;
bool ethernetInitialized = false;
bool linkUp = false;

// ─── Ethernet Initialization ─────────────────────────────────────────────────
void initEthernet() {
  Serial.println(F("\nInitializing Ethernet (W5500)..."));

  SPI.begin(config::W5500_SCK, config::W5500_MISO, config::W5500_MOSI, config::W5500_CS);

  // Hardware reset: hold RST low briefly to guarantee a clean chip state
  // independent of prior sketch or power-cycle conditions.
  pinMode(config::W5500_RST, OUTPUT);
  digitalWrite(config::W5500_RST, LOW);
  delay(100);
  digitalWrite(config::W5500_RST, HIGH);
  delay(500);  // W5500 datasheet requires ~150ms post-reset stabilization

  Ethernet.init(config::W5500_CS);

  // Ethernet.begin() requires a non-const MAC buffer; copy from Config.h
  uint8_t mac[6];
  memcpy(mac, config::MAC_ADDR, sizeof(mac));
  Ethernet.begin(mac, IPAddress(config::LOCAL_IP));

  switch (Ethernet.hardwareStatus()) {
    case EthernetW5500:
      Serial.println(F("  W5500 detected."));
      ethernetInitialized = true;
      break;
    case EthernetNoHardware:
      Serial.println(F("  ERROR: W5500 not found. Check SPI wiring. UDP disabled."));
      return;
    default:
      Serial.println(F("  ERROR: Unexpected hardware detected. UDP disabled."));
      return;
  }

  // Non-blocking link wait — log and continue if the cable is not yet connected.
  // linkUp will be set to true by updateLinkStatus() once the cable is plugged in.
  Serial.print(F("  Waiting for link"));
  unsigned long t = millis();
  while (Ethernet.linkStatus() != LinkON) {
    if (millis() - t > config::LINK_TIMEOUT_MS) {
      Serial.println(F("\n  TIMEOUT: no link. Continuing — link may come up later."));
      break;
    }
    delay(500);
    Serial.print('.');
  }

  if (Ethernet.linkStatus() == LinkON) {
    Serial.println(F("\n  Link UP."));
    linkUp = true;
  }

  udp.begin(config::UDP_PORT);
  Serial.print(F("  UDP bound to port "));
  Serial.println(config::UDP_PORT);
}

// ─── UDP Send ─────────────────────────────────────────────────────────────────
// Sends a null-terminated command string as a raw UDP payload to the BrightSign.
// Silent no-op if ethernet was not initialized or the link is currently down.
void sendUdpCommand(const char *command) {
  if (!ethernetInitialized || !linkUp) {
    Serial.println(F("  UDP skipped — ethernet not ready."));
    return;
  }

  udp.beginPacket(IPAddress(config::TARGET_IP), config::UDP_PORT);
  udp.write(reinterpret_cast<const uint8_t *>(command), strlen(command));
  udp.endPacket();

  Serial.print(F("  UDP sent: \""));
  Serial.print(command);
  Serial.println('"');
}

// ─── Link Monitor ─────────────────────────────────────────────────────────────
// Tracks ethernet link state transitions and logs them. Called every loop()
// iteration; uses a static local to avoid redundant prints on stable state.
void updateLinkStatus() {
  static EthernetLinkStatus lastStatus = Unknown;

  EthernetLinkStatus current = Ethernet.linkStatus();
  if (current == lastStatus) return;
  lastStatus = current;

  switch (current) {
    case LinkON:
      Serial.println(F("[ETH] Link UP"));
      linkUp = true;
      break;
    case LinkOFF:
      Serial.println(F("[ETH] Link DOWN"));
      linkUp = false;
      break;
    default:
      linkUp = false;
      break;
  }
}

// ─── DHCP Maintenance ────────────────────────────────────────────────────────
// Wraps Ethernet.maintain() so the main sketch has no direct Ethernet.h dependency.
// We're using a static IP setup, maintain only does something if we used DHCP, so we can drop for now
void maintainEthernet() {
  if (ethernetInitialized) Ethernet.maintain();
}
