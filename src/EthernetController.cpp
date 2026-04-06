#include "EthernetController.h"
#include "Config.h"

void EthernetController::init() {

  Serial.println(F("\nInitializing Ethernet (W5500)..."));

  SPI.begin(config::W5500_SCK, config::W5500_MISO, config::W5500_MOSI,
            config::W5500_CS);

  // Hardware reset: hold RST low briefly to guarantee a clean chip state
  // independent of prior sketch or power-cycle conditions.
  pinMode(config::W5500_RST, OUTPUT);
  digitalWrite(config::W5500_RST, LOW);
  delay(100);
  digitalWrite(config::W5500_RST, HIGH);
  delay(500); // W5500 datasheet requires ~150ms post-reset stabilization

  Ethernet.init(config::W5500_CS);

  // Ethernet.begin() requires a non-const MAC buffer; copy from Config.h
  uint8_t mac[6];
  memcpy(mac, config::MAC_ADDR, sizeof(mac));
  Ethernet.begin(mac, IPAddress(config::LOCAL_IP));

  switch (Ethernet.hardwareStatus()) {
  case EthernetW5500:
    Serial.println(F("  W5500 detected."));
    ethernetInitialized_ = true;
    break;
  case EthernetNoHardware:
    Serial.println(
        F("  ERROR: W5500 not found. Check SPI wiring. UDP disabled."));
    return;
  default:
    Serial.println(F("  ERROR: Unexpected hardware detected. UDP disabled."));
    return;
  }

  // Non-blocking link wait — log and continue if the cable is not yet
  // connected. linkUp will be set to true by updateLinkStatus() once the cable
  // is plugged in.
  Serial.print(F("  Waiting for link"));
  unsigned long t = millis();
  while (Ethernet.linkStatus() != LinkON) {
    if (millis() - t > config::LINK_TIMEOUT_MS) {
      Serial.println(
          F("\n  TIMEOUT: no link. Continuing — link may come up later."));
      break;
    }
    delay(500);
    Serial.print('.');
  }

  if (Ethernet.linkStatus() == LinkON) {
    Serial.println(F("\n  Link UP."));
    linkUp_ = true;
  }

  udp_.begin(config::UDP_PORT);
  Serial.print(F("  UDP bound to port "));
  Serial.println(config::UDP_PORT);
}

// ─── Link Monitor ─────────────────────────────────────────────────────
// Tracks ethernet link state transitions and logs them. Called every loop()
// iteration; uses a static local to avoid redundant prints on stable state.
void EthernetController::updateLinkStatus() {
  static EthernetLinkStatus lastStatus = Unknown;

  EthernetLinkStatus current = Ethernet.linkStatus();
  if (current == lastStatus)
    return;
  lastStatus = current;

  switch (current) {
  case LinkON:
    Serial.println(F("[ETH] Link UP"));
    linkUp_ = true;
    break;
  case LinkOFF:
    Serial.println(F("[ETH] Link DOWN"));
    linkUp_ = false;
    break;
  default:
    linkUp_ = false;
    break;
  }
}

void EthernetController::sendUdpCommand(const char *command) {
  if (!ethernetInitialized_ || !linkUp_) {
    Serial.println(F("  UDP skipped — ethernet not ready."));
    return;
  }

  udp_.beginPacket(IPAddress(config::TARGET_IP), config::UDP_PORT);
  udp_.write(reinterpret_cast<const uint8_t *>(command), strlen(command));
  udp_.endPacket();

  Serial.print(F("  UDP sent: \""));
  Serial.print(command);
  Serial.println('"');
}
