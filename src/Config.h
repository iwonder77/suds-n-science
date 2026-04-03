#pragma once
/**
 * Config.h
 *
 * Centralized hardware and network constants for Suds and Science.
 * - Include with #include "Config.h", access via config::CONSTANT_NAME
 * - Units are encoded in names (_MS, _US, _DBM, etc.)
 */

#include <Arduino.h>

namespace config {

// ===== RFID UART PINS =====
constexpr uint32_t RFID_BAUD = 115200;
constexpr uint8_t  RXD1      = 18;  // ESP32-S3 RX ← M7E TXO
constexpr uint8_t  TXD1      = 17;  // ESP32-S3 TX → M7E RXI

// ===== LED / BUTTON PINS =====
constexpr uint8_t PIN_LED_YELLOW   = 1;
constexpr uint8_t PIN_LED_BLUE     = 2;
constexpr uint8_t PIN_LED_GREEN    = 3;
constexpr uint8_t PIN_LED_RED      = 15;
constexpr uint8_t BUTTON_PIN       = 48;
constexpr uint32_t BUTTON_DEBOUNCE_MS = 250;

// ===== RFID SCAN PARAMETERS =====
constexpr uint8_t  MAX_TAGS       = 10;   // max unique tags stored per scan
constexpr uint8_t  EPC_MAX_BYTES  = 12;   // 96-bit standard EPC
constexpr uint32_t SCAN_WINDOW_MS = 500;  // collection window after button press (ms)
constexpr uint32_t READ_POWER     = 1500; // 15.00 dBm (units: centidBm)
constexpr uint32_t LED_DISPLAY_MS = 3000; // how long LEDs stay lit after a scan (ms)

// ===== KNOWN PUCK EPCs =====
// TODO: replace hardcoded EPCs with user data written directly onto each tag
constexpr uint8_t NUM_PUCKS  = 4;
constexpr uint8_t EPC_LENGTH = 12;
static const uint8_t PUCK_EPC_YELLOW[EPC_LENGTH] = {
    0xE2, 0x80, 0x68, 0x94, 0x00, 0x00, 0x40, 0x33, 0x56, 0x39, 0x29, 0x0A };
static const uint8_t PUCK_EPC_BLUE[EPC_LENGTH] = {
    0xE2, 0x80, 0x68, 0x94, 0x00, 0x00, 0x40, 0x33, 0x56, 0x38, 0xED, 0x0A };
static const uint8_t PUCK_EPC_GREEN[EPC_LENGTH] = {
    0xE2, 0x80, 0x68, 0x94, 0x00, 0x00, 0x50, 0x33, 0x56, 0x39, 0x2D, 0x0A };
static const uint8_t PUCK_EPC_RED[EPC_LENGTH] = {
    0xE2, 0x80, 0x68, 0x94, 0x00, 0x00, 0x50, 0x33, 0x56, 0x38, 0xF1, 0x0A };

// ===== PUCK BITMASKS =====
// Each bit corresponds to one puck. Used to compose combination masks in COMBOS[]
// and stored directly in PuckConfig::bit so bitmask derivation is explicit, not
// fragile array-position arithmetic.
constexpr uint8_t PUCK_BIT_YELLOW = (1 << 0);
constexpr uint8_t PUCK_BIT_BLUE   = (1 << 1);
constexpr uint8_t PUCK_BIT_GREEN  = (1 << 2);
constexpr uint8_t PUCK_BIT_RED    = (1 << 3);

// ===== W5500 ETHERNET MODULE PINS =====
// ESP32-S3-ETH onboard W5500 SPI mapping
// (ref: https://www.waveshare.com/wiki/ESP32-S3-ETH)
constexpr uint8_t W5500_CS   = 14;
constexpr uint8_t W5500_RST  = 9;
constexpr uint8_t W5500_INT  = 10;  // optional interrupt line, not polled
constexpr uint8_t W5500_MISO = 12;
constexpr uint8_t W5500_MOSI = 11;
constexpr uint8_t W5500_SCK  = 13;

// ===== NETWORK CONFIG =====
static const uint8_t MAC_ADDR[6]  = { 0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01 };
static const uint8_t LOCAL_IP[4]  = { 192, 168, 50,  2  };  // ESP32 static IP
static const uint8_t TARGET_IP[4] = { 192, 168, 50,  10 };  // BrightSign target IP
constexpr uint16_t   UDP_PORT        = 5000;
constexpr uint32_t   LINK_TIMEOUT_MS = 10000;  // how long to wait for link on boot (ms)

} // namespace config
