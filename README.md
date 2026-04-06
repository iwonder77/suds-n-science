# Suds n Science

## Overview

This is the main firmware for the the **Suds n Science** interactive exhibit at Thanksgiving Point's Museum of Natural Curiosity. In this exhibit, guests place three molecule pucks (each containing an embedded UHF tag) into a beaker; a SparkFun M7E Hecto reader identifies the combination and triggers different video outcomes on a HD6 BrightSign through a direct Ethernet connection and UDP.

When pucks are lost, damaged, or need replacement, use [this]() sketch to write the correct molecule identification data onto fresh UHF tags. The sketch provides a serial monitor menu for selecting a molecule type, writing the EPC payload, and verifying the result.

---

## Hardware

### Components

| Component   | Part                                                                                                                                                                                                                                                                                                                | Notes                                                             |
| ----------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------- |
| MCU         | ESP32-S3-ETH                                                                                                                                                                                                                                                                                                        | Waveshare's [module](https://www.waveshare.com/wiki/ESP32-S3-ETH) |
| RFID Reader | [SparkFun M7E Hecto](https://www.sparkfun.com/sparkfun-simultaneous-rfid-reader-m7e-hecto.html?gad_source=1&gad_campaignid=21251727806&gbraid=0AAAAADsj4ETK53PAkDBaA3DwlrPRrLj-8&gclid=Cj0KCQjws83OBhD4ARIsACblj1_ITF5wKjlS58gAkDFubuO9uDybveUE-tBHFFYV4kj0Lr911EZYlVAaAtGoEALw_wcB) (Simultaneous UHF RFID Reader) | Must be the M7E variant, not M6E Nano                             |
| RFID Tags   | UHF Gen2 sticker tags, 96-bit EPC                                                                                                                                                                                                                                                                                   | Standard EPC Gen2 / ISO 18000-6C                                  |

### Wiring

The M7E Hecto communicates with the ESP32 over **UART** (Serial1):

| ESP32-S3 Pin | M7E Hecto Pin | Signal                                             |
| ------------ | ------------- | -------------------------------------------------- |
| GPIO 18      | TXO           | ESP32 RX <- M7E TX                                 |
| GPIO 17      | RXI           | ESP32 TX -> M7E RX                                 |
| 5V           | 5V            | Power (M7E needs 5V, draws up to ~300mA during TX) |
| GND          | GND           | Common ground                                      |

> **Power note:** The M7E Hecto can draw significant current during RF transmission. Only power it from the ESP32's 5V pin for prototyping and set the read power accordingly, also ensure your USB supply can handle the load (1A+ recommended). For reliable operation, a dedicated 5V supply (2A+) to the M7E is preferred, with shared ground to the ESP32.

### Pin Configuration

UART pins are defined in `src/Config.h` and can be changed if your board layout differs:

```cpp
constexpr uint8_t RXD1 = 18;  // ESP32 RX <- M7E TXO
constexpr uint8_t TXD1 = 17;  // ESP32 TX -> M7E RXI
```

---

## Software Architecture

### File Structure

```
suds-n-science/
  suds-n-science.ino   # Main sketch
  src/
    Config.h                                 # Pin assignments, power levels, molecule EPC struct
    EthernetController.h                     # Lightweight class to handle ethernet functionality (header)
    EthernetController.cpp                   # Lightweight class to handle ethernet functionality (implementation)
    SparkFun_UHF_RFID_Reader.h               # Modified SparkFun library (header)
    SparkFun_UHF_RFID_Reader.cpp             # Modified SparkFun library (implementation)
```

### Gen2 Session Parameters

Deeper explanation in `src/README.md`, but these parameters are configured in `configureGen2Parameters()`:

| Parameter | Value           | Why                                                                                           |
| --------- | --------------- | --------------------------------------------------------------------------------------------- |
| Session   | S1              | Tag flags persist ~0.5-5s, naturally suppressing re-reads within a scan window                |
| Target    | AB              | Inventories A-flagged tags, then B-flagged, then cycles — ensures all tags eventually respond |
| Q         | Dynamic, init=3 | Reader auto-adjusts slot count; 8 initial slots suits 2-4 tag populations                     |
| RF Mode   | 250/M4/20       | High link reliability for indoor museum environments                                          |

### Modified SparkFun Library

The `src/` directory contains a customized version of [paulvha's fork](https://github.com/paulvha/ThingMagic) of the SparkFun Simultaneous RFID Reader library. Key additions relevant to this project:

- `setGen2Session()`, `setGen2Target()`, `setGen2Q()`, `setGen2RFmode()` — Gen2 parameter configuration methods (done by paulvha)
- `setPowerMode()` — M7E sleep mode support (also done by paulvha)
- Temperature statistics in continuous mode (done by paulvha)
- Optimized `stopReading()` drain logic by replacing per-byte `delay(100)` with bulk drain + settle (done by me)

---

## References

- [SparkFun M7E Hecto Hookup Guide](https://docs.sparkfun.com/SparkFun_Simultaneous_RFID_Reader_M7E/introduction/)
- [ThingMagic Mercury API](https://www.jadaktech.com/product/thingmagic-mercury-api/) for a deeper dive into the source code
- [ThingMagic M7E Hecto Datasheet](https://cdn.sparkfun.com/assets/f/2/4/0/a/M7E-HECTO-Spec-Sheet_06142024.pdf)
- [paulvha ThingMagic Library Fork](https://github.com/paulvha/ThingMagic) — the zip file in the `Arduino_lib_special` folder contains the basis for the modified library in `src/`
- [EPC Gen2 (ISO 18000-6C) Overview](https://www.gs1.org/standards/epc-rfid/uhf-air-interface-protocol)
