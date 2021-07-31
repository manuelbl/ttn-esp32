# ttn-esp32

**The Things Network device library for ESP-IDF (ESP32) supporting devices with Semtech SX127x chips**

This ESP32 component provides LoRaWAN communication with
[The Things Network](https://www.thethingsnetwork.org/). It supports

- OTAA (over-the-air activation)
- uplink and downlink messages
- saving the EUIs and key in non-volatile memory
- [AT commands](https://github.com/manuelbl/ttn-esp32/wiki/AT-Commands) for provisioning EUIs and key
(so the same code can be flashed to several devices)
- support for regions Europe, North and South America, Australia, Korea, Asia and India
- C and C++ API

The library is based on the LMIC library from IBM (specifically the well-maintained version by MCCI
– see their [GitHub repository](https://github.com/mcci-catena/arduino-lmic)) and provides a high-level API specifically targeted at The Things Network.

## New in version 4.0 (prerelease)

- Verified compatibility with ESP-IDF v4.3
- Upgraded underlying library mcci-catena/arduino-lmic to v4.0.1 (improved channel shuffling)
- C API
- Support for sub-bands


## Get Started

Follow the detailed [Get Started Guide](https://github.com/manuelbl/ttn-esp32/wiki/Get-Started) in the Wiki.

## Supported Boards

All boards with Semtech SX127x chips, RFM9x and compatibles are supported. It includes boards from ttgo, Heltec and HopeRF. For several of them, the [Pin Configuration](https://github.com/manuelbl/ttn-esp32/wiki/Boards-and-Pins) is described in detail.

## API Documentation

See [API Documentation](https://codecrete.net/ttn-esp32/) for both the C ad C++ API

## More information

More information can be found on the [Wiki pages](https://github.com/manuelbl/ttn-esp32/wiki).
