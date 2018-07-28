# ttn-esp32

**The Things Network device library for ESP-IDF (ESP32) supporting devices with Semtech SX127x chips**

This ESP32 component provides LoRaWAN communication with [The Things Network](https://www.thethingsnetwork.org/). It supports

- OTAA (over-the-air activation)
- uplink and downlink messages
- saving the EUIs and key in non-volatile memory
- AT commands for provisioning EUIs and key (so the same code can be flashed to several devices)

The library is based on the LMIC library from IBM and provides a high-level API specifically targeted at The Things Network.

## Get Started

Follow the detailed [Get Started Guide](https://github.com/manuelbl/ttn-esp32/wiki/Get-Started) in the Wiki.

## Supported Boards

All boards with Semtech SX127x chips, RFM9x and compatibles are supported. It includes boards from ttgo, Heltec and HopeRF.

## API Documentation

See the Wiki page: [API Documentation](https://github.com/manuelbl/ttn-esp32/wiki/API-Documentation)

## More information

More information can be found on the [Wiki pages](https://github.com/manuelbl/ttn-esp32/wiki).
