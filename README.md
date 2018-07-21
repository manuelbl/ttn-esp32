# ttn-esp32

**The Things Network device library for ESP-IDF (ESP32) supporting devices with Semtech SX127x chips**

This ESP32 component provides LoRaWAN communication with [The Things Network](https://www.thethingsnetwork.org/). It supports OTAA (over-the-air activation), uplink and downlink messages and saving the EUIs and key in non-volatile memory.

The library is based on the LMIC library from IBM and provides a high-level API specifically targeted at The Things Network.

## Get Started

Follow the detailed [Get Started Guide](https://github.com/manuelbl/ttn-esp32/wiki/Get-Started) in the Wiki.

## Supported Boards

All boards with Semtech SX127x chips, RFM9x and compatibles are supported. It includes boards from ttgo, Heltec and HopeRF.


## Resources

### SPI bus and pins

Before the TTN device is configured, the SPI bus / host (`SPI_HOST`, `HSPI_HOST` or `VSPI_HOST`) must be configured by calling `spi_bus_initialize()`. In the SPI configuration, the pin number for SCK, MISO and MOSI are specified.

The pin number of NSS (the radio chip's SPI chip select) is specified by calling `TheThingsNetwork::configurePins()`.

The SPI bus frequency can be changed by running `make menuconfig`.

### SX127x pins

Except for the SPI pins, the pins connected to the SX127x radio chip are configured by calling `TheThingsNetwork::configurePins()`. The pins *RXTX* and *RTS* are optional. The chip can be used without connecting them.

### Timer

To implement the LoRaWAN protocol, a timer is needed. The ESP32 has four timers. By default,
 timer 1 of timer group 0 is used. It can be changed by running `make menuconfig`.

### Background thread

Most of the LoRaWAN code is run in a background process with high priority (as the timing is crucial). The default priority is 10. It can be changed by running `make menuconfig`.
