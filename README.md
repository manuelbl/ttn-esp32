# ttn-esp32

**The Things Network device library for ESP-IDF (ESP32)**

This ESP32 component provides LoRaWAN communication with [The Things Network](https://www.thethingsnetwork.org/). It supports OTAA (over-the-air activation), uplink and downlink messages.

## Installation

Install the component by adding it as a git submodule:

    git submodule add https://github.com/manuelbl/ttn-esp32.git components/ttn-esp32
    git submodule update --init --recursive

The frequency plan must be configured by running `make menuconfig`. Otherwise the component will not compile. In the menu it can be found at *Components* / *The Things Network*.

More to follow...

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
