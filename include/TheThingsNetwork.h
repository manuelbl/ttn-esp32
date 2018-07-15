/*******************************************************************************
 * Copyright (c) 2018 Manuel Bleichenbacher
 * 
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * This the hardware abstraction layer to run LMIC in on ESP32 using ESP-iDF.
 *******************************************************************************/

#ifndef _THETHINGSNETWORK_H_
#define _THETHINGSNETWORK_H_

#include <stdint.h>
#include "driver/spi_master.h"

#define TTN_DEFAULT_SF 7
#define TTN_DEFAULT_FSB 2

/**
 * @brief Constant for indicating that a pin is not connected
 */
#define TTN_NOT_CONNECTED 0xff


typedef uint8_t port_t;

/**
 * @brief Response codes
 */
enum ttn_response_t
{
  TTN_ERROR_SEND_COMMAND_FAILED = (-1),
  TTN_ERROR_UNEXPECTED_RESPONSE = (-10),
  TTN_SUCCESSFUL_TRANSMISSION = 1,
  TTN_SUCCESSFUL_RECEIVE = 2
};


/**
 * @brief TTN device
 * 
 * The TheThingsNetwork class enables ESP32 devices with SX1272/73/76/77/78/79 LoRaWAN chips
 * to communicate via The Things Network.
 * 
 * Only one instance of this class must be created.
 */
class TheThingsNetwork
{
public:
    /**
     * @brief Construct a new The Things Network device
     * 
     * @param sf        The spreading factor. 7 to 10 for US915 frequency plan. 7 to 12 for other frequency plans. Defaults to 7.
     * @param fsb       Optional custom frequency subband. 1 to 8. Defaults to 2.
     */
    TheThingsNetwork(uint8_t sf = TTN_DEFAULT_SF, uint8_t fsb = TTN_DEFAULT_FSB);

    /**
     * @brief Destroy the The Things Network device.
     */
    ~TheThingsNetwork();

    /**
     * @brief Reset the LoRaWAN radio.
     * 
     * Does not clear provisioned keys.
     */
    void reset();

    /**
     * @brief Configures the pins used to communicate with the LoRaWAN radio chip.
     * 
     * The SPI bus must be first configured using spi_bus_initialize(). Then it is passed as the first parameter.
     * 
     * @param spi_host  The SPI bus/peripherial to use (SPI_HOST, HSPI_HOST or VSPI_HOST).
     * @param nss       The GPIO pin number connected to the radio chip's NSS pin (serving as the SPI chip select)
     * @param rxtx      The GPIO pin number connected to the radio chip's RXTX pin (TTN_NOT_CONNECTED if not connected)
     * @param rst       The GPIO pin number connected to the radio chip's RST pin (TTN_NOT_CONNECTED if not connected)
     * @param dio0      The GPIO pin number connected to the radio chip's DIO0 pin
     * @param dio1      The GPIO pin number connected to the radio chip's DIO1 pin
     */
    void configurePins(spi_host_device_t spi_host, uint8_t nss, uint8_t rxtx, uint8_t rst, uint8_t dio0, uint8_t dio1);

    /**
     * @brief Sets the information needed to activate the device via OTAA, without actually activating.
     * 
     * Call join() without the first 2 arguments to activate.
     * 
     * @param appEui  Application EUI of the device (16 character string with hexadecimal data)
     * @param appKey  App Key of the device (32 character string with hexadecimal data)
     * @param devEui  Device EUI (16 character string with hexadecimal data) or NULL if already set
     * @return true   if the provisioning was successful
     * @return false  if the provisioning failed
     */
    bool provision(const char *appEui, const char *appKey, const char *devEui = NULL);

    /**
     * @brief Activate the device via OTAA.
     * 
     * @param appEui      Application EUI of the device (16 character string with hexadecimal data)
     * @param appKey      App Key of the device (32 character string with hexadecimal data)
     * @param devEui  Device EUI (16 character string with hexadecimal data) or NULL if already set
     * @param retries     Number of times to retry after failed or unconfirmed join. Defaults to -1 which means infinite.
     * @param retryDelay  Delay in ms between attempts. Defaults to 10 seconds.
     * @return true 
     * @return false 
     */
    bool join(const char *appEui, const char *appKey, const char *devEui = NULL, int8_t retries = -1, uint32_t retryDelay = 10000);

    /**
     * @brief Activate the device via OTAA.
     * 
     * The app EUI and key must already have been provisioned.
     * 
     * @param retries     Number of times to retry after failed or unconfirmed join. Defaults to -1 which means infinite.
     * @param retryDelay  Delay in ms between attempts. Defaults to 10 seconds.
     * @return true 
     * @return false 
     */
    bool join(int8_t retries = -1, uint32_t retryDelay = 10000);

    ttn_response_t sendBytes(const uint8_t *payload, size_t length, port_t port = 1, bool confirm = false, uint8_t sf = 0);

private:
    uint8_t spreadingFactor = TTN_DEFAULT_SF;
    uint8_t frequencySubband = TTN_DEFAULT_FSB;

    bool decodeKeys(const char *appEui, const char *appKey, const char *devEui);
};

#endif
