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
 * @brief Callback for recieved messages
 * 
 * @param payload  pointer to the received bytes
 * @param length   number of received bytes
 * @param port     port the message was received on
 */
typedef void (*message_cb_t)(const uint8_t* payload, size_t length, port_t port);

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
     */
    TheThingsNetwork();

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
     * @param devEui  Device EUI (16 character string with hexadecimal data)
     * @param appEui  Application EUI of the device (16 character string with hexadecimal data)
     * @param appKey  App Key of the device (32 character string with hexadecimal data)
     * @return true   if the provisioning was successful
     * @return false  if the provisioning failed
     */
    bool provision(const char *devEui, const char *appEui, const char *appKey);

    /**
     * @brief Activate the device via OTAA.
     * 
     * @param devEui  Device EUI (16 character string with hexadecimal data)
     * @param appEui  Application EUI of the device (16 character string with hexadecimal data)
     * @param appKey  App Key of the device (32 character string with hexadecimal data)
     * @return true 
     * @return false 
     */
    bool join(const char *devEui, const char *appEui, const char *appKey);

    /**
     * @brief Activate the device via OTAA.
     * 
     * The app EUI, app key and dev EUI must already have been provisioned.
     * 
     * @return true 
     * @return false 
     */
    bool join();

    /**
     * @brief Send a message
     * 
     * @param payload  bytes to be sent
     * @param length   number of bytes to be sent
     * @param port     port (default to 1)
     * @param confirm  flag indicating if a confirmation should be requested. Default to 'false'
     * @return TTTN_SUCCESSFUL_TRANSMISSION    Successful transmission
     * @return TTTN_SUCCESSFUL_RECEIVE         Successful transmission and a message has been received
     * @return TTN_ERROR_SEND_COMMAND_FAILED   Transmission failed
     * @return TTTN_ERROR_UNEXPECTED_RESPONSE  Unexpected response
     */
    ttn_response_t sendBytes(const uint8_t *payload, size_t length, port_t port = 1, bool confirm = false);

    /**
     * @brief Set the function to be called when a message is received
     * 
     * When a message is received, the specified function is called. The
     * message, its length and the port is was received on are provided as
     * parameters. The values are only valid during the duration of the
     * callback. So they must be immediately processed or copied.
     * 
     * @param callback  the callback function
     */
    void onMessage(message_cb_t callback);


private:
    message_cb_t messageCallback;

    bool decodeKeys(const char *devEui, const char *appEui, const char *appKey);
};

#endif