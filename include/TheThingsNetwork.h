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
enum TTNResponseCode
{
  kTTNErrorTransmissionFailed = -1,
  kTTNErrorUnexpected = -10,
  kTTNSuccessfulTransmission = 1,
  kTTNSuccessfulReceive = 2
};

/**
 * @brief Callback for recieved messages
 * 
 * @param payload  pointer to the received bytes
 * @param length   number of received bytes
 * @param port     port the message was received on
 */
typedef void (*TTNMessageCallback)(const uint8_t* payload, size_t length, port_t port);

/**
 * @brief TTN device
 * 
 * The 'TheThingsNetwork' class enables ESP32 devices with SX1272/73/76/77/78/79 LoRaWAN chips
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
     * 
     * The SPI bus must be first configured using spi_bus_initialize(). Then it is passed as the first parameter.
     * Additionally, 'gpio_install_isr_service()' must be called to initialize the GPIO ISR handler service.
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
     * The provided EUIs and key are saved in non-volatile memory. Call join() without arguments to activate.
     * 
     * @param devEui  Device EUI (16 character string with hexadecimal data)
     * @param appEui  Application EUI of the device (16 character string with hexadecimal data)
     * @param appKey  App Key of the device (32 character string with hexadecimal data)
     * @return true   if the provisioning was successful
     * @return false  if the provisioning failed
     */
    bool provision(const char *devEui, const char *appEui, const char *appKey);

    /**
     * @brief Set the EUIs and keys and activate the device via OTAA.
     * 
     * The EUIs and key are NOT saved in non-volatile memory.
     * 
     * The function blocks until the activation has completed or failed.
     * 
     * @param devEui  Device EUI (16 character string with hexadecimal data)
     * @param appEui  Application EUI of the device (16 character string with hexadecimal data)
     * @param appKey  App Key of the device (32 character string with hexadecimal data)
     * @return true   if the activation was succeful
     * @return false  if the activation failed
     */
    bool join(const char *devEui, const char *appEui, const char *appKey);

    /**
     * @brief Activate the device via OTAA.
     * 
     * The app EUI, app key and dev EUI must already have been provisioned by a call to 'provision()'.
     * 
     * The function blocks until the activation has completed or failed.
     * 
     * @return true   if the activation was succeful
     * @return false  if the activation failed
     */
    bool join();

    /**
     * @brief Transmit a message
     * 
     * The function blocks until the message could be transmitted and a message has been received
     * in the subsequent receive window (or the window expires). Additionally, the function will
     * first wait until the duty cycle allows a transmission (enforcing the duty cycle limits).
     * 
     * @param payload  bytes to be transmitted
     * @param length   number of bytes to be transmitted
     * @param port     port (default to 1)
     * @param confirm  flag indicating if a confirmation should be requested. Default to 'false'
     * @return TkTTNSuccessfulTransmission   Successful transmission
     * @return kTTNErrorTransmissionFailed   Transmission failed
     * @return TkTTNErrorUnexpected          Unexpected error
     */
    TTNResponseCode transmitBytes(const uint8_t *payload, size_t length, port_t port = 1, bool confirm = false);

    /**
     * @brief Set the function to be called when a message is received
     * 
     * When a message is received, the specified function is called. The
     * message, its length and the port number are provided as
     * parameters. The values are only valid during the duration of the
     * callback. So they must be immediately processed or copied.
     * 
     * Messages are received as a result of 'transmitBytes' or 'poll'. The callback is called
     * in the task that called any of these functions and it occurs before these functions
     * return control to the caller.
     * 
     * @param callback  the callback function
     */
    void onMessage(TTNMessageCallback callback);


private:
    TTNMessageCallback messageCallback;

    bool decodeKeys(const char *devEui, const char *appEui, const char *appKey);
};

#endif
