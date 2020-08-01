/*******************************************************************************
 * 
 * ttn-esp32 - The Things Network device library for ESP-IDF / SX127x
 * 
 * Copyright (c) 2018 Manuel Bleichenbacher
 * 
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 *
 * High-level API for ttn-esp32.
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
 * @brief RX/TX window
 */
enum TTNRxTxWindow
{
    /**
     * @brief Outside RX/TX window
     */
    kTTNIdleWindow = 0,
    /**
     * @brief Transmission window (up to RX1 window)
     */
    kTTNTxWindow = 1,
    /**
     * @brief Reception window 1 (up to RX2 window)
     */
    kTTNRx1Window = 2,
    /**
     * @brief Reception window 2
     */
    kTTNRx2Window = 3
};


/**
 * @brief Spreading Factor
 */
enum TTNSpreadingFactor
{
    /**
     * @brief Unused / undefined spreading factor
     */
    kTTNSFNone = 0,
    /**
     * @brief Frequency Shift Keying (FSK)
     */
    kTTNFSK = 1,
    /**
     * @brief Spreading Factor 7 (SF7)
     */
    kTTNSF7 = 2,
    /**
     * @brief Spreading Factor 8 (SF8)
     */
    kTTNSF8 = 3,
    /**
     * @brief Spreading Factor 9 (SF9)
     */
    kTTNSF9 = 4,
    /**
     * @brief Spreading Factor 10 (SF10)
     */
    kTTNSF10 = 5,
    /**
     * @brief Spreading Factor 11 (SF11)
     */
    kTTNSF11 = 6,
    /**
     * @brief Spreading Factor 12 (SF12)
     */
    kTTNSF12 = 7
};


/**
 * @brief Bandwidth
 */
enum TTNBandwidth
{
    /**
     * @brief Undefined/unused bandwidth
     */
    kTTNBWNone = 0,
    /**
     * @brief Bandwidth of 125 kHz
     */
    kTTNBW125 = 1,
    /**
     * @brief Bandwidth of 250 kHz
     */
    kTTNBW250 = 2,
    /**
     * @brief Bandwidth of 500 kHz
     */
    kTTNBW500 = 3
};


/**
 * @brief RF settings for TX or RX
 */
struct TTNRFSettings
{
    /**
     * @brief Spreading Factor (SF)
     */
    TTNSpreadingFactor spreadingFactor;
    /**
     * @brief Bandwidth (BW)
     */
    TTNBandwidth bandwidth;
    /**
     * @brief Frequency, in Hz
     */
    uint32_t frequency;
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
     * @brief Construct a new The Things Network device instance.
     */
    TheThingsNetwork();

    /**
     * @brief Destroy the The Things Network device instance.
     */
    ~TheThingsNetwork();

    /**
     * @brief Reset the LoRaWAN radio.
     * 
     * To restart communication, `join()` must be called.
     * Neither clears the provisioned keys nor the configured pins.
     */
    void reset();

    /**
     * @brief Configures the pins used to communicate with the LoRaWAN radio chip.
     * 
     * 
     * Before calling this member function, the SPI bus needs to be configured using `spi_bus_initialize()`. 
     * Additionally, `gpio_install_isr_service()` must have been called to initialize the GPIO ISR handler service.
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
     * The provided device EUI, app EUI and app key are saved in non-volatile memory. Before
     * this function is called, 'nvs_flash_init' must have been called once.
     * 
     * Call join() without arguments to activate.
     * 
     * @param devEui  Device EUI (16 character string with hexadecimal data)
     * @param appEui  Application EUI of the device (16 character string with hexadecimal data)
     * @param appKey  App Key of the device (32 character string with hexadecimal data)
     * @return true   if the provisioning was successful
     * @return false  if the provisioning failed
     */
    bool provision(const char *devEui, const char *appEui, const char *appKey);

    /**
     * @brief Sets the information needed to activate the device via OTAA, using the MAC to generate the device EUI
     * and without actually activating.
     * 
     * The generated device EUI and the provided app EUI and app key are saved in non-volatile memory. Before
     * this function is called, 'nvs_flash_init' must have been called once.
     * 
     * The device EUI is generated by retrieving the ESP32's WiFi MAC address and expanding it into a device EUI
     * by adding FFFE in the middle. So the MAC address A0:B1:C2:01:02:03 becomes the EUI A0B1C2FFFE010203.
     * This hexadecimal data can be entered into the Device EUI field in the TTN console.
     * 
     * Generating the device EUI from the MAC address allows to flash the same app EUI and app key to a batch of
     * devices. However, using the same app key for multiple devices is insecure. Only use this approach if
     * it is okay for that the LoRa communication of your application can easily be intercepted and that
     * forged data can be injected.
     * 
     * Call join() without arguments to activate.
     * 
     * @param appEui  Application EUI of the device (16 character string with hexadecimal data)
     * @param appKey  App Key of the device (32 character string with hexadecimal data)
     * @return true   if the provisioning was successful
     * @return false  if the provisioning failed
     */
    bool provisionWithMAC(const char *appEui, const char *appKey);

    /**
     * @brief Start task that listens on configured UART for AT commands.
     * 
     * Run 'make menuconfig' to configure it.
     */
    void startProvisioningTask();

    /**
     * @brief Wait until the device EUI, app EUI and app key have been provisioned
     * via the provisioning task.
     * 
     * If device is already provisioned (stored data in NVS, call to 'provision()'
     * or call to 'join(const char*, const char*, const char*)', this function
     * immediately returns.
     */
    void waitForProvisioning();

     /**
     * @brief Activate the device via OTAA.
     * 
     * The app EUI, app key and dev EUI must already have been provisioned by a call to 'provision()'.
     * Before this function is called, 'nvs_flash_init' must have been called once.
     * 
     * The function blocks until the activation has completed or failed.
     * 
     * @return true   if the activation was succeful
     * @return false  if the activation failed
     */
    bool join();

   /**
     * @brief Set the device EUI, app EUI and app key and activate the device via OTAA.
     * 
     * The device EUI, app EUI and app key are NOT saved in non-volatile memory.
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
    TTNResponseCode transmitMessage(const uint8_t *payload, size_t length, port_t port = 1, bool confirm = false);

    /**
     * @brief Set the function to be called when a message is received
     * 
     * When a message is received, the specified function is called. The
     * message, its length and the port number are provided as
     * parameters. The values are only valid during the duration of the
     * callback. So they must be immediately processed or copied.
     * 
     * Messages are received as a result of 'transmitMessage'. The callback is called
     * in the task that called any of these functions and it occurs before these functions
     * return control to the caller.
     * 
     * @param callback  the callback function
     */
    void onMessage(TTNMessageCallback callback);

    /**
     * @brief Checks if device EUI, app EUI and app key have been stored in non-volatile storage
     * or have been provided as by a call to 'join(const char*, const char*, const char*)'.
     * 
     * @return true   if they are stored, complete and of the correct size
     * @return false  otherwise
     */
    bool isProvisioned();

    /**
     * @brief Sets the RSSI calibration value for LBT (Listen Before Talk).
     * 
     * This value is added to RSSI measured prior to decision. It must include the guardband.
     * Ignored in US, EU, IN and other countries where LBT is not required.
     * Defaults to 10 dB.
     * 
     * @param rssiCal RSSI calibration value, in dB
     */
    void setRSSICal(int8_t rssiCal);

    /**
     * Returns whether Adaptive Data Rate (ADR) is enabled.
     * 
     * @return true  if enabled
     * @return false if disabled
     */
    bool adrEnabled();

    /**
     * @brief Enables or disabled Adaptive Data Rate (ADR).
     * 
     * ADR is enabled by default. It optimizes data rate, airtime and energy consumption
     * for devices with stable RF conditions. It should be turned off for mobile devices.
     * 
     * @param enabled `true` to enable, `false` to disable
     */ 
    void setAdrEnabled(bool enabled);

    /**
     * @brief Stops all activies and shuts down the RF module and the background tasks.
     * 
     * To restart communication, `startup()` and `join()` must be called.
     * Neither clears the provisioned keys nor the configured pins.
     */
    void shutdown();

    /**
     * @brief Restarts the background tasks and RF module.
     * 
     * This member function must only be called after a call to `shutdowna()`.
     */
    void startup();

    /**
     * @brief Gets currentRX/TX window
     * @return window
     */
    TTNRxTxWindow rxTxWindow();

    /**
     * @brief Gets the RF settings for the specified window
     * @param window RX/TX windows (valid values are `kTTNTxWindow`, `kTTNRx1Window` and `kTTNRx2Window`)
     */
    TTNRFSettings getRFSettings(TTNRxTxWindow window);

    /**
     * @brief Gets the RF settings of the last (or ongoing) transmission.
     * @return RF settings
     */
    TTNRFSettings txSettings();

    /**
     * @brief Gets the RF settings of the last (or ongoing) reception of RX window 1.
     * @return RF settings
     */
    TTNRFSettings rx1Settings();

    /**
     * @brief Gets the RF settings of the last (or ongoing) reception of RX window 2.
     * @return RF settings
     */
    TTNRFSettings rx2Settings();

    /**
     * @brief Gets the received signal strength indicator (RSSI).
     * 
     * RSSI is the measured signal strength of the last recevied message (incl. join responses).
     * 
     * @return RSSI, in dBm
     */
    int rssi();

private:
    TTNMessageCallback messageCallback;

    bool joinCore();
};

#endif
