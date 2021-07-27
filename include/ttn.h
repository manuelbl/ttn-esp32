/*******************************************************************************
 * 
 * ttn-esp32 - The Things Network device library for ESP-IDF / SX127x
 * 
 * Copyright (c) 2018-2021 Manuel Bleichenbacher
 * 
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 *
 * High-level C API for ttn-esp32.
 *******************************************************************************/

#ifndef TTN_C_H
#define TTN_C_H

#include <stdint.h>
#include "driver/spi_master.h"


#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Constant for indicating that a pin is not connected
 */
#define TTN_NOT_CONNECTED 0xff


typedef uint8_t ttn_port_t;

/**
 * @brief Response codes
 */
typedef enum
{
    TTN_ERROR_TRANSMISSION_FAILED = -1,
    TTN_ERROR_UNEXPECTED = -10,
    TTN_SUCCESSFUL_TRANSMISSION = 1,
    TTN_SUCCESSFUL_RECEIVE = 2
} ttn_response_code_t;


/**
 * @brief RX/TX window
 */
typedef enum
{
    /**
     * @brief Outside RX/TX window
     */
    TTN_WINDOW_IDLE = 0,
    /**
     * @brief Transmission window (up to RX1 window)
     */
    TTN_WINDOW_TX = 1,
    /**
     * @brief Reception window 1 (up to RX2 window)
     */
    TTN_WINDOW_RX1 = 2,
    /**
     * @brief Reception window 2
     */
    TTN_WINDOW_RX2 = 3
} ttn_rx_tx_window_t;


/**
 * @brief Spreading Factor
 */
typedef enum
{
    /**
     * @brief Unused / undefined spreading factor
     */
    TTN_SF_NONE = 0,
    /**
     * @brief Frequency Shift Keying (FSK)
     */
    TTN_FSK = 1,
    /**
     * @brief Spreading Factor 7 (SF7)
     */
    TTN_SF7 = 2,
    /**
     * @brief Spreading Factor 8 (SF8)
     */
    TTN_SF8 = 3,
    /**
     * @brief Spreading Factor 9 (SF9)
     */
    TTN_SF9 = 4,
    /**
     * @brief Spreading Factor 10 (SF10)
     */
    TTN_SF10 = 5,
    /**
     * @brief Spreading Factor 11 (SF11)
     */
    TTN_SF11 = 6,
    /**
     * @brief Spreading Factor 12 (SF12)
     */
    TTN_SF12 = 7
} ttn_spreading_factor_t;


/**
 * @brief Bandwidth
 */
typedef enum
{
    /**
     * @brief Undefined/unused bandwidth
     */
    TTN_BW_NONE = 0,
    /**
     * @brief Bandwidth of 125 kHz
     */
    TTN_BW125 = 1,
    /**
     * @brief Bandwidth of 250 kHz
     */
    TTN_BW250 = 2,
    /**
     * @brief Bandwidth of 500 kHz
     */
    TTN_BW500 = 3
} ttn_bandwidth_t;


/**
 * @brief RF settings for TX or RX
 */
typedef struct
{
    /**
     * @brief Spreading Factor (SF)
     */
    ttn_spreading_factor_t spreading_factor;
    /**
     * @brief Bandwidth (BW)
     */
    ttn_bandwidth_t bandwidth;
    /**
     * @brief Frequency, in Hz
     */
    uint32_t frequency;
} ttn_rf_settings_t;


/**
 * @brief Callback for recieved messages
 * 
 * @param payload  pointer to the received bytes
 * @param length   number of received bytes
 * @param port     port the message was received on
 */
typedef void (*ttn_message_cb)(const uint8_t* payload, size_t length, ttn_port_t port);


/**
 * @brief Constructs a new The Things Network device instance.
 */
void ttn_init(void);

/**
 * @brief Resets the LoRaWAN radio.
 * 
 * To restart communication, join() must be called.
 * It neither clears the provisioned keys nor the configured pins.
 */
void ttn_reset(void);

/**
 * @brief Configures the pins used to communicate with the LoRaWAN radio chip.
 * 
 * Before calling this member function, the SPI bus needs to be configured using `spi_bus_initialize()`. 
 * Additionally, `gpio_install_isr_service()` must have been called to initialize the GPIO ISR handler service.
 * 
 * @param spi_host  The SPI bus/peripherial to use (`SPI_HOST`, `HSPI_HOST` or `VSPI_HOST`).
 * @param nss       The GPIO pin number connected to the radio chip's NSS pin (serving as the SPI chip select)
 * @param rxtx      The GPIO pin number connected to the radio chip's RXTX pin (`TTN_NOT_CONNECTED` if not connected)
 * @param rst       The GPIO pin number connected to the radio chip's RST pin (`TTN_NOT_CONNECTED` if not connected)
 * @param dio0      The GPIO pin number connected to the radio chip's DIO0 pin
 * @param dio1      The GPIO pin number connected to the radio chip's DIO1 pin
 */
void ttn_configure_pins(spi_host_device_t spi_host, uint8_t nss, uint8_t rxtx, uint8_t rst, uint8_t dio0, uint8_t dio1);

/**
 * @brief Sets the frequency sub-band to be used.
 * 
 * For regions with sub-bands (USA, Australia), sets the sub-band to be used for uplink communication.
 * For other regions, this function has no effect.
 * 
 * The sub-band must be set before joining or sending the first message.
 * 
 * If not set, it defaults to sub-band 2 as defined by TTN.
 * 
 * @param band band (0 for all bands, or value between 1 and 8)
 */
void ttn_set_subband(int band);

/**
 * @brief Sets the credentials needed to activate the device via OTAA, without activating it.
 * 
 * The provided DevEUI, AppEUI/JoinEUI and AppKey are saved in non-volatile memory. Before
 * this function is called, `nvs_flash_init()` must have been called once.
 * 
 * Call join() to activate the device.
 * 
 * @param dev_eui  DevEUI (16 character string with hexadecimal data)
 * @param app_eui  AppEUI/JoinEUI of the device (16 character string with hexadecimal data)
 * @param app_key  AppKey of the device (32 character string with hexadecimal data)
 * @return `true` if the provisioning was successful, `false`  if the provisioning failed
 */
bool ttn_provision(const char *dev_eui, const char *app_eui, const char *app_key);

/**
 * @brief Sets the information needed to activate the device via OTAA, using the MAC to generate the DevEUI
 * and without activating it.
 * 
 * The generated DevEUI and the provided AppEUI/JoinEUI and AppKey are saved in non-volatile memory. Before
 * this function is called, 'nvs_flash_init' must have been called once.
 * 
 * The DevEUI is generated by retrieving the ESP32's WiFi MAC address and expanding it into a DevEUI
 * by adding FFFE in the middle. So the MAC address A0:B1:C2:01:02:03 becomes the EUI A0B1C2FFFE010203.
 * This hexadecimal data can be entered into the DevEUI field in the TTN console.
 * 
 * Generating the DevEUI from the MAC address allows to flash the same AppEUI/JoinEUI and AppKey to a batch of
 * devices. However, using the same AppKey for multiple devices is insecure. Only use this approach if
 * it is okay for that the LoRa communication of your application can easily be intercepted and that
 * forged data can be injected.
 * 
 * Call join() to activate.
 * 
 * @param app_eui  AppEUI/JoinEUI of the device (16 character string with hexadecimal data)
 * @param app_key  AppKey of the device (32 character string with hexadecimal data)
 * @return `true` if the provisioning was successful, `false`  if the provisioning failed
 */
bool ttn_provision_with_mac(const char *app_eui, const char *app_key);

/**
 * @brief Starts task listening on configured UART for AT commands.
 * 
 * Run `make menuconfig` to configure it.
 */
void ttn_start_provisioning_task(void);

/**
 * @brief Waits until the DevEUI, AppEUI/JoinEUI and AppKey have been provisioned
 * by the provisioning task.
 * 
 * If the device has already been provisioned (stored data in NVS, call of provision()
 * or call of join(const char*, const char*, const char*), this function
 * immediately returns.
 */
void ttn_wait_for_provisioning(void);

    /**
 * @brief Activates the device via OTAA.
 * 
 * The DevEUI, AppEUI/JoinEUI and AppKey must have already been provisioned by a call to provision().
 * Before this function is called, `nvs_flash_init()` must have been called once.
 * 
 * The function blocks until the activation has completed or failed.
 * 
 * @return `true` if the activation was succesful, `false` if the activation failed
 */
bool ttn_join_provisioned(void);

/**
 * @brief Sets the DevEUI, AppEUI/JoinEUI and AppKey and activate the device via OTAA.
 * 
 * The DevEUI, AppEUI/JoinEUI and AppKey are NOT saved in non-volatile memory.
 * 
 * The function blocks until the activation has completed or failed.
 * 
 * @param dev_eui  DevEUI (16 character string with hexadecimal data)
 * @param app_eui  AppEUI/JoinEUI of the device (16 character string with hexadecimal data)
 * @param app_key  AppKey of the device (32 character string with hexadecimal data)
 * @return `true` if the activation was succesful, `false` if the activation failed
 */
bool ttn_join(const char *dev_eui, const char *app_eui, const char *app_key);

/**
 * @brief Transmits a message
 * 
 * The function blocks until the message could be transmitted and a message has been received
 * in the subsequent receive window (or the window expires). Additionally, the function will
 * first wait until the duty cycle allows a transmission (enforcing the duty cycle limits).
 * 
 * @param payload  bytes to be transmitted
 * @param length   number of bytes to be transmitted
 * @param port     port (use 1 as default)
 * @param confirm  flag indicating if a confirmation should be requested (use `false` as default)
 * @return `kTTNSuccessfulTransmission` for successful transmission, `kTTNErrorTransmissionFailed` for failed transmission, `kTTNErrorUnexpected` for unexpected error
 */
ttn_response_code_t ttn_transmit_message(const uint8_t *payload, size_t length, ttn_port_t port, bool confirm);

/**
 * @brief Sets the function to be called when a message is received
 * 
 * When a message is received, the specified function is called. The
 * message, its length and the port number are provided as
 * parameters. The values are only valid during the duration of the
 * callback. So they must be immediately processed or copied.
 * 
 * Messages are received as a result of transmitMessage(). The callback is called
 * in the task that called any of these functions and it occurs before these functions
 * return control to the caller.
 * 
 * @param callback  the callback function
 */
void ttn_on_message(ttn_message_cb callback);

/**
 * @brief Checks if DevEUI, AppEUI/JoinEUI and AppKey have been stored in non-volatile storage
 * or have been provided as by a call to join(const char*, const char*, const char*).
 * 
 * @return `true` if they are stored, complete and of the correct size, `false` otherwise
 */
bool ttn_is_provisioned(void);

/**
 * @brief Sets the RSSI calibration value for LBT (Listen Before Talk).
 * 
 * This value is added to RSSI measured prior to decision. It must include the guardband.
 * Ignored in US, EU, IN and other countries where LBT is not required.
 * Defaults to 10 dB.
 * 
 * @param rssi_cal RSSI calibration value, in dB
 */
void ttn_set_rssi_cal(int8_t rssi_cal);

/**
 * Returns whether Adaptive Data Rate (ADR) is enabled.
 * 
 * @return `true` if enabled, `false` if disabled
 */
bool ttn_adr_enabled(void);

/**
 * @brief Enables or disabled Adaptive Data Rate (ADR).
 * 
 * ADR is enabled by default. It optimizes data rate, airtime and energy consumption
 * for devices with stable RF conditions. It should be turned off for mobile devices.
 * 
 * @param enabled `true` to enable, `false` to disable
 */ 
void ttn_set_adr_enabled(bool enabled);

/**
 * @brief Stops all activies and shuts down the RF module and the background tasks.
 * 
 * To restart communication, startup() and join() must be called.
 * it neither clears the provisioned keys nor the configured pins.
 */
void ttn_shutdown(void);

/**
 * @brief Restarts the background tasks and RF module.
 * 
 * This member function must only be called after a call to shutdowna().
 */
void ttn_startup(void);

/**
 * @brief Gets current RX/TX window
 * @return window
 */
ttn_rx_tx_window_t ttn_rx_tx_window(void);

/**
 * @brief Gets the RF settings for the specified window
 * @param window RX/TX windows (valid values are `kTTNTxWindow`, `kTTNRx1Window` and `kTTNRx2Window`)
 */
ttn_rf_settings_t ttn_get_rf_settings(ttn_rx_tx_window_t window);

/**
 * @brief Gets the RF settings of the last (or ongoing) transmission.
 * @return RF settings
 */
ttn_rf_settings_t ttn_tx_settings(void);

/**
 * @brief Gets the RF settings of the last (or ongoing) reception of RX window 1.
 * @return RF settings
 */
ttn_rf_settings_t ttn_rx1_settings(void);

/**
 * @brief Gets the RF settings of the last (or ongoing) reception of RX window 2.
 * @return RF settings
 */
ttn_rf_settings_t ttn_rx2_settings(void);

/**
 * @brief Gets the received signal strength indicator (RSSI).
 * 
 * RSSI is the measured signal strength of the last recevied message (incl. join responses).
 * 
 * @return RSSI, in dBm
 */
int ttn_rssi();


#ifdef __cplusplus
}
#endif


#endif
