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

/**
 * @defgroup c_api C API
 */

#ifndef TTN_C_H
#define TTN_C_H

#include "driver/spi_master.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @addtogroup c_api
 *
 * @{
 */

/**
 * @brief Constant for indicating that a pin is not connected
 */
#define TTN_NOT_CONNECTED 0xff

    /**
     * @brief Integer data type for specifiying the port of an uplink or downlink message.
     */
    typedef uint8_t ttn_port_t;

    /**
     * @brief Response codes
     */
    typedef enum
    {
        /** @brief Transmission failed error */
        TTN_ERROR_TRANSMISSION_FAILED = -1,
        /** @brief Unexpected or internal error */
        TTN_ERROR_UNEXPECTED = -10,
        /** @brief Successful transmission of an uplink message */
        TTN_SUCCESSFUL_TRANSMISSION = 1,
        /** @brief Successful receipt of a downlink message */
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
        TTN_BW_125 = 1,
        /**
         * @brief Bandwidth of 250 kHz
         */
        TTN_BW_250 = 2,
        /**
         * @brief Bandwidth of 500 kHz
         */
        TTN_BW_500 = 3
    } ttn_bandwidth_t;

    /**
     * @brief Data Rate
     *
     * Note that the spreading factor, bandwidth, bit rate and maximum message
     * size associated with each data rate depends on the region.
     */
    typedef enum
    {
        /**
         * @brief Data rate for region AS923 using SF12 and 125 kHz bandwidth.
         */
        TTN_DR_AS923_SF12 = 0,
        /**
         * @brief Data rate for region AS923 using SF11 and 125 kHz bandwidth.
         */
        TTN_DR_AS923_SF11 = 1,
        /**
         * @brief Data rate for region AS923 using SF10 and 125 kHz bandwidth.
         */
        TTN_DR_AS923_SF10 = 2,
        /**
         * @brief Data rate for region AS923 using SF9 and 125 kHz bandwidth.
         */
        TTN_DR_AS923_SF9 = 3,
        /**
         * @brief Data rate for region AS923 using SF8 and 125 kHz bandwidth.
         */
        TTN_DR_AS923_SF8 = 4,
        /**
         * @brief Data rate for region AS923 using SF7 and 125 kHz bandwidth.
         */
        TTN_DR_AS923_SF7_BW125 = 5,
        /**
         * @brief Data rate for region AS923 using SF7 and 250 kHz bandwidth.
         */
        TTN_DR_AS923_SF7_BW250 = 6,
        /**
         * @brief Data rate for region AS923 using FSK and 50 kpbs.
         */
        TTN_DR_AS923_FSK = 7,

        /**
         * @brief Data rate for region AU915 using SF12 and 125 kHz bandwidth.
         */
        TTN_DR_AU915_SF12 = 0,
        /**
         * @brief Data rate for region AU915 using SF11 and 125 kHz bandwidth.
         */
        TTN_DR_AU915_SF11 = 1,
        /**
         * @brief Data rate for region AU915 using SF10 and 125 kHz bandwidth.
         */
        TTN_DR_AU915_SF10 = 2,
        /**
         * @brief Data rate for region AU915 using SF9 and 125 kHz bandwidth.
         */
        TTN_DR_AU915_SF9 = 3,
        /**
         * @brief Data rate for region AU915 using SF8 and 125 kHz bandwidth.
         */
        TTN_DR_AU915_SF8 = 4,
        /**
         * @brief Data rate for region AU915 using SF7 and 125 kHz bandwidth.
         */
        TTN_DR_AU915_SF7 = 5,
        /**
         * @brief Data rate for region AU915 using SF8 and 500 kHz bandwidth.
         */
        TTN_DR_AU915_SF8_BW500 = 6,
        /**
         * @brief Data rate for region AU915 using SF12 and 500 kHz bandwidth.
         *
         * Reserved for future applications.
         */
        TTN_DR_AU915_SF12_BW500 = 8,
        /**
         * @brief Data rate for region AU915 using SF11 and 500 kHz bandwidth.
         *
         * Reserved for future applications.
         */
        TTN_DR_AU915_SF11_BW500 = 9,
        /**
         * @brief Data rate for region AU915 using SF10 and 500 kHz bandwidth.
         *
         * Reserved for future applications.
         */
        TTN_DR_AU915_SF10_BW500 = 10,
        /**
         * @brief Data rate for region AU915 using SF9 and 500 kHz bandwidth.
         *
         * Reserved for future applications.
         */
        TTN_DR_AU915_SF9_BW500 = 11,
        /**
         * @brief Data rate for region AU915 using SF8 and 500 kHz bandwidth.
         *
         * Reserved for future applications.
         */
        TTN_DR_AU915_SF8_BW500_DR12 = 12,
        /**
         * @brief Data rate for region AU915 using SF7 and 500 kHz bandwidth.
         *
         * Reserved for future applications.
         */
        TTN_DR_AU915_SF7_BW500 = 13,

        /**
         * @brief Data rate for region EU868 using SF12 and 125 kHz bandwidth.
         */
        TTN_DR_EU868_SF12 = 0,
        /**
         * @brief Data rate for region EU868 using SF11 and 125 kHz bandwidth.
         */
        TTN_DR_EU868_SF11 = 1,
        /**
         * @brief Data rate for region EU868 using SF10 and 125 kHz bandwidth.
         */
        TTN_DR_EU868_SF10 = 2,
        /**
         * @brief Data rate for region EU868 using SF9 and 125 kHz bandwidth.
         */
        TTN_DR_EU868_SF9 = 3,
        /**
         * @brief Data rate for region EU868 using SF8 and 125 kHz bandwidth.
         */
        TTN_DR_EU868_SF8 = 4,
        /**
         * @brief Data rate for region EU868 using SF7 and 125 kHz bandwidth.
         */
        TTN_DR_EU868_SF7_BW125 = 5,
        /**
         * @brief Data rate for region EU868 using SF7 and 250 kHz bandwidth.
         */
        TTN_DR_EU868_SF7_BW250 = 6,
        /**
         * @brief Data rate for region EU868 using FSK and 50 kpbs.
         */
        TTN_DR_EU868_FSK = 7,

        /**
         * @brief Data rate for region IN866 using SF12 and 125 kHz bandwidth.
         */
        TTN_DR_IN866_SF12 = 0,
        /**
         * @brief Data rate for region IN866 using SF11 and 125 kHz bandwidth.
         */
        TTN_DR_IN866_SF11 = 1,
        /**
         * @brief Data rate for region IN866 using SF10 and 125 kHz bandwidth.
         */
        TTN_DR_IN866_SF10 = 2,
        /**
         * @brief Data rate for region IN866 using SF9 and 125 kHz bandwidth.
         */
        TTN_DR_IN866_SF9 = 3,
        /**
         * @brief Data rate for region IN866 using SF8 and 125 kHz bandwidth.
         */
        TTN_DR_IN866_SF8 = 4,
        /**
         * @brief Data rate for region IN866 using SF7 and 125 kHz bandwidth.
         */
        TTN_DR_IN866_SF7 = 5,
        /**
         * @brief Data rate for region IN866 using FSK and 50 kpbs.
         */
        TTN_DR_IN866_FSK = 7,

        /**
         * @brief Data rate for region KR920 using SF12 and 125 kHz bandwidth.
         */
        TTN_DR_KR920_SF12 = 0,
        /**
         * @brief Data rate for region KR920 using SF11 and 125 kHz bandwidth.
         */
        TTN_DR_KR920_SF11 = 1,
        /**
         * @brief Data rate for region KR920 using SF10 and 125 kHz bandwidth.
         */
        TTN_DR_KR920_SF10 = 2,
        /**
         * @brief Data rate for region KR920 using SF9 and 125 kHz bandwidth.
         */
        TTN_DR_KR920_SF9 = 3,
        /**
         * @brief Data rate for region KR920 using SF8 and 125 kHz bandwidth.
         */
        TTN_DR_KR920_SF8 = 4,
        /**
         * @brief Data rate for region KR920 using SF7 and 125 kHz bandwidth.
         */
        TTN_DR_KR920_SF7 = 5,

        /**
         * @brief Data rate for region US915 using SF10 and 125 kHz bandwidth.
         */
        TTN_DR_US915_SF10 = 0,
        /**
         * @brief Data rate for region US915 using SF9 and 125 kHz bandwidth.
         */
        TTN_DR_US915_SF9 = 1,
        /**
         * @brief Data rate for region US915 using SF8 and 125 kHz bandwidth.
         */
        TTN_DR_US915_SF8 = 2,
        /**
         * @brief Data rate for region US915 using SF7 and 125 kHz bandwidth.
         */
        TTN_DR_US915_SF7 = 3,
        /**
         * @brief Data rate for region US915 using SF8 and 500 kHz bandwidth.
         */
        TTN_DR_US915_SF8_BW500 = 4,
        /**
         * @brief Data rate for region US915 using SF12 and 500 kHz bandwidth.
         *
         * Reserved for future applications.
         */
        TTN_DR_US915_SF12_BW500 = 8,
        /**
         * @brief Data rate for region US915 using SF11 and 500 kHz bandwidth.
         *
         * Reserved for future applications.
         */
        TTN_DR_US915_SF11_BW500 = 9,
        /**
         * @brief Data rate for region US915 using SF10 and 500 kHz bandwidth.
         *
         * Reserved for future applications.
         */
        TTN_DR_US915_SF10_BW500 = 10,
        /**
         * @brief Data rate for region US915 using SF9 and 500 kHz bandwidth.
         *
         * Reserved for future applications.
         */
        TTN_DR_US915_SF9_BW500 = 11,
        /**
         * @brief Data rate for region US915 using SF8 and 500 kHz bandwidth.
         *
         * Reserved for future applications.
         */
        TTN_DR_US915_SF8_BW500_DR12 = 12,
        /**
         * @brief Data rate for region US915 using SF7 and 500 kHz bandwidth.
         *
         * Reserved for future applications.
         */
        TTN_DR_US915_SF7_BW500 = 13,

        /**
         * @brief Default data rate for joining.
         */
        TTN_DR_JOIN_DEFAULT = 255
    } ttn_data_rate_t;

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
    typedef void (*ttn_message_cb)(const uint8_t *payload, size_t length, ttn_port_t port);

    /**
     * @brief Initializes The Things Network device instance.
     *
     * Call this function once at the start of the program and before all other TTN functions.
     */
    void ttn_init(void);

    /**
     * @brief Configures the pins used to communicate with the LoRaWAN radio chip.
     *
     * Before calling this member function, the SPI bus needs to be configured using `spi_bus_initialize()`.
     * Additionally, `gpio_install_isr_service()` must have been called to initialize the GPIO ISR handler service.
     * 
     * Call this function after @ref ttn_init() and before all other TTN functions.
     *
     * @param spi_host  The SPI bus/peripherial to use (`SPI1_HOST`, `SPI2_HOST`,  `SPI3_HOST`, `FSPI_HOST`, `HSPI_HOST`, or `VSPI_HOST`).
     * @param nss       The GPIO pin number connected to the radio chip's NSS pin (serving as the SPI chip select)
     * @param rxtx      The GPIO pin number connected to the radio chip's RXTX pin (@ref TTN_NOT_CONNECTED if not
     * connected)
     * @param rst       The GPIO pin number connected to the radio chip's RST pin (@ref TTN_NOT_CONNECTED if not
     * connected)
     * @param dio0      The GPIO pin number connected to the radio chip's DIO0 pin
     * @param dio1      The GPIO pin number connected to the radio chip's DIO1 pin
     */
    void ttn_configure_pins(spi_host_device_t spi_host, uint8_t nss, uint8_t rxtx, uint8_t rst, uint8_t dio0,
                            uint8_t dio1);

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
     * @brief Sets the keys needed to activate the device via OTAA, without activating it.
     *
     * The provided DevEUI, AppEUI/JoinEUI and AppKey are saved in non-volatile memory. Before
     * this function is called, `nvs_flash_init()` must have been called once.
     * 
     * In order to reduce flash wear, this function detects if the keys have not changed
     * and will not write them again.
     *
     * Call @ref ttn_join() to activate the device.
     *
     * @param dev_eui  DevEUI (16 character string with hexadecimal data)
     * @param app_eui  AppEUI/JoinEUI of the device (16 character string with hexadecimal data)
     * @param app_key  AppKey of the device (32 character string with hexadecimal data)
     * @return `true` if the provisioning was successful, `false`  if the provisioning failed
     */
    bool ttn_provision(const char *dev_eui, const char *app_eui, const char *app_key);

    /**
     * @brief Sets the keys needed to activate the device via OTAA, without activating it.
     *
     * The provided DevEUI, AppEUI/JoinEUI and AppKey are only stored in RAM and will be lost
     * when the device is powered off or put in deep sleep.
     *
     * Call @ref ttn_join() to activate the device.
     *
     * @param dev_eui  DevEUI (16 character string with hexadecimal data)
     * @param app_eui  AppEUI/JoinEUI of the device (16 character string with hexadecimal data)
     * @param app_key  AppKey of the device (32 character string with hexadecimal data)
     * @return `true` if the provisioning was successful, `false`  if the provisioning failed
     */
    bool ttn_provision_transiently(const char *dev_eui, const char *app_eui, const char *app_key);

    /**
     * @brief Sets the information needed to activate the device via OTAA, using the MAC to generate the DevEUI
     * and without activating it.
     *
     * The generated DevEUI and the provided AppEUI/JoinEUI and AppKey are saved in non-volatile memory. Before
     * this function is called, `nvs_flash_init` must have been called once.
     *
     * In order to reduce flash wear, this function detects if the keys have not changed
     * and will not write them again.
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
     * Call @ref ttn_join() to activate.
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
     * or call of @ref ttn_join_with_keys(), this function immediately returns.
     */
    void ttn_wait_for_provisioning(void);

    /**
     * @brief Activates the device via OTAA using previously provisioned keys.
     *
     * The DevEUI, AppEUI/JoinEUI and AppKey must have already been provisioned by a call
     * to @ref ttn_provision() or @ref ttn_provision_with_mac().
     * Before this function is called, `nvs_flash_init()` must have been called once.
     *
     * The RF module is initialized and the TTN background task is started.
     *
     * The function blocks until the activation has completed or failed.
     *
     * @return `true` if the activation was succesful, `false` if the activation failed
     */
    bool ttn_join(void);

    /**
     * @brief Activates the device via OTAA using the provided keys.
     * 
     * For the activation, the provided DevEUI, AppEUI/JoinEUI and AppKey are used.
     * They are NOT saved in non-volatile memory.
     *
     * The RF module is initialized and the TTN background task is started.
     *
     * The function blocks until the activation has completed or failed.
     *
     * @param dev_eui  DevEUI (16 character string with hexadecimal data)
     * @param app_eui  AppEUI/JoinEUI of the device (16 character string with hexadecimal data)
     * @param app_key  AppKey of the device (32 character string with hexadecimal data)
     * @return `true` if the activation was succesful, `false` if the activation failed
     */
    bool ttn_join_with_keys(const char *dev_eui, const char *app_eui, const char *app_key);

    /**
     * @brief Resumes TTN communication after deep sleep.
     * 
     * The communcation state is restored from data previously saved in RTC memory.
     * The RF module and the TTN background task are started.
     * 
     * This function is called instead of @ref ttn_join_with_keys() or @ref ttn_join()
     * to continue with the established communication and to avoid a further join procedure.
     *
     * @return `true` if the device was able to resume, `false` otherwise.
     */
    bool ttn_resume_after_deep_sleep(void);

    /**
     * @brief Resumes TTN communication after power off.
     * 
     * The communcation state is restored from data previously saved in NVS (non-volatile storage).
     * The RF module and the TTN background task are started.
     * 
     * This function is called instead of @ref ttn_join_with_keys() or @ref ttn_join()
     * to continue with the established communication and to avoid a further join procedure.
     * 
     * In order to advance the clock, the estimated duration the device was powered off has to
     * be specified. As the exact duration is probably not known, an estimation of the shortest
     * duration between power-off and next power-on can be used instead.
     * 
     * If the device has access to the real time, set the system time (using `settimeofday()`)
     * before calling this function (and before @ref ttn_join()) and pass 0 for `off_duration`.
     * 
     * Before this function is called, `nvs_flash_init()` must have been called once.
     *
     * @param off_duration duration the device was powered off (in minutes)
     * @return `true` if the device was able to resume, `false` otherwise.
     */
    bool ttn_resume_after_power_off(int off_duration);

    /**
     * @brief Stops all activies and prepares for deep sleep.
     * 
     * This function is called before entering deep sleep. It saves the current
     * communication state in RTC memory and shuts down the RF module and the
     * TTN background task.
     * 
     * It neither clears the provisioned keys nor the configured pins
     * but they will be lost if the device goes into deep sleep.
     * 
     * Before calling this function, use @ref ttn_busy_duration() to check
     * that the TTN device is idle and ready to go to deep sleep.
     * 
     * To restart communication, @ref ttn_resume_after_deep_sleep() must be called.
     */
    void ttn_prepare_for_deep_sleep(void);

    /**
     * @brief Stops all activies and prepares for power off.
     * 
     * This function is called before powering off the device. It saves the current
     * communication state in NVS (non-volatile storage) and shuts down the RF module
     * and the TTN background task.
     * 
     * It neither clears the provisioned keys nor the configured pins
     * but they will be lost if the device is powered off.
     * 
     * Before calling this function, use @ref ttn_busy_duration() to check
     * that the TTN device is idle and ready to be powered off.
     *
     * To restart communication, @ref ttn_resume_after_power_off(int) must be called.
     * 
     * Before this function is called, `nvs_flash_init()` must have been called once.
     */
    void ttn_prepare_for_power_off(void);

    /**
     * @brief Waits until the TTN device is idle.
     * 
     * If the TTN device is idle, the ESP32 can go into deep sleep mode
     * or be powered off without disrupting an on-going communication.
     */
    void ttn_wait_for_idle(void);

    /**
     * @brief Returns the minimum duration the TTN device will be busy.
     * 
     * This function can be called to check whether the TTN device is
     * still involved in communication or ready to go to deep sleep or
     * to be powered off.
     * 
     * If it returns 0, the TTN communication is idle and the device can go
     * to deep sleep or can be powered off.
     * 
     * If it returns a value different from 0, the value indicates the duration
     * the device will be certainly busy. After that time, this function must be
     * called again. It might still return a value different from 0.
     * 
     * @return busy duration (in FreeRTOS ticks)
     */
    TickType_t ttn_busy_duration(void);

    /**
     * @brief Stops all activies.
     * 
     * This function shuts down the RF module and the TTN background task. It neither clears the
     * provisioned keys nor the configured pins. The currentat device state (and activation)
     * are lost.
     *
     * To restart communication, @ref ttn_join_with_keys() and @ref ttn_join() must be called.
     */
    void ttn_shutdown(void);

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
     * @return @ref TTN_SUCCESSFUL_TRANSMISSION for successful transmission, @ref TTN_ERROR_TRANSMISSION_FAILED for
     * failed transmission, @ref TTN_ERROR_UNEXPECTED for unexpected error
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
     * Messages are received as a result of @ref ttn_transmit_message(). The callback is called
     * in the task that called this function and it occurs before this function
     * returns control to the caller.
     *
     * @param callback  the callback function
     */
    void ttn_on_message(ttn_message_cb callback);

    /**
     * @brief Checks if DevEUI, AppEUI/JoinEUI and AppKey have been stored in non-volatile storage
     * or have been provided by a call to @ref ttn_join_with_keys() or to @ref ttn_provision_transiently().
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
     * @brief Sets the transmission data rate (i.e. the data rate for uplink messages).
     *
     * If ADR is enabled, it's is used as the initial data rate and later adjusted depending
     * on the RF conditions. If ADR is disabled, it is used for all uplink messages.
     *
     * @param data_rate data rate (use constants of enum @ref ttn_data_rate_t)
     */
    void ttn_set_data_rate(ttn_data_rate_t data_rate);

    /**
     * @brief Sets the maximum power for transmission
     *
     * The power is specified in dBm and sets the power emitted by the radio.
     * If the antenna has a gain, it must be substracted from the specified value to
     * achieve the correct transmission power.
     *
     * @param tx_pow power, in dBm
     */
    void ttn_set_max_tx_pow(int tx_pow);

    /**
     * @brief Gets current RX/TX window
     * @return window
     */
    ttn_rx_tx_window_t ttn_rx_tx_window(void);

    /**
     * @brief Gets the RF settings for the specified window
     * @param window RX/TX window (valid values are @ref TTN_WINDOW_TX, @ref TTN_WINDOW_RX1 and @ref TTN_WINDOW_RX2
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

    /**
     * @}
     */

#ifdef __cplusplus
}
#endif

#endif
