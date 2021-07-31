/*******************************************************************************
 *
 * ttn-esp32 - The Things Network device library for ESP-IDF / SX127x
 *
 * Copyright (c) 2018 Manuel Bleichenbacher
 *
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 *
 * Convert SDK configuration (make menuconfig) into LMIC configuration.
 *******************************************************************************/

#include "sdkconfig.h"

#if defined(CONFIG_TTN_LORA_FREQ_EU_868)
#define CFG_eu868 1
#elif defined(CONFIG_TTN_LORA_FREQ_US_915)
#define CFG_us915 1
#elif defined(CONFIG_TTN_LORA_FREQ_AU_921)
#warning                                                                                                               \
    "CONFIG_TTN_LORA_FREQ_AU_921 was deprecated in favour of CONFIG_TTN_LORA_FREQ_AU_921. Support for CONFIG_TTN_LORA_FREQ_AU_921 will be removed in the future."
#define CFG_au915 1
#elif defined(CONFIG_TTN_LORA_FREQ_AU_915)
#define CFG_au915 1
#elif defined(CONFIG_TTN_LORA_FREQ_AS_923)
#define CFG_as923 1
#elif defined(CONFIG_TTN_LORA_FREQ_AS_923_JP)
#define CFG_as923 1
#define LMIC_COUNTRY_CODE LMIC_COUNTRY_CODE_JP
#elif defined(CONFIG_TTN_LORA_FREQ_KR_920)
#define CFG_kr920 1
#elif defined(CONFIG_TTN_LORA_FREQ_IN_866)
#define CFG_in866 1
#else
#define TTN_IS_DISABLED 1
#define CFG_eu868 1
#endif

#if defined(CONFIG_TTN_RADIO_SX1272_73)
#define CFG_sx1272_radio 1
#elif defined(CONFIG_TTN_RADIO_SX1276_77_78_79)
#define CFG_sx1276_radio 1
#else
#error TTN LoRa radio chip must be configured using 'make menuconfig'
#endif

#if !defined(CONFIG_TTN_PROVISION_UART_NONE)
#define TTN_HAS_AT_COMMANDS 1
#if defined(CONFIG_TTN_PROVISION_UART_CONFIG_YES)
#define TTN_CONFIG_UART 1
#endif
#endif

// 16 μs per tick
// LMIC requires ticks to be 15.5μs - 100 μs long
#define US_PER_OSTICK 16
#define OSTICKS_PER_SEC (1000000 / US_PER_OSTICK)

//#define USE_ORIGINAL_AES
#define USE_MBEDTLS_AES

#if LMIC_DEBUG_LEVEL > 0 || LMIC_X_DEBUG_LEVEL > 0
#include <stdio.h>
#endif

#define LMIC_ENABLE_onEvent 0

#define DISABLE_PING

#define DISABLE_BEACONS
