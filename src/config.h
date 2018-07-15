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

#ifndef _lmic_config_h_
#define _lmic_config_h_

#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(CONFIG_TTN_LORA_FREQ_EU_868)
#define CFG_eu868 1
#elif defined(CONFIG_TTN_LORA_FREQ_US_915)
#define CFG_us915 1
#else
#error TTN LoRa frequency must be configured
#endif

#if defined(CONFIG_TTN_RADIO_SX1272_73)
#define CFG_sx1272_radio 1
#elif defined(CONFIG_TTN_RADIO_SX1276_77_78_79)
#define CFG_sx1276_radio 1
#else
#error TTN LoRa radio chip must be configured
#endif

// 16 μs per tick
// LMIC requires ticks to be 15.5μs - 100 μs long
#define US_PER_OSTICK 16
#define OSTICKS_PER_SEC (1000000 / US_PER_OSTICK)

#ifdef __cplusplus
}
#endif

#endif // _lmic_config_h_
