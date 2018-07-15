/*******************************************************************************
 * Copyright (c) 2018 Manuel Bleichenbacher
 * 
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * Hardware abstraction layer to run LMIC on a ESP32 using ESP-iDF.
 *******************************************************************************/

#ifndef _hal_esp32_h_
#define _hal_esp32_h_

#include <stdint.h>
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct lmic_pinmap {
    spi_host_device_t spi_host;
    uint8_t nss;
    uint8_t rxtx;
    uint8_t rst;
    uint8_t dio0;
    uint8_t dio1;
} lmic_pinmap;

extern lmic_pinmap lmic_pins;

void hal_startBgTask();
void hal_wakeUp();
void hal_initCriticalSection();
void hal_enterCriticalSection();
void hal_leaveCriticalSection();


#ifdef __cplusplus
}
#endif

#endif // _hal_esp32_h_