/*******************************************************************************
 * 
 * ttn-esp32 - The Things Network device library for ESP-IDF / SX127x
 * 
 * Copyright (c) 2018-2021 Manuel Bleichenbacher
 * 
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 *
 * Hardware abstraction layer to run LMIC on a ESP32 using ESP-IDF.
 *******************************************************************************/

#ifndef HAL_ESP32_H
#define HAL_ESP32_H

#include "freertos/FreeRTOS.h"
#include "driver/spi_master.h"


#ifdef __cplusplus
extern "C" {
#endif


void hal_esp32_configure_pins(spi_host_device_t spi_host, uint8_t nss, uint8_t rxtx, uint8_t rst, uint8_t dio0, uint8_t dio1);
void hal_esp32_start_lmic_task(void);
void hal_esp32_stop_lmic_task(void);

void hal_esp32_wake_up(void);
void hal_esp32_init_critical_section(void);
void hal_esp32_enter_critical_section(void);
void hal_esp32_leave_critical_section(void);

void hal_esp32_set_rssi_cal(int8_t rssi_cal);


#ifdef __cplusplus
}
#endif


#endif // HAL_ESP32_H
