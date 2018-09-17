/*******************************************************************************
 * 
 * ttn-esp32 - The Things Network device library for ESP-IDF / SX127x
 * 
 * Copyright (c) 2018 Manuel Bleichenbacher
 * 
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 *
 * Task listening on a UART port for provisioning commands.
 *******************************************************************************/

#ifndef _provision_task_h_
#define _provision_task_h_

#include "oslmic.h"


#ifdef __cplusplus
extern "C" {
#endif


void provisioning_start_task();
bool provisioning_have_keys();
bool provisioning_decode_keys(const char *dev_eui, const char *app_eui, const char *app_key);
bool provisioning_from_mac(const char *app_eui, const char *app_key);
bool provisioning_save_keys();
bool provisioning_restore_keys(bool silent);


#ifdef __cplusplus
}
#endif

#endif
