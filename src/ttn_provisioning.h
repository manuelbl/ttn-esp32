/*******************************************************************************
 *
 * ttn-esp32 - The Things Network device library for ESP-IDF / SX127x
 *
 * Copyright (c) 2018-2021 Manuel Bleichenbacher
 *
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 *
 * Task listening on a UART port for provisioning commands.
 *******************************************************************************/

#ifndef TTN_PROVISIONING_H
#define TTN_PROVISIONING_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    void ttn_provisioning_init(void);

    bool ttn_provisioning_have_keys(void);
    bool ttn_provisioning_decode_keys(const char *dev_eui, const char *app_eui, const char *app_key);
    bool ttn_provisioning_from_mac(const char *app_eui, const char *app_key);
    bool ttn_provisioning_save_keys(void);
    bool ttn_provisioning_restore_keys(bool silent);

#if defined(TTN_HAS_AT_COMMANDS)
    void ttn_provisioning_start_task(void);
#endif

#ifdef __cplusplus
}
#endif

#endif
