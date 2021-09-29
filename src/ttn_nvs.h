/*******************************************************************************
 *
 * ttn-esp32 - The Things Network device library for ESP-IDF / SX127x
 *
 * Copyright (c) 2018-2021 Manuel Bleichenbacher
 *
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 *
 * Functions for storing and retrieving TTN communication state from NVS.
 *******************************************************************************/

#ifndef TTN_NVS_H
#define TTN_NVS_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    void ttn_nvs_save();
    bool ttn_nvs_restore(int off_duration);

#ifdef __cplusplus
}
#endif

#endif
