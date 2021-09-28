/*******************************************************************************
 *
 * ttn-esp32 - The Things Network device library for ESP-IDF / SX127x
 *
 * Copyright (c) 2018-2021 Manuel Bleichenbacher
 *
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 *
 * Functions for storing and retrieving TTN communication state from RTC memory.
 *******************************************************************************/

#ifndef TTN_RTC_H
#define TTN_RTC_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    void ttn_rtc_save();
    bool ttn_rtc_restore();

#ifdef __cplusplus
}
#endif

#endif
