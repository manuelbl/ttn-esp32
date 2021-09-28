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

#include "ttn_rtc.h"
#include "esp_system.h"
#include "lmic/lmic.h"
#include <string.h>

#define LMIC_OFFSET(field) __builtin_offsetof(struct lmic_t, field)
#define LMIC_DIST(field1, field2) (LMIC_OFFSET(field2) - LMIC_OFFSET(field1))
#define TTN_RTC_MEM_SIZE (sizeof(struct lmic_t) - LMIC_OFFSET(radio) - MAX_LEN_PAYLOAD - MAX_LEN_FRAME)

#define TTN_RTC_FLAG_VALUE 0xf30b84ce

RTC_DATA_ATTR uint8_t ttn_rtc_mem_buf[TTN_RTC_MEM_SIZE];
RTC_DATA_ATTR uint32_t ttn_rtc_flag;

void ttn_rtc_save()
{
    // Copy LMIC struct except client, osjob, pendTxData and frame
    size_t len1 = LMIC_DIST(radio, pendTxData);
    memcpy(ttn_rtc_mem_buf, &LMIC.radio, len1);
    size_t len2 = LMIC_DIST(pendTxData, frame) - MAX_LEN_PAYLOAD;
    memcpy(ttn_rtc_mem_buf + len1, (u1_t *)&LMIC.pendTxData + MAX_LEN_PAYLOAD, len2);
    size_t len3 = sizeof(struct lmic_t) - LMIC_OFFSET(frame) - MAX_LEN_FRAME;
    memcpy(ttn_rtc_mem_buf + len1 + len2, (u1_t *)&LMIC.frame + MAX_LEN_FRAME, len3);

    ttn_rtc_flag = TTN_RTC_FLAG_VALUE;
}

bool ttn_rtc_restore()
{
    if (ttn_rtc_flag != TTN_RTC_FLAG_VALUE)
        return false;

    // Restore data
    size_t len1 = LMIC_DIST(radio, pendTxData);
    memcpy(&LMIC.radio, ttn_rtc_mem_buf, len1);
    memset(LMIC.pendTxData, 0, MAX_LEN_PAYLOAD);
    size_t len2 = LMIC_DIST(pendTxData, frame) - MAX_LEN_PAYLOAD;
    memcpy((u1_t *)&LMIC.pendTxData + MAX_LEN_PAYLOAD, ttn_rtc_mem_buf + len1, len2);
    memset(LMIC.frame, 0, MAX_LEN_FRAME);
    size_t len3 = sizeof(struct lmic_t) - LMIC_OFFSET(frame) - MAX_LEN_FRAME;
    memcpy((u1_t *)&LMIC.frame + MAX_LEN_FRAME, ttn_rtc_mem_buf + len1 + len2, len3);

    ttn_rtc_flag = 0xffffffff; // invalidate RTC data

    return true;
}
