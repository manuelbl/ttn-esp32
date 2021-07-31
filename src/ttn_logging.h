/*******************************************************************************
 *
 * ttn-esp32 - The Things Network device library for ESP-IDF / SX127x
 *
 * Copyright (c) 2018-2021 Manuel Bleichenbacher
 *
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 *
 * Circular buffer for detailed logging without affecting LMIC timing.
 *******************************************************************************/

#ifndef TTN_LOGGING_H
#define TTN_LOGGING_H

#if LMIC_ENABLE_event_logging

#include <freertos/FreeRTOS.h>
#include <freertos/ringbuf.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Logging functions.
     *
     * Logs internal information from LMIC in an asynchrnous fashion in order
     * not to distrub the sensitive LORA timing.
     *
     * A ring buffer and a separate logging task is ued. The LMIC core records
     * relevant values from the current LORA settings and writes them to a ring
     * buffer. The logging tasks receives the message and the values, formats
     * them and outputs them via the regular ESP-IDF logging mechanism.
     *
     * In order to activate the detailed logging, set the macro
     * `LMIC_ENABLE_event_logging` to 1.
     */

    void ttn_log_init(void);
    void ttn_log_event(int event, const char *message, uint32_t datum);

#ifdef __cplusplus
}
#endif

#endif

#endif
