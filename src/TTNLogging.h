/*******************************************************************************
 * 
 * ttn-esp32 - The Things Network device library for ESP-IDF / SX127x
 * 
 * Copyright (c) 2018-2019 Manuel Bleichenbacher
 * 
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 *
 * Circular buffer for detailed logging without affecting LMIC timing.
 *******************************************************************************/

#ifndef _ttnlogging_h_
#define _ttnlogging_h_


#if LMIC_ENABLE_event_logging

#include <freertos/FreeRTOS.h>
#include <freertos/ringbuf.h>


class TTNLogging {
public:
    static void initInstance();

    void init();
    void logEvent(const char* message, uint32_t datum);

private:
    static void loggingTask(void* param);

    RingbufHandle_t ringBuffer;
};

#endif

#endif
