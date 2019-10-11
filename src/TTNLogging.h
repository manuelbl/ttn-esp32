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


/**
 * @brief Logging class.
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
 * 
 * This class is not to be used directly.
 */
class TTNLogging {
public:
    static TTNLogging* initInstance();

    void init();
    void logEvent(int event, const char* message, uint32_t datum);

private:
    static void loggingTask(void* param);
    static void logFatal(const char* file, uint16_t line);

    RingbufHandle_t ringBuffer;
};

#endif

#endif
