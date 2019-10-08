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


#if LMIC_ENABLE_event_logging

#include <string.h>
#include <esp_log.h>
#include "lmic/lmic.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "TTNLogging.h"


#define NO_DATUM 0x7cabcde3

static const char* const TAG = "lmic";
static TTNLogging ttnLog;

struct TTNLogMessage {
    const char* message;
    uint32_t datum;
};


void TTNLogging::initInstance()
{
    ttnLog.init();
}

void TTNLogging::init()
{
    ringBuffer = xRingbufferCreate(50 * sizeof(TTNLogMessage), RINGBUF_TYPE_NOSPLIT);
    if (ringBuffer == NULL) {
        ESP_LOGE(TAG, "Failed to create ring buffer");
        ASSERT(0);
    }

    xTaskCreate(loggingTask, "ttn_log", 1024 * 4, ringBuffer, 4, NULL);
}

void TTNLogging::logEvent(const char* message, uint32_t datum)
{
    if (ringBuffer == NULL)
        return;

    TTNLogMessage log;
    
    log.message = message;
    log.datum = datum;

    xRingbufferSend(ringBuffer, &log, sizeof(log), 0);
}


void TTNLogging::loggingTask(void* param)
{
    RingbufHandle_t ringBuffer = (RingbufHandle_t)param;

    while (true) {
        size_t size;
        TTNLogMessage* log = (TTNLogMessage*) xRingbufferReceive(ringBuffer, &size, portMAX_DELAY);
        if (log == NULL)
            continue;

        if (log->datum == NO_DATUM)
            ESP_LOGI(TAG, "%s", log->message);
        else
            ESP_LOGI(TAG, "%s (0x%x)", log->message, log->datum);

        vRingbufferReturnItem(ringBuffer, log);
    }
}


extern "C" void LMICOS_logEvent(const char *pMessage)
{
    ttnLog.logEvent(pMessage, NO_DATUM);

}

extern "C" void LMICOS_logEventUint32(const char *pMessage, uint32_t datum)
{
    ttnLog.logEvent(pMessage, datum);
}

#endif
