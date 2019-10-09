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


static const char* const TAG = "lmic";
static TTNLogging ttnLog;

struct TTNLogMessage {
    const char* message;
    uint32_t    datum;
    ev_t        event;
    ostime_t    time;
    ostime_t    txend;
    ostime_t    globalDutyAvail;
    u4_t        freq;
    u2_t        opmode;
    u2_t        fcntDn;
    u2_t        fcntUp;
    u2_t        rxsyms;
    rps_t       rps;
    u1_t        txChnl;
    u1_t        datarate;
    u1_t        txrxFlags;
    u1_t        saveIrqFlags;
};


void TTNLogging::initInstance()
{
    ttnLog.init();
}

void TTNLogging::init()
{
    ringBuffer = xRingbufferCreate(50 * sizeof(TTNLogMessage), RINGBUF_TYPE_NOSPLIT);
    if (ringBuffer == nullptr) {
        ESP_LOGE(TAG, "Failed to create ring buffer");
        ASSERT(0);
    }

    xTaskCreate(loggingTask, "ttn_log", 1024 * 4, ringBuffer, 4, nullptr);
    hal_set_failure_handler(logFatal);
}

void TTNLogging::logEvent(int event, const char* message, uint32_t datum)
{
    if (ringBuffer == nullptr)
        return;

    TTNLogMessage log;
    log.message = message;
    log.datum = datum;
    
    // capture state
    log.time = os_getTime();
    log.txend = LMIC.txend;
    log.globalDutyAvail = LMIC.globalDutyAvail;
    log.event = (ev_t)event;
    log.freq = LMIC.freq;
    log.opmode = LMIC.opmode;
    log.fcntDn = (u2_t) LMIC.seqnoDn;
    log.fcntUp = (u2_t) LMIC.seqnoUp;
    log.rxsyms = LMIC.rxsyms;
    log.rps = LMIC.rps;
    log.txChnl = LMIC.txChnl;
    log.datarate = LMIC.datarate;
    log.txrxFlags = LMIC.txrxFlags;
    log.saveIrqFlags = LMIC.saveIrqFlags;

    xRingbufferSend(ringBuffer, &log, sizeof(log), 0);
}


void TTNLogging::logFatal(const char* file, uint16_t line)
{
    ttnLog.logEvent(-3, file, line);
}



extern "C" void LMICOS_logEvent(const char *pMessage)
{
    ttnLog.logEvent(-1, pMessage, 0);

}

extern "C" void LMICOS_logEventUint32(const char *pMessage, uint32_t datum)
{
    ttnLog.logEvent(-2, pMessage, datum);
}


// ---------------------------------------------------------------------------
// Log output

void TTNLogging::loggingTask(void* param)
{
    RingbufHandle_t ringBuffer = (RingbufHandle_t)param;

    while (true) {
        size_t size;
        TTNLogMessage* log = (TTNLogMessage*) xRingbufferReceive(ringBuffer, &size, portMAX_DELAY);
        if (log == nullptr)
            continue;

        if (log->event == -1)
        {
            ESP_LOGI(TAG, "%s: opmode=0x%x", log->message, log->opmode);
        }
        else if (log->event == -2)
        {
            ESP_LOGI(TAG, "%s: datum=0x%x, opmode=0x%x)", log->message, log->datum, log->opmode);
        }
        else if (log->event == -3)
        {
            ESP_LOGE(TAG, "%s, %d: freq=%d.%d",
                log->message, log->datum,
                log->freq / 1000000, (log->freq % 1000000) / 100000
            );
        }

        vRingbufferReturnItem(ringBuffer, log);
    }
}


#endif
