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

static const char* const SF_NAMES[] = { "FSK", "SF7", "SF8", "SF9", "SF10", "SF11", "SF12", "SFrfu" };
static const char* const BW_NAMES[] = { "BW125", "BW250", "BW500", "BWrfu" };
static const char* const CR_NAMES[] = { "CR 4/5", "CR 4/6", "CR 4/7", "CR 4/8" };
static const char* const CRC_NAMES[] = { "NoCrc", "Crc" };

static void printMessage(TTNLogMessage* log);
static void printEvtJoined(TTNLogMessage* log);
static void printEvtJoinFailed(TTNLogMessage* log);
static void bin2hex(const uint8_t* bin, unsigned len, char* buf, char sep = 0);


TTNLogging* TTNLogging::initInstance()
{
    ttnLog.init();
    return &ttnLog;
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

        printMessage(log);

        vRingbufferReturnItem(ringBuffer, log);
    }
}


void printMessage(TTNLogMessage* log)
{
    switch((int)log->event)
    {
        case -1:
            ESP_LOGI(TAG, "%s: opmode=0x%x", log->message, log->opmode);
            break;

        case -2:
            ESP_LOGI(TAG, "%s: datum=0x%x, opmode=0x%x)", log->message, log->datum, log->opmode);
            break;

        case -3:
            ESP_LOGE(TAG, "%s, %d: freq=%d.%d",
                log->message, log->datum,
                log->freq / 1000000, (log->freq % 1000000) / 100000
            );
            break;

        case EV_JOINED:
            printEvtJoined(log);
            break;

        case EV_JOIN_FAILED:
            printEvtJoinFailed(log);
            break;

        default:
            break;
    }
}


void printEvtJoined(TTNLogMessage* log)
{
    ESP_LOGI(TAG, "%s: ch=%d", log->message, (unsigned)log->txChnl);

    u4_t netid = 0;
    devaddr_t devaddr = 0;
    u1_t nwkKey[16];
    u1_t artKey[16];
    LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);

    ESP_LOGI(TAG, "netid: %d", netid);

    ESP_LOGI(TAG, "devaddr: %08x", devaddr);

    char hexBuf[48];
    bin2hex((uint8_t*)&artKey, sizeof(artKey), hexBuf, '-');
    ESP_LOGI(TAG, "artKey: %s", hexBuf);

    bin2hex((uint8_t*)&nwkKey, sizeof(nwkKey), hexBuf, '-');
    ESP_LOGI(TAG, "nwkKey: %s", hexBuf);
}


void printEvtJoinFailed(TTNLogMessage* log)
{
    rps_t rps = log->rps;
    ESP_LOGE(TAG, "%s: freq=%d.%d, opmode=0x%x, rps=0x%02x (%s, %s, %s, %s, IH=%d)",
        log->message,
        log->freq / 1000000, (log->freq % 1000000) / 100000,
        log->opmode,
        rps,
        SF_NAMES[getSf(rps)],
        BW_NAMES[getBw(rps)],
        CR_NAMES[getCr(rps)],
        CRC_NAMES[getNocrc(rps)],
        getIh(rps)
    );
}


static const char* HEX_DIGITS = "0123456789ABCDEF";

void bin2hex(const uint8_t* bin, unsigned len, char* buf, char sep)
{
    int tgt = 0;
    for (int i = 0; i < len; i++) {
        if (sep != 0 && i != 0)
            buf[tgt++] = sep;
        buf[tgt++] = HEX_DIGITS[bin[i] >> 4];
        buf[tgt++] = HEX_DIGITS[bin[i] & 0xf];
    }
    buf[tgt] = 0;
}


#endif
