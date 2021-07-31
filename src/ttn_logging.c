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

#if LMIC_ENABLE_event_logging

#include "ttn_logging.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lmic/lmic.h"
#include <string.h>

#define NUM_RINGBUF_MSG 50
#define TAG "lmic"

/**
 * @brief Message structure used in ring buffer
 *
 * The structure is sent from the LMIC task to the logging task.
 */
typedef struct
{
    const char *message;
    uint32_t datum;
    ev_t event;
    ostime_t time;
    ostime_t txend;
    ostime_t globalDutyAvail;
    u4_t freq;
    u2_t opmode;
    u2_t fcntDn;
    u2_t fcntUp;
    u2_t rxsyms;
    rps_t rps;
    u1_t txChnl;
    u1_t datarate;
    u1_t txrxFlags;
    u1_t saveIrqFlags;
} TTNLogMessage;

static void loggingTask(void *param);
static void logFatal(const char *const file, const uint16_t line);

static void printMessage(TTNLogMessage *log);
static void printFatalError(TTNLogMessage *log);
static void printEvent(TTNLogMessage *log);
static void printEvtJoined(TTNLogMessage *log);
static void printEvtJoinFailed(TTNLogMessage *log);
static void printEvtTxComplete(TTNLogMessage *log);
static void printEvtTxStart(TTNLogMessage *log);
static void printEvtRxStart(TTNLogMessage *log);
static void printEvtJoinTxComplete(TTNLogMessage *log);
static void bin2hex(const uint8_t *bin, unsigned len, char *buf, char sep);

// Constants for formatting LORA values
static const char *const SF_NAMES[] = {"FSK", "SF7", "SF8", "SF9", "SF10", "SF11", "SF12", "SFrfu"};
static const char *const BW_NAMES[] = {"BW125", "BW250", "BW500", "BWrfu"};
static const char *const CR_NAMES[] = {"CR 4/5", "CR 4/6", "CR 4/7", "CR 4/8"};
static const char *const CRC_NAMES[] = {"NoCrc", "Crc"};

static RingbufHandle_t ringBuffer;

// Initialize logging
void ttn_log_init(void)
{
    ringBuffer = xRingbufferCreate(NUM_RINGBUF_MSG * sizeof(TTNLogMessage), RINGBUF_TYPE_NOSPLIT);
    if (ringBuffer == NULL)
    {
        ESP_LOGE(TAG, "Failed to create ring buffer");
        ASSERT(0);
    }

    xTaskCreate(loggingTask, "ttn_log", 1024 * 4, ringBuffer, 4, NULL);
    hal_set_failure_handler(logFatal);
}

// Record a logging event for later output
void ttn_log_event(int event, const char *message, uint32_t datum)
{
    if (ringBuffer == NULL)
        return;

    // capture state
    TTNLogMessage log = {
        .message = message,
        .datum = datum,
        .time = os_getTime(),
        .txend = LMIC.txend,
        .globalDutyAvail = LMIC.globalDutyAvail,
        .event = (ev_t)event,
        .freq = LMIC.freq,
        .opmode = LMIC.opmode,
        .fcntDn = (u2_t)LMIC.seqnoDn,
        .fcntUp = (u2_t)LMIC.seqnoUp,
        .rxsyms = LMIC.rxsyms,
        .rps = LMIC.rps,
        .txChnl = LMIC.txChnl,
        .datarate = LMIC.datarate,
        .txrxFlags = LMIC.txrxFlags,
        .saveIrqFlags = LMIC.saveIrqFlags,
    };

    xRingbufferSend(ringBuffer, &log, sizeof(log), 0);
}

// record a fatal event (failed assert) for later output
void logFatal(const char *const file, const uint16_t line)
{
    ttn_log_event(-3, file, line);
}

// Record an informational message for later output
// The message must not be freed.
void LMICOS_logEvent(const char *pMessage)
{
    ttn_log_event(-1, pMessage, 0);
}

// Record an information message with an integer value for later output
// The message must not be freed.
void LMICOS_logEventUint32(const char *pMessage, uint32_t datum)
{
    ttn_log_event(-2, pMessage, datum);
}

// ---------------------------------------------------------------------------
// Log output

// Tasks that receiveds the recorded messages, formats and outputs them.
void loggingTask(void *param)
{
    RingbufHandle_t ringBuffer = (RingbufHandle_t)param;

    while (true)
    {
        size_t size;
        TTNLogMessage *log = (TTNLogMessage *)xRingbufferReceive(ringBuffer, &size, portMAX_DELAY);
        if (log == NULL)
            continue;

        printMessage(log);

        vRingbufferReturnItem(ringBuffer, log);
    }
}

// Format and output a log message
void printMessage(TTNLogMessage *log)
{
    switch ((int)log->event)
    {
    case -1:
        ESP_LOGI(TAG, "%u (%d ms) - %s: opmode=%x", log->time, osticks2ms(log->time), log->message, log->opmode);
        break;

    case -2:
        ESP_LOGI(TAG, "%u (%d ms) - %s: datum=0x%x, opmode=%x)", log->time, osticks2ms(log->time), log->message,
                 log->datum, log->opmode);
        break;

    case -3:
        printFatalError(log);
        break;

    default:
        printEvent(log);
        break;
    }
}

void printFatalError(TTNLogMessage *log)
{
    ESP_LOGE(TAG, "%u (%d ms) - %s, %d", log->time, osticks2ms(log->time), log->message, log->datum);
    ESP_LOGE(TAG, "- freq=%d.%d, txend=%u, avail=%u, ch=%u", log->freq / 1000000, (log->freq % 1000000) / 100000,
             log->txend, log->globalDutyAvail, (unsigned)log->txChnl);
    rps_t rps = log->rps;
    ESP_LOGE(TAG, "- rps=0x%02x (%s, %s, %s, %s, IH=%d)", rps, SF_NAMES[getSf(rps)], BW_NAMES[getBw(rps)],
             CR_NAMES[getCr(rps)], CRC_NAMES[getNocrc(rps)], getIh(rps));
    ESP_LOGE(TAG, "- opmode=%x, txrxFlags=0x%02x%s, saveIrqFlags=0x%02x", log->opmode, log->txrxFlags,
             (log->txrxFlags & TXRX_ACK) != 0 ? "; received ack" : "", log->saveIrqFlags);
}

void printEvent(TTNLogMessage *log)
{
    ESP_LOGI(TAG, "%u (%d ms) - %s", log->time, osticks2ms(log->time), log->message);

    switch ((int)log->event)
    {
    case EV_JOINED:
        printEvtJoined(log);
        break;

    case EV_JOIN_FAILED:
        printEvtJoinFailed(log);
        break;

    case EV_TXCOMPLETE:
        printEvtTxComplete(log);
        break;

    case EV_TXSTART:
        printEvtTxStart(log);
        break;

    case EV_RXSTART:
        printEvtRxStart(log);
        break;

    case EV_JOIN_TXCOMPLETE:
        printEvtJoinTxComplete(log);
        break;

    default:
        break;
    };
}

// Format and output the detail of a successful network join
void printEvtJoined(TTNLogMessage *log)
{
    ESP_LOGI(TAG, "- ch=%d", (unsigned)log->txChnl);

    u4_t netid = 0;
    devaddr_t devaddr = 0;
    u1_t nwkKey[16];
    u1_t artKey[16];
    LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);

    ESP_LOGI(TAG, "- netid: %d", netid);

    ESP_LOGI(TAG, "- devaddr: %08x", devaddr);

    char hexBuf[48];
    bin2hex((uint8_t *)&artKey, sizeof(artKey), hexBuf, '-');
    ESP_LOGI(TAG, "- artKey: %s", hexBuf);

    bin2hex((uint8_t *)&nwkKey, sizeof(nwkKey), hexBuf, '-');
    ESP_LOGI(TAG, "- nwkKey: %s", hexBuf);
}

// Format and output the detail of a failed network join
void printEvtJoinFailed(TTNLogMessage *log)
{
    rps_t rps = log->rps;
    ESP_LOGE(TAG, "- freq=%d.%d, opmode=%x, rps=0x%02x (%s, %s, %s, %s, IH=%d)", log->freq / 1000000,
             (log->freq % 1000000) / 100000, log->opmode, rps, SF_NAMES[getSf(rps)], BW_NAMES[getBw(rps)],
             CR_NAMES[getCr(rps)], CRC_NAMES[getNocrc(rps)], getIh(rps));
}

void printEvtTxComplete(TTNLogMessage *log)
{
    rps_t rps = log->rps;
    ESP_LOGI(TAG, "- ch=%d, rps=0x%02x (%s, %s, %s, %s, IH=%d)", (unsigned)log->txChnl, rps, SF_NAMES[getSf(rps)],
             BW_NAMES[getBw(rps)], CR_NAMES[getCr(rps)], CRC_NAMES[getNocrc(rps)], getIh(rps));
    ESP_LOGI(TAG, "- txrxFlags=0x%02x%s, FcntUp=%04x, FcntDn=%04x, txend=%u", log->txrxFlags,
             (log->txrxFlags & TXRX_ACK) != 0 ? "; received ack" : "", log->fcntUp, log->fcntDn, log->txend);
}

void printEvtTxStart(TTNLogMessage *log)
{
    rps_t rps = log->rps;
    ESP_LOGI(TAG, "- ch=%d, rps=0x%02x (%s, %s, %s, %s, IH=%d)", (unsigned)log->txChnl, rps, SF_NAMES[getSf(rps)],
             BW_NAMES[getBw(rps)], CR_NAMES[getCr(rps)], CRC_NAMES[getNocrc(rps)], getIh(rps));
    ESP_LOGI(TAG, "- datarate=%u, opmode=%x, txend=%u", log->datarate, log->opmode, log->txend);
}

void printEvtRxStart(TTNLogMessage *log)
{
    rps_t rps = log->rps;
    ESP_LOGI(TAG, "- freq=%d.%d, rps=0x%02x (%s, %s, %s, %s, IH=%d)", log->freq / 1000000,
             (log->freq % 1000000) / 100000, rps, SF_NAMES[getSf(rps)], BW_NAMES[getBw(rps)], CR_NAMES[getCr(rps)],
             CRC_NAMES[getNocrc(rps)], getIh(rps));
    ESP_LOGI(TAG, "- delta=%dms, rxsysm=%u", osticks2ms(log->time - log->txend), log->rxsyms);
}

void printEvtJoinTxComplete(TTNLogMessage *log)
{
    ESP_LOGI(TAG, "- saveIrqFlags=0x%02x", log->saveIrqFlags);
}

static const char *HEX_DIGITS = "0123456789ABCDEF";

/**
 * @brief Convert binary data to hexadecimal representation.
 *
 * @param bin start of binary data
 * @param len length of binary data (in bytes)
 * @param buf buffer for hexadecimal result
 * @param sep separator used between bytes (or 0 for none)
 */
void bin2hex(const uint8_t *bin, unsigned len, char *buf, char sep)
{
    int tgt = 0;
    for (int i = 0; i < len; i++)
    {
        if (sep != 0 && i != 0)
            buf[tgt++] = sep;
        buf[tgt++] = HEX_DIGITS[bin[i] >> 4];
        buf[tgt++] = HEX_DIGITS[bin[i] & 0xf];
    }
    buf[tgt] = 0;
}

#endif
