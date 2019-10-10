/*******************************************************************************
 * 
 * ttn-esp32 - The Things Network device library for ESP-IDF / SX127x
 * 
 * Copyright (c) 2018-2019 Manuel Bleichenbacher
 * 
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 *
 * High-level API for ttn-esp32.
 *******************************************************************************/

#include "freertos/FreeRTOS.h"
#include "esp_event.h"
#include "esp_log.h"
#include "TheThingsNetwork.h"
#include "hal/hal_esp32.h"
#include "lmic/lmic.h"
#include "TTNProvisioning.h"
#include "TTNLogging.h"


enum TTNClientAction
{
    eActionUnrelated,
    eActionJoining,
    eActionTransmission
};

enum TTNEvent {
    EvtNone,
    EvtJoinCompleted,
    EvtJoinFailed,
    EvtMessageReceived,
    EvtTransmissionCompleted,
    EvtTransmissionFailed
};

struct TTNResult {
    TTNResult(TTNEvent ev = EvtNone): event(ev) { }

    TTNEvent event;
    uint8_t port;
    const uint8_t *message;
    size_t messageSize;
};

static const char *TAG = "ttn";

static TheThingsNetwork* ttnInstance;
static QueueHandle_t resultQueue;
static TTNClientAction clientAction = eActionUnrelated;
static TTNProvisioning provisioning;
#if LMIC_ENABLE_event_logging
static TTNLogging* logging;
#endif

static void eventCallback(void* userData, ev_t event);
static void messageReceivedCallback(void *userData, uint8_t port, const uint8_t *message, size_t messageSize);
static void messageTransmittedCallback(void *userData, int success);


TheThingsNetwork::TheThingsNetwork()
    : messageCallback(nullptr)
{
#if defined(TTN_IS_DISABLED)
    ESP_LOGE(TAG, "TTN is disabled. Configure a frequency plan using 'make menuconfig'");
    ASSERT(0);
    esp_restart();
#endif

    ASSERT(ttnInstance == nullptr);
    ttnInstance = this;
    ttn_hal.initCriticalSection();
}

TheThingsNetwork::~TheThingsNetwork()
{
    // nothing to do
}

void TheThingsNetwork::configurePins(spi_host_device_t spi_host, uint8_t nss, uint8_t rxtx, uint8_t rst, uint8_t dio0, uint8_t dio1)
{
    ttn_hal.configurePins(spi_host, nss, rxtx, rst, dio0, dio1);

#if LMIC_ENABLE_event_logging
    logging = TTNLogging::initInstance();
#endif

    LMIC_registerEventCb(eventCallback, nullptr);
    LMIC_registerRxMessageCb(messageReceivedCallback, nullptr);

    os_init_ex(nullptr);
    reset();

    resultQueue = xQueueCreate(4, sizeof(TTNResult));
    ASSERT(resultQueue != nullptr);
    ttn_hal.startBackgroundTask();
}

void TheThingsNetwork::reset()
{
    ttn_hal.enterCriticalSection();
    LMIC_reset();
    ttn_hal.leaveCriticalSection();
}

bool TheThingsNetwork::provision(const char *devEui, const char *appEui, const char *appKey)
{
    if (!provisioning.decodeKeys(devEui, appEui, appKey))
        return false;
    
    return provisioning.saveKeys();
}

bool TheThingsNetwork::provisionWithMAC(const char *appEui, const char *appKey)
{
    if (!provisioning.fromMAC(appEui, appKey))
        return false;
    
    return provisioning.saveKeys();
}


void TheThingsNetwork::startProvisioningTask()
{
#if defined(TTN_HAS_AT_COMMANDS)
    provisioning.startTask();
#else
    ESP_LOGE(TAG, "AT commands are disabled. Change the configuration using 'make menuconfig'");
    ASSERT(0);
    esp_restart();
#endif
}

void TheThingsNetwork::waitForProvisioning()
{
#if defined(TTN_HAS_AT_COMMANDS)
    if (isProvisioned())
    {
        ESP_LOGI(TAG, "Device is already provisioned");
        return;
    }

    while (!provisioning.haveKeys())
        vTaskDelay(1000 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "Device successfully provisioned");
#else
    ESP_LOGE(TAG, "AT commands are disabled. Change the configuration using 'make menuconfig'");
    ASSERT(0);
    esp_restart();
#endif
}

bool TheThingsNetwork::join(const char *devEui, const char *appEui, const char *appKey)
{
    if (!provisioning.decodeKeys(devEui, appEui, appKey))
        return false;
    
    return joinCore();
}

bool TheThingsNetwork::join()
{
    if (!provisioning.haveKeys())
    {
        if (!provisioning.restoreKeys(false))
            return false;
    }

    return joinCore();
}

bool TheThingsNetwork::joinCore()
{
    if (!provisioning.haveKeys())
    {
        ESP_LOGW(TAG, "Device EUI, App EUI and/or App key have not been provided");
        return false;
    }

    ttn_hal.enterCriticalSection();
    clientAction = eActionJoining;
    LMIC_startJoining();
    ttn_hal.wakeUp();
    ttn_hal.leaveCriticalSection();

    TTNResult result;
    xQueueReceive(resultQueue, &result, portMAX_DELAY);
    return result.event == EvtJoinCompleted;
}

TTNResponseCode TheThingsNetwork::transmitMessage(const uint8_t *payload, size_t length, port_t port, bool confirm)
{
    ttn_hal.enterCriticalSection();
    if (LMIC.opmode & OP_TXRXPEND)
    {
        ttn_hal.leaveCriticalSection();
        return kTTNErrorTransmissionFailed;
    }

    clientAction = eActionTransmission;
    LMIC.client.txMessageCb = messageTransmittedCallback;
    LMIC.client.txMessageUserData = nullptr;
    LMIC_setTxData2(port, (xref2u1_t)payload, length, confirm);
    ttn_hal.wakeUp();
    ttn_hal.leaveCriticalSection();

    while (true)
    {
        TTNResult result;
        xQueueReceive(resultQueue, &result, portMAX_DELAY);

        switch (result.event)
        {
            case EvtMessageReceived:
                if (messageCallback != nullptr)
                    messageCallback(result.message, result.messageSize, result.port);
                break;

            case EvtTransmissionCompleted:
                return kTTNSuccessfulTransmission;

            case EvtTransmissionFailed:
                return kTTNErrorTransmissionFailed;

            default:
                ASSERT(0);
        }
    }
}

void TheThingsNetwork::onMessage(TTNMessageCallback callback)
{
    messageCallback = callback;
}


bool TheThingsNetwork::isProvisioned()
{
    if (provisioning.haveKeys())
        return true;
    
    provisioning.restoreKeys(true);

    return provisioning.haveKeys();
}

void TheThingsNetwork::setRSSICal(int8_t rssiCal)
{
    ttn_hal.rssiCal = rssiCal;
}


// --- Callbacks ---

#if CONFIG_LOG_DEFAULT_LEVEL >= 3 || LMIC_ENABLE_event_logging
const char *eventNames[] = { LMIC_EVENT_NAME_TABLE__INIT };
#endif



void eventCallback(void* userData, ev_t event)
{
#if LMIC_ENABLE_event_logging
    logging->logEvent(event, eventNames[event], 0);
#elif CONFIG_LOG_DEFAULT_LEVEL >= 3
    ESP_LOGI(TAG, "event %s", eventNames[event]);
#endif

    if (event == EV_TXCOMPLETE) {
        if (LMIC.txrxFlags & TXRX_ACK)
            ESP_LOGI(TAG, "ACK received\n");
    }

    TTNEvent ttnEvent = EvtNone;

    if (clientAction == eActionJoining)
    {
        if (event == EV_JOINED)
        {
            ttnEvent = EvtJoinCompleted;
        }
        else if (event == EV_REJOIN_FAILED || event == EV_RESET)
        {
            ttnEvent = EvtJoinFailed;
        }
    }

    if (ttnEvent == EvtNone)
        return;

    TTNResult result(ttnEvent);
    clientAction = eActionUnrelated;
    xQueueSend(resultQueue, &result, 100 / portTICK_PERIOD_MS);
}

void messageReceivedCallback(void *userData, uint8_t port, const uint8_t *message, size_t nMessage)
{
    TTNResult result(EvtMessageReceived);
    result.port = port;
    result.message = message;
    result.messageSize = nMessage;
    xQueueSend(resultQueue, &result, 100 / portTICK_PERIOD_MS);
}

void messageTransmittedCallback(void *userData, int success)
{
    clientAction = eActionUnrelated;
    TTNResult result(success ? EvtTransmissionCompleted : EvtTransmissionFailed);
    xQueueSend(resultQueue, &result, 100 / portTICK_PERIOD_MS);
}
