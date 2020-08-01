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
#include "hal/hal_esp32.h"
#include "lmic/lmic.h"
#include "TheThingsNetwork.h"
#include "TTNProvisioning.h"
#include "TTNLogging.h"


/**
 * @brief Reason the user code is waiting
 */
enum TTNWaitingReason
{
    eWaitingNone,
    eWaitingForJoin,
    eWaitingForTransmission
};

/**
 * @brief Event type
 */
enum TTNEvent {
    eEvtNone,
    eEvtJoinCompleted,
    eEvtJoinFailed,
    eEvtMessageReceived,
    eEvtTransmissionCompleted,
    eEvtTransmissionFailed
};

/**
 * @brief Event message sent from LMIC task to waiting client task
 */
struct TTNLmicEvent {
    TTNLmicEvent(TTNEvent ev = eEvtNone): event(ev) { }

    TTNEvent event;
    uint8_t port;
    const uint8_t* message;
    size_t messageSize;
};

static const char *TAG = "ttn";

static TheThingsNetwork* ttnInstance;
static QueueHandle_t lmicEventQueue = nullptr;
static TTNWaitingReason waitingReason = eWaitingNone;
static TTNProvisioning provisioning;
#if LMIC_ENABLE_event_logging
static TTNLogging* logging;
#endif
static TTNRFSettings lastRfSettings[4];
static TTNRxTxWindow currentWindow;

static void eventCallback(void* userData, ev_t event);
static void messageReceivedCallback(void *userData, uint8_t port, const uint8_t *message, size_t messageSize);
static void messageTransmittedCallback(void *userData, int success);
static void saveRFSettings(TTNRFSettings& rfSettings);
static void clearRFSettings(TTNRFSettings& rfSettings);


TheThingsNetwork::TheThingsNetwork()
    : messageCallback(nullptr)
{
#if defined(TTN_IS_DISABLED)
    ESP_LOGE(TAG, "TTN is disabled. Configure a frequency plan using 'make menuconfig'");
    ASSERT(0);
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

    lmicEventQueue = xQueueCreate(4, sizeof(TTNLmicEvent));
    ASSERT(lmicEventQueue != nullptr);
    ttn_hal.startLMICTask();
}

void TheThingsNetwork::reset()
{
    ttn_hal.enterCriticalSection();
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 4 / 100);
    waitingReason = eWaitingNone;
    ttn_hal.leaveCriticalSection();
}

void TheThingsNetwork::shutdown()
{
    ttn_hal.enterCriticalSection();
    LMIC_shutdown();
    ttn_hal.stopLMICTask();
    waitingReason = eWaitingNone;
    ttn_hal.leaveCriticalSection();
}

void TheThingsNetwork::startup()
{
    ttn_hal.enterCriticalSection();
    LMIC_reset();
    ttn_hal.startLMICTask();
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
        vTaskDelay(pdMS_TO_TICKS(1000));

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
    xQueueReset(lmicEventQueue);
    waitingReason = eWaitingForJoin;
    LMIC_startJoining();
    ttn_hal.wakeUp();
    ttn_hal.leaveCriticalSection();

    TTNLmicEvent event;
    xQueueReceive(lmicEventQueue, &event, portMAX_DELAY);
    return event.event == eEvtJoinCompleted;
}

TTNResponseCode TheThingsNetwork::transmitMessage(const uint8_t *payload, size_t length, port_t port, bool confirm)
{
    ttn_hal.enterCriticalSection();
    if (waitingReason != eWaitingNone || (LMIC.opmode & OP_TXRXPEND) != 0)
    {
        ttn_hal.leaveCriticalSection();
        return kTTNErrorTransmissionFailed;
    }

    waitingReason = eWaitingForTransmission;
    LMIC.client.txMessageCb = messageTransmittedCallback;
    LMIC.client.txMessageUserData = nullptr;
    LMIC_setTxData2(port, (xref2u1_t)payload, length, confirm);
    ttn_hal.wakeUp();
    ttn_hal.leaveCriticalSection();

    while (true)
    {
        TTNLmicEvent result;
        xQueueReceive(lmicEventQueue, &result, portMAX_DELAY);

        switch (result.event)
        {
            case eEvtMessageReceived:
                if (messageCallback != nullptr)
                    messageCallback(result.message, result.messageSize, result.port);
                break;

            case eEvtTransmissionCompleted:
                return kTTNSuccessfulTransmission;

            case eEvtTransmissionFailed:
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

bool TheThingsNetwork::adrEnabled()
{
    return LMIC.adrEnabled != 0;
}

void TheThingsNetwork::setAdrEnabled(bool enabled)
{
    LMIC_setAdrMode(enabled);
}

TTNRFSettings TheThingsNetwork::getRFSettings(TTNRxTxWindow window)
{
    int index = static_cast<int>(window) & 0x03;
    return lastRfSettings[index];
}

TTNRFSettings TheThingsNetwork::txSettings()
{
    return lastRfSettings[static_cast<int>(kTTNTxWindow)];
}

TTNRFSettings TheThingsNetwork::rx1Settings()
{
    return lastRfSettings[static_cast<int>(kTTNRx1Window)];
}

TTNRFSettings TheThingsNetwork::rx2Settings()
{
    return lastRfSettings[static_cast<int>(kTTNRx2Window)];
}

TTNRxTxWindow TheThingsNetwork::rxTxWindow()
{
    return currentWindow;
}


// --- Callbacks ---

#if CONFIG_LOG_DEFAULT_LEVEL >= 3 || LMIC_ENABLE_event_logging
const char *eventNames[] = { LMIC_EVENT_NAME_TABLE__INIT };
#endif


// Called by LMIC when an LMIC event (join, join failed, reset etc.) occurs
void eventCallback(void* userData, ev_t event)
{
    // update monitoring information
    switch(event)
    {
        case EV_TXSTART:
            currentWindow = kTTNTxWindow;
            saveRFSettings(lastRfSettings[static_cast<int>(kTTNTxWindow)]);
            clearRFSettings(lastRfSettings[static_cast<int>(kTTNRx1Window)]);
            clearRFSettings(lastRfSettings[static_cast<int>(kTTNRx2Window)]);
            break;

        case EV_RXSTART:
            if (currentWindow != kTTNRx1Window)
            {
                currentWindow = kTTNRx1Window;
                saveRFSettings(lastRfSettings[static_cast<int>(kTTNRx1Window)]);
            }
            else
            {
                currentWindow = kTTNRx2Window;
                saveRFSettings(lastRfSettings[static_cast<int>(kTTNRx2Window)]);
            }
            break;

        default:
            currentWindow = kTTNIdleWindow;
            break;
    };

#if LMIC_ENABLE_event_logging
    logging->logEvent(event, eventNames[event], 0);
#elif CONFIG_LOG_DEFAULT_LEVEL >= 3
    ESP_LOGI(TAG, "event %s", eventNames[event]);
#endif

    TTNEvent ttnEvent = eEvtNone;

    if (waitingReason == eWaitingForJoin)
    {
        if (event == EV_JOINED)
        {
            ttnEvent = eEvtJoinCompleted;
        }
        else if (event == EV_REJOIN_FAILED || event == EV_RESET)
        {
            ttnEvent = eEvtJoinFailed;
        }
    }

    if (ttnEvent == eEvtNone)
        return;

    TTNLmicEvent result(ttnEvent);
    waitingReason = eWaitingNone;
    xQueueSend(lmicEventQueue, &result, pdMS_TO_TICKS(100));
}

// Called by LMIC when a message has been received
void messageReceivedCallback(void *userData, uint8_t port, const uint8_t *message, size_t nMessage)
{
    TTNLmicEvent result(eEvtMessageReceived);
    result.port = port;
    result.message = message;
    result.messageSize = nMessage;
    xQueueSend(lmicEventQueue, &result, pdMS_TO_TICKS(100));
}

// Called by LMIC when a message has been transmitted (or the transmission failed)
void messageTransmittedCallback(void *userData, int success)
{
    waitingReason = eWaitingNone;
    TTNLmicEvent result(success ? eEvtTransmissionCompleted : eEvtTransmissionFailed);
    xQueueSend(lmicEventQueue, &result, pdMS_TO_TICKS(100));
}


// --- Helpers


void saveRFSettings(TTNRFSettings& rfSettings)
{
    rfSettings.spreadingFactor = static_cast<TTNSpreadingFactor>(getSf(LMIC.rps) + 1);
    rfSettings.bandwidth = static_cast<TTNBandwidth>(getBw(LMIC.rps) + 1);
    rfSettings.frequency = LMIC.freq;
}

void clearRFSettings(TTNRFSettings& rfSettings)
{
    memset(&rfSettings, 0, sizeof(rfSettings));
}
