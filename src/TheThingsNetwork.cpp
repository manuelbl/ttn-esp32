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
#include "ttn_provisioning.h"
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
    hal_esp32_init_critical_section();
}

TheThingsNetwork::~TheThingsNetwork()
{
    // nothing to do
}

void TheThingsNetwork::configurePins(spi_host_device_t spi_host, uint8_t nss, uint8_t rxtx, uint8_t rst, uint8_t dio0, uint8_t dio1)
{
    hal_esp32_configure_pins(spi_host, nss, rxtx, rst, dio0, dio1);

#if LMIC_ENABLE_event_logging
    logging = TTNLogging::initInstance();
#endif

    LMIC_registerEventCb(eventCallback, nullptr);
    LMIC_registerRxMessageCb(messageReceivedCallback, nullptr);

    os_init_ex(nullptr);
    reset();

    lmicEventQueue = xQueueCreate(4, sizeof(TTNLmicEvent));
    ASSERT(lmicEventQueue != nullptr);
    hal_esp32_start_lmic_task();
}

void TheThingsNetwork::reset()
{
    hal_esp32_enter_critical_section();
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 4 / 100);
    waitingReason = eWaitingNone;
    hal_esp32_leave_critical_section();
}

void TheThingsNetwork::shutdown()
{
    hal_esp32_enter_critical_section();
    LMIC_shutdown();
    hal_esp32_stop_lmic_task();
    waitingReason = eWaitingNone;
    hal_esp32_leave_critical_section();
}

void TheThingsNetwork::startup()
{
    hal_esp32_enter_critical_section();
    LMIC_reset();
    hal_esp32_start_lmic_task();
    hal_esp32_leave_critical_section();
}

bool TheThingsNetwork::provision(const char *devEui, const char *appEui, const char *appKey)
{
    if (!ttn_provision_decode_keys(devEui, appEui, appKey))
        return false;
    
    return ttn_provision_save_keys();
}

bool TheThingsNetwork::provisionWithMAC(const char *appEui, const char *appKey)
{
    if (!ttn_provision_from_mac(appEui, appKey))
        return false;
    
    return ttn_provision_save_keys();
}


void TheThingsNetwork::startProvisioningTask()
{
#if defined(TTN_HAS_AT_COMMANDS)
    ttn_provision_start_task();
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

    while (!ttn_provision_have_keys())
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
    if (!ttn_provision_decode_keys(devEui, appEui, appKey))
        return false;
    
    return joinCore();
}

bool TheThingsNetwork::join()
{
    if (!ttn_provision_have_keys())
    {
        if (!ttn_provision_restore_keys(false))
            return false;
    }

    return joinCore();
}

bool TheThingsNetwork::joinCore()
{
    if (!ttn_provision_have_keys())
    {
        ESP_LOGW(TAG, "Device EUI, App EUI and/or App key have not been provided");
        return false;
    }

    hal_esp32_enter_critical_section();
    xQueueReset(lmicEventQueue);
    waitingReason = eWaitingForJoin;
    LMIC_startJoining();
    hal_esp32_wake_up();
    hal_esp32_leave_critical_section();

    TTNLmicEvent event;
    xQueueReceive(lmicEventQueue, &event, portMAX_DELAY);
    return event.event == eEvtJoinCompleted;
}

TTNResponseCode TheThingsNetwork::transmitMessage(const uint8_t *payload, size_t length, port_t port, bool confirm)
{
    hal_esp32_enter_critical_section();
    if (waitingReason != eWaitingNone || (LMIC.opmode & OP_TXRXPEND) != 0)
    {
        hal_esp32_leave_critical_section();
        return kTTNErrorTransmissionFailed;
    }

    waitingReason = eWaitingForTransmission;
    LMIC.client.txMessageCb = messageTransmittedCallback;
    LMIC.client.txMessageUserData = nullptr;
    LMIC_setTxData2(port, (xref2u1_t)payload, length, confirm);
    hal_esp32_wake_up();
    hal_esp32_leave_critical_section();

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
    if (ttn_provision_have_keys())
        return true;
    
    ttn_provision_restore_keys(true);

    return ttn_provision_have_keys();
}

void TheThingsNetwork::setRSSICal(int8_t rssiCal)
{
    hal_esp32_set_rssi_cal(rssiCal);
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

int TheThingsNetwork::rssi()
{
    return LMIC.rssi;
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
