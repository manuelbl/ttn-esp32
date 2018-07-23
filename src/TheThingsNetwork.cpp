/*******************************************************************************
 * 
 * ttn-esp32 - The Things Network device library for ESP-IDF / SX127x
 * 
 * Copyright (c) 2018 Manuel Bleichenbacher
 * 
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 *
 * This the hardware abstraction layer to run LMIC in on ESP32 using ESP-iDF.
 *******************************************************************************/

#include "freertos/FreeRTOS.h"
#include "esp_event.h"
#include "esp_log.h"
#include "TheThingsNetwork.h"
#include "hal.h"
#include "hal_esp32.h"
#include "lmic.h"
#include "provisioning.h"


enum ClientAction
{
    eActionUnrelated,
    eActionJoining,
    eActionSending
};

static const char *TAG = "ttn";

static TheThingsNetwork* ttnInstance;
static QueueHandle_t resultQueue;
static ClientAction clientAction = eActionUnrelated;


TheThingsNetwork::TheThingsNetwork()
    : messageCallback(NULL)
{
    ASSERT(ttnInstance == NULL);
    ttnInstance = this;
    hal_initCriticalSection();

}

TheThingsNetwork::~TheThingsNetwork()
{
    // nothing to do
}

void TheThingsNetwork::configurePins(spi_host_device_t spi_host, uint8_t nss, uint8_t rxtx, uint8_t rst, uint8_t dio0, uint8_t dio1)
{
    lmic_pins.spi_host = spi_host;
    lmic_pins.nss = nss;
    lmic_pins.rxtx = rxtx;
    lmic_pins.rst = rst;
    lmic_pins.dio0 = dio0;
    lmic_pins.dio1 = dio1;

    os_init();
    reset();

    resultQueue = xQueueCreate(12, sizeof(int));
    ASSERT(resultQueue != NULL);
    hal_startBgTask();
}

void TheThingsNetwork::reset()
{
    hal_enterCriticalSection();
    LMIC_reset();
    hal_leaveCriticalSection();
}

bool TheThingsNetwork::provision(const char *devEui, const char *appEui, const char *appKey)
{
    if (!provisioning_decode_keys(devEui, appEui, appKey))
        return false;
    
    return provisioning_save_keys();
}

void TheThingsNetwork::startProvisioningTask()
{
#if !defined(CONFIG_TTN_PROVISION_UART_NONE)
    provisioning_start_task();
#endif
}

void TheThingsNetwork::waitForProvisioning()
{
#if !defined(CONFIG_TTN_PROVISION_UART_NONE)
    if (isProvisioned())
    {
        ESP_LOGI(TAG, "Device is already provisioned");
        return;
    }

    while (!provisioning_have_keys())
        vTaskDelay(1000 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "Device successfully provisioned");
#endif
}

bool TheThingsNetwork::join(const char *devEui, const char *appEui, const char *appKey)
{
    if (!provisioning_decode_keys(devEui, appEui, appKey))
        return false;
    
    return joinCore();
}

bool TheThingsNetwork::join()
{
    if (!provisioning_have_keys())
    {
        if (!provisioning_restore_keys(false))
            return false;
    }

    return joinCore();
}

bool TheThingsNetwork::joinCore()
{
    if (!provisioning_have_keys())
    {
        ESP_LOGW(TAG, "Device EUI, App EUI and/or App key have not been provided");
        return false;
    }

    hal_enterCriticalSection();
    clientAction = eActionJoining;
    LMIC_startJoining();
    hal_wakeUp();
    hal_leaveCriticalSection();

    int result = 0;
    xQueueReceive(resultQueue, &result, portMAX_DELAY);
    return result == EV_JOINED;
}

TTNResponseCode TheThingsNetwork::transmitMessage(const uint8_t *payload, size_t length, port_t port, bool confirm)
{
    hal_enterCriticalSection();
    if (LMIC.opmode & OP_TXRXPEND)
    {
        hal_leaveCriticalSection();
        return kTTNErrorTransmissionFailed;
    }

    clientAction = eActionSending;
    LMIC_setTxData2(port, (xref2u1_t)payload, length, confirm);
    hal_wakeUp();
    hal_leaveCriticalSection();

    int result = 0;
    xQueueReceive(resultQueue, &result, portMAX_DELAY);

    if (result == EV_TXCOMPLETE)
    {
        bool hasRecevied = (LMIC.txrxFlags & (TXRX_DNW1 | TXRX_DNW2)) != 0;
        if (hasRecevied && messageCallback != NULL)
        {
            port_t port = 0;
            if ((LMIC.txrxFlags & TXRX_PORT))
                port = LMIC.frame[LMIC.dataBeg - 1];
            const uint8_t* msg = NULL;
            if (LMIC.dataLen > 0)
                msg = LMIC.frame + LMIC.dataBeg;
            messageCallback(msg, LMIC.dataLen, port);
        }

        return kTTNSuccessfulTransmission;
    }

    return  kTTNErrorTransmissionFailed;
}

void TheThingsNetwork::onMessage(TTNMessageCallback callback)
{
    messageCallback = callback;
}


bool TheThingsNetwork::isProvisioned()
{
    if (provisioning_have_keys())
        return true;
    
    return provisioning_restore_keys(true);
}


// --- LMIC functions ---

#if CONFIG_LOG_DEFAULT_LEVEL >= 3
static const char *eventNames[] = {
    NULL,
    "EV_SCAN_TIMEOUT", "EV_BEACON_FOUND",
    "EV_BEACON_MISSED", "EV_BEACON_TRACKED", "EV_JOINING",
    "EV_JOINED", "EV_RFU1", "EV_JOIN_FAILED", "EV_REJOIN_FAILED",
    "EV_TXCOMPLETE", "EV_LOST_TSYNC", "EV_RESET",
    "EV_RXCOMPLETE", "EV_LINK_DEAD", "EV_LINK_ALIVE"
};
#endif

void onEvent (ev_t ev) {
    #if CONFIG_LOG_DEFAULT_LEVEL >= 3
        ESP_LOGI(TAG, "event %s", eventNames[ev]);
    #endif

    if (ev == EV_TXCOMPLETE) {
        if (LMIC.txrxFlags & TXRX_ACK)
            ESP_LOGI(TAG, "ACK received\n");
    }

    if (clientAction == eActionUnrelated)
    {
        return;
    }    
    else if (clientAction == eActionJoining)
    {
        if (ev != EV_JOINED && ev != EV_REJOIN_FAILED && ev != EV_RESET)
            return;
    }
    else
    {
        if (ev != EV_TXCOMPLETE && ev != EV_LINK_DEAD && ev != EV_RESET)
            return;
    }

    int result = ev;
    clientAction = eActionUnrelated;
    xQueueSend(resultQueue, &result, 100 / portTICK_PERIOD_MS);
}
