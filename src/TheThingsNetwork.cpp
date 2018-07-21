/*******************************************************************************
 * Copyright (c) 2018 Manuel Bleichenbacher
 * 
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * This the hardware abstraction layer to run LMIC in on ESP32 using ESP-iDF.
 *******************************************************************************/

#include "freertos/FreeRTOS.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "TheThingsNetwork.h"
#include "esp_log.h"
#include "oslmic.h"
#include "hal.h"
#include "hal_esp32.h"
#include "lmic.h"

enum ClientAction
{
    eActionUnrelated,
    eActionJoining,
    eActionSending
};

static const char *TAG = "ttn";
static const char *NVS_FLASH_PARTITION = "ttn";
static const char *NVS_FLASH_KEY_DEV_EUI = "devEui";
static const char *NVS_FLASH_KEY_APP_EUI = "appEui";
static const char *NVS_FLASH_KEY_APP_KEY = "appKey";

static TheThingsNetwork* ttnInstance;
static uint8_t devEui[8];
static uint8_t appEui[8];
static uint8_t appKey[16];
static QueueHandle_t resultQueue;
static ClientAction clientAction = eActionUnrelated;


static bool readNvsValue(nvs_handle handle, const char* key, uint8_t* data, size_t expectedLength, bool silent);
static bool writeNvsValue(nvs_handle handle, const char* key, const uint8_t* data, size_t len);
static bool hexStringToBin(const char *hex, uint8_t *buf, int len);
static int hexTupleToByte(const char *hex);
static int hexDigitToVal(char ch);
static void swapByteOrder(uint8_t* buf, int len);


TheThingsNetwork::TheThingsNetwork()
    : messageCallback(NULL), haveKeys(false)
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
    if (!decodeKeys(devEui, appEui, appKey))
        return false;
    
    return saveKeys();
}

bool TheThingsNetwork::decodeKeys(const char *devEui, const char *appEui, const char *appKey)
{
    haveKeys = false;

    if (strlen(devEui) != 16 || !hexStringToBin(devEui, ::devEui, 8))
    {
        ESP_LOGW(TAG, "Invalid device EUI: %s", devEui);
        return false;
    }

    swapByteOrder(::devEui, 8);

    if (strlen(appEui) != 16 || !hexStringToBin(appEui, ::appEui, 8))
    {
        ESP_LOGW(TAG, "Invalid application EUI: %s", appEui);
        return false;
    }

    swapByteOrder(::appEui, 8);

    if (strlen(appKey) != 32 || !hexStringToBin(appKey, ::appKey, 16))
    {
        ESP_LOGW(TAG, "Invalid application key: %s", appEui);
        return false;
    }

    haveKeys = true;
    return true;
}

bool TheThingsNetwork::join(const char *devEui, const char *appEui, const char *appKey)
{
    if (!decodeKeys(devEui, appEui, appKey))
        return false;
    
    return joinCore();
}

bool TheThingsNetwork::join()
{
    if (!haveKeys)
    {
        if (!restoreKeys(false))
            return false;
    }

    return joinCore();
}

bool TheThingsNetwork::joinCore()
{
    if (!haveKeys)
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

bool TheThingsNetwork::saveKeys()
{
    bool result = false;

    nvs_handle handle = 0;
    esp_err_t res = nvs_open(NVS_FLASH_PARTITION, NVS_READWRITE, &handle);
    if (res == ESP_ERR_NVS_NOT_INITIALIZED)
    {
        ESP_LOGW(TAG, "NVS storage is not initialized. Call 'nvs_flash_init()' first.");
        goto done;
    }
    ESP_ERROR_CHECK(res);
    if (res != ESP_OK)
        goto done;

    if (!writeNvsValue(handle, NVS_FLASH_KEY_DEV_EUI, ::devEui, sizeof(::devEui)))
        goto done;
        
    if (!writeNvsValue(handle, NVS_FLASH_KEY_APP_EUI, ::appEui, sizeof(::appEui)))
        goto done;
        
    if (!writeNvsValue(handle, NVS_FLASH_KEY_APP_KEY, ::appKey, sizeof(::appKey)))
        goto done;

    res = nvs_commit(handle);
    ESP_ERROR_CHECK(res);
    
    result = true;
    ESP_LOGI(TAG, "Dev and app EUI and app key saved in NVS storage");

done:
    nvs_close(handle);
    return result;
}

bool TheThingsNetwork::restoreKeys(bool silent)
{
    haveKeys = false;
    
    nvs_handle handle = 0;
    esp_err_t res = nvs_open(NVS_FLASH_PARTITION, NVS_READONLY, &handle);
    if (res == ESP_ERR_NVS_NOT_FOUND)
        return false; // partition does not exist yet
    if (res == ESP_ERR_NVS_NOT_INITIALIZED)
    {
        ESP_LOGW(TAG, "NVS storage is not initialized. Call 'nvs_flash_init()' first.");
        goto done;
    }
    ESP_ERROR_CHECK(res);
    if (res != ESP_OK)
        goto done;

    if (!readNvsValue(handle, NVS_FLASH_KEY_DEV_EUI, ::devEui, sizeof(::devEui), silent))
        goto done;

    if (!readNvsValue(handle, NVS_FLASH_KEY_APP_EUI, ::appEui, sizeof(::appEui), silent))
        goto done;

    if (!readNvsValue(handle, NVS_FLASH_KEY_APP_KEY, ::appKey, sizeof(::appKey), silent))
        goto done;

    haveKeys = true;
    ESP_LOGI(TAG, "Dev and app EUI and app key have been restored from NVS storage");

done:
    nvs_close(handle);
    return haveKeys;
}

bool TheThingsNetwork::isProvisioned()
{
    if (haveKeys)
        return true;
    
    return restoreKeys(true);
}

bool readNvsValue(nvs_handle handle, const char* key, uint8_t* data, size_t expectedLength, bool silent)
{
    size_t size = expectedLength;
    esp_err_t res = nvs_get_blob(handle, key, data, &size);
    if (res == ESP_OK && size == expectedLength)
        return true;

    if (res == ESP_OK && size != expectedLength)
    {
        if (!silent)
            ESP_LOGW(TAG, "Invalid size of NVS data for %s", key);
        return false;
    }

    if (res == ESP_ERR_NVS_NOT_FOUND)
    {
        if (!silent)
            ESP_LOGW(TAG, "No NVS data found for %s", key);
        return false;
    }
    
    ESP_ERROR_CHECK(res);
    return false;
}

bool writeNvsValue(nvs_handle handle, const char* key, const uint8_t* data, size_t len)
{
    uint8_t buf[16];
    if (readNvsValue(handle, key, buf, len, true) && memcmp(buf, data, len) == 0)
        return true; // unchanged
    
    esp_err_t res = nvs_set_blob(handle, key, data, len);
    ESP_ERROR_CHECK(res);

    return res == ESP_OK;
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

// This EUI must be in little-endian format, so least-significant-byte first.
// When copying an EUI from ttnctl output, this means to reverse the bytes.
// For TTN issued EUIs the last bytes should be 0xD5, 0xB3, 0x70.
// The order is swapped in TheThingsNetwork::provision().
void os_getArtEui (u1_t* buf)
{
    memcpy(buf, appEui, 8);
}

// This should also be in little endian format, see above.
void os_getDevEui (u1_t* buf)
{
    memcpy(buf, devEui, 8);
}

// This key should be in big endian format (or, since it is not really a number
// but a block of memory, endianness does not really apply). In practice, a key
// taken from ttnctl can be copied as-is.
void os_getDevKey (u1_t* buf)
{
    memcpy(buf, appKey, 16);
}

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
        if (ev != EV_JOINED && EV_REJOIN_FAILED)
            return;
    }
    else
    {
        if (ev != EV_TXCOMPLETE && ev != EV_LINK_DEAD)
            return;
    }

    int result = ev;
    clientAction = eActionUnrelated;
    xQueueSend(resultQueue, &result, 100 / portTICK_PERIOD_MS);
}


// --- Helper functions ---

bool hexStringToBin(const char *hex, uint8_t *buf, int len)
{
    const char* ptr = hex;
    for (int i = 0; i < len; i++)
    {
        int val = hexTupleToByte(ptr);
        if (val < 0)
            return false;
        buf[i] = val;
        ptr += 2;
    }
    return true;
}

int hexTupleToByte(const char *hex)
{
    int nibble1 = hexDigitToVal(hex[0]);
    if (nibble1 < 0)
        return -1;
    int nibble2 = hexDigitToVal(hex[1]);
    if (nibble2 < 0)
        return -1;
    return (nibble1 << 4) | nibble2;
}

int hexDigitToVal(char ch)
{
    if (ch >= '0' && ch <= '9')
        return ch - '0';
    if (ch >= 'A' && ch <= 'F')
        return ch + 10 - 'A';
    if (ch >= 'a' && ch <= 'f')
        return ch + 10 - 'a';
    return -1;
}

void swapByteOrder(uint8_t* buf, int len)
{
    uint8_t* p1 = buf;
    uint8_t* p2 = buf + len - 1;
    while (p1 < p2)
    {
        uint8_t t = *p1;
        *p1 = *p2;
        *p2 = t;
        p1++;
        p2--;
    }
}