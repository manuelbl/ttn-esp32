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

#include "TheThingsNetwork.h"
#include "esp_log.h"
#include "oslmic.h"
#include "hal.h"
#include "hal_esp32.h"
#include "lmic.h"

static const char *TAG = "ttn";

static TheThingsNetwork* ttnInstance;
static uint8_t appEui[8];
static uint8_t appKey[16];
static uint8_t devEui[8];

static bool hexStringToBin(const char *hex, uint8_t *buf, int len);
static int hexTupleToByte(const char *hex);
static int hexDigitToVal(char ch);
static void swapByteOrder(uint8_t* buf, int len);


TheThingsNetwork::TheThingsNetwork(uint8_t sf, uint8_t fsb)
{
    ttnInstance = this;
    spreadingFactor = sf;
    frequencySubband = fsb;
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

    hal_startBgTask();
}

void TheThingsNetwork::reset()
{
    hal_enterCriticalSection();
    LMIC_reset();
    hal_leaveCriticalSection();
}

bool TheThingsNetwork::provision(const char *appEui, const char *appKey, const char* devEui)
{
    return decodeKeys(appEui, appKey, devEui);
}

bool TheThingsNetwork::decodeKeys(const char *appEui, const char *appKey, const char* devEui)
{
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

    if (strlen(devEui) != 16 || !hexStringToBin(devEui, ::devEui, 8))
    {
        ESP_LOGW(TAG, "Invalid device EUI: %s", devEui);
        return false;
    }

    swapByteOrder(::devEui, 8);

    return true;
}

bool TheThingsNetwork::join(const char *appEui, const char *appKey, const char *devEui, int8_t retries, uint32_t retryDelay)
{
    if (!decodeKeys(appEui, appKey, devEui))
        return false;
    
    return join(retries, retryDelay);
}

bool TheThingsNetwork::join(int8_t retries, uint32_t retryDelay)
{
    hal_enterCriticalSection();
    LMIC_startJoining();
    hal_wakeUp();
    hal_leaveCriticalSection();
    return true;
}

ttn_response_t TheThingsNetwork::sendBytes(const uint8_t *payload, size_t length, port_t port, bool confirm, uint8_t sf)
{
    hal_enterCriticalSection();
    if (LMIC.opmode & OP_TXRXPEND)
    {
        hal_leaveCriticalSection();
        return TTN_ERROR_SEND_COMMAND_FAILED;
    }

    LMIC_setTxData2(port, (xref2u1_t)payload, length, confirm);
    hal_wakeUp();
    hal_leaveCriticalSection();
    return TTN_SUCCESSFUL_TRANSMISSION;
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
        ESP_LOGI(TAG, "%ld: event %s", os_getTime(), eventNames[ev]);
    #endif

    if (ev == EV_JOINED)
    {
        // Disable link check validation (automatically enabled
        // during join, but not supported by TTN at this time).
        LMIC_setLinkCheckMode(0);
    } else if (ev == EV_TXCOMPLETE) {
            if (LMIC.txrxFlags & TXRX_ACK)
              printf("Received ack\n");
            if (LMIC.dataLen) {
              printf("Received %d bytes of payload\n", LMIC.dataLen);
            }
    }
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