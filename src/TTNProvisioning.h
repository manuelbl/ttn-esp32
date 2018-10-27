/*******************************************************************************
 * 
 * ttn-esp32 - The Things Network device library for ESP-IDF / SX127x
 * 
 * Copyright (c) 2018 Manuel Bleichenbacher
 * 
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 *
 * Task listening on a UART port for provisioning commands.
 *******************************************************************************/

#ifndef _ttnprovisioning_h_
#define _ttnprovisioning_h_

#include "lmic/oslmic.h"
#include "nvs_flash.h"


class TTNProvisioning
{
public:
    TTNProvisioning();

    bool haveKeys();
    bool decodeKeys(const char *dev_eui, const char *app_eui, const char *app_key);
    bool fromMAC(const char *app_eui, const char *app_key);
    bool saveKeys();
    bool restoreKeys(bool silent);

#if defined(TTN_HAS_AT_COMMANDS)
    void startTask();
#endif

private:
    bool decode(bool incl_dev_eui, const char *dev_eui, const char *app_eui, const char *app_key);
    bool readNvsValue(nvs_handle handle, const char* key, uint8_t* data, size_t expected_length, bool silent);
    bool writeNvsValue(nvs_handle handle, const char* key, const uint8_t* data, size_t len);

#if defined(TTN_HAS_AT_COMMANDS)
    void provisioningTask();
    void addLineData(int numBytes);
    void detectLineEnd(int start_at);
    void processLine();
#endif

#if defined(TTN_CONFIG_UART)
    void configUART();
#endif

    static bool hexStrToBin(const char *hex, uint8_t *buf, int len);
    static int hexTupleToByte(const char *hex);
    static int hexDigitToVal(char ch);
    static void binToHexStr(const uint8_t* buf, int len, char* hex);
    static char valToHexDigit(int val);
    static void swapBytes(uint8_t* buf, int len);
    static bool isAllZeros(const uint8_t* buf, int len);

private:
    bool have_keys = false;

#if defined(TTN_HAS_AT_COMMANDS)
    QueueHandle_t uart_queue;
    char* line_buf;
    int line_length;
    uint8_t last_line_end_char;
    bool quit_task;

    friend void ttn_provisioning_task_caller(void* pvParameter);
#endif
};

#endif
