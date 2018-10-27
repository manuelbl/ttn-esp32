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

#include "freertos/FreeRTOS.h"
#include "driver/uart.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "TTNProvisioning.h"
#include "lmic/lmic.h"
#include "hal/hal_esp32.h"

#if defined(TTN_HAS_AT_COMMANDS)
const uart_port_t UART_NUM = (uart_port_t) CONFIG_TTN_PROVISION_UART_NUM;
const int MAX_LINE_LENGTH = 128;
#endif

static const char* const TAG = "ttn_prov";
static const char* const NVS_FLASH_PARTITION = "ttn";
static const char* const NVS_FLASH_KEY_DEV_EUI = "devEui";
static const char* const NVS_FLASH_KEY_APP_EUI = "appEui";
static const char* const NVS_FLASH_KEY_APP_KEY = "appKey";

static uint8_t global_dev_eui[8];
static uint8_t global_app_eui[8];
static uint8_t global_app_key[16];


#if defined(TTN_HAS_AT_COMMANDS)
void ttn_provisioning_task_caller(void* pvParameter);
#endif


// --- LMIC callbacks

// This EUI must be in little-endian format, so least-significant-byte first.
// When copying an EUI from ttnctl output, this means to reverse the bytes.
// For TTN issued EUIs the last bytes should be 0xD5, 0xB3, 0x70.
// The order is swapped in provisioning_decode_keys().
void os_getArtEui (u1_t* buf)
{
    memcpy(buf, global_app_eui, 8);
}

// This should also be in little endian format, see above.
void os_getDevEui (u1_t* buf)
{
    memcpy(buf, global_dev_eui, 8);
}

// This key should be in big endian format (or, since it is not really a number
// but a block of memory, endianness does not really apply). In practice, a key
// taken from ttnctl can be copied as-is.
void os_getDevKey (u1_t* buf)
{
    memcpy(buf, global_app_key, 16);
}

// --- Constructor

TTNProvisioning::TTNProvisioning()
    : have_keys(false)
#if defined(TTN_HAS_AT_COMMANDS)
        , uart_queue(nullptr), line_buf(nullptr), line_length(0), last_line_end_char(0), quit_task(false)
#endif
{
}


// --- Provisioning task

#if defined(TTN_HAS_AT_COMMANDS)

void TTNProvisioning::startTask()
{
#if defined(TTN_CONFIG_UART)
    configUART();
#endif

    esp_err_t err = uart_driver_install(UART_NUM, 2048, 2048, 20, &uart_queue, 0);
    ESP_ERROR_CHECK(err);

    xTaskCreate(ttn_provisioning_task_caller, "provisioning", 2048, this, 1, nullptr);
}

void ttn_provisioning_task_caller(void* pvParameter)
{
    TTNProvisioning* provisioning = (TTNProvisioning*)pvParameter;
    provisioning->provisioningTask();
}

void TTNProvisioning::provisioningTask()
{
    line_buf = (char*)malloc(MAX_LINE_LENGTH + 1);
    line_length = 0;

    uart_event_t event;

    ESP_LOGI(TAG, "Provisioning task started");

    while (!quit_task)
    {
        if (!xQueueReceive(uart_queue, &event, portMAX_DELAY))
            continue;

        switch (event.type)
        {
            case UART_DATA:
                addLineData(event.size);
                break;

            case UART_FIFO_OVF:
            case UART_BUFFER_FULL:
                uart_flush_input(UART_NUM);
                xQueueReset(uart_queue);
                break;

            default:
                break;
        }
    }

    free(line_buf);
    uart_driver_delete(UART_NUM);
    vTaskDelete(nullptr);
}

void TTNProvisioning::addLineData(int numBytes)
{
    int n;
top:
    n = numBytes;
    if (line_length + n > MAX_LINE_LENGTH)
        n = MAX_LINE_LENGTH - line_length;
    
    uart_read_bytes(UART_NUM, (uint8_t*)line_buf + line_length, n, portMAX_DELAY);
    int start_at = line_length;
    line_length += n;

    detectLineEnd(start_at);

    if (n < numBytes)
    {
        numBytes -= n;
        goto top;
    }
}

void TTNProvisioning::detectLineEnd(int start_at)
{
top:
    for (int p = start_at; p < line_length; p++)
    {
        char ch = line_buf[p];
        if (ch == 0x0d || ch == 0x0a)
        {
            if (p > 0)
                uart_write_bytes(UART_NUM, line_buf + start_at, line_length - start_at - 1);
            if (p > 0 || ch == 0x0d || last_line_end_char == 0x0a)
                uart_write_bytes(UART_NUM, "\r\n", 2);

            line_buf[p] = 0;
            last_line_end_char = ch;

            if (p > 0)
                processLine();

            memcpy(line_buf, line_buf + p + 1, line_length - p - 1);
            line_length -= p + 1;
            start_at = 0;
            goto top;
        }
    }

    if (line_length > 0)
        uart_write_bytes(UART_NUM, line_buf + start_at, line_length - start_at);

    if (line_length == MAX_LINE_LENGTH)
        line_length = 0; // Line too long; flush it
}

void TTNProvisioning::processLine()
{
    bool is_ok = true;
    bool reset_needed = false;

    // Expected format:
    // AT+PROV?
    // AT+PROV=hex16-hex16-hex32
    // AT+PROVM=hex16-hex32
    // AT+MAC?
    // AT+HWEUI?

    if (strcmp(line_buf, "AT+PROV?") == 0)
    {
        uint8_t binbuf[8];
        char hexbuf[16];

        memcpy(binbuf, global_dev_eui, 8);
        swapBytes(binbuf, 8);
        binToHexStr(binbuf, 8, hexbuf);
        uart_write_bytes(UART_NUM, hexbuf, 16);
        uart_write_bytes(UART_NUM, "-", 1);

        memcpy(binbuf, global_app_eui, 8);
        swapBytes(binbuf, 8);
        binToHexStr(binbuf, 8, hexbuf);
        uart_write_bytes(UART_NUM, hexbuf, 16);

        uart_write_bytes(UART_NUM, "-00000000000000000000000000000000\r\n", 35);
    }
    else if (strncmp(line_buf, "AT+PROV=", 8) == 0)
    {
        is_ok  = strlen(line_buf) == 74 && line_buf[24] == '-' && line_buf[41] == '-';
        if (is_ok)
        {
            line_buf[24] = 0;
            line_buf[41] = 0;
            is_ok = decodeKeys(line_buf + 8, line_buf + 25, line_buf + 42);
            reset_needed = is_ok;
        }
    }
    else if (strncmp(line_buf, "AT+PROVM=", 8) == 0)
    {
        is_ok = strlen(line_buf) == 58 && line_buf[25] == '-';
        if (is_ok)
        {
            line_buf[25] = 0;
            is_ok = fromMAC(line_buf + 9, line_buf + 26);
            reset_needed = is_ok;
        }
    }
    else if (strcmp(line_buf, "AT+MAC?") == 0)
    {
        uint8_t mac[6];
        char hexbuf[12];

        esp_err_t err = esp_efuse_mac_get_default(mac);
        ESP_ERROR_CHECK(err);

        binToHexStr(mac, 6, hexbuf);
        for (int i = 0; i < 12; i += 2) {
            if (i > 0)
                uart_write_bytes(UART_NUM, ":", 1);
            uart_write_bytes(UART_NUM, hexbuf + i, 2);
        }
        uart_write_bytes(UART_NUM, "\r\n", 2);
    }
    else if (strcmp(line_buf, "AT+HWEUI?") == 0)
    {
        uint8_t mac[6];
        char hexbuf[12];

        esp_err_t err = esp_efuse_mac_get_default(mac);
        ESP_ERROR_CHECK(err);

        binToHexStr(mac, 6, hexbuf);
        for (int i = 0; i < 12; i += 2) {
            uart_write_bytes(UART_NUM, hexbuf + i, 2);
            if (i == 4)
              uart_write_bytes(UART_NUM, "FFFE", 4);
        }
        uart_write_bytes(UART_NUM, "\r\n", 2);
    }
    else if (strcmp(line_buf, "AT+PROVQ") == 0)
    {
        quit_task = true;
    }
    else if (strcmp(line_buf, "AT") != 0)
    {
        is_ok = false;
    }

    if (reset_needed)
    {
        ttn_hal.enterCriticalSection();
        LMIC_reset();
        ttn_hal.leaveCriticalSection();
        onEvent(EV_RESET);
    }

    uart_write_bytes(UART_NUM, is_ok ? "OK\r\n" : "ERROR\r\n", is_ok ? 4 : 7);
}

#endif


#if defined(TTN_CONFIG_UART)

void TTNProvisioning::configUART()
{
    esp_err_t err;

    uart_config_t uart_config = {
        .baud_rate = CONFIG_TTN_PROVISION_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .use_ref_tick = false
    };
    err = uart_param_config(UART_NUM, &uart_config);
    ESP_ERROR_CHECK(err);

    err = uart_set_pin(UART_NUM, CONFIG_TTN_PROVISION_UART_TX_GPIO, CONFIG_TTN_PROVISION_UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ESP_ERROR_CHECK(err);
}

#endif


// --- Key handling

bool TTNProvisioning::haveKeys()
{
    return have_keys;
}

bool TTNProvisioning::decodeKeys(const char *dev_eui, const char *app_eui, const char *app_key)
{
    return decode(true, dev_eui, app_eui, app_key);
}

bool TTNProvisioning::fromMAC(const char *app_eui, const char *app_key)
{
    uint8_t mac[6];
    esp_err_t err = esp_efuse_mac_get_default(mac);
    ESP_ERROR_CHECK(err);
    
    global_dev_eui[7] = mac[0];
    global_dev_eui[6] = mac[1];
    global_dev_eui[5] = mac[2];
    global_dev_eui[4] = 0xff;
    global_dev_eui[3] = 0xfe;
    global_dev_eui[2] = mac[3];
    global_dev_eui[1] = mac[4];
    global_dev_eui[0] = mac[5];

    return decode(false, nullptr, app_eui, app_key);
}

bool TTNProvisioning::decode(bool incl_dev_eui, const char *dev_eui, const char *app_eui, const char *app_key)
{
    uint8_t buf_dev_eui[8];
    uint8_t buf_app_eui[8];
    uint8_t buf_app_key[16];

    if (incl_dev_eui && (strlen(dev_eui) != 16 || !hexStrToBin(dev_eui, buf_dev_eui, 8)))
    {
        ESP_LOGW(TAG, "Invalid device EUI: %s", dev_eui);
        return false;
    }

    if (incl_dev_eui)
        swapBytes(buf_dev_eui, 8);

    if (strlen(app_eui) != 16 || !hexStrToBin(app_eui, buf_app_eui, 8))
    {
        ESP_LOGW(TAG, "Invalid application EUI: %s", app_eui);
        return false;
    }

    swapBytes(buf_app_eui, 8);

    if (strlen(app_key) != 32 || !hexStrToBin(app_key, buf_app_key, 16))
    {
        ESP_LOGW(TAG, "Invalid application key: %s", app_key);
        return false;
    }

    if (incl_dev_eui)
        memcpy(global_dev_eui, buf_dev_eui, sizeof(global_dev_eui));
    memcpy(global_app_eui, buf_app_eui, sizeof(global_app_eui));
    memcpy(global_app_key, buf_app_key, sizeof(global_app_key));

    have_keys = !isAllZeros(global_dev_eui, sizeof(global_dev_eui))
        && !isAllZeros(global_app_eui, sizeof(global_app_eui))
        && !isAllZeros(global_app_key, sizeof(global_app_key));

    if (!saveKeys())
        return false;
    
    return true;
}


// --- Non-volatile storage

bool TTNProvisioning::saveKeys()
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

    if (!writeNvsValue(handle, NVS_FLASH_KEY_DEV_EUI, global_dev_eui, sizeof(global_dev_eui)))
        goto done;
        
    if (!writeNvsValue(handle, NVS_FLASH_KEY_APP_EUI, global_app_eui, sizeof(global_app_eui)))
        goto done;
        
    if (!writeNvsValue(handle, NVS_FLASH_KEY_APP_KEY, global_app_key, sizeof(global_app_key)))
        goto done;

    res = nvs_commit(handle);
    ESP_ERROR_CHECK(res);
    
    result = true;
    ESP_LOGI(TAG, "Dev and app EUI and app key saved in NVS storage");

done:
    nvs_close(handle);
    return result;
}

bool TTNProvisioning::restoreKeys(bool silent)
{
    uint8_t buf_dev_eui[8];
    uint8_t buf_app_eui[8];
    uint8_t buf_app_key[16];
    
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

    if (!readNvsValue(handle, NVS_FLASH_KEY_DEV_EUI, buf_dev_eui, sizeof(global_dev_eui), silent))
        goto done;

    if (!readNvsValue(handle, NVS_FLASH_KEY_APP_EUI, buf_app_eui, sizeof(global_app_eui), silent))
        goto done;

    if (!readNvsValue(handle, NVS_FLASH_KEY_APP_KEY, buf_app_key, sizeof(global_app_key), silent))
        goto done;

    memcpy(global_dev_eui, buf_dev_eui, sizeof(global_dev_eui));
    memcpy(global_app_eui, buf_app_eui, sizeof(global_app_eui));
    memcpy(global_app_key, buf_app_key, sizeof(global_app_key));

    have_keys = !isAllZeros(global_dev_eui, sizeof(global_dev_eui))
        && !isAllZeros(global_app_eui, sizeof(global_app_eui))
        && !isAllZeros(global_app_key, sizeof(global_app_key));

    if (have_keys)
    {
       ESP_LOGI(TAG, "Dev and app EUI and app key have been restored from NVS storage");
    }
    else
    {
        ESP_LOGW(TAG, "Dev and app EUI and app key are invalid (zeroes only)");
    }

done:
    nvs_close(handle);
    return true;
}

bool TTNProvisioning::readNvsValue(nvs_handle handle, const char* key, uint8_t* data, size_t expected_length, bool silent)
{
    size_t size = expected_length;
    esp_err_t res = nvs_get_blob(handle, key, data, &size);
    if (res == ESP_OK && size == expected_length)
        return true;

    if (res == ESP_OK && size != expected_length)
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

bool TTNProvisioning::writeNvsValue(nvs_handle handle, const char* key, const uint8_t* data, size_t len)
{
    uint8_t buf[16];
    if (readNvsValue(handle, key, buf, len, true) && memcmp(buf, data, len) == 0)
        return true; // unchanged
    
    esp_err_t res = nvs_set_blob(handle, key, data, len);
    ESP_ERROR_CHECK(res);

    return res == ESP_OK;
}


// --- Helper functions ---

bool TTNProvisioning::hexStrToBin(const char *hex, uint8_t *buf, int len)
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

int TTNProvisioning::hexTupleToByte(const char *hex)
{
    int nibble1 = hexDigitToVal(hex[0]);
    if (nibble1 < 0)
        return -1;
    int nibble2 = hexDigitToVal(hex[1]);
    if (nibble2 < 0)
        return -1;
    return (nibble1 << 4) | nibble2;
}

int TTNProvisioning::hexDigitToVal(char ch)
{
    if (ch >= '0' && ch <= '9')
        return ch - '0';
    if (ch >= 'A' && ch <= 'F')
        return ch + 10 - 'A';
    if (ch >= 'a' && ch <= 'f')
        return ch + 10 - 'a';
    return -1;
}

void TTNProvisioning::binToHexStr(const uint8_t* buf, int len, char* hex)
{
    for (int i = 0; i < len; i++)
    {
        uint8_t b = buf[i];
        *hex = valToHexDigit((b & 0xf0) >> 4);
        hex++;
        *hex = valToHexDigit(b & 0x0f);
        hex++;
    }
}

char TTNProvisioning::valToHexDigit(int val)
{
    return "0123456789ABCDEF"[val];
}

void TTNProvisioning::swapBytes(uint8_t* buf, int len)
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

bool TTNProvisioning::isAllZeros(const uint8_t* buf, int len)
{
    for (int i = 0; i < len; i++)
        if (buf[i] != 0)
            return false;
    return true;
}