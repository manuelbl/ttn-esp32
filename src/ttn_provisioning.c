/*******************************************************************************
 *
 * ttn-esp32 - The Things Network device library for ESP-IDF / SX127x
 *
 * Copyright (c) 2018-2021 Manuel Bleichenbacher
 *
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 *
 * Task listening on a UART port for provisioning commands.
 *******************************************************************************/

#include "ttn_provisioning.h"
#include "driver/uart.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "hal/hal_esp32.h"
#include "lmic/lmic.h"
#include "nvs_flash.h"

#if defined(TTN_HAS_AT_COMMANDS)
#define UART_NUM CONFIG_TTN_PROVISION_UART_NUM
#define MAX_LINE_LENGTH 128
#endif

#define TAG "ttn_prov"
#define NVS_FLASH_PARTITION "ttn"
#define NVS_FLASH_KEY_DEV_EUI "devEui"
#define NVS_FLASH_KEY_APP_EUI "appEui"
#define NVS_FLASH_KEY_APP_KEY "appKey"

#if defined(TTN_HAS_AT_COMMANDS)
static void provisioning_task(void *pvParameter);
static void add_line_data(int num_bytes);
static void detect_line_end(int start_at);
static void process_line(void);
#endif

#if defined(TTN_CONFIG_UART)
static void config_uart(void);
#endif

static bool decode(bool incl_dev_eui, const char *dev_eui, const char *app_eui, const char *app_key);
static bool read_nvs_value(nvs_handle handle, const char *key, uint8_t *data, size_t expected_length, bool silent);
static bool write_nvs_value(nvs_handle handle, const char *key, const uint8_t *data, size_t len);

static bool hex_str_to_bin(const char *hex, uint8_t *buf, int len);
static int hex_tuple_to_byte(const char *hex);
static int hex_digit_to_val(char ch);
static void bin_to_hex_str(const uint8_t *buf, int len, char *hex);
static char val_to_hex_digit(int val);
static void swap_bytes(uint8_t *buf, int len);
static bool is_all_zeros(const uint8_t *buf, int len);

static uint8_t global_dev_eui[8];
static uint8_t global_app_eui[8];
static uint8_t global_app_key[16];

static bool have_keys = false;

#if defined(TTN_HAS_AT_COMMANDS)
static QueueHandle_t uart_queue;
static char *line_buf;
static int line_length;
static uint8_t last_line_end_char;
static bool quit_task;
#endif

// --- LMIC callbacks

// This EUI must be in little-endian format, so least-significant-byte first.
// When copying an EUI from ttnctl output, this means to reverse the bytes.
// For TTN issued EUIs the last bytes should be 0xD5, 0xB3, 0x70.
// The order is swapped in provisioning_decode_keys().
void os_getArtEui(u1_t *buf)
{
    memcpy(buf, global_app_eui, 8);
}

// This should also be in little endian format, see above.
void os_getDevEui(u1_t *buf)
{
    memcpy(buf, global_dev_eui, 8);
}

// This key should be in big endian format (or, since it is not really a number
// but a block of memory, endianness does not really apply). In practice, a key
// taken from ttnctl can be copied as-is.
void os_getDevKey(u1_t *buf)
{
    memcpy(buf, global_app_key, 16);
}

// --- Constructor

void ttn_provisioning_init(void)
{
}

// --- Provisioning task

#if defined(TTN_HAS_AT_COMMANDS)

void ttn_provisioning_start_task(void)
{
#if defined(TTN_CONFIG_UART)
    config_uart();
#endif

    esp_err_t err = uart_driver_install(UART_NUM, 2048, 2048, 20, &uart_queue, 0);
    ESP_ERROR_CHECK(err);

    xTaskCreate(provisioning_task, "ttn_provision", 2048, NULL, 1, NULL);
}

void provisioning_task(void *pvParameter)
{
    line_buf = (char *)malloc(MAX_LINE_LENGTH + 1);
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
            add_line_data(event.size);
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
    vTaskDelete(NULL);
}

void add_line_data(int num_bytes)
{
    int n;
top:
    n = num_bytes;
    if (line_length + n > MAX_LINE_LENGTH)
        n = MAX_LINE_LENGTH - line_length;

    uart_read_bytes(UART_NUM, (uint8_t *)line_buf + line_length, n, portMAX_DELAY);
    int start_at = line_length;
    line_length += n;

    detect_line_end(start_at);

    if (n < num_bytes)
    {
        num_bytes -= n;
        goto top;
    }
}

void detect_line_end(int start_at)
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
                process_line();

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

void process_line(void)
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
        swap_bytes(binbuf, 8);
        bin_to_hex_str(binbuf, 8, hexbuf);
        uart_write_bytes(UART_NUM, hexbuf, 16);
        uart_write_bytes(UART_NUM, "-", 1);

        memcpy(binbuf, global_app_eui, 8);
        swap_bytes(binbuf, 8);
        bin_to_hex_str(binbuf, 8, hexbuf);
        uart_write_bytes(UART_NUM, hexbuf, 16);

        uart_write_bytes(UART_NUM, "-00000000000000000000000000000000\r\n", 35);
    }
    else if (strncmp(line_buf, "AT+PROV=", 8) == 0)
    {
        is_ok = strlen(line_buf) == 74 && line_buf[24] == '-' && line_buf[41] == '-';
        if (is_ok)
        {
            line_buf[24] = 0;
            line_buf[41] = 0;
            is_ok = ttn_provisioning_decode_keys(line_buf + 8, line_buf + 25, line_buf + 42);
            if (is_ok)
                is_ok = ttn_provisioning_save_keys();
            reset_needed = is_ok;
        }
    }
    else if (strncmp(line_buf, "AT+PROVM=", 8) == 0)
    {
        is_ok = strlen(line_buf) == 58 && line_buf[25] == '-';
        if (is_ok)
        {
            line_buf[25] = 0;
            is_ok = ttn_provisioning_from_mac(line_buf + 9, line_buf + 26);
            if (is_ok)
                is_ok = ttn_provisioning_save_keys();
            reset_needed = is_ok;
        }
    }
    else if (strcmp(line_buf, "AT+MAC?") == 0)
    {
        uint8_t mac[6];
        char hexbuf[12];

        esp_err_t err = esp_efuse_mac_get_default(mac);
        ESP_ERROR_CHECK(err);

        bin_to_hex_str(mac, 6, hexbuf);
        for (int i = 0; i < 12; i += 2)
        {
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

        bin_to_hex_str(mac, 6, hexbuf);
        for (int i = 0; i < 12; i += 2)
        {
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
        hal_esp32_enter_critical_section();
        LMIC_reset();
        hal_esp32_leave_critical_section();
        LMIC.client.eventCb(LMIC.client.eventUserData, EV_RESET);
    }

    uart_write_bytes(UART_NUM, is_ok ? "OK\r\n" : "ERROR\r\n", is_ok ? 4 : 7);
}

#endif

#if defined(TTN_CONFIG_UART)

void config_uart(void)
{
    esp_err_t err;

    uart_config_t uart_config = {.baud_rate = CONFIG_TTN_PROVISION_UART_BAUDRATE,
                                 .data_bits = UART_DATA_8_BITS,
                                 .parity = UART_PARITY_DISABLE,
                                 .stop_bits = UART_STOP_BITS_1,
                                 .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    err = uart_param_config(UART_NUM, &uart_config);
    ESP_ERROR_CHECK(err);

    err = uart_set_pin(UART_NUM, CONFIG_TTN_PROVISION_UART_TX_GPIO, CONFIG_TTN_PROVISION_UART_RX_GPIO,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ESP_ERROR_CHECK(err);
}

#endif

// --- Key handling

bool ttn_provisioning_have_keys(void)
{
    return have_keys;
}

bool ttn_provisioning_decode_keys(const char *dev_eui, const char *app_eui, const char *app_key)
{
    return decode(true, dev_eui, app_eui, app_key);
}

bool ttn_provisioning_from_mac(const char *app_eui, const char *app_key)
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

    return decode(false, NULL, app_eui, app_key);
}

bool decode(bool incl_dev_eui, const char *dev_eui, const char *app_eui, const char *app_key)
{
    uint8_t buf_dev_eui[8];
    uint8_t buf_app_eui[8];
    uint8_t buf_app_key[16];

    if (incl_dev_eui && (strlen(dev_eui) != 16 || !hex_str_to_bin(dev_eui, buf_dev_eui, 8)))
    {
        ESP_LOGW(TAG, "Invalid DevEUI: %s", dev_eui);
        return false;
    }

    if (incl_dev_eui)
        swap_bytes(buf_dev_eui, 8);

    if (strlen(app_eui) != 16 || !hex_str_to_bin(app_eui, buf_app_eui, 8))
    {
        ESP_LOGW(TAG, "Invalid AppEUI/JoinEUI: %s", app_eui);
        return false;
    }

    swap_bytes(buf_app_eui, 8);

    if (strlen(app_key) != 32 || !hex_str_to_bin(app_key, buf_app_key, 16))
    {
        ESP_LOGW(TAG, "Invalid application key: %s", app_key);
        return false;
    }

    if (incl_dev_eui)
        memcpy(global_dev_eui, buf_dev_eui, sizeof(global_dev_eui));
    memcpy(global_app_eui, buf_app_eui, sizeof(global_app_eui));
    memcpy(global_app_key, buf_app_key, sizeof(global_app_key));

    have_keys =
        !is_all_zeros(global_dev_eui, sizeof(global_dev_eui)) && !is_all_zeros(global_app_key, sizeof(global_app_key));

    return true;
}

// --- Non-volatile storage

bool ttn_provisioning_save_keys()
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

    if (!write_nvs_value(handle, NVS_FLASH_KEY_DEV_EUI, global_dev_eui, sizeof(global_dev_eui)))
        goto done;

    if (!write_nvs_value(handle, NVS_FLASH_KEY_APP_EUI, global_app_eui, sizeof(global_app_eui)))
        goto done;

    if (!write_nvs_value(handle, NVS_FLASH_KEY_APP_KEY, global_app_key, sizeof(global_app_key)))
        goto done;

    res = nvs_commit(handle);
    ESP_ERROR_CHECK(res);

    result = true;
    ESP_LOGI(TAG, "DevEUI, AppEUI/JoinEUI and AppKey saved in NVS storage");

done:
    nvs_close(handle);
    return result;
}

bool ttn_provisioning_restore_keys(bool silent)
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

    if (!read_nvs_value(handle, NVS_FLASH_KEY_DEV_EUI, buf_dev_eui, sizeof(global_dev_eui), silent))
        goto done;

    if (!read_nvs_value(handle, NVS_FLASH_KEY_APP_EUI, buf_app_eui, sizeof(global_app_eui), silent))
        goto done;

    if (!read_nvs_value(handle, NVS_FLASH_KEY_APP_KEY, buf_app_key, sizeof(global_app_key), silent))
        goto done;

    memcpy(global_dev_eui, buf_dev_eui, sizeof(global_dev_eui));
    memcpy(global_app_eui, buf_app_eui, sizeof(global_app_eui));
    memcpy(global_app_key, buf_app_key, sizeof(global_app_key));

    have_keys =
        !is_all_zeros(global_dev_eui, sizeof(global_dev_eui)) && !is_all_zeros(global_app_key, sizeof(global_app_key));

    if (have_keys)
    {
        ESP_LOGI(TAG, "DevEUI, AppEUI/JoinEUI and AppKey have been restored from NVS storage");
    }
    else
    {
        ESP_LOGW(TAG, "DevEUI or DevEUI is invalid (zeroes only)");
    }

done:
    nvs_close(handle);
    return true;
}

bool read_nvs_value(nvs_handle handle, const char *key, uint8_t *data, size_t expected_length, bool silent)
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

bool write_nvs_value(nvs_handle handle, const char *key, const uint8_t *data, size_t len)
{
    uint8_t buf[16];
    if (read_nvs_value(handle, key, buf, len, true) && memcmp(buf, data, len) == 0)
        return true; // unchanged

    esp_err_t res = nvs_set_blob(handle, key, data, len);
    ESP_ERROR_CHECK(res);

    return res == ESP_OK;
}

// --- Helper functions ---

bool hex_str_to_bin(const char *hex, uint8_t *buf, int len)
{
    const char *ptr = hex;
    for (int i = 0; i < len; i++)
    {
        int val = hex_tuple_to_byte(ptr);
        if (val < 0)
            return false;
        buf[i] = val;
        ptr += 2;
    }
    return true;
}

int hex_tuple_to_byte(const char *hex)
{
    int nibble1 = hex_digit_to_val(hex[0]);
    if (nibble1 < 0)
        return -1;
    int nibble2 = hex_digit_to_val(hex[1]);
    if (nibble2 < 0)
        return -1;
    return (nibble1 << 4) | nibble2;
}

int hex_digit_to_val(char ch)
{
    if (ch >= '0' && ch <= '9')
        return ch - '0';
    if (ch >= 'A' && ch <= 'F')
        return ch + 10 - 'A';
    if (ch >= 'a' && ch <= 'f')
        return ch + 10 - 'a';
    return -1;
}

void bin_to_hex_str(const uint8_t *buf, int len, char *hex)
{
    for (int i = 0; i < len; i++)
    {
        uint8_t b = buf[i];
        *hex = val_to_hex_digit((b & 0xf0) >> 4);
        hex++;
        *hex = val_to_hex_digit(b & 0x0f);
        hex++;
    }
}

char val_to_hex_digit(int val)
{
    return "0123456789ABCDEF"[val];
}

void swap_bytes(uint8_t *buf, int len)
{
    uint8_t *p1 = buf;
    uint8_t *p2 = buf + len - 1;
    while (p1 < p2)
    {
        uint8_t t = *p1;
        *p1 = *p2;
        *p2 = t;
        p1++;
        p2--;
    }
}

bool is_all_zeros(const uint8_t *buf, int len)
{
    for (int i = 0; i < len; i++)
        if (buf[i] != 0)
            return false;
    return true;
}