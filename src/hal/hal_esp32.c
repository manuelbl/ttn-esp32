/*******************************************************************************
 * 
 * ttn-esp32 - The Things Network device library for ESP-IDF / SX127x
 * 
 * Copyright (c) 2018-2019 Manuel Bleichenbacher
 * 
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 *
 * Hardware abstraction layer to run LMIC on a ESP32 using ESP-IDF.
 *******************************************************************************/

#include "hal_esp32.h"
#include "../lmic/lmic.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <time.h>
#include <sys/time.h>

#define LMIC_UNUSED_PIN 0xff

#define NOTIFY_BIT_DIO 1
#define NOTIFY_BIT_TIMER 2
#define NOTIFY_BIT_WAKEUP 4
#define NOTIFY_BIT_STOP 8


#define TAG "ttn_hal"


typedef enum {
    WAIT_KIND_NONE = 0,
    WAIT_KIND_CHECK_IO,
    WAIT_KIND_WAIT_FOR_ANY_EVENT,
    WAIT_KIND_WAIT_FOR_TIMER
} wait_kind_e;


static void lmic_background_task(void* pvParameter);
static void qio_irq_handler(void* arg);
static void timer_callback(void *arg);
static int64_t os_time_to_esp_time(int64_t esp_now, uint32_t os_time);
static int64_t get_current_time();
static void init_io(void);
static void init_spi(void);
static void assert_nss(spi_transaction_t* trans);
static void deassert_nss(spi_transaction_t* trans);
static void init_timer(void);

static void set_next_alarm(int64_t time);
static void arm_timer(int64_t esp_now);
static void disarm_timer(void);
static bool wait(wait_kind_e wait_kind);

static spi_host_device_t spi_host;
static gpio_num_t pin_nss;
static gpio_num_t pin_rx_tx;
static gpio_num_t pin_rst;
static gpio_num_t pin_dio0;
static gpio_num_t pin_dio1;
static int8_t rssi_cal = 10;

static TaskHandle_t lmic_task;
static uint32_t dio_interrupt_time;
static uint8_t dio_num;

static spi_device_handle_t spi_handle;
static spi_transaction_t spi_transaction;
static SemaphoreHandle_t mutex;
static esp_timer_handle_t timer;
static int64_t time_offset;
static int32_t initial_time_offset;
static int64_t next_alarm;
static volatile bool run_background_task;
static volatile wait_kind_e current_wait_kind;


// -----------------------------------------------------------------------------
// I/O

void hal_esp32_configure_pins(spi_host_device_t host, uint8_t nss, uint8_t rxtx, uint8_t rst, uint8_t dio0, uint8_t dio1)
{
    spi_host = host;
    pin_nss = (gpio_num_t)nss;
    pin_rx_tx = (gpio_num_t)rxtx;
    pin_rst = (gpio_num_t)rst;
    pin_dio0 = (gpio_num_t)dio0;
    pin_dio1 = (gpio_num_t)dio1;

    // Until the background process has been started, use the current task
    // for supporting calls like `hal_waitUntil()`.
    lmic_task = xTaskGetCurrentTaskHandle();
}


void IRAM_ATTR qio_irq_handler(void *arg)
{
    dio_interrupt_time = hal_ticks();
    dio_num = (u1_t)(long)arg;
    BaseType_t higher_prio_task_woken = pdFALSE;
    xTaskNotifyFromISR(lmic_task, NOTIFY_BIT_DIO, eSetBits, &higher_prio_task_woken);
    if (higher_prio_task_woken)
        portYIELD_FROM_ISR();
}

void init_io(void)
{
    // pin_nss, pin_dio0 and pin_dio1 are required
    ASSERT(pin_nss != LMIC_UNUSED_PIN);
    ASSERT(pin_dio0 != LMIC_UNUSED_PIN);
    ASSERT(pin_dio1 != LMIC_UNUSED_PIN);

    gpio_config_t output_pin_config = {
        .pin_bit_mask = BIT64(pin_nss),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = false,
        .pull_down_en = false,
        .intr_type = GPIO_INTR_DISABLE
    };

    if (pin_rx_tx != LMIC_UNUSED_PIN)
        output_pin_config.pin_bit_mask |= BIT64(pin_rx_tx);

    if (pin_rst != LMIC_UNUSED_PIN)
        output_pin_config.pin_bit_mask |= BIT64(pin_rst);

    gpio_config(&output_pin_config);

    gpio_set_level(pin_nss, 1);
    if (pin_rx_tx != LMIC_UNUSED_PIN)
        gpio_set_level(pin_rx_tx, 0);
    if (pin_rst != LMIC_UNUSED_PIN)
        gpio_set_level(pin_rst, 0);

    // DIO pins with interrupt handlers
    gpio_config_t input_pin_config = {
        .pin_bit_mask = BIT64(pin_dio0) | BIT64(pin_dio1),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = false,
        .pull_down_en = true,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    gpio_config(&input_pin_config);

    ESP_LOGI(TAG, "IO initialized");
}

__attribute__((weak)) // duplicate this symbol if your pin is controlled by e.g. I2C
void hal_pin_rxtx(u1_t val)
{
    if (pin_rx_tx == LMIC_UNUSED_PIN)
        return;
    
    gpio_set_level(pin_rx_tx, val);
}

__attribute__((weak)) // duplicate this symbol if your pin is controlled by e.g. I2C
void hal_pin_rst(u1_t val)
{
    if (pin_rst == LMIC_UNUSED_PIN)
        return;

    if (val == 0 || val == 1)
    {
        // drive pin
        gpio_set_level(pin_rst, val);
        gpio_set_direction(pin_rst, GPIO_MODE_OUTPUT);
    }
    else
    {
#if defined(CONFIG_TTN_RESET_STATES_ASSERTED)
        // drive up the pin because the hardware is nonstandard
        gpio_set_level(pin_rst, 1);
        gpio_set_direction(pin_rst, GPIO_MODE_OUTPUT);
#else
        // keep pin floating
        gpio_set_direction(pin_rst, GPIO_MODE_INPUT);
#endif
    }
}

s1_t hal_getRssiCal(void)
{
    return rssi_cal;
}

ostime_t hal_setModuleActive(bit_t val)
{
    return 0;
}

bit_t hal_queryUsingTcxo(void)
{
    return false;
}

uint8_t hal_getTxPowerPolicy(u1_t inputPolicy, s1_t requestedPower, u4_t frequency)
{
    return LMICHAL_radio_tx_power_policy_paboost;
}



// -----------------------------------------------------------------------------
// SPI

void init_spi(void)
{
    // init device
    spi_device_interface_config_t spi_config = {
        .mode = 0,
        .clock_speed_hz = CONFIG_TTN_SPI_FREQ,
        .command_bits = 0,
        .address_bits = 8,
        .spics_io_num = -1,
        .queue_size = 1,
        .pre_cb = assert_nss,
        .post_cb = deassert_nss,
    };

    esp_err_t ret = spi_bus_add_device(spi_host, &spi_config, &spi_handle);
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "SPI initialized");
}

void hal_spi_write(u1_t cmd, const u1_t *buf, size_t len)
{
    memset(&spi_transaction, 0, sizeof(spi_transaction));
    spi_transaction.addr = cmd;
    spi_transaction.length = 8 * len;
    spi_transaction.tx_buffer = buf;
    esp_err_t err = spi_device_transmit(spi_handle, &spi_transaction);
    ESP_ERROR_CHECK(err);
}

void hal_spi_read(u1_t cmd, u1_t *buf, size_t len)
{
    memset(buf, 0, len);
    memset(&spi_transaction, 0, sizeof(spi_transaction));
    spi_transaction.addr = cmd;
    spi_transaction.length = 8 * len;
    spi_transaction.rxlength = 8 * len;
    spi_transaction.tx_buffer = buf;
    spi_transaction.rx_buffer = buf;
    esp_err_t err = spi_device_transmit(spi_handle, &spi_transaction);
    ESP_ERROR_CHECK(err);
}


void IRAM_ATTR assert_nss(spi_transaction_t* trans)
{
    gpio_set_level(pin_nss, 0);
}

void IRAM_ATTR deassert_nss(spi_transaction_t* trans)
{
    gpio_set_level(pin_nss, 1);
}


// -----------------------------------------------------------------------------
// TIME

/*
 * LIMIC uses a 32 bit time system (ostime_t) counting ticks. In this
 * implementation, each tick is 16µs. It will wrap arounnd every 19 hours.
 * 
 * The ESP32 has a 64 bit timer counting microseconds. It will wrap around
 * every 584,000 years. So we don't need to bother.
 * 
 * The time includes an offset initialized from `gettimeofday()`
 * to ensure the time correctly advances during deep sleep.
 * 
 * Based on this timer, future callbacks can be scheduled. This is used to
 * schedule the next LMIC job.
 */

// Convert LMIC tick time (ostime_t) to ESP absolute time.
// `os_time` is assumed to be somewhere between one hour in the past and
// 18 hours into the future. 
int64_t os_time_to_esp_time(int64_t esp_now, uint32_t os_time)
{
    int64_t esp_time;
    uint32_t os_now = (uint32_t)(esp_now >> 4);

    // unsigned difference:
    // 0x00000000 - 0xefffffff: future (0 to about 18 hours)
    // 0xf0000000 - 0xffffffff: past (about 1 to 0 hours)
    uint32_t os_diff = os_time - os_now;
    if (os_diff < 0xf0000000)
    {
        esp_time = esp_now + (((int64_t)os_diff) << 4);
    }
    else
    {
        os_diff = -os_diff;
        esp_time = esp_now - (((int64_t)os_diff) << 4);
    }

    return esp_time;
}

int64_t IRAM_ATTR get_current_time()
{
    return esp_timer_get_time() + time_offset;
}

void init_timer(void)
{
    esp_timer_create_args_t timer_config = {
        .callback = &timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "lmic_job"
    };
    esp_err_t err = esp_timer_create(&timer_config, &timer);
    ESP_ERROR_CHECK(err);

    struct timeval now;
    gettimeofday(&now, NULL);
    time_offset += (int64_t)now.tv_sec * 1000000;
    initial_time_offset = 0;

    ESP_LOGI(TAG, "Timer initialized");
}

uint32_t hal_esp32_get_time(void)
{
    struct timeval now;
    gettimeofday(&now, NULL);
    return now.tv_sec + initial_time_offset;
}

void hal_esp32_set_time(uint32_t time_val)
{
    initial_time_offset = time_val;
    time_offset = (int64_t)time_val * 1000000;
}

void set_next_alarm(int64_t time)
{
    next_alarm = time;
}

void arm_timer(int64_t esp_now)
{
    if (next_alarm == 0)
        return;
    int64_t timeout = next_alarm - get_current_time();
    if (timeout < 0)
        timeout = 10;
    esp_timer_start_once(timer, timeout);
}

void disarm_timer(void)
{
    esp_timer_stop(timer);
}

void timer_callback(void *arg)
{
    xTaskNotify(lmic_task, NOTIFY_BIT_TIMER, eSetBits);
}

// Wait for the next external event. Either:
// - scheduled timer due to scheduled job or waiting for a given time
// - wake up event from the client code
// - I/O interrupt (DIO0 or DIO1 pin)
bool wait(wait_kind_e wait_kind)
{
    TickType_t ticks_to_wait = wait_kind == WAIT_KIND_CHECK_IO ? 0 : portMAX_DELAY;
    while (true)
    {
        current_wait_kind = wait_kind;
        uint32_t bits = ulTaskNotifyTake(pdTRUE, ticks_to_wait);
        current_wait_kind = WAIT_KIND_NONE;
        if (bits == 0)
            return false;

        if ((bits & NOTIFY_BIT_STOP) != 0)
            return false;

        if ((bits & NOTIFY_BIT_WAKEUP) != 0)
        {
            if (wait_kind != WAIT_KIND_WAIT_FOR_TIMER)
            {
                disarm_timer();
                return true;
            }
        }
        else if ((bits & NOTIFY_BIT_TIMER) != 0)
        {
            disarm_timer();
            set_next_alarm(0);
            if (wait_kind != WAIT_KIND_CHECK_IO)
                return true;
        }
        else // IO interrupt
        {
            if (wait_kind != WAIT_KIND_WAIT_FOR_TIMER)
                disarm_timer();
            hal_esp32_enter_critical_section();
            radio_irq_handler_v2(dio_num, dio_interrupt_time);
            hal_esp32_leave_critical_section();
            if (wait_kind != WAIT_KIND_WAIT_FOR_TIMER)
                return true;
        }
    }
}

TickType_t hal_esp32_get_timer_duration(void)
{
    wait_kind_e wait_kind = current_wait_kind;
    int64_t alarm_time = next_alarm;

    if (wait_kind == WAIT_KIND_NONE || wait_kind == WAIT_KIND_CHECK_IO)
        return 1; // busy, not waiting

    if (alarm_time != 0)
    {
        TickType_t dur = pdMS_TO_TICKS((alarm_time - get_current_time() + 999) / 1000);
        if (dur > pdMS_TO_TICKS(30000))
            dur = pdMS_TO_TICKS(200);
        return dur;
    }


    return 0; // waiting indefinitely
}


// Gets current time in LMIC ticks
u4_t IRAM_ATTR hal_ticks(void)
{
    // LMIC tick unit: 16µs
    // esp_timer unit: 1µs
    return (u4_t)(get_current_time() >> 4);
}

// Wait until the specified time.
// Called if the LMIC code needs to wait for a precise time.
// All other events are ignored and will be served later.
u4_t hal_waitUntil(u4_t time)
{
    int64_t esp_now = get_current_time();
    int64_t esp_time = os_time_to_esp_time(esp_now, time);
    set_next_alarm(esp_time);
    arm_timer(esp_now);
    wait(WAIT_KIND_WAIT_FOR_TIMER);

    u4_t os_now = hal_ticks();
    u4_t diff = os_now - time;
    return diff < 0x80000000U ? diff : 0;
}

// Called by client code to wake up LMIC to do something,
// e.g. send a submitted messages.
void hal_esp32_wake_up(void)
{
    xTaskNotify(lmic_task, NOTIFY_BIT_WAKEUP, eSetBits);
}

// Check if the specified time has been reached or almost reached.
// Otherwise, save it as alarm time.
// LMIC calls this function with the scheduled time of the next job
// in the queue. If the job is not due yet, LMIC will go to sleep.
u1_t hal_checkTimer(uint32_t time)
{
    int64_t esp_now = get_current_time();
    int64_t esp_time = os_time_to_esp_time(esp_now, time);
    int64_t diff = esp_time - esp_now;
    if (diff < 100)
        return 1; // timer has expired or will expire very soon

    set_next_alarm(esp_time);
    return 0;
}

// Go to sleep until next event.
// Called when LMIC is not busy and not job is due to be executed.
void hal_sleep(void)
{
    if (wait(WAIT_KIND_CHECK_IO))
        return;

    arm_timer(get_current_time());
    wait(WAIT_KIND_WAIT_FOR_ANY_EVENT);
}


// -----------------------------------------------------------------------------
// IRQ

void hal_disableIRQs(void)
{
    // nothing to do as interrupt handlers post message to queue
    // and don't access any shared data structures
}

void hal_enableIRQs(void)
{
    // nothing to do as interrupt handlers post message to queue
    // and don't access any shared data structures
}

void hal_processPendingIRQs(void)
{
    // nothing to do as interrupt handlers post message to queue
    // and don't access any shared data structures
}


// -----------------------------------------------------------------------------
// Synchronization between application code and background task

void hal_esp32_init_critical_section(void)
{
    mutex = xSemaphoreCreateRecursiveMutex();
}

void hal_esp32_enter_critical_section(void)
{
    xSemaphoreTakeRecursive(mutex, portMAX_DELAY);
}

void hal_esp32_leave_critical_section(void)
{
    xSemaphoreGiveRecursive(mutex);
}

// -----------------------------------------------------------------------------

void lmic_background_task(void* pvParameter)
{
    while (run_background_task)
        os_runloop_once();
    vTaskDelete(NULL);
}

void hal_init_ex(const void *pContext)
{
    // configure radio I/O and interrupt handler
    init_io();
    // configure radio SPI
    init_spi();
    // configure timer and alarm callback
    init_timer();
}

void hal_esp32_start_lmic_task(void)
{
    run_background_task = true;
    xTaskCreate(lmic_background_task, "ttn_lmic", 1024 * 4, NULL, CONFIG_TTN_BG_TASK_PRIO, &lmic_task);

    // enable interrupts
    gpio_isr_handler_add(pin_dio0, qio_irq_handler, (void *)0);
    gpio_isr_handler_add(pin_dio1, qio_irq_handler, (void *)1);
}

void hal_esp32_stop_lmic_task(void)
{
    run_background_task = false;
    gpio_isr_handler_remove(pin_dio0);
    gpio_isr_handler_remove(pin_dio1);
    disarm_timer();
    set_next_alarm(0);
    xTaskNotify(lmic_task, NOTIFY_BIT_STOP, eSetBits);
    lmic_task = xTaskGetCurrentTaskHandle();
}


// -----------------------------------------------------------------------------
// Fatal failure

static hal_failure_handler_t* custom_hal_failure_handler = NULL;

void hal_set_failure_handler(hal_failure_handler_t* const handler)
{
    custom_hal_failure_handler = handler;
}

void hal_failed(const char *file, u2_t line)
{
    if (custom_hal_failure_handler != NULL)
        (*custom_hal_failure_handler)(file, line);

    ESP_LOGE(TAG, "LMIC failed and stopped: %s:%d", file, line);

    // go to sleep forever
    while (true)
    {
        vTaskDelay(portMAX_DELAY);
    }
}


// -----------------------------------------------------------------------------
// RSSI

void hal_esp32_set_rssi_cal(int8_t cal)
{
    rssi_cal = cal;
}
