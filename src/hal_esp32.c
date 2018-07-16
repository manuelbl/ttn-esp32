/*******************************************************************************
 * Copyright (c) 2018 Manuel Bleichenbacher
 * 
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * Hardware abstraction layer to run LMIC on a ESP32 using ESP-iDF.
 *******************************************************************************/

#include "lmic.h"
#include "hal_esp32.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/timer.h"
#include "esp_log.h"

#define LMIC_UNUSED_PIN 0xff

static const char *TAG = "ttn_hal";

lmic_pinmap lmic_pins;

typedef enum {
    DIO0 = 0,
    DIO1,
    DIO2,
    TIMER,
    WAKEUP
} event_t;

typedef struct {
    ostime_t time;
    event_t ev;
} queue_item_t;

// -----------------------------------------------------------------------------
// I/O

static QueueHandle_t dio_queue;

void IRAM_ATTR dio_irq_handler(void *arg)
{
    uint64_t now;
    timer_get_counter_value(TTN_TIMER_GROUP, TTN_TIMER, &now);
    event_t ev = (long)arg;
    BaseType_t higher_prio_task_woken = pdFALSE;
    queue_item_t item = {
        .time = (ostime_t)now,
        .ev = ev
    };
    xQueueSendFromISR(dio_queue, &item, &higher_prio_task_woken);
    if (higher_prio_task_woken)
        portYIELD_FROM_ISR();
}

static void hal_io_init()
{
    ESP_LOGI(TAG, "Starting IO initialization");

    // NSS and DIO0 and DIO1 are required
    ASSERT(lmic_pins.nss != LMIC_UNUSED_PIN);
    ASSERT(lmic_pins.dio0 != LMIC_UNUSED_PIN);
    ASSERT(lmic_pins.dio1 != LMIC_UNUSED_PIN);

    gpio_pad_select_gpio(lmic_pins.nss);
    gpio_set_level(lmic_pins.nss, 0);
    gpio_set_direction(lmic_pins.nss, GPIO_MODE_OUTPUT);

    if (lmic_pins.rxtx != LMIC_UNUSED_PIN)
    {
        gpio_pad_select_gpio(lmic_pins.rxtx);
        gpio_set_level(lmic_pins.rxtx, 0);
        gpio_set_direction(lmic_pins.rxtx, GPIO_MODE_OUTPUT);
    }

    if (lmic_pins.rst != LMIC_UNUSED_PIN)
    {
        gpio_pad_select_gpio(lmic_pins.rst);
        gpio_set_level(lmic_pins.rst, 0);
        gpio_set_direction(lmic_pins.rst, GPIO_MODE_OUTPUT);
    }

    dio_queue = xQueueCreate(12, sizeof(queue_item_t));
    assert(dio_queue != NULL);

    gpio_pad_select_gpio(lmic_pins.dio0);
    gpio_set_direction(lmic_pins.dio0, GPIO_MODE_INPUT);
    gpio_set_intr_type(lmic_pins.dio0, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(lmic_pins.dio0, dio_irq_handler, (void *)0);

    gpio_pad_select_gpio(lmic_pins.dio1);
    gpio_set_direction(lmic_pins.dio1, GPIO_MODE_INPUT);
    gpio_set_intr_type(lmic_pins.dio1, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(lmic_pins.dio1, dio_irq_handler, (void *)1);

    ESP_LOGI(TAG, "Finished IO initialization");
}

void hal_pin_rxtx(u1_t val)
{
    if (lmic_pins.rxtx == LMIC_UNUSED_PIN)
        return;
    
    gpio_set_level(lmic_pins.rxtx, val);
}

void hal_pin_rst(u1_t val)
{
    if (lmic_pins.rst == LMIC_UNUSED_PIN)
        return;

    if (val == 0 || val == 1)
    { // drive pin
        gpio_set_level(lmic_pins.rst, val);
        gpio_set_direction(lmic_pins.rst, GPIO_MODE_OUTPUT);
    }
    else
    { // keep pin floating
        gpio_set_level(lmic_pins.rst, val);
        gpio_set_direction(lmic_pins.rst, GPIO_MODE_INPUT);
    }
}

// -----------------------------------------------------------------------------
// SPI

#define SPI_QUEUE_SIZE 4
#define SPI_NUM_TRX_SLOTS (SPI_QUEUE_SIZE + 1)

static spi_device_handle_t spi_handle;
static spi_transaction_t spi_trx_queue[SPI_NUM_TRX_SLOTS];
static int spi_trx_queue_head = 0;
static int spi_num_outstanding_trx = 0;

static spi_transaction_t* get_next_spi_trx_desc()
{
    spi_transaction_t* trx = spi_trx_queue + spi_trx_queue_head;
    memset(trx, 0, sizeof(spi_transaction_t));
    return trx;
}

static void collect_spi_result()
{
    int head = spi_trx_queue_head;
    int tail = head - spi_num_outstanding_trx;
    if (tail < 0)
        tail += SPI_NUM_TRX_SLOTS;

    spi_transaction_t* trx;
    esp_err_t err = spi_device_get_trans_result(spi_handle, &trx, 100 / portTICK_PERIOD_MS);
    assert(err == ESP_OK);
    assert(trx == spi_trx_queue + tail);
    spi_num_outstanding_trx--;
}

static void submit_spi_trx()
{
    if (spi_num_outstanding_trx >= SPI_QUEUE_SIZE)
        collect_spi_result();

    int head = spi_trx_queue_head;
    esp_err_t err = spi_device_queue_trans(spi_handle, spi_trx_queue + head, 100 / portTICK_PERIOD_MS);
    assert(err == ESP_OK);
    spi_num_outstanding_trx++;

    head++;
    if (head >= SPI_NUM_TRX_SLOTS)
        head = 0;
    spi_trx_queue_head = head;
}

static void hal_spi_init()
{
    ESP_LOGI(TAG, "Starting SPI initialization");
    esp_err_t ret;

    // init device
    spi_device_interface_config_t spi_device_intf_config = {
        .mode = 0,
        .clock_speed_hz = CONFIG_TTN_SPI_FREQ,
        .command_bits = 0,
        .address_bits = 8,
        .spics_io_num = lmic_pins.nss,
        .queue_size = SPI_QUEUE_SIZE
    };

    ret = spi_bus_add_device(lmic_pins.spi_host, &spi_device_intf_config, &spi_handle);
    assert(ret == ESP_OK);

    ESP_LOGI(TAG, "Finished SPI initialization");
}

void hal_spi_write(u1_t cmd, const u1_t *buf, int len)
{
    spi_transaction_t* trx = get_next_spi_trx_desc();
    trx->addr = cmd;
    trx->length = 8 * len;
    trx->tx_buffer = buf;
    submit_spi_trx();
}

void hal_spi_read(u1_t cmd, u1_t *buf, int len)
{
    memset(buf, 0, len);
    spi_transaction_t* trx = get_next_spi_trx_desc();
    trx->addr = cmd;
    trx->length = 8 * len;
    trx->rxlength = 8 * len;
    trx->tx_buffer = buf;
    trx->rx_buffer = buf;
    submit_spi_trx();

    while (spi_num_outstanding_trx > 0)
        collect_spi_result();
}

// -----------------------------------------------------------------------------
// TIME

static uint64_t nextTimerEvent = 0xffffffff;

static void IRAM_ATTR timer_irq_handler(void *arg)
{
    TIMERG0.int_clr_timers.t1 = 1;
    BaseType_t higher_prio_task_woken = pdFALSE;
    queue_item_t item = {
        .time = (ostime_t)nextTimerEvent,
        .ev = TIMER
    };
    xQueueSendFromISR(dio_queue, &item, &higher_prio_task_woken);
    if (higher_prio_task_woken)
        portYIELD_FROM_ISR();
}

typedef enum {
    CHECK_IO,
    WAIT_FOR_ANY_EVENT,
    WAIT_FOR_TIMER
} WaitOption;

static bool hal_wait(WaitOption waitOption)
{
    TickType_t ticksToWait = waitOption == CHECK_IO ? 0 : portMAX_DELAY;
    while (true)
    {
        queue_item_t item;
        if (xQueueReceive(dio_queue, &item, ticksToWait) == pdFALSE)
            return false;

        if (item.ev == WAKEUP)
        {
            return true;
            // nothing to do; just wake up event
        }
        else if (item.ev == TIMER) {
            ostime_t t = (ostime_t)nextTimerEvent;
            if (item.time == t)
            {
                nextTimerEvent = 0xffffffff;
                if (waitOption != CHECK_IO)
                    return true;
            }
        }
        else
        {
            hal_enterCriticalSection();
            radio_irq_handler(item.ev, item.time);
            hal_leaveCriticalSection();
            if (waitOption != WAIT_FOR_TIMER)
                return true;
        }
    }
}

static void hal_time_init()
{
    ESP_LOGI(TAG, "Starting initialisation of timer");

    timer_config_t config = {
        .alarm_en = false,
        .counter_en = false,
        .intr_type = TIMER_INTR_LEVEL,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = false,
        .divider = 1280
    };
    timer_init(TTN_TIMER_GROUP, TTN_TIMER, &config);
    timer_set_counter_value(TTN_TIMER_GROUP, TTN_TIMER, 0x0);
    timer_isr_register(TTN_TIMER_GROUP, TTN_TIMER, timer_irq_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);
    timer_start(TTN_TIMER_GROUP, TTN_TIMER);

    ESP_LOGI(TAG, "Finished initalisation of timer");
}

u4_t hal_ticks()
{
    uint64_t val;
    timer_get_counter_value(TTN_TIMER_GROUP, TTN_TIMER, &val);
    return (u4_t)val;
}

void hal_waitUntil(u4_t time)
{
    nextTimerEvent = time;
    timer_set_alarm(TTN_TIMER_GROUP, TTN_TIMER, TIMER_ALARM_DIS);
    timer_set_alarm_value(TTN_TIMER_GROUP, TTN_TIMER, nextTimerEvent);
    timer_set_alarm(TTN_TIMER_GROUP, TTN_TIMER, TIMER_ALARM_EN);
    hal_wait(WAIT_FOR_TIMER);
}

void hal_wakeUp() {
    queue_item_t item = {
        .ev = WAKEUP
    };
    xQueueSend(dio_queue, &item, 0);
}

// check and rewind for target time
u1_t hal_checkTimer(u4_t time)
{
    uint64_t now;
    timer_get_counter_value(TTN_TIMER_GROUP, TTN_TIMER, &now);
    if (time <= now)
        return 1;
    if (time - now < 5)
        return 1;

    nextTimerEvent = time;
    return 0;
}

// -----------------------------------------------------------------------------
// IRQ

void hal_disableIRQs()
{
    // nothing to do as interrupt handlers post message to queue
    // and don't access any shared data structures
}

void hal_enableIRQs()
{
    // nothing to do as interrupt handlers post message to queue
    // and don't access any shared data structures
}

void hal_sleep()
{
    if (hal_wait(CHECK_IO))
        return;

    timer_set_alarm(TTN_TIMER_GROUP, TTN_TIMER, TIMER_ALARM_DIS);
    timer_set_alarm_value(TTN_TIMER_GROUP, TTN_TIMER, nextTimerEvent);
    timer_set_alarm(TTN_TIMER_GROUP, TTN_TIMER, TIMER_ALARM_EN);
    hal_wait(WAIT_FOR_ANY_EVENT);
}

// -----------------------------------------------------------------------------
// Synchronization between application code and background task

static SemaphoreHandle_t mutex;

void hal_initCriticalSection()
{
    mutex = xSemaphoreCreateRecursiveMutex();
}

void hal_enterCriticalSection()
{
    xSemaphoreTakeRecursive(mutex, portMAX_DELAY);
}

void hal_leaveCriticalSection()
{
    xSemaphoreGiveRecursive(mutex);
}

// -----------------------------------------------------------------------------

static void hal_bgTask(void* pvParameter) {
    os_runloop();
}

void hal_init()
{
    // configure radio I/O and interrupt handler
    hal_io_init();
    // configure radio SPI
    hal_spi_init();
    // configure timer and interrupt handler
    hal_time_init();
}

void hal_startBgTask() {
    xTaskCreate(hal_bgTask, "ttn_lora_task", 1024 * 4, NULL, CONFIG_TTN_BG_TASK_PRIO, NULL);
}

void hal_failed(const char *file, u2_t line)
{
    ESP_LOGE(TAG, "%s:%d", file, line);
    assert(0);
}
