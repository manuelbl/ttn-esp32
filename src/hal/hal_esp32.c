/*******************************************************************************
 * 
 * ttn-esp32 - The Things Network device library for ESP-IDF / SX127x
 * 
 * Copyright (c) 2018 Manuel Bleichenbacher
 * 
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 *
 * Hardware abstraction layer to run LMIC on a ESP32 using ESP-iDF.
 *******************************************************************************/

#include "../lmic/lmic.h"
#include "../hal/hal_esp32.h"

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
    ASSERT(dio_queue != NULL);

    gpio_pad_select_gpio(lmic_pins.dio0);
    gpio_set_direction(lmic_pins.dio0, GPIO_MODE_INPUT);
    gpio_set_intr_type(lmic_pins.dio0, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(lmic_pins.dio0, dio_irq_handler, (void *)0);

    gpio_pad_select_gpio(lmic_pins.dio1);
    gpio_set_direction(lmic_pins.dio1, GPIO_MODE_INPUT);
    gpio_set_intr_type(lmic_pins.dio1, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(lmic_pins.dio1, dio_irq_handler, (void *)1);

    ESP_LOGI(TAG, "IO initialized");
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

s1_t hal_getRssiCal (void) {
    return lmic_pins.rssi_cal;
}


// -----------------------------------------------------------------------------
// SPI

static spi_device_handle_t spi_handle;
static spi_transaction_t spi_trx;

static void hal_spi_init()
{
    // init device
    spi_device_interface_config_t spi_device_intf_config = {
        .mode = 1,
        .clock_speed_hz = CONFIG_TTN_SPI_FREQ,
        .command_bits = 0,
        .address_bits = 8,
        .spics_io_num = lmic_pins.nss,
        .queue_size = 1,
        .cs_ena_posttrans = 2
    };

    esp_err_t ret = spi_bus_add_device(lmic_pins.spi_host, &spi_device_intf_config, &spi_handle);
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "SPI initialized");
}

void hal_spi_write(u1_t cmd, const u1_t *buf, int len)
{
    memset(&spi_trx, 0, sizeof(spi_trx));
    spi_trx.addr = cmd;
    spi_trx.length = 8 * len;
    spi_trx.tx_buffer = buf;
    esp_err_t err = spi_device_transmit(spi_handle, &spi_trx);
    ESP_ERROR_CHECK(err);
}

void hal_spi_read(u1_t cmd, u1_t *buf, int len)
{
    memset(buf, 0, len);
    memset(&spi_trx, 0, sizeof(spi_trx));
    spi_trx.addr = cmd;
    spi_trx.length = 8 * len;
    spi_trx.rxlength = 8 * len;
    spi_trx.tx_buffer = buf;
    spi_trx.rx_buffer = buf;
    esp_err_t err = spi_device_transmit(spi_handle, &spi_trx);
    ESP_ERROR_CHECK(err);
}

// -----------------------------------------------------------------------------
// TIME

/*
 * LIMIC uses a 32 bit time (ostime_t) counting ticks. In this implementation
 * each tick is 16µs. So the timer will wrap around once every 19 hour.
 * The timer alarm should trigger when a specific value has been reached.
 * Due to the wrap around, an alarm time in the future can have a lower value
 * than the current timer value.
 * 
 * ESP32 has 64 bits counters with a pecularity: the alarm does not only
 * trigger when the exact value has been reached but also when the clock is
 * higer than the alarm value. Therefore, the wrap around is more difficult to
 * handle.
 * 
 * The approach here is to always use a higher value than the current timer
 * value. If it would be lower than the timer value, 0x100000000 is added.
 * The lower 32 bits still represent the desired value. After the timer has
 * triggered an alarm and is higher than 0x100000000, it's value is reduced
 * by 0x100000000.
 */

#define OVERRUN_TRESHOLD 0x10000 // approx 10 seconds

static uint64_t next_timer_event = 0x200000000;

static void IRAM_ATTR hal_timer_irq_handler(void *arg);

static void hal_time_init()
{
    timer_config_t config = {
        .alarm_en = false,
        .counter_en = false,
        .intr_type = TIMER_INTR_LEVEL,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = false,
        .divider = 1280  /* 80 MHz APB_CLK * 16µs */
    };
    timer_init(TTN_TIMER_GROUP, TTN_TIMER, &config);
    timer_set_counter_value(TTN_TIMER_GROUP, TTN_TIMER, 0x0);
    timer_isr_register(TTN_TIMER_GROUP, TTN_TIMER, hal_timer_irq_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);
    timer_start(TTN_TIMER_GROUP, TTN_TIMER);

    ESP_LOGI(TAG, "Timer initialized");
}

static void hal_prepare_next_alarm(u4_t time)
{
    uint64_t now;
    timer_get_counter_value(TTN_TIMER_GROUP, TTN_TIMER, &now);
    u4_t now32 = (u4_t)now;

    if (now != now32)
    {
        // decrease timer to 32 bit value
        now = now32;
        timer_pause(TTN_TIMER_GROUP, TTN_TIMER);
        timer_set_counter_value(TTN_TIMER_GROUP, TTN_TIMER, now);
        timer_start(TTN_TIMER_GROUP, TTN_TIMER);
    }

    next_timer_event = time;
    if (now32 > time && now32 - time > OVERRUN_TRESHOLD)
        next_timer_event += 0x100000000;
}

static void hal_arm_timer()
{
    timer_set_alarm(TTN_TIMER_GROUP, TTN_TIMER, TIMER_ALARM_DIS);
    timer_set_alarm_value(TTN_TIMER_GROUP, TTN_TIMER, next_timer_event);
    timer_set_alarm(TTN_TIMER_GROUP, TTN_TIMER, TIMER_ALARM_EN);
}

static void hal_disarm_timer()
{
    timer_set_alarm(TTN_TIMER_GROUP, TTN_TIMER, TIMER_ALARM_DIS);
    next_timer_event = 0x200000000; // wait indefinitely (almost)
}

static void IRAM_ATTR hal_timer_irq_handler(void *arg)
{
    TTN_CLEAR_TIMER_ALARM;
    BaseType_t higher_prio_task_woken = pdFALSE;
    queue_item_t item = {
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
} wait_open_e;

static bool hal_wait(wait_open_e wait_option)
{
    TickType_t ticks_to_wait = wait_option == CHECK_IO ? 0 : portMAX_DELAY;
    while (true)
    {
        queue_item_t item;
        if (xQueueReceive(dio_queue, &item, ticks_to_wait) == pdFALSE)
            return false;

        if (item.ev == WAKEUP)
        {
            if (wait_option != WAIT_FOR_TIMER)
            {
                hal_disarm_timer();
                return true;
            }
        }
        else if (item.ev == TIMER)
        {
            hal_disarm_timer();
            if (wait_option != CHECK_IO)
                return true;
        }
        else // IO interrupt
        {
            if (wait_option != WAIT_FOR_TIMER)
                hal_disarm_timer();
            hal_enterCriticalSection();
            radio_irq_handler_v2(item.ev, item.time);
            hal_leaveCriticalSection();
            if (wait_option != WAIT_FOR_TIMER)
                return true;
        }
    }
}

u4_t hal_ticks()
{
    uint64_t val;
    timer_get_counter_value(TTN_TIMER_GROUP, TTN_TIMER, &val);
    return (u4_t)val;
}

void hal_waitUntil(u4_t time)
{
    hal_prepare_next_alarm(time);
    hal_arm_timer();
    hal_wait(WAIT_FOR_TIMER);
}

void hal_wakeUp()
{
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
    u4_t now32 = (u4_t)now;

    if (time >= now32)
    {
        if (time - now32 < 5)
            return 1; // timer will expire very soon
    }
    else
    {
        if (now32 - time < OVERRUN_TRESHOLD)
            return 1; // timer has expired recently
    }

    hal_prepare_next_alarm(time);
    return 0;
}

void hal_sleep()
{
    if (hal_wait(CHECK_IO))
        return;

    hal_arm_timer();
    hal_wait(WAIT_FOR_ANY_EVENT);
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

void hal_init_ex(const void *pContext)
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
    ASSERT(0);
}
