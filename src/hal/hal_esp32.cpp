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

static const char * const TAG = "ttn_hal";

lmic_pinmap lmic_pins;
HAL_ESP32 ttn_hal;


struct HALQueueItem {
    ostime_t time;
    HAL_Event ev;

    HALQueueItem() : time(0), ev(DIO0) { }
    HALQueueItem(HAL_Event e, ostime_t t = 0)
        : time(t), ev(e) { }
};

// -----------------------------------------------------------------------------
// Constructor

HAL_ESP32::HAL_ESP32()
    : nextTimerEvent(0x200000000)
{    
}

// -----------------------------------------------------------------------------
// I/O

void IRAM_ATTR HAL_ESP32::dioIrqHandler(void *arg)
{
    uint64_t now;
    timer_get_counter_value(TTN_TIMER_GROUP, TTN_TIMER, &now);
    BaseType_t higherPrioTaskWoken = pdFALSE;
    HALQueueItem item { (HAL_Event)(long)arg, (ostime_t)now };
    xQueueSendFromISR(ttn_hal.dioQueue, &item, &higherPrioTaskWoken);
    if (higherPrioTaskWoken)
        portYIELD_FROM_ISR();
}

void HAL_ESP32::ioInit()
{
    // NSS and DIO0 and DIO1 are required
    ASSERT(lmic_pins.nss != LMIC_UNUSED_PIN);
    ASSERT(lmic_pins.dio0 != LMIC_UNUSED_PIN);
    ASSERT(lmic_pins.dio1 != LMIC_UNUSED_PIN);

    gpio_pad_select_gpio(lmic_pins.nss);
    gpio_set_level((gpio_num_t)lmic_pins.nss, 0);
    gpio_set_direction((gpio_num_t)lmic_pins.nss, GPIO_MODE_OUTPUT);

    if (lmic_pins.rxtx != LMIC_UNUSED_PIN)
    {
        gpio_pad_select_gpio(lmic_pins.rxtx);
        gpio_set_level((gpio_num_t)lmic_pins.rxtx, 0);
        gpio_set_direction((gpio_num_t)lmic_pins.rxtx, GPIO_MODE_OUTPUT);
    }

    if (lmic_pins.rst != LMIC_UNUSED_PIN)
    {
        gpio_pad_select_gpio((gpio_num_t)lmic_pins.rst);
        gpio_set_level((gpio_num_t)lmic_pins.rst, 0);
        gpio_set_direction((gpio_num_t)lmic_pins.rst, GPIO_MODE_OUTPUT);
    }

    dioQueue = xQueueCreate(12, sizeof(HALQueueItem));
    ASSERT(dioQueue != NULL);

    gpio_pad_select_gpio(lmic_pins.dio0);
    gpio_set_direction((gpio_num_t)lmic_pins.dio0, GPIO_MODE_INPUT);
    gpio_set_intr_type((gpio_num_t)lmic_pins.dio0, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add((gpio_num_t)lmic_pins.dio0, dioIrqHandler, (void *)0);

    gpio_pad_select_gpio((gpio_num_t)lmic_pins.dio1);
    gpio_set_direction((gpio_num_t)lmic_pins.dio1, GPIO_MODE_INPUT);
    gpio_set_intr_type((gpio_num_t)lmic_pins.dio1, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add((gpio_num_t)lmic_pins.dio1, dioIrqHandler, (void *)1);

    ESP_LOGI(TAG, "IO initialized");
}

void hal_pin_rxtx(u1_t val)
{
    if (lmic_pins.rxtx == LMIC_UNUSED_PIN)
        return;
    
    gpio_set_level((gpio_num_t)lmic_pins.rxtx, val);
}

void hal_pin_rst(u1_t val)
{
    if (lmic_pins.rst == LMIC_UNUSED_PIN)
        return;

    if (val == 0 || val == 1)
    { // drive pin
        gpio_set_level((gpio_num_t)lmic_pins.rst, val);
        gpio_set_direction((gpio_num_t)lmic_pins.rst, GPIO_MODE_OUTPUT);
    }
    else
    { // keep pin floating
        gpio_set_level((gpio_num_t)lmic_pins.rst, val);
        gpio_set_direction((gpio_num_t)lmic_pins.rst, GPIO_MODE_INPUT);
    }
}

s1_t hal_getRssiCal (void)
{
    return lmic_pins.rssi_cal;
}

ostime_t hal_setModuleActive (bit_t val)
{
    return 0;
}

bit_t hal_queryUsingTcxo(void)
{
    return false;
}


// -----------------------------------------------------------------------------
// SPI

void HAL_ESP32::spiInit()
{
    // init device
    spi_device_interface_config_t spiConfig;
    memset(&spiConfig, 0, sizeof(spiConfig));
    spiConfig.mode = 1;
    spiConfig.clock_speed_hz = CONFIG_TTN_SPI_FREQ;
    spiConfig.command_bits = 0;
    spiConfig.address_bits = 8;
    spiConfig.spics_io_num = lmic_pins.nss;
    spiConfig.queue_size = 1;
    spiConfig.cs_ena_posttrans = 2;

    esp_err_t ret = spi_bus_add_device(lmic_pins.spi_host, &spiConfig, &spiHandle);
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "SPI initialized");
}

void hal_spi_write(u1_t cmd, const u1_t *buf, size_t len)
{
    ttn_hal.spiWrite(cmd, buf, len);
}

void HAL_ESP32::spiWrite(uint8_t cmd, const uint8_t *buf, size_t len)
{
    memset(&spiTransaction, 0, sizeof(spiTransaction));
    spiTransaction.addr = cmd;
    spiTransaction.length = 8 * len;
    spiTransaction.tx_buffer = buf;
    esp_err_t err = spi_device_transmit(spiHandle, &spiTransaction);
    ESP_ERROR_CHECK(err);
}

void hal_spi_read(u1_t cmd, u1_t *buf, size_t len)
{
    ttn_hal.spiRead(cmd, buf, len);
}

void HAL_ESP32::spiRead(uint8_t cmd, uint8_t *buf, size_t len)
{
    memset(buf, 0, len);
    memset(&spiTransaction, 0, sizeof(spiTransaction));
    spiTransaction.addr = cmd;
    spiTransaction.length = 8 * len;
    spiTransaction.rxlength = 8 * len;
    spiTransaction.tx_buffer = buf;
    spiTransaction.rx_buffer = buf;
    esp_err_t err = spi_device_transmit(spiHandle, &spiTransaction);
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

static const ostime_t OVERRUN_TRESHOLD = 0x10000; // approx 10 seconds

void HAL_ESP32::timerInit()
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
    timer_isr_register(TTN_TIMER_GROUP, TTN_TIMER, timerIrqHandler, NULL, ESP_INTR_FLAG_IRAM, NULL);
    timer_start(TTN_TIMER_GROUP, TTN_TIMER);

    ESP_LOGI(TAG, "Timer initialized");
}

void HAL_ESP32::prepareNextAlarm(u4_t time)
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

    nextTimerEvent = time;
    if (now32 > time && now32 - time > OVERRUN_TRESHOLD)
        nextTimerEvent += 0x100000000;
}

void HAL_ESP32::armTimer()
{
    timer_set_alarm(TTN_TIMER_GROUP, TTN_TIMER, TIMER_ALARM_DIS);
    timer_set_alarm_value(TTN_TIMER_GROUP, TTN_TIMER, nextTimerEvent);
    timer_set_alarm(TTN_TIMER_GROUP, TTN_TIMER, TIMER_ALARM_EN);
}

void HAL_ESP32::disarmTimer()
{
    timer_set_alarm(TTN_TIMER_GROUP, TTN_TIMER, TIMER_ALARM_DIS);
    nextTimerEvent = 0x200000000; // wait indefinitely (almost)
}

void IRAM_ATTR HAL_ESP32::timerIrqHandler(void *arg)
{
    TTN_CLEAR_TIMER_ALARM;
    BaseType_t higherPrioTaskWoken = pdFALSE;
    HALQueueItem item { TIMER };
    xQueueSendFromISR(ttn_hal.dioQueue, &item, &higherPrioTaskWoken);
    if (higherPrioTaskWoken)
        portYIELD_FROM_ISR();
}

bool HAL_ESP32::wait(WaitKind waitKind)
{
    TickType_t ticksToWait = waitKind == CHECK_IO ? 0 : portMAX_DELAY;
    while (true)
    {
        HALQueueItem item;
        if (xQueueReceive(dioQueue, &item, ticksToWait) == pdFALSE)
            return false;

        if (item.ev == WAKEUP)
        {
            if (waitKind != WAIT_FOR_TIMER)
            {
                disarmTimer();
                return true;
            }
        }
        else if (item.ev == TIMER)
        {
            disarmTimer();
            if (waitKind != CHECK_IO)
                return true;
        }
        else // IO interrupt
        {
            if (waitKind != WAIT_FOR_TIMER)
                disarmTimer();
            enterCriticalSection();
            radio_irq_handler_v2(item.ev, item.time);
            leaveCriticalSection();
            if (waitKind != WAIT_FOR_TIMER)
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
    ttn_hal.waitUntil(time);
}

void HAL_ESP32::waitUntil(uint32_t time)
{
    prepareNextAlarm(time);
    armTimer();
    wait(WAIT_FOR_TIMER);
}

void HAL_ESP32::wakeUp()
{
    HALQueueItem item { WAKEUP };
    xQueueSend(dioQueue, &item, 0);
}

// check and rewind for target time
u1_t hal_checkTimer(u4_t time)
{
    return ttn_hal.checkTimer(time);
}

uint8_t HAL_ESP32::checkTimer(uint32_t time)
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

    prepareNextAlarm(time);
    return 0;
}

void hal_sleep()
{
    ttn_hal.sleep();
}

void HAL_ESP32::sleep()
{
    if (wait(CHECK_IO))
        return;

    armTimer();
    wait(WAIT_FOR_ANY_EVENT);
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

void HAL_ESP32::initCriticalSection()
{
    mutex = xSemaphoreCreateRecursiveMutex();
}

void HAL_ESP32::enterCriticalSection()
{
    xSemaphoreTakeRecursive(mutex, portMAX_DELAY);
}

void HAL_ESP32::leaveCriticalSection()
{
    xSemaphoreGiveRecursive(mutex);
}

// -----------------------------------------------------------------------------

void HAL_ESP32::backgroundTask(void* pvParameter) {
    os_runloop();
}

void hal_init_ex(const void *pContext)
{
    ttn_hal.init();
}

void HAL_ESP32::init()
{
    // configure radio I/O and interrupt handler
    ioInit();
    // configure radio SPI
    spiInit();
    // configure timer and interrupt handler
    timerInit();
}

void HAL_ESP32::startBackgroundTask() {
    xTaskCreate(backgroundTask, "ttn_lora_task", 1024 * 4, NULL, CONFIG_TTN_BG_TASK_PRIO, NULL);
}

void hal_failed(const char *file, u2_t line)
{
    ESP_LOGE(TAG, "%s:%d", file, line);
    ASSERT(0);
}
