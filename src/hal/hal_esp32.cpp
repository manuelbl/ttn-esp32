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

#include "../lmic/lmic.h"
#include "../hal/hal_esp32.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/timer.h"
#include "esp_log.h"

#define LMIC_UNUSED_PIN 0xff

#define NOTIFY_BIT_DIO 1
#define NOTIFY_BIT_TIMER 2
#define NOTIFY_BIT_WAKEUP 4
#define NOTIFY_BIT_STOP 8


static const char* const TAG = "ttn_hal";

HAL_ESP32 ttn_hal;

TaskHandle_t HAL_ESP32::lmicTask = nullptr;
uint32_t HAL_ESP32::dioInterruptTime = 0;
uint8_t HAL_ESP32::dioNum = 0;


// -----------------------------------------------------------------------------
// Constructor

HAL_ESP32::HAL_ESP32()
    : rssiCal(10), nextAlarm(0)
{    
}

// -----------------------------------------------------------------------------
// I/O

void HAL_ESP32::configurePins(spi_host_device_t spi_host, uint8_t nss, uint8_t rxtx, uint8_t rst, uint8_t dio0, uint8_t dio1)
{
    spiHost = spi_host;
    pinNSS = (gpio_num_t)nss;
    pinRxTx = (gpio_num_t)rxtx;
    pinRst = (gpio_num_t)rst;
    pinDIO0 = (gpio_num_t)dio0;
    pinDIO1 = (gpio_num_t)dio1;

    // Until the background process has been started, use the current task
    // for supporting calls like `hal_waitUntil()`.
    lmicTask = xTaskGetCurrentTaskHandle();
}


void IRAM_ATTR HAL_ESP32::dioIrqHandler(void *arg)
{
    dioInterruptTime = hal_ticks();
    dioNum = (u1_t)(long)arg;
    BaseType_t higherPrioTaskWoken = pdFALSE;
    xTaskNotifyFromISR(lmicTask, NOTIFY_BIT_DIO, eSetBits, &higherPrioTaskWoken);
    if (higherPrioTaskWoken)
        portYIELD_FROM_ISR();
}

void HAL_ESP32::ioInit()
{
    // pinNSS and pinDIO0 and pinDIO1 are required
    ASSERT(pinNSS != LMIC_UNUSED_PIN);
    ASSERT(pinDIO0 != LMIC_UNUSED_PIN);
    ASSERT(pinDIO1 != LMIC_UNUSED_PIN);

    gpio_pad_select_gpio(pinNSS);
    gpio_set_level(pinNSS, 0);
    gpio_set_direction(pinNSS, GPIO_MODE_OUTPUT);

    if (pinRxTx != LMIC_UNUSED_PIN)
    {
        gpio_pad_select_gpio(pinRxTx);
        gpio_set_level(pinRxTx, 0);
        gpio_set_direction(pinRxTx, GPIO_MODE_OUTPUT);
    }

    if (pinRst != LMIC_UNUSED_PIN)
    {
        gpio_pad_select_gpio(pinRst);
        gpio_set_level(pinRst, 0);
        gpio_set_direction(pinRst, GPIO_MODE_OUTPUT);
    }

    // DIO pins with interrupt handlers
    gpio_pad_select_gpio(pinDIO0);
    gpio_set_direction(pinDIO0, GPIO_MODE_INPUT);
    gpio_set_intr_type(pinDIO0, GPIO_INTR_POSEDGE);

    gpio_pad_select_gpio(pinDIO1);
    gpio_set_direction(pinDIO1, GPIO_MODE_INPUT);
    gpio_set_intr_type(pinDIO1, GPIO_INTR_POSEDGE);

    ESP_LOGI(TAG, "IO initialized");
}

void hal_pin_rxtx(u1_t val)
{
    if (ttn_hal.pinRxTx == LMIC_UNUSED_PIN)
        return;
    
    gpio_set_level(ttn_hal.pinRxTx, val);
}

void hal_pin_rst(u1_t val)
{
    if (ttn_hal.pinRst == LMIC_UNUSED_PIN)
        return;

    if (val == 0 || val == 1)
    {
        // drive pin
        gpio_set_level(ttn_hal.pinRst, val);
        gpio_set_direction(ttn_hal.pinRst, GPIO_MODE_OUTPUT);
    }
    else
    {
#if defined(CONFIG_TTN_RESET_STATES_ASSERTED)
        // drive up the pin because the hardware is nonstandard
        gpio_set_level(ttn_hal.pinRst, 1);
        gpio_set_direction(ttn_hal.pinRst, GPIO_MODE_OUTPUT);
#else
        // keep pin floating
        gpio_set_level(ttn_hal.pinRst, val);
        gpio_set_direction(ttn_hal.pinRst, GPIO_MODE_INPUT);
#endif
    }
}

s1_t hal_getRssiCal (void)
{
    return ttn_hal.rssiCal;
}

ostime_t hal_setModuleActive (bit_t val)
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

void HAL_ESP32::spiInit()
{
    // init device
    spi_device_interface_config_t spiConfig;
    memset(&spiConfig, 0, sizeof(spiConfig));
    spiConfig.mode = 1;
    spiConfig.clock_speed_hz = CONFIG_TTN_SPI_FREQ;
    spiConfig.command_bits = 0;
    spiConfig.address_bits = 8;
    spiConfig.spics_io_num = pinNSS;
    spiConfig.queue_size = 1;
    spiConfig.cs_ena_posttrans = 2;

    esp_err_t ret = spi_bus_add_device(spiHost, &spiConfig, &spiHandle);
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
 * LIMIC uses a 32 bit time system (ostime_t) counting ticks. In this
 * implementation each tick is 16µs. It will wrap arounnd every 19 hours.
 * 
 * The ESP32 has a 64 bit timer counting microseconds. It will wrap around
 * every 584,000 years. So we don't need to bother.
 * 
 * Based on this timer, future callbacks can be scheduled. This is used to
 * schedule the next LMIC job.
 */

// Convert LMIC tick time (ostime_t) to ESP absolute time.
// `osTime` is assumed to be somewhere between one hour in the past and
// 18 hours into the future. 
int64_t HAL_ESP32::osTimeToEspTime(int64_t espNow, uint32_t osTime)
{
    int64_t espTime;
    uint32_t osNow = (uint32_t)(espNow >> 4);

    // unsigned difference:
    // 0x00000000 - 0xefffffff: future (0 to about 18 hours)
    // 0xf0000000 - 0xffffffff: past (about 1 to 0 hours)
    uint32_t osDiff = osTime - osNow;
    if (osDiff < 0xf0000000)
    {
        espTime = espNow + (((int64_t)osDiff) << 4);
    }
    else
    {
        // one's complement instead of two's complement:
        // off by 1 µs and ignored
        osDiff = ~osDiff;
        espTime = espNow - (((int64_t)osDiff) << 4);
    }

    return espTime;
}

void HAL_ESP32::timerInit()
{
    esp_timer_create_args_t timerConfig = {
        .callback = &timerCallback,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "lmic_job"
    };
    esp_err_t err = esp_timer_create(&timerConfig, &timer);
    ESP_ERROR_CHECK(err);

    ESP_LOGI(TAG, "Timer initialized");
}

void HAL_ESP32::setNextAlarm(int64_t time)
{
    nextAlarm = time;
}

void HAL_ESP32::armTimer(int64_t espNow)
{
    if (nextAlarm == 0)
        return;
    int64_t timeout = nextAlarm - esp_timer_get_time();
    if (timeout < 0)
        timeout = 10;
    esp_timer_start_once(timer, timeout);
}

void HAL_ESP32::disarmTimer()
{
    esp_timer_stop(timer);
}

void HAL_ESP32::timerCallback(void *arg)
{
    xTaskNotify(lmicTask, NOTIFY_BIT_TIMER, eSetBits);
}

// Wait for the next external event. Either:
// - scheduled timer due to scheduled job or waiting for a given time
// - wake up event from the client code
// - I/O interrupt (DIO0 or DIO1 pin)
bool HAL_ESP32::wait(WaitKind waitKind)
{
    TickType_t ticksToWait = waitKind == CHECK_IO ? 0 : portMAX_DELAY;
    while (true)
    {
        uint32_t bits = ulTaskNotifyTake(pdTRUE, ticksToWait);
        if (bits == 0)
            return false;

        if ((bits & NOTIFY_BIT_STOP) != 0)
            return false;

        if ((bits & NOTIFY_BIT_WAKEUP) != 0)
        {
            if (waitKind != WAIT_FOR_TIMER)
            {
                disarmTimer();
                return true;
            }
        }
        else if ((bits & NOTIFY_BIT_TIMER) != 0)
        {
            disarmTimer();
            setNextAlarm(0);
            if (waitKind != CHECK_IO)
                return true;
        }
        else // IO interrupt
        {
            if (waitKind != WAIT_FOR_TIMER)
                disarmTimer();
            enterCriticalSection();
            radio_irq_handler_v2(dioNum, dioInterruptTime);
            leaveCriticalSection();
            if (waitKind != WAIT_FOR_TIMER)
                return true;
        }
    }
}

// Gets current time in LMIC ticks
u4_t IRAM_ATTR hal_ticks()
{
    // LMIC tick unit: 16µs
    // esp_timer unit: 1µs
    return (u4_t)(esp_timer_get_time() >> 4);
}

// Wait until the specified time.
// Called if the LMIC code needs to wait for a precise time.
// All other events are ignored and will be served later.
u4_t hal_waitUntil(u4_t time)
{
    return ttn_hal.waitUntil(time);
}

uint32_t HAL_ESP32::waitUntil(uint32_t osTime)
{
    int64_t espNow = esp_timer_get_time();
    int64_t espTime = osTimeToEspTime(espNow, osTime);
    setNextAlarm(espTime);
    armTimer(espNow);
    wait(WAIT_FOR_TIMER);

    u4_t osNow = hal_ticks();
    u4_t diff = osNow - osTime;
    return diff < 0x80000000U ? diff : 0;
}

// Called by client code to wake up LMIC to do something,
// e.g. send a submitted messages.
void HAL_ESP32::wakeUp()
{
    xTaskNotify(lmicTask, NOTIFY_BIT_WAKEUP, eSetBits);
}

// Check if the specified time has been reached or almost reached.
// Otherwise, save it as alarm time.
// LMIC calls this function with the scheduled time of the next job
// in the queue. If the job is not due yet, LMIC will go to sleep.
u1_t hal_checkTimer(uint32_t time)
{
    return ttn_hal.checkTimer(time);
}

uint8_t HAL_ESP32::checkTimer(u4_t osTime)
{
    int64_t espNow = esp_timer_get_time();
    int64_t espTime = osTimeToEspTime(espNow, osTime);
    int64_t diff = espTime - espNow;
    if (diff < 100)
        return 1; // timer has expired or will expire very soon

    setNextAlarm(espTime);
    return 0;
}

// Go to sleep until next event.
// Called when LMIC is not busy and not job is due to be executed.
void hal_sleep()
{
    ttn_hal.sleep();
}

void HAL_ESP32::sleep()
{
    if (wait(CHECK_IO))
        return;

    armTimer(esp_timer_get_time());
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

void hal_processPendingIRQs()
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

void HAL_ESP32::lmicBackgroundTask(void* pvParameter)
{
    HAL_ESP32* instance = (HAL_ESP32*)pvParameter;
    while (instance->runBackgroundTask)
        os_runloop_once();
    vTaskDelete(nullptr);
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
    // configure timer and alarm callback
    timerInit();
}

void HAL_ESP32::startLMICTask()
{
    runBackgroundTask = true;
    xTaskCreate(lmicBackgroundTask, "ttn_lmic", 1024 * 4, this, CONFIG_TTN_BG_TASK_PRIO, &lmicTask);

    // enable interrupts
    gpio_isr_handler_add(pinDIO0, dioIrqHandler, (void *)0);
    gpio_isr_handler_add(pinDIO1, dioIrqHandler, (void *)1);
}

void HAL_ESP32::stopLMICTask()
{
    runBackgroundTask = false;
    gpio_isr_handler_remove(pinDIO0);
    gpio_isr_handler_remove(pinDIO1);
    disarmTimer();
    xTaskNotify(lmicTask, NOTIFY_BIT_STOP, eSetBits);
}


// -----------------------------------------------------------------------------
// Fatal failure

static hal_failure_handler_t* custom_hal_failure_handler = nullptr;

void hal_set_failure_handler(const hal_failure_handler_t* const handler)
{
    custom_hal_failure_handler = handler;
}

void hal_failed(const char *file, u2_t line)
{
    if (custom_hal_failure_handler != nullptr)
        (*custom_hal_failure_handler)(file, line);

    ESP_LOGE(TAG, "LMIC failed and stopped: %s:%d", file, line);

    // go to sleep forever
    while (true)
    {
        vTaskDelay(portMAX_DELAY);
    }
}
