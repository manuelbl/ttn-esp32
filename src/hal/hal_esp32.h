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

#ifndef _hal_esp32_h_
#define _hal_esp32_h_

#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "driver/spi_master.h"


enum HAL_Event {
    DIO0 = 0,
    DIO1,
    DIO2,
    TIMER,
    WAKEUP
};


enum WaitKind {
    CHECK_IO,
    WAIT_FOR_ANY_EVENT,
    WAIT_FOR_TIMER
};



class HAL_ESP32
{
public:
    HAL_ESP32();

    void configurePins(spi_host_device_t spi_host, uint8_t nss, uint8_t rxtx, uint8_t rst, uint8_t dio0, uint8_t dio1);
    void init();
    void startBackgroundTask();
    void wakeUp();
    void initCriticalSection();
    void enterCriticalSection();
    void leaveCriticalSection();
    void spiWrite(uint8_t cmd, const uint8_t *buf, size_t len);
    void spiRead(uint8_t cmd, uint8_t *buf, size_t len);
    uint8_t checkTimer(uint32_t time);
    void sleep();
    void waitUntil(uint32_t time);

    spi_host_device_t spiHost;
    gpio_num_t pinNSS;
    gpio_num_t pinRxTx;
    gpio_num_t pinRst;
    gpio_num_t pinDIO0;
    gpio_num_t pinDIO1;
    int8_t rssiCal;

private:
    static void backgroundTask(void* pvParameter);
    static void dioIrqHandler(void* arg);
    void ioInit();
    void spiInit();
    void timerInit();
    void prepareNextAlarm(uint32_t time);
    void armTimer();
    void disarmTimer();
    static void IRAM_ATTR timerIrqHandler(void *arg);
    bool wait(WaitKind waitKind);

    QueueHandle_t dioQueue;
    spi_device_handle_t spiHandle;
    spi_transaction_t spiTransaction;
    uint64_t nextTimerEvent;
    SemaphoreHandle_t mutex;
};

extern HAL_ESP32 ttn_hal;


#endif // _hal_esp32_h_