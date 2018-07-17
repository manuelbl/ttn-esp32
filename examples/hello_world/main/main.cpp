/*******************************************************************************
 * Copyright (c) 2018 Manuel Bleichenbacher
 * 
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * Sample program showing how to send a test message every 30 second.
 *******************************************************************************/

#include "freertos/FreeRTOS.h"
#include "esp_event.h"

#include "TheThingsNetwork.h"

// NOTE:
// The LoRaWAN frequency and the radio chip must be configured by running 'make menuconfig'.
// Go to Components / The Things Network, select the appropriate values and save.

// Copy the below hex string from the "Device EUI" field
// on your device's overview page in the TTN console.
const char *devEui = "????????????????";;

// Copy the below two lines from bottom of the same page
const char *appEui = "????????????????";
const char *appKey = "????????????????????????????????";


static TheThingsNetwork ttn;

const unsigned TX_INTERVAL = 30;
static uint8_t msgData[] = "Hello, world";


void send_messages(void* pvParameter)
{
    while (1) {
        printf("Sending message...\n");
        ttn_response_t res = ttn.sendBytes(msgData, sizeof(msgData) - 1);
        if (res == TTN_SUCCESSFUL_TRANSMISSION)
            printf("Message sent.\n");
        else
            printf("Message transmission failed.\n");

        vTaskDelay(TX_INTERVAL * 1000 / portTICK_PERIOD_MS);
    }
}

extern "C" void app_main(void)
{
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);

    // Initialize SPI bus
    spi_bus_config_t spi_bus_config;
    spi_bus_config.miso_io_num = 19;
    spi_bus_config.mosi_io_num = 27;
    spi_bus_config.sclk_io_num = 5;
    spi_bus_config.quadwp_io_num = -1;
    spi_bus_config.quadhd_io_num = -1;
    spi_bus_config.max_transfer_sz = 0;

    esp_err_t ret = spi_bus_initialize(HSPI_HOST, &spi_bus_config, 1);
    assert(ret == ESP_OK);

    ttn.configurePins(HSPI_HOST, 18, TTN_NOT_CONNECTED, 14, 26, 33);

    ttn.provision(devEui, appEui, appKey);

    printf("Joining...\n");
    ttn.join();
    printf("Joined.\n");

    xTaskCreate(send_messages, "send_messages", 1024 * 4, (void* )0, 3, NULL);
}
