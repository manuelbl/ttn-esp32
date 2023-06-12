/*******************************************************************************
 * 
 * ttn-esp32 - The Things Network device library for ESP-IDF / SX127x
 * 
 * Copyright (c) 2018 Manuel Bleichenbacher
 * 
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 *
 * Sample program showing how to send and receive messages.
 *******************************************************************************/

#include "freertos/FreeRTOS.h"
#include "esp_event.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include <string.h>

#include "TheThingsNetwork.h"


// NOTE:
// The LoRaWAN frequency and the radio chip must be configured by running 'idf.py menuconfig'.
// Go to Components / The Things Network, select the appropriate values and save.

// Copy the below hex strings from the TTN console (Applications > Your application > End devices
// > Your device > Activation information)

// AppEUI (sometimes called JoinEUI)
const char *appEui = "????????????????";
// DevEUI
const char *devEui = "????????????????";
// AppKey
const char *appKey = "????????????????????????????????";


// Pins and other resources
#define TTN_SPI_HOST      SPI2_HOST
#define TTN_SPI_DMA_CHAN  SPI_DMA_DISABLED
#define TTN_PIN_SPI_SCLK  5
#define TTN_PIN_SPI_MOSI  27
#define TTN_PIN_SPI_MISO  19
#define TTN_PIN_NSS       18
#define TTN_PIN_RXTX      TTN_NOT_CONNECTED
#define TTN_PIN_RST       14
#define TTN_PIN_DIO0      26
#define TTN_PIN_DIO1      35

static TheThingsNetwork ttn;

const unsigned TX_INTERVAL = 30;
static uint8_t msgData[] = "Hello, world";


void printRFSettings(const char* window, const TTNRFSettings& settings)
{
    int bw = (1 << (static_cast<int>(settings.bandwidth) - 1)) * 125;
    int sf = static_cast<int>(settings.spreadingFactor) + 5;

    if (settings.spreadingFactor == kTTNSFNone)
    {
        printf("%s: not used\n", window);
    }
    else if (settings.spreadingFactor == kTTNFSK)
    {
        printf("%s: FSK, BW %dkHz, %ld.%ld MHz\n",
                window, bw, settings.frequency / 1000000, (settings.frequency % 1000000 + 50000) / 100000);
    }
    else
    {
        printf("%s: SF%d, BW %dkHz, %ld.%ld MHz\n",
                window, sf, bw, settings.frequency / 1000000, (settings.frequency % 1000000 + 50000) / 100000);
    }
}

void printAllRFSettings()
{
    printRFSettings("TX ", ttn.txSettings());
    printRFSettings("RX1", ttn.rx1Settings());
    printRFSettings("RX2", ttn.rx2Settings());
}

void sendMessages(void* pvParameter)
{
    while (1) {
        printf("Sending message...\n");
        TTNResponseCode res = ttn.transmitMessage(msgData, sizeof(msgData) - 1);
        printf(res == kTTNSuccessfulTransmission ? "Message sent.\n" : "Transmission failed.\n");
        printAllRFSettings();

        vTaskDelay(TX_INTERVAL * pdMS_TO_TICKS(1000));
    }
}

void messageReceived(const uint8_t* message, size_t length, ttn_port_t port)
{
    printf("Message of %d bytes received on port %d:", length, port);
    for (int i = 0; i < length; i++)
        printf(" %02x", message[i]);
    printf("\n");
    printf("RSSI: %d dBm\n", ttn.rssi());
}

extern "C" void app_main(void)
{
    esp_err_t err;
    // Initialize the GPIO ISR handler service
    err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    ESP_ERROR_CHECK(err);
    
    // Initialize the NVS (non-volatile storage) for saving and restoring the keys
    err = nvs_flash_init();
    ESP_ERROR_CHECK(err);

    // Initialize SPI bus
    spi_bus_config_t spi_bus_config;
    memset(&spi_bus_config, 0, sizeof(spi_bus_config));
    spi_bus_config.miso_io_num = TTN_PIN_SPI_MISO;
    spi_bus_config.mosi_io_num = TTN_PIN_SPI_MOSI;
    spi_bus_config.sclk_io_num = TTN_PIN_SPI_SCLK;
    err = spi_bus_initialize(TTN_SPI_HOST, &spi_bus_config, TTN_SPI_DMA_CHAN);
    ESP_ERROR_CHECK(err);

    // Configure the SX127x pins
    ttn.configurePins(TTN_SPI_HOST, TTN_PIN_NSS, TTN_PIN_RXTX, TTN_PIN_RST, TTN_PIN_DIO0, TTN_PIN_DIO1);

    // The below line can be commented after the first run as the data is saved in NVS
    ttn.provision(devEui, appEui, appKey);

    // Register callback for received messages
    ttn.onMessage(messageReceived);

    printf("Joining...\n");
    if (ttn.join())
    {
        printf("Joined.\n");
        printAllRFSettings();
        printf("RSSI: %d dBm\n", ttn.rssi());
        xTaskCreate(sendMessages, "send_messages", 1024 * 4, (void* )0, 3, nullptr);
    }
    else
    {
        printf("Join failed. Goodbye\n");
    }
}
