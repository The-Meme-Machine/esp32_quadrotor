/*
 * SPI_ws2812.c
 *
 *  Created on: 02-Nov-2020
 *      Author: Dhananjay Khairnar
 */

#include "ws2812.h"
#define LED_MAX_NBER_LEDS 1
#define LED_DMA_BUFFER_SIZE ((LED_MAX_NBER_LEDS * 16 * (24 / 4))) + 1
#define LED_PIN GPIO_NUM_21
typedef struct
{
    spi_host_device_t host;
    spi_device_handle_t spi;
    int dma_chan;
    spi_device_interface_config_t devcfg;
    spi_bus_config_t buscfg;
} SPI_settings_t;

uint16_t *ledDMAbuffer;

uint32_t table[LED_MAX_NBER_LEDS];

// CRGB led = {.r = 0, .g = 0xff, .b = 0};

static SPI_settings_t SPI_settings = {
    .host = SPI2_HOST,
    .dma_chan = 2,
    .buscfg = {
        .miso_io_num = -1,
        .mosi_io_num = LED_PIN,
        .sclk_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LED_DMA_BUFFER_SIZE},
    .devcfg = {.clock_speed_hz = 3.2 * 1000 * 1000, // Clock out at 3.2 MHz
               .mode = 0,                           // SPI mode 0
               .spics_io_num = -1,                  // CS pin
               .queue_size = 1,                     // Not sure if needed
               .command_bits = 0,
               .address_bits = 0}};

void initSPIws2812()
{
    esp_err_t err;

    err = spi_bus_initialize(SPI_settings.host, &SPI_settings.buscfg, SPI_settings.dma_chan);
    ESP_ERROR_CHECK(err);

    // Attach the Accel to the SPI bus
    err = spi_bus_add_device(SPI_settings.host, &SPI_settings.devcfg, &SPI_settings.spi);
    ESP_ERROR_CHECK(err);

    ledDMAbuffer = heap_caps_malloc(LED_DMA_BUFFER_SIZE, MALLOC_CAP_DMA); // Critical to be DMA memory.
}

void led_strip_update(uint32_t led)
{
    uint32_t temp = led; // Data you want to write to each LEDs, I'm my case it's 95 RGB x 3 color

    uint16_t LedBitPattern[16] = {
        0x8888,
        0x8C88,
        0xC888,
        0xCC88,
        0x888C,
        0x8C8C,
        0xC88C,
        0xCC8C,
        0x88C8,
        0x8CC8,
        0xC8C8,
        0xCCC8,
        0x88CC,
        0x8CCC,
        0xC8CC,
        0xCCCC};
    // uint32_t i;
    esp_err_t err;

    memset(ledDMAbuffer, 0, LED_DMA_BUFFER_SIZE);
    int n = 0;

    // R
    ledDMAbuffer[n++] = LedBitPattern[0x0f & (temp >> 12)];
    ledDMAbuffer[n++] = LedBitPattern[0x0f & (temp) >> 8];

    // G
    ledDMAbuffer[n++] = LedBitPattern[0x0f & (temp >> 4)];
    ledDMAbuffer[n++] = LedBitPattern[0x0f & (temp)];

    // B
    ledDMAbuffer[n++] = LedBitPattern[0x0f & (temp >> 20)];
    ledDMAbuffer[n++] = LedBitPattern[0x0f & (temp) >> 16];

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = LED_DMA_BUFFER_SIZE * 8; // length is in bits
    t.tx_buffer = ledDMAbuffer;

    err = spi_device_transmit(SPI_settings.spi, &t);
    ESP_ERROR_CHECK(err);
}