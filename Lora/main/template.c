#include "driver/spi_master.h"
#include "esp_log.h"

#define PIN_NUM_MISO 4
#define PIN_NUM_MOSI 5
#define PIN_NUM_CLK 3
#define PIN_NUM_CS 2

#define SX1276_REG_FRF_MSB 0x06
#define SX1276_REG_FRF_MID 0x07
#define SX1276_REG_FRF_LSB 0x08
#define SX1276_REG_VERSION 0x42
static const char *TAG = "SX1276";
static const char *tag = "SX1276_CHECK";
// SPI handle
spi_device_handle_t spi;

void sx1276_init_spi() {
    spi_bus_config_t buscfg = {.miso_io_num = PIN_NUM_MISO,
                               .mosi_io_num = PIN_NUM_MOSI,
                               .sclk_io_num = PIN_NUM_CLK,
                               .quadwp_io_num = -1,
                               .quadhd_io_num = -1};

    spi_device_interface_config_t devcfg = {.clock_speed_hz = 1 * 1000 * 100,
                                            .mode = 0,
                                            .spics_io_num = PIN_NUM_CS,
                                            .queue_size = 1};

    // Let ESP32 automatically pick DMA channel
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi));
}

// Read a single register
uint8_t sx1276_read_reg(uint8_t reg) {
    spi_transaction_t t = {0};
    uint8_t rx_data = 0;
    uint8_t tx_data = reg & 0x7F; // MSB=0 for read

    t.length = 8;
    t.tx_buffer = &tx_data;
    t.rx_buffer = &rx_data;
    t.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;

    ESP_ERROR_CHECK(spi_device_transmit(spi, &t));
    return t.rx_data[0];
}

void sx1276_read_frequency() {
    uint8_t msb = sx1276_read_reg(SX1276_REG_FRF_MSB);
    uint8_t mid = sx1276_read_reg(SX1276_REG_FRF_MID);
    uint8_t lsb = sx1276_read_reg(SX1276_REG_FRF_LSB);

    uint32_t frf = ((uint32_t)msb << 16) | ((uint32_t)mid << 8) | lsb;
    double freq_hz = frf * 32e6 / 524288; // 2^19 = 524288

    ESP_LOGI(TAG, "RegFrf: 0x%06X", frf);
    ESP_LOGI(TAG, "Current Frequency: %.2f MHz", freq_hz / 1e6);
}

void app_main(void) {
    sx1276_init_spi();
    uint8_t version = sx1276_read_reg(SX1276_REG_VERSION);
    if (version == 0x12) {
        ESP_LOGI(TAG, "SPI wiring is good! RegVersion = 0x%02X", version);
    } else {
        ESP_LOGE(TAG, "SPI read failed! RegVersion = 0x%02X", version);
    }
    sx1276_read_frequency();
}
