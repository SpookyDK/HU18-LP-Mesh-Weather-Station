#include "big_data.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "sd_card.h"
#include "web_server.h"
#include <stdio.h>

const char *TAG = "main";

static void init_my_spi() {
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = 4,
        .miso_io_num = 5,
        .sclk_io_num = 6,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting SPI");
    init_my_spi();

    ESP_LOGI(TAG, "Starting Data Collection");
    xTaskCreate(generate_big_data_task, "DataGen", 4096, NULL, 0, NULL);

    ESP_LOGI(TAG, "Starting Wi-Fi as Access Point");
    wifi_init_softap();

    ESP_LOGI(TAG, "Starting SD Card");
    test_sd_card();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(60000));
    }
}
