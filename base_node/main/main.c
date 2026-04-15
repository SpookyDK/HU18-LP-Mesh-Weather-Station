#include "NEO_6M_UART.h"
#include "big_data.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "portmacro.h"
#include "sd_card.h"
#include "web_server.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/_timeval.h>
#include <sys/time.h>
#include <time.h>

const char *TAG = "main";

static inline void init_my_spi() {
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
    // This is a static time for testing
    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
    tzset();
    struct tm tm;
    strptime(__DATE__ " " __TIME__, "%b %d %Y %H:%M:%S", &tm);
    struct timespec ts = {.tv_sec = mktime(&tm), .tv_nsec = 0};
    clock_settime(CLOCK_REALTIME, &ts);

    ESP_LOGI(TAG, "Starting SPI");
    init_my_spi();

    ESP_LOGI(TAG, "Starting Data Collection");
    start_receive_task();

    // For no apparant reason this has to be called after receive task
    ESP_LOGI(TAG, "Starting SD Card");
    init_sd_card();

    ESP_LOGI(TAG, "Starting Wi-Fi as Access Point");
    wifi_init_softap();

    ESP_LOGI(TAG, "Starting GPS task");
    xTaskCreate(gps_task, "gpsTask", 4096, NULL, 0, NULL);

    // Dont use It will save the loaded packet again, and im not refactoring that much
    // read_last_packet();

    while (1) {
        // heap_caps_print_heap_info(MALLOC_CAP_8BIT);
        vTaskDelay(pdMS_TO_TICKS(60000));
    }
}
