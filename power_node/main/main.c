#include "NEO_6M_UART.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "inter_comm.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "pin_config.h"
#include "portmacro.h"
#include <i2c_tasks.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "main";

static void print_runtime_stats(void) {
    char buffer[1024];
    vTaskGetRunTimeStats(buffer);
    printf("Task Name\tRuntime\t\tCPU %%\n");
    printf("%s", buffer);
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    spi_bus_config_t bus = {
        .miso_io_num = SPI_MISO_GPIO,
        .mosi_io_num = SPI_MOSI_GPIO,
        .sclk_io_num = SPI_SCK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, 0));

    // ESP_LOGI(TAG, "Starting GPS Task");
    // xTaskCreate(gps_task, "gpsTask", 4096, NULL, 0, NULL);

    // ESP_LOGI(TAG, "Starting Communication Task");
    // xTaskCreate(inter_comm_task, "commTask", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Finished starting all tasks");
    sensor_payload_t payload = {0};
    while (1) {
        get_power_read(&payload, 0x41);
        // print_runtime_stats();
        // heap_caps_print_heap_info(MALLOC_CAP_8BIT);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
