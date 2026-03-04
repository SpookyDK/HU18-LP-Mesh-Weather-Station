#include "NEO_6M_UART.h"
#include "dht11.h"
#include "driver/gpio.h"
#include "ds18b20.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_log_level.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "inttypes.h"
#include "moist_soil.h"
#include "onewire_bus_impl_rmt.h"
#include "onewire_device.h"
#include "onewire_types.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define CONFIG_DHT11_PIN GPIO_NUM_22
#define CONFIG_CONNECTION_TIMEOUT 5

#define ONEWIRE_BUS_GPIO 12
#define ONEWIRE_MAX_DEVS 4

static const char *TAG = "main";

onewire_bus_handle_t bus = NULL;
int8_t ds18b20_device_num = 0;
ds18b20_device_handle_t ds18b20s[ONEWIRE_MAX_DEVS];
onewire_device_iter_handle_t iter = NULL;
onewire_device_t next_onewire_device;
esp_err_t search_result = ESP_OK;

void print_runtime_stats(void) {
    char buffer[1024];

    vTaskGetRunTimeStats(buffer);

    printf("Task Name\tRuntime\t\tCPU %%\n");
    printf("%s", buffer);
}

void TEMP_TASK() {
    int16_t dht11_tempeture, dht11_humidity, ds18b20_temperature;
    while (true) {
        if (dht_read_data(CONFIG_DHT11_PIN, &dht11_humidity, &dht11_tempeture) == ESP_OK) {
            ESP_LOGI("DHT11", "[Temperature]> %d", dht11_tempeture);
            ESP_LOGI("DHT11", "[Humidity]> %d", dht11_humidity);
        } else {
            ESP_LOGW("DHT11", "Failed to read");
        }

        ESP_ERROR_CHECK(ds18b20_trigger_temperature_conversion_for_all(bus));
        for (int i = 0; i < ds18b20_device_num; i++) {
            ESP_ERROR_CHECK(ds18b20_get_temperature(ds18b20s[i], &ds18b20_temperature));
            ESP_LOGI("DS18B20", "[%d] [Temperature]> %.2f", i, ds18b20_temperature / 16.0f);
        }
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}

void app_main(void) {
    onewire_bus_config_t bus_config = {
        .bus_gpio_num = ONEWIRE_BUS_GPIO,
        .flags = {.en_pull_up = true},
    };
    onewire_bus_rmt_config_t rmt_config = {.max_rx_bytes = 10};
    ESP_ERROR_CHECK(onewire_new_bus_rmt(&bus_config, &rmt_config, &bus));
    ESP_LOGI(TAG, "1-Wire bus iniated");

    ESP_ERROR_CHECK(onewire_new_device_iter(bus, &iter));
    ESP_LOGI(TAG, "Device iterrator created, start search...");
    do {
        search_result = onewire_device_iter_get_next(iter, &next_onewire_device);
        if (search_result == ESP_OK) {
            ds18b20_config_t ds_config = {};
            onewire_device_address_t address;
            if (ds18b20_new_device_from_enumeration(&next_onewire_device, &ds_config, &ds18b20s[ds18b20_device_num]) ==
                ESP_OK) {
                ds18b20_get_device_address(ds18b20s[ds18b20_device_num], &address);
                ESP_LOGI(TAG, "Found a ds18b20[%d], address: %016llx", ds18b20_device_num, address);
                ds18b20_device_num++;
                if (ds18b20_device_num >= ONEWIRE_MAX_DEVS) {
                    ESP_LOGI(TAG, "Max 1-Wire devices Reached, stop searhing...");
                    break;
                }
            } else {
                ESP_LOGI(TAG, "Found unkown device, address: %016llx", next_onewire_device.address);
            }
        }
    } while (search_result != ESP_ERR_NOT_FOUND);
    ESP_ERROR_CHECK(onewire_del_device_iter(iter));
    ESP_LOGI(TAG, "Searching over, %d ds18b20 devices found", ds18b20_device_num);

    xTaskCreate(TEMP_TASK, "TEMPTask", 4096, NULL, 0, NULL);

    xTaskCreate(moist_task, "moistTask", 4096, NULL, 0, NULL);

    xTaskCreate(gpsTask, "gpsTask", 4096, NULL, 0, NULL);

    while (1) {
        // print_runtime_stats();
        vTaskDelay(pdMS_TO_TICKS(60000));
    }
}
