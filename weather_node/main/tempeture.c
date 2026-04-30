#include "dht11.h"
#include "ds18b20.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "onewire_bus_impl_rmt.h"
#include "onewire_device.h"
#include "onewire_types.h"
#include "packet_def.h"
#include "pin_config.h"
#include "portmacro.h"
#include "tempeture.h"
#include <stdbool.h>
#include <stdint.h>

static const char *TAG = "temp_sensor";

esp_err_t get_air_reads(sensor_payload_t *packet) {
    if (!dht11_exists(CONFIG_DHT11_PIN)) {
        ESP_LOGW(TAG, "DHT11 does not exist");
        return ESP_FAIL;
    }
    int16_t dht_humd, dht_temp;
    if (dht_read_data(CONFIG_DHT11_PIN, &dht_humd, &dht_temp) == ESP_OK) {
        packet->air_humidity = dht_humd;
        packet->air_tempeture = dht_temp;
    } else {
        ESP_LOGW(TAG, "DHT11> Failed to read data");
        return ESP_FAIL;
    }
    return ESP_OK;
}

static onewire_bus_handle_t bus = NULL;
static int8_t ds18b20_device_num = 0;
static ds18b20_device_handle_t ds18b20s[ONEWIRE_MAX_DEVS];

static bool temp_initated = false;

static void init_tempeture() {
    esp_err_t search_result;
    onewire_device_t next_onewire_device;
    onewire_device_iter_handle_t iter = NULL;
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
            if (ds18b20_new_device_from_enumeration(&next_onewire_device, &ds_config, &ds18b20s[ds18b20_device_num]) == ESP_OK) {
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
    ESP_LOGI(TAG, "Search over, %d ds18b20 devices found", ds18b20_device_num);
    temp_initated = true;
}

esp_err_t get_soil_temp(sensor_payload_t *packet) {
    if (!temp_initated) {
        ESP_LOGW(TAG, "Temperature module not initated, initiating...");
        init_tempeture();
    }
    if (ds18b20_device_num != 4) {
        ESP_LOGW(TAG, "Not correct amount of ds18b20 devices, there is %d", ds18b20_device_num);
        return ESP_FAIL;
    }
    int16_t temp_temp;
    if (ds18b20_trigger_temperature_conversion_for_all(bus) != ESP_OK)
        return ESP_FAIL;
    for (int i = 0; i < ds18b20_device_num; i++) {
        if (ds18b20_get_temperature(ds18b20s[i], &temp_temp) == ESP_OK) {
            packet->soil_tempeture[i] = temp_temp;
            // ESP_LOGI("DS18B20", "[%d] [Temperature]> %.2f, Shared> %d", i, (double)(temp_temp / 16.0f), temp_temp);
        }
    }
    if (ds18b20_device_num != 4) {
        ESP_LOGW(TAG, "DS18> Not found ideal number");
        return ESP_ERR_NOT_FOUND;
    }
    return ESP_OK;
}
