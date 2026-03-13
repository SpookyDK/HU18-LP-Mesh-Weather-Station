#include "esp_log.h"
#include "esp_mac.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "lora.h"
#include "packet_def.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define DUTY_CYCLES_MS 30000
#include "NEO_6M_UART.h"
#include "i2c_tasks.h"
#include "moist_soil.h"
#include "tempeture.h"
#include "wind_sensor.h"

static const char *TAG = "main";

extern int32_t gps_shared_longitude;
extern int32_t gps_shared_latitude;
extern uint8_t dht_shared_air_humidity;
extern int16_t dht_shared_air_tempeture;
extern int16_t ds18b20_shared_soil_tempeture[ONEWIRE_MAX_DEVS];
extern uint8_t moist_shared_percentage;
extern int16_t bmp_shared_pressure;
extern uint16_t tsl_shared_lux;
extern uint16_t wind_shared_rpm;

sensor_payload_t payload = {0};
static void prepare_payload() {
    payload.longitude = gps_shared_longitude;
    payload.latitude = gps_shared_latitude;
    payload.air_humidity = dht_shared_air_humidity;
    payload.air_tempeture = dht_shared_air_tempeture;
    for (int i = 0; i < ONEWIRE_MAX_DEVS; i++) {
        payload.soil_tempeture[i] = ds18b20_shared_soil_tempeture[i];
    }
    payload.soil_moisture = moist_shared_percentage;
    payload.pressure = bmp_shared_pressure;
    payload.lux = tsl_shared_lux;
    payload.precipitation = 0; // TODO: Missing sensor
    payload.wind_speed = wind_shared_rpm;
}

static uint8_t payload_string[sizeof(sensor_payload_t) * 2 + 1] = {0};
static void payload_to_string(void) {
    uint8_t *ptr = (uint8_t *)&payload;
    for (int i = 0; i < sizeof(payload); i++) {
        sprintf((char *)&payload_string[i * 2], "%02x", ptr[i]);
    }
}

static uint16_t crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

static void transmit_task(void *p) {
    lora_init();
    lora_set_frequency(868e6);
    lora_set_spreading_factor(7);
    lora_set_bandwidth(125000);
    lora_set_coding_rate(5);
    lora_set_preamble_length(8);
    lora_set_sync_word(0x12);
    lora_set_tx_power(2);
    lora_enable_crc();
    lora_dump_registers();

    // Wait a little to hope some data has been generated
    vTaskDelay(pdMS_TO_TICKS(10000));

    while (1) {
        prepare_payload();
        payload_to_string();
        ESP_LOGI("TXtask", "The payload: '%s'", payload_string);
        lora_send_packet(payload_string, sizeof(payload_string));
        ESP_LOGI("TXtask", "Sent Pakcet");
        vTaskDelay(pdMS_TO_TICKS(DUTY_CYCLES_MS));
    }
    ESP_LOGW("TXtask", "Left task");
}

static void print_runtime_stats(void) {
    char buffer[1024];

    vTaskGetRunTimeStats(buffer);

    printf("Task Name\tRuntime\t\tCPU %%\n");
    printf("%s", buffer);
}

void app_main(void) {

    ESP_LOGI(TAG, "Starting Tempeture Task");
    xTaskCreate(temp_task, "tempTask", 4096, (void *)DUTY_CYCLES_MS, 0, NULL);

    ESP_LOGI(TAG, "Starting Soil Moisture Reading Task");
    xTaskCreate(moist_task, "moistTask", 4096, (void *)DUTY_CYCLES_MS, 0, NULL);

    ESP_LOGI(TAG, "Starting Accelerometer Task");
    xTaskCreate(barometer_task, "barTask", 4096, (void *)DUTY_CYCLES_MS, 0, NULL);

    ESP_LOGI(TAG, "Starting Light Sensor Task");
    xTaskCreate(light_sensor_task, "lightTask", 4096, (void *)DUTY_CYCLES_MS, 0, NULL);

    ESP_LOGI(TAG, "Starting Wind Sensor Task");
    xTaskCreate(wind_task, "windTask", 4096, (void *)DUTY_CYCLES_MS, 0, NULL);

    ESP_LOGI(TAG, "Starting GPS Task");
    xTaskCreate(gps_task, "gpsTask", 4096, NULL, 0, NULL);

    ESP_LOGI(TAG, "Starting Transmitter Task");
    xTaskCreate(transmit_task, "txTask", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Finished starting all tasks");
    while (1) {
        // print_runtime_stats();
        vTaskDelay(pdMS_TO_TICKS(DUTY_CYCLES_MS));
    }
}
