#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#define DUTY_CYCLES_MS 30000
#include "NEO_6M_UART.h"
#include "i2c_tasks.h"
#include "moist_soil.h"
#include "tempeture.h"

static const char *TAG = "main";

extern int32_t gps_shared_longitude;
extern int32_t gps_shared_latitude;
extern uint8_t dht_shared_air_humidity;
extern int16_t dht_shared_air_tempeture;
extern int16_t ds18b20_shared_soil_tempeture[ONEWIRE_MAX_DEVS];
extern uint8_t moist_shared_percentage;
extern int16_t bmp_shared_pressure;
extern uint16_t tsl_shared_lux;

/**
 * Main Struct
 * @version 3
 * @param longitude Is scaled with 1e-7
 * @param latitude Is scaled with 1e-7
 * @param air_humidity From 0 to 100%RH
 * @param air_tempeture Is scaled with 10
 * @param soil_tempeture[4] Is scaled with 16
 * @param soil_moisture From 0 to 100% where 0 is our measured lowest moisture and 100% is most
 * @param pressure The diff from 100000 Pa
 * @param lux Lux
 * @param precipitation In units of 100 micro meters per hour
 * @param wind_speed In 0.1 m/s
 */
typedef struct __attribute__((packed)) {
    int32_t longitude;
    int32_t latitude;
    uint8_t air_humidity;
    int16_t air_tempeture;
    int16_t soil_tempeture[4];
    uint8_t soil_moisture;
    int16_t pressure;
    uint16_t lux;
    uint16_t precipitation; // TODO: Missing Sensor
    uint16_t wind_speed;    // TODO: Missing Sensor
} sensor_payload_t;

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
    payload.wind_speed = 0;    // TODO: Missing sensor
}

static char payload_string[sizeof(sensor_payload_t) * 2 + 1] = {0};
static void payload_to_string(void) {
    ESP_LOGI(TAG, "temp %d", payload.air_tempeture);
    uint8_t *ptr = (uint8_t *)&payload;
    for (int i = 0; i < sizeof(payload); i++) {
        sprintf(&payload_string[i * 2], "%02x", ptr[i]);
    }
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

    ESP_LOGI(TAG, "Starting GPS Task");
    xTaskCreate(gps_task, "gpsTask", 4096, NULL, 0, NULL);

    ESP_LOGI(TAG, "Finished starting all tasks");
    while (1) {
        // print_runtime_stats();
        prepare_payload();
        payload_to_string();
        ESP_LOGI(TAG, "The payload: '%s'", payload_string);
        vTaskDelay(pdMS_TO_TICKS(DUTY_CYCLES_MS));
    }
}
