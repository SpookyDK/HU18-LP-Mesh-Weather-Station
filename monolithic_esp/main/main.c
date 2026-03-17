#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"
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
#include "pcnt_sensor.h"
#include "tempeture.h"

static const char *TAG = "main";

#define PACKET_TIME_TO_LIVE 5

extern int32_t gps_shared_longitude;
extern int32_t gps_shared_latitude;
extern uint8_t dht_shared_air_humidity;
extern int16_t dht_shared_air_tempeture;
extern int16_t ds18b20_shared_soil_tempeture[4];
extern uint8_t moist_shared_percentage;
extern int16_t bmp_shared_pressure;
extern uint16_t tsl_shared_lux;
extern uint16_t wind_shared_speed;

sensor_payload_t payload = {0};
static void prepare_payload() {
    payload.longitude = gps_shared_longitude;
    payload.latitude = gps_shared_latitude;
    payload.air_humidity = dht_shared_air_humidity;
    payload.air_tempeture = dht_shared_air_tempeture;
    for (int i = 0; i < 4; i++) {
        payload.soil_tempeture[i] = ds18b20_shared_soil_tempeture[i];
    }
    payload.soil_moisture = moist_shared_percentage;
    payload.pressure = bmp_shared_pressure;
    payload.lux = tsl_shared_lux;
    payload.precipitation = get_rain();
    payload.wind_speed = wind_shared_speed;
}

packet_header_t header = {0};
static void prepare_header() {
    header.network_id = 1;
    header.orig_node_id = 1;
    header.packet_id = esp_random() % sizeof(header.packet_id);
    header.hop_count = 5;
    header.flags = 1;
}

full_packet_t packet = {0};
static void prepare_packet() {
    prepare_header();
    packet.head = header;
    prepare_payload();
    packet.payload = payload;
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
        prepare_packet();
        block_if_receiving();
        lora_send_packet((uint8_t *)&packet, sizeof(packet));
        ESP_LOGI("TXtask", "Sent Packet with length: '%d'", sizeof(packet));
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

    ESP_LOGI(TAG, "Starting Barometer Task");
    xTaskCreate(barometer_task, "barTask", 4096, (void *)DUTY_CYCLES_MS, 0, NULL);

    ESP_LOGI(TAG, "Starting Light Sensor Task");
    xTaskCreate(light_sensor_task, "lightTask", 4096, (void *)DUTY_CYCLES_MS, 0, NULL);

    ESP_LOGI(TAG, "Starting Wind and Rain Sensor Task");
    xTaskCreate(pcnt_task, "windTask", 4096, (void *)DUTY_CYCLES_MS, 0, NULL);

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
