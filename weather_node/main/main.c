#include "NEO_6M_UART.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_random.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "i2c_tasks.h"
#include "lora.h"
#include "moist_soil.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "packet_def.h"
#include "pcnt_sensor.h"
#include "pin_config.h"
#include "portmacro.h"
#include "tempeture.h"
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "main";

#define DUTY_CYCLES_MS (900 * 1000)
#define PACKET_TIME_TO_LIVE 3
#define NVS_NAMESPACE "lora_conf"
#define NVS_NETWORK_KEY "network_key"
#define NVS_NODE_KEY "node_key"

static uint8_t NETWORK_ID = 1;
static uint8_t NODE_ID = 1;
static uint8_t PACKET_ID = 0;

extern int32_t gps_shared_longitude;
extern int32_t gps_shared_latitude;
extern uint8_t dht_shared_air_humidity;
extern int16_t dht_shared_air_tempeture;
extern int16_t ds18b20_shared_soil_tempeture[4];
extern uint8_t moist_shared_percentage;
extern int16_t bmp_shared_pressure;
extern uint16_t tsl_shared_spectrum;
extern uint16_t wind_shared_speed;
extern uint8_t power_shared_solar_production;
extern int8_t power_shared_bat_volatage;

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
    payload.spectrum = tsl_shared_spectrum;
    payload.precipitation = get_rain();
    payload.wind_speed = wind_shared_speed;
    payload.solar_output = power_shared_solar_production;
    payload.bat_voltage = power_shared_bat_volatage;
}

extern bool dht_shared_air_tempeture_status;
extern bool ds18b20_shared_status;
extern bool moist_shared_status;
extern bool bmp_shared_status;
extern bool power_shared_solar_state;

packet_header_t header = {0};
static void prepare_header() {
    header.network_id = NETWORK_ID;
    header.orig_node_id = NODE_ID;
    header.packet_id = PACKET_ID++;
    header.hop_count = PACKET_TIME_TO_LIVE;
    header.flags = 1 ^ (dht_shared_air_tempeture_status << 2) ^ (ds18b20_shared_status << 3) ^ (moist_shared_status << 4) ^
                   (bmp_shared_status << 6) ^ (power_shared_solar_state << 7);
}

full_packet_t packet = {0};
static void prepare_packet() {
    prepare_header();
    packet.head = header;
    prepare_payload();
    packet.payload = payload;
}

#define CACHE_SIZE 16
#define CACHE_MASK (CACHE_SIZE - 1)

typedef struct {
    uint8_t packet_id;
    uint8_t node_id;
} packet_signature_t;

packet_signature_t packet_cache[CACHE_SIZE];
uint8_t write_idx = 0;

static bool is_duplicate(packet_header_t head) {
    for (int i = 0; i < CACHE_SIZE; i++) {
        if (packet_cache[i].node_id == head.orig_node_id && packet_cache[i].packet_id == head.packet_id) {
            return true;
        }
    }
    packet_cache[write_idx].packet_id = head.packet_id;
    packet_cache[write_idx].node_id = head.orig_node_id;
    write_idx = (write_idx + 1) & CACHE_MASK;
    return false;
}

static void communication_task(void *p) {
    esp_err_t ret;

    spi_bus_config_t bus = {
        .miso_io_num = LORA_MISO_GPIO,
        .mosi_io_num = LORA_MOSI_GPIO,
        .sclk_io_num = LORA_SCK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
    };

    ret = spi_bus_initialize(SPI2_HOST, &bus, 0);
    assert(ret == ESP_OK);

    lora_init();
    lora_dump_registers();

    // Wait a little to hope some data has been generated
    vTaskDelay(pdMS_TO_TICKS(10000));

    TickType_t last_send = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(DUTY_CYCLES_MS);
    const char *tag = "comTask";
    int buffer_len;
    uint8_t buffer[256] = {0};
    full_packet_t temp_packet = {0};

    while (1) {
        lora_receive();
        // Check for incomming packets
        if (lora_received()) {
            buffer_len = lora_receive_packet(buffer, sizeof(buffer));
            if (buffer_len != sizeof(full_packet_t)) {
                ESP_LOGW(tag, "Unknown packet, with len='%d'", buffer_len);
                continue;
            }
            memcpy(&temp_packet, buffer, buffer_len);
            if (temp_packet.head.network_id != NETWORK_ID) {
                ESP_LOGI(tag, "Ignoring received Packet from other Network '%d'", temp_packet.head.network_id);
                continue;
            }
            if (--temp_packet.head.hop_count <= 0) {
                ESP_LOGI(tag, "Ignoring received packet with low hop_count");
                continue;
            }
            if (temp_packet.head.orig_node_id == NODE_ID) {
                ESP_LOGI(tag, "Ignoring received Own Packet id='%d'", temp_packet.head.packet_id);
                continue;
            }
            if (is_duplicate(temp_packet.head)) {
                ESP_LOGI(tag, "Ignoring received cached packet id='%d' node='%d'", temp_packet.head.packet_id,
                         temp_packet.head.orig_node_id);
                continue;
            }

            block_if_receiving();
            lora_send_packet((uint8_t *)&temp_packet, buffer_len);
            ESP_LOGI(tag, "Bouncing packet from node='%d' id='%d'", temp_packet.head.orig_node_id, temp_packet.head.packet_id);
        }

        // Is it time to send own packet?
        if ((xTaskGetTickCount() - last_send) >= interval) {
            prepare_packet();
            block_if_receiving();
            lora_send_packet((uint8_t *)&packet, sizeof(packet));
            ESP_LOGI("TXtask", "Sent Packet with id: '%d'", packet.head.packet_id);
            last_send += interval;
            lora_receive();
        }

        // Nominal wait to not work too much
        vTaskDelay(10);
    }
    ESP_LOGW("TXtask", "Left task");
}

static void print_runtime_stats(void) {
    char buffer[1024];
    vTaskGetRunTimeStats(buffer);
    printf("Task Name\tRuntime\t\tCPU %%\n");
    printf("%s", buffer);
}

static void save_config_nvs(void) {
    ESP_LOGI(TAG, "Saving Config to NVS");
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open Config");
        return;
    }
    if (nvs_set_u8(handle, NVS_NETWORK_KEY, NETWORK_ID) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to write '%s' with '%d'", NVS_NETWORK_KEY, NETWORK_ID);
    }
    if (nvs_set_u8(handle, NVS_NODE_KEY, NODE_ID) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to write '%s' with '%d'", NVS_NODE_KEY, NODE_ID);
    }
    ESP_LOGI(TAG, "Saved: Network_ID='%d'  Node_ID='%d'", NETWORK_ID, NODE_ID);
    nvs_close(handle);
}

static void load_config_nvs(void) {
    ESP_LOGI(TAG, "Loading Config from NVS");
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to load Config; Generating values...");
        NETWORK_ID = 1;
        esp_fill_random(&NODE_ID, sizeof(NODE_ID));
        ESP_LOGI(TAG, "Network ID: '%d'  Node ID: '%d'", NETWORK_ID, NODE_ID);
        save_config_nvs();
        return;
    }
    if (nvs_get_u8(handle, NVS_NETWORK_KEY, &NETWORK_ID) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read '%s' with '%d'", NVS_NETWORK_KEY, NETWORK_ID);
    }
    if (nvs_get_u8(handle, NVS_NODE_KEY, &NODE_ID) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read '%s' with '%d'", NVS_NODE_KEY, NODE_ID);
    }
    ESP_LOGI(TAG, "Loaded: Network_ID='%d'  Node_ID='%d'", NETWORK_ID, NODE_ID);
    nvs_close(handle);
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    load_config_nvs();

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

    ESP_LOGI(TAG, "Starting power Task");
    xTaskCreate(power_sensor_task, "powerTask", 4096, (void *)DUTY_CYCLES_MS, 0, NULL);

    ESP_LOGI(TAG, "Starting GPS Task");
    xTaskCreate(gps_task, "gpsTask", 4096, NULL, 0, NULL);

    ESP_LOGI(TAG, "Starting Communication Task");
    xTaskCreate(communication_task, "TxRxTask", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Finished starting all tasks");
    while (1) {
        // print_runtime_stats();
        vTaskDelay(pdMS_TO_TICKS(DUTY_CYCLES_MS));
    }
}
