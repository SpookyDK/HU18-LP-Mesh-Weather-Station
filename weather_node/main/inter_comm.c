#include "esp_bit_defs.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_random.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/semphr.h"
#include "i2c_tasks.h"
#include "inter_comm.h"
#include "led_driver.h"
#include "lora.h"
#include "moist_soil.h"
#include "nvs_flash.h"
#include "packet_def.h"
#include "pcnt_sensor.h"
#include "pin_config.h"
#include "portmacro.h"
#include "tempeture.h"
#include <limits.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static const char *TAG = "InComm";
static TaskHandle_t comm_taskhandle;
static TaskHandle_t worker_taskhandle;

/// =======================================================
///     The configuration of the node
/// =======================================================

#define DUTY_CYCLES_MS (900 * 1000)

static uint8_t NETWORK_ID = 0;
static uint8_t NODE_ID = 0;
static uint8_t PACKET_ID = 0;
static uint16_t EXPECTING_PAIRING = 0; // So nonce

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
        // ESP_LOGW(TAG, "Failed to load Config; Generating values...");
        /* NETWORK_ID = 1;
        esp_fill_random(&NODE_ID, sizeof(NODE_ID));
        ESP_LOGI(TAG, "Network ID: '%d'  Node ID: '%d'", NETWORK_ID, NODE_ID);
        save_config_nvs(); */
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

/// =======================================================
///     This section covers the preperation of the data
/// =======================================================

extern int32_t gps_shared_longitude; // I would like to remove these but i dont mind them that much
extern int32_t gps_shared_latitude;

static void prepare_packet(full_packet_t *packet) {
    // A base value of 1 means its a payload package
    packet->head.flags = 1;

    // The retreivel and insertion of the data
    sensor_payload_t *payload = &packet->payload;
    payload->longitude = gps_shared_longitude;
    payload->latitude = gps_shared_latitude;
    if (get_air_reads(payload) != ESP_OK)
        packet->head.flags |= FLAG_DHT;
    if (get_soil_temp(payload) != ESP_OK)
        packet->head.flags |= FLAG_DS18B20;
    if (get_moist_read(payload) != ESP_OK)
        packet->head.flags |= FLAG_MOIST;
    if (get_pres_read(payload) != ESP_OK)
        packet->head.flags |= FLAG_PRESSURE;
    get_light_read(payload); // It does return an error state, but it does not have a flag
    get_rain_read(payload);
    get_wind_read(payload);
    if (get_power_read(payload, POWER_ADDR_SOLAR) != ESP_OK)
        packet->head.flags |= FLAG_SOLAR;
    get_power_read(payload, POWER_ADDR_BATTERY);

    // Packet Id increment
    nvs_handle_t nvs;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (ret == ESP_OK) {
        nvs_get_u8(nvs, NVS_PACKET_ID_KEY, &PACKET_ID);
    }
    packet->head.packet_id = PACKET_ID++;
    if (ret == ESP_OK) {
        nvs_set_u8(nvs, NVS_PACKET_ID_KEY, PACKET_ID);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
}

static full_packet_t data_packet = {0};

static void packet_worker() {
    const TickType_t send_interval = pdMS_TO_TICKS(DUTY_CYCLES_MS);
    data_packet.head.hop_count = PACKET_TIME_TO_LIVE;
    data_packet.head.header = PACKET_HEADER_VALUE;
    const char *name = pcTaskGetName(NULL);

    /* ESP_LOGI(name, "Going to sleep.");
    vTaskSuspend(NULL); // Get resumed externally to functionally begin
    ESP_LOGI(name, "Rizzle"); */

    uint32_t notif = 0;
    TickType_t last_send_tick = xTaskGetTickCount();
    while (1) {
        xTaskDelayUntil(&last_send_tick, send_interval);
        xTaskNotifyWait(0, 0, &notif, 0);
        if (!notif) {
            continue;
        }
        data_packet.head.network_id = NETWORK_ID;
        data_packet.head.orig_node_id = NODE_ID;
        ESP_LOGI(name, "Begun prepping packet...");
        prepare_packet(&data_packet);
        ESP_LOGI(name, "Notifying comm task...");
        xTaskNotify(comm_taskhandle, 0, eNoAction);
    }
}

/// ===================================
///     Duplicate packet protection
/// ===================================

#define CACHE_SIZE 16
#define CACHE_MASK (CACHE_SIZE - 1)

typedef struct {
    uint8_t packet_id;
    uint8_t node_id;
    // uint16_t nonce; might be needed dependant on how the nonce is implemented
} packet_signature_t;

static packet_signature_t packet_cache[CACHE_SIZE];
static uint8_t write_idx = 0;

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

/// =======================
///     Pairing Request
/// =======================

static pairing_packet_t pairing_packet = {0};
static void pair_to_network() {
    pairing_packet.head.hop_count = PACKET_TIME_TO_LIVE;
    pairing_packet.head.flags = 0b10;
    pairing_packet.head.header = PACKET_HEADER_VALUE;
    esp_fill_random(&EXPECTING_PAIRING, sizeof(uint16_t));
    pairing_packet.nonce = EXPECTING_PAIRING;

    block_if_receiving();
    lora_send_packet((uint8_t *)&pairing_packet, sizeof(pairing_packet_t));
    ESP_LOGI(TAG, "Sent pairing request, with nonce='%04X'", EXPECTING_PAIRING);
}

/// ==============================
///     Packet interpretation
/// ==============================

#define LORA_BUF_MAX_LEN 256 // This is the maximum the lora module can contain
static uint8_t lora_buf[LORA_BUF_MAX_LEN];
static int lora_buf_len = 0;
static void receive_something() {
    lora_buf_len = lora_receive_packet(lora_buf, LORA_BUF_MAX_LEN);
    if (lora_buf_len <= sizeof(packet_header_t)) {
        ESP_LOGI(TAG, "Lora Packet too small, dropped");
        return;
    }
    packet_header_t head = {0};
    memcpy(&head, lora_buf, sizeof(packet_header_t));
    if (head.header != PACKET_HEADER_VALUE) {
        ESP_LOGI(TAG, "Invalid packet header, received='%02X', dropped", head.header);
        return;
    }

    switch (head.flags & 0b11) {
    case 0b01: {
        ESP_LOGI(TAG, "Its a packet");
        full_packet_t temp_packet = {0};
        memcpy(&temp_packet, lora_buf, sizeof(full_packet_t));
        if (temp_packet.head.network_id != NETWORK_ID) {
            ESP_LOGI(TAG, "Ignoring received Packet from other Network '%d'", temp_packet.head.network_id);
            break;
        }
        if (--temp_packet.head.hop_count <= 0) {
            ESP_LOGI(TAG, "Ignoring received packet with low hop_count");
            break;
        }
        if (temp_packet.head.orig_node_id == NODE_ID) {
            ESP_LOGI(TAG, "Ignoring received Own Packet id='%d'", temp_packet.head.packet_id);
            break;
        }
        if (is_duplicate(temp_packet.head)) {
            ESP_LOGI(TAG, "Ignoring received cached packet id='%d' node='%d'", temp_packet.head.packet_id, temp_packet.head.orig_node_id);
            break;
        }
        ws2812_on_blue();
        vTaskDelay(pdMS_TO_TICKS(1000));
        block_if_receiving();
        lora_send_packet((uint8_t *)&temp_packet, lora_buf_len);
        ESP_LOGI(TAG, "Bouncing packet from node='%d' id='%d'", temp_packet.head.orig_node_id, temp_packet.head.packet_id);
        break;
    }
    case 0b10: {
        ESP_LOGI(TAG, "Its a pairing thingy...");
        pairing_packet_t pkt = {0};
        memcpy(&pkt, lora_buf, sizeof(pairing_packet_t));
        if (EXPECTING_PAIRING && pkt.head.network_id && pkt.nonce == EXPECTING_PAIRING) {
            ESP_LOGI(TAG, "Pairing packet has been consumed");
            NODE_ID = pkt.head.packet_id;
            NETWORK_ID = pkt.head.network_id;

            save_config_nvs();
            EXPECTING_PAIRING = 0;
            xTaskNotify(worker_taskhandle, 1, eSetValueWithOverwrite);

            ESP_LOGI(TAG, "Returning acknowledgement packet");
            pkt.nonce = 0;
            pkt.head.orig_node_id = NODE_ID;
            pkt.head.packet_id = 0;
            block_if_receiving();
            lora_send_packet((uint8_t *)&pkt, sizeof(pairing_packet_t));
            break;
        }
        if (NETWORK_ID && pkt.head.network_id != 0 && NETWORK_ID != pkt.head.network_id) {
            ESP_LOGI(TAG, "Dropped due to pairing response from other network");
            break;
        }
        if (pkt.head.orig_node_id == NODE_ID && pkt.nonce == 0xffff) {
            ESP_LOGI(TAG, "I was disconnected :(");
            xTaskNotify(worker_taskhandle, 0, eSetValueWithOverwrite);
            NODE_ID = 0;
            NETWORK_ID = 0;
            pair_to_network();
            break;
        }
        if (--pkt.head.hop_count <= 0) {
            ESP_LOGI(TAG, "Dropped due to low hop count");
            break;
        }
        block_if_receiving();
        lora_send_packet((uint8_t *)&pkt, sizeof(pairing_packet_t));
        ESP_LOGI(TAG, "Pairing packet not for me, bouncing it away");
        break;
    }
    default: {
        ESP_LOGW(TAG, "Unrecognized packet type %02b", head.flags & 0b11);
        break;
    }
    }
}

/// ============================
///     Main listening loop
/// ============================

void inter_comm_task(void *arg) {
    comm_taskhandle = xTaskGetCurrentTaskHandle();

    lora_init();
    lora_dump_registers();

    xTaskCreate(packet_worker, "workerTask", 4096, NULL, 10, &worker_taskhandle);
    vTaskDelay(pdMS_TO_TICKS(200));

    load_config_nvs();
    if (NODE_ID && NETWORK_ID) {
        xTaskNotify(worker_taskhandle, 1, eSetValueWithOverwrite);
    } else {
        ESP_LOGI(TAG, "Node is not connected to network");
        pair_to_network();
    }
    const TickType_t interval = pdMS_TO_TICKS(30000);
    TickType_t last_interval = xTaskGetTickCount();
    ws2812_init();

    while (1) {
        lora_receive();
        if (lora_received()) {
            receive_something();
        }

        if (xTaskNotifyWait(0, ULONG_MAX, NULL, 0) == pdPASS) {
            ws2812_on_green();
            block_if_receiving();
            lora_send_packet((uint8_t *)&data_packet, sizeof(full_packet_t));
            ESP_LOGI(TAG, "Sent Packet with id: '%d'", data_packet.head.packet_id);
        }

        if (EXPECTING_PAIRING && xTaskGetTickCount() > interval + last_interval) {
            ESP_LOGI(TAG, "Sending Pairing request again, with uid='%04X'", pairing_packet.nonce);
            block_if_receiving();
            lora_send_packet((uint8_t *)&pairing_packet, sizeof(pairing_packet_t));
            last_interval = xTaskGetTickCount();
        }
        vTaskDelay(10);
        ws2812_off();
    }
    ESP_LOGE(TAG, "Unexpectedly left comm task");
    vTaskDelete(NULL);
}
