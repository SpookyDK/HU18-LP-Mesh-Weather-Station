#include "big_data.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "hal/gpio_types.h"
#include "lora.h"
#include "packet_def.h"
#include "pin_config.h"
#include "portmacro.h"
#include "sd_card.h"
#include "site_content.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/_timeval.h>
#include <sys/time.h>
#include <time.h>

static const char *TAG = "RxTask";

static TaskHandle_t lora_task_handle = NULL;

// This should be set somewhere else, but prototype it fine
static uint8_t NETWORK_ID = 1;

static void IRAM_ATTR lora_isr_handler(void *arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(lora_task_handle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

void init_dio0_interrupt() {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .pin_bit_mask = (1ULL << LORA_DIO0_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = 1,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(LORA_DIO0_GPIO, lora_isr_handler, NULL);
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

/// ==============================
///     Packet interpretation
/// ==============================

#define LORA_BUF_MAX_LEN 256 // This is the maximum the lora module can contain
static uint8_t lora_buf[LORA_BUF_MAX_LEN];
static int lora_buf_len = 0;

static void receive_something() {
    lora_buf_len = lora_receive_packet(lora_buf, LORA_BUF_MAX_LEN);
    if (lora_buf_len < sizeof(packet_header_t)) {
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
        full_packet_t full_packet = {0};
        struct timespec ts_now = {0};
        struct tm timeinfo = {0};
        char strtime_buf[64] = {0};
        ESP_LOGI(TAG, "Received full packet...");
        memcpy(&full_packet, lora_buf, lora_buf_len);
        if (full_packet.head.network_id != NETWORK_ID) {
            ESP_LOGI(TAG, "Ignoring received Packet from other Network '%d'", full_packet.head.network_id);
            return;
        }
        if (is_duplicate(full_packet.head)) {
            ESP_LOGI(TAG, "Ignoring received cached packet id='%d' node='%d'", full_packet.head.packet_id, full_packet.head.orig_node_id);
            return;
        }

        clock_gettime(CLOCK_REALTIME, &ts_now);
        localtime_r(&ts_now.tv_sec, &timeinfo);
        strftime(strtime_buf, sizeof(strtime_buf), "%c", &timeinfo);

        ESP_LOGI(TAG, "Received packet id='%d' from '%d' at %s", full_packet.head.packet_id, full_packet.head.orig_node_id, strtime_buf);
        if (b_append_file(PACKET_FILE, full_packet) == ESP_OK) {
            ESP_LOGI(TAG, "Stored packet to SD card");
            // Notify site_content.c that it needs to send new data through websocket
            notify_websocket(NOTIF_WS_NEW_DATA);
        } else {
            ESP_LOGW(TAG, "Failed to store packet");
        }
        ESP_LOGI(TAG, "Packet signal thing  rssi='%d'  snr='%f'", lora_packet_rssi(), (double)lora_packet_snr());
        break;
    }
    case 0b10: {
        pairing_packet_t pair_packet = {0};
        memcpy(&pair_packet, lora_buf, sizeof(pairing_packet_t));
        notify_websocket(pair_packet.nonce << 16 & NOTIF_WS_PAIRING);
        break;
    }
    default:
        ESP_LOGW(TAG, "Unrecognized packet type %02b", head.flags & 0b11);
        break;
    }
}

/// ============================
///     Main listening loop
/// ============================

static void receive_task(void *p) {
    init_dio0_interrupt();

    lora_init();
    lora_dump_registers();

    while (1) {
        lora_receive();
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (!lora_received()) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        receive_something();
    }
    ESP_LOGW(TAG, "Left task");
}

void start_receive_task() { xTaskCreate(receive_task, TAG, 4096, NULL, 5, &lora_task_handle); }
