#include "big_data.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_bit_defs.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "fun_cache.h"
#include "hal/gpio_types.h"
#include "lora.h"
#include "packet_def.h"
#include "pin_config.h"
#include "portmacro.h"
#include "sd_card.h"
#include "site_content.h"
#include <limits.h>
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
QueueHandle_t new_packet_queue;

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
        full_packet_t full_packet = {0};
        ESP_LOGI(TAG, "Received full packet...");
        memcpy(&full_packet, lora_buf, lora_buf_len);
        if (full_packet.head.network_id != NETWORK_ID) {
            ESP_LOGI(TAG, "Ignoring received Packet from other Network '%d'", full_packet.head.network_id);
            break;
        }
        if (is_duplicate(full_packet.head)) {
            ESP_LOGI(TAG, "Ignoring received cached packet id='%d' node='%d'", full_packet.head.packet_id, full_packet.head.orig_node_id);
            break;
        }
        struct timespec ts_now = {0};
        struct tm timeinfo = {0};
        char strtime_buf[64] = {0};
        clock_gettime(CLOCK_REALTIME, &ts_now);
        localtime_r(&ts_now.tv_sec, &timeinfo);
        strftime(strtime_buf, sizeof(strtime_buf), "%c", &timeinfo);
        full_packet_time_t data = {.time = ts_now.tv_sec, .pkt = full_packet};

        ESP_LOGI(TAG, "Received packet id='%d' from '%d' at %s", full_packet.head.packet_id, full_packet.head.orig_node_id, strtime_buf);
        if (b_append_file(data) == ESP_OK) {
            ESP_LOGI(TAG, "Stored packet to SD card");
            xQueueSend(new_packet_queue, &full_packet, 0);
            // Notify site_content.c that it needs to send new data through websocket
            notify_websocket(NOTIF_WS_NEW_PACKET);
        } else {
            ESP_LOGW(TAG, "Failed to store packet");
        }
        break;
    }
    case 0b10: {
        pairing_packet_t pkt = {0};
        memcpy(&pkt, lora_buf, sizeof(pairing_packet_t));
        ESP_LOGI(TAG, "Received a Pairing packet with Nonce='%04X',  node_id='%d'", pkt.nonce, pkt.head.orig_node_id);
        if (pkt.nonce) {
            if (is_nonce_known(pkt.nonce))
                break;
            notify_websocket((pkt.nonce << 16) | NOTIF_WS_PAIRING);
        } else {
            set_node_id_state(pkt.head.orig_node_id, true);
            uint16_t nonce = is_node_known(pkt.head.orig_node_id);
            if (nonce)
                notify_websocket((nonce << 16) | NOTIF_WS_PAIRING_ACK);
            else
                ESP_LOGW(TAG, "Node not in cache");
        }
        break;
    }
    default:
        ESP_LOGW(TAG, "Unrecognized packet type %02b", head.flags & 0b11);
        break;
    }
    ESP_LOGI(TAG, "Packet signal thing  rssi='%d'  snr='%f'", lora_packet_rssi(), (double)lora_packet_snr());
}

static void send_pairing_confirmation(uint16_t nonce) {
    ESP_LOGI(TAG, "Sending confirmation to %04X", nonce);
    pairing_packet_t pkt = {0};
    pkt.head.header = PACKET_HEADER_VALUE;
    pkt.head.packet_id = get_free_node_id(nonce);
    if (pkt.head.packet_id == 0) {
        ESP_LOGW(TAG, "Not enough free node ids");
        return;
    }
    pkt.head.flags = 0b10;
    pkt.head.network_id = NETWORK_ID;
    pkt.head.hop_count = 5; // Higher than normal
    pkt.nonce = nonce;

    block_if_receiving();
    lora_send_packet((uint8_t *)&pkt, sizeof(pairing_packet_t));
}

static void send_disconnect(uint8_t node_id) {
    pairing_packet_t pkt = {0};
    pkt.head.header = PACKET_HEADER_VALUE;
    pkt.head.network_id = NETWORK_ID;
    pkt.head.orig_node_id = node_id;
    pkt.head.hop_count = 3;
    pkt.head.flags = 0b10;
    pkt.nonce = 0xffff; // disconnect message

    block_if_receiving();
    lora_send_packet((uint8_t *)&pkt, sizeof(pairing_packet_t));
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
        uint32_t notif;
        xTaskNotifyWait(0, ULONG_MAX, &notif, portMAX_DELAY);
        switch (notif & 0xffff) {
        case NOTIF_LORA_PAIRING:
            send_pairing_confirmation(notif >> 16);
            break;
        case NOTIF_LORA_DISCONNECT:
            send_disconnect((uint8_t)(notif >> 16));
            break;
        default:
            if (lora_received())
                receive_something();
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGW(TAG, "Left task");
}
void notify_big_data(uint32_t notif) {
    if (lora_task_handle != NULL) {
        xTaskNotify(lora_task_handle, notif, eSetBits);
    }
}

void start_receive_task() {
    new_packet_queue = xQueueCreate(5, sizeof(full_packet_time_t));
    xTaskCreate(receive_task, TAG, 4096, NULL, 5, &lora_task_handle);
}
