#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "hal/gpio_types.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "portmacro.h"
#include <driver/uart.h>
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/param.h>

#define DEFINE_ALL_BYTE_ARRAYS
#include "NEO_6M_UART.h"

#define MIN_TO_TICKS(min) pdMS_TO_TICKS((min) * 60 * 1000)

static void print_frame(ubx_frame_t *frame);

static ubx_frame_t *frame_alloc(void) {
    // This function is NOT safe, but should be fine as long as there is only one
    // task capable of reading it
    for (int i = 0; i < UBX_FRAME_POOL_SIZE; i++) {
        if (!frame_pool[i].in_use) {
            frame_pool[i].in_use = true;
            frame_pool[i].len = 0;
            return &frame_pool[i];
        }
    }
    ESP_LOGE("GPS_POOL", "Frame pool Exhausted");
    return NULL;
}

static void frame_free(ubx_frame_t *frame) {
    if (frame) {
        frame->in_use = false;
    }
}

// 0x0201 is NAV-POSLLH; 0x2101 is NAV-TIMEUTC
static inline bool is_nav_message(uint16_t msg_id) { return msg_id == 0x0201 || msg_id == 0x2101; }

void uart_reader_task(void *args) {
    uart_event_t event;

    while (1) {
        if (xQueueReceive(uart_event_queue, &event, portMAX_DELAY) != pdTRUE)
            continue;

        if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL) {
            ESP_LOGW("GPS_UART", "Buffer overflow, flushing");
            uart_flush_input(GPS_UART_NUM);
            xQueueReset(uart_event_queue);
            continue;
        }

        if (event.type != UART_DATA)
            continue;

        ubx_frame_t *frame = frame_alloc();
        if (!frame) {
            uart_flush_input(GPS_UART_NUM);
            left_over_buffer_len = 0;
            continue;
        }

        memcpy(frame->data, left_over_buffer, left_over_buffer_len);

        int16_t read = uart_read_bytes(GPS_UART_NUM, frame->data,
                                       MIN(event.size, UBX_FRAME_BUF_SIZE - left_over_buffer_len), pdMS_TO_TICKS(100));
        if (read < 0) {
            frame_free(frame);
            ESP_LOGE("GPS_UART", "Uart failed to read buffer");
            continue;
        }
        frame->len = (uint8_t)(left_over_buffer_len + read);
        left_over_buffer_len = 0;

        frame->frame_count = 0;
        for (int i = 0; i + 5 < frame->len; i++) {
            if (frame->data[i] == 0xb5 && frame->data[i + 1] == 0x62) {
                uint16_t frame_len = *(uint16_t *)&frame->data[i + 4] + 8;

                if (i + frame_len > frame->len) {
                    left_over_buffer_len = frame->len - i;
                    if (left_over_buffer_len <= UBX_FRAME_BUF_SIZE) {
                        memcpy(left_over_buffer, &frame->data[i], left_over_buffer_len);
                        ESP_LOGW("GPS_UART", "Saved %d bytes for next frame", left_over_buffer_len);
                    } else {
                        ESP_LOGW("GPS_UART", "Incomplete frame too large, discarded");
                    }
                    break;
                }

                if (frame->frame_count < 4) {
                    frame->frame_offsets[frame->frame_count++] = i;
                }

                i += frame_len - 1;
            }
        }

        if (frame->frame_count == 0) {
            frame_free(frame);
            continue;
        }

        QueueHandle_t dest = response_queue;
        for (int f = 0; f < frame->frame_count; f++) {
            uint16_t msg_id = *(uint16_t *)&frame->data[frame->frame_offsets[f] + 2];
            if (is_nav_message(msg_id)) {
                dest = nav_queue;
            } else {
                dest = response_queue;
                break;
            }
        }

        bool routed = false;
        if (xQueueSend(dest, &frame, 0) == pdTRUE) {
            routed = true;
        } else {
            ESP_LOGW("GPS_UART", "Queue full, dropping frame");
            frame_free(frame);
        }

        if (!routed) {
            frame_free(frame);
        }
    }
}

static const char *human_readable_name(uint16_t msg_id) {
    switch (msg_id) {
    case 0x0201:
        return "NAV-POSLLH";
    case 0x0906:
        return "CFG-CFG";
    case 0x0006:
        return "CFG-PRT";
    case 0x2406:
        return "CFG-NAV5";
    default:
        ESP_LOGW("HRN", "Unkown msg_id:  %04x", msg_id);
        return "Unkown";
    }
}

static void gpsCalcCheckSum(uint8_t *sentence, uint8_t messageLenghtInc) {
    uint8_t CK_A = 0, CK_B = 0;

    for (int i = 2; i < messageLenghtInc - 2; i++) {
        CK_A = CK_A + sentence[i];
        CK_B = CK_B + CK_A;
    }
    sentence[messageLenghtInc - 2] = CK_A;
    sentence[messageLenghtInc - 1] = CK_B;
}

static void print_frame(ubx_frame_t *frame) {
    bool did_something = false;
    int read_next = 0;
    uint8_t byt;

    for (int i = 0; i + 5 < frame->len; i++) {
        byt = frame->data[i];
        if (byt == 0xb5 && frame->data[i + 1] == 0x62) {
            if (did_something) {
                printf("\n");
            }
            read_next = frame->data[i + 4] + (frame->data[i + 5] << 8) + 8;
            did_something = true;
        }
        if (read_next > 0) {
            printf("%02x,", byt);
            read_next--;
        }
    }
    if (did_something) {
        printf("\n");
    }
}

int32_t gps_shared_longitude = 0.0f;
int32_t gps_shared_latitude = 0.0f;

static void evaluate_frame(ubx_frame_t *frame) {
    uint16_t offset, msg_id, year;
    int32_t nano;
    uint8_t month, day, hour, min, sec;
    for (int f = 0; f < frame->frame_count; f++) {
        offset = frame->frame_offsets[f];
        msg_id = *(uint16_t *)&frame->data[offset + 2];
        switch (msg_id) {
        case 0x0105: // ACK-ACK
            ESP_LOGI("GPS", "Success from %s", human_readable_name(*(uint16_t *)&frame->data[offset + 6]));
            break;
        case 0x0005: // ACK-NAK
            ESP_LOGI("GPS", "Failure from %s", human_readable_name(*(uint16_t *)&frame->data[offset + 6]));
            break;
        case 0x010b: // AID-INI
            ESP_LOGI("GPS_HotStart", "Recieved hot start data");
            if (offset + 56 > frame->len) {
                ESP_LOGW("GPS_HotStart", "Invalid Frame");
                break;
            }
            nvs_handle_t nvs;
            ESP_ERROR_CHECK(nvs_open("cfg", NVS_READWRITE, &nvs));
            ESP_ERROR_CHECK(nvs_set_blob(nvs, "hot_start", &frame->data[offset], 56));
            nvs_commit(nvs);
            nvs_close(nvs);
            ESP_LOGI("GPS_HotStart", "Data saved");
            break;
        case 0x0301: // NAV-STATUS
            ESP_LOGW("GPS", "Nav Status should not be here");
            break;
        case 0x0201: // NAV-POSLLH
            gps_shared_longitude = *(int32_t *)&frame->data[offset + 10];
            gps_shared_latitude = *(int32_t *)&frame->data[offset + 14];
            ESP_LOGI("GPS", "Latitude:Longitude:  %.7f, %.7f", gps_shared_latitude / 1e7f, gps_shared_longitude / 1e7);
            break;
        case 0x2101: // NAV-TIMEUTC
            nano = *(int32_t *)&frame->data[offset + 14];
            year = *(uint16_t *)&frame->data[offset + 18];
            month = frame->data[offset + 20];
            day = frame->data[offset + 21];
            hour = frame->data[offset + 22];
            min = frame->data[offset + 23];
            sec = frame->data[offset + 24];
            // clang-format off
            ESP_LOGI("GPS",
                     "UTC:  %04" PRIu16    "-%02" PRIu8    "-%02" PRIu8    "T%02" PRIu8    ":%02" PRIu8    ":%02" PRIu8    ".%" PRId32  " UTC",
                            year,            month,          day,           hour,           min,            sec,            nano);
            // clang-format on
            break;
        case 0x3b06: // CFG-PM2
            print_frame(frame);
            break;
        default: // should not really happen
            ESP_LOGW("GPS", "Unkown frame msg_id: %04x", msg_id);
            break;
        }
    }
}

static int gps_send_request(uint8_t *sentence, uint8_t messageLengthInc) {
    gpsCalcCheckSum(sentence, messageLengthInc);

    ubx_frame_t *stale;
    while (xQueueReceive(response_queue, &stale, 0) == pdTRUE) {
        frame_free(stale);
    }

    uart_write_bytes(GPS_UART_NUM, (const char *)sentence, messageLengthInc);

    ubx_frame_t *frame;

    if (xQueueReceive(response_queue, &frame, pdMS_TO_TICKS(120000)) == pdTRUE) {
        evaluate_frame(frame);

        frame_free(frame);
        return 0;
    }
    ESP_LOGW("GPS", "Failed to recieve poll request");
    return 1;
}

static void gps_hot_start() {
    nvs_handle_t nvs;
    ESP_ERROR_CHECK(nvs_open("cfg", NVS_READWRITE, &nvs));
    size_t length = 56;
    esp_err_t hs_err = nvs_get_blob(nvs, "hot_start", hot_start_data, &length);
    nvs_close(nvs);
    if (hs_err == ESP_OK && hot_start_data[0] == 0xB5) {
        ESP_LOGI("GPS_HotStart", "Sending HotStart data");
        uart_write_bytes(GPS_UART_NUM, hot_start_data, length);
    } else {
        ESP_LOGW("GPS_HotStart", "No valid hot-start data found (err=0x%x)", hs_err);
    }
}

void ensure_gps_fix() {
    gpsCalcCheckSum(poll_nav_status, sizeof(poll_nav_status));
    uint32_t ttff, msss;
    int attempts = 0;
    ubx_frame_t *frame;
    while (1) {
        attempts++;
        /* ESP_LOGI("GPS_FIX", "Attempt nr %d to get good fix...", attempts); */
        if (attempts == 3) {
            ESP_LOGI("GPS_FIX", "Attempting to hot start");
            gps_hot_start();
        }
        uart_write_bytes(GPS_UART_NUM, (const char *)poll_nav_status, sizeof(poll_nav_status));

        if (xQueueReceive(response_queue, &frame, pdMS_TO_TICKS(30000)) == pdTRUE) {
            for (int f = 0; f < frame->frame_count; f++) {
                if (*(uint16_t *)&frame->data[frame->frame_offsets[f] + 2] == 0x0301) {
                    // Good fix (3d fix)
                    if (frame->data[frame->frame_offsets[f] + 10] == 0x03) {
                        ttff = *(uint32_t *)&frame->data[frame->frame_offsets[f] + 14];
                        msss = *(uint32_t *)&frame->data[frame->frame_offsets[f] + 18];
                        ESP_LOGI("GPS_FIX", "There is good fix after %d attempts", attempts);
                        ESP_LOGI("GPS_FIX", "Time to first fix %" PRIu32 "  ms since start/reset %" PRIu32, ttff, msss);
                        frame_free(frame);
                        return;
                    }
                }
            }
            frame_free(frame);
        }
    }
}

void gpsInitUart() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    gpio_set_direction(GPS_DISABLE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(GPS_DISABLE_PIN, 0);

    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    uart_param_config(GPS_UART_NUM, &uart_config);
    uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(GPS_UART_NUM, BUF_SIZE, 0, 20, &uart_event_queue, 0);

    response_queue = xQueueCreate(1, sizeof(ubx_frame_t *));
    nav_queue = xQueueCreate(2, sizeof(ubx_frame_t *));
    xTaskCreate(uart_reader_task, "uart_reader", 4096, NULL, 10, NULL);

    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI("GPS", "Wiping settings...");
    gps_send_request(ubx_wipe_settings, sizeof(ubx_wipe_settings));
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI("GPS", "Disabling all NMEA Communication");
    for (int i = 0; i < 7; i++) {
        uart_write_bytes(GPS_UART_NUM, nmea_dis[i], strlen(nmea_dis[i]));
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ESP_LOGI("GPS", "Setting Communication protocol to only UBX...");
    ESP_ERROR_CHECK(gps_send_request(cfg_prt_ubx_only, sizeof(cfg_prt_ubx_only)));
    vTaskDelay(pdMS_TO_TICKS(200));

    ESP_LOGI("GPS", "Setting to Stationary...");
    ESP_ERROR_CHECK(gps_send_request(cfg_nav5_stationary_3d, sizeof(cfg_nav5_stationary_3d)));
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI("GPS_HotStart", "Attempting to resume with HotStart...");

    ensure_gps_fix();

    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI("GPS_HotStart", "Sending Poll request...");
    ESP_ERROR_CHECK(gps_send_request(aid_ini_poll, sizeof(aid_ini_poll)));

    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI("GPS", "Init Complete.");
}

void gps_task() {
    gpsInitUart();

    ubx_frame_t *frame;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t gps_polling_rate = MIN_TO_TICKS(10);
    gpsCalcCheckSum(nav_posllh_poll, sizeof(nav_posllh_poll));
    gpsCalcCheckSum(nav_timeutc_poll, sizeof(nav_timeutc_poll));

    while (1) {
        ESP_LOGI("GPS", "Reading frame");
        gpio_set_level(GPS_DISABLE_PIN, 0);

        vTaskDelay(pdMS_TO_TICKS(2000));

        ensure_gps_fix();

        uart_write_bytes(GPS_UART_NUM, (const void *)nav_posllh_poll, sizeof(nav_posllh_poll));
        uart_write_bytes(GPS_UART_NUM, (const void *)nav_timeutc_poll, sizeof(nav_timeutc_poll));

        if (xQueueReceive(nav_queue, &frame, pdMS_TO_TICKS(120000)) != pdTRUE) {
            ESP_LOGW("GPS", "Failed to get reading, retrying...");
            continue;
        }
        evaluate_frame(frame);
        frame_free(frame);

        ESP_LOGI("GPS", "Going to sleep after %d ms", pdTICKS_TO_MS(xTaskGetTickCount() - last_wake));
        gpio_set_level(GPS_DISABLE_PIN, 1);
        vTaskDelayUntil(&last_wake, gps_polling_rate);
    }
    ESP_LOGW("GPS", "Leaving Task");
}
