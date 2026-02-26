#include "NEO_6M_UART.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "portmacro.h"
#include <ctype.h>
#include <driver/uart.h>
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

uint8_t data[1024];

void gpsInitUart() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    uart_param_config(GPS_UART_NUM, &uart_config);
    uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(GPS_UART_NUM, BUF_SIZE, 0, 0, NULL, 0);

    ESP_LOGI("GPS", "Wiping settings...");
    gpsSendMessage(ubx_wipe_settings, sizeof(ubx_wipe_settings));
    vTaskDelay(pdMS_TO_TICKS(500));

    for (int i = 0; i < 7; i++) {
        uart_write_bytes(GPS_UART_NUM, nmea_dis[i], strlen(nmea_dis[i]));
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ESP_LOGI("GPS", "Setting Communication protocol to only UBX...");
    ESP_ERROR_CHECK(gpsSendMessage(cfg_prt_ubx, sizeof(cfg_prt_ubx)));
    vTaskDelay(pdMS_TO_TICKS(200));

    ESP_LOGI("GPS", "Setting full power mode...");
    ESP_ERROR_CHECK(gpsSendMessage(cfg_power_full, sizeof(cfg_power_full)));

    ESP_LOGI("GPS_HotStart", "Attempting to resume with HotStart...");
    nvs_handle_t nvs;
    ESP_ERROR_CHECK(nvs_open("cfg", NVS_READWRITE, &nvs));
    size_t length = 56;
    esp_err_t hs_err = nvs_get_blob(nvs, "hot_start", hot_start_data, &length);
    nvs_close(nvs);
    if (hs_err == ESP_OK && hot_start_data[0] == 0xB5) {
        ESP_LOGI("GPS_HotStart", "Sending HotStart data...");
        gpsSendMessage(hot_start_data, (uint8_t)length);
    } else {
        ESP_LOGW("GPS_HotStart", "No valid hot-start data found (err=0x%x)", hs_err);
    }

    ESP_LOGI("GPS", "Setting Rate to high...");
    gpsSendMessage(cfg_rate_high, sizeof(cfg_rate_high));

    ESP_LOGI("GPS", "Setting to Stationary...");
    ESP_ERROR_CHECK(gpsSendMessage(cfg_nav5_stationary_3d, sizeof(cfg_nav5_stationary_3d)));

    ESP_LOGI("GPS", "Setting to ECO...");
    ESP_ERROR_CHECK(gpsSendMessage(cfg_power_eco, sizeof(cfg_power_eco)));

    ESP_LOGI("GPS", "Setting to 1 min interval...");
    ESP_ERROR_CHECK(gpsSendMessage(cfg_rate_low, sizeof(cfg_rate_low)));

    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI("GPS", "Enabling NAV-POSLLH on UART1...");
    ESP_ERROR_CHECK(gpsSendMessage(cfg_msg_posllh, sizeof(cfg_msg_posllh)));

    ESP_LOGI("GPS", "Enabling NAV-TIMEUTC on UART1...");
    ESP_ERROR_CHECK(gpsSendMessage(cfg_msg_timeutc, sizeof(cfg_msg_timeutc)));

    ESP_LOGI("GPS", "Init Complete.");
}

void gpsCalcCheckSum(uint8_t *sentence, uint8_t messageLenghtInc) {
    uint8_t CK_A = 0, CK_B = 0;

    for (int i = 2; i < messageLenghtInc - 2; i++) {
        CK_A = CK_A + sentence[i];
        CK_B = CK_B + CK_A;
    }
    sentence[messageLenghtInc - 2] = CK_A;
    sentence[messageLenghtInc - 1] = CK_B;
}

int find_target(uint16_t target_id, size_t len, uint16_t *target_location, uint16_t *target_len) {
    uint8_t byt;

    for (int i = 0; i < len; i++) {
        byt = data[i];
        if (byt == 0xb5 && i + 5 < len && data[i + 1] == 0x62) {
            if ((*(uint16_t *)&data[i + 2]) == target_id) {
                *target_location = i;
                *target_len = data[i + 4] + (data[i + 5] << 8) + 8;
                return 0;
            }
        }
    }
    return 1;
}

void print_data(size_t len) {
    bool did_something = false;
    int read_next = 0;
    uint8_t byt;

    for (int i = 0; i < len; i++) {
        byt = data[i];
        if (byt == 0xb5 && i + 5 < len && data[i + 1] == 0x62) {
            if (did_something) {
                printf("\n");
            }
            read_next = data[i + 4] + (data[i + 5] << 8) + 8;
            did_something = true;
        }
        if (read_next > 0) {
            printf("%02x,", byt);
            // printf("%d  %02x\n", read_next, byt);
            read_next--;
        }
    }
    if (did_something) {
        printf("\n");
    }
}

int gpsSendMessage(uint8_t *sentence, uint8_t messageLengthInc) {
    uart_flush_input(GPS_UART_NUM);

    gpsCalcCheckSum(sentence, messageLengthInc);
    uart_write_bytes(GPS_UART_NUM, (const char *)sentence, messageLengthInc);

    int rxlen = uart_read_bytes(GPS_UART_NUM, data, BUF_SIZE, pdMS_TO_TICKS(10000));
    uint16_t target_location, target_len;

    if (find_target(0x0105, rxlen, &target_location, &target_len) == 0) {
        // ACK-ACK agreement
        ESP_LOGI("GPS", "Success from  %02x %02x", data[target_location + 6], data[target_location + 7]);
        return 0;
    }
    if (*(uint32_t *)data == 0x000662b5) {
        // CFG-PRT Response to poll request
        ESP_LOGI("GPS", "Success, CFG-PRT, Configurations:");
        print_data(rxlen);
        return 0;
    }
    if (*(uint32_t *)data == 0x020662b5) {
        // CFG-INF Response to poll request
        ESP_LOGI("GPS", "Success, CFG-INF, Stuff:");
        print_data(rxlen);
        return 0;
    }
    if (*(uint32_t *)data == 0x170662b5) {
        // CFG-NMEA Response to poll request
        ESP_LOGI("GPS", "Success, CFG-NMEA, Stuff:");
        print_data(rxlen);
        return 0;
    }

    ESP_LOGE("GPS", "Request returned unkown message:");
    print_data(rxlen);
    return 0;
}

void gpsTask() {
    int len = 0;
    uint16_t target_len, target_location;

    gpsSaveHotStartData();

    while (1) {
        uart_flush(GPS_UART_NUM);
        len = uart_read_bytes(GPS_UART_NUM, data, BUF_SIZE, pdMS_TO_TICKS(10000));

        if (find_target(0x0201, len, &target_location, &target_len) == 0) {
            int32_t longitude = *(int32_t *)&data[target_location + 10];
            int32_t latitude = *(int32_t *)&data[target_location + 14];

            ESP_LOGI("GPS", "Latitude:Longitude:  %.7f, %.7f", latitude / 1e7f, longitude / 1e7);
        }

        if (find_target(0x2101, len, &target_location, &target_len) == 0) {
            int32_t nano = *(int32_t *)&data[target_location + 14];
            uint16_t year = *(uint16_t *)&data[target_location + 18];
            uint8_t month = data[target_location + 20];
            uint8_t day = data[target_location + 21];
            uint8_t hour = data[target_location + 22];
            uint8_t min = data[target_location + 23];
            uint8_t sec = data[target_location + 24];
            // clang-format off
            ESP_LOGI("GPS",
                     "UTC:  %04" PRIu16    "-%02" PRIu8    "-%02" PRIu8    "T%02" PRIu8    ":%02" PRIu8    ":%02" PRIu8    ".%" PRId32  " UTC",
                            year,            month,          day,           hour,           min,            sec,            nano);
            // clang-format on
        }
    }
}

void gpsSaveHotStartData() {
    uint16_t target_id = 0x010b;
    uint16_t target_location, target_len;
    while (1) {
        ESP_LOGI("GPS_HotStart", "Sending Poll request...");
        gpsCalcCheckSum(cfg_data_poll, sizeof(cfg_data_poll));
        uart_write_bytes(GPS_UART_NUM, (const char *)cfg_data_poll, sizeof(cfg_data_poll));

        int len = uart_read_bytes(GPS_UART_NUM, data, BUF_SIZE, pdMS_TO_TICKS(2000));
        if (len >= 48 && find_target(target_id, len, &target_location, &target_len) == 0) {
            ESP_LOGI("GPS_HotStart", "Poll recieved");
            nvs_handle_t nvs;
            ESP_ERROR_CHECK(nvs_open("cfg", NVS_READWRITE, &nvs));
            ESP_ERROR_CHECK(nvs_set_blob(nvs, "hot_start", &data[target_location], 56));
            nvs_commit(nvs);
            nvs_close(nvs);
            ESP_LOGI("GPS_HotStart", "Data saved");
            break;
        } else {
            ESP_LOGE("GPS_HotStart", "Poll has wrong header  %08x", *(uint32_t *)data);
            print_data(len);
            uart_flush_input(GPS_UART_NUM);
            continue;
        }
        ESP_LOGW("GPS_HotStart", "Failed poll, trying again...");
    }
}
