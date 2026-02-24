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

    const uart_config_t uart_config = {.baud_rate = 9600,
                                       .data_bits = UART_DATA_8_BITS,
                                       .parity = UART_PARITY_DISABLE,
                                       .stop_bits = UART_STOP_BITS_1,
                                       .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

    uart_param_config(GPS_UART_NUM, &uart_config);
    uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(GPS_UART_NUM, BUF_SIZE, 0, 0, NULL, 0);

    ESP_LOGI("GPS", "Resetting settings...");
    gpsSendMessage(ubx_wipe_settings, sizeof(ubx_wipe_settings));

    // Why is this done??
    ESP_LOGI("GPS", "Disabling pos...");
    gpsSendMessage(cfg_msg_posllhdis, sizeof(cfg_msg_posllhdis));

    // Why is this done??
    ESP_LOGI("GPS", "Disabling time...");
    gpsSendMessage(cfg_msg_timeutcdis, sizeof(cfg_msg_timeutcdis));

    ESP_LOGI("GPS", "Setting Rate to high...");
    gpsSendMessage(cfg_rate_high, sizeof(cfg_rate_high));

    ESP_LOGI("GPS_HotStart", "Attempting to resume with HotStart...");
    nvs_handle_t nvs;
    ESP_ERROR_CHECK(nvs_open("cfg", NVS_READWRITE, &nvs));
    size_t length = 56;
    ESP_ERROR_CHECK(nvs_get_blob(nvs, "hot_start", hot_start_data, &length));
    if (hot_start_data[0] == 0xB5) {
        gpsSendMessage(hot_start_data, length);
    } else {
        ESP_LOGW("GPS_HotStart", "Failed retrieve HotStart data");
    }
    nvs_close(nvs);

    uart_write_bytes(GPS_UART_NUM, (const char *)DISGGA, strlen(DISGGA));
    vTaskDelay(50 / portTICK_PERIOD_MS);

    uart_write_bytes(GPS_UART_NUM, (const char *)DISGLL, strlen(DISGLL));
    vTaskDelay(50 / portTICK_PERIOD_MS);

    uart_write_bytes(GPS_UART_NUM, (const char *)DISGSA, strlen(DISGSA));
    vTaskDelay(50 / portTICK_PERIOD_MS);

    uart_write_bytes(GPS_UART_NUM, (const char *)DISGSV, strlen(DISGSV));
    vTaskDelay(50 / portTICK_PERIOD_MS);

    uart_write_bytes(GPS_UART_NUM, (const char *)DISRMC, strlen(DISRMC));
    vTaskDelay(50 / portTICK_PERIOD_MS);

    uart_write_bytes(GPS_UART_NUM, (const char *)DISVTG, strlen(DISVTG));
    vTaskDelay(50 / portTICK_PERIOD_MS);
    uart_write_bytes(GPS_UART_NUM, (const char *)DISZDA, strlen(DISZDA));
    vTaskDelay(50 / portTICK_PERIOD_MS);

    int ret;
    printf("Setting full power mode  ");
    ret = gpsSendMessage(cfg_power_full, sizeof(cfg_power_full));
    if (ret == 0) {
        printf("success\n");
    }

    ret = gpsSendMessage(cfg_power_get, sizeof(cfg_power_get));
    if (ret == 0) {
        printf("success\n");
    }

    printf("Setting to ubx  ");
    ret = gpsSendMessage(cfg_prt_ubx, sizeof(cfg_prt_ubx));
    if (ret == 0) {
        printf("success\n");
    }

    gpsSaveHotStartData();

    printf("Setting to Stationary  ");
    ret = gpsSendMessage(cfg_nav5_stationary_3d, sizeof(cfg_nav5_stationary_3d));
    if (ret == 0) {
        printf("success\n");
    }

    printf("Setting to ECO  ");
    ret = gpsSendMessage(cfg_power_eco, sizeof(cfg_power_eco));
    if (ret == 0) {
        printf("success\n");
    }
}

int gpsCalcCheckSum(uint8_t *sentence, uint8_t messageLenghtInc) {
    uint8_t CK_A = 0, CK_B = 0;

    for (int i = 2; i < messageLenghtInc - 2; i++) {
        CK_A = CK_A + sentence[i];
        CK_B = CK_B + CK_A;
    }
    sentence[messageLenghtInc - 2] = CK_A;
    sentence[messageLenghtInc - 1] = CK_B;

    return 0;
}

int gpsSendMessage(uint8_t *sentence, uint8_t messageLengthInc) {
    int len = 0;
    uart_flush_input(GPS_UART_NUM);

    gpsCalcCheckSum(sentence, messageLengthInc);
    uart_write_bytes(GPS_UART_NUM, (const char *)sentence, messageLengthInc);

    len = uart_read_bytes(GPS_UART_NUM, data, BUF_SIZE, pdMS_TO_TICKS(300));

    // ACK-ACK agreement
    if (data[2] == 0x05 && data[3] == 0x01) {
        ESP_LOGI("GPS", "Success");
        return 0;
    } else {
        for (int i = 0; i < len; i++) {
            printf("%02x,", data[i]);
        }
        printf("\n");
        return 1;
    }
}
void gpsTask() {

    printf("Setting to 1 min interval  ");
    int ret = gpsSendMessage(cfg_rate_low, sizeof(cfg_rate_low));
    if (ret == 0) {
        printf("success\n");
    }

    printf("Setting to Enabling pos update  ");
    ret = gpsSendMessage(cfg_msg_posllh, sizeof(cfg_msg_posllh));
    if (ret == 0) {
        printf("success\n");
    }

    printf("Setting to Enabling UTC update  ");
    ret = gpsSendMessage(cfg_msg_timeutc, sizeof(cfg_msg_timeutc));
    if (ret == 0) {
        printf("success\n");
    }

    printf("Getting settings from thing   ");
    ret = gpsSendMessage(poll_cfg_prt, sizeof(poll_cfg_prt));
    if (ret == 0) {
        printf("success\n");
    }

    int len = 0;

    while (1) {
        len = uart_read_bytes(GPS_UART_NUM, data, BUF_SIZE, pdMS_TO_TICKS(60000));

        if (len == 0) {
            ESP_LOGW("GPS", "Recieved nothing");
            continue;
        }

        if (len != 64) {
            ESP_LOGW("GPS", "Recieved unkown package with len %d", len);
            for (int i = 0; i < len; i++) {
                printf("%x,", data[i]);
            }
            printf("\n");
            uart_flush(GPS_UART_NUM);
            continue;
        }

        if (!(data[0] == 0xb5 && data[1] == 0x62)) {
            ESP_LOGW("GPS", "Recived unknown package, %x  %x", data[0], data[1]);
            continue;
        }

        // check if first message is pos
        if (len == 64 && data[2] == 0x01 && data[3] == 0x02) {
            int32_t longitude = (int32_t)(((uint32_t)data[10]) | ((uint32_t)data[11] << 8) |
                                          ((uint32_t)data[12] << 16) | ((uint32_t)data[13] << 24));
            int32_t latitude = (int32_t)(((uint32_t)data[14]) | ((uint32_t)data[15] << 8) | ((uint32_t)data[16] << 16) |
                                         ((uint32_t)data[17] << 24));

            ESP_LOGI("GPS", "Latitude:Longitude:  %.7f, %.7f\n", latitude / 1e7f, longitude / 1e7);
        }

        // check if second part of message is time
        if (len == 64 && data[38] == 0x01 && data[39] == 0x21) {
            uint8_t *time_data = &data[38];
            uint16_t year = time_data[16] | (time_data[17] << 8);
            uint8_t month = time_data[18];
            uint8_t day = time_data[19];
            uint8_t hour = time_data[20];
            uint8_t min = time_data[21];
            uint8_t sec = time_data[22];
            int32_t nano = data[56] | (data[57] << 8) | (data[58] << 16) | (data[59] << 24);
            // clang-format off
            ESP_LOGI("GPS",
                     "UTC:  %04" PRIu16    "-%02" PRIu8    "-%02T" PRIu8    "%02" PRIu8    ":%02" PRIu8    ":%02" PRIu8    ".%" PRId32  "UTC",
                            year,            month,          day,           hour,           min,            sec,            nano);
            // clang-format on
        }
    }
}

void gpsSaveHotStartData() {
    while (1) {
        ESP_LOGI("GPS_HotStart", "Sending Poll request...");
        gpsCalcCheckSum(cfg_data_poll, sizeof(cfg_data_poll));
        uart_write_bytes(GPS_UART_NUM, (const char *)cfg_data_poll, sizeof(cfg_data_poll));

        int len = uart_read_bytes(GPS_UART_NUM, data, 56, pdMS_TO_TICKS(500));
        if (len == 56) {
            ESP_LOGI("GPS_HotStart", "Poll recieved");
            if (*(uint32_t *)data != 0x010b62b5) {
                ESP_LOGE("GPS_HotStart", "Poll has wrong header  %08x", *(uint32_t *)data);
                continue;
            }
            nvs_handle_t nvs;
            ESP_ERROR_CHECK(nvs_open("cfg", NVS_READWRITE, &nvs));
            ESP_ERROR_CHECK(nvs_set_blob(nvs, "hot_start", data, 56));
            nvs_commit(nvs);
            nvs_close(nvs);
            ESP_LOGI("GPS_HotStart", "Data saved");
            break;
        }
        ESP_LOGW("GPS_HotStart", "Failed poll, trying again...");
    }
}
