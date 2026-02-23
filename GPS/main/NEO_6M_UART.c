
#include "NEO_6M_UART.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "portmacro.h"
#include <ctype.h>
#include <driver/uart.h>
#include <inttypes.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void gpsInitUart() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    const uart_config_t uart_config = {.baud_rate = 9600,
                                       .data_bits = UART_DATA_8_BITS,
                                       .parity = UART_PARITY_DISABLE,
                                       .stop_bits = UART_STOP_BITS_1,
                                       .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

    printf("uart\n");
    uart_param_config(GPS_UART_NUM, &uart_config);
    uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE);
    printf("uart\n");
    uart_driver_install(GPS_UART_NUM, BUF_SIZE, 0, 0, NULL, 0);

    printf("Resetting GPS settings  ");
    int ret = gpsSendMessage(ubx_wipe_settings, sizeof(ubx_wipe_settings));
    if (ret == 0) {
        printf("success\n");
    }

    printf("Disabling pos ");
    ret = gpsSendMessage(cfg_msg_posllhdis, sizeof(cfg_msg_posllhdis));
    if (ret == 0) {
        printf("  success\n");
    }

    printf("Disabling time ");
    ret = gpsSendMessage(cfg_msg_timeutcdis, sizeof(cfg_msg_timeutcdis));
    if (ret == 0) {
        printf("  success\n");
    }

    printf("Setting to 5hz ");
    ret = gpsSendMessage(cfg_rate_5hz, sizeof(cfg_rate_5hz));
    if (ret == 0) {
        printf("  success\n");
    }
    printf("loading hotstart data\n");
    nvs_handle_t nvs;
    nvs_open("cfg", NVS_READWRITE, &nvs);
    size_t length = 56;
    ret = nvs_get_blob(nvs, "hot_start", hot_start_data, &length);
    printf("flash read = %x\n", ret);
    if (hot_start_data[0] == 0xb5) {
        ret = gpsSendMessage(hot_start_data, length);
        if (ret == 0) {
            printf("HOT start success: used data\n");
            for (int i = 0; i < length; i++) {
                printf("%x,", hot_start_data[i]);
            }
            printf("\n");
        } else {
            printf("Hot start not success\n");
            for (int i = 0; i < length; i++) {
                printf("%x,", hot_start_data[i]);
            }
            printf("\n");
        }
    } else {
        printf("no hotstart data, exppected after flash %x\n",
               hot_start_data[0]);
    }
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
    ret =
        gpsSendMessage(cfg_nav5_stationary_3d, sizeof(cfg_nav5_stationary_3d));
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
    // Read to clear gps buffer
    len = uart_read_bytes(GPS_UART_NUM, data, BUF_SIZE,
                          100 / portTICK_PERIOD_MS); // short timeout
                                                     //
    gpsCalcCheckSum(sentence, messageLengthInc);

    uart_write_bytes(GPS_UART_NUM, (const char *)sentence, messageLengthInc);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    len = uart_read_bytes(GPS_UART_NUM, data, BUF_SIZE,
                          100 / portTICK_PERIOD_MS); // short timeout

    for (int i = 0; i < len; i++) {
        if (data[i] == 0xb5) {
            printf("\n");
        }
        printf("%x,", data[i]);
    }
    printf("\n");
    if (len >= 10 && data[2] == 0x05 && data[3] == 0x01) {
        return 0;
    } else {
        return 1;
    }
}
void gpsTask() {

    printf("Setting to 1 min interval  ");
    int ret = gpsSendMessage(cfg_rate_1m, sizeof(cfg_rate_1m));
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

    int len = 0;

    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        len = uart_read_bytes(GPS_UART_NUM, data, BUF_SIZE,
                              500 / portTICK_PERIOD_MS); // short timeout
        if (len == 0) {
            printf("recieved nothing\n");
            continue;
        }

        printf("something recieved len = %d \n", len);
        // check if first message is pos
        if (len == 64 && data[2] == 0x01 && data[3] == 0x02) {
            printf("\n");

            int32_t lon =
                (int32_t)(((uint32_t)data[10]) | ((uint32_t)data[11] << 8) |
                          ((uint32_t)data[12] << 16) |
                          ((uint32_t)data[13] << 24));

            int32_t lat =
                (int32_t)(((uint32_t)data[14]) | ((uint32_t)data[15] << 8) |
                          ((uint32_t)data[16] << 16) |
                          ((uint32_t)data[17] << 24));

            double longitude_deg = lon / 1e7;
            double latitude_deg = lat / 1e7;
            printf("Latitude:Longiture:  %.7f, %.7f\n", latitude_deg,
                   longitude_deg);
            printf("\n");
        }
        // check if second part of message is time
        if (len == 64 && data[38] == 0x01 && data[39] == 0x21) {
            uint8_t *time_data = &data[38];
            uint8_t day = time_data[19];
            uint8_t hour = time_data[20];
            uint8_t min = time_data[21];
            uint8_t sec = time_data[22];
            int32_t nano = data[56] | (data[57] << 8) | (data[58] << 16) |
                           (data[59] << 24);
            printf("UTC:  day %02" PRIu8 " %02" PRIu8 ":%02" PRIu8 ":%02" PRIu8
                   ".%" PRId32 "\n",
                   day, hour, min, sec, nano);
        }
        if (len != 64) {
            printf("recieved unknown package\n");
            for (int i = 0; i < len; i++) {
                if (data[i] == 0xB5) {
                    printf("\n");
                }
                printf("%x,", data[i]);
            }
            continue;
        }
        vTaskDelay(59000 / portTICK_PERIOD_MS);
    }
}
int gpsSaveHotStartData() {

    while (1) {
        printf("polling\n");
        gpsCalcCheckSum(cfg_data_poll, sizeof(cfg_data_poll));

        uart_write_bytes(GPS_UART_NUM, (const char *)cfg_data_poll,
                         sizeof(cfg_data_poll));
        int tlen = uart_read_bytes(GPS_UART_NUM, data, BUF_SIZE,
                                   100 / portTICK_PERIOD_MS); // short timeout
        printf(" len = %d\n", tlen);
        if (tlen == 56) {
            ESP_LOGI("GPS", "date recieved\n");
            for (int i = 0; i < tlen; i++) {
                printf("%x,", data[i]);
            }
            printf("\n");
            nvs_handle_t nvs;

            nvs_open("cfg", NVS_READWRITE, &nvs);
            int ret = nvs_set_blob(nvs, "hot_start", data, 56);
            printf("flash save =%x\n", ret);
            nvs_commit(nvs); // REQUIRED
            nvs_close(nvs);
            printf("saved to flash\n");
            break;
        }
        for (int i = 0; i < tlen; i++) {
            if (data[i] == 0xb5) {
                printf("\n");
            }
            printf("%x,", data[i]);
        }
    }
    return 0;
}
