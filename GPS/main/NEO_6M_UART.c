
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

#define DISGGA "$PUBX,40,GGA,0,0,0,0,0,0*5A\r\n"
#define DISGLL "$PUBX,40,GLL,0,0,0,0,0,0*5C\r\n"
#define DISGSA "$PUBX,40,GSA,0,0,0,0,0,0*4E\r\n"
#define DISGSV "$PUBX,40,GSV,0,0,0,0,0,0*59\r\n"
#define DISRMC "$PUBX,40,RMC,0,0,0,0,0,0*47\r\n"
#define DISVTG "$PUBX,40,VTG,0,0,0,0,0,0*5E\r\n"
#define DISZDA "$PUBX,40,ZDA,0,0,0,0,0,0*44\r\n"

#define ENGGA "$PUBX,40,GGA,0,1,0,0,0,0*5B\r\n"
#define ENGLL "$PUBX,40,GLL,0,1,0,0,0,0*5D\r\n"
#define ENGSA "$PUBX,40,GSA,0,1,0,0,0,0*4F\r\n"
#define ENGSV "$PUBX,40,GSV,0,1,0,0,0,0*58\r\n"
#define ENRMC "$PUBX,40,RMC,0,1,0,0,0,0*46\r\n"
#define ENVTG "$PUBX,40,VTG,0,1,0,0,0,0*5F\r\n"
#define ENZDA "$PUBX,40,ZDA,0,1,0,0,0,0*45\r\n"

uint8_t hot_start_data[56];
uint8_t cfg_rate_01hz[] = {
    0xB5, 0x62, // UBX header
    0x06, 0x08, // CFG-RATE
    0x06, 0x00, // payload length = 6
    0x10, 0x27, // measRate = 10000 ms (0.1 Hz)
    0x01, 0x00, // navRate = 1
    0x01, 0x00, // timeRef = UTC
    0x00, 0x00  // placeholder checksum → calculate next
};

uint8_t cfg_rate_5hz[] = {
    0xB5, 0x62, // UBX header
    0x06, 0x08, // CFG-RATE
    0x06, 0x00, // payload length = 6
    0x68, 0x00, // measRate = 10000 ms (0.1 Hz)
    0x01, 0x00, // navRate = 1
    0x01, 0x00, // timeRef = UTC
    0x00, 0x00  // placeholder checksum → calculate next
};
uint8_t cfg_power_eco[] = {0xB5, 0x62, 0x06, 0x11, 0x02,
                           0x00, 0x08, 0x01, 0x00, 0x00};

uint8_t cfg_power_full[] = {0xB5, 0x62, 0x06, 0x11, 0x02,
                            0x00, 0x08, 0x00, 0x00, 0x00};

uint8_t cfg_power_get[] = {0xB5, 0x62, 0x06, 0x11, 0x00, 0x00, 0x00, 0x00};

uint8_t cfg_data_poll[] = {
    0xB5, 0x62, 0x0B, 0x01, 0x00, 0x00, 0x0C, 0x2D,
};

uint8_t cfg_nav5_stationary_3d[] = {
    0xB5, 0x62,             // UBX header
    0x06, 0x24,             // CFG-NAV5
    0x24, 0x00,             // payload length = 36 bytes
    0x01, 0x00,             // mask: apply dynModel only (bit 0)
    0x02,                   // dynModel: 2 = Stationary
    0x03,                   // fixMode: 3 = Auto 2D/3D
    0x00, 0x00, 0x00, 0x00, // fixedAlt
    0x00, 0x00, 0x00, 0x00, // fixedAltVar
    0x00,                   // minElev
    0x00,                   // drLimit
    0x00, 0x00,             // pDOP
    0x00, 0x00,             // tDOP
    0x00, 0x00,             // pAcc
    0x00, 0x00,             // tAcc
    0x00,                   // staticHoldThresh
    0x00,                   // dgpsTimeOut
    0x00, 0x00, 0x00, 0x00, // reserved2
    0x00, 0x00, 0x00, 0x00, // reserved3
    0x00, 0x00, 0x00, 0x00, // reserved4
    0x00, 0x00              // placeholder checksum → calculate next
};
uint8_t cfg_nav_binary[] = {
    0xB5, 0x62, // header
    0x06, 0x01, // CFG-MSG
    0x03, 0x00, // payload length = 3
    0x01, 0x07,
    0x01, // class 0x01, ID 0x07 (NAV-PVT), rate = 1 (every navigation solution)
    0x00, 0x00 // placeholder checksum
};
uint8_t cfg_prt_ubx[] = {
    0xB5, 0x62,             // header
    0x06, 0x00,             // CFG-PRT
    0x14, 0x00,             // payload length = 20 bytes
    0x01,                   // portID = 1 (UART1)
    0x00,                   // reserved
    0x00, 0x00,             // txReady
    0xD0, 0x08, 0x00, 0x00, // mode (8N1, no parity)
    0x80, 0x25, 0x00, 0x00, // baud rate = 9600 (little-endian)
    0x07, 0x00,             // inProtoMask = UBX+NMEA
    0x03, 0x00,             // outProtoMask = UBX+NMEA
    0x00, 0x00,             // flags
    0x00, 0x00,             // reserved
    0x00, 0x00              // placeholder checksum
};

uint8_t poll_nav_posllh[] = {
    0xB5, 0x62, // UBX header
    0x01, 0x02, // Class = NAV, ID = POSLLH
    0x00, 0x00, // payload length = 0
    0x03, 0x05  // checksum CK_A, CK_B
};
uint8_t cfg_msg_posllh[] = {
    0xB5, 0x62, // UBX header
    0x06, 0x01, // CFG-MSG
    0x08, 0x00, // payload length = 8
    0x01, 0x02, // NAV-POSLLH
    0x00,       // rate on I2C
    0x01,       // rate on UART1
    0x00,       // rate on UART2
    0x00,       // rate on USB
    0x00,       // rate on SPI
    0x00,       // reserved
    0x00, 0x00  // checksum (calculate)
};
uint8_t cfg_msg_timeutc[] = {
    0xB5, 0x62, // UBX header
    0x06, 0x01, // CFG-MSG
    0x03, 0x00, // payload length
    0x01, 0x21, // NAV-TIMEUTC
    0x01,       // rate = 1 (every nav solution)
    0x00, 0x00  // checksum (calculate)
};

uint8_t ubx_wipe_settings[] = {
    0xB5, 0x62,             // UBX header
    0x06, 0x09,             // CFG-CFG
    0x0D, 0x00,             // payload length = 13
    0xFF, 0xFF, 0x00, 0x00, // clearMask (clear all)
    0x00, 0x00, 0x00, 0x00, // saveMask (nothing)
    0x00, 0x00, 0x00, 0x00, // loadMask (nothing)
    0x07,                   // deviceMask: RAM | BBR | FLASH
    0x00, 0x00              // checksum (calculate)
};
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

    uart_param_config(GPS_UART_NUM, &uart_config);
    uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE);
    uart_driver_install(GPS_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);

    printf("Resetting GPS settings  ");
    int ret = gpsSendMessage(ubx_wipe_settings, sizeof(ubx_wipe_settings));
    if (ret == 0) {
        printf("success\n");
    }

    nvs_handle_t nvs;
    nvs_open("cfg", NVS_READWRITE, &nvs);
    size_t length = 56;
    nvs_get_blob(nvs, "hot_start", hot_start_data, &length);
    ret = gpsSendMessage(hot_start_data, length);
    if (ret == 0) {
        printf("how startt success\n");
    }
    uart_write_bytes(GPS_UART_NUM, (const char *)DISGGA, strlen(DISGGA));
    vTaskDelay(100 / portTICK_PERIOD_MS);

    uart_write_bytes(GPS_UART_NUM, (const char *)DISGLL, strlen(DISGLL));
    vTaskDelay(100 / portTICK_PERIOD_MS);

    uart_write_bytes(GPS_UART_NUM, (const char *)DISGSA, strlen(DISGSA));
    vTaskDelay(100 / portTICK_PERIOD_MS);

    uart_write_bytes(GPS_UART_NUM, (const char *)DISGSV, strlen(DISGSV));
    vTaskDelay(100 / portTICK_PERIOD_MS);

    uart_write_bytes(GPS_UART_NUM, (const char *)DISRMC, strlen(DISRMC));
    vTaskDelay(100 / portTICK_PERIOD_MS);

    uart_write_bytes(GPS_UART_NUM, (const char *)DISVTG, strlen(DISVTG));
    vTaskDelay(100 / portTICK_PERIOD_MS);
    uart_write_bytes(GPS_UART_NUM, (const char *)DISZDA, strlen(DISZDA));
    vTaskDelay(100 / portTICK_PERIOD_MS);

    printf("Setting full power mode  ");
    ret = gpsSendMessage(cfg_power_full, sizeof(cfg_power_full));
    if (ret == 0) {
        printf("success\n");
    }

    ret = gpsSendMessage(cfg_power_get, sizeof(cfg_power_get));
    if (ret == 0) {
        printf("success\n");
    }

    // printf("Polling data  ");
    // ret = gpsSendMessage(cfg_data_poll, sizeof(cfg_data_poll));
    // if (ret == 0) {
    //     printf("success\n");
    // }

    printf("Setting to Stationary  ");
    ret =
        gpsSendMessage(cfg_nav5_stationary_3d, sizeof(cfg_nav5_stationary_3d));
    if (ret == 0) {
        printf("success\n");
    }

    printf("Setting to ubx  ");
    ret = gpsSendMessage(cfg_prt_ubx, sizeof(cfg_prt_ubx));
    if (ret == 0) {
        printf("success\n");
    }

    printf("Setting to 0.1hz  ");

    ret = gpsSendMessage(cfg_rate_01hz, sizeof(cfg_rate_01hz));
    if (ret == 0) {
        printf("success\n");
    }

    printf("Setting to ECO  ");
    ret = gpsSendMessage(cfg_power_eco, sizeof(cfg_power_eco));
    if (ret == 0) {
        printf("success\n");
    }

    gpsSaveHotstartData();

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
                          200 / portTICK_PERIOD_MS); // short timeout
                                                     //
    gpsCalcCheckSum(sentence, messageLengthInc);

    uart_write_bytes(GPS_UART_NUM, (const char *)sentence, messageLengthInc);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    len = uart_read_bytes(GPS_UART_NUM, data, BUF_SIZE,
                          200 / portTICK_PERIOD_MS); // short timeout
    if (data[3] == 0x01) {
        return 0;
    } else {
        return 1;
    }
}
void gpsTask() {

    int len = 0;

    while (1) {
        // doing some reads to clear buffer
        len = uart_read_bytes(GPS_UART_NUM, data, BUF_SIZE,
                              500 / portTICK_PERIOD_MS); // short timeout
                                                         //
        vTaskDelay(3000 / portTICK_PERIOD_MS);
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
            printf("Latitude:  %.7f\n", latitude_deg);
            printf("Longitude: %.7f\n", longitude_deg);
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
        }
    }
}
int gpsSaveHotstartData() {

    uint8_t cfg_msg_posllhdis[] = {
        0xB5, 0x62, // UBX header
        0x06, 0x01, // CFG-MSG
        0x08, 0x00, // payload length = 8
        0x01, 0x02, // NAV-POSLLH
        0x00,       // rate on I2C
        0x00,       // rate on UART1
        0x00,       // rate on UART2
        0x00,       // rate on USB
        0x00,       // rate on SPI
        0x00,       // reserved
        0x00, 0x00  // checksum (calculate)
    };
    uint8_t cfg_msg_timeutcdis[] = {
        0xB5, 0x62, // UBX header
        0x06, 0x01, // CFG-MSG
        0x03, 0x00, // payload length
        0x01, 0x21, // NAV-TIMEUTC
        0x00,       // rate = 1 (every nav solution)
        0x00, 0x00  // checksum (calculate)
    };
    printf("Disabling pos ");
    int ret = gpsSendMessage(cfg_msg_posllhdis, sizeof(cfg_msg_posllhdis));
    if (ret == 0) {
        printf("  success\n");
    }

    printf("Disabling time ");
    ret = gpsSendMessage(cfg_msg_timeutcdis, sizeof(cfg_msg_timeutcdis));
    if (ret == 0) {
        printf("  success\n");
    }
    while (1) {
        printf("polling\n");
        gpsCalcCheckSum(cfg_data_poll, sizeof(cfg_data_poll));

        uart_write_bytes(GPS_UART_NUM, (const char *)cfg_data_poll,
                         sizeof(cfg_data_poll));
        int tlen = uart_read_bytes(GPS_UART_NUM, data, BUF_SIZE,
                                   1000 / portTICK_PERIOD_MS); // short timeout
        printf(" len = %d\n", tlen);
        if (tlen == 56) {
            nvs_handle_t nvs;

            nvs_open("cfg", NVS_READWRITE, &nvs);
            nvs_set_blob(nvs, "hot_start", hot_start_data, 56);
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
