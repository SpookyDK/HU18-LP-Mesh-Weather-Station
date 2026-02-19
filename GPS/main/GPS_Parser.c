
#include "GPS_Parser.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "portmacro.h"
#include <ctype.h>
#include <driver/uart.h>
#include <inttypes.h>
#include <math.h>
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
    0x01,       // rate on UART1  ✅
    0x00,       // rate on UART2
    0x00,       // rate on USB
    0x00,       // rate on SPI
    0x00,       // reserved

    0x00, 0x00 // checksum (calculate)
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
    0xB5, 0x62, // UBX header
    0x06, 0x09, // CFG-CFG
    0x0D, 0x00, // payload length = 13

    0xFF, 0xFF, 0x00, 0x00, // clearMask (clear all)
    0x00, 0x00, 0x00, 0x00, // saveMask (nothing)
    0x00, 0x00, 0x00, 0x00, // loadMask (nothing)

    0x07, // deviceMask: RAM | BBR | FLASH

    0x00, 0x00 // checksum (calculate)
};
void init_gps_uart() {

    const uart_config_t uart_config = {.baud_rate = 9600,
                                       .data_bits = UART_DATA_8_BITS,
                                       .parity = UART_PARITY_DISABLE,
                                       .stop_bits = UART_STOP_BITS_1,
                                       .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

    uart_param_config(GPS_UART_NUM, &uart_config);
    uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE);
    uart_driver_install(GPS_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);

    printf("Resetting GPS settings\n");
    calcCheckSum(ubx_wipe_settings, sizeof(ubx_wipe_settings));

    uart_write_bytes(GPS_UART_NUM, (const char *)ubx_wipe_settings,
                     sizeof(ubx_wipe_settings));
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

    int len = uart_read_bytes(GPS_UART_NUM, data, BUF_SIZE,
                              100 / portTICK_PERIOD_MS); // short timeout

    printf("Setting full power mode\n");
    calcCheckSum(cfg_power_full, sizeof(cfg_power_full));

    uart_write_bytes(GPS_UART_NUM, (const char *)cfg_power_full,
                     sizeof(cfg_power_full));

    vTaskDelay(200 / portTICK_PERIOD_MS);
    calcCheckSum(cfg_power_get, sizeof(cfg_power_get));

    printf("Asking for power_state\n");
    uart_write_bytes(GPS_UART_NUM, (const char *)cfg_power_get,
                     sizeof(cfg_power_get));
    printf("Reading from gps\n");
    len = uart_read_bytes(GPS_UART_NUM, data, BUF_SIZE,
                          100 / portTICK_PERIOD_MS); // short timeout
                                                     //
    printf("len = %d\n", len);
    for (int i = 0; i < len; i++) {
        if (data[i] == 0xB5) {
            printf("\n");
        }
        printf("%x,", data[i]);
    }
    printf("\n");

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    calcCheckSum(cfg_data_poll, sizeof(cfg_data_poll));
    printf("Polling data\n");
    uart_write_bytes(GPS_UART_NUM, (const char *)cfg_data_poll,
                     sizeof(cfg_data_poll));

    printf("Reading relod cords\n");
    len = uart_read_bytes(GPS_UART_NUM, data, BUF_SIZE,
                          200 / portTICK_PERIOD_MS); // short timeout
    for (int i = 0; i < len; i++) {
        if (data[i] == 0xB5) {
            printf("\n");
        }
        printf("%x,", data[i]);
    }
    printf("\n");

    vTaskDelay(100 / portTICK_PERIOD_MS);

    calcCheckSum(cfg_nav5_stationary_3d, sizeof(cfg_nav5_stationary_3d));
    printf("Setting to stationary\n");
    uart_write_bytes(GPS_UART_NUM, (const char *)cfg_nav5_stationary_3d,
                     sizeof(cfg_nav5_stationary_3d));

    printf("Reading ack cords\n");
    len = uart_read_bytes(GPS_UART_NUM, data, BUF_SIZE,
                          200 / portTICK_PERIOD_MS); // short timeout
    for (int i = 0; i < len; i++) {
        if (data[i] == 0xB5) {
            printf("\n");
        }
        printf("%x,", data[i]);
    }
    printf("\n");

    vTaskDelay(100 / portTICK_PERIOD_MS);

    calcCheckSum(cfg_prt_ubx, sizeof(cfg_prt_ubx));
    printf("Setting to ubx\n");
    uart_write_bytes(GPS_UART_NUM, (const char *)cfg_prt_ubx,
                     sizeof(cfg_prt_ubx));

    printf("Reading ack cords\n");
    len = uart_read_bytes(GPS_UART_NUM, data, BUF_SIZE,
                          200 / portTICK_PERIOD_MS); // short timeout
    for (int i = 0; i < len; i++) {
        if (data[i] == 0xB5) {
            printf("\n");
        }
        printf("%x,", data[i]);
    }
    printf("\n");

    vTaskDelay(100 / portTICK_PERIOD_MS);

    printf("Setting to 0.1hz\n");

    calcCheckSum(cfg_rate_01hz, sizeof(cfg_rate_01hz));

    uart_write_bytes(GPS_UART_NUM, (const char *)cfg_rate_01hz,
                     sizeof(cfg_rate_01hz));

    printf("Setting to ECO\n");
    calcCheckSum(cfg_power_eco, sizeof(cfg_power_eco));

    uart_write_bytes(GPS_UART_NUM, (const char *)cfg_power_eco,
                     sizeof(cfg_power_eco));

    len = uart_read_bytes(GPS_UART_NUM, data, BUF_SIZE,
                          200 / portTICK_PERIOD_MS); // short timeout
    for (int i = 0; i < len; i++) {
        if (data[i] == 0xB5) {
            printf("\n");
        }
        printf("%x,", data[i]);
    }
    printf("\n");

    printf("\n");

    printf("Setting to Enabling pos update\n");
    calcCheckSum(cfg_msg_posllh, sizeof(cfg_msg_posllh));

    uart_write_bytes(GPS_UART_NUM, (const char *)cfg_msg_posllh,
                     sizeof(cfg_msg_posllh));

    len = uart_read_bytes(GPS_UART_NUM, data, BUF_SIZE,
                          200 / portTICK_PERIOD_MS); // short timeout
    for (int i = 0; i < len; i++) {
        if (data[i] == 0xB5) {
            printf("\n");
        }
        printf("%x,", data[i]);
    }
    printf("\n");

    printf("Setting to Enabling UTC update\n");
    calcCheckSum(cfg_msg_timeutc, sizeof(cfg_msg_timeutc));

    uart_write_bytes(GPS_UART_NUM, (const char *)cfg_msg_timeutc,
                     sizeof(cfg_msg_timeutc));

    len = uart_read_bytes(GPS_UART_NUM, data, BUF_SIZE,
                          200 / portTICK_PERIOD_MS); // short timeout
    for (int i = 0; i < len; i++) {
        if (data[i] == 0xB5) {
            printf("\n");
        }
        printf("%x,", data[i]);
    }
    printf("\n");
    len = uart_read_bytes(GPS_UART_NUM, data, BUF_SIZE,
                          500 / portTICK_PERIOD_MS); // short timeout
                                                     //
    calcCheckSum(poll_nav_posllh, sizeof(poll_nav_posllh));
    while (1) {
        // printf("reading\n");
        len = uart_read_bytes(GPS_UART_NUM, data, BUF_SIZE,
                              500 / portTICK_PERIOD_MS); // short timeout
                                                         //
        // uart_write_bytes(GPS_UART_NUM, (const char *)poll_nav_posllh,
        //                  sizeof(poll_nav_posllh));
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        len = uart_read_bytes(GPS_UART_NUM, data, BUF_SIZE,
                              500 / portTICK_PERIOD_MS); // short timeout
        if (len == 0) {
            printf("recieved nothing\n");
            continue;
        }

        printf("something recieved len = %d \n", len);
        // check if first message is pos
        if (len == 64 && data[2] == 0x01 && data[3] == 0x02) {
            for (int i = 0; i < len; i++) {
                printf("%x,", data[i]);
            }
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

            // UTC time

            printf("Latitude:  %.7f\n", latitude_deg);
            printf("Longitude: %.7f\n", longitude_deg);
            printf("\n");
        }
        // check if second part of message is time
        printf("%x %x \n", data[38], data[39]);
        if (len == 64 && data[38] == 0x01 && data[39] == 0x21) {
            printf("parsing time\n");
            uint32_t iTOW = data[40] | (data[41] << 8) | (data[42] << 16) |
                            (data[43] << 24);
            uint32_t tAcc = data[44] | (data[45] << 8) | (data[46] << 16) |
                            (data[47] << 24);
            uint16_t year = data[48] | (data[49] << 8);
            uint8_t month = data[50];
            uint8_t day = data[51];
            uint8_t hour = data[52];
            uint8_t min = data[53];
            uint8_t sec = data[54];
            uint8_t valid = data[55];
            int32_t nano = data[56] | (data[57] << 8) | (data[58] << 16) |
                           (data[59] << 24);

            if (valid & 0x04) {
                printf("UTC: %" PRIu16 "-%02" PRIu8 "-%02" PRIu8 " %02" PRIu8
                       ":%02" PRIu8 ":%02" PRIu8 ".%" PRId32 "\n",
                       year, month, day, hour, min, sec, nano);
            }
        }
    }
}

int calcCheckSum(uint8_t *sentence, uint8_t messageLenghtInc) {
    uint8_t CK_A = 0, CK_B = 0;

    for (int i = 2; i < messageLenghtInc - 2; i++) {
        CK_A = CK_A + sentence[i];
        CK_B = CK_B + CK_A;
    }
    sentence[messageLenghtInc - 2] = CK_A;
    sentence[messageLenghtInc - 1] = CK_B;

    return 0;
}

// standard NMEA checksum
int CheckSumCheck_NMEA(const char *sentence) {
    unsigned char checksum = 0;
    // Skip inital $
    if (*sentence == '$') // skip $
        sentence++;

    // XOR checksum
    while (*sentence && *sentence != '*') {
        checksum ^= (unsigned char)(*sentence);
        sentence++;
    }

    // $GPGGA,124612.00,5700.80660,N,00959.16325,E,1,07,1.15,-6.9,M,42.4,M,,*70
    if (*sentence == '*' && *(sentence + 1) && *(sentence + 2)) {
        unsigned char ggchecksum =
            (unsigned char)strtol(sentence + 1, NULL, 16);
        return checksum == ggchecksum;
    }
    return 0;
}

/* Helper to get the next token boundaries while preserving empty tokens.
   p points at start of token. returns pointer to delimiter or NULL if last
   token. toklen = length of token (may be 0). */
static char *find_next_delim(char *start, char **tok_start, size_t *toklen) {
    char *c = start;
    *tok_start = start;
    while (*c && *c != ',' && *c != '*')
        c++;
    *toklen = (size_t)(c - start);
    return (*c == '\0') ? NULL : c; // pointer to delimiter or NULL
}

// parses the GPGGA message
int parseGPGGA(const char *sentence, struct GPGGA_Message *msg) {
    if (!CheckSumCheck_NMEA(sentence))
        return 0;

    memset(msg, 0, sizeof(*msg)); // clear struct each time

    char copy[256];
    strncpy(copy, sentence, sizeof(copy) - 1);
    copy[sizeof(copy) - 1] = '\0';

    char *p = copy;
    int field = 0;

    while (1) {
        char *tok_start = NULL;
        size_t toklen = 0;
        char *delim = find_next_delim(p, &tok_start, &toklen);

        // process token [tok_start .. tok_start+toklen-1]
        // token may be empty (toklen == 0)
        switch (field) {
        case 0: // ID (skip leading $)
            if (toklen > 0 && tok_start[0] == '$') {
                size_t cp_len = toklen - 1;
                if (cp_len > sizeof(msg->ID) - 1)
                    cp_len = sizeof(msg->ID) - 1;
                strncpy(msg->ID, tok_start + 1, cp_len);
                msg->ID[cp_len] = '\0';
            }
            break;
        case 1: // UTC
            if (toklen) {
                char tmp[32];
                memcpy(tmp, tok_start, toklen);
                tmp[toklen] = '\0';
                msg->UTC = atof(tmp);
            }
            break;
        case 2: // latitude raw DDMM.MMMM
            if (toklen) {
                char tmp[32];
                memcpy(tmp, tok_start, toklen);
                tmp[toklen] = '\0';
                msg->latitude = atof(tmp);
            }
            break;
        case 3: // N/S
            if (toklen)
                msg->direction_Latitude = tok_start[0];
            break;
        case 4: // longitude raw DDDMM.MMMM
            if (toklen) {
                char tmp[32];
                memcpy(tmp, tok_start, toklen);
                tmp[toklen] = '\0';
                msg->longitude = atof(tmp);
            }
            break;
        case 5: // E/W
            if (toklen)
                msg->direction_Longitude = tok_start[0];
            break;
        case 6: // fix quality
            if (toklen) {
                char tmp[16];
                memcpy(tmp, tok_start, toklen);
                tmp[toklen] = '\0';
                msg->GPS_Quality_indicator = atoi(tmp);
            }
            break;
        case 7: // number of satellites
            if (toklen) {
                char tmp[16];
                memcpy(tmp, tok_start, toklen);
                tmp[toklen] = '\0';
                msg->number_of_GPS = atoi(tmp);
            }
            break;
        case 8: // HDOP
            if (toklen) {
                char tmp[16];
                memcpy(tmp, tok_start, toklen);
                tmp[toklen] = '\0';
                msg->HDOP = atof(tmp);
            }
            break;
        case 9: // altitude
            if (toklen) {
                char tmp[32];
                memcpy(tmp, tok_start, toklen);
                tmp[toklen] = '\0';
                msg->height = atof(tmp);
            }
            break;
        case 10: // altitude unit
            if (toklen)
                msg->unit_of_height = tok_start[0];
            break;
        case 11: // geoid separation
            if (toklen) {
                char tmp[32];
                memcpy(tmp, tok_start, toklen);
                tmp[toklen] = '\0';
                msg->geoid_Separation = atof(tmp);
            }
            break;
        case 12: // geoid unit
            if (toklen)
                msg->geoid_unit = tok_start[0];
            break;
        case 14: // checksum token (after '*' — we treat this position as
                 // checksum)
            if (toklen) {
                // copy up to 2 hex digits
                size_t i = 0;
                for (; i < toklen && i < (sizeof(msg->checkSum) - 1); ++i) {
                    if (!isxdigit((unsigned char)tok_start[i]))
                        break;
                    msg->checkSum[i] = tok_start[i];
                }
                msg->checkSum[i] = '\0';
            }
            break;
        default:
            break;
        }

        // if there is no delimiter, we're at the last token -> done
        if (!delim)
            break;

        // advance to char after delimiter
        p = delim + 1;
        field++;
        // loop continues; this preserves empty fields since toklen may be 0
    }
    return 1;
}

// Convert raw NMEA lat/lon (DDMM.MMMM) to decimal degrees
double convertToDecimalDegrees(
    float raw,
    char dir) { // weird math to convert the time position to degrees
    if (raw == 0 || dir == '\0')
        return 0.0;
    int degrees = (int)(raw / 100);
    double minutes = raw - (degrees * 100);
    double decimal = degrees + minutes / 60.0;
    if (dir == 'S' || dir == 'W')
        decimal *= -1.0;
    return decimal;
}

int GPSParser(char *sentence, struct Coordinate *retCoordinate) {
    struct GPGGA_Message msg;
    // struct Coordinate cords;
    retCoordinate->lat = 0;
    retCoordinate->lon = 0;

    if (!parseGPGGA(sentence, &msg)) { // checks if the sentence is valid.
        // retCoordinate= cords;
        return 0;
    }
    // converts to decimal degrees
    retCoordinate->lat =
        (float)convertToDecimalDegrees(msg.latitude, msg.direction_Latitude);
    retCoordinate->lon =
        (float)convertToDecimalDegrees(msg.longitude, msg.direction_Longitude);

    retCoordinate->UTC = msg.UTC;
    retCoordinate->height = msg.height;

    return 1;
}

int main(void) { // eksempel på hvordan GPSParser skal kaldes/bruges
    struct Coordinate out1;
    struct Coordinate out2;
    struct Coordinate out3;
    struct Coordinate out4;

    const char *text1 = "$GPGGA,102903.60,5700.82752,N,00959.18060,E,1,05,2.90,"
                        "15.6,M,42.4,M,,*6F";
    const char *text2 = "$GPGGA,103004.50,5730.12345,N,01001.56789,E,1,08,1.20,"
                        "20.5,M,45.0,M,,*68";
    const char *text3 = "$GPGGA,102903.40,5700.82745,N,00959.18057,E,1,05,2.90,"
                        "15.5,M,42.4,M,,*6C\n";
    const char *text4 = "$GPGGA,102903.00,,,,,0,00,99.99,,,,,,*6F";

    GPSParser(text1, &out1);
    GPSParser(text2, &out2);
    GPSParser(text3, &out3);
    GPSParser(text4, &out4);

    printf("%f, %f\n", out1.lat, out1.lon); // Lat/Lon from first sentence
    printf("%f, %f\n", out2.lat, out2.lon); // Lat/Lon from second sentence
    printf("%f, %f\n", out3.lat, out3.lon); // Lat/Lon from second sentence
    printf("%f, %f\n", out4.lat, out4.lon);

    return 0;
}

double distance_meters(double lat1, double lon1, double lat2, double lon2) {
    // Convert degrees to radians once
    double rad = M_PI / 180.0;

    // Average latitude (radians)
    double lat_avg = (lat1 + lat2) * 0.5 * rad;

    // Latitude/Longitude differences in radians
    double dlat = (lat2 - lat1) * rad;
    double dlon = (lon2 - lon1) * rad;

    // Cosine approximation for “compression” of longitude
    double cos_lat = cos(lat_avg);

    // Scale deltas
    double x = dlon * 6371000.0 * cos_lat; // east-west distance
    double y = dlat * 6371000.0;           // north-south distance

    // Pythagoras for near-flat distances
    return sqrt(x * x + y * y);
}

volatile struct Coordinate newCoordinate;

void gps_task(void *arg) {
    printf("starting task\n");
    while (1) {
        vTaskDelay(200 / portTICK_PERIOD_MS);

        int len = uart_read_bytes(GPS_UART_NUM, data, BUF_SIZE,
                                  5000 / portTICK_PERIOD_MS); // short timeout
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                printf("%c", data[i]); // print each byte in hex
            }
            GPSParser((char *)data, &newCoordinate);
            printf("\n%f, %f, time: %.1f, height: %i\n", newCoordinate.lat,
                   newCoordinate.lon, newCoordinate.UTC, newCoordinate.height);
            if (lastCoordinate.lat == 0 &&
                lastCoordinate.lon == 0) { // checks if it loses connection
                lastCoordinate = newCoordinate;
            }

        } else {
            printf("No Data recieved\n");
            continue;
        }
        printf("distance: %f\n",
               distance_meters(newCoordinate.lat, newCoordinate.lon,
                               lastCoordinate.lat, lastCoordinate.lon));

        if (distance_meters(newCoordinate.lat, newCoordinate.lon,
                            lastCoordinate.lat,
                            lastCoordinate.lon) < 5) { // 2 er måske bedst
            ESP_LOGW("GPS", "distance too small");
            continue;
        }
    }
}
