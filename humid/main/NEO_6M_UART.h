#ifndef NEO6MUART
#define NEO6MUART

#include "freertos/idf_additions.h"
#include <stdint.h>

#define GPS_UART_NUM UART_NUM_1
#define GPS_RX_PIN 1
#define GPS_TX_PIN 2
#define BUF_SIZE 1024
#include <stdint.h> // needed for uint32_t, uint16_t, etc.

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

static uint8_t cfg_msg_posllhdis[] = {
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
static uint8_t cfg_msg_timeutcdis[] = {
    0xB5, 0x62, // UBX header
    0x06, 0x01, // CFG-MSG
    0x03, 0x00, // payload length
    0x01, 0x21, // NAV-TIMEUTC
    0x00,       // rate = 1 (every nav solution)
    0x00, 0x00  // checksum (calculate)
};
static uint8_t hot_start_data[56];
static uint8_t cfg_rate_01hz[] = {
    0xB5, 0x62, // UBX header
    0x06, 0x08, // CFG-RATE
    0x06, 0x00, // payload length = 6
    0x10, 0x27, // measRate = 10000 ms (0.1 Hz)
    0x01, 0x00, // navRate = 1
    0x01, 0x00, // timeRef = UTC
    0x00, 0x00  // placeholder checksum → calculate next
};

static uint8_t cfg_rate_5hz[] = {
    0xB5, 0x62, // UBX header
    0x06, 0x08, // CFG-RATE
    0x06, 0x00, // payload length = 6
    0x68, 0x00, // measRate = 200 ms (5 Hz)
    0x01, 0x00, // navRate = 1
    0x01, 0x00, // timeRef = UTC
    0x00, 0x00  // placeholder checksum → calculate next
};

static uint8_t cfg_rate_1m[] = {
    0xB5, 0x62, // UBX header
    0x06, 0x08, // CFG-RATE
    0x06, 0x00, // payload length = 6
    0x60, 0xEA, // measRate = 60000 ms (0.0 Hz)
    0x01, 0x00, // navRate = 1
    0x01, 0x00, // timeRef = UTC
    0x00, 0x00  // placeholder checksum → calculate next
};
static uint8_t cfg_power_eco[] = {0xB5, 0x62, 0x06, 0x11, 0x02,
                                  0x00, 0x08, 0x01, 0x00, 0x00};

static uint8_t cfg_power_full[] = {0xB5, 0x62, 0x06, 0x11, 0x02,
                                   0x00, 0x08, 0x00, 0x00, 0x00};

static uint8_t cfg_power_get[] = {0xB5, 0x62, 0x06, 0x11,
                                  0x00, 0x00, 0x00, 0x00};

static uint8_t cfg_data_poll[] = {
    0xB5, 0x62, 0x0B, 0x01, 0x00, 0x00, 0x0C, 0x2D,
};

static uint8_t cfg_nav5_stationary_3d[] = {
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
static uint8_t cfg_nav_binary[] = {
    0xB5, 0x62, // header
    0x06, 0x01, // CFG-MSG
    0x03, 0x00, // payload length = 3
    0x01, 0x07,
    0x01, // class 0x01, ID 0x07 (NAV-PVT), rate = 1 (every navigation solution)
    0x00, 0x00 // placeholder checksum
};
static uint8_t cfg_prt_ubx[] = {
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

static uint8_t poll_nav_posllh[] = {
    0xB5, 0x62, // UBX header
    0x01, 0x02, // Class = NAV, ID = POSLLH
    0x00, 0x00, // payload length = 0
    0x03, 0x05  // checksum CK_A, CK_B
};
static uint8_t cfg_msg_posllh[] = {
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
static uint8_t cfg_msg_timeutc[] = {
    0xB5, 0x62, // UBX header
    0x06, 0x01, // CFG-MSG
    0x03, 0x00, // payload length
    0x01, 0x21, // NAV-TIMEUTC
    0x01,       // rate = 1 (every nav solution)
    0x00, 0x00  // checksum (calculate)
};

static uint8_t ubx_wipe_settings[] = {
    0xB5, 0x62,             // UBX header
    0x06, 0x09,             // CFG-CFG
    0x0D, 0x00,             // payload length = 13
    0xFF, 0xFF, 0x00, 0x00, // clearMask (clear all)
    0x00, 0x00, 0x00, 0x00, // saveMask (nothing)
    0x00, 0x00, 0x00, 0x00, // loadMask (nothing)
    0x07,                   // deviceMask: RAM | BBR | FLASH
    0x00, 0x00              // checksum (calculate)
};
int gpsCalcCheckSum(uint8_t *sentence, uint8_t messageLengthInc);
int gpsSendMessage(uint8_t *sentencte, uint8_t messageLengthInc);
void gpsInitUart();
void gpsTask();
int gpsSaveHotStartData();
static uint8_t data[1024];
#endif // !NEO6MUART
