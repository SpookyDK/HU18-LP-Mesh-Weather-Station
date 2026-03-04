#ifndef NEO6MUART
#define NEO6MUART

#include <stdint.h>

#define GPS_UART_NUM UART_NUM_1
#define GPS_DISABLE_PIN 0
#define GPS_RX_PIN 1
#define GPS_TX_PIN 2
#define BUF_SIZE 1024

void gpsInitUart();
void gpsTask();

// In case of many missing things, ensure this is a "ifdef" not a "ifndef"
#ifdef DEFINE_ALL_BYTE_ARRAYS

#include "freertos/idf_additions.h"

static const char *nmea_dis[] = {
    "$PUBX,40,GGA,0,0,0,0,0,0*5A\r\n", "$PUBX,40,GLL,0,0,0,0,0,0*5C\r\n", "$PUBX,40,GSA,0,0,0,0,0,0*4E\r\n",
    "$PUBX,40,GSV,0,0,0,0,0,0*59\r\n", "$PUBX,40,RMC,0,0,0,0,0,0*47\r\n", "$PUBX,40,VTG,0,0,0,0,0,0*5E\r\n",
    "$PUBX,40,ZDA,0,0,0,0,0,0*44\r\n",
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

static uint8_t cfg_prt_ubx_only[] = {
    0xB5, 0x62,             // header
    0x06, 0x00,             // CFG-PRT
    0x14, 0x00,             // payload length = 20 bytes
    0x01,                   // portID = 1 (UART1)
    0x00,                   // reserved
    0x00, 0x00,             // txReady
    0xD0, 0x08, 0x00, 0x00, // mode (8N1, no parity)
    0x80, 0x25, 0x00, 0x00, // baud rate = 9600 (little-endian)
    0x03, 0x00,             // inProtoMask = UBX
    0x01, 0x00,             // outProtoMask = UBX
    0x00, 0x00,             // flags
    0x00, 0x00,             // reserved
    0x00, 0x00              // placeholder checksum
};

static uint8_t aid_ini_poll[] = {
    0xB5, 0x62, // UBX header
    0x0B, 0x01, // AID-INI
    0x00, 0x00, // Payload len = 0
    0x00, 0x00  // Checksum (calculate)
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

static uint8_t poll_nav_status[] = {
    0xB5, 0x62, // UBX header
    0x01, 0x03, // Class = NAV, ID = Status
    0x00, 0x00, // payload length = 0
    0x03, 0x05  // checksum CK_A, CK_B
};

static uint8_t nav_posllh_poll[] = {
    0xB5, 0x62, // UBX header
    0x01, 0x02, // NAV-POSLLH
    0x00, 0x00, // payload length = 0
    0x00, 0x00  // checksum (calculate)
};

static uint8_t nav_timeutc_poll[] = {
    0xB5, 0x62, // UBX header
    0x01, 0x21, // NAV-TIMEUTC
    0x00, 0x00, // payload length = 0
    0x00, 0x00  // checksum (calculate)
};

#define UBX_FRAME_BUF_SIZE 128
#define UBX_FRAME_POOL_SIZE 8

typedef struct {
    uint8_t data[UBX_FRAME_BUF_SIZE];
    uint8_t len;
    bool in_use;
    uint8_t frame_count;
    uint8_t frame_offsets[4];
} ubx_frame_t;

static ubx_frame_t frame_pool[UBX_FRAME_POOL_SIZE];
static uint8_t left_over_buffer[UBX_FRAME_BUF_SIZE];
static uint16_t left_over_buffer_len = 0;

static QueueHandle_t uart_event_queue;
static QueueHandle_t response_queue;
static QueueHandle_t nav_queue;

static uint8_t hot_start_data[56];

#endif // DEFINE_ALL_BYTE_ARRAYS
#endif // !NEO6MUART
