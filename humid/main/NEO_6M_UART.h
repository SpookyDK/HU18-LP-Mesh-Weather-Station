#ifndef NEO6MUART
#define NEO6MUART

#include "freertos/idf_additions.h"
#include <stdint.h>

#define GPS_UART_NUM UART_NUM_1
#define GPS_DISABLE_PIN 0
#define GPS_RX_PIN 1
#define GPS_TX_PIN 2
#define BUF_SIZE 1024
#include <stdint.h> // needed for uint32_t, uint16_t, etc.

static const char *nmea_dis[] = {
    "$PUBX,40,GGA,0,0,0,0,0,0*5A\r\n", "$PUBX,40,GLL,0,0,0,0,0,0*5C\r\n", "$PUBX,40,GSA,0,0,0,0,0,0*4E\r\n",
    "$PUBX,40,GSV,0,0,0,0,0,0*59\r\n", "$PUBX,40,RMC,0,0,0,0,0,0*47\r\n", "$PUBX,40,VTG,0,0,0,0,0,0*5E\r\n",
    "$PUBX,40,ZDA,0,0,0,0,0,0*44\r\n",
};

// Do we ever intend to enable NMEA?
#define ENGGA "$PUBX,40,GGA,0,1,0,0,0,0*5B\r\n"
#define ENGLL "$PUBX,40,GLL,0,1,0,0,0,0*5D\r\n"
#define ENGSA "$PUBX,40,GSA,0,1,0,0,0,0*4F\r\n"
#define ENGSV "$PUBX,40,GSV,0,1,0,0,0,0*58\r\n"
#define ENRMC "$PUBX,40,RMC,0,1,0,0,0,0*46\r\n"
#define ENVTG "$PUBX,40,VTG,0,1,0,0,0,0*5F\r\n"
#define ENZDA "$PUBX,40,ZDA,0,1,0,0,0,0*45\r\n"

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

static uint8_t cfg_inf_poll_protocol[] = {
    0xB5, 0x62, // header
    0x06, 0x02, // CFG-INF
    0x01, 0x00, // payload length
    0x00, 0x00, // Get NMEA protol
    0x00, 0x00  // placeholder checksum
};

static uint8_t cfg_rate_01hz[] = {
    0xB5, 0x62, // UBX header
    0x06, 0x08, // CFG-RATE
    0x06, 0x00, // payload length = 6
    0x10, 0x27, // measRate = 10000 ms (0.1 Hz)
    0x01, 0x00, // navRate = 1
    0x01, 0x00, // timeRef = UTC
    0x00, 0x00  // placeholder checksum → calculate next
};

static uint8_t cfg_rate_high[] = {
    0xB5, 0x62, // UBX header
    0x06, 0x08, // CFG-RATE
    0x06, 0x00, // payload length = 6
    0xE8, 0x03, // measRate = 1000 ms (1 Hz)
    0x01, 0x00, // navRate = 1
    0x00, 0x00, // timeRef = UTC
    0x00, 0x00  // placeholder checksum → calculate next
};

static uint8_t cfg_rate_low[] = {
    0xB5, 0x62, // UBX header
    0x06, 0x08, // CFG-RATE
    0x06, 0x00, // payload length = 6
    0x60, 0xEA, // measRate = 60000 ms
    0x01, 0x00, // navRate = 1
    0x00, 0x00, // timeRef = UTC
    0x00, 0x00  // placeholder checksum → calculate next
};

static uint8_t cfg_power_full[] = {
    0xB5, 0x62, // UBX header
    0x06, 0x11, // CFG-RXM
    0x02, 0x00, // payload len = 2
    0x08,       // Reserved to 8
    0x00,       // Power mode {0: Max performance, 1: Power save, 4: Eco mode}
    0x00, 0x00  // Checksum (calculate)
};

static uint8_t cfg_power_eco[] = {
    0xB5, 0x62, // UBX header
    0x06, 0x11, // CFG-RXM
    0x02, 0x00, // payload len = 2
    0x08,       // Reserved to 8
    0x04,       // Power mode {0: Max performance, 1: Power save, 4: Eco mode}
    0x00, 0x00  // Checksum (calculate)
};

static uint8_t cfg_power_poll[] = {
    0xB5, 0x62, // UBX header
    0x06, 0x11, // CFG-RXM
    0x00, 0x00, // payload len = 0
    0x00, 0x00  // Checksum (calculate)
};

static uint8_t cfg_power_our[] = {
    0xB5, 0x62, // UBX header
    0x06, 0x11, // CFG-RXM
    0x02, 0x00, // payload len = 2
    0x08,       // Reserved to 8
    0x01,       // Power mode {0: Max performance, 1: Power save, 4: Eco mode}
    0x00, 0x00  // Checksum (calculate)
};

static uint8_t cfg_pm2_poll[] = {
    0xB5, 0x62, // UBX header
    0x06, 0x3B, // CFG-PM2
    0x00, 0x00, // payload len = 44
    0x00, 0x00, // Checksum
};

static uint8_t cfg_pm2_whatever[] = {
    0xB5, 0x62,             // Header
    0x06, 0x3B,             // Class/ID: CFG-PM2
    0x2C, 0x00,             // Length: 44 bytes
    0x01,                   // Version
    0x06, 0x00, 0x00,       // Reserved
    0x00, 0x10, 0x00, 0x00, // flags (enable cyclic tracking)
    0x60, 0xEA, 0x00, 0x00, // updatePeriod: 60000ms
    0xE8, 0x03, 0x00, 0x00, // searchPeriod: 1000ms
    0x00, 0x00, 0x00, 0x00, // gridOffset
    0x00, 0x00,             // onTime
    0x00, 0x00,             // minAcqTime
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Reserved
    0x00, 0x00,                                                 // Checksum
};

static uint8_t cfg_pm2_low[] = {
    0xB5, 0x62,             // UBX header
    0x06, 0x3B,             // CFG-PM2
    0x2c, 0x00,             // payload len = 44
    0x01,                   // Version has to be 1
    0x06, 0x00, 0x00,       // Reserved
    0x00, 0x90, 0x02, 0x00, // flags
    0xe8, 0x03, 0x00, 0x00, // Update period = 60000 ms = 60 seconds
    0x10, 0x27, 0x00, 0x00, // Search period = 10000 ms = 10 seconds
    0x00, 0x00, 0x00, 0x00, // grid offset
    0x02, 0x00,             // on time after first fix
    0x00, 0x00,             // min search time
    0x2c, 0x01, 0x00, 0x00, 0x4f, 0xc1, 0x03, 0x00, 0x86, 0x02,
    0x00, 0x00, 0xfe, 0x00, 0x00, 0x00, 0x64, 0x40, 0x01, 0x00, // Reserved
    0x00, 0x00                                                  // Checksum (calculate)
};

static uint8_t aid_ini_poll[] = {
    0xB5, 0x62, // UBX header
    0x0B, 0x01, // AID-INI
    0x00, 0x00, // Payload len = 0
    0x00, 0x00  // Checksum (calculate)
};

static uint8_t cfg_nmea_poll[] = {
    0xB5, 0x62, // UBX header
    0x06, 0x17, // CFG-NMEA
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
static uint8_t cfg_nav_binary[] = {
    0xB5, 0x62, // header
    0x06, 0x01, // CFG-MSG
    0x03, 0x00, // payload length = 3
    0x01, 0x07,
    0x01,      // class 0x01, ID 0x07 (NAV-PVT), rate = 1 (every navigation solution)
    0x00, 0x00 // placeholder checksum
};

static uint8_t poll_cfg_prt[] = {
    0xB5, 0x62, 0x06, 0x00, // CFG-PRT
    0x01, 0x00,             // payload = 1 byte (port ID)
    0x01,                   // ask for UART1
    0x00, 0x00              // checksum
};

static uint8_t poll_nav_posllh[] = {
    0xB5, 0x62, // UBX header
    0x01, 0x02, // Class = NAV, ID = POSLLH
    0x00, 0x00, // payload length = 0
    0x03, 0x05  // checksum CK_A, CK_B
};

static uint8_t poll_nav_status[] = {
    0xB5, 0x62, // UBX header
    0x01, 0x03, // Class = NAV, ID = Status
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

static uint8_t nav_posllh_poll[] = {
    0xB5, 0x62, // UBX header
    0x01, 0x02, // NAV-POSLLH
    0x00, 0x00, // payload length = 0
    0x00, 0x00  // checksum (calculate)
};

static uint8_t cfg_msg_posllh_dis[] = {
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

static uint8_t cfg_msg_timeutc[] = {
    0xB5, 0x62, // UBX header
    0x06, 0x01, // CFG-MSG
    0x08, 0x00, // payload length = 8
    0x01, 0x21, // NAV-TIMEUTC
    0x00,       // rate on I2C
    0x01,       // rate on UART1
    0x00,       // rate on UART2
    0x00,       // rate on USB
    0x00,       // rate on SPI
    0x00,       // reserved
    0x00, 0x00  // checksum
};

static uint8_t nav_timeutc_poll[] = {
    0xB5, 0x62, // UBX header
    0x01, 0x21, // NAV-TIMEUTC
    0x00, 0x00, // payload length = 0
    0x00, 0x00  // checksum (calculate)
};

static uint8_t cfg_msg_timeutc_dis[] = {
    0xB5, 0x62, // UBX header
    0x06, 0x01, // CFG-MSG
    0x08, 0x00, // payload length = 8
    0x01, 0x21, // NAV-TIMEUTC
    0x00,       // rate on I2C
    0x00,       // rate on UART1
    0x00,       // rate on UART2
    0x00,       // rate on USB
    0x00,       // rate on SPI
    0x00,       // reserved
    0x00, 0x00  // checksum
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

void gpsInitUart();
void gpsTask();
#endif // !NEO6MUART
