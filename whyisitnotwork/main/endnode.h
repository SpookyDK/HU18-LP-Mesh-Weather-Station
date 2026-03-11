#pragma once

// This is the header file for the endnode (Weather Stations).
#include <stdint.h>
#include <stddef.h>


// Node Configuration
#define MAX_RELAY_HOPS 2 // Star Topoogy modified to allow smaller jumps
#define ONEWIRE_MAX_DEVICES 4 // Max number of temp sensors soil

// SPI Setup
#define PIN_NUM_MISO  13
#define PIN_NUM_MOSI  14
#define PIN_NUM_SCK   25
#define PIN_NUM_NSS   26
#define PIN_NUM_DIO0  27

// SX1276 Register configuration we dont need it all but I cannot be bothered to find out atm
#define REG_FIFO                    0x00  // RegFifo
#define REG_OP_MODE                 0x01  // RegOpMode
#define REG_FRF_MSB                 0x06  // RegFrfMsb
#define REG_FRF_MID                 0x07  // RegFrfMid
#define REG_FRF_LSB                 0x08  // RegFrfLsb
#define REG_PA_CONFIG               0x09  // RegPaConfig
#define REG_LNA                     0x0C  // RegLna
#define REG_FIFO_ADDR_PTR           0x0D  // RegFifoAddrPtr
#define REG_FIFO_TX_BASE_ADDR       0x0E  // RegFifoTxBaseAddr (default 0x80)
#define REG_FIFO_RX_BASE_ADDR       0x0F  // RegFifoRxBaseAddr (default 0x00)
#define REG_FIFO_RX_CURRENT_ADDR    0x10  // RegFifoRxCurrentAddr
#define REG_IRQ_FLAGS_MASK          0x11  // RegIrqFlagsMask
#define REG_IRQ_FLAGS               0x12  // RegIrqFlags
#define REG_RX_NB_BYTES             0x13  // RegRxNbBytes
#define REG_PKT_SNR_VALUE           0x19  // RegPktSnrValue (two's complement * 4)
#define REG_PKT_RSSI_VALUE          0x1A  // RegPktRssiValue
#define REG_RSSI_VALUE              0x1B  // RegRssiValue (current)
#define REG_MODEM_CONFIG_1          0x1D  // BW, CodingRate, HeaderMode
#define REG_MODEM_CONFIG_2          0x1E  // SF, TxContinuous, RxPayloadCrcOn
#define REG_PREAMBLE_MSB            0x20  // RegPreambleMsb
#define REG_PREAMBLE_LSB            0x21  // RegPreambleLsb
#define REG_PAYLOAD_LENGTH          0x22  // RegPayloadLength
#define REG_FIFO_RX_BYTE_ADDR       0x25  // RegFifoRxByteAddr
#define REG_MODEM_CONFIG_3          0x26  // LowDataRateOptimize, AgcAutoOn
#define REG_DETECTION_OPTIMIZE      0x31  // RegDetectOptimize + AutomaticIFOn errata
#define REG_DETECTION_THRESHOLD     0x37  // RegDetectionThreshold
#define REG_SYNC_WORD               0x39  // RegSyncWord (0x34 = LoRaWAN reserved)
#define REG_VERSION                 0x42  // RegVersion — SX1276 returns 0x12

// Operation Modes uses bit [2:0] of RegOpMode (Lora is bit 7)
#define MODE_LONG_RANGE_MODE       0x80  // LoRa
#define MODE_SLEEP                 0x00
#define MODE_STDBY                 0x01
#define MODE_TX                    0x03
#define MODE_RX_CONTINUOUS         0x05

// IRQ (Interrupt Request) Flags
#define IRQ_RX_TIMEOUT          0x80 // 7
#define IRQ_RX_DONE             0x40 // 6
#define IRQ_PAYLOAD_CRC_ERROR   0x20 // 5    
#define IRQ_VALID_HEADER        0x10 // 4    
#define IRQ_TX_DONE             0x08 // 3    
#define IRQ_CAD_DONE            0x04 // 2    
#define IRQ_FHSS_CHANGE_CH      0x02 // 1    
#define IRQ_CAD_DETECTED        0x01 // 0

// EU868 Channel 1
#define FRF_MSB_868_1            0xD9
#define FRF_MID_868_1            0x06
#define FRF_LSB_868_1            0x66

// Modem Configs (1-3) in order
// 1 BW 125KHz, Coding Rate 4/5, explicit header mode(0)
#define LORA_MODEM_CONFIG1 0x72
// 2 SF 12 TX continuous mode = 0, CRC on = 1, symb timeout = 0 
#define LORA_MODEM_CONFIG2 0xC4
// 3 LowDataRateOptimize = 1, AgcAutoOn = 1, (mandatory cause symbol time > 16ms)
#define LORA_MODEM_CONFIG3 0x0C

// Sync word - will be shared across all devices hopefully should be handled by network ID
#define LORA_SYNC_WORD 0xF3

// Flash storage keys
#define NVS_NAMESPACE      "lora_node"
#define NVS_KEY_NETWORK_ID "network_id" // uint16
#define NVS_KEY_CONFIGURED "configured" // uint8 (0 or 1, unconfigured or configured/paired)

// Pairing types
#define PKT_TYPE_PAIR_BEACON 0xF0
#define PKT_TYPE_PAIR_ACK    0xF1

typedef struct __attribute__((packed)) {
    uint8_t pkt_type; // set to PKT_TYPE_PAIR_BEACON or PKT_TYPE_PAIR_ACK
    uint16_t network_id;
} pair_packet_t;


// Header / Routing info
typedef struct __attribute__((packed)) {
    uint8_t node_id;
    uint16_t network_id;
    uint8_t orig_id;
    uint8_t hop_count;
} mesh_header_t;

// Actual Sensor Payload
typedef struct __attribute__((packed)) {
    int32_t longitude;
    int32_t latitude;
    uint8_t air_humidity;
    int16_t air_temperature;
    int16_t soil_temperature[ONEWIRE_MAX_DEVICES];
    uint8_t soil_moisture;
    int16_t pressure;
    uint16_t lux;
    uint16_t wind_speed;
    uint8_t perceptation;
} mesh_payload_t;


// Full packet
typedef struct __attribute__((packed)) {
    mesh_header_t  header;
    mesh_payload_t payload;
    uint16_t crc16;
} mesh_packet_t;

#define MESH_PACKET_SIZE sizeof(mesh_packet_t) // 32

void node_send_sensor_data(mesh_packet_t *pkt);
