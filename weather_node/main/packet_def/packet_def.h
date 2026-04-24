#ifndef PACKET_DEF_H
#define PACKET_DEF_H

#include "esp_bit_defs.h"
#include <stdint.h>

/**
 * Packet Header
 * @version 4
 * @param a staticly written header to identify the thing
 * @param packet_id Is a squential series that is incremented from each node
 * @param hop_count Decrementing until zero, then drop
 * @param flags
 */
typedef struct __attribute__((packed)) {
    uint8_t header;
    uint8_t network_id;
    uint8_t orig_node_id;
    uint8_t packet_id;
    uint8_t hop_count;
    uint8_t flags;
} packet_header_t;

#define PACKET_HEADER_VALUE 0x69

/**
 * Main Struct
 * @version 4
 * @param longitude Is scaled with 1e-7
 * @param latitude Is scaled with 1e-7
 * @param air_humidity From 0 to 100%RH
 * @param air_tempeture Is scaled with 10
 * @param soil_tempeture[4] Is scaled with 16
 * @param soil_moisture From 0 to 100% where 0 is our measured lowest moisture and 100% is most
 * @param pressure The diff from 100000 Pa
 * @param spectrum spectrum including both Lux and Ir
 * @param precipitation Gives the amount since last reading/packet, scaled with 10
 * @param wind_speed In mili meters per second
 * @param solar_output Solar panel power production in mA scaled with 4
 * @param bat_voltage The diff from 3.5 V in 0.04 V increments
 */
typedef struct __attribute__((packed)) {
    int32_t longitude;
    int32_t latitude;
    uint8_t air_humidity;
    int16_t air_tempeture;     // Can be 0, Will just send a warning
    int16_t soil_tempeture[4]; // Can be 0, Will just send a warning
    uint8_t soil_moisture;     // Can be 0, Assume out of range, Implemented
    int16_t pressure;          // Can be 0, Might work
    uint16_t spectrum;
    uint16_t precipitation; // Can be 0, how would i check? Should i base it wheter its been initated?
    uint16_t wind_speed;    // Can be 0, --//--
    uint8_t solar_output;   // Can be 0, Implemented
    int8_t bat_voltage;
} sensor_payload_t;

/**
 * Full Packet
 * @version 2
 */
typedef struct __attribute__((packed)) {
    packet_header_t head;
    sensor_payload_t payload;
} full_packet_t;

typedef struct __attribute__((packed)) {
    packet_header_t head;
    uint16_t nonce;
} pairing_packet_t;

enum PACKET_BIT_FLAG {
    FLAG_DHT = BIT(7),
    FLAG_DS18B20 = BIT(6),
    FLAG_PRESSURE = BIT(5),
    FLAG_MOIST = BIT(4),
    FLAG_SOLAR = BIT(3),
};
#endif // !PACKET_DEF_H
