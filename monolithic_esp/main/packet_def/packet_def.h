#ifndef PACKET_DEF_H
#define PACKET_DEF_H

#include <stdint.h>

/**
 * Packet Header
 * @version 3
 * @param packet_id Is a unique identifier per packet
 * @param hop_count Decrementing until zero, then drop
 * @param flags Currently not used
 */
typedef struct __attribute__((packed)) {
    uint8_t network_id;
    uint8_t orig_node_id;
    uint16_t packet_id;
    uint8_t hop_count;
    uint8_t flags;
} packet_header_t;

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
 * @param bat_voltage The diff from 3.7 V in 0.01 V increments
 */
typedef struct __attribute__((packed)) {
    int32_t longitude;
    int32_t latitude;
    uint8_t air_humidity;
    int16_t air_tempeture;     // Can be 0, Will just send a warning
    int16_t soil_tempeture[4]; // Can be 0, Will just send a warning
    uint8_t soil_moisture;     // Can be 0
    int16_t pressure;          // Can be 0, Might work
    uint16_t spectrum;
    uint16_t precipitation; // Can be 0, how would i check? Should i base it wheter its been initated?
    uint16_t wind_speed;    // Can be 0, --//--
    uint8_t solar_output;
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

#endif // !PACKET_DEF_H
