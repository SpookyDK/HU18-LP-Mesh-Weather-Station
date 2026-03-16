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
 * @version 3
 * @param longitude Is scaled with 1e-7
 * @param latitude Is scaled with 1e-7
 * @param air_humidity From 0 to 100%RH
 * @param air_tempeture Is scaled with 10
 * @param soil_tempeture[4] Is scaled with 16
 * @param soil_moisture From 0 to 100% where 0 is our measured lowest moisture and 100% is most
 * @param pressure The diff from 100000 Pa
 * @param lux Lux
 * @param precipitation In units of 100 micro meters per hour
 * @param wind_speed In 0.1 m/s
 */
typedef struct __attribute__((packed)) {
    int32_t longitude;
    int32_t latitude;
    uint8_t air_humidity;
    int16_t air_tempeture;
    int16_t soil_tempeture[4];
    uint8_t soil_moisture;
    int16_t pressure;
    uint16_t lux;
    uint16_t precipitation; // TODO: Missing Sensor
    uint16_t wind_speed;    // TODO: Missing Sensor
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
