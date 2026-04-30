#include "fun_cache.h"
#include "packet_def.h"
#include <stdbool.h>
#include <stdint.h>

static uint8_t node_map[32];

void init_node_map() {
    for (int i = 0; i < 32; i++)
        node_map[i] = 0;
    node_map[0] |= 1; // Make sure the very first index is allocated, as that is this server
}

uint8_t get_free_node_id() {
    for (int i = 0; i < 32; i++) {
        if (node_map[i] == 0xFF)
            continue;
        for (int bit = 0; bit < 8; bit++) {
            if (node_map[i] & (1 << bit))
                continue;
            uint16_t id = (i * 8) + bit;
            if (id > 0 && id <= 255)
                return (uint8_t)id;
        }
    }
    return 0;
}

void set_node_id_state(uint8_t id, bool taken) {
    if (taken) {
        node_map[id / 8] |= (1 << (id % 8));
    } else {
        node_map[id / 8] &= ~(1 << (id % 8));
    }
}

/// ===================================
///     Duplicate packet protection
/// ===================================

#define CACHE_SIZE 16
#define CACHE_MASK (CACHE_SIZE - 1)

typedef struct {
    uint8_t packet_id;
    uint8_t node_id;
    // uint16_t nonce; might be needed dependant on how the nonce is implemented
} packet_signature_t;

static packet_signature_t packet_cache[CACHE_SIZE];
static uint8_t write_idx = 0;

bool is_duplicate(packet_header_t head) {
    for (int i = 0; i < CACHE_SIZE; i++) {
        if (packet_cache[i].node_id == head.orig_node_id && packet_cache[i].packet_id == head.packet_id) {
            return true;
        }
    }
    packet_cache[write_idx].packet_id = head.packet_id;
    packet_cache[write_idx].node_id = head.orig_node_id;
    write_idx = (write_idx + 1) & CACHE_MASK;
    return false;
}
