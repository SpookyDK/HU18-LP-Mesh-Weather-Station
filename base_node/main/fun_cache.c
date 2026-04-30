#include "esp_log.h"
#include "fun_cache.h"
#include "packet_def.h"
#include <stdbool.h>
#include <stdint.h>

#define CACHE_SIZE 16
#define CACHE_MASK (CACHE_SIZE - 1)

static uint8_t node_map[32] = {0};

typedef struct {
    uint8_t node_id;
    uint16_t nonce;
} node_nonce_t;

static node_nonce_t node_cache[CACHE_SIZE];
static uint8_t node_write_idx = 0;

uint8_t is_nonce_known(uint16_t nonce) {
    for (int i = 0; i < CACHE_SIZE; i++) {
        if (node_cache[i].nonce == nonce) {
            return node_cache[i].node_id;
        }
    }
    return 0;
}

uint16_t is_node_known(uint8_t node_id) {
    for (int i = 0; i < CACHE_SIZE; i++) {
        if (node_cache[i].node_id == node_id) {
            return node_cache[i].nonce;
        }
    }
    return 0;
}

uint8_t get_free_node_id(uint16_t nonce) {
    uint8_t nid = is_nonce_known(nonce);
    if (nid)
        return nid;
    for (int i = 0; i < 32; i++) {
        if (node_map[i] == 0xFF)
            continue;
        for (int bit = 0; bit < 8; bit++) {
            if (node_map[i] & (1 << bit))
                continue;
            uint16_t id = (i * 8) + bit;
            if (id > 0 && id <= 255 && !is_node_known((uint8_t)id)) {
                node_cache[node_write_idx].node_id = (uint8_t)id;
                node_cache[node_write_idx].nonce = nonce;
                node_write_idx = (node_write_idx + 1) & CACHE_MASK;
                return (uint8_t)id;
            }
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
