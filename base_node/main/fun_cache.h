#ifndef FUN_CACHE
#define FUN_CACHE

#include "packet_def.h"
#include <stdbool.h>
#include <stdint.h>

void init_node_map();
uint8_t get_free_node_id(uint16_t nonce);
void set_node_id_state(uint8_t id, bool taken);
uint8_t is_nonce_known(uint16_t nonce);
uint16_t is_node_known(uint8_t node_id);

/*
 * Determine if a header has already been received
 * @return A bool indicating if it has been seen before
 */
bool is_duplicate(packet_header_t head);
#endif // !FUN_CACHE
