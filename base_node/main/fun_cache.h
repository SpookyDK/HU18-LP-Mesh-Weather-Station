#ifndef FUN_CACHE
#define FUN_CACHE

#include "packet_def.h"
#include <stdbool.h>
#include <stdint.h>

void init_node_map();
uint8_t get_free_node_id();
void set_node_id_state(uint8_t id, bool taken);

/*
 * Determine if a header has already been received
 * @return A bool indicating if it has been seen before
 */
bool is_duplicate(packet_header_t head);
#endif // !FUN_CACHE
