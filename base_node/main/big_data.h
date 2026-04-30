#ifndef BIG_DATA
#define BIG_DATA

#include <stdint.h>

void start_receive_task();
void notify_big_data(uint32_t notif);

enum NOTIF_LORA {
    NOTIF_LORA_PAIRING = 36203,    // There is no reason behind this value
    NOTIF_LORA_DISCONNECT = 29629, // Nor this
};

#endif // !BIG_DATA
