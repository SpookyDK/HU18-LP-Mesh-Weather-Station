#ifndef SITE_CONTENT
#define SITE_CONTENT

#include "esp_http_server.h"
#include "freertos/idf_additions.h"
#include <stddef.h>
#include <stdint.h>

typedef struct {
    httpd_handle_t hd;
    int fd;
} ws_push_args_t;

/*
 * @note This is limited to just the first 16 bits, as the latter is used for data passing
 */
enum notif_ws {
    NOTIF_WS_NEW_DATA = 1,
    NOTIF_WS_FLUSH,
    NOTIF_WS_EXIT,
    NOTIF_WS_PAIRING,
    NOTIF_WS_PAIRING_ACK,
};

#define WS_PUSH_CHUNK 512

extern TaskHandle_t g_websocket_task;

httpd_handle_t start_webserver(void);

void notify_websocket(uint32_t notif);

#endif // !SITE_CONTENT
