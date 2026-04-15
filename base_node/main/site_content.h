#ifndef SITE_CONTENT
#define SITE_CONTENT

#include "esp_http_server.h"
#include "freertos/idf_additions.h"
#include <stddef.h>
#include <stdint.h>

typedef struct {
    httpd_handle_t hd;
    int fd;
    size_t start_idx;
} ws_push_args_t;

#define NOTIFY_NEW_DATA BIT(0)
#define NOTIFY_FLUSH_NOW BIT(1)
#define NOTIFY_EXIT BIT(2)

#define WS_PUSH_CHUNK 512

extern TaskHandle_t g_websocket_task;

httpd_handle_t start_webserver(void);

void notify_ws_new_sd_data(void);

#endif // !SITE_CONTENT
