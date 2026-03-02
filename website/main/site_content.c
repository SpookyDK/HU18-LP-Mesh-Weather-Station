#include "esp_err.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "site_content.h"
#include <stddef.h>
#include <stdint.h>

static esp_err_t get_handler_root(httpd_req_t *req) {
    const char *resp_str = "<h1>Hello from ESP32!</h1><p>Sent from your mom</p>";
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}
static httpd_uri_t uri_get_root = {.uri = "/", .method = HTTP_GET, .handler = get_handler_root, .user_ctx = NULL};

static esp_err_t get_handler_favicon(httpd_req_t *req) {
    extern const uint8_t favicon_ico_start[] asm("_binary_favicon_ico_start");
    extern const uint8_t favicon_ico_end[] asm("_binary_favicon_ico_end");
    const size_t favicon_ico_size = (favicon_ico_end - favicon_ico_start);

    httpd_resp_set_type(req, "image/x-icon");
    httpd_resp_send(req, (const char *)favicon_ico_start, favicon_ico_size);
    return ESP_OK;
}
static httpd_uri_t uri_get_favicon = {
    .uri = "/favicon.ico", .method = HTTP_GET, .handler = get_handler_favicon, .user_ctx = NULL};

httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        ESP_LOGI("webserver", "Registring uri");
        httpd_register_uri_handler(server, &uri_get_root);
        httpd_register_uri_handler(server, &uri_get_favicon);
        ESP_LOGI("webserver", "Registred all uri");
        return server;
    }
    return NULL;
}
