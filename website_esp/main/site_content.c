#include "big_data.h"
#include "esp_err.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "http_parser.h"
#include "site_content.h"
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

extern int16_t big_int_data;
extern uint16_t big_uint_data;

static esp_err_t get_handler_root(httpd_req_t *req) {
    // clang-format off
    const char *resp_str =
        "<head> <style> p { white-space: pre-wrap; line-height: 1.05; } body { font-family: monospace; } </style> </head>"
        "<body>"
        "<h1>Esp32 Status page</h1>"
        "<h2>The current Value is <span id='val1'>0</span> random int</h2>"
        "<h2>The current Value is <span id='val2'>0</span> random uint</h2>"
        "<p>"
        COCIO_LOGO
        "</p>"
        "<script> setInterval(function() {"
        "fetch('/data').then((response) => response.json()) .then((data) => { Object.keys(data).forEach((key) => {"
        "const element = document.getElementById(key); if (element) { element.innerText = data[key]; } });"
        "}) .catch((error) => console.error('Update failed:', error)); },"
        "1000); </script>"
        "</body>";
    // clang-format on
    httpd_resp_set_type(req, "text/html; charset=utf-8");
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

static esp_err_t get_handler_data(httpd_req_t *req) {
    char val_str[64];
    sniprintf(val_str, sizeof(val_str), "{\"val1\":%d,\"val2\":%d}", big_int_data, big_uint_data);
    httpd_resp_set_type(req, "text/json");
    httpd_resp_send(req, val_str, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}
static httpd_uri_t uri_get_data = {.uri = "/data", .method = HTTP_GET, .handler = get_handler_data, .user_ctx = NULL};

httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        ESP_LOGI("webserver", "Registring uri");
        httpd_register_uri_handler(server, &uri_get_root);
        httpd_register_uri_handler(server, &uri_get_favicon);
        httpd_register_uri_handler(server, &uri_get_data);
        ESP_LOGI("webserver", "Registred all uri");
        return server;
    }
    return NULL;
}
