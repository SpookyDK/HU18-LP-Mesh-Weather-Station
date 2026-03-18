#include "big_data.h"
#include "esp_err.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "http_parser.h"
#include "packet_def.h"
#include "site_content.h"
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

static esp_err_t get_handler_root(httpd_req_t *req) {
    extern const uint8_t index_html_start[] asm("_binary_index_html_start");
    extern const uint8_t index_html_end[] asm("_binary_index_html_end");
    const size_t index_html_size = (index_html_end - index_html_start);
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, (const char *)index_html_start, index_html_size);
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
static httpd_uri_t uri_get_favicon = {.uri = "/favicon.ico", .method = HTTP_GET, .handler = get_handler_favicon, .user_ctx = NULL};

extern full_packet_t big_data_packet;
static esp_err_t get_handler_data(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_send(req, (const char *)&big_data_packet, sizeof(full_packet_t));
    return ESP_OK;
}
static httpd_uri_t uri_get_data = {.uri = "/data", .method = HTTP_GET, .handler = get_handler_data, .user_ctx = NULL};

static esp_err_t get_handler_code(httpd_req_t *req) {
    extern const uint8_t code_js_start[] asm("_binary_code_js_start");
    extern const uint8_t code_js_end[] asm("_binary_code_js_end");
    const size_t code_js_size = (code_js_end - code_js_start);
    httpd_resp_set_type(req, "text/jscript");
    httpd_resp_send(req, (const char *)code_js_start, code_js_size);
    return ESP_OK;
}
static httpd_uri_t uri_get_code = {.uri = "/code.js", .method = HTTP_GET, .handler = get_handler_code, .user_ctx = NULL};

httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        ESP_LOGI("webserver", "Registring uri");
        httpd_register_uri_handler(server, &uri_get_root);
        httpd_register_uri_handler(server, &uri_get_favicon);
        httpd_register_uri_handler(server, &uri_get_data);
        httpd_register_uri_handler(server, &uri_get_code);
        ESP_LOGI("webserver", "Registred all uri");
        return server;
    }
    return NULL;
}
