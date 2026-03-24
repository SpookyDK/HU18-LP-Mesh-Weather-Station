#include "esp_err.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "http_parser.h"
#include "packet_def.h"
#include "sd_card.h"
#include "site_content.h"
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "webserver";

static esp_err_t get_handler_root(httpd_req_t *req) {
    extern const uint8_t index_html_start[] asm("_binary_index_html_gz_start");
    extern const uint8_t index_html_end[] asm("_binary_index_html_gz_end");
    const size_t index_html_size = (index_html_end - index_html_start);
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
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
    extern const uint8_t code_js_start[] asm("_binary_code_js_gz_start");
    extern const uint8_t code_js_end[] asm("_binary_code_js_gz_end");
    const size_t code_js_size = (code_js_end - code_js_start);
    httpd_resp_set_type(req, "text/jscript");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    httpd_resp_send(req, (const char *)code_js_start, code_js_size);
    return ESP_OK;
}
static httpd_uri_t uri_get_code = {.uri = "/code.js", .method = HTTP_GET, .handler = get_handler_code, .user_ctx = NULL};

static esp_err_t get_handler_viewer(httpd_req_t *req) {
    extern const uint8_t viewer_html_gz_start[] asm("_binary_viewer_html_gz_start");
    extern const uint8_t viewer_html_gz_end[] asm("_binary_viewer_html_gz_end");
    const size_t viewer_html_gz_size = (viewer_html_gz_end - viewer_html_gz_start);
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    httpd_resp_send(req, (const char *)viewer_html_gz_start, viewer_html_gz_size);
    return ESP_OK;
}
static httpd_uri_t uri_get_viewer = {.uri = "/viewer", .method = HTTP_GET, .handler = get_handler_viewer, .user_ctx = NULL};

static void send_ws_data(httpd_req_t *req, httpd_ws_frame_t ws_pkt, size_t start_idx) {
    ESP_LOGI(TAG, "Starting Transmission, '%d'", start_idx);
    ws_pkt.type = HTTPD_WS_TYPE_BINARY;
    esp_err_t ret;
    const size_t max_chunk = 512;
    uint8_t *buf = (uint8_t *)malloc(sizeof(uint8_t) * max_chunk);
    // It is set to max, and is later set by the b_read_file
    size_t len = max_chunk;
    READ_RETURN_STATE read_ret;
    while ((read_ret = b_read_file(PACKET_FILE, start_idx, &len, buf)) == READ_NOT_DONE) {
        if (len == 0) {
            break;
        }
        ws_pkt.len = len;
        ws_pkt.payload = buf;
        ESP_LOGD(TAG, "Found %d bytes to send", ws_pkt.len);
        ret = httpd_ws_send_frame(req, &ws_pkt);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "WS stream error: %s", esp_err_to_name(ret));
            break;
        }
        start_idx += len;
        len = max_chunk;
    }
    ESP_LOGI(TAG, "Sending end of Transmission flag");
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    snprintf((char *)buf, max_chunk, "END_OF_TRANSMISSION:%d", start_idx);
    ws_pkt.payload = buf;
    ws_pkt.len = strlen((char *)buf);
    httpd_ws_send_frame(req, &ws_pkt);
    free(buf);
}
static esp_err_t get_handler_dataviewer(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "Handshake done, client connected");
        return ESP_OK;
    }
    httpd_ws_frame_t ws_pkt = {0};
    uint8_t *buf = NULL;
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to get ws packet len");
        return ret;
    }

    if (ws_pkt.len > 0) {
        buf = (uint8_t *)calloc(1, ws_pkt.len + 1);
        ws_pkt.payload = buf;
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
    }

    if (ret == ESP_OK && ws_pkt.type == HTTPD_WS_TYPE_TEXT) {
        const char *target = "START_SD_STREAM";
        if (ws_pkt.len == strlen(target) && strcmp((char *)buf, target) == 0) { // Completely new
            send_ws_data(req, ws_pkt, 0);
        } else if (strncmp((char *)buf, target, strlen(target)) == 0) { // Continue of this point
            ESP_LOGI(TAG, "Continuing with: '%s'", (char *)buf);
            send_ws_data(req, ws_pkt, atoi((char *)&buf[strlen(target) + 1]));
        } else {
            ESP_LOGI(TAG, "Received: %s", (char *)buf);
        }
    }
    free(buf);
    return ret;
}
static httpd_uri_t uri_get_dataviewer = {
    .uri = "/dataviewer", .method = HTTP_GET, .handler = get_handler_dataviewer, .user_ctx = NULL, .is_websocket = true};

httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        ESP_LOGI("webserver", "Registring uri");
        httpd_register_uri_handler(server, &uri_get_root);
        httpd_register_uri_handler(server, &uri_get_favicon);
        httpd_register_uri_handler(server, &uri_get_data);
        httpd_register_uri_handler(server, &uri_get_code);
        httpd_register_uri_handler(server, &uri_get_viewer);
        httpd_register_uri_handler(server, &uri_get_dataviewer);
        ESP_LOGI("webserver", "Registred all uri");
        return server;
    }
    ESP_LOGE("webserver", "Failed to start server");
    return NULL;
}
