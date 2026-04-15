#include "esp_err.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "http_parser.h"
#include "portmacro.h"
#include "sd_card.h"
#include "site_content.h"
#include <limits.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "webserver";
TaskHandle_t g_websocket_task = NULL;
static httpd_handle_t s_server = NULL;
static int s_ws_fd = 0;

static esp_err_t get_handler_favicon(httpd_req_t *req) {
    extern const uint8_t favicon_ico_start[] asm("_binary_favicon_ico_start");
    extern const uint8_t favicon_ico_end[] asm("_binary_favicon_ico_end");
    const size_t favicon_ico_size = (favicon_ico_end - favicon_ico_start);

    httpd_resp_set_type(req, "image/x-icon");
    httpd_resp_send(req, (const char *)favicon_ico_start, favicon_ico_size);
    return ESP_OK;
}
static httpd_uri_t uri_get_favicon = {.uri = "/favicon.ico", .method = HTTP_GET, .handler = get_handler_favicon, .user_ctx = NULL};

static esp_err_t get_handler_viewer(httpd_req_t *req) {
    extern const uint8_t viewer_html_gz_start[] asm("_binary_viewer_html_gz_start");
    extern const uint8_t viewer_html_gz_end[] asm("_binary_viewer_html_gz_end");
    const size_t viewer_html_gz_size = (viewer_html_gz_end - viewer_html_gz_start);
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    httpd_resp_send(req, (const char *)viewer_html_gz_start, viewer_html_gz_size);
    return ESP_OK;
}
static httpd_uri_t uri_get_viewer = {.uri = "/", .method = HTTP_GET, .handler = get_handler_viewer, .user_ctx = NULL};

static void ws_push_task(void *arg) {
    ws_push_args_t *a = (ws_push_args_t *)arg;
    httpd_handle_t hd = a->hd;
    int fd = a->fd;
    size_t sd_tail_idx = a->start_idx;
    free(a);

    uint8_t *buf = (uint8_t *)malloc(WS_PUSH_CHUNK);
    if (!buf) {
        ESP_LOGE(TAG, "Push task: OOM, exiting");
        g_websocket_task = NULL;
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "WS push task started, fd='%d' start_idx='%d'", fd, sd_tail_idx);
    httpd_ws_frame_t ws_pkt = {.type = HTTPD_WS_TYPE_BINARY, .payload = buf};

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        size_t len = WS_PUSH_CHUNK;
        READ_RETURN_STATE read_ret;
        while ((read_ret = b_read_file(PACKET_FILE, sd_tail_idx, &len, buf)) != READ_FAILURE) {
            if (len == 0)
                break;
            ws_pkt.type = HTTPD_WS_TYPE_BINARY;
            ws_pkt.len = len;
            ws_pkt.payload = buf;
            esp_err_t ret = httpd_ws_send_frame_async(hd, fd, &ws_pkt);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "WS push: send error %s, client gone", esp_err_to_name(ret));
                goto cleanup;
            }
            sd_tail_idx += len;
            len = WS_PUSH_CHUNK;
            if (read_ret == READ_DONE)
                break;
        }
        char end_msg[48];
        snprintf(end_msg, sizeof(end_msg), "END_OF_TRANSMISSION:%d", sd_tail_idx);
        httpd_ws_frame_t end_pkt = {.type = HTTPD_WS_TYPE_TEXT, .payload = (uint8_t *)end_msg, .len = strlen(end_msg)};
        httpd_ws_send_frame_async(hd, fd, &end_pkt);
    }
cleanup:
    ESP_LOGI(TAG, "WS push: Task Exiting, fd='%d'", fd);
    free(buf);
    g_websocket_task = NULL;
    vTaskDelete(NULL);
}
void notify_ws_new_sd_data(void) {
    if (g_websocket_task != NULL) {
        xTaskNotifyGive(g_websocket_task);
    }
}

static esp_err_t get_handler_dataviewer(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        if (g_websocket_task != NULL) {
            ESP_LOGI(TAG, "New WS handshake - Killing stale push task");
            vTaskDelete(g_websocket_task);
            g_websocket_task = NULL;
        }
        s_ws_fd = httpd_req_to_sockfd(req);
        ESP_LOGI(TAG, "Handshake done, client connected fd'%d'", s_ws_fd);
        return ESP_OK;
    }
    httpd_ws_frame_t ws_pkt = {0};
    uint8_t *buf = NULL;
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to get ws packet len");
        return ret;
    }
    if (ws_pkt.type == HTTPD_WS_TYPE_PONG || ws_pkt.type == HTTPD_WS_TYPE_CLOSE) {
        free(buf);
        return ESP_OK;
    }
    if (ws_pkt.len > 0) {
        buf = (uint8_t *)calloc(1, ws_pkt.len + 1);
        if (!buf)
            return ESP_ERR_NO_MEM;
        ws_pkt.payload = buf;
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
    }
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to receive WS frame: %s", esp_err_to_name(ret));
        free(buf);
        return ret;
    }
    if (ws_pkt.type != HTTPD_WS_TYPE_TEXT) {
        ESP_LOGD(TAG, "Ignoring non-text WS frame type=%d", ws_pkt.type);
        free(buf);
        return ESP_OK;
    }

    const char *target = "START_SD_STREAM";
    if (ws_pkt.len >= strlen(target) && strncmp((char *)buf, target, strlen(target)) == 0) {
        size_t resume_idx = 0;
        if (ws_pkt.len > strlen(target) && buf[strlen(target)] == ':') {
            resume_idx = (size_t)atoi((char *)&buf[strlen(target) + 1]);
            ESP_LOGI(TAG, "Resuming SD stream from byte %d", resume_idx);
        } else {
            ESP_LOGI(TAG, "Starting fresh SD stream");
        }
        ws_push_args_t *args = (ws_push_args_t *)malloc(sizeof(ws_push_args_t));
        if (args) {
            args->hd = req->handle;
            args->fd = httpd_req_to_sockfd(req);
            args->start_idx = resume_idx;
            if (xTaskCreate(ws_push_task, "ws_push", 4096, args, 5, &g_websocket_task) != pdPASS) {
                ESP_LOGE(TAG, "Failed to create WS push task");
                free(args);
                g_websocket_task = NULL;
            }
            notify_ws_new_sd_data();
        }
    } else {
        ESP_LOGI(TAG, "WS Received: %s", (char *)buf);
    }
    free(buf);
    return ESP_OK;
}
static httpd_uri_t uri_get_dataviewer = {
    .uri = "/data", .method = HTTP_GET, .handler = get_handler_dataviewer, .user_ctx = NULL, .is_websocket = true};

httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.keep_alive_enable = true;
    config.keep_alive_interval = 10;

    if (httpd_start(&s_server, &config) == ESP_OK) {
        ESP_LOGI("webserver", "Registring uri");
        httpd_register_uri_handler(s_server, &uri_get_favicon);
        httpd_register_uri_handler(s_server, &uri_get_viewer);
        httpd_register_uri_handler(s_server, &uri_get_dataviewer);
        ESP_LOGI("webserver", "Registred all uri");
        return s_server;
    }
    ESP_LOGE("webserver", "Failed to start server");
    return NULL;
}
