#include "big_data.h"
#include "esp_err.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "fun_cache.h"
#include "http_parser.h"
#include "packet_def.h"
#include "portmacro.h"
#include "sd_card.h"
#include "site_content.h"
#include <limits.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

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

extern QueueHandle_t new_packet_queue;
static uint8_t ws_buf[WS_PUSH_CHUNK];
static void ws_push_task(void *arg) {
    ws_push_args_t *a = (ws_push_args_t *)arg;
    httpd_handle_t hd = a->hd;
    int fd = a->fd;
    size_t sd_tail_idx = 0;
    free(a);

    ESP_LOGI(TAG, "WS push task started, fd='%d'", fd);
    httpd_ws_frame_t ws_pkt = {0};
    struct tm timeinfo = {0};
    struct timespec ts = {0};

    while (1) {
        uint32_t notif;
        xTaskNotifyWait(0, ULONG_MAX, &notif, portMAX_DELAY);
        char msg[64] = {0};

        switch (notif & 0xffff) {
        case NOTIF_WS_EXIT:
            ESP_LOGI(TAG, "WS push: Exit requested, fd='%d'", fd);
            goto cleanup;
        case NOTIF_WS_NEW_CON: {
            clock_gettime(CLOCK_REALTIME, &ts);
            localtime_r(&ts.tv_sec, &timeinfo);

            size_t len = WS_PUSH_CHUNK;
            READ_RETURN_STATE read_ret;
            while ((read_ret = b_read_date(ws_buf, timeinfo, sd_tail_idx, &len)) != READ_FAILURE) {
                if (len == 0)
                    break;
                ws_pkt.type = HTTPD_WS_TYPE_BINARY;
                ws_pkt.len = len;
                ws_pkt.payload = ws_buf;
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
            break;
        }
        case NOTIF_WS_NEW_PACKET: {
            full_packet_time_t pkt = {0};
            if (xQueueReceive(new_packet_queue, &pkt, pdMS_TO_TICKS(15000)) != pdTRUE) {
                ESP_LOGW(TAG, "WS> Failed to get new packet from queue");
                break;
            }
            ws_pkt.type = HTTPD_WS_TYPE_BINARY;
            ws_pkt.len = sizeof(full_packet_time_t);
            ws_pkt.payload = (uint8_t *)&pkt;
            httpd_ws_send_frame_async(hd, fd, &ws_pkt);
            break;
        }
        case NOTIF_WS_PAIRING: {
            ws_pkt.type = HTTPD_WS_TYPE_TEXT;
            uint16_t nonce = (uint16_t)(notif >> 16);
            snprintf(msg, sizeof(msg), "PAIRING:%04X", nonce);
            ws_pkt.len = strlen(msg);
            ws_pkt.payload = (uint8_t *)msg;
            httpd_ws_send_frame_async(hd, fd, &ws_pkt);
            break;
        }
        case NOTIF_WS_PAIRING_ACK: {
            ws_pkt.type = HTTPD_WS_TYPE_TEXT;
            uint16_t nonce = (uint16_t)(notif >> 16);
            snprintf(msg, sizeof(msg), "PAIRING_ACK:%04X:%d", nonce, is_nonce_known(nonce));
            ws_pkt.len = strlen(msg);
            ws_pkt.payload = (uint8_t *)msg;
            httpd_ws_send_frame_async(hd, fd, &ws_pkt);
            break;
        }
        case NOTIF_WS_FLUSH:
            ESP_LOGW(TAG, "Not used");
            break;
        default:
            ESP_LOGW(TAG, "Unkown thing");
            break;
        }
    }
cleanup:
    ESP_LOGI(TAG, "WS push: Task Exiting, fd='%d'", fd);
    g_websocket_task = NULL;
    vTaskDelete(NULL);
}
void notify_websocket(uint32_t notif) {
    if (g_websocket_task != NULL) {
        xTaskNotify(g_websocket_task, notif, eSetBits);
    }
}
static void stop_push_task(void) {
    if (g_websocket_task == NULL)
        return;
    ESP_LOGI(TAG, "Requesting push task exit");
    xTaskNotify(g_websocket_task, NOTIF_WS_EXIT, eSetBits);
    for (int i = 0; i < 10; i++) {
        vTaskDelay(pdMS_TO_TICKS(50));
        if (g_websocket_task == NULL)
            return;
    }
    ESP_LOGW(TAG, "Push task did not take exit in due time");
    if (g_websocket_task != NULL) {
        vTaskDelete(g_websocket_task);
        g_websocket_task = NULL;
    }
}

static esp_err_t get_handler_dataviewer(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        if (g_websocket_task != NULL) {
            ESP_LOGI(TAG, "New WS handshake - Killing stale push task");
            stop_push_task();
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

    const char *start_sd = "START_SD_STREAM";
    const char *pairing = "PAIRING";
    const char *disconnect = "DISCONNECT";
    if (ws_pkt.len >= strlen(start_sd) && strncmp((char *)buf, start_sd, strlen(start_sd)) == 0) {
        ESP_LOGI(TAG, "Full query  '%s'", (char *)buf);
        ws_push_args_t *args = (ws_push_args_t *)calloc(1, sizeof(ws_push_args_t));
        if (sscanf((char *)buf, "START_SD_STREAM:%d-%d-%d:%d-%d-%d", &args->time_from.tm_year, &args->time_from.tm_mon,
                   &args->time_from.tm_mday, &args->time_to.tm_year, &args->time_to.tm_mon, &args->time_to.tm_mday) == 6) {
            args->time_from.tm_isdst = -1;
            args->time_from.tm_year -= 1900;
            args->time_from.tm_mon -= 1;
            args->time_to.tm_isdst = -1;
            args->time_to.tm_year -= 1900;
            args->time_to.tm_mon -= 1;
            mktime(&args->time_from);
            mktime(&args->time_to);
        } else {
            // TODO: Make the default values ie today
        }
        ESP_LOGI(TAG, "Starting SD stream");
        if (args) {
            args->hd = req->handle;
            args->fd = httpd_req_to_sockfd(req);
            if (xTaskCreate(ws_push_task, "ws_push", 4096, args, 5, &g_websocket_task) != pdPASS) {
                ESP_LOGE(TAG, "Failed to create WS push task");
                free(args);
                g_websocket_task = NULL;
            }
            notify_websocket(NOTIF_WS_NEW_CON);
        }
    } else if (ws_pkt.len >= strlen(pairing) && strncmp((char *)buf, pairing, strlen(pairing)) == 0) {
        ESP_LOGI(TAG, "WS Received a pairing response");
        const size_t offset = strlen(pairing) + 1;
        const char *accepted = "ACCEPTED";
        if (ws_pkt.len >= strlen(accepted) + offset && strncmp((char *)buf + offset, accepted, strlen(accepted)) == 0) {
            ESP_LOGI(TAG, "WS> Pairing Accepted sending down the line");
            const size_t ofset = offset + strlen(accepted) + 1;
            uint16_t val = strtoul((char *)&buf[ofset], NULL, 16);
            ESP_LOGI(TAG, "WS> Pairing had value='%04x'", val);
            notify_big_data(NOTIF_LORA_PAIRING | ((uint32_t)val << 16));
        } else {
            ESP_LOGI(TAG, "WS> Pairing denied");
        }
    } else if (ws_pkt.len >= strlen(disconnect) && strncmp((char *)buf, disconnect, strlen(disconnect)) == 0) {
        const size_t offset = strlen(disconnect) + 1;
        uint8_t node_id = strtoul((char *)&buf[offset], NULL, 10);
        ESP_LOGI(TAG, "WS> node id %d has been disconnected", node_id);
        notify_big_data(NOTIF_LORA_DISCONNECT | ((uint32_t)node_id << 16));
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
