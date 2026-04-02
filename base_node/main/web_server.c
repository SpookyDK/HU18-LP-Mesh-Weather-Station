#include "esp_err.h"
#include "esp_event.h"
#include "esp_event_base.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_netif_ip_addr.h"
#include "esp_netif_types.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "esp_wifi_types_generic.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "hotspot_codes.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "portmacro.h"
#include "site_content.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static EventGroupHandle_t s_wifi_event_group;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

httpd_handle_t start_webserver(void);

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {}

void wifi_init_softap(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .ap =
            {
                .ssid = NOT_HOTSPOT_SSID,
                .ssid_len = strlen(NOT_HOTSPOT_SSID),
                .channel = NOT_HOTSPOT_CHANNEL,
                .password = NOT_HOTSPOT_PASS,
                .max_connection = 4,
                .authmode = WIFI_AUTH_WPA2_PSK,
                .pmf_cfg = {.required = true},
                .gtk_rekey_interval = 600,
            },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI("wifi", "wifi_init_softap finished. SSID:%s  password:%s channel:%d", NOT_HOTSPOT_SSID, NOT_HOTSPOT_PASS, NOT_HOTSPOT_CHANNEL);
    start_webserver();
}
