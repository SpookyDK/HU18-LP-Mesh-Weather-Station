idf_component_register(SRCS "main.c" "web_server.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_wifi nvs_flash esp_http_server)
