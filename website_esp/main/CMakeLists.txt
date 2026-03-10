idf_component_register(SRCS "main.c" "site_content.c" "web_server.c" "big_data.c" "sd_card.c"
                    INCLUDE_DIRS "."
                    EMBED_FILES "content/favicon.ico"
                    REQUIRES esp_wifi nvs_flash esp_http_server fatfs)
