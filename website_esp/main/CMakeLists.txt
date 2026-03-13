idf_component_register(SRCS "main.c" "site_content.c" "web_server.c" "big_data.c" "sd_card.c" "lora.c"
                    INCLUDE_DIRS "."
                    EMBED_FILES "content/favicon.ico" "content/code.js"
                    REQUIRES esp_wifi nvs_flash esp_http_server fatfs)
