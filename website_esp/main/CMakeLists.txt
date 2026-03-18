idf_component_register(SRCS "main.c" "site_content.c" "web_server.c" "big_data.c" "sd_card.c" "../../monolithic_esp/main/my_drivers/lora.c"
                    INCLUDE_DIRS "." "../../monolithic_esp/main/packet_def" "../../monolithic_esp/main/my_drivers/"
                    EMBED_FILES "content/favicon.ico" "content/code.js" "content/index.html"
                    REQUIRES esp_wifi nvs_flash esp_http_server fatfs)
