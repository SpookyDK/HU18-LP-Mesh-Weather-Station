set(EXTERNAL_LIBS "../../monolithic_esp/main/my_drivers")

idf_component_register(SRCS "main.c" "site_content.c" "web_server.c" "big_data.c" "sd_card.c" "${EXTERNAL_LIBS}/lora.c" "${EXTERNAL_LIBS}/NEO_6M_UART.c"
  INCLUDE_DIRS "." "${EXTERNAL_LIBS}/../packet_def" "${EXTERNAL_LIBS}/"
                    EMBED_FILES "content/favicon.ico"
                    REQUIRES esp_wifi nvs_flash esp_http_server fatfs esp_timer driver)

set(CONTENT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/content)
set(GZ_DIR ${CMAKE_CURRENT_BINARY_DIR}/gz)

file(MAKE_DIRECTORY ${GZ_DIR})

set(GZ_FILES "")
foreach(SRC_FILE IN ITEMS viewer.html index.html code.js)
  set(GZ_OUT ${GZ_DIR}/${SRC_FILE}.gz)
  add_custom_command(
        OUTPUT ${GZ_OUT}
        COMMAND gzip -9 -c ${CONTENT_DIR}/${SRC_FILE} > ${GZ_OUT}
        DEPENDS ${CONTENT_DIR}/${SRC_FILE}
        VERBATIM
    )
  list(APPEND GZ_FILES ${GZ_OUT})
  target_add_binary_data(${COMPONENT_LIB} ${GZ_OUT} BINARY)
endforeach()


add_custom_target(generate_gz ALL DEPENDS ${GZ_FILES})
add_dependencies(${COMPONENT_LIB} generate_gz)

#[[ foreach(GZ_FILE IN LISTS GZ_FILES)
  target_add_binary_data($COMPONENT_LIB ${GZ_OUT} BINARY) ]]
