set(EXTERNAL_LIBS "../../weather_node/main/my_drivers")

idf_component_register(SRCS "main.c" "site_content.c" "web_server.c" "big_data.c" "sd_card.c" "${EXTERNAL_LIBS}/lora.c" "${EXTERNAL_LIBS}/NEO_6M_UART.c"
  INCLUDE_DIRS "." "${EXTERNAL_LIBS}/../packet_def" "${EXTERNAL_LIBS}/"
                    EMBED_FILES "content/favicon.ico"
                    REQUIRES esp_wifi nvs_flash esp_http_server fatfs esp_timer driver)

include(CheckTypeSize)
set(CMAKE_EXTRA_INCLUDE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/${EXTERNAL_LIBS}/../packet_def/packet_def.h")
check_type_size("full_packet_t" FULL_PACKET)
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/content/viewer.html.in" "${CMAKE_CURRENT_BINARY_DIR}/viewer.html")

set(CONTENT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/content")
set(GZ_DIR "${CMAKE_CURRENT_BINARY_DIR}/gz")

file(MAKE_DIRECTORY ${GZ_DIR})

set(GZ_FILES "")
foreach(SRC_NAME IN ITEMS viewer.html index.html)
  if(EXISTS "${CMAKE_CURRENT_BINARY_DIR}/${SRC_NAME}")
    set(SRC_PATH "${CMAKE_CURRENT_BINARY_DIR}/${SRC_NAME}")
  else()
    set(SRC_PATH "${CMAKE_CURRENT_SOURCE_DIR}/content/${SRC_NAME}")
  endif()

  set(GZ_OUT "${GZ_DIR}/${SRC_NAME}.gz")
  add_custom_command(
        OUTPUT ${GZ_OUT}
        COMMAND gzip -9 -c ${SRC_PATH} > ${GZ_OUT}
        DEPENDS ${SRC_PATH}
        VERBATIM
    )
  list(APPEND GZ_FILES ${GZ_OUT})
  target_add_binary_data(${COMPONENT_LIB} ${GZ_OUT} BINARY)
endforeach()
if(ENABLE_ANALYZER)
target_compile_options(${COMPONENT_LIB} PRIVATE 
    "-fanalyzer"           # The logic/path analyzer
    # "-Wanalyzer-too-complex" # Warns if the analyzer gives up on a complex function
    "-Wall"                # Standard good warnings
    "-Wextra"              # Extra "suspicious" code warnings
    "-Wshadow"             # Warns if you name a local variable the same as a global
    "-Wdouble-promotion"   # CRITICAL for ESP32: warns if you accidentally use double math on a float CPU
    "-Wnull-dereference"   # Additional check for null pointers outside the analyzer
    "-Wstack-usage=2048"   # Warns if a function uses more than 2KB of stack (prevents stack overflows)
)
endif()
add_custom_target(generate_gz ALL DEPENDS ${GZ_FILES})
add_dependencies(${COMPONENT_LIB} generate_gz)
