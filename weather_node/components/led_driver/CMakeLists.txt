
idf_component_register(SRCS "led_driver.c"
                       INCLUDE_DIRS "."
                      REQUIRES esp_driver_gpio esp_driver_rmt)
