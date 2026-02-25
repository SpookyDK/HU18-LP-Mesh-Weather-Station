idf_component_register(
    SRCS "lora.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_driver_spi
    )