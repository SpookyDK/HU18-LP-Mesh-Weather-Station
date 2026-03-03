# Register the component
idf_component_register(
    SRCS i2cdev.c
    INCLUDE_DIRS include
    REQUIRES esp-idf-lib__esp_idf_lib_helpers esp_driver_gpio esp_driver_i2c
)
