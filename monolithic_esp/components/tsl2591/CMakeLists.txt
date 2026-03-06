idf_component_register(
    SRCS tsl2591.c
    INCLUDE_DIRS .
    REQUIRES i2cdev log freertos
)
