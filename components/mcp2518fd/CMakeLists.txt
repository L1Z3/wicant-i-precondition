idf_component_register(
    SRCS "mcp2518fd.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio esp_driver_spi
)