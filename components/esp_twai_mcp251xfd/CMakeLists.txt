idf_component_register(
    SRCS "esp_twai_mcp251xfd.c" "mcp251xfd_core.c" "vendor/MCP251XFD.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "." "vendor"
    REQUIRES esp_driver_twai esp_driver_spi
    PRIV_REQUIRES esp_driver_gpio esp_timer
)
