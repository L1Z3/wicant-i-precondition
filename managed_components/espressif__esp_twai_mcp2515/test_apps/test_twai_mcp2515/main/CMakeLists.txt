idf_component_register(
    SRCS "test_app_main.c" "test_twai_mcp2515.cpp"
    PRIV_REQUIRES unity esp_twai_mcp2515 esp_driver_gpio esp_driver_spi
    WHOLE_ARCHIVE
)
