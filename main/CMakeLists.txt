idf_component_register(
                    SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_gpio esp_driver_spi esp_driver_twai esp_twai_mcp2515 esp_adc)
