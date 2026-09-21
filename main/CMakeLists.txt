idf_component_register(
                    SRCS "main.c" "app_console.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_gpio esp_driver_spi esp_driver_twai mcp2518fd esp_adc console esp_timer)
