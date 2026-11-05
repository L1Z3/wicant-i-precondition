idf_component_register(SRCS "src/log_stream.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES heap esp_system esp_app_format log freertos)
