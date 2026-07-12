idf_component_register(
    SRCS "src/file_logs.c"
    INCLUDE_DIRS "include"
    REQUIRES freertos log main
    PRIV_REQUIRES esp_system heap vfs
)
