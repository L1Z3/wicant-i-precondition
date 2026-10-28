# MCP2518FD TWAI node on Zephyr's MCP251xFD driver; see README.md
idf_component_register(
    SRCS "esp_twai_mcp251xfd.c"
         "zephyr/drivers/can/can_common.c"
         "zephyr/subsys/crc/crc16_sw.c"
         "compat/zephyr_compat.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "compat" "zephyr/include"
    REQUIRES esp_driver_twai esp_driver_spi
    PRIV_REQUIRES esp_driver_gpio esp_rom log
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    # Zephyr sources expect their Kconfig values preincluded, as autoconf.h.
    "SHELL:-include ${CMAKE_CURRENT_LIST_DIR}/compat/autoconf.h"
    # Zephyr's can.h declares syscalls for features this build currently 
    # leaves out (CAN FD, stats, manual recovery),
    -Wno-unused-function
    # The driver casts its thread function to Zephyr's three-argument entry type
    -Wno-cast-function-type)
