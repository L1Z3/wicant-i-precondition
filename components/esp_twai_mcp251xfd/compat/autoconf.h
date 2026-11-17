#pragma once

// Kconfig values for the vendored Zephyr sources, which Zephyr normally
// generates as autoconf.h and preincludes. CMakeLists.txt does the same here.

// TX FIFO and TEF depth, i.e. frames in flight. Matches WiCAN's 32 TX slots.
#define CONFIG_CAN_MCP251XFD_MAX_TX_QUEUE          32
// RAM: TEF 32x8 + TX 32x16 + RX 32x16 = 1280 of 2048 bytes.
#define CONFIG_CAN_MCP251XFD_RX_FIFO_ITEMS         32
#define CONFIG_CAN_MCP251XFD_MAX_FILTERS           16
#define CONFIG_CAN_MCP251XFD_READ_CRC_RETRIES      5
// Bytes, as FreeRTOS counts them on ESP-IDF. WiCAN's callbacks run here.
#define CONFIG_CAN_MCP251XFD_INT_THREAD_STACK_SIZE 4096
// A FreeRTOS priority (see K_PRIO_COOP). Same as the MCP2515 worker: above
// lwIP, below Wi-Fi.
#define CONFIG_CAN_MCP251XFD_INT_THREAD_PRIO       20
// Deliver remote frames; WiCAN forwards them.
#define CONFIG_CAN_ACCEPT_RTR                      1
#define CONFIG_CAN_SAMPLE_POINT_MARGIN             50
// Unused: ESP-IDF filters logs per tag at runtime.
#define CONFIG_CAN_LOG_LEVEL                       3

// Shim settings, not Zephyr options. The interrupt thread is pinned like the
// MCP2515 worker, away from Wi-Fi/lwIP on core 0.
#define COMPAT_K_THREAD_NAME                       "mcp251xfd"
#define COMPAT_K_THREAD_CORE                       ((configNUMBER_OF_CORES > 1) ? 1 : tskNO_AFFINITY)
