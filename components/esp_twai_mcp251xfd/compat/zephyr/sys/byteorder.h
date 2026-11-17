#pragma once

#include <zephyr/toolchain.h>

BUILD_ASSERT(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "ESP32 targets are little-endian");

#define sys_le16_to_cpu(val) (val)
#define sys_cpu_to_le16(val) (val)
#define sys_le32_to_cpu(val) (val)
#define sys_cpu_to_le32(val) (val)
#define sys_cpu_to_be16(val) __builtin_bswap16(val)
