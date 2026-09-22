#pragma once

#include <zephyr/types.h>

// Definitions as in Zephyr v4.4.2 include/zephyr/sys/util.h and util_macro.h.
// BIT, MIN and MAX may already come from ESP-IDF (esp_bit_defs.h) or newlib
// (sys/param.h); MIN and MAX match newlib token for token.
#ifndef BIT
#define BIT(n) (1UL << (n))
#endif
#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))
#endif

#define BITS_PER_LONG (__CHAR_BIT__ * __SIZEOF_LONG__)
#define GENMASK(h, l) (((~0UL) - (1UL << (l)) + 1) & (~0UL >> (BITS_PER_LONG - 1 - (h))))
#define LSB_GET(value) ((value) & -(value))
#define FIELD_GET(mask, value)  (((value) & (mask)) / LSB_GET(mask))
#define FIELD_PREP(mask, value) (((value) * LSB_GET(mask)) & (mask))

#define ROUND_UP(x, align)                                                     \
	((((unsigned long)(x) + ((unsigned long)(align) - 1)) / (unsigned long)(align)) * \
	 (unsigned long)(align))
#define ROUND_DOWN(x, align) (((unsigned long)(x) / (unsigned long)(align)) * (unsigned long)(align))
#define DIV_ROUND_UP(n, d) (((n) + (d) - 1) / (d))

#define __z_log2d(x) (32 - __builtin_clz(x) - 1)
#define __z_log2q(x) (64 - __builtin_clzll(x) - 1)
#define __z_log2(x)  (sizeof(__typeof__(x)) > 4 ? __z_log2q(x) : __z_log2d(x))
#define LOG2(x) ((x) < 1 ? -1 : __z_log2(x))

#define CLAMP(val, low, high) (((val) <= (low)) ? (low) : MIN(val, high))
#define IN_RANGE(val, min, max) ((val) >= (min) && (val) <= (max))
#define CONTAINER_OF(ptr, type, field) ((type *)(((char *)(ptr)) - offsetof(type, field)))
