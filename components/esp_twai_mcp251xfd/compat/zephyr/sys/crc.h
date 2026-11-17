#pragma once

#include <zephyr/types.h>

// Implemented by the vendored subsys/crc/crc16_sw.c.
uint16_t crc16(uint16_t poly, uint16_t seed, const uint8_t *src, size_t len);
uint16_t crc16_reflect(uint16_t poly, uint16_t seed, const uint8_t *src, size_t len);
uint16_t crc16_ccitt(uint16_t seed, const uint8_t *src, size_t len);
uint16_t crc16_itu_t(uint16_t seed, const uint8_t *src, size_t len);
