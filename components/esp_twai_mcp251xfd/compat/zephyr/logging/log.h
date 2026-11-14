#pragma once

#include "esp_log.h"

// Zephyr logging on ESP-IDF, tagged with the module name. The driver logs each
// frame at debug level, so debug messages are compiled out. Zephyr sources
// print uint32_t with %d/%x (uint32_t is long on ESP-IDF), so formats are not
// type-checked here; both are 32 bits wide.
void zephyr_log(esp_log_level_t level, const char *tag, const char *format, ...);

#define LOG_MODULE_REGISTER(name, ...) \
	static const char *const z_log_tag __attribute__((unused)) = #name
#define LOG_ERR(...) zephyr_log(ESP_LOG_ERROR, z_log_tag, __VA_ARGS__)
#define LOG_WRN(...) zephyr_log(ESP_LOG_WARN, z_log_tag, __VA_ARGS__)
#define LOG_INF(...) zephyr_log(ESP_LOG_INFO, z_log_tag, __VA_ARGS__)
#define LOG_DBG(...)                                                           \
	do {                                                                   \
		if (0) {                                                       \
			zephyr_log(ESP_LOG_DEBUG, z_log_tag, __VA_ARGS__);     \
		}                                                              \
	} while (0)
