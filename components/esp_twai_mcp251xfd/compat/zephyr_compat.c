#include <stdarg.h>
#include <stdio.h>
#include <zephyr/logging/log.h>

void zephyr_log(esp_log_level_t level, const char *tag, const char *format, ...)
{
	char message[160];
	va_list args;

	va_start(args, format);
	vsnprintf(message, sizeof(message), format, args);
	va_end(args);
	ESP_LOG_LEVEL(level, tag, "%s", message);
}
