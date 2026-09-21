#include <ctype.h>
#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app.h"
#include "esp_console.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/task.h"
#include "hw_config.h"

#define USAGE_CAN_SET   "can_set [--nominal hz] [--data hz] [--fd 0|1] [--brs 0|1] [--mode fd|classic|listen|iloop|eloop]"
#define USAGE_CAN_SEND  "can_send <ext|int> <hex_id> [hexdata] [--ext] [--fd] [--brs]"

static bool parse_u32(const char *text, uint32_t min_value, uint32_t max_value, uint32_t *out_value)
{
	if (text == NULL || out_value == NULL) {
		return false;
	}
	errno = 0;
	char *end = NULL;
	unsigned long long parsed = strtoull(text, &end, 0);
	if (errno != 0 || end == text || *end != '\0' || parsed < min_value || parsed > max_value) {
		return false;
	}
	*out_value = (uint32_t)parsed;
	return true;
}

static bool parse_hex_id(const char *text, uint32_t max_value, uint32_t *out_value)
{
	errno = 0;
	char *end = NULL;
	unsigned long v = strtoul(text, &end, 16);   /* CAN IDs are always hex */
	if (errno != 0 || end == text || *end != 0 || v > max_value) {
		return false;
	}
	*out_value = (uint32_t)v;
	return true;
}

static bool parse_bool(const char *text, bool *out)
{
	if (strcmp(text, "1") == 0 || strcasecmp(text, "on") == 0 || strcasecmp(text, "true") == 0) {
		*out = true;
		return true;
	}
	if (strcmp(text, "0") == 0 || strcasecmp(text, "off") == 0 || strcasecmp(text, "false") == 0) {
		*out = false;
		return true;
	}
	return false;
}

static bool parse_hex_payload(const char *text, uint8_t *out, size_t max_len, size_t *out_len)
{
	size_t len = 0;
	int nibble = -1;
	for (const char *p = text; *p; p++) {
		if (*p == ':' || *p == ' ' || *p == ',') {
			continue;
		}
		if (!isxdigit((unsigned char)*p)) {
			return false;
		}
		int v = isdigit((unsigned char)*p) ? (*p - '0') : (tolower((unsigned char)*p) - 'a' + 10);
		if (nibble < 0) {
			nibble = v;
		} else {
			if (len >= max_len) {
				return false;
			}
			out[len++] = (uint8_t)((nibble << 4) | v);
			nibble = -1;
		}
	}
	if (nibble >= 0) {
		return false;
	}
	*out_len = len;
	return true;
}

static int cmd_status(int argc, char **argv)
{
	(void)argc;
	(void)argv;
	app_print_status();
	return 0;
}

static int cmd_can_set(int argc, char **argv)
{
	if (argc == 1) {
		app_print_status();
		return 0;
	}

	mcp2518fd_config_t cfg = s_app.external.mcp_cfg;
	for (int i = 1; i < argc; i++) {
		const char *a = argv[i];
		if (strcmp(a, "--nominal") == 0 && i + 1 < argc) {
			if (!parse_u32(argv[++i], 10000U, 1000000U, &cfg.nominal_bitrate)) {
				printf("usage: %s\n", USAGE_CAN_SET);
				return 1;
			}
		} else if (strcmp(a, "--data") == 0 && i + 1 < argc) {
			if (!parse_u32(argv[++i], 10000U, 8000000U, &cfg.data_bitrate)) {
				printf("usage: %s\n", USAGE_CAN_SET);
				return 1;
			}
		} else if (strcmp(a, "--fd") == 0 && i + 1 < argc) {
			if (!parse_bool(argv[++i], &cfg.enable_can_fd)) {
				printf("usage: %s\n", USAGE_CAN_SET);
				return 1;
			}
			cfg.mode = cfg.enable_can_fd ? MCP2518FD_MODE_NORMAL_FD : MCP2518FD_MODE_NORMAL_CLASSIC;
		} else if (strcmp(a, "--brs") == 0 && i + 1 < argc) {
			if (!parse_bool(argv[++i], &cfg.enable_brs)) {
				printf("usage: %s\n", USAGE_CAN_SET);
				return 1;
			}
		} else if (strcmp(a, "--mode") == 0 && i + 1 < argc) {
			if (!app_parse_can_mode(argv[++i], &cfg.mode)) {
				printf("usage: %s\n", USAGE_CAN_SET);
				return 1;
			}
		} else {
			printf("usage: %s\n", USAGE_CAN_SET);
			return 1;
		}
	}
	if (!cfg.enable_can_fd && cfg.enable_brs) {
		printf("BRS requires FD (--fd 1)\n");
		return 1;
	}

	esp_err_t err = app_external_reconfigure(&cfg);
	if (err != ESP_OK) {
		printf("external reconfigure failed: %s\n", esp_err_to_name(err));
		return 1;
	}
	app_print_status();
	return 0;
}

static int cmd_can_set_int(int argc, char **argv)
{
	uint32_t bitrate = 0;
	if (argc != 2 || !parse_u32(argv[1], 10000U, 1000000U, &bitrate)) {
		printf("usage: can_set_int <bitrate_hz>\n");
		return 1;
	}
	esp_err_t err = app_internal_reconfigure(bitrate);
	if (err != ESP_OK) {
		printf("internal reconfigure failed: %s\n", esp_err_to_name(err));
		return 1;
	}
	printf("internal bitrate = %" PRIu32 "\n", bitrate);
	return 0;
}

static int cmd_can_send(int argc, char **argv)
{
	if (argc < 3) {
		printf("usage: %s\n", USAGE_CAN_SEND);
		return 1;
	}
	bool external;
	if (strcmp(argv[1], "ext") == 0) {
		external = true;
	} else if (strcmp(argv[1], "int") == 0) {
		external = false;
	} else {
		printf("usage: %s\n", USAGE_CAN_SEND);
		return 1;
	}

	bool extended = false, fd = false, brs = false;
	const char *data_text = NULL;
	for (int i = 3; i < argc; i++) {
		if (strcmp(argv[i], "--ext") == 0) {
			extended = true;
		} else if (strcmp(argv[i], "--fd") == 0) {
			fd = true;
		} else if (strcmp(argv[i], "--brs") == 0) {
			brs = true;
		} else if (data_text == NULL) {
			data_text = argv[i];
		} else {
			printf("usage: %s\n", USAGE_CAN_SEND);
			return 1;
		}
	}

	uint32_t id = 0;
	if (!parse_hex_id(argv[2], extended ? 0x1FFFFFFFU : 0x7FFU, &id)) {
		printf("bad id\n");
		return 1;
	}

	uint8_t data[MCP2518FD_MAX_DATA_LEN] = {0};
	size_t len = 0;
	if (data_text && !parse_hex_payload(data_text, data, sizeof(data), &len)) {
		printf("bad payload (hex bytes, e.g. 0102AABB or 01:02:AA:BB, max 64)\n");
		return 1;
	}
	if (!fd && len > 8) {
		printf("classic frames carry at most 8 bytes (use --fd)\n");
		return 1;
	}
	if (!external && (fd || len > 8)) {
		printf("internal channel is classic CAN only\n");
		return 1;
	}

	esp_err_t err = external ? app_send_external(id, extended, fd, brs, data, len)
							 : app_send_internal(id, extended, data, len);
	if (err != ESP_OK) {
		printf("tx failed: %s\n", esp_err_to_name(err));
		return 1;
	}
	printf("%s tx id=0x%" PRIX32 " len=%u%s%s%s\n", external ? "external" : "internal", id, (unsigned)len,
		   extended ? " ext" : "", fd ? " fd" : "", brs ? " brs" : "");
	return 0;
}

static int cmd_test(int argc, char **argv)
{
	bool on;
	if (argc != 2 || !parse_bool(argv[1], &on)) {
		printf("usage: test on|off   (periodic 0x611/0x612 test frames + 0x7DF OBD request)\n");
		return 1;
	}
	s_app.test_traffic = on;
	printf("test traffic %s\n", on ? "on" : "off");
	return 0;
}

static int cmd_rxprint(int argc, char **argv)
{
	bool on;
	if (argc != 2 || !parse_bool(argv[1], &on)) {
		printf("usage: rxprint on|off   (print received frames)\n");
		return 1;
	}
	s_app.rx_print = on;
	printf("rx print %s\n", on ? "on" : "off");
	return 0;
}

static int cmd_vbat(int argc, char **argv)
{
	(void)argc;
	(void)argv;
	printf("VBAT = %.3f V\n", app_read_vbat());
	return 0;
}

static int cmd_sleep(int argc, char **argv)
{
	uint32_t seconds = 0;
	bool light = false;
	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "light") == 0) {
			light = true;
		} else if (strcmp(argv[i], "deep") == 0) {
			light = false;
		} else if (!parse_u32(argv[i], 0, 86400U, &seconds)) {
			printf("usage: sleep [seconds]   (0 or omitted = deep sleep until reset/EN)\n");
			return 1;
		}
	}
	if (light) {
		/* The ESP32-S3 USB-Serial/JTAG console does not survive light sleep (IDF powers the
		 * USJ pads down and the console tasks block on the dead link). Use deep sleep. */
		printf("light sleep is not supported with the USB console - use deep sleep\n");
		return 1;
	}
	app_enter_sleep(seconds, light);
	return 0;
}

static int cmd_restart(int argc, char **argv)
{
	(void)argc;
	(void)argv;
	printf("restarting...\n");
	fflush(stdout);
	vTaskDelay(pdMS_TO_TICKS(50));
	esp_restart();
	return 0;
}

static void register_cmd(const char *name, const char *help, const char *hint, esp_console_cmd_func_t fn)
{
	const esp_console_cmd_t cmd = {
		.command = name,
		.help = help,
		.hint = hint,
		.func = fn,
	};
	ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

void app_console_start(void)
{
	esp_console_repl_t *repl = NULL;
	esp_console_dev_usb_serial_jtag_config_t usj_cfg = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
	esp_console_repl_config_t repl_cfg = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
	repl_cfg.prompt = "wican>";
	repl_cfg.max_history_len = 30;
	repl_cfg.task_stack_size = 6144;

	ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&usj_cfg, &repl_cfg, &repl));
	ESP_ERROR_CHECK(esp_console_register_help_command());

	register_cmd("status", "Show both CAN channels, error counters and VBAT", NULL, cmd_status);
	register_cmd("can_set", "Reconfigure the MCP2518FD (external/FD) channel", USAGE_CAN_SET + 8, cmd_can_set);
	register_cmd("can_set_int", "Reconfigure the internal (TWAI) channel bitrate", "<bitrate_hz>", cmd_can_set_int);
	register_cmd("can_send", "Send one frame on a channel", USAGE_CAN_SEND + 9, cmd_can_send);
	register_cmd("test", "Enable/disable periodic test traffic", "on|off", cmd_test);
	register_cmd("rxprint", "Enable/disable printing of received frames", "on|off", cmd_rxprint);
	register_cmd("vbat", "Read battery voltage", NULL, cmd_vbat);
	register_cmd("sleep", "Power down CAN controllers/transceivers and sleep the ESP32", "[seconds]   (0 = until reset)", cmd_sleep);
	register_cmd("restart", "Reboot", NULL, cmd_restart);

	printf("\nebb_wican console ready - type 'help'\n\n");
	ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
