// TWAI node adapter for the vendored Zephyr MCP251xFD driver (see README.md).
// The driver source is compiled into this file so its static functions can be
// instantiated directly, without Zephyr's devicetree.
#include "zephyr/drivers/can/can_mcp251xfd.c"

#include <string.h>
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_private/twai_interface.h"
#include "esp_twai_mcp251xfd.h"

#define TAG "mcp251xfd"
#define NO_FILTER (-1)

typedef struct {
	struct twai_node_base base;
	struct device dev;
	struct mcp251xfd_config config;
	struct mcp251xfd_data data;
	spi_device_handle_t spi;
	twai_event_callbacks_t callbacks;
	void *user_data;
	// The frame on_rx_done is reporting, for receive_isr.
	const struct can_frame *rx_frame;
	twai_error_state_t state;
	int filters[2];
	bool listen_only;
	bool in_use;
	esp_err_t init_error;
} mcp251xfd_node_t;

// One controller per board. The driver has no teardown, so it is kept.
static mcp251xfd_node_t *s_node;

static mcp251xfd_node_t *node_of(twai_node_handle_t handle)
{
	return CONTAINER_OF(handle, mcp251xfd_node_t, base);
}

// Zephyr returns negative errno values; some calls return a result >= 0.
static esp_err_t esp_err_from(int ret)
{
	if (ret >= 0) {
		return ESP_OK;
	}
	switch (ret) {
	case -EAGAIN: return ESP_ERR_TIMEOUT;
	case -EINVAL: return ESP_ERR_INVALID_ARG;
	case -ENOTSUP: return ESP_ERR_NOT_SUPPORTED;
	case -ENOSPC: return ESP_ERR_NO_MEM;
	case -EALREADY:
	case -EBUSY:
	case -ENETDOWN:
	case -ENETUNREACH: return ESP_ERR_INVALID_STATE;
	default: return ESP_FAIL;
	}
}

static void tx_done(const struct device *dev, int error, void *user_data)
{
	mcp251xfd_node_t *node = CONTAINER_OF(dev, mcp251xfd_node_t, dev);
	twai_tx_done_event_data_t event = {.done_tx_frame = user_data, .is_tx_success = error == 0};

	if (node->callbacks.on_tx_done) {
		node->callbacks.on_tx_done(&node->base, &event, node->user_data);
	}
}

static void rx_done(const struct device *dev, struct can_frame *frame, void *user_data)
{
	mcp251xfd_node_t *node = user_data;
	twai_rx_done_event_data_t event = {};

	(void)dev;
	if (node->callbacks.on_rx_done) {
		node->rx_frame = frame;
		node->callbacks.on_rx_done(&node->base, &event, node->user_data);
		node->rx_frame = NULL;
	}
}

static twai_error_state_t twai_state(enum can_state state)
{
	switch (state) {
	case CAN_STATE_ERROR_ACTIVE: return TWAI_ERROR_ACTIVE;
	case CAN_STATE_ERROR_WARNING: return TWAI_ERROR_WARNING;
	case CAN_STATE_ERROR_PASSIVE: return TWAI_ERROR_PASSIVE;
	default: return TWAI_ERROR_BUS_OFF;
	}
}

// On bus-off the driver has already failed every queued frame.
static void state_changed(const struct device *dev, enum can_state state,
			  struct can_bus_err_cnt err_cnt, void *user_data)
{
	mcp251xfd_node_t *node = user_data;
	twai_state_change_event_data_t event = {.old_sta = node->state, .new_sta = twai_state(state)};

	(void)dev;
	(void)err_cnt;
	// After twai_node_disable() the driver can report STOPPED. TWAI has no
	// such state, and reporting it as bus-off would start a recovery.
	if (state == CAN_STATE_STOPPED) {
		return;
	}
	node->state = event.new_sta;
	if (node->callbacks.on_state_change) {
		node->callbacks.on_state_change(&node->base, &event, node->user_data);
	}
}

static void remove_filters(mcp251xfd_node_t *node)
{
	for (int i = 0; i < 2; i++) {
		if (node->filters[i] != NO_FILTER) {
			can_remove_rx_filter(&node->dev, node->filters[i]);
			node->filters[i] = NO_FILTER;
		}
	}
}

static esp_err_t add_filter(mcp251xfd_node_t *node, int slot, uint32_t id, uint32_t mask, bool ext)
{
	struct can_filter filter = {.id = id, .mask = mask, .flags = ext ? CAN_FILTER_IDE : 0};
	int ret = can_add_rx_filter(&node->dev, rx_done, node, &filter);

	node->filters[slot] = ret < 0 ? NO_FILTER : ret;
	return esp_err_from(ret);
}

// The controller receives nothing without a filter. Default to accepting
// every standard and extended frame, like the on-chip node.
static esp_err_t accept_all(mcp251xfd_node_t *node)
{
	remove_filters(node);
	esp_err_t err = add_filter(node, 0, 0, 0, false);
	return err == ESP_OK ? add_filter(node, 1, 0, 0, true) : err;
}

static esp_err_t node_enable(twai_node_handle_t handle)
{
	mcp251xfd_node_t *node = node_of(handle);

	node->state = TWAI_ERROR_ACTIVE;
	return esp_err_from(can_start(&node->dev));
}

// Aborts queued frames, reporting each through on_tx_done before returning.
static esp_err_t node_disable(twai_node_handle_t handle)
{
	return esp_err_from(can_stop(&node_of(handle)->dev));
}

// Releases the node for the next twai_new_node_mcp251xfd(); the controller
// stays initialized in configuration mode.
static esp_err_t node_delete(twai_node_handle_t handle)
{
	mcp251xfd_node_t *node = node_of(handle);

	if (node->data.common.started) {
		return ESP_ERR_INVALID_STATE;
	}
	node->callbacks = (twai_event_callbacks_t){};
	node->in_use = false;
	return ESP_OK;
}

static esp_err_t config_mask_filter(twai_node_handle_t handle, uint8_t index,
				    const twai_mask_filter_config_t *config)
{
	mcp251xfd_node_t *node = node_of(handle);

	if (index != 0 || config->dual_filter || config->no_classic || config->num_of_ids > 1) {
		return ESP_ERR_NOT_SUPPORTED;
	}
	if (node->data.common.started) {
		return ESP_ERR_INVALID_STATE;
	}
	remove_filters(node);
	return add_filter(node, 0, config->num_of_ids ? config->id_list[0] : config->id,
			  config->mask, config->is_ext);
}

static esp_err_t register_cbs(twai_node_handle_t handle, const twai_event_callbacks_t *cbs,
			      void *user_data)
{
	mcp251xfd_node_t *node = node_of(handle);

	if (node->data.common.started) {
		return ESP_ERR_INVALID_STATE;
	}
	node->callbacks = *cbs;
	node->user_data = user_data;
	return ESP_OK;
}

static esp_err_t transmit(twai_node_handle_t handle, const twai_frame_t *frame, int timeout_ms)
{
	mcp251xfd_node_t *node = node_of(handle);

	// The driver writes over SPI under a mutex. Listen-only frames never complete.
	if (xPortInIsrContext() || node->listen_only || frame->header.fdf) {
		return ESP_ERR_NOT_SUPPORTED;
	}
	if (frame->header.dlc > CAN_MAX_DLC || (frame->buffer_len && !frame->buffer)) {
		return ESP_ERR_INVALID_ARG;
	}
	struct can_frame message = {
		.id = frame->header.id,
		.dlc = frame->header.dlc,
		.flags = (frame->header.ide ? CAN_FRAME_IDE : 0) | (frame->header.rtr ? CAN_FRAME_RTR : 0),
	};
	if (!frame->header.rtr && frame->buffer_len) {
		memcpy(message.data, frame->buffer, MIN(frame->buffer_len, sizeof(message.data)));
	}
	// The driver frees a TX mailbox just after on_tx_done returns. A caller
	// whose frame slot was recycled by on_tx_done can get here first, so wait
	// at least a tick even when asked not to.
	k_timeout_t timeout = timeout_ms < 0 ? K_FOREVER : K_MSEC(MAX(timeout_ms, 1));

	return esp_err_from(can_send(&node->dev, &message, timeout, tx_done, (void *)frame));
}

// Only valid inside on_rx_done, which runs in the driver's thread.
static esp_err_t receive_isr(twai_node_handle_t handle, twai_frame_t *out)
{
	const struct can_frame *frame = node_of(handle)->rx_frame;

	if (!frame) {
		return ESP_ERR_INVALID_STATE;
	}
	bool rtr = frame->flags & CAN_FRAME_RTR;
	// Zephyr can report DLC 9..15 for FD traffic in Classical CAN mode.
	size_t len = MIN(rtr ? 0 : MIN(can_dlc_to_bytes(frame->dlc), CAN_MAX_DLEN), out->buffer_len);

	out->header = (twai_frame_header_t){
		.id = frame->id,
		.dlc = frame->dlc,
		.ide = (frame->flags & CAN_FRAME_IDE) != 0,
		.rtr = rtr,
	};
	if (len) {
		memcpy(out->buffer, frame->data, len);
	}
	out->buffer_len = len;
	return ESP_OK;
}

// Bit timing, mode and default filters, applied while the controller is stopped.
static esp_err_t configure(mcp251xfd_node_t *node, const twai_mcp251xfd_node_config_t *config)
{
	struct can_timing timing = {0};
	can_mode_t mode = (config->flags.enable_listen_only ? CAN_MODE_LISTENONLY : 0) |
			  (config->flags.enable_loopback ? CAN_MODE_LOOPBACK : 0);
	int ret = can_calc_timing(&node->dev, &timing, config->bit_timing.bitrate,
				  config->bit_timing.sp_permill);

	if (ret >= 0) {
		ret = can_set_timing(&node->dev, &timing);
	}
	if (ret >= 0) {
		ret = can_set_mode(&node->dev, mode);
	}
	ESP_RETURN_ON_ERROR(esp_err_from(ret), TAG, "bit timing/mode setup failed: %d", ret);
	node->listen_only = config->flags.enable_listen_only;
	return accept_all(node);
}

static esp_err_t create(spi_host_device_t host, const twai_mcp251xfd_node_config_t *config)
{
	mcp251xfd_node_t *node = heap_caps_calloc(1, sizeof(*node), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
	ESP_RETURN_ON_FALSE(node, ESP_ERR_NO_MEM, TAG, "no memory");

	spi_device_interface_config_t spi_config = {
		.clock_speed_hz = config->spi_clock_hz,
		.mode = 0,
		.spics_io_num = config->io_cfg.cs_gpio,
		.queue_size = 1,
	};
	esp_err_t err = spi_bus_add_device(host, &spi_config, &node->spi);
	// The bus is dedicated to this controller: reserve it once instead of
	// arbitrating on every short transfer.
	if (err == ESP_OK) {
		err = spi_device_acquire_bus(node->spi, portMAX_DELAY);
		if (err != ESP_OK) {
			spi_bus_remove_device(node->spi);
		}
	}
	if (err != ESP_OK) {
		free(node);
		return err;
	}

	// Mirrors the driver's devicetree instantiation (MCP251XFD_INIT). The
	// bitrate here is only for initialization, which cannot be retried;
	// configure() applies the requested one.
	const struct mcp251xfd_config driver_config = {
		.common = {.max_bitrate = 1000000, .bitrate = 500000},
		.bus = {.bus = &node->dev, .device = node->spi},
		.int_gpio_dt = {.pin = config->io_cfg.int_gpio, .dt_flags = GPIO_ACTIVE_LOW | GPIO_PULL_UP},
		.osc_freq = config->oscillator_hz,
		.clko_div = MCP251XFD_REG_OSC_CLKODIV_10,
		.rx_fifo = {
			.ram_start_addr = MCP251XFD_RX_FIFO_START_ADDR,
			.reg_fifocon_addr = MCP251XFD_REG_FIFOCON(MCP251XFD_RX_FIFO_IDX),
			.capacity = MCP251XFD_RX_FIFO_ITEMS,
			.item_size = MCP251XFD_RX_FIFO_ITEM_SIZE,
			.msg_handler = mcp251xfd_rx_fifo_handler,
		},
		.tef_fifo = {
			.ram_start_addr = MCP251XFD_TEF_FIFO_START_ADDR,
			.reg_fifocon_addr = MCP251XFD_REG_TEFCON,
			.capacity = MCP251XFD_TEF_FIFO_ITEMS,
			.item_size = MCP251XFD_TEF_FIFO_ITEM_SIZE,
			.msg_handler = mcp251xfd_tef_fifo_handler,
		},
	};
	// The driver's config type has const members, so it cannot be assigned.
	memcpy(&node->config, &driver_config, sizeof(driver_config));
	node->dev = (struct device){
		.name = TAG,
		.config = &node->config,
		.api = &mcp251xfd_api_funcs,
		.data = &node->data,
	};
	node->base = (struct twai_node_base){
		.enable = node_enable,
		.disable = node_disable,
		.del = node_delete,
		.config_mask_filter = config_mask_filter,
		.transmit = transmit,
		.receive_isr = receive_isr,
		.register_cbs = register_cbs,
	};
	node->filters[0] = node->filters[1] = NO_FILTER;

	// Starts the driver's interrupt thread, which references the node, so the
	// node is kept even if initialization fails.
	int ret = mcp251xfd_init(&node->dev);
	if (ret < 0) {
		ESP_LOGE(TAG, "controller initialization failed: %d", ret);
		node->init_error = esp_err_from(ret);
	} else {
		// Initialization went through ESP-IDF's SPI driver, which has now
		// configured the bus for this device; drive it directly from here on.
		k_mutex_lock(&node->data.mutex, K_FOREVER);
		node->config.bus.hw = SPI_LL_GET_HW(host);
		k_mutex_unlock(&node->data.mutex);
		can_set_state_change_callback(&node->dev, state_changed, node);
	}
	s_node = node;
	return ESP_OK;
}

esp_err_t twai_new_node_mcp251xfd(spi_host_device_t host, const twai_mcp251xfd_node_config_t *config,
				  twai_node_handle_t *node_ret)
{
	ESP_RETURN_ON_FALSE(config && node_ret, ESP_ERR_INVALID_ARG, TAG, "null argument");
	if (!s_node) {
		ESP_RETURN_ON_ERROR(create(host, config), TAG, "create failed");
	}
	ESP_RETURN_ON_FALSE(!s_node->in_use, ESP_ERR_INVALID_STATE, TAG, "node already in use");
	ESP_RETURN_ON_ERROR(s_node->init_error, TAG, "controller unavailable until restart");
	ESP_RETURN_ON_ERROR(configure(s_node, config), TAG, "configure failed");
	s_node->in_use = true;
	*node_ret = &s_node->base;
	return ESP_OK;
}
