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

// The oscillator is stable within 3 ms of a power-on or wake (datasheet
// TOSCSTAB, TOSCSLEEP). Linux's mcp251xfd driver waits as long, and retries its
// reset three times.
#define OSC_STAB_USEC  3000
#define RESET_ATTEMPTS 3
// Bounds the wait for the driver's thread in controller_sleep(), in 1-2 ms
// sleeps. It only runs out if a controller fault keeps the thread retrying.
#define IDLE_POLLS     100

// Writing OSC wakes the controller: asserting nCS ends Low Power Mode (see
// controller_sleep()), and clearing OSCDIS ends Sleep mode. The caller holds
// the driver's mutex, or the driver isn't initialized yet.
static void controller_wake(mcp251xfd_node_t *node)
{
	uint32_t *reg = mcp251xfd_get_spi_buf_ptr(&node->dev);

	*reg = sys_cpu_to_le32(FIELD_PREP(MCP251XFD_REG_OSC_CLKODIV_MASK, node->config.clko_div));
	(void)mcp251xfd_write(&node->dev, MCP251XFD_REG_OSC, MCP251XFD_REG_SIZE);
	k_sleep(K_USEC(OSC_STAB_USEC));
}

// Brings the controller from any state (running, asleep, or partly set up) to
// Configuration mode with mcp251xfd_init()'s register setup; configure() then
// applies bit timing, mode and filters. Like
// Linux's driver: wake the controller, reset it, and check that it's awake.
static int controller_reset(mcp251xfd_node_t *node)
{
	const struct device *dev = &node->dev;
	int ret = -EIO;

	k_mutex_lock(&node->data.mutex, K_FOREVER);
	for (int attempt = 0; attempt < RESET_ATTEMPTS && ret < 0; attempt++) {
		controller_wake(node);
		// Requests Configuration mode, then sends RESET.
		ret = mcp251xfd_reset(dev);
		// A controller still in Sleep mode reads as in Configuration mode
		// and ignores RESET, so check that it's awake. That also catches
		// erratum DS80000789 #7, a wake from Sleep mode that doesn't last.
		// This firmware only sleeps in Low Power Mode, where it doesn't
		// apply, but a controller keeps its power across restarts.
		if (ret == 0) {
			ret = mcp251xfd_reg_check_value_wtimeout(
				dev, MCP251XFD_REG_OSC, MCP251XFD_REG_OSC_OSCRDY,
				MCP251XFD_REG_OSC_OSCRDY | MCP251XFD_REG_OSC_OSCDIS, OSC_STAB_USEC, 1,
				true);
		}
	}
	if (ret == 0) {
		node->data.current_mcp251xfd_mode = MCP251XFD_REG_CON_MODE_CONFIG;
		ret = mcp251xfd_init_con_reg(dev);
	}
	if (ret == 0) {
		ret = mcp251xfd_init_osc_reg(dev);
	}
	if (ret == 0) {
		ret = mcp251xfd_init_iocon_reg(dev);
	}
	if (ret == 0) {
		ret = mcp251xfd_init_int_reg(dev);
	}
	if (ret == 0) {
		ret = mcp251xfd_set_tdc(dev, false);
	}
	if (ret == 0) {
		ret = mcp251xfd_init_tef_fifo(dev);
	}
	if (ret == 0) {
		ret = mcp251xfd_init_tx_queue(dev);
	}
	if (ret == 0) {
		ret = mcp251xfd_init_rx_fifo(dev);
	}
	k_mutex_unlock(&node->data.mutex);
	return ret;
}

// Low Power Mode stops the controller's clock and powers most of it down (4 uA
// typical instead of ~15 mA), losing its registers and RAM, which
// controller_reset() rebuilds. Asserting nCS wakes it, so there is no
// confirmation that it went to sleep. Without CiINT.WAKIE, bus activity does
// not wake it.
static int controller_sleep(mcp251xfd_node_t *node)
{
	const struct device *dev = &node->dev;
	uint32_t *reg;
	uint8_t *reg_byte;

	// Release INT first: entering Sleep sets MODIF, and an asserted INT would
	// otherwise stay low for as long as the controller sleeps.
	k_mutex_lock(&node->data.mutex, K_FOREVER);
	reg = mcp251xfd_get_spi_buf_ptr(dev);
	*reg = 0;
	int ret = mcp251xfd_write(dev, MCP251XFD_REG_INT, MCP251XFD_REG_SIZE);
	k_mutex_unlock(&node->data.mutex);
	// Any later SPI access would wake the controller again, and the driver's
	// thread can still be servicing the mode change of can_stop(). Wait until
	// it has re-enabled INT's interrupt: it has then finished, and with INT
	// released it won't be woken again. If it doesn't, leave the controller
	// awake rather than have the thread wake it.
	for (int i = 0; i < IDLE_POLLS && !gpio_pin_interrupt_enabled_dt(&node->config.int_gpio_dt);
	     i++) {
		k_sleep(K_MSEC(1));
	}
	if (ret == 0 && !gpio_pin_interrupt_enabled_dt(&node->config.int_gpio_dt)) {
		ret = -EBUSY;
	}

	k_mutex_lock(&node->data.mutex, K_FOREVER);
	// LPMEN makes the Sleep mode request below enter Low Power Mode.
	if (ret == 0) {
		reg_byte = mcp251xfd_get_spi_buf_ptr(dev);
		*reg_byte = FIELD_PREP(MCP251XFD_REG_OSC_CLKODIV_MASK, node->config.clko_div) |
			    MCP251XFD_REG_OSC_LPMEN;
		ret = mcp251xfd_write(dev, MCP251XFD_REG_OSC, 1);
	}
	if (ret == 0) {
		reg_byte = mcp251xfd_get_spi_buf_ptr(dev);
		*reg_byte = FIELD_PREP(MCP251XFD_REG_CON_REQOP_MASK, MCP251XFD_REG_CON_MODE_SLEEP) >> 24;
		ret = mcp251xfd_write(dev, MCP251XFD_REG_CON_B3, 1);
	}
	k_mutex_unlock(&node->data.mutex);
	return ret;
}

// Releases the node for the next twai_new_node_mcp251xfd(), which resets the
// controller, and puts the controller to sleep until then.
static esp_err_t node_delete(twai_node_handle_t handle)
{
	mcp251xfd_node_t *node = node_of(handle);

	// can_stop() failed partway (an SPI failure or a mode-change timeout) and
	// left the driver started, possibly with the controller still on the bus.
	// Reset it so it can go to sleep, and release the node anyway.
	if (node->data.common.started) {
		ESP_LOGW(TAG, "controller did not stop, resetting it");
		int ret = controller_reset(node);
		if (ret < 0) {
			ESP_LOGW(TAG, "controller reset failed: %d", ret);
		}
		mcp251xfd_reset_tx_fifos(&node->dev, -ENETDOWN);
		node->data.common.started = false;
	}
	node->callbacks = (twai_event_callbacks_t){};
	node->in_use = false;
	int ret = controller_sleep(node);
	if (ret < 0) {
		// The node is still released; the next creation resets the controller.
		ESP_LOGW(TAG, "controller sleep failed: %d", ret);
	}
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
	// A Classical CAN frame can carry DLC 9..15, which still means 8 bytes;
	// can_dlc_to_bytes() maps those to FD lengths.
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
	// bitrate here is only for mcp251xfd_init(); configure() applies the
	// requested one.
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

	// A restart leaves the controller as it was, and WiCAN's sleep ends in
	// one: wake it from Low Power Mode for initialization.
	controller_wake(node);

	// Starts the driver's interrupt thread, which references the node, so the
	// node is kept even if initialization fails.
	int ret = mcp251xfd_init(&node->dev);
	if (!node->data.int_thread.handle) {
		// Failed setting up the INT pin, before any controller access.
		ESP_LOGE(TAG, "driver initialization failed: %d", ret);
		node->init_error = esp_err_from(ret);
	} else {
		// Transfers so far went through ESP-IDF's SPI driver, which has
		// configured the bus for this device; drive it directly from here on.
		k_mutex_lock(&node->data.mutex, K_FOREVER);
		node->config.bus.hw = SPI_LL_GET_HW(host);
		k_mutex_unlock(&node->data.mutex);
		can_set_state_change_callback(&node->dev, state_changed, node);
		// The rest of initialization only sets up the controller, which every
		// creation repeats with controller_reset().
		if (ret < 0) {
			ESP_LOGW(TAG, "controller initialization failed: %d; resetting it", ret);
		}
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
	// Enabled only here, not by controller_reset(): controller_sleep() takes an
	// enabled interrupt to mean that the driver's thread is idle.
	int ret = controller_reset(s_node);
	if (ret == 0) {
		ret = gpio_pin_interrupt_configure_dt(&s_node->config.int_gpio_dt, GPIO_INT_LEVEL_ACTIVE);
	}
	ESP_RETURN_ON_FALSE(ret >= 0, esp_err_from(ret), TAG, "controller reset failed: %d", ret);
	ESP_RETURN_ON_ERROR(configure(s_node, config), TAG, "configure failed");
	s_node->in_use = true;
	*node_ret = &s_node->base;
	return ESP_OK;
}
