// TWAI node adapter for the vendored Zephyr MCP251xFD driver (see README.md).
// The driver source is compiled into this file so its static functions can be
// instantiated directly, without Zephyr's devicetree.
#include "zephyr/drivers/can/can_mcp251xfd.c"

#include <string.h>
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
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
// Bring-up diagnostics: what the adapter believes the controller's power
// state is. Set by controller_sleep(), cleared by any wake/reset. Used by
// can_dump_diagnostics() to show whether a bus that was never enabled is
// leaving the MCP2518FD awake.
static bool s_controller_sleeping;
static uint32_t s_tx_reject_log;
static uint32_t s_log_missing_cb;

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
	default:
		if (mcp251xfd_diag_log(&s_tx_reject_log, 10, 1000)) {
			ESP_LOGW(TAG, "unmapped driver error %d -> ESP_FAIL", ret);
		}
		return ESP_FAIL;
	}
}

static const char *twai_state_name(twai_error_state_t state)
{
	switch (state) {
	case TWAI_ERROR_ACTIVE: return "active";
	case TWAI_ERROR_WARNING: return "warning";
	case TWAI_ERROR_PASSIVE: return "error-passive";
	case TWAI_ERROR_BUS_OFF: return "bus-off";
	default: return "?";
	}
}

// Read one 32-bit register for diagnostics. Caller must hold data.mutex.
static bool mcp251xfd_dump_read(mcp251xfd_node_t *node, uint16_t addr, uint32_t *out)
{
	uint32_t *p = mcp251xfd_read_crc(&node->dev, addr, MCP251XFD_REG_SIZE);

	if (!p) {
		return false;
	}
	*out = sys_le32_to_cpu(*p);
	return true;
}

// Log the controller identity, clock and error state after a reset or from the
// periodic dump. Caller must hold data.mutex. Pure read-only SPI.
static void mcp251xfd_log_registers(mcp251xfd_node_t *node, const char *why)
{
	uint32_t devid = 0, osc = 0, iocon = 0, con = 0, intr = 0, trec = 0;
	uint32_t fifosta_tx = 0, fifoua_tx = 0, fifosta_rx = 0, fifoua_rx = 0, tefsta = 0;
	uint32_t nbtcfg = 0;
	bool ok = true;

	ok &= mcp251xfd_dump_read(node, MCP251XFD_REG_DEVID, &devid);
	ok &= mcp251xfd_dump_read(node, MCP251XFD_REG_OSC, &osc);
	ok &= mcp251xfd_dump_read(node, MCP251XFD_REG_IOCON, &iocon);
	ok &= mcp251xfd_dump_read(node, MCP251XFD_REG_CON, &con);
	ok &= mcp251xfd_dump_read(node, MCP251XFD_REG_INT, &intr);
	ok &= mcp251xfd_dump_read(node, MCP251XFD_REG_TREC, &trec);
	ok &= mcp251xfd_dump_read(node, MCP251XFD_REG_FIFOSTA(MCP251XFD_TX_FIFO_IDX), &fifosta_tx);
	ok &= mcp251xfd_dump_read(node, MCP251XFD_REG_FIFOUA(MCP251XFD_TX_FIFO_IDX), &fifoua_tx);
	ok &= mcp251xfd_dump_read(node, MCP251XFD_REG_FIFOSTA(MCP251XFD_RX_FIFO_IDX), &fifosta_rx);
	ok &= mcp251xfd_dump_read(node, MCP251XFD_REG_FIFOUA(MCP251XFD_RX_FIFO_IDX), &fifoua_rx);
	ok &= mcp251xfd_dump_read(node, MCP251XFD_REG_TEFSTA, &tefsta);
	ok &= mcp251xfd_dump_read(node, MCP251XFD_REG_NBTCFG, &nbtcfg);

	ESP_LOGI(TAG, "[%s] registers%s", why, ok ? "" : " (SOME READS FAILED)");
	ESP_LOGI(TAG, "[%s] DEVID=0x%08lx id=%lu rev=%lu OSC=0x%08lx "
			 "(OSCRDY=%d PLLRDY=%d LPMEN=%d OSCDIS=%d CLKODIV=%lu PLLEN=%d)",
			 why, (unsigned long)devid,
			 (unsigned long)FIELD_GET(MCP251XFD_REG_DEVID_ID_MASK, devid),
			 (unsigned long)FIELD_GET(MCP251XFD_REG_DEVID_REV_MASK, devid),
			 (unsigned long)osc,
			 (osc & MCP251XFD_REG_OSC_OSCRDY) != 0, (osc & MCP251XFD_REG_OSC_PLLRDY) != 0,
			 (osc & MCP251XFD_REG_OSC_LPMEN) != 0, (osc & MCP251XFD_REG_OSC_OSCDIS) != 0,
			 (unsigned long)FIELD_GET(MCP251XFD_REG_OSC_CLKODIV_MASK, osc),
			 (osc & MCP251XFD_REG_OSC_PLLEN) != 0);
	ESP_LOGI(TAG, "[%s] IOCON=0x%08lx (XSTBYEN=%d TRIS0=%d TRIS1=%d PM0=%d PM1=%d) "
			 "CON=0x%08lx (OPMOD=%lu REQOP=%lu ABAT=%d)",
			 why, (unsigned long)iocon, (iocon & MCP251XFD_REG_IOCON_XSTBYEN) != 0,
			 (iocon & MCP251XFD_REG_IOCON_TRIS0) != 0, (iocon & MCP251XFD_REG_IOCON_TRIS1) != 0,
			 (iocon & MCP251XFD_REG_IOCON_PM0) != 0, (iocon & MCP251XFD_REG_IOCON_PM1) != 0,
			 (unsigned long)con,
			 (unsigned long)FIELD_GET(MCP251XFD_REG_CON_OPMOD_MASK, con),
			 (unsigned long)FIELD_GET(MCP251XFD_REG_CON_REQOP_MASK, con),
			 (con & MCP251XFD_REG_CON_ABAT) != 0);
	ESP_LOGI(TAG, "[%s] TREC=0x%08lx (TEC=%lu REC=%lu TXBO=%d TXBP=%d RXBP=%d EWARN=%d) "
			 "INT=0x%08lx TEFSTA=0x%08lx",
			 why, (unsigned long)trec,
			 (unsigned long)FIELD_GET(MCP251XFD_REG_TREC_TEC_MASK, trec),
			 (unsigned long)FIELD_GET(MCP251XFD_REG_TREC_REC_MASK, trec),
			 (trec & MCP251XFD_REG_TREC_TXBO) != 0, (trec & MCP251XFD_REG_TREC_TXBP) != 0,
			 (trec & MCP251XFD_REG_TREC_RXBP) != 0, (trec & MCP251XFD_REG_TREC_EWARN) != 0,
			 (unsigned long)intr, (unsigned long)tefsta);
	ESP_LOGI(TAG, "[%s] NBTCFG=0x%08lx (prescaler=%lu tseg1=%lu tseg2=%lu sjw=%lu; "
			 "fields are value-1)",
			 why, (unsigned long)nbtcfg,
			 (unsigned long)FIELD_GET(MCP251XFD_REG_NBTCFG_BRP_MASK, nbtcfg) + 1,
			 (unsigned long)FIELD_GET(MCP251XFD_REG_NBTCFG_TSEG1_MASK, nbtcfg) + 1,
			 (unsigned long)FIELD_GET(MCP251XFD_REG_NBTCFG_TSEG2_MASK, nbtcfg) + 1,
			 (unsigned long)FIELD_GET(MCP251XFD_REG_NBTCFG_SJW_MASK, nbtcfg) + 1);
	ESP_LOGI(TAG, "[%s] FIFO%d TX: STA=0x%08lx UA=0x%03lx FIFO%d RX: STA=0x%08lx UA=0x%03lx "
			 "(RXOVIF=%d TEFOVIF=%d, overflows rx=%lu tef=%lu)",
			 why, MCP251XFD_TX_FIFO_IDX, (unsigned long)fifosta_tx, (unsigned long)fifoua_tx,
			 MCP251XFD_RX_FIFO_IDX, (unsigned long)fifosta_rx, (unsigned long)fifoua_rx,
			 (fifosta_rx & MCP251XFD_REG_FIFOSTA_RXOVIF) != 0,
			 (tefsta & MCP251XFD_REG_TEFSTA_TEFOVIF) != 0,
			 (unsigned long)node->data.rx_overflow_count,
			 (unsigned long)node->data.tef_overflow_count);
}

static void tx_done(const struct device *dev, int error, void *user_data)
{
	mcp251xfd_node_t *node = CONTAINER_OF(dev, mcp251xfd_node_t, dev);
	twai_tx_done_event_data_t event = {.done_tx_frame = user_data, .is_tx_success = error == 0};

	if (node->callbacks.on_tx_done) {
		node->callbacks.on_tx_done(&node->base, &event, node->user_data);
	} else if (mcp251xfd_diag_log(&s_log_missing_cb, 5, 1000)) {
		// A callback after node_delete() means a frame completed while the
		// node was being torn down; can.c has already reset its TX slots.
		ESP_LOGW(TAG, "TX done with no registered handler (error=%d, in_use=%d)",
			 error, node->in_use);
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
	} else if (mcp251xfd_diag_log(&s_log_missing_cb, 5, 1000)) {
		ESP_LOGW(TAG, "RX frame with no registered handler (id=0x%x, in_use=%d)",
			 frame->id, node->in_use);
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
	// After twai_node_disable() the driver can report STOPPED. TWAI has no
	// such state, and reporting it as bus-off would start a recovery.
	if (state == CAN_STATE_STOPPED) {
		ESP_LOGI(TAG, "driver reports STOPPED (TEC=%u REC=%u); not forwarding as a TWAI state",
			 (unsigned)err_cnt.tx_err_cnt, (unsigned)err_cnt.rx_err_cnt);
		return;
	}
	ESP_LOGI(TAG, "state: TWAI %s -> %s (driver %s, TEC=%u REC=%u, in_use=%d)",
		 twai_state_name(event.old_sta), twai_state_name(event.new_sta),
		 mcp251xfd_state_name(state), (unsigned)err_cnt.tx_err_cnt,
		 (unsigned)err_cnt.rx_err_cnt, node->in_use);
	node->state = event.new_sta;
	if (node->callbacks.on_state_change) {
		node->callbacks.on_state_change(&node->base, &event, node->user_data);
	} else if (mcp251xfd_diag_log(&s_log_missing_cb, 5, 1000)) {
		ESP_LOGW(TAG, "state change with no registered handler (new=%s, in_use=%d)",
			 twai_state_name(event.new_sta), node->in_use);
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
	ESP_LOGI(TAG, "add_filter slot=%d id=0x%lx mask=0x%lx ext=%d -> driver index %d (%s)",
		 slot, (unsigned long)id, (unsigned long)mask, ext, ret,
		 esp_err_to_name(esp_err_from(ret)));
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
	int started_before = node->data.common.started;
	int64_t t0 = esp_timer_get_time();

	// Re-enable the INT pin in case this node was disabled without being
	// deleted (controller_sleep() normally goes through node_delete).
	k_mutex_lock(&node->data.mutex, K_FOREVER);
	node->data.int_suspended = false;
	int int_ret = gpio_pin_interrupt_configure_dt(&node->config.int_gpio_dt,
						      GPIO_INT_LEVEL_ACTIVE);
	k_mutex_unlock(&node->data.mutex);

	ESP_LOGI(TAG, "node_enable: listen_only=%d driver_state=%s started=%d mode=%u int_gpio=%d",
		 node->listen_only, mcp251xfd_state_name(node->data.state), started_before,
		 (unsigned)node->data.next_mcp251xfd_mode, int_ret);
	node->state = TWAI_ERROR_ACTIVE;
	int ret = can_start(&node->dev);
	ESP_LOGI(TAG, "node_enable: can_start -> %d after %lld ms (started=%d, driver_state=%s)",
		 ret, (long long)((esp_timer_get_time() - t0) / 1000),
		 node->data.common.started, mcp251xfd_state_name(node->data.state));
	return esp_err_from(ret);
}

// Aborts queued frames, reporting each through on_tx_done before returning.
static esp_err_t node_disable(twai_node_handle_t handle)
{
	mcp251xfd_node_t *node = node_of(handle);
	int64_t t0 = esp_timer_get_time();
	int ret = can_stop(&node->dev);

	ESP_LOGI(TAG, "node_disable: can_stop -> %d after %lld ms (started=%d, driver_state=%s, "
		 "mailbox_usage=0x%08lx)",
		 ret, (long long)((esp_timer_get_time() - t0) / 1000), node->data.common.started,
		 mcp251xfd_state_name(node->data.state),
		 (unsigned long)node->data.mailbox_usage);
	return esp_err_from(ret);
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
	int64_t t0 = esp_timer_get_time();

	ESP_LOGI(TAG, "controller_wake: sleeping=%d in_use=%d (asserting nCS / clearing "
		 "LPMEN+OSCDIS)", s_controller_sleeping, node->in_use);
	*reg = sys_cpu_to_le32(FIELD_PREP(MCP251XFD_REG_OSC_CLKODIV_MASK, node->config.clko_div));
	int ret = mcp251xfd_write(&node->dev, MCP251XFD_REG_OSC, MCP251XFD_REG_SIZE);
	k_sleep(K_USEC(OSC_STAB_USEC));
	s_controller_sleeping = false;
	ESP_LOGI(TAG, "controller_wake: OSC write -> %d, waited %lld ms",
		 ret, (long long)((esp_timer_get_time() - t0) / 1000));
}

// Brings the controller from any state (running, asleep, or partly set up) to
// Configuration mode with mcp251xfd_init()'s register setup; configure() then
// applies bit timing, mode and filters. Like
// Linux's driver: wake the controller, reset it, and check that it's awake.
static int controller_reset(mcp251xfd_node_t *node)
{
	const struct device *dev = &node->dev;
	int ret = -EIO;
	int64_t t0 = esp_timer_get_time();

	s_controller_sleeping = false;
	ESP_LOGI(TAG, "controller_reset: begin (driver_state=%s started=%d mailbox_usage=0x%08lx)",
		 mcp251xfd_state_name(node->data.state), node->data.common.started,
		 (unsigned long)node->data.mailbox_usage);

	k_mutex_lock(&node->data.mutex, K_FOREVER);
	for (int attempt = 0; attempt < RESET_ATTEMPTS && ret < 0; attempt++) {
		ESP_LOGI(TAG, "controller_reset: attempt %d/%d", attempt + 1, RESET_ATTEMPTS);
		controller_wake(node);
		// Requests Configuration mode, then sends RESET.
		ret = mcp251xfd_reset(dev);
		ESP_LOGI(TAG, "controller_reset: reset attempt %d -> %d", attempt + 1, ret);
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
			ESP_LOGI(TAG, "controller_reset: oscillator/awake check -> %d", ret);
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
	if (ret == 0) {
		mcp251xfd_log_registers(node, "after-reset");
	}
	ESP_LOGI(TAG, "controller_reset: done ret=%d in %lld ms (state=%s started=%d)",
		 ret, (long long)((esp_timer_get_time() - t0) / 1000),
		 mcp251xfd_state_name(node->data.state), node->data.common.started);
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
	int64_t t0 = esp_timer_get_time();
	int idle_polls = 0;

	ESP_LOGI(TAG, "controller_sleep: begin (driver_state=%s started=%d mailbox_usage=0x%08lx, "
		 "in_use=%d)", mcp251xfd_state_name(node->data.state),
		 node->data.common.started, (unsigned long)node->data.mailbox_usage, node->in_use);

	// Release INT first: entering Sleep sets MODIF, and an asserted INT would
	// otherwise stay low for as long as the controller sleeps.
	k_mutex_lock(&node->data.mutex, K_FOREVER);
	reg = mcp251xfd_get_spi_buf_ptr(dev);
	*reg = 0;
	int ret = mcp251xfd_write(dev, MCP251XFD_REG_INT, MCP251XFD_REG_SIZE);
	k_mutex_unlock(&node->data.mutex);
	ESP_LOGI(TAG, "controller_sleep: INT enables cleared -> %d", ret);
	// Any later SPI access would wake the controller again, and the driver's
	// thread can still be servicing the mode change of can_stop(). Wait until
	// it has re-enabled INT's interrupt: it has then finished, and with INT
	// released it won't be woken again. If it doesn't, leave the controller
	// awake rather than have the thread wake it.
	for (int i = 0; i < IDLE_POLLS && !gpio_pin_interrupt_enabled_dt(&node->config.int_gpio_dt);
	     i++) {
		k_sleep(K_MSEC(1));
		idle_polls++;
	}
	ESP_LOGI(TAG, "controller_sleep: INT thread idle after %d polls (int_enabled=%d), "
		 "%lld ms elapsed", idle_polls,
		 gpio_pin_interrupt_enabled_dt(&node->config.int_gpio_dt),
		 (long long)((esp_timer_get_time() - t0) / 1000));
	if (ret == 0 && !gpio_pin_interrupt_enabled_dt(&node->config.int_gpio_dt)) {
		ESP_LOGW(TAG, "controller_sleep: driver thread did not go idle; leaving the "
			 "controller awake");
		ret = -EBUSY;
	}

	if (ret == 0) {
		// The thread has finished. Mark it suspended while holding the same
		// mutex the thread uses before re-enabling the pin, then disable the
		// pin: a queued edge can no longer read INT from the sleeping part.
		k_mutex_lock(&node->data.mutex, K_FOREVER);
		node->data.int_suspended = true;
		int gpio_ret = gpio_pin_interrupt_configure_dt(&node->config.int_gpio_dt,
							      GPIO_INT_DISABLE);
		k_mutex_unlock(&node->data.mutex);
		ESP_LOGI(TAG, "controller_sleep: INT GPIO%d disabled -> %d (suspended=%d)",
			 node->config.int_gpio_dt.pin, gpio_ret, node->data.int_suspended);
	}

	k_mutex_lock(&node->data.mutex, K_FOREVER);
	// LPMEN makes the Sleep mode request below enter Low Power Mode.
	if (ret == 0) {
		reg_byte = mcp251xfd_get_spi_buf_ptr(dev);
		*reg_byte = FIELD_PREP(MCP251XFD_REG_OSC_CLKODIV_MASK, node->config.clko_div) |
			    MCP251XFD_REG_OSC_LPMEN;
		ret = mcp251xfd_write(dev, MCP251XFD_REG_OSC, 1);
		ESP_LOGI(TAG, "controller_sleep: OSC LPMEN write -> %d", ret);
	}
	if (ret == 0) {
		reg_byte = mcp251xfd_get_spi_buf_ptr(dev);
		*reg_byte = FIELD_PREP(MCP251XFD_REG_CON_REQOP_MASK, MCP251XFD_REG_CON_MODE_SLEEP) >> 24;
		ret = mcp251xfd_write(dev, MCP251XFD_REG_CON_B3, 1);
		ESP_LOGI(TAG, "controller_sleep: CON sleep request -> %d", ret);
	}
	k_mutex_unlock(&node->data.mutex);
	s_controller_sleeping = (ret == 0);
	ESP_LOGI(TAG, "controller_sleep: done ret=%d, sleeping=%d, %lld ms total",
		 ret, s_controller_sleeping, (long long)((esp_timer_get_time() - t0) / 1000));
	return ret;
}

// Releases the node for the next twai_new_node_mcp251xfd(), which resets the
// controller, and puts the controller to sleep until then.
static esp_err_t node_delete(twai_node_handle_t handle)
{
	mcp251xfd_node_t *node = node_of(handle);

	ESP_LOGI(TAG, "node_delete: started=%d state=%s mailbox_usage=0x%08lx filters=(%d,%d)",
		 node->data.common.started, mcp251xfd_state_name(node->data.state),
		 (unsigned long)node->data.mailbox_usage, node->filters[0], node->filters[1]);

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
		ESP_LOGW(TAG, "controller sleep failed: %d (controller stays awake, "
			 "sleeping=%d)", ret, s_controller_sleeping);
	}
	return ESP_OK;
}

static esp_err_t config_mask_filter(twai_node_handle_t handle, uint8_t index,
				    const twai_mask_filter_config_t *config)
{
	mcp251xfd_node_t *node = node_of(handle);

	ESP_LOGI(TAG, "config_mask_filter: index=%u id=0x%lx mask=0x%lx ext=%d dual=%d "
		 "no_classic=%d num_ids=%u started=%d",
		 (unsigned)index, (unsigned long)config->id, (unsigned long)config->mask,
		 (int)config->is_ext, (int)config->dual_filter, (int)config->no_classic,
		 (unsigned)config->num_of_ids, node->data.common.started);
	if (index != 0 || config->dual_filter || config->no_classic || config->num_of_ids > 1) {
		ESP_LOGW(TAG, "config_mask_filter: unsupported combination, returning NOT_SUPPORTED");
		return ESP_ERR_NOT_SUPPORTED;
	}
	if (node->data.common.started) {
		ESP_LOGW(TAG, "config_mask_filter: node is started, returning INVALID_STATE");
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

	ESP_LOGI(TAG, "register_cbs: rx=%d tx=%d state=%d started=%d",
		 cbs->on_rx_done != NULL, cbs->on_tx_done != NULL, cbs->on_state_change != NULL,
		 node->data.common.started);
	if (node->data.common.started) {
		ESP_LOGW(TAG, "register_cbs: node is started, returning INVALID_STATE");
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
		if (mcp251xfd_diag_log(&s_tx_reject_log, 10, 1000)) {
			ESP_LOGW(TAG, "transmit rejected: isr=%d listen_only=%d fdf=%d "
				 "(id=0x%lx dlc=%u)", xPortInIsrContext() != 0, node->listen_only,
				 frame->header.fdf, (unsigned long)frame->header.id, frame->header.dlc);
		}
		return ESP_ERR_NOT_SUPPORTED;
	}
	if (frame->header.dlc > CAN_MAX_DLC || (frame->buffer_len && !frame->buffer)) {
		ESP_LOGW(TAG, "transmit rejected: invalid dlc=%u buffer_len=%u buffer=%p",
			 (unsigned)frame->header.dlc, (unsigned)frame->buffer_len, frame->buffer);
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

	int ret = can_send(&node->dev, &message, timeout, tx_done, (void *)frame);
	esp_err_t err = esp_err_from(ret);
	if (ret < 0 && mcp251xfd_diag_log(&s_tx_reject_log, 10, 1000)) {
		ESP_LOGW(TAG, "transmit: driver can_send -> %d (%s) id=0x%lx dlc=%u "
			 "timeout_ms=%d started=%d state=%s listen_only=%d",
			 ret, esp_err_to_name(err), (unsigned long)frame->header.id,
			 (unsigned)frame->header.dlc, timeout_ms, node->data.common.started,
			 mcp251xfd_state_name(node->data.state), (int)node->listen_only);
	}
	return err;
}

// Only valid inside on_rx_done, which runs in the driver's thread.
static esp_err_t receive_isr(twai_node_handle_t handle, twai_frame_t *out)
{
	const struct can_frame *frame = node_of(handle)->rx_frame;

	if (!frame) {
		if (mcp251xfd_diag_log(&s_tx_reject_log, 10, 1000)) {
			ESP_LOGW(TAG, "receive_isr called with no frame (rx_frame=NULL)");
		}
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

	ESP_LOGI(TAG, "configure: bitrate=%lu sample_point=%u permille, mode=%s%s, "
		 "timing calc -> %d", (unsigned long)config->bit_timing.bitrate,
		 (unsigned)config->bit_timing.sp_permill,
		 config->flags.enable_listen_only ? "listen-only" : "normal",
		 config->flags.enable_loopback ? "+loopback" : "", ret);
	if (ret >= 0) {
		ESP_LOGI(TAG, "configure: timing prescaler=%u phase_seg1=%u phase_seg2=%u sjw=%u",
			 (unsigned)timing.prescaler, (unsigned)timing.phase_seg1,
			 (unsigned)timing.phase_seg2, (unsigned)timing.sjw);
		ret = can_set_timing(&node->dev, &timing);
		ESP_LOGI(TAG, "configure: can_set_timing -> %d", ret);
	}
	if (ret >= 0) {
		ret = can_set_mode(&node->dev, mode);
		ESP_LOGI(TAG, "configure: can_set_mode(0x%x) -> %d (next driver mode=%u)",
			 (unsigned)mode, ret, (unsigned)node->data.next_mcp251xfd_mode);
	}
	ESP_RETURN_ON_ERROR(esp_err_from(ret), TAG, "bit timing/mode setup failed: %d", ret);
	node->listen_only = config->flags.enable_listen_only;
	return accept_all(node);
}

static esp_err_t create(spi_host_device_t host, const twai_mcp251xfd_node_config_t *config)
{
	mcp251xfd_node_t *node = heap_caps_calloc(1, sizeof(*node), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
	ESP_RETURN_ON_FALSE(node, ESP_ERR_NO_MEM, TAG, "no memory");

	ESP_LOGI(TAG, "create: SPI%d clock=%lu Hz osc=%lu Hz int=GPIO%d cs=GPIO%d "
		 "(node=%p, sizeof=%u)", host, (unsigned long)config->spi_clock_hz,
		 (unsigned long)config->oscillator_hz, config->io_cfg.int_gpio,
		 config->io_cfg.cs_gpio, node, (unsigned)sizeof(*node));

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
			ESP_LOGE(TAG, "create: spi_device_acquire_bus failed: %s",
				 esp_err_to_name(err));
			spi_bus_remove_device(node->spi);
		}
	}
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "create: SPI device setup failed: %s", esp_err_to_name(err));
		free(node);
		return err;
	}
	ESP_LOGI(TAG, "create: SPI device added and bus acquired");

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
		ESP_LOGE(TAG, "driver initialization failed: %d; node unavailable until restart",
			 ret);
		node->init_error = esp_err_from(ret);
	} else {
		ESP_LOGI(TAG, "create: driver init -> %d, INT thread %s (handle=%p)",
			 ret, node->data.int_thread.handle ? "started" : "MISSING",
			 node->data.int_thread.handle);
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
		ESP_LOGI(TAG, "create: direct SPI (LL) mode enabled, state change callback set");
	}
	s_node = node;
	return ESP_OK;
}

esp_err_t twai_new_node_mcp251xfd(spi_host_device_t host, const twai_mcp251xfd_node_config_t *config,
				  twai_node_handle_t *node_ret)
{
	ESP_RETURN_ON_FALSE(config && node_ret, ESP_ERR_INVALID_ARG, TAG, "null argument");
	ESP_LOGI(TAG, "twai_new_node: request (node=%p, in_use=%d, init_error=0x%x, sleeping=%d)",
		 s_node, s_node ? s_node->in_use : 0,
		 (unsigned)(s_node ? s_node->init_error : 0), s_controller_sleeping);
	if (!s_node) {
		ESP_RETURN_ON_ERROR(create(host, config), TAG, "create failed");
	}
	ESP_RETURN_ON_FALSE(!s_node->in_use, ESP_ERR_INVALID_STATE, TAG, "node already in use");
	ESP_RETURN_ON_ERROR(s_node->init_error, TAG, "controller unavailable until restart");
	// Enabled only here, not by controller_reset(): controller_sleep() takes an
	// enabled interrupt to mean that the driver's thread is idle.
	int ret = controller_reset(s_node);
	if (ret == 0) {
		// controller_sleep() leaves the pin disabled and marks the driver so
		// the INT thread won't re-enable it; a reused node must clear that
		// before the interrupt is enabled again.
		s_node->data.int_suspended = false;
		ret = gpio_pin_interrupt_configure_dt(&s_node->config.int_gpio_dt, GPIO_INT_LEVEL_ACTIVE);
		ESP_LOGI(TAG, "twai_new_node: INT GPIO%d interrupt -> %d",
			 s_node->config.int_gpio_dt.pin, ret);
	}
	ESP_RETURN_ON_FALSE(ret >= 0, esp_err_from(ret), TAG, "controller reset failed: %d", ret);
	esp_err_t cfg_err = configure(s_node, config);
	ESP_RETURN_ON_ERROR(cfg_err, TAG, "configure failed");
	{
		int64_t t_dump = esp_timer_get_time();
		k_mutex_lock(&s_node->data.mutex, K_FOREVER);
		mcp251xfd_log_registers(s_node, "configured");
		k_mutex_unlock(&s_node->data.mutex);
		ESP_LOGI(TAG, "twai_new_node: register dump after configure took %lld ms, "
			 "node ready", (long long)((esp_timer_get_time() - t_dump) / 1000));
	}
	s_node->in_use = true;
	*node_ret = &s_node->base;
	return ESP_OK;
}

// ---------------------------------------------------------------------------
// Bring-up diagnostics, exposed to can.c. task-context only.
// ---------------------------------------------------------------------------
bool twai_mcp251xfd_is_sleeping(void)
{
	return s_controller_sleeping;
}

void twai_mcp251xfd_dump_state(const char *why)
{
	mcp251xfd_node_t *node = s_node;
	const char *reason = why ? why : "(none)";

	if (!node) {
		ESP_LOGI(TAG, "[%s] adapter: no node ever created; the MCP2518FD has never been "
			 "initialised and is in its power-on state (it can be awake)", reason);
		return;
	}

	ESP_LOGI(TAG, "[%s] adapter: in_use=%d sleeping=%d listen_only=%d twai_state=%s "
			 "driver_state=%s started=%d current_mode=%u next_mode=%u "
			 "filters=(%d,%d) mailbox_usage=0x%08lx filter_usage=0x%08lx",
			 reason, node->in_use, s_controller_sleeping, (int)node->listen_only,
			 twai_state_name(node->state), mcp251xfd_state_name(node->data.state),
			 node->data.common.started, (unsigned)node->data.current_mcp251xfd_mode,
			 (unsigned)node->data.next_mcp251xfd_mode, node->filters[0], node->filters[1],
			 (unsigned long)node->data.mailbox_usage,
			 (unsigned long)node->data.filter_usage);

	if (node->data.int_thread.handle) {
		// ESP-IDF reports this in bytes (FreeRTOS stack sizes are bytes here).
		ESP_LOGI(TAG, "[%s] adapter: INT thread stack high-water=%u bytes",
			 reason, (unsigned)uxTaskGetStackHighWaterMark(node->data.int_thread.handle));
	} else {
		ESP_LOGI(TAG, "[%s] adapter: INT thread not started", reason);
	}

	// Check in_use/started under the driver mutex: node_delete() sets
	// in_use=false before controller_sleep(), so an SPI read here cannot race
	// the controller into LPM and wake it again.
	k_mutex_lock(&node->data.mutex, K_FOREVER);
	if (!node->in_use || !node->data.common.started) {
		k_mutex_unlock(&node->data.mutex);
		ESP_LOGI(TAG, "[%s] adapter: skipping register reads (in_use=%d started=%d); "
			 "SPI access would wake a stopped/asleep controller",
			 reason, node->in_use, node->data.common.started);
		return;
	}
	mcp251xfd_log_registers(node, reason);
	k_mutex_unlock(&node->data.mutex);
}
