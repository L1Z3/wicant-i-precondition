#pragma once

#include <errno.h>
#include <string.h>
#include "driver/spi_master.h"
#include "hal/spi_ll.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "log_flash.h"
#include <zephyr/device.h>

// Bring-up diagnostics for the direct (LL) SPI path. The spin limit is a
// last-resort hang detector: a normal 18-byte transfer at 16 MHz completes in
// ~9 us, so it should never be reached.
#define SPI_LL_TIMEOUT_SPINS 1000000U
#define SPI_LL_SLOW_US       200
#define SPI_LL_STATS_US      (10LL * 1000 * 1000)
#define SPI_LL_TAG           "mcp251xfd_spi"

// Zephyr SPI API on an ESP-IDF SPI device that the adapter adds and reserves.
struct spi_dt_spec {
	const struct device *bus;
	spi_device_handle_t device;
	// Set by the adapter once ESP-IDF has configured the bus for this device;
	// transfers then drive the peripheral directly (see spi_transfer_ll()).
	spi_dev_t *hw;
};

struct spi_buf {
	void *buf;
	size_t len;
};

struct spi_buf_set {
	const struct spi_buf *buffers;
	size_t count;
};

static inline bool spi_is_ready_dt(const struct spi_dt_spec *spec)
{
	return spec->device != NULL;
}

// ESP-IDF's polling transfer recomputes the whole transfer setup and checks the
// bus lock every time, which costs more than clocking the bytes. The device
// holds the bus for itself, and every transfer here has the same shape
// (full duplex, no command/address/dummy phases), so after the driver's first
// transfers only the length and data change. Transfers must fit the
// peripheral's 64-byte buffer.
//
// The adapter acquires the bus for good (spi_device_acquire_bus()), which
// also holds the bus's ESP_PM_APB_FREQ_MAX power-management lock whenever
// CONFIG_PM_ENABLE is set (it isn't in this firmware). That lock is what
// keeps this path safe under power management: the clock divider set up for
// the device assumes the APB clock doesn't change. It also keeps APB at
// 80 MHz and prevents automatic light sleep from the first node creation
// until restart, as the node is never freed; an explicit
// esp_light_sleep_start() ignores it. Don't release the bus to save power.
//
// Returns 0, or -EIO/-EINVAL; the statistics and the slow/stuck logs are
// kept here so a transfer that hangs is reported with its length and the
// flash-log marker.
static inline int spi_transfer_ll(spi_dev_t *hw, const void *tx, void *rx, size_t len)
{
	// Statistics; the header is only included from the adapter's TU.
	static uint32_t s_xfers;
	static uint32_t s_timeouts;
	static uint32_t s_max_us;
	static uint64_t s_total_us;
	static int64_t s_stats_us;
	static int64_t s_slow_log_us;
	size_t bits = len * 8;
	int64_t t0 = esp_timer_get_time();
	uint32_t spins = 0;
	// Snapshot the file-log flash marker before the transfer. The LL loop
	// runs from flash; if a littlefs write/erase starts while it polls, the
	// CPU cannot execute the next instruction until the cache is back, and
	// the sequence counter changes even though active is false again by the
	// time we log.
	uint32_t log_flash_seq0 = log_flash_seq();
	bool log_flash_active0 = log_flash_active();

	if (len > SOC_SPI_MAXIMUM_BUFFER_SIZE) {
		ESP_LOGE(SPI_LL_TAG, "transfer too long for the LL path: %u bytes", (unsigned)len);
		return -EINVAL;
	}

	// The IDF path leaves its DMA channels enabled on the peripheral, and with
	// either DMA bit set the peripheral clocks from the DMA descriptor chain
	// instead of the CPU buffer. Keep this path CPU-driven so a long transfer
	// followed by a short one cannot send garbage.
	spi_ll_dma_tx_enable(hw, false);
	spi_ll_dma_rx_enable(hw, false);

	spi_ll_clear_int_stat(hw);
	spi_ll_set_mosi_bitlen(hw, bits);
	spi_ll_set_miso_bitlen(hw, bits);
	spi_ll_enable_mosi(hw, 1);
	spi_ll_enable_miso(hw, 1);
	spi_ll_write_buffer(hw, tx, bits);
	spi_ll_apply_config(hw);
	spi_ll_user_start(hw);
	while (!spi_ll_usr_is_done(hw)) {
		if (++spins >= SPI_LL_TIMEOUT_SPINS) {
			s_timeouts++;
			ESP_LOGE(SPI_LL_TAG, "LL transfer stuck: len=%u bits=%u spins=%lu "
				 "cmd_usr=%lu done=%d timeouts=%lu log_flash=%d seq=%lu (SPI "
				 "clock/CS/MISO or cache stall?)",
				 (unsigned)len, (unsigned)bits, (unsigned long)spins,
				 (unsigned long)spi_ll_get_running_cmd(hw),
				 spi_ll_usr_is_done(hw), (unsigned long)s_timeouts,
				 log_flash_active(), (unsigned long)log_flash_seq());
			return -EIO;
		}
	}
	if (rx) {
		spi_ll_read_buffer(hw, rx, bits);
	}

	uint32_t log_flash_seq1 = log_flash_seq();
	bool log_flash_active1 = log_flash_active();
	bool log_flash_during = log_flash_active0 || log_flash_active1 ||
				 log_flash_seq1 != log_flash_seq0;

	int64_t dt = esp_timer_get_time() - t0;
	s_xfers++;
	s_total_us += (uint64_t)dt;
	if ((uint32_t)dt > s_max_us) {
		s_max_us = (uint32_t)dt;
	}
	if (dt > SPI_LL_SLOW_US) {
		int64_t now_us = esp_timer_get_time();
		if (now_us - s_slow_log_us > 1000000LL) {
			s_slow_log_us = now_us;
			ESP_LOGW(SPI_LL_TAG, "slow LL transfer: %lld us (len=%u bits=%u, "
				 "max=%lu us, avg=%lu us over %lu transfers, log_flash=%d "
				 "act=%d/%d seq=%lu->%lu)",
				 (long long)dt, (unsigned)len, (unsigned)bits,
				 (unsigned long)s_max_us,
				 (unsigned long)(s_xfers ? s_total_us / s_xfers : 0),
				 (unsigned long)s_xfers, log_flash_during,
				 log_flash_active0, log_flash_active1,
				 (unsigned long)log_flash_seq0, (unsigned long)log_flash_seq1);
		}
	}
	int64_t now_us = esp_timer_get_time();
	if (s_xfers != 0 && (now_us - s_stats_us) >= SPI_LL_STATS_US) {
		s_stats_us = now_us;
		ESP_LOGI(SPI_LL_TAG, "LL stats: %lu transfers, avg=%lu us, max=%lu us, "
			 "timeouts=%lu",
			 (unsigned long)s_xfers, (unsigned long)(s_total_us / s_xfers),
			 (unsigned long)s_max_us, (unsigned long)s_timeouts);
		s_xfers = 0;
		s_total_us = 0;
		s_max_us = 0;
	}
	return 0;
}

// Largest transfer the DMA path accepts: one CAN FD message object (8 header +
// 64 payload bytes) plus its 2-byte command. Must match the bus's
// max_transfer_sz.
#define SPI_BOUNCE_LEN 128

// DMA transfers need 4-byte-aligned buffers, and the driver's CAN FD object
// buffer is a packed struct whose 2-byte command sits at a padded offset, so it
// is not aligned (and the same memory is used for TX and RX). Copy through an
// aligned scratch buffer. Only message objects reach this path: register
// accesses are short and stay on the LL path, so the copy is not hot.
static inline int spi_transceive_dma(const struct spi_dt_spec *spec, const void *tx,
				     void *rx, size_t len)
{
	static uint8_t bounce_tx[SPI_BOUNCE_LEN] __attribute__((aligned(4)));
	static uint8_t bounce_rx[SPI_BOUNCE_LEN] __attribute__((aligned(4)));

	if (len > SPI_BOUNCE_LEN) {
		ESP_LOGE(SPI_LL_TAG, "transfer too long: %u bytes", (unsigned)len);
		return -EINVAL;
	}
	memcpy(bounce_tx, tx, len);
	if (rx) {
		memset(bounce_rx, 0, len);
	}
	spi_transaction_t transaction = {
		.length = len * 8,
		.tx_buffer = bounce_tx,
		.rx_buffer = rx ? bounce_rx : NULL,
	};

	esp_err_t err = spi_device_polling_transmit(spec->device, &transaction);
	if (err != ESP_OK) {
		return -EIO;
	}
	if (rx) {
		memcpy(rx, bounce_rx, len);
	}
	return 0;
}

// The driver only issues single-buffer full-duplex transfers, with TX and RX
// sharing memory. Transfers without DMA copy through the peripheral's buffer,
// which allows that and limits a transfer to 64 bytes; anything longer needs
// DMA (CAN FD message objects) and goes through spi_transceive_dma().
static inline int spi_transceive_dt(const struct spi_dt_spec *spec, const struct spi_buf_set *tx,
				    const struct spi_buf_set *rx)
{
	if (tx->count != 1 || (rx && (rx->count != 1 || rx->buffers[0].len != tx->buffers[0].len))) {
		return -ENOTSUP;
	}
	size_t len = tx->buffers[0].len;
	void *rx_buf = rx ? rx->buffers[0].buf : NULL;

	if (spec->hw && len <= SOC_SPI_MAXIMUM_BUFFER_SIZE) {
		return spi_transfer_ll(spec->hw, tx->buffers[0].buf, rx_buf, len);
	}

	return spi_transceive_dma(spec, tx->buffers[0].buf, rx_buf, len);
}

static inline int spi_write_dt(const struct spi_dt_spec *spec, const struct spi_buf_set *tx)
{
	return spi_transceive_dt(spec, tx, NULL);
}
