#pragma once

#include <errno.h>
#include <string.h>
#include "driver/spi_master.h"
#include "hal/spi_ll.h"
#include "esp_log.h"
#include <zephyr/device.h>

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
static inline void spi_transfer_ll(spi_dev_t *hw, const void *tx, void *rx, size_t len)
{
	size_t bits = len * 8;

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
	}
	if (rx) {
		spi_ll_read_buffer(hw, rx, bits);
	}
}

// Largest transfer the DMA path accepts: one CAN FD message object (8 header +
// 64 payload bytes) plus its 2-byte command. Must match the bus's
// max_transfer_sz.
#define SPI_BOUNCE_LEN 128
#define SPI_DMA_TAG      "mcp251xfd_spi"

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
		ESP_LOGE(SPI_DMA_TAG, "transfer too long: %u bytes", (unsigned)len);
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
		spi_transfer_ll(spec->hw, tx->buffers[0].buf, rx_buf, len);
		return 0;
	}

	return spi_transceive_dma(spec, tx->buffers[0].buf, rx_buf, len);
}

static inline int spi_write_dt(const struct spi_dt_spec *spec, const struct spi_buf_set *tx)
{
	return spi_transceive_dt(spec, tx, NULL);
}
