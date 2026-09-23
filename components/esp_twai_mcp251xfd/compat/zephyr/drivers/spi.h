#pragma once

#include <errno.h>
#include "driver/spi_master.h"
#include "hal/spi_ll.h"
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
static inline void spi_transfer_ll(spi_dev_t *hw, const void *tx, void *rx, size_t len)
{
	size_t bits = len * 8;

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

// The driver only issues single-buffer full-duplex transfers, with TX and RX
// sharing memory. Transfers without DMA copy through the peripheral's buffer,
// which allows that and limits a transfer to 64 bytes.
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
	spi_transaction_t transaction = {
		.length = len * 8,
		.tx_buffer = tx->buffers[0].buf,
		.rx_buffer = rx_buf,
	};

	return spi_device_polling_transmit(spec->device, &transaction) == ESP_OK ? 0 : -EIO;
}

static inline int spi_write_dt(const struct spi_dt_spec *spec, const struct spi_buf_set *tx)
{
	return spi_transceive_dt(spec, tx, NULL);
}
