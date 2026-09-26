#pragma once

#include <errno.h>
#include "driver/spi_master.h"
#include <zephyr/device.h>

// Zephyr SPI API on an ESP-IDF SPI device that the adapter adds and reserves.
struct spi_dt_spec {
	const struct device *bus;
	spi_device_handle_t device;
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

// The driver only issues single-buffer full-duplex transfers, with TX and RX
// sharing memory. Polling transfers without DMA copy through the peripheral's
// FIFO, which allows that and limits a transfer to 64 bytes.
static inline int spi_transceive_dt(const struct spi_dt_spec *spec, const struct spi_buf_set *tx,
				    const struct spi_buf_set *rx)
{
	if (tx->count != 1 || (rx && (rx->count != 1 || rx->buffers[0].len != tx->buffers[0].len))) {
		return -ENOTSUP;
	}
	spi_transaction_t transaction = {
		.length = tx->buffers[0].len * 8,
		.tx_buffer = tx->buffers[0].buf,
		.rx_buffer = rx ? rx->buffers[0].buf : NULL,
	};

	return spi_device_polling_transmit(spec->device, &transaction) == ESP_OK ? 0 : -EIO;
}

static inline int spi_write_dt(const struct spi_dt_spec *spec, const struct spi_buf_set *tx)
{
	return spi_transceive_dt(spec, tx, NULL);
}
