#pragma once

#include <zephyr/drivers/can.h>

// No PHY device is configured: WiCAN drives the transceiver standby pins.
static inline int can_transceiver_enable(const struct device *dev, can_mode_t mode)
{
	(void)dev;
	(void)mode;
	return -ENOSYS;
}

static inline int can_transceiver_disable(const struct device *dev)
{
	(void)dev;
	return -ENOSYS;
}
