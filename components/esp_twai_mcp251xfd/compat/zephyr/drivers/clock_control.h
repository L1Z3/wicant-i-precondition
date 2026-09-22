#pragma once

#include <errno.h>
#include <zephyr/device.h>

// No clock controller: the MCP2518FD runs from its own crystal.
typedef void *clock_control_subsys_t;

static inline int clock_control_on(const struct device *dev, clock_control_subsys_t sys)
{
	(void)dev;
	(void)sys;
	return -ENOSYS;
}
