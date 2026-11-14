#pragma once

#include <zephyr/types.h>

// The adapter builds the one device instance itself, so the driver's
// devicetree instantiation at the end of the file expands to nothing.
#define DT_INST_FOREACH_STATUS_OKAY(fn)

struct device {
	const char *name;
	const void *config;
	const void *api;
	void *data;
};

#define DEVICE_API(_class, _name) const struct _class##_driver_api _name
#define DEVICE_API_GET(_class, _dev) ((const struct _class##_driver_api *)(_dev)->api)

static inline bool device_is_ready(const struct device *dev)
{
	return dev != NULL;
}
