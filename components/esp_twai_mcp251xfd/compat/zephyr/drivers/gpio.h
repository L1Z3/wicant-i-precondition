#pragma once

#include <errno.h>
#include "driver/gpio.h"
#include <zephyr/device.h>
#include <zephyr/sys/util.h>

// Zephyr GPIO API for the driver's INT pin, on the ESP-IDF GPIO driver.
// Flag values are private to this shim.
typedef uint32_t gpio_flags_t;
typedef uint16_t gpio_dt_flags_t;
typedef uint32_t gpio_port_pins_t;

#define GPIO_ACTIVE_LOW       BIT(0)
#define GPIO_PULL_UP          BIT(1)
#define GPIO_INPUT            BIT(2)
#define GPIO_INT_DISABLE      BIT(3)
#define GPIO_INT_LEVEL_ACTIVE BIT(4)

struct gpio_dt_spec {
	const struct device *port;
	gpio_num_t pin;
	gpio_dt_flags_t dt_flags;
};

struct gpio_callback;
typedef void (*gpio_callback_handler_t)(const struct device *port, struct gpio_callback *cb,
					gpio_port_pins_t pins);

struct gpio_callback {
	gpio_callback_handler_t handler;
	gpio_port_pins_t pin_mask;
};

static inline bool gpio_is_ready_dt(const struct gpio_dt_spec *spec)
{
	return GPIO_IS_VALID_GPIO(spec->pin);
}

static inline int gpio_pin_configure_dt(const struct gpio_dt_spec *spec, gpio_flags_t extra_flags)
{
	gpio_flags_t flags = spec->dt_flags | extra_flags;
	gpio_config_t config = {
		.pin_bit_mask = BIT64(spec->pin),
		.mode = (flags & GPIO_INPUT) ? GPIO_MODE_INPUT : GPIO_MODE_DISABLE,
		.pull_up_en = (flags & GPIO_PULL_UP) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};

	return gpio_config(&config) == ESP_OK ? 0 : -EIO;
}

static inline void gpio_init_callback(struct gpio_callback *cb, gpio_callback_handler_t handler,
				      gpio_port_pins_t pin_mask)
{
	cb->handler = handler;
	cb->pin_mask = pin_mask;
}

static inline void z_gpio_isr(void *arg)
{
	struct gpio_callback *cb = arg;

	cb->handler(NULL, cb, cb->pin_mask);
}

// The caller must have installed the GPIO ISR service.
static inline int gpio_add_callback_dt(const struct gpio_dt_spec *spec, struct gpio_callback *cb)
{
	return gpio_isr_handler_add(spec->pin, z_gpio_isr, cb) == ESP_OK ? 0 : -EIO;
}

// Called from the ISR (disable) and the driver thread (re-enable).
static inline int gpio_pin_interrupt_configure_dt(const struct gpio_dt_spec *spec,
						  gpio_flags_t flags)
{
	if (flags & GPIO_INT_LEVEL_ACTIVE) {
		gpio_int_type_t type = (spec->dt_flags & GPIO_ACTIVE_LOW) ? GPIO_INTR_LOW_LEVEL
									  : GPIO_INTR_HIGH_LEVEL;
		if (gpio_set_intr_type(spec->pin, type) != ESP_OK) {
			return -EIO;
		}
		return gpio_intr_enable(spec->pin) == ESP_OK ? 0 : -EIO;
	}
	return gpio_intr_disable(spec->pin) == ESP_OK ? 0 : -EIO;
}

// Logical level: 1 when the pin is active.
static inline int gpio_pin_get_dt(const struct gpio_dt_spec *spec)
{
	int level = gpio_get_level(spec->pin);

	return (spec->dt_flags & GPIO_ACTIVE_LOW) ? !level : level;
}
