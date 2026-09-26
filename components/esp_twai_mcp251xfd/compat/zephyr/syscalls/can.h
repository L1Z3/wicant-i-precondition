#pragma once

// Stands in for Zephyr's generated syscall header. Without user mode, each
// __syscall in can.h forwards to its z_impl_ function. Only the calls this
// component makes are defined; using another one fails to compile.

// Implemented in can_common.c.
int z_impl_can_calc_timing(const struct device *dev, struct can_timing *res, uint32_t bitrate,
			   uint16_t sample_pnt);
int z_impl_can_set_timing(const struct device *dev, const struct can_timing *timing);
int z_impl_can_send(const struct device *dev, const struct can_frame *frame,
		    k_timeout_t timeout, can_tx_callback_t callback, void *user_data);

static inline int can_calc_timing(const struct device *dev, struct can_timing *res,
				  uint32_t bitrate, uint16_t sample_pnt)
{
	return z_impl_can_calc_timing(dev, res, bitrate, sample_pnt);
}

static inline int can_set_timing(const struct device *dev, const struct can_timing *timing)
{
	return z_impl_can_set_timing(dev, timing);
}

static inline int can_send(const struct device *dev, const struct can_frame *frame,
			   k_timeout_t timeout, can_tx_callback_t callback, void *user_data)
{
	return z_impl_can_send(dev, frame, timeout, callback, user_data);
}

// Implemented inline in can.h.
static inline int can_get_core_clock(const struct device *dev, uint32_t *rate)
{
	return z_impl_can_get_core_clock(dev, rate);
}

static inline uint32_t can_get_bitrate_min(const struct device *dev)
{
	return z_impl_can_get_bitrate_min(dev);
}

static inline uint32_t can_get_bitrate_max(const struct device *dev)
{
	return z_impl_can_get_bitrate_max(dev);
}

static inline const struct can_timing *can_get_timing_min(const struct device *dev)
{
	return z_impl_can_get_timing_min(dev);
}

static inline const struct can_timing *can_get_timing_max(const struct device *dev)
{
	return z_impl_can_get_timing_max(dev);
}

static inline int can_start(const struct device *dev)
{
	return z_impl_can_start(dev);
}

static inline int can_stop(const struct device *dev)
{
	return z_impl_can_stop(dev);
}

static inline int can_set_mode(const struct device *dev, can_mode_t mode)
{
	return z_impl_can_set_mode(dev, mode);
}

static inline void can_remove_rx_filter(const struct device *dev, int filter_id)
{
	z_impl_can_remove_rx_filter(dev, filter_id);
}
