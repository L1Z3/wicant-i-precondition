#pragma once

// Stands in for Zephyr's generated syscall header. Without user mode, each
// __syscall in can.h is a static inline that forwards to its z_impl_
// function, as here. Every __syscall needs one: a missing forwarder shows up
// as a "declared 'static' but never defined" warning. Keep in step with
// can.h when upgrading Zephyr (zephyr/README.md).

// Implemented in can_common.c.
int z_impl_can_calc_timing(const struct device *dev, struct can_timing *res, uint32_t bitrate,
			   uint16_t sample_pnt);
int z_impl_can_set_timing(const struct device *dev, const struct can_timing *timing);
int z_impl_can_set_bitrate(const struct device *dev, uint32_t bitrate);
int z_impl_can_send(const struct device *dev, const struct can_frame *frame, k_timeout_t timeout,
		    can_tx_callback_t callback, void *user_data);
int z_impl_can_add_rx_filter_msgq(const struct device *dev, struct k_msgq *msgq,
				  const struct can_filter *filter);

static inline int can_calc_timing(const struct device *dev, struct can_timing *res,
				  uint32_t bitrate, uint16_t sample_pnt)
{
	return z_impl_can_calc_timing(dev, res, bitrate, sample_pnt);
}

static inline int can_set_timing(const struct device *dev, const struct can_timing *timing)
{
	return z_impl_can_set_timing(dev, timing);
}

static inline int can_set_bitrate(const struct device *dev, uint32_t bitrate)
{
	return z_impl_can_set_bitrate(dev, bitrate);
}

static inline int can_send(const struct device *dev, const struct can_frame *frame,
			   k_timeout_t timeout, can_tx_callback_t callback, void *user_data)
{
	return z_impl_can_send(dev, frame, timeout, callback, user_data);
}

static inline int can_add_rx_filter_msgq(const struct device *dev, struct k_msgq *msgq,
					 const struct can_filter *filter)
{
	return z_impl_can_add_rx_filter_msgq(dev, msgq, filter);
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

static inline int can_get_capabilities(const struct device *dev, can_mode_t *cap)
{
	return z_impl_can_get_capabilities(dev, cap);
}

static inline const struct device *can_get_transceiver(const struct device *dev)
{
	return z_impl_can_get_transceiver(dev);
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

static inline can_mode_t can_get_mode(const struct device *dev)
{
	return z_impl_can_get_mode(dev);
}

static inline void can_remove_rx_filter(const struct device *dev, int filter_id)
{
	z_impl_can_remove_rx_filter(dev, filter_id);
}

static inline int can_get_max_filters(const struct device *dev, bool ide)
{
	return z_impl_can_get_max_filters(dev, ide);
}

static inline int can_get_state(const struct device *dev, enum can_state *state,
				struct can_bus_err_cnt *err_cnt)
{
	return z_impl_can_get_state(dev, state, err_cnt);
}

// Implemented only with CONFIG_CAN_FD_MODE, CONFIG_CAN_STATS or
// CONFIG_CAN_MANUAL_RECOVERY_MODE, none of which is set here (compat/autoconf.h),
// so calling one of these fails to link. CONFIG_CAN_STATS would also need a
// zephyr/stats/stats.h shim.
const struct can_timing *z_impl_can_get_timing_data_min(const struct device *dev);
const struct can_timing *z_impl_can_get_timing_data_max(const struct device *dev);
int z_impl_can_calc_timing_data(const struct device *dev, struct can_timing *res, uint32_t bitrate,
				uint16_t sample_pnt);
int z_impl_can_set_timing_data(const struct device *dev, const struct can_timing *timing_data);
int z_impl_can_set_bitrate_data(const struct device *dev, uint32_t bitrate_data);
int z_impl_can_recover(const struct device *dev, k_timeout_t timeout);
uint32_t z_impl_can_stats_get_bit_errors(const struct device *dev);
uint32_t z_impl_can_stats_get_bit0_errors(const struct device *dev);
uint32_t z_impl_can_stats_get_bit1_errors(const struct device *dev);
uint32_t z_impl_can_stats_get_stuff_errors(const struct device *dev);
uint32_t z_impl_can_stats_get_crc_errors(const struct device *dev);
uint32_t z_impl_can_stats_get_form_errors(const struct device *dev);
uint32_t z_impl_can_stats_get_ack_errors(const struct device *dev);
uint32_t z_impl_can_stats_get_rx_overruns(const struct device *dev);

static inline const struct can_timing *can_get_timing_data_min(const struct device *dev)
{
	return z_impl_can_get_timing_data_min(dev);
}

static inline const struct can_timing *can_get_timing_data_max(const struct device *dev)
{
	return z_impl_can_get_timing_data_max(dev);
}

static inline int can_calc_timing_data(const struct device *dev, struct can_timing *res,
				       uint32_t bitrate, uint16_t sample_pnt)
{
	return z_impl_can_calc_timing_data(dev, res, bitrate, sample_pnt);
}

static inline int can_set_timing_data(const struct device *dev,
				      const struct can_timing *timing_data)
{
	return z_impl_can_set_timing_data(dev, timing_data);
}

static inline int can_set_bitrate_data(const struct device *dev, uint32_t bitrate_data)
{
	return z_impl_can_set_bitrate_data(dev, bitrate_data);
}

static inline int can_recover(const struct device *dev, k_timeout_t timeout)
{
	return z_impl_can_recover(dev, timeout);
}

static inline uint32_t can_stats_get_bit_errors(const struct device *dev)
{
	return z_impl_can_stats_get_bit_errors(dev);
}

static inline uint32_t can_stats_get_bit0_errors(const struct device *dev)
{
	return z_impl_can_stats_get_bit0_errors(dev);
}

static inline uint32_t can_stats_get_bit1_errors(const struct device *dev)
{
	return z_impl_can_stats_get_bit1_errors(dev);
}

static inline uint32_t can_stats_get_stuff_errors(const struct device *dev)
{
	return z_impl_can_stats_get_stuff_errors(dev);
}

static inline uint32_t can_stats_get_crc_errors(const struct device *dev)
{
	return z_impl_can_stats_get_crc_errors(dev);
}

static inline uint32_t can_stats_get_form_errors(const struct device *dev)
{
	return z_impl_can_stats_get_form_errors(dev);
}

static inline uint32_t can_stats_get_ack_errors(const struct device *dev)
{
	return z_impl_can_stats_get_ack_errors(dev);
}

static inline uint32_t can_stats_get_rx_overruns(const struct device *dev)
{
	return z_impl_can_stats_get_rx_overruns(dev);
}
