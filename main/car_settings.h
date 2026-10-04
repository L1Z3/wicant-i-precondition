#ifndef __CAR_SETTINGS_H__
#define __CAR_SETTINGS_H__

#include <stdbool.h>
#include <stdint.h>
#include "can.h"

#define CHARGE_LIMIT_MIN 50
#define CHARGE_LIMIT_MAX 100
#define CHARGE_LIMIT_DEFAULT 100

// A charge-limit AC/DC pair. Fields are percents (CHARGE_LIMIT_MIN ..
// CHARGE_LIMIT_MAX) unless a name says *_raw.
typedef struct {
    uint8_t ac_percent;
    uint8_t dc_percent;
} charge_limit_pair_t;

// Last 0x1F9 reply from the car: raw bus bytes (percent * 2) and how long ago
// it arrived. See charge_limit_get_reply().
typedef struct {
    uint8_t ac_raw;
    uint8_t dc_raw;
    int64_t age_us;
} charge_limit_reply_t;

// 0x4C5 carries charge limit as percent * 2 (DBC factor 0.5: 50% -> 0x64,
// 100% -> 0xC8). These helpers centralize that mapping.
uint8_t charge_limit_percent_to_raw(uint8_t percent);
uint8_t charge_limit_raw_to_percent(uint8_t raw);

// car_settings_init() must run before any other entry point here (it creates
// the mutex and loads the configured target).
void car_settings_init(void);
void car_settings_tick(void);
void car_settings_bus_up(void);
// Arm a one-shot startup probe: 3 passive (all 0xFF) 0x4C5 frames, emitted
// one per tick once the bus is up. The car's 0x1F9 reply seeds the status
// display via car_settings_can_rx_hook.
void car_settings_probe_status(void);

// Configured target: what we want the car's charge limits to be. Sets the RAM
// copy only; the caller commits with config_server_save_cfg(). Rejects
// out-of-range percents (returns false).
bool charge_limit_set_target(charge_limit_pair_t target);
charge_limit_pair_t charge_limit_get_target(void);

// Car-reported (0x1F9) limits in percent; only meaningful once a reply has
// been seen (see charge_limit_get_reply for validity/age).
charge_limit_pair_t charge_limit_get_reported(void);

// Last 0x1F9 reply (raw bytes + age). Returns true while a reply has been seen
// and is still fresh; *out is filled either way.
bool charge_limit_get_reply(charge_limit_reply_t *out);

void car_settings_can_rx_hook(twai_message_t *to_push, can_bus_t rx_bus);

#endif
