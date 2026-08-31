#ifndef __CAR_SETTINGS_H__
#define __CAR_SETTINGS_H__

#include <stdbool.h>
#include <stdint.h>
#include "can.h"

#define CHARGE_LIMIT_MIN 50
#define CHARGE_LIMIT_MAX 100
#define CHARGE_LIMIT_DEFAULT 100

// 0x4C5 carries charge limit as percent * 2 (DBC factor 0.5: 50% -> 0x64,
// 100% -> 0xC8). These helpers centralize that mapping.
uint8_t charge_limit_percent_to_raw(uint8_t percent);
uint8_t charge_limit_raw_to_percent(uint8_t raw);

void car_settings_init(void);
void car_settings_tick(void);
void car_settings_bus_up(void);
// Arm a one-shot startup probe: 3 passive (all 0xFF) 0x4C5 frames, emitted
// one per tick once the bus is up. The car's 0x1F9 reply seeds the status
// display via car_settings_can_rx_hook.
void car_settings_probe_status(void);
bool charge_limit_set(uint8_t ac_percent, uint8_t dc_percent);
void charge_limit_get(uint8_t *ac_percent, uint8_t *dc_percent);
// Car-reported (0x1F9) limits in percent; only meaningful once a reply has
// been seen (see charge_limit_get_last_reply for validity/age).
void charge_limit_get_actuals(uint8_t *ac_percent, uint8_t *dc_percent);
bool charge_limit_get_last_reply(uint8_t *ac_raw, uint8_t *dc_raw, int64_t *age_us);
void car_settings_can_rx_hook(twai_message_t *to_push, can_bus_t rx_bus);

#endif
