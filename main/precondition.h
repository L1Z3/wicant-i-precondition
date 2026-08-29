#ifndef __PRECONDITION_H__
#define __PRECONDITION_H__

#include <stdbool.h>
#include <stdint.h>
#include "hsm.h"

void precondition_init(void);
void precondition_can_rx_hook(twai_message_t *to_push, can_bus_t rx_bus);
fwd_result_t precondition_fwd_hook(twai_message_t *to_send, can_bus_t fwd_bus);
void precondition_tick(void);

typedef struct {
    int8_t min_c;
    int8_t max_c;
    int64_t updated_at_us;
} precondition_temperature_t;

bool precondition_get_battery_temperature(precondition_temperature_t *out);

typedef struct {
    bool requested;
    bool active;
    bool starting;
} precondition_state_t;

void precondition_toggle_request(void);
bool precondition_get_state(precondition_state_t *out);

// SoC on 0x2FC frame is transmitted in half-percent steps.
#define BATTERY_SOC_SCALE 0.5f

typedef struct {
    // raw byte from the frame; percent is raw * BATTERY_SOC_SCALE
    uint8_t raw;
    int64_t updated_at_us;
} precondition_soc_t;

bool precondition_get_battery_soc(precondition_soc_t *out);

// E-GMP model identified from the precondition status frame. Only the web UI's
// button filter uses it, when the vehicle setting is "auto".
typedef enum {
    EGMP_CAR_IONIQ5,
    EGMP_CAR_IONIQ6,
    EGMP_CAR_EV6,
} egmp_car_model_t;

bool precondition_get_car_model(egmp_car_model_t *out);

// Is the car currently in READY? Tracks the 0x038 power-status edge the same
// way the state machine does; false until a 0x038 frame is seen. The negation
// is the persistent state behind the EV_CAR_NOT_READY event.
bool car_in_ready(void);

// Car power state, from the low nibble of byte 0 of frame 0x038.
#define CAR_POWER_OFF 0x01U      // awake but not in READY
#define CAR_POWER_READY 0x04U
// Observed steady while charging with the ignition off on a 2022 EV6. Appears
// only as a rare transient otherwise: 12 frames out of 10225 across every
// recorded non-charging log in tylerharvey/Ioniq5_CAN.
#define CAR_POWER_CHARGING 0x06U

typedef struct {
    // true while the low nibble reads CAR_POWER_READY
    bool ready;
    // true while the low nibble reads CAR_POWER_CHARGING
    bool charging;
    // raw byte 0, so callers can see states not named here
    uint8_t raw;
    // updated on every 0x038 frame, not just on edges, so callers can tell a
    // live reading from one left behind by a bus that went quiet
    int64_t updated_at_us;
} precondition_power_t;

bool precondition_get_car_power(precondition_power_t *out);

#endif
