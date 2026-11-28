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

// SoC is transmitted in half-percent steps on the 0x2FC frame (Ioniq 5/EV6) and
// the 29-bit 0x0BC2FA02 frame (Ioniq 6).
#define BATTERY_SOC_SCALE 0.5f

typedef struct {
    // raw byte from the frame; percent is raw * BATTERY_SOC_SCALE
    uint8_t raw;
    int64_t updated_at_us;
} precondition_soc_t;

bool precondition_get_battery_soc(precondition_soc_t *out);

// Charging power, measured: 0x3B7 (OBC_02_200ms_2) byte 3, 0.1 kW per count.
// Verified against three captures whose real draw differs by 6.9x: 0x60 = 9.6 kW
//
// 0x3B7 is transmitted only while a session is running - it is absent when the
// car is plugged in and idle, and from every driving capture - so an old sample
// means charging stopped, not that a frame was missed. Callers should treat a
// sample older than CHARGING_POWER_STALE_US as invalid.
#define CHARGING_POWER_STALE_US (5 * 1000 * 1000LL)

typedef struct {
    uint16_t deci_kw;       // 0.1 kW per count; weird but nice to keep it uint
    int64_t updated_at_us;
} precondition_charging_power_t;

bool precondition_get_charging_power(precondition_charging_power_t *out);

// E-GMP model identified from the precondition status frame. The web UI's
// button filter uses it when the vehicle setting is "auto"; an Ioniq 6 also gets
// a longer start timeout and its own popup fallback media type.
typedef enum {
    EGMP_CAR_IONIQ5,
    EGMP_CAR_IONIQ6,
    EGMP_CAR_EV6,
} egmp_car_model_t;

bool precondition_get_car_model(egmp_car_model_t *out);

// Is the car currently in READY? Tracks the power-status edge (0x038, or the
// 29-bit 0x00C03503 on Ioniq 6) the same way the state machine does; false until
// such a frame is seen. The negation is the persistent state behind the
// EV_CAR_NOT_READY event.
bool car_in_ready(void);

// True when the car is in READY or a higher power state: byte 0's low nibble
// (the DBC's Power_Status_Ready signal) is >= 0x04, i.e. READY, an active
// charging session, or the remote-wake transient. False until a power-status
// frame is seen. Car-off sleep uses this instead of car_in_ready() so a
// charging or remotely woken car is not mistaken for a sleeping one.
bool car_in_ready_or_above(void);

// Decoded 0x038/0x00C03503 power state. 0x00/0x01/0x02/0x04/0x06 are the DBC's
// Power_Status_Ready codes. 0x06 is split by byte 1 (Power_Status_Detail)
// because the nibble alone cannot tell an active charging session from a remote
// wake: 0x8D is Charging_Session_Active, 0xAB is the Boot_Wake_Transient.
// REMOTE_WAKE is a synthetic code for the latter pairing. Any other code is
// UNDOCUMENTED, including 0x06 with byte 1 0x00: 0x00 is now documented as
// "Idle", but the observed plugged-but-idle frame is 0x01 0x00, so the 0x06/0x00
// pairing is unevidenced.
typedef enum {
    CAR_POWER_STATE_UNKNOWN = -1,           // no power-status frame seen yet
    CAR_POWER_STATE_UNDOCUMENTED = -2,      // not a byte 0/byte 1 pair in the DBC
    CAR_POWER_STATE_OFF_ASLEEP = 0x00,      // 0 "Off_Asleep"
    CAR_POWER_STATE_OFF_AWAKE = 0x01,       // 1 "Off_Awake"
    CAR_POWER_STATE_ON_NOT_READY = 0x02,    // 2 "ON_Not_Ready"
    CAR_POWER_STATE_READY = 0x04,           // 4 "READY"
    CAR_POWER_STATE_CHARGING_SESSION = 0x06, // 6 with byte 1 0x8D "Charging_Session_Active"
    CAR_POWER_STATE_REMOTE_WAKE = 0x07,     // 6 with byte 1 0xAB "Boot_Wake_Transient"
} car_power_state_t;

// Latest decoded power state, for display. CAR_POWER_STATE_UNKNOWN until the
// first power-status frame; never blocks.
int car_power_state(void);

// Latest power-status decode, published on every frame rather than only on
// READY edges, so a reader can tell a live value from one left behind when the
// bus went quiet.
typedef struct {
    // byte 0's low nibble is 0x06 and byte 1 is 0x8D: a charging session is
    // active. 0x8D tracks the session, not the connector, so the cable can stay
    // plugged in with this false.
    bool charging_session;
    int64_t updated_at_us;
} precondition_power_t;

bool precondition_get_car_power(precondition_power_t *out);

#endif
