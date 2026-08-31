#include "car_settings.h"
#include "config_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include "freertos/semphr.h"
// todo(TRH): fix population from defaults
// choose between refresh and slider onchange

#define TAG __func__

#define CAR_BUS CAN_BUS_0

#define CHARGE_LIMIT_FRAME_ID 0x4C5U

// Car-side charge-limit status echo (BO_ 505 Charge_Limit_Status_1F9 in
// ioniq5-2022.dbc, ~200 ms periodic): D3 (data[2]) = AC limit, D4 (data[3])
// = DC limit, same factor 0.5 as the command frame. Mirrors the 0x4C5
// limits while the car accepts them.
#define CHARGE_LIMIT_STATUS_FRAME_ID 0x1F9U

// Inject our limits when no 0x4C5 is seen on the bus for this long (one
// burst per quiet episode; re-arms once traffic reappears).
#define CAR_SETTINGS_INJECTION_TIMEOUT_US 10000000LL
// When a disagreeing 0x1F9 reply is fresher than this, hold off and try
// again on a later tick instead of answering immediately.
#define CAR_SETTINGS_CONFLICT_BACKOFF_US 100000LL
// Frames per injection phase (active burst and passive tail). The tick runs
// every 40 ms and sends one frame per tick while a phase is armed, giving
// 40 ms spacing; a single idle tick separates the phases.
#define CAR_SETTINGS_BURST_COUNT 3U

static SemaphoreHandle_t s_mutex = NULL;

static uint8_t s_ac_limit = CHARGE_LIMIT_DEFAULT;
static uint8_t s_dc_limit = CHARGE_LIMIT_DEFAULT;

// Last seen 0x4C5 payload (raw bytes D5/D6): drives the injection timing in
// car_settings_tick (quiet detection and conflict answers).
static uint8_t s_last_ac_raw = 0;
static uint8_t s_last_dc_raw = 0;
static int64_t s_last_seen_us = 0;
static bool s_last_seen_valid = false;

// Last 0x1F9 car-side reply for the status display (raw bytes D3/D4).
static uint8_t s_reply_ac_raw = 0;
static uint8_t s_reply_dc_raw = 0;
static int64_t s_reply_seen_us = 0;
static bool s_reply_seen_valid = false;

// Enforcement reference: car's reported limits in percent, fed by every
// 0x1F9 reply. Freshness tracked by s_reply_seen_* above; the tick answers
// disagreements with the configured target (s_ac_limit/s_dc_limit).
static uint8_t s_ac = 0;
static uint8_t s_dc = 0;

// Full last-seen frame as the template for active injection: the DBC only
// defines bytes 4/5, so replay the other bytes as observed rather than
// fabricating them.
static uint8_t s_last_dlc = 0;
static uint8_t s_last_data[8] = {0};
static bool s_has_template = false;

// Boot time, so the 10 s quiet timeout also applies before the first frame
// is ever seen (otherwise injection would fire immediately at startup).
static int64_t s_boot_us = 0;

// One-shot startup probe armed by car_settings_probe_status(): the tick
// turns it into a passive burst once idle, eliciting a 0x1F9 reply that
// seeds the status display.
static bool s_probe_pending = false;

// Tracks whether CAN bus is enabled; quiet timer only starts after bus-up.
static bool s_bus_up = false;

// Enforcement burst sequencing (one frame per tick, see car_settings_tick):
// active phase, one idle tick, passive tail. Managed entirely on the
// precondition task, so no mutex needed around these.
static uint8_t s_burst_remaining = 0;
static bool s_gap_tick = false;
static uint8_t s_passive_remaining = 0;
static int64_t s_answered_reply_us = 0;
static bool s_quiet_injected = false;

// Last disagreeing (actual,target) pair we fired a conflict burst for. A
// conflict is answered at most once per distinct pair, so a car that
// refuses to move (echo stays put) does not get burst at every ~200 ms reply
// forever. Firing again requires the car's reported value OR the configured
// target to change.
static uint8_t s_conflict_ac = 0;
static uint8_t s_conflict_dc = 0;
static uint8_t s_conflict_target_ac = CHARGE_LIMIT_DEFAULT;
static uint8_t s_conflict_target_dc = CHARGE_LIMIT_DEFAULT;

static bool is_valid_percent(uint8_t p) {
    return p >= CHARGE_LIMIT_MIN && p <= CHARGE_LIMIT_MAX;
}

uint8_t charge_limit_percent_to_raw(uint8_t percent) {
    return (uint8_t)(percent * 2U);
}

uint8_t charge_limit_raw_to_percent(uint8_t raw) {
    // 0xFF = "off" per the DBC; map it to 0 so it never reads as a real
    // limit. Any other out-of-range byte is clamped to the valid window.
    if (raw >= 0xFFU) {
        return 0;
    }
    return (uint8_t)(raw / 2U);
}

// A raw limit byte is a usable percent only within 50%..100% (0x64..0xC8).
// 0xFF = "off" and anything below 50% is not a configured limit; reject it
// so the status display and the conflict reference never see a bogus value.
static bool is_valid_reply_raw(uint8_t raw) {
    return raw >= CHARGE_LIMIT_MIN * 2U && raw <= CHARGE_LIMIT_MAX * 2U;
}

// config_server_get_charge_*_limit() already clamp to the valid range
void car_settings_init(void) {
    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateMutex();
        configASSERT(s_mutex != NULL);
    }
    s_boot_us = esp_timer_get_time();
    if (xSemaphoreTake(s_mutex, portMAX_DELAY)) {
        s_ac_limit = (uint8_t)config_server_get_charge_ac_limit();
        s_dc_limit = (uint8_t)config_server_get_charge_dc_limit();
        s_conflict_ac = s_ac;
        s_conflict_dc = s_dc;
        s_conflict_target_ac = s_ac_limit;
        s_conflict_target_dc = s_dc_limit;
        xSemaphoreGive(s_mutex);
    }
    ESP_LOGI(TAG, "charge limits init AC %u%% (0x%02X) DC %u%% (0x%02X)",
             s_ac_limit, charge_limit_percent_to_raw(s_ac_limit),
             s_dc_limit, charge_limit_percent_to_raw(s_dc_limit));
}

// Call once CAN is enabled; starts the quiet timer from bus-up, not init.
void car_settings_bus_up(void) {
    if (xSemaphoreTake(s_mutex, portMAX_DELAY)) {
        s_bus_up = true;
        s_boot_us = esp_timer_get_time(); // reset quiet timer to bus-up
        xSemaphoreGive(s_mutex);
    }
}

// Arm the one-shot startup probe: 3 passive (all 0xFF) 0x4C5 frames. The
// tick emits them one per tick once the bus is up; the car's 0x1F9 reply
// populates the status display via car_settings_can_rx_hook. Safe to call
// before CAN is enabled (attempts fail fast and retry silently). Passive
// frames carry no limit, so the probe runs regardless of configured target.
void car_settings_probe_status(void) {
    ESP_LOGI(TAG, "probing charge limit status (3x passive 0x4C5)");
    if (xSemaphoreTake(s_mutex, portMAX_DELAY)) {
        s_probe_pending = true;
        xSemaphoreGive(s_mutex);
    }
}

bool charge_limit_set(uint8_t ac_percent, uint8_t dc_percent) {
    if (!is_valid_percent(ac_percent) || !is_valid_percent(dc_percent)) {
        return false;
    }
    if (xSemaphoreTake(s_mutex, portMAX_DELAY)) {
        s_ac_limit = ac_percent;
        s_dc_limit = dc_percent;
        // New target: reset the conflict latch so the next disagreeing reply
        // is answered again.
        s_conflict_target_ac = s_ac_limit;
        s_conflict_target_dc = s_dc_limit;
        xSemaphoreGive(s_mutex);
    }
    ESP_LOGI(TAG, "charge limit set AC %u%% DC %u%%", ac_percent, dc_percent);
    // Update the persisted config (RAM copy); caller commits with
    // config_server_save_cfg() so a combined request writes the file once.
    config_server_set_charge_ac_limit(ac_percent);
    config_server_set_charge_dc_limit(dc_percent);
    return true;
}

void charge_limit_get(uint8_t *ac_percent, uint8_t *dc_percent) {
    if (xSemaphoreTake(s_mutex, portMAX_DELAY)) {
        if (ac_percent != NULL) {
            *ac_percent = s_ac_limit;
        }
        if (dc_percent != NULL) {
            *dc_percent = s_dc_limit;
        }
        xSemaphoreGive(s_mutex);
    }
}

void charge_limit_get_actuals(uint8_t *ac_percent, uint8_t *dc_percent) {
    if (xSemaphoreTake(s_mutex, portMAX_DELAY)) {
        if (ac_percent != NULL) {
            *ac_percent = s_ac;
        }
        if (dc_percent != NULL) {
            *dc_percent = s_dc;
        }
        xSemaphoreGive(s_mutex);
    }
}

bool charge_limit_get_last_reply(uint8_t *ac_raw, uint8_t *dc_raw, int64_t *age_us) {
    if (xSemaphoreTake(s_mutex, portMAX_DELAY)) {
        bool valid = s_reply_seen_valid;
        if (ac_raw != NULL) {
            *ac_raw = s_reply_ac_raw;
        }
        if (dc_raw != NULL) {
            *dc_raw = s_reply_dc_raw;
        }
        if (age_us != NULL) {
            int64_t age = esp_timer_get_time() - s_reply_seen_us;
            *age_us = age;
            // "Seen" alone is not enough: only report a live value if the reply
            // is still within the freshness window. The caller gets the age in
            // age_us either way.
            if (valid && age >= CAR_SETTINGS_INJECTION_TIMEOUT_US) {
                valid = false;
            }
        }
        xSemaphoreGive(s_mutex);
        return valid;
    }
    return false;
}

void car_settings_can_rx_hook(twai_message_t *to_push, can_bus_t rx_bus) {
    if (to_push == NULL) {
        return;
    }
    // Trust CAR_BUS only. The head-unit side (bus 1) may carry its own
    // 0x4C5/0x1F9 with different values; on a two-bus harness those would
    // poison the template, the conflict reference, and the quiet timer.
    if (rx_bus != CAR_BUS) {
        return;
    }
    if (xSemaphoreTake(s_mutex, portMAX_DELAY)) {
        if (to_push->identifier == CHARGE_LIMIT_FRAME_ID) {
            if (to_push->data_length_code >= 6U) {
                s_last_ac_raw = to_push->data[4];
                s_last_dc_raw = to_push->data[5];
                s_last_seen_us = esp_timer_get_time();
                s_last_seen_valid = true;
                // Keep the full frame as the active-injection template so bytes other
                // than D5/D6 are replayed as observed.
                s_last_dlc = to_push->data_length_code;
                memcpy(s_last_data, to_push->data, sizeof(s_last_data));
                s_has_template = true;
            }
        } else if (to_push->identifier == CHARGE_LIMIT_STATUS_FRAME_ID) {
            if (to_push->data_length_code >= 4U
                    && is_valid_reply_raw(to_push->data[2])
                    && is_valid_reply_raw(to_push->data[3])) {
                // Car-side reply: D3 (data[2]) = AC limit, D4 (data[3]) = DC limit.
                // Feeds the status display and the enforcement reference (s_ac/s_dc).
                s_reply_ac_raw = to_push->data[2];
                s_reply_dc_raw = to_push->data[3];
                s_ac = charge_limit_raw_to_percent(to_push->data[2]);
                s_dc = charge_limit_raw_to_percent(to_push->data[3]);
                s_reply_seen_us = esp_timer_get_time();
                s_reply_seen_valid = true;
            }
        }
        xSemaphoreGive(s_mutex);
    }
}

// Build and transmit a 0x4C5. Active frames carry our limits in D5/D6 with
// the other bytes replayed from the last observed frame (the DBC only
// defines those two); with no observation yet, the rest are zero. Passive
// frames carry 0xFF on every byte (rest state). Returns true when queued.
static bool car_settings_inject(bool passive) {
    uint8_t ac_limit = 0, dc_limit = 0, dlc = 0;
    uint8_t data[8];
    bool has_template = false;

    if (xSemaphoreTake(s_mutex, portMAX_DELAY)) {
        ac_limit = s_ac_limit;
        dc_limit = s_dc_limit;
        has_template = s_has_template;
        dlc = s_last_dlc;
        if (has_template) {
            memcpy(data, s_last_data, sizeof(data));
        }
        xSemaphoreGive(s_mutex);
    }

    twai_message_t pkt = {0};
    pkt.identifier = CHARGE_LIMIT_FRAME_ID;
    if (has_template && dlc >= 6U) {
        pkt.data_length_code = dlc;
    } else {
        pkt.data_length_code = 8U;
    }
    if (passive) {
        memset(pkt.data, 0xFF, sizeof(pkt.data));
    } else {
        if (has_template) {
            memcpy(pkt.data, data, sizeof(pkt.data));
        }
        pkt.data[4] = charge_limit_percent_to_raw(ac_limit);
        pkt.data[5] = charge_limit_percent_to_raw(dc_limit);
    }
    if (can_send(CAR_BUS, &pkt, 1) != ESP_OK) {
        ESP_LOGW(TAG, "charge limit inject failed");
        return false;
    }
    if (passive) {
        ESP_LOGI(TAG, "injected 0x4C5 passive (all 0xFF)");
    } else {
        ESP_LOGI(TAG, "injected 0x4C5 AC %u%% DC %u%%",
                 ac_limit, dc_limit);
    }
    return true;
}

// Called every 40 ms from the precondition task (only while awake).
// Pure-inject design: forwarded 0x4C5 frames are never touched; our limits
// are enforced with our own frames only. Each trigger arms a sequence:
// CAR_SETTINGS_BURST_COUNT active frames, one idle tick (40 ms gap), then
// CAR_SETTINGS_BURST_COUNT passive (all 0xFF) frames - one frame per tick
// throughout.
//
// The enforcement reference is the 0x1F9 reply (s_ac/s_dc) while fresh.
// Charge-limit values are 10%-discrete, so ANY difference between the car's
// reported limit and the configured target is a conflict worth answering.
// To avoid bursting a car that will not move (its echo stays put) at every
// ~200 ms reply forever, each distinct disagreeing (actual,target) pair is
// answered at most once: firing again requires the reported value OR the
// configured target to change. With no fresh reply, a bus quiet for
// CAR_SETTINGS_INJECTION_TIMEOUT_US (10 s of no 0x4C5, or since bus-up if
// never seen) gets one burst to establish the limits. Passive tail only
// fires if active frames were actually sent.
void car_settings_tick(void) {
    int64_t now = esp_timer_get_time();

    // Snapshot shared state under mutex
    uint8_t ac_limit = 0, dc_limit = 0, ac = 0, dc = 0, last_ac_raw = 0, last_dc_raw = 0;
    int64_t last_seen_us = 0, reply_seen_us = 0, boot_us = 0;
    bool last_seen_valid = false, reply_seen_valid = false, bus_up = false, probe_pending = false;
    if (xSemaphoreTake(s_mutex, portMAX_DELAY)) {
        ac_limit = s_ac_limit;
        dc_limit = s_dc_limit;
        ac = s_ac;
        dc = s_dc;
        last_ac_raw = s_last_ac_raw;
        last_dc_raw = s_last_dc_raw;
        last_seen_us = s_last_seen_us;
        last_seen_valid = s_last_seen_valid;
        reply_seen_us = s_reply_seen_us;
        reply_seen_valid = s_reply_seen_valid;
        boot_us = s_boot_us;
        bus_up = s_bus_up;
        probe_pending = s_probe_pending;
        xSemaphoreGive(s_mutex);
    }

    // One-shot startup probe: becomes a passive burst once idle.
    if (probe_pending
            && xSemaphoreTake(s_mutex, portMAX_DELAY)) {
        if (s_burst_remaining == 0 && s_passive_remaining == 0 && !s_gap_tick) {
            s_probe_pending = false;
            s_passive_remaining = CAR_SETTINGS_BURST_COUNT;
        }
        xSemaphoreGive(s_mutex);
    }

    // Before anything is ever seen, the quiet timeout runs from bus-up so the
    // fallback doesn't fire immediately at startup.
    int64_t cmd_last = last_seen_valid ? last_seen_us : boot_us;
    bool cmd_quiet = (now - cmd_last) >= CAR_SETTINGS_INJECTION_TIMEOUT_US;
    bool reply_live = reply_seen_valid
        && (now - reply_seen_us) < CAR_SETTINGS_INJECTION_TIMEOUT_US;
    if (!cmd_quiet) {
        // 0x4C5 traffic reappeared; later quiet episodes re-arm the fallback.
        if (xSemaphoreTake(s_mutex, portMAX_DELAY)) {
            s_quiet_injected = false;
            xSemaphoreGive(s_mutex);
        }
    }
    if (reply_live) {
        // Any difference is a real conflict (10%-discrete values).
        bool conflict = (ac != ac_limit || dc != dc_limit);
        if (conflict
                && xSemaphoreTake(s_mutex, portMAX_DELAY)) {
            // Answer at most once per distinct disagreeing (actual,target)
            // pair; re-arm only when the car moved or the target changed.
            bool changed = (ac != s_conflict_ac || dc != s_conflict_dc
                    || ac_limit != s_conflict_target_ac
                    || dc_limit != s_conflict_target_dc);
            if (changed
                    && reply_seen_us != s_answered_reply_us
                    && (now - reply_seen_us) >= CAR_SETTINGS_CONFLICT_BACKOFF_US
                    && s_burst_remaining == 0 && s_passive_remaining == 0 && !s_gap_tick) {
                s_answered_reply_us = reply_seen_us;
                s_conflict_ac = ac;
                s_conflict_dc = dc;
                s_conflict_target_ac = ac_limit;
                s_conflict_target_dc = dc_limit;
                s_burst_remaining = CAR_SETTINGS_BURST_COUNT;
            }
            xSemaphoreGive(s_mutex);
        }
    } else if (cmd_quiet && xSemaphoreTake(s_mutex, portMAX_DELAY)) {
        uint8_t want_ac = charge_limit_percent_to_raw(ac_limit);
        uint8_t want_dc = charge_limit_percent_to_raw(dc_limit);
        if (!s_quiet_injected
                && s_burst_remaining == 0 && s_passive_remaining == 0 && !s_gap_tick
                && (!last_seen_valid
                    || last_ac_raw != want_ac || last_dc_raw != want_dc)) {
            s_quiet_injected = true;
            s_burst_remaining = CAR_SETTINGS_BURST_COUNT;
        }
        xSemaphoreGive(s_mutex);
    }

    // can_send() fails fast while the bus is down; skip until bus-up so the
    // probe/quiet fallback don't spin on an unconfigured driver.
    if (!bus_up) {
        return;
    }
    if (s_burst_remaining > 0) {
        if (car_settings_inject(false)) {
            if (xSemaphoreTake(s_mutex, portMAX_DELAY)) {
                if (--s_burst_remaining == 0) {
                    // One idle tick before the passive tail.
                    s_gap_tick = true;
                    s_passive_remaining = CAR_SETTINGS_BURST_COUNT;
                }
                xSemaphoreGive(s_mutex);
            }
        } else {
            // can_send() failed (bus down/wedged): drop the rest of the burst
            // rather than re-attempting every 40 ms and spamming the log. A
            // later conflict/quiet re-arm re-establishes the limits once the
            // bus recovers.
            if (xSemaphoreTake(s_mutex, portMAX_DELAY)) {
                s_burst_remaining = 0;
                s_gap_tick = false;
                s_passive_remaining = 0;
                xSemaphoreGive(s_mutex);
            }
        }
        return;
    }
    if (s_gap_tick) {
        if (xSemaphoreTake(s_mutex, portMAX_DELAY)) {
            s_gap_tick = false;
            xSemaphoreGive(s_mutex);
        }
        return;
    }
    // Passive tail only fires if we actually sent active frames (s_passive_remaining was set)
    if (s_passive_remaining > 0 && car_settings_inject(true)) {
        if (xSemaphoreTake(s_mutex, portMAX_DELAY)) {
            s_passive_remaining--;
            xSemaphoreGive(s_mutex);
        }
    }
}
