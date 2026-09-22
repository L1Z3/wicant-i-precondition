// Charge-limit bridging for E-GMP cars.
//
// The Gen5W head unit commands its charge limits on 0x4C5; the car echoes the
// limits it accepted on 0x1F9 (~200 ms periodic). This module mirrors the
// head unit's command, watches the echo, and falls back to injecting our
// configured target when the head unit stops talking (see car_settings_tick).
//
// Pure-inject design: forwarded 0x4C5 frames are never touched; our limits
// are enforced with our own frames only.
#include "car_settings.h"
#include "config_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include "freertos/semphr.h"

#define TAG __func__

// fallback for v300
// note: never use `!= HEAD_UNIT_BUS` as the fallback breaks this
#define CAR_BUS CAN_BUS_0
#if CAN_BUS_COUNT > 1
#define HEAD_UNIT_BUS CAN_BUS_1
#else
#define HEAD_UNIT_BUS CAN_BUS_0
#endif

// Head-unit charge-limit command (D5 = AC limit, D6 = DC limit, same factor
// 0.5): only ever seen on the head-unit bus.
#define CHARGE_LIMIT_CMD_FRAME_ID 0x4C5U
#define CMD_DLC_MIN 6U
#define CMD_AC_BYTE 4U
#define CMD_DC_BYTE 5U

// Car-side charge-limit status echo (BO_ 505 Charge_Limit_Status_1F9 in
// ioniq5-2022.dbc, ~200 ms periodic): D3 (data[2]) = AC limit, D4 (data[3])
// = DC limit, same factor 0.5 as the command frame. Mirrors the 0x4C5
// limits while the car accepts them.
#define CHARGE_LIMIT_REPLY_FRAME_ID 0x1F9U
#define REPLY_DLC_MIN 4U
#define REPLY_AC_BYTE 2U
#define REPLY_DC_BYTE 3U

// Raw limit byte values: 0xFF is "off" per the DBC, and a configured limit is
// the percent window scaled by 2 (50% -> 0x64, 100% -> 0xC8).
#define CHARGE_LIMIT_RAW_OFF 0xFFU
#define CHARGE_LIMIT_RAW_MIN (CHARGE_LIMIT_MIN * 2U)
#define CHARGE_LIMIT_RAW_MAX (CHARGE_LIMIT_MAX * 2U)

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

// A pair of raw (percent * 2) bus bytes.
typedef struct {
    uint8_t ac;
    uint8_t dc;
} raw_limit_pair_t;

// The last disagreeing (reported, target) pair we fired a conflict burst for.
// A conflict is answered at most once per distinct pair, so a car that
// refuses to move (echo stays put) does not get burst at every ~200 ms reply
// forever. Firing again requires the car's reported value OR the configured
// target to change.
typedef struct {
    charge_limit_pair_t reported;
    charge_limit_pair_t target;
} conflict_latch_t;

// Burst sequencing (one frame per tick, see car_settings_tick): active
// phase, one idle tick, passive tail.
typedef enum {
    BURST_IDLE = 0,
    BURST_ACTIVE,
    BURST_GAP,
    BURST_PASSIVE,
} burst_phase_t;

static SemaphoreHandle_t s_mutex = NULL;

// ---------------------------------------------------------------------------
// Shared state: written from the CAN rx task, the config server task and the
// precondition task, so every access goes through cs_lock()/cs_unlock().
// ---------------------------------------------------------------------------

// Configured target; the tick answers 0x1F9 disagreements with this.
static charge_limit_pair_t s_target = { CHARGE_LIMIT_DEFAULT, CHARGE_LIMIT_DEFAULT };

// Enforcement reference: car's reported limits in percent, fed by every 0x1F9
// reply. Freshness tracked by s_reply_seen_* below; the tick compares it
// against s_target.
static charge_limit_pair_t s_reported;

// Last 0x1F9 reply for the status display (raw bytes).
static raw_limit_pair_t s_reply_raw;
static int64_t s_reply_seen_us;
static bool s_reply_seen;

// Last 0x4C5 payload (raw bytes D5/D6): drives the injection timing in
// car_settings_tick (quiet detection and conflict answers).
static raw_limit_pair_t s_cmd_raw;
static int64_t s_cmd_seen_us;
static bool s_cmd_seen;

// Full last-seen 0x4C5 as the template for active injection: the DBC only
// defines bytes 4/5, so replay the other bytes as observed rather than
// fabricating them.
static uint8_t s_template_data[8] = {0};
static uint8_t s_template_dlc;
static bool s_has_template;

// Origin of the quiet timer, set at init and reset by car_settings_bus_up(),
// so the 10 s timeout also applies before the first frame is ever seen
// (otherwise injection would fire immediately at startup).
static int64_t s_quiet_ref_us;

// Tracks whether CAN bus is enabled; injection waits for bus-up.
static bool s_bus_up;

// One-shot startup probe armed by car_settings_probe_status(): the tick turns
// it into a passive burst once idle, eliciting a 0x1F9 reply that seeds the
// status display.
static bool s_probe_pending;

// Armed when the probe burst is armed; the first valid 0x1F9 reply after that
// is adopted as the configured target (one-shot).
static bool s_probe_await_reply;

// Last answered conflict (reported, target) pair, see conflict_latch_t.
static conflict_latch_t s_conflict;

// ---------------------------------------------------------------------------
// Burst sequencing state: touched only by the precondition task (through
// car_settings_tick), so no locking is needed or wanted here.
// ---------------------------------------------------------------------------
static burst_phase_t s_burst_phase = BURST_IDLE;
static uint8_t s_burst_frames_left = 0U;
static int64_t s_answered_reply_us = 0;
static bool s_quiet_injected = false;

// All shared state is accessed through these. s_mutex is a plain
// (non-recursive) mutex created by car_settings_init(): never nest cs_lock().
static void cs_lock(void) {
    configASSERT(s_mutex != NULL);
    xSemaphoreTake(s_mutex, portMAX_DELAY);
}

static void cs_unlock(void) {
    xSemaphoreGive(s_mutex);
}

static bool is_valid_percent(uint8_t p) {
    return p >= CHARGE_LIMIT_MIN && p <= CHARGE_LIMIT_MAX;
}

// A raw limit byte is a usable percent only within 50%..100% (0x64..0xC8).
// 0xFF = "off" and anything below 50% is not a configured limit; reject it
// so the status display and the conflict reference never see a bogus value.
static bool is_valid_raw_limit(uint8_t raw) {
    return raw >= CHARGE_LIMIT_RAW_MIN && raw <= CHARGE_LIMIT_RAW_MAX;
}

// Call only while holding the lock.
static void cs_latch_conflict(charge_limit_pair_t reported, charge_limit_pair_t target) {
    s_conflict.reported = reported;
    s_conflict.target = target;
}

uint8_t charge_limit_percent_to_raw(uint8_t percent) {
    return (uint8_t)(percent * 2U);
}

uint8_t charge_limit_raw_to_percent(uint8_t raw) {
    // 0xFF = "off" per the DBC; map it to 0 so it never reads as a real
    // limit. Any other out-of-range byte is clamped to the valid window.
    if (raw >= CHARGE_LIMIT_RAW_OFF) {
        return 0;
    }
    return (uint8_t)(raw / 2U);
}

// config_server_get_charge_*_limit() already clamp to the valid range
void car_settings_init(void) {
    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateMutex();
        configASSERT(s_mutex != NULL);
    }

    cs_lock();
    s_quiet_ref_us = esp_timer_get_time();
    s_target.ac_percent = (uint8_t)config_server_get_charge_ac_limit();
    s_target.dc_percent = (uint8_t)config_server_get_charge_dc_limit();
    cs_latch_conflict(s_reported, s_target);
    charge_limit_pair_t target = s_target;
    cs_unlock();

    ESP_LOGI(TAG, "charge limits init AC %u%% (0x%02X) DC %u%% (0x%02X)",
             target.ac_percent, charge_limit_percent_to_raw(target.ac_percent),
             target.dc_percent, charge_limit_percent_to_raw(target.dc_percent));
}

// Call once CAN is enabled; starts the quiet timer from bus-up, not init.
void car_settings_bus_up(void) {
    cs_lock();
    s_bus_up = true;
    s_quiet_ref_us = esp_timer_get_time(); // reset quiet timer to bus-up
    cs_unlock();
}

// Arm the one-shot startup probe: CAR_SETTINGS_BURST_COUNT passive (all 0xFF)
// 0x4C5 frames. The tick emits them once the bus is up; the car's 0x1F9 reply
// populates the status display via car_settings_can_rx_hook. Safe to call
// before CAN is enabled (attempts fail fast and retry silently). Passive
// frames carry no limit, so the probe runs regardless of configured target.
void car_settings_probe_status(void) {
    ESP_LOGI(TAG, "probing charge limit status (%ux passive 0x4C5)",
             CAR_SETTINGS_BURST_COUNT);
    cs_lock();
    s_probe_pending = true;
    cs_unlock();
}

bool charge_limit_set_target(charge_limit_pair_t target) {
    if (!is_valid_percent(target.ac_percent) || !is_valid_percent(target.dc_percent)) {
        return false;
    }
    cs_lock();
    s_target = target;
    // Leave the conflict latch untouched: it records the last (reported,
    // target) pair we ANSWERED. The tick compares the live reply against it,
    // so a changed target is what re-arms the burst. Resetting the latch here
    // would make the first post-apply reply look unchanged and the injection
    // would never fire.
    cs_unlock();

    ESP_LOGI(TAG, "charge limit set AC %u%% DC %u%%", target.ac_percent, target.dc_percent);
    // Update the persisted config (RAM copy); caller commits with
    // config_server_save_cfg() so a combined request writes the file once.
    config_server_set_charge_ac_limit(target.ac_percent);
    config_server_set_charge_dc_limit(target.dc_percent);
    return true;
}

charge_limit_pair_t charge_limit_get_target(void) {
    cs_lock();
    charge_limit_pair_t target = s_target;
    cs_unlock();
    return target;
}

charge_limit_pair_t charge_limit_get_reported(void) {
    cs_lock();
    charge_limit_pair_t reported = s_reported;
    cs_unlock();
    return reported;
}

bool charge_limit_get_reply(charge_limit_reply_t *out) {
    cs_lock();
    bool valid = s_reply_seen;
    int64_t age = esp_timer_get_time() - s_reply_seen_us;
    raw_limit_pair_t raw = s_reply_raw;
    cs_unlock();

    if (out != NULL) {
        out->ac_raw = raw.ac;
        out->dc_raw = raw.dc;
        out->age_us = age;
    }
    // "Seen" alone is not enough: only report a live value if the reply is
    // still within the freshness window. The caller gets the age either way.
    if (valid && age >= CAR_SETTINGS_INJECTION_TIMEOUT_US) {
        valid = false;
    }
    return valid;
}

// 0x4C5 seen on the head-unit bus: refresh the quiet timer and the injection
// template.
static void cs_rx_command(const twai_message_t *msg) {
    if (msg->data_length_code < CMD_DLC_MIN) {
        return;
    }
    cs_lock();
    s_cmd_raw.ac = msg->data[CMD_AC_BYTE];
    s_cmd_raw.dc = msg->data[CMD_DC_BYTE];
    s_cmd_seen_us = esp_timer_get_time();
    s_cmd_seen = true;
    // Keep the full frame as the active-injection template so bytes other
    // than D5/D6 are replayed as observed.
    s_template_dlc = msg->data_length_code;
    memcpy(s_template_data, msg->data, sizeof(s_template_data));
    s_has_template = true;
    cs_unlock();
}

// 0x1F9 reply from the car: refresh the status display, the enforcement
// reference, and (once) the configured target.
static void cs_rx_reply(const twai_message_t *msg) {
    if (msg->data_length_code < REPLY_DLC_MIN) {
        return;
    }
    uint8_t ac_raw = msg->data[REPLY_AC_BYTE];
    uint8_t dc_raw = msg->data[REPLY_DC_BYTE];
    if (!is_valid_raw_limit(ac_raw) || !is_valid_raw_limit(dc_raw)) {
        return;
    }

    bool adopted = false;
    charge_limit_pair_t reported = {0};
    cs_lock();
    s_reply_raw.ac = ac_raw;
    s_reply_raw.dc = dc_raw;
    s_reported.ac_percent = charge_limit_raw_to_percent(ac_raw);
    s_reported.dc_percent = charge_limit_raw_to_percent(dc_raw);
    s_reply_seen_us = esp_timer_get_time();
    s_reply_seen = true;
    // One-shot: the first valid reply after the startup probe burst is
    // adopted as the configured target, matching what the car already has
    // rather than fighting it. Cleared so later ~200 ms periodic replies never
    // re-adopt.
    if (s_probe_await_reply) {
        s_probe_await_reply = false;
        s_target = s_reported;
        cs_latch_conflict(s_reported, s_target);
        reported = s_reported;
        adopted = true;
    }
    cs_unlock();

    // Persisted-config RAM strings (no save_cfg: lock-free writes like
    // charge_limit_set_target() does after releasing the mutex; a reboot
    // re-adopts).
    if (adopted) {
        config_server_set_charge_ac_limit(reported.ac_percent);
        config_server_set_charge_dc_limit(reported.dc_percent);
    }
}

void car_settings_can_rx_hook(twai_message_t *to_push, can_bus_t rx_bus) {
    if (to_push == NULL) {
        return;
    }
    // Each frame only ever comes from one side: 0x4C5 is the head unit's
    // command and only appears on the head-unit bus, 0x1F9 is the car's reply
    // and only appears on the car bus. Rejecting the other side keeps a stray
    // same-ID frame from poisoning the injection template, the conflict
    // reference, and the quiet timer.
    if (to_push->identifier == CHARGE_LIMIT_CMD_FRAME_ID) {
        if (rx_bus == HEAD_UNIT_BUS) {
            cs_rx_command(to_push);
        }
    } else if (to_push->identifier == CHARGE_LIMIT_REPLY_FRAME_ID) {
        if (rx_bus == CAR_BUS) {
            cs_rx_reply(to_push);
        }
    }
}

// Build and transmit a 0x4C5. Active frames carry our target in D5/D6 with
// the other bytes replayed from the last observed frame (the DBC only defines
// those two); with no observation yet, the rest are zero. Passive frames
// carry 0xFF on every byte (rest state). Returns true when queued.
static bool car_settings_inject(bool passive) {
    charge_limit_pair_t target;
    uint8_t dlc = 8U;
    uint8_t data[8] = {0};

    cs_lock();
    target = s_target;
    if (s_has_template) {
        dlc = s_template_dlc;
        memcpy(data, s_template_data, sizeof(data));
    }
    cs_unlock();

    twai_message_t pkt = {0};
    pkt.identifier = CHARGE_LIMIT_CMD_FRAME_ID;
    pkt.data_length_code = dlc;
    if (passive) {
        memset(pkt.data, CHARGE_LIMIT_RAW_OFF, sizeof(pkt.data));
    } else {
        memcpy(pkt.data, data, sizeof(pkt.data));
        pkt.data[CMD_AC_BYTE] = charge_limit_percent_to_raw(target.ac_percent);
        pkt.data[CMD_DC_BYTE] = charge_limit_percent_to_raw(target.dc_percent);
    }

    if (can_send(CAR_BUS, &pkt, 1) != ESP_OK) {
        ESP_LOGW(TAG, "charge limit inject failed");
        return false;
    }
    if (passive) {
        ESP_LOGI(TAG, "injected 0x4C5 passive (all 0xFF)");
    } else {
        ESP_LOGI(TAG, "injected 0x4C5 AC %u%% DC %u%%",
                 target.ac_percent, target.dc_percent);
    }
    return true;
}

// The shared-state readings one tick needs, taken under a single lock so the
// decisions below see a consistent picture instead of re-locking per branch.
typedef struct {
    charge_limit_pair_t target;
    charge_limit_pair_t reported;
    raw_limit_pair_t cmd_raw;
    int64_t cmd_seen_us;
    bool cmd_seen;
    int64_t reply_seen_us;
    bool reply_seen;
    int64_t quiet_ref_us;
    bool bus_up;
    bool probe_pending;
    int64_t now;
} cs_tick_view_t;

static cs_tick_view_t cs_take_view(void) {
    cs_tick_view_t v;
    v.now = esp_timer_get_time();
    cs_lock();
    v.target = s_target;
    v.reported = s_reported;
    v.cmd_raw = s_cmd_raw;
    v.cmd_seen_us = s_cmd_seen_us;
    v.cmd_seen = s_cmd_seen;
    v.reply_seen_us = s_reply_seen_us;
    v.reply_seen = s_reply_seen;
    v.quiet_ref_us = s_quiet_ref_us;
    v.bus_up = s_bus_up;
    v.probe_pending = s_probe_pending;
    cs_unlock();
    return v;
}

static bool burst_idle(void) {
    return s_burst_phase == BURST_IDLE;
}

// Start a sequence of CAR_SETTINGS_BURST_COUNT frames in the given phase.
static void burst_arm(burst_phase_t phase) {
    s_burst_phase = phase;
    s_burst_frames_left = CAR_SETTINGS_BURST_COUNT;
}

// can_send() failed (bus down/wedged): drop the rest of the sequence rather
// than re-attempting every 40 ms and spamming the log. A later conflict/quiet
// re-arm re-establishes the limits once the bus recovers.
static void burst_abort(void) {
    s_burst_phase = BURST_IDLE;
}

// One-shot startup probe: becomes a passive burst once the machine is idle.
static void cs_arm_probe(const cs_tick_view_t *v) {
    if (!v->probe_pending || !burst_idle()) {
        return;
    }
    cs_lock();
    s_probe_pending = false;
    // Arm adoption: the first valid 0x1F9 reply after this burst seeds the
    // configured target (see cs_rx_reply).
    s_probe_await_reply = true;
    cs_unlock();
    burst_arm(BURST_PASSIVE);
}

// Decide whether this tick starts a burst. Two triggers, in priority order: a
// fresh 0x1F9 reply that disagrees with the target, else a bus gone quiet for
// CAR_SETTINGS_INJECTION_TIMEOUT_US.
//
// Charge-limit values are 10%-discrete, so ANY difference between the car's
// reported limit and the configured target is a conflict worth answering. To
// avoid bursting a car that will not move (its echo stays put) at every
// ~200 ms reply forever, each distinct disagreeing (reported, target) pair is
// answered at most once.
static void cs_arm_on_trigger(const cs_tick_view_t *v) {
    int64_t cmd_last = v->cmd_seen ? v->cmd_seen_us : v->quiet_ref_us;
    bool cmd_quiet = (v->now - cmd_last) >= CAR_SETTINGS_INJECTION_TIMEOUT_US;
    bool reply_live = v->reply_seen
        && (v->now - v->reply_seen_us) < CAR_SETTINGS_INJECTION_TIMEOUT_US;

    if (!cmd_quiet) {
        // 0x4C5 traffic reappeared; later quiet episodes re-arm the fallback.
        cs_lock();
        s_quiet_injected = false;
        cs_unlock();
    }
    if (!burst_idle()) {
        return; // a sequence is already running
    }

    bool arm = false;
    if (reply_live) {
        bool conflict = (v->reported.ac_percent != v->target.ac_percent
                || v->reported.dc_percent != v->target.dc_percent);
        if (conflict) {
            cs_lock();
            // Answer at most once per distinct disagreeing (reported, target)
            // pair; re-arm only when the car moved or the target changed.
            bool changed = (v->reported.ac_percent != s_conflict.reported.ac_percent
                    || v->reported.dc_percent != s_conflict.reported.dc_percent
                    || v->target.ac_percent != s_conflict.target.ac_percent
                    || v->target.dc_percent != s_conflict.target.dc_percent);
            if (changed
                    && v->reply_seen_us != s_answered_reply_us
                    && (v->now - v->reply_seen_us) >= CAR_SETTINGS_CONFLICT_BACKOFF_US) {
                s_answered_reply_us = v->reply_seen_us;
                cs_latch_conflict(v->reported, v->target);
                arm = true;
            }
            cs_unlock();
        }
    } else if (cmd_quiet) {
        raw_limit_pair_t want;
        want.ac = charge_limit_percent_to_raw(v->target.ac_percent);
        want.dc = charge_limit_percent_to_raw(v->target.dc_percent);
        cs_lock();
        if (!s_quiet_injected
                && (!v->cmd_seen
                    || v->cmd_raw.ac != want.ac || v->cmd_raw.dc != want.dc)) {
            s_quiet_injected = true;
            arm = true;
        }
        cs_unlock();
    }
    if (arm) {
        burst_arm(BURST_ACTIVE);
    }
}

// Emit one frame of the armed sequence. The passive tail only ever runs after
// the active phase, so it fires only if active frames were actually sent.
static void cs_step_burst(void) {
    switch (s_burst_phase) {
    case BURST_ACTIVE:
        if (!car_settings_inject(false)) {
            burst_abort();
        } else if (--s_burst_frames_left == 0U) {
            s_burst_phase = BURST_GAP;
        }
        break;
    case BURST_GAP:
        // One idle tick before the passive tail.
        s_burst_frames_left = CAR_SETTINGS_BURST_COUNT;
        s_burst_phase = BURST_PASSIVE;
        break;
    case BURST_PASSIVE:
        if (!car_settings_inject(true)) {
            burst_abort();
        } else if (--s_burst_frames_left == 0U) {
            s_burst_phase = BURST_IDLE;
        }
        break;
    case BURST_IDLE:
    default:
        break;
    }
}

// Called every 40 ms from the precondition task (only while awake). Each
// trigger arms a sequence: CAR_SETTINGS_BURST_COUNT active frames, one idle
// tick (40 ms gap), then CAR_SETTINGS_BURST_COUNT passive (all 0xFF) frames -
// one frame per tick throughout. With no fresh reply, a bus quiet for
// CAR_SETTINGS_INJECTION_TIMEOUT_US (10 s of no 0x4C5, or since bus-up if
// never seen) gets one burst to establish the limits.
void car_settings_tick(void) {
    cs_tick_view_t v = cs_take_view();
    cs_arm_probe(&v);
    cs_arm_on_trigger(&v);
    // can_send() fails fast while the bus is down; skip until bus-up so the
    // probe/quiet fallback don't spin on an unconfigured driver.
    if (v.bus_up) {
        cs_step_burst();
    }
}
