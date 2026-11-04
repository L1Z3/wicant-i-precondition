#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "can.h"
#include "climate.h"
#include "hw_config.h"

#define TAG "climate"

#if HW_HAS_MCP2515

// ********************* what the head unit sends *********************
//
// Reverse-engineered from real Bluelink presses, logged on M-CAN with the head
// unit's second bus (I-CAN) captured in parallel:
//
//   WiCAN_MITM/Dual_M-CAN_MITM_I-CAN_parallel_*.csv
//
// Every remote command looks the same on M-CAN:
//
//   t+0.000s  0x4F1  00 D0 00 00 00 00 00 00   session bit, x3
//   t+1.03s   0x4A2  <request payload>        request, x3
//   t+1.15s   0x4A2  <apply payload>          apply, x3
//             0x0C7  00 00 00 01 00 00 00 00   remote command opcode, x3
//   t+1.27s   0x0C7  00 00 00 00 00 00 00 00   trailer, x3
//   ...       0x4F1  00 D0 ...                held ~8 s at 1 Hz
//   t+8.10s   0x4F1  00 C0 ...                session closed, x3
//
// The 0x4A2 payload carries the command:
//
//   start             00 00 0F 03 00 00 00 00  -> 00 00 FF 03 FF FF 00 00
//   + front defrost   00 00 0F C3 00 00 F0 00  -> 00 00 FF C3 FF FF F0 00
//   + rear/side       00 00 7F C3 00 00 F0 00  -> 00 00 FF C3 FF FF F0 00
//   + steering heat   00 00 DF C3 00 00 F0 00  -> 00 00 FF C3 FF FF F0 00
//   lock              00 00 F3 43 00 00 C0 00  -> 00 00 F3 C3 00 00 40 00
//   unlock            00 00 F3 43 00 00 C0 00  -> 00 00 F3 C3 00 00 D0 00
//
// The car answers ~0.6-1.2 s later: 0x2AD moves to its running state, 0x380
// becomes 3A 85 00 06 06 06 06 01 (DATC up) and 0x037 reports the HV system
// engaged. The stop direction carries no 0x4A2 at all, just 0x4F1 00 D0.
//
// 0x0C7 byte 3 = 0x01 is optional: some presses sent it, others did not and
// still started climate. It is replayed here because most captures have it.
//
// The head unit's second bus carries the session heartbeat 0x39B
// (`50 06 00 XX 00 03 10 00`, XX counting 00, 04 .. 3C at 2 Hz, byte 0 = 0x50
// while climate runs, 0x40 as it closes, byte 5 = command class). It is not
// transmitted on M-CAN. This build sends it on M-CAN anyway: if anything on
// the car side listens for it through the gateway, this is where it would show
// up, and if nothing does it is a few harmless bytes every 200 ms.
//
// None of these frames carries authentication, a counter, or a seed/key
// exchange: 0x4A2 bytes 0-1 are 0x00 in every payload ever captured, the
// payloads are byte-identical across model years and head units, and no 0x7xx
// diagnostic traffic appears in any remote-command capture. The three-frame
// repetition is the same latch the 0x0C7 preconditioning commands use.
//
// ********************* why there are several buttons *********************
//
// The climate replay was tried byte-for-byte and the car ignored it, while the
// head unit's own frames work. The captures show no difference on M-CAN, so
// the suspicion is the I-CAN session: the telematics unit opens it before the
// head unit sends its 0x4A2 burst. Lock and unlock are the interesting control
// case, because their M-CAN request/apply payloads are identical and only the
// follow-up state nibble differs (0x40 vs 0xD0).
//
// ********************* what the car answers *********************
//
//   0x037  byte 0          vehicle power state. 0x00 while off, 0x22/0x23 once
//                          the HV system engages, which follows a successful
//                          remote climate start by about 0.1 s.
//   0x2AD  byte 1          BMS precondition state; 00 02 -> 04 01 when the
//                          car starts the remote request.
//   0x380  bytes 3..6      DATC setpoints (car bus). 00 00 00 00 while the
//                          climate unit is asleep, 06 06 06 06 once it is up.
//   0x4DD  byte 2          state bit. 0xFE -> 0x00 at the 0x4A2 apply.
//   0x4EB  byte 4 bit 7    head unit's active flag.
//   0x4A2  bytes 2..3      the request itself, seen on either bus.
//   0x0C7  bytes 3..4      the remote opcode and its trailer, echoed back.
//   0x4F1  byte 1          the session bit, on while a remote session is open.
//   0x39B  byte 0          I-CAN session heartbeat (not received on M-CAN).

// Bus wiring of the test harness (2026-11-05): the .223 WiCAN has I-CAN on
// bus 0 and M-CAN on bus 1. The commands (0x4F1/0x4A2/0x0C7) are M-CAN and the
// telematics heartbeat (0x39B) is I-CAN, so the two halves go out on opposite
// buses. Override either define at build time if the harness is wired the other
// way round.
#ifndef CLIMATE_MCAN_BUS
#define CLIMATE_MCAN_BUS CAN_BUS_1
#endif
#ifndef CLIMATE_ICAN_BUS
#define CLIMATE_ICAN_BUS CAN_BUS_0
#endif
#define CLIMATE_CAR_BUS CLIMATE_MCAN_BUS
#define CLIMATE_TMU_BUS CLIMATE_ICAN_BUS


#define CLIMATE_WAKE_FRAME_ID 0x4F1U
#define CLIMATE_REQUEST_FRAME_ID 0x4A2U
#define CLIMATE_OPCODE_FRAME_ID 0x0C7U
#define CLIMATE_TMU_FRAME_ID 0x39BU

// The car announces the cabin setpoint the way its own ECUs do: the telematics
// frame 0x39B carries it in byte 1, the VCU re-broadcasts it on M-CAN in 0x2AC
// byte 4, and the HVAC reports the applied value in 0x380 bytes 3..6 about
// 1.7 s after 0x39B. All three use the same code, raw = 2T - 28, i.e.
// 0x06 = 17.0 C, 0x10 = 22.0 C, 0x16 = 25.0 C (DBC: BO_ 923, BO_ 684, BO_ 896).
// Every replay before this one sent 0x06 (17 C) whatever the request asked for.

// Every command is repeated three times, matching the head unit and the 0x0C7
// burst helper in precondition.c.
#define CLIMATE_BURST_TICKS 3U

// The head unit holds the session bit for ~1.04 s before it sends 0x4A2.
#define CLIMATE_WAKE_LEAD_TICKS 26U

// Waking a sleeping car takes a preamble: the head unit holds 0x4F1 00 C0 (the
// rest bit, not the session bit) for 3.0 s before it opens the session with
// 0x4A2 00 00 F3 43. Captured 2026-11-04 at 4713.954 s (C0 burst) and
// 4716.951 s (F3 43 burst), both on the head-unit side of the MITM.
#define CLIMATE_PREWAKE_TICKS 75U

// 0x39B and 0x4DD run at 5 Hz during the session, 0x4F1 at 1 Hz.
#define CLIMATE_SESSION_PERIOD_TICKS 5U
#define CLIMATE_KEEPALIVE_TICKS 25U

// The head unit keeps the session open for about 8 s in total (wake burst to
// the closing 0x4F1 00 C0 burst); everything after the trailer is this phase.
#define CLIMATE_SESSION_TICKS 170U

#define CLIMATE_TRAILER_TICK \
    (CLIMATE_WAKE_LEAD_TICKS + 2U * CLIMATE_BURST_TICKS)
#define CLIMATE_SESSION_TICK (CLIMATE_TRAILER_TICK + CLIMATE_BURST_TICKS)
#define CLIMATE_SEQUENCE_TICKS (CLIMATE_SESSION_TICK + CLIMATE_SESSION_TICKS)
#define CLIMATE_CLEAR_TICKS (CLIMATE_SEQUENCE_TICKS + CLIMATE_BURST_TICKS)

// ********************* the command table *********************

typedef struct {
    const char *label;
    // 0x39B byte 5: 0x01 lock/unlock, 0x03 climate.
    uint8_t tmu_class;
    // 0x39B byte 0 while the command is running: 0x50 climate, 0x40 closing,
    // 0x00 for lock/unlock.
    uint8_t tmu_state;
    // Tick at which tmu_state replaces 0x00 in the heartbeat.
    uint8_t state_tick;
    bool send_request;
    bool send_opcode;
    uint8_t request[8];
    uint8_t apply[8];
    // Prepend the 3 s 0x4F1 00 C0 preamble and the 0x4A2 F3 43/F3 C3 session
    // envelope, the way the head unit does when the car has to be woken.
    bool prewake;
    // The 0x4A2 payload is not fixed: it is built from the setpoint and the
    // checkbox options when the burst goes out (see build_climate_payload).
    // The request/apply arrays above are then unused and only kept readable.
    bool climate_start;
    // A stop has no 0x4A2 at all. The car is stopped by the telematics
    // telegram 0x39B on the I-CAN: class 0x01 phase 0x10 for a few seconds,
    // then class 0x03 phase 0x10, then class 0x03 phase 0x14, which the car
    // applies within ~90 ms (captured 2026-11-04 18:2x and 19:0x).
    bool tmu_off;
} climate_command_t;

#define CLIMATE_COMMAND_COUNT \
    ((uint8_t)(sizeof(climate_commands) / sizeof(climate_commands[0])))

static const climate_command_t climate_commands[] = {
    // Every start opens with the wake preamble: 3 s of 0x4F1 00 C0, then the
    // 0x4A2 F3 43 / F3 C3 session envelope with 0x4F1 00 D0 120 ms later. A car
    // that is already awake ignores it, and a sleeping car needs it, so there is
    // no reason to guess which state the car is in.
    {"start",
     0x03U, 0x50U, (uint8_t)CLIMATE_WAKE_LEAD_TICKS + CLIMATE_BURST_TICKS, true, true,
     {0x00U, 0x00U, 0x0FU, 0x03U, 0x00U, 0x00U, 0x00U, 0x00U},
     {0x00U, 0x00U, 0xFFU, 0x03U, 0xFFU, 0xFFU, 0x00U, 0x00U},
     true, true, false},
    // A stop is not a 0x4A2 command. The 0x4A2 FF 43 / FF C3 pair that older
    // notes called the stop never stops the car when replayed; what does is
    // the telematics telegram below, sent on the I-CAN (see tmu_off).
    {"stop",
     0x03U, 0x40U, (uint8_t)CLIMATE_WAKE_LEAD_TICKS, false, false,
     {0x00U, 0x00U, 0xFFU, 0x43U, 0xFFU, 0xFFU, 0x00U, 0x00U},
     {0x00U, 0x00U, 0xFFU, 0xC3U, 0xFFU, 0xFFU, 0x00U, 0x00U},
     false, false, true},
    {"lock",
     0x01U, 0x00U, (uint8_t)CLIMATE_WAKE_LEAD_TICKS + CLIMATE_BURST_TICKS, true, true,
     {0x00U, 0x00U, 0xF3U, 0x43U, 0x00U, 0x00U, 0xC0U, 0x00U},
     {0x00U, 0x00U, 0xF3U, 0xC3U, 0x00U, 0x00U, 0x40U, 0x00U},
     false, false, false},
    {"unlock",
     0x01U, 0x00U, (uint8_t)CLIMATE_WAKE_LEAD_TICKS + CLIMATE_BURST_TICKS, true, true,
     {0x00U, 0x00U, 0xF3U, 0x43U, 0x00U, 0x00U, 0xC0U, 0x00U},
     {0x00U, 0x00U, 0xF3U, 0xC3U, 0x00U, 0x00U, 0xD0U, 0x00U},
     false, false, false},
};

// ********************* sequence storage *********************

typedef struct {
    // Set by the web UI handler, cleared by the tick when the sequence starts.
    // Blocking keys off this too, so the request takes effect immediately.
    _Atomic bool requested;
    _Atomic bool active;
    _Atomic uint8_t command;
    // Ticks since the sequence started; owned by the precondition task.
    _Atomic uint32_t tick;
    // Free-running 40 ms counter, so a received frame can be timestamped
    // against the start of the sequence whichever task sees it.
    _Atomic uint32_t now_tick;
    _Atomic uint32_t start_tick;
    _Atomic bool started;
    // Last frame that looked like the car answering, packed by pack_event().
    _Atomic uint32_t event;
    _Atomic uint32_t event_tick;
    _Atomic bool event_valid;
    // 0x39B's counter byte, advanced only by the tick.
    uint8_t session_counter;
    // Setpoint announced to the car, in the car's own raw code (2T - 28), so
    // 0x39B byte 1 and 0x2AC byte 4 agree. Written by the web handler before
    // it raises `requested`, read by the tick.
    uint8_t setpoint_raw;
    // CLIMATE_OPTION_* bits for a climate start, written with the setpoint.
    uint8_t options;
} climate_state_t;

static climate_state_t climate;

// Event word: id in bits 0..14, above that the two payload bytes worth
// reporting, then which bus it arrived on, so one atomic load is a complete
// reading (the id is 11 bits, so bit 15 is free).
#define CLIMATE_EVENT_HU 0x8000U

// ********************* payloads *********************

static const uint8_t wake_data[8] = {0x00U, 0xD0U, 0x00U, 0x00U,
                                     0x00U, 0x00U, 0x00U, 0x00U};
static const uint8_t idle_data[8] = {0x00U, 0xC0U, 0x00U, 0x00U,
                                     0x00U, 0x00U, 0x00U, 0x00U};
// 0x4A2 D2 = 0xF3: the session-open envelope. The head unit sends D3 = 0x43 one
// burst ahead of 0x4F1 00 D0 and D3 = 0xC3 120 ms later, but only when the car
// was asleep; an awake car gets no F3 pair at all (2026-11-04, 01:57 local).
static const uint8_t session_open_data[8] = {0x00U, 0x00U, 0xF3U, 0x43U,
                                             0x00U, 0x00U, 0x00U, 0x00U};
static const uint8_t session_commit_data[8] = {0x00U, 0x00U, 0xF3U, 0xC3U,
                                               0x00U, 0x00U, 0x00U, 0x00U};
static const uint8_t opcode_data[8] = {0x00U, 0x00U, 0x00U, 0x01U,
                                       0x00U, 0x00U, 0x00U, 0x00U};
static const uint8_t trailer_data[8] = {0x00U, 0x00U, 0x00U, 0x00U,
                                        0x00U, 0x00U, 0x00U, 0x00U};
// 0x4DD is 5 Hz on this car and reads 08 in byte 4 while it is awake; only
// byte 2 changes when the session opens.
static const uint8_t state_data[8] = {0x00U, 0x00U, 0x00U, 0x00U,
                                      0x08U, 0x00U, 0x00U, 0x00U};

static void send_frame_on(can_bus_t bus, uint32_t identifier,
                          const uint8_t data[8]) {
    twai_message_t packet = {0};
    packet.identifier = identifier;
    packet.data_length_code = 8U;
    memcpy(packet.data, data, 8U);
    can_send(bus, &packet, 1);
}

static void send_climate_frame(uint32_t identifier, const uint8_t data[8]) {
    send_frame_on(CLIMATE_CAR_BUS, identifier, data);
}

// raw = 2T - 28, expressed in tenths of a degree so the half-degree steps the
// app offers (22.5 C -> 0x11) come out exactly.
static uint8_t setpoint_raw_from_tenths(int16_t tenths) {
    if (tenths < CLIMATE_SETPOINT_MIN_TENTHS) {
        tenths = CLIMATE_SETPOINT_MIN_TENTHS;
    } else if (tenths > CLIMATE_SETPOINT_MAX_TENTHS) {
        tenths = CLIMATE_SETPOINT_MAX_TENTHS;
    }
    return (uint8_t)((tenths - CLIMATE_SETPOINT_MIN_TENTHS) / 5);
}

// 0x4A2 payload for a climate start, built from the options instead of the
// fixed arrays in the table. Without options this is the plain start
// (00 00 0F 03 .. / 00 00 FF 03 FF FF ..). Every option turns the burst into
// the 0xC3 form and sets byte 6 to 0xF0, and the option's bits go into the
// request mask:
//
//   front defrost     0F C3 00 00 F0 00   mask unchanged
//   rear/side defrost 7F C3 00 00 F0 00   mask 0x0F | 0x40 | 0x10 | 0x20
//   steering heat     DF C3 00 00 F0 00   mask 0x0F | 0x40 | 0x10 | 0x80
//
// Only one option at a time was ever captured, so combinations are an OR of
// the bits seen in those three: rear + steering together come out as mask
// 0xFF, which no capture shows and which collides with the apply mask. That
// combination has not been tested on a car and should be treated as a guess
// until it is.
static void build_climate_payload(uint8_t out[8], bool apply) {
    const uint8_t options = climate.options;
    uint8_t mask = 0x0FU;
    uint8_t commit = 0x03U;
    uint8_t extra = 0x00U;
    if ((options & (CLIMATE_OPTION_REAR_DEFROST | CLIMATE_OPTION_STEERING_HEAT)) != 0U) {
        mask |= 0x40U | 0x10U;
    }
    if ((options & CLIMATE_OPTION_REAR_DEFROST) != 0U) {
        mask |= 0x20U;
    }
    if ((options & CLIMATE_OPTION_STEERING_HEAT) != 0U) {
        mask |= 0x80U;
    }
    if (options != 0U) {
        commit = 0xC3U;
        extra = 0xF0U;
    }
    out[0] = 0x00U;
    out[1] = 0x00U;
    out[2] = apply ? 0xFFU : mask;
    out[3] = commit;
    out[4] = apply ? 0xFFU : 0x00U;
    out[5] = apply ? 0xFFU : 0x00U;
    out[6] = extra;
    out[7] = 0x00U;
}

static void send_climate_burst(const climate_command_t *command, bool apply) {
    if (command->climate_start) {
        uint8_t payload[8];
        build_climate_payload(payload, apply);
        send_climate_frame(CLIMATE_REQUEST_FRAME_ID, payload);
    } else {
        send_climate_frame(CLIMATE_REQUEST_FRAME_ID,
                           apply ? command->apply : command->request);
    }
}

static void send_wake_frame(void) {
    send_climate_frame(CLIMATE_WAKE_FRAME_ID, wake_data);
}

static void send_idle_frame(void) {
    send_climate_frame(CLIMATE_WAKE_FRAME_ID, idle_data);
}

// The I-CAN session heartbeat. A car whose climate is running shows
// `50 <setpoint> 00 <counter> 00 00 00 00` at 5 Hz (read off the live bus on
// 2026-11-04 while the cloud had the climate on): byte 5 (the command class)
// and byte 6 (the phase) are zero except while a command is being executed.
// Sending a permanent class 0x03 phase 0x10 here made the car treat every
// replay as a stop, which is how the 2026-11-04 stop telegram was found.
static void send_tmu_frame(const climate_command_t *command, bool running) {
    uint8_t data[8] = {(uint8_t)(running ? command->tmu_state : 0x00U),
                       climate.setpoint_raw, 0x00U, climate.session_counter, 0x00U,
                       0x00U, 0x00U, 0x00U};
    climate.session_counter = (uint8_t)((climate.session_counter + 4U) & 0x3FU);
    send_frame_on(CLIMATE_TMU_BUS, CLIMATE_TMU_FRAME_ID, data);
}

// One frame of the stop telegram. The car reports "not running, no setpoint"
// while the command executes, so bytes 0..1 are zero here.
static void send_tmu_telegram(uint8_t tmu_class, uint8_t phase) {
    uint8_t data[8] = {0x00U, 0x00U, 0x00U, climate.session_counter, 0x00U,
                       tmu_class, phase, 0x00U};
    climate.session_counter = (uint8_t)((climate.session_counter + 4U) & 0x3FU);
    send_frame_on(CLIMATE_TMU_BUS, CLIMATE_TMU_FRAME_ID, data);
}

// Stop telegram schedule, in 40 ms ticks from the 0x4F1 00 D0 burst: a
// class 0x01 phase 0x10 preamble, then class 0x03 phase 0x10, then the
// class 0x03 phase 0x14 that the car applies.
#define CLIMATE_STOP_PREAMBLE_TICK 5U
#define CLIMATE_STOP_PREAMBLE_END 80U
#define CLIMATE_STOP_CLIMATE_END 105U
#define CLIMATE_STOP_APPLY_END 116U

// Keeps the session alive: the heartbeat and the state frame at 5 Hz, the
// session bit at 1 Hz, on their own offsets inside a 200 ms period.
static void send_session_frames(const climate_command_t *command, uint32_t tick) {
    const bool running = tick >= command->state_tick;
    const uint32_t phase = tick % CLIMATE_SESSION_PERIOD_TICKS;
    if (command->tmu_off) {
        if (phase == 0U) {
            if (tick >= CLIMATE_STOP_PREAMBLE_TICK
                    && tick < CLIMATE_STOP_PREAMBLE_END) {
                send_tmu_telegram(0x01U, 0x10U);
            } else if (tick >= CLIMATE_STOP_PREAMBLE_END
                    && tick < CLIMATE_STOP_CLIMATE_END) {
                send_tmu_telegram(0x03U, 0x10U);
            } else if (tick >= CLIMATE_STOP_CLIMATE_END
                    && tick < CLIMATE_STOP_APPLY_END) {
                send_tmu_telegram(0x03U, 0x14U);
            } else if (tick >= CLIMATE_STOP_APPLY_END) {
                send_tmu_frame(command, false);
            }
        }
    } else if (phase == 3U) {
        send_tmu_frame(command, running);
    } else if (phase == 4U) {
        send_climate_frame(0x4DDU, state_data);
    }
    if ((tick % CLIMATE_KEEPALIVE_TICKS) == 11U) {
        send_wake_frame();
    }
}

// ********************* response log *********************

// Only a couple of bytes per frame say anything about the command; the rest is
// a counter, a clock or a checksum. These are the ones seen moving in the
// captures (see the header comment).
typedef struct {
    uint32_t id;
    uint8_t first;
    uint8_t second;
} climate_watch_t;

static const climate_watch_t climate_watch[] = {
    {CLIMATE_WAKE_FRAME_ID, 1U, 0U},      // session bit, stay-alive byte
    {CLIMATE_REQUEST_FRAME_ID, 2U, 3U},   // request mask, command byte
    {CLIMATE_OPCODE_FRAME_ID, 3U, 4U},    // opcode, its trailer
    {CLIMATE_TMU_FRAME_ID, 0U, 3U},       // I-CAN session heartbeat
    {0x037U, 0U, 1U},                     // vehicle power state
    {0x4DDU, 2U, 4U},                     // state byte, mode
    {0x4EBU, 4U, 2U},                     // active flag, minutes
    {0x380U, 3U, 4U},                     // DATC setpoints
    {0x2ADU, 1U, 0U},                     // BMS precondition state
};

static uint32_t pack_event(uint32_t id, uint8_t first, uint8_t second,
                           can_bus_t bus) {
    return id | ((uint32_t)first << 16) | ((uint32_t)second << 24)
           | (bus == CLIMATE_CAR_BUS ? 0U : CLIMATE_EVENT_HU);
}

static void log_event(uint32_t event) {
    // these frames repeat every few hundred milliseconds; only edges matter
    if (event == atomic_load_explicit(&climate.event, memory_order_relaxed)) {
        return;
    }
    atomic_store_explicit(&climate.event, event, memory_order_relaxed);
    atomic_store_explicit(&climate.event_tick,
                          atomic_load_explicit(&climate.now_tick, memory_order_relaxed),
                          memory_order_relaxed);
    atomic_store_explicit(&climate.event_valid, true, memory_order_relaxed);

    const char *bus = (event & CLIMATE_EVENT_HU) != 0U ? "head-unit" : "car";
    const unsigned id = (unsigned)(event & 0x7FFU);
    const unsigned first = (unsigned)((event >> 16) & 0xFFU);
    const unsigned second = (unsigned)((event >> 24) & 0xFFU);

    // say how far into the replay the response arrived, which is the only
    // thing that makes these lines useful later
    if (atomic_load_explicit(&climate.started, memory_order_relaxed)) {
        const uint32_t start = atomic_load_explicit(&climate.start_tick, memory_order_relaxed);
        const uint32_t now = atomic_load_explicit(&climate.now_tick, memory_order_relaxed);
        const int32_t delta = (int32_t)(now - start);
        const uint32_t since = delta > 0 ? (uint32_t)delta * 4U / 10U : 0U;
        ESP_LOGI(TAG, "response 0x%03X %02X %02X on the %s bus at +%u.%u s", id,
                 first, second, bus, (unsigned)(since / 10U), (unsigned)(since % 10U));
    } else {
        ESP_LOGI(TAG, "response 0x%03X %02X %02X on the %s bus", id, first, second,
                 bus);
    }
}

void climate_rx_hook(const twai_message_t *msg, can_bus_t rx_bus) {
    if (msg == NULL || msg->data_length_code < 8U) {
        return;
    }
    for (size_t i = 0; i < sizeof(climate_watch) / sizeof(climate_watch[0]); i++) {
        if (msg->identifier != climate_watch[i].id) {
            continue;
        }
        const uint8_t first = msg->data[climate_watch[i].first];
        const uint8_t second = msg->data[climate_watch[i].second];
        // skip the idle states of the two frames that also carry a trailer
        if (msg->identifier == CLIMATE_OPCODE_FRAME_ID && first == 0U
                && second == 0U) {
            return;
        }
        if (msg->identifier == CLIMATE_WAKE_FRAME_ID && (first & 0x10U) == 0U) {
            return;
        }
        log_event(pack_event(msg->identifier, first, second, rx_bus));
        return;
    }
}

// ********************* public API *********************

static bool climate_blocking(void) {
    return atomic_load_explicit(&climate.requested, memory_order_relaxed)
           || atomic_load_explicit(&climate.active, memory_order_relaxed);
}

void climate_tick(void) {
    const uint32_t now = atomic_fetch_add_explicit(
        &climate.now_tick, 1U, memory_order_relaxed);

    if (!atomic_load_explicit(&climate.active, memory_order_relaxed)) {
        if (!atomic_exchange_explicit(&climate.requested, false,
                                      memory_order_relaxed)) {
            return;
        }
        atomic_store_explicit(&climate.active, true, memory_order_relaxed);
        atomic_store_explicit(&climate.tick, 0U, memory_order_relaxed);
        atomic_store_explicit(&climate.start_tick, now, memory_order_relaxed);
        atomic_store_explicit(&climate.started, true, memory_order_relaxed);
        climate.session_counter = 0U;
        ESP_LOGI(TAG, "remote command: replaying %s",
                 climate_commands[atomic_load_explicit(&climate.command,
                                                       memory_order_relaxed)].label);
    }

    const uint8_t index = atomic_load_explicit(&climate.command, memory_order_acquire);
    const climate_command_t *command = &climate_commands[index];

    uint32_t t = atomic_fetch_add_explicit(
        &climate.tick, 1U, memory_order_relaxed);

    if (command->prewake) {
        // Waking a sleeping car: 3.0 s of 0x4F1 00 C0, then the F3 43 session
        // envelope, then 0x4F1 00 D0 with F3 C3 120 ms later. Captured
        // 2026-11-04 at 4713.954 / 4716.951 / 4716.962 / 4717.072 s.
        if (t < CLIMATE_PREWAKE_TICKS) {
            if (t < CLIMATE_BURST_TICKS || (t % CLIMATE_KEEPALIVE_TICKS) == 0U) {
                send_idle_frame();
            }
            return;
        }
        const uint32_t since = t - CLIMATE_PREWAKE_TICKS;
        if (since < CLIMATE_BURST_TICKS) {
            if (since == 0U) {
                // the I-CAN heartbeat, and with it the setpoint, opens with the
                // session envelope, 1 s before the climate request
                send_tmu_frame(command, false);
            }
            send_climate_frame(CLIMATE_REQUEST_FRAME_ID, session_open_data);
            return;
        }
        if (since < 2U * CLIMATE_BURST_TICKS) {
            send_wake_frame();
            send_climate_frame(CLIMATE_REQUEST_FRAME_ID, session_commit_data);
            return;
        }
        // Past the wake edge: same schedule as an awake start, with t = 0 at
        // the 0x4F1 00 D0 burst.
        t = since - CLIMATE_BURST_TICKS;
    }

    if (t < CLIMATE_BURST_TICKS) {
        send_wake_frame();
    } else if (t < CLIMATE_WAKE_LEAD_TICKS) {
        send_session_frames(command, t);
    } else if (t < CLIMATE_WAKE_LEAD_TICKS + CLIMATE_BURST_TICKS) {
        if (command->send_request) {
            send_climate_burst(command, false);
        }
        send_session_frames(command, t);
    } else if (t < CLIMATE_WAKE_LEAD_TICKS + 2U * CLIMATE_BURST_TICKS) {
        if (command->send_request) {
            send_climate_burst(command, true);
        }
        if (command->send_opcode) {
            send_climate_frame(CLIMATE_OPCODE_FRAME_ID, opcode_data);
        }
        send_session_frames(command, t);
    } else if (t < CLIMATE_SESSION_TICK) {
        if (command->send_opcode) {
            send_climate_frame(CLIMATE_OPCODE_FRAME_ID, trailer_data);
        }
        send_session_frames(command, t);
    } else if (t < CLIMATE_SEQUENCE_TICKS) {
        send_session_frames(command, t);
    } else if (t < CLIMATE_CLEAR_TICKS) {
        // close the session the way the head unit does, so the car does not
        // think a remote request is still open
        send_idle_frame();
    } else {
        atomic_store_explicit(&climate.active, false, memory_order_relaxed);
        ESP_LOGI(TAG, "remote command: %s sequence finished", command->label);
    }
}

fwd_result_t climate_fwd_hook(twai_message_t *msg, can_bus_t fwd_bus) {
    if (fwd_bus != CLIMATE_CAR_BUS || !climate_blocking()) {
        return FWD_PASSTHROUGH;
    }
    if (msg->identifier == CLIMATE_REQUEST_FRAME_ID
            || msg->identifier == CLIMATE_WAKE_FRAME_ID) {
        // The head unit starts the same remote command on its own as soon as
        // the request reaches it; let only our replay through to the car.
        return FWD_BLOCK;
    }
    return FWD_PASSTHROUGH;
}

esp_err_t climate_request_start_temp(uint8_t command, int16_t setpoint_tenths,
                                     uint8_t options) {
    if (command >= CLIMATE_COMMAND_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    // A request that is still waiting for the tick to pick it up counts as
    // busy too: the UI disables its buttons on climate_is_active(), which is
    // exactly this condition.
    if (climate_blocking()) {
        return ESP_ERR_INVALID_STATE;
    }
    // The setpoint and the options have to be in place before the tick sees
    // the command.
    climate.setpoint_raw = setpoint_raw_from_tenths(setpoint_tenths);
    climate.options = (uint8_t)(options
        & (CLIMATE_OPTION_FRONT_DEFROST | CLIMATE_OPTION_REAR_DEFROST
           | CLIMATE_OPTION_STEERING_HEAT));
    // Block the head unit's copies from here, before the precondition task
    // gets a chance to start the sequence.
    atomic_store_explicit(&climate.command, command, memory_order_release);
    atomic_store_explicit(&climate.requested, true, memory_order_relaxed);
    return ESP_OK;
}

esp_err_t climate_request_start(uint8_t command) {
    return climate_request_start_temp(command, CLIMATE_SETPOINT_DEFAULT_TENTHS, 0U);
}

uint8_t climate_command_count(void) {
    return CLIMATE_COMMAND_COUNT;
}

const char *climate_command_label(uint8_t command) {
    if (command >= CLIMATE_COMMAND_COUNT) {
        return "unknown";
    }
    return climate_commands[command].label;
}

bool climate_is_active(void) {
    return climate_blocking();
}

bool climate_get_status(char *out, size_t len) {
    if (out == NULL || len == 0U) {
        return false;
    }

    const bool active = atomic_load_explicit(&climate.active, memory_order_relaxed);
    const bool queued = !active
        && atomic_load_explicit(&climate.requested, memory_order_relaxed);
    const bool started = atomic_load_explicit(&climate.started, memory_order_relaxed);
    const uint32_t start_tick = atomic_load_explicit(&climate.start_tick, memory_order_relaxed);
    const uint32_t tick = atomic_load_explicit(&climate.tick, memory_order_relaxed);
    const char *label = climate_commands[atomic_load_explicit(
        &climate.command, memory_order_relaxed)].label;

    // one tick is 40 ms, so tick * 4 / 10 is tenths of a second. The buffer
    // is sized for the worst case the compiler assumes (ten digits per %u),
    // not for the 8.3 s the sequence actually takes.
    char phase[48];
    if (active) {
        const uint32_t elapsed = tick * 4U / 10U;
        const uint32_t total = (uint32_t)CLIMATE_CLEAR_TICKS * 4U / 10U;
        snprintf(phase, sizeof(phase), "replaying %s %u.%u/%u.%u s", label,
                 (unsigned)(elapsed / 10U), (unsigned)(elapsed % 10U),
                 (unsigned)(total / 10U), (unsigned)(total % 10U));
    } else if (queued) {
        snprintf(phase, sizeof(phase), "queued %s", label);
    } else {
        snprintf(phase, sizeof(phase), "idle");
    }

    if (!atomic_load_explicit(&climate.event_valid, memory_order_relaxed)) {
        snprintf(out, len, "%s", phase);
        return true;
    }

    // the event and its timestamp come from two atomics; re-read so a frame
    // arriving in between cannot pair one id with another id's time
    uint32_t event = 0;
    uint32_t event_tick = 0;
    for (int attempt = 0; attempt < 3; attempt++) {
        event = atomic_load_explicit(&climate.event, memory_order_relaxed);
        event_tick = atomic_load_explicit(&climate.event_tick, memory_order_relaxed);
        if (event == atomic_load_explicit(&climate.event, memory_order_relaxed)) {
            break;
        }
    }

    const unsigned id = (unsigned)(event & 0x7FFU);
    const unsigned first = (unsigned)((event >> 16) & 0xFFU);
    const unsigned second = (unsigned)((event >> 24) & 0xFFU);
    const char *bus = (event & CLIMATE_EVENT_HU) != 0U ? "hu" : "car";

    if (started) {
        const int32_t delta = (int32_t)(event_tick - start_tick);
        const uint32_t since = delta > 0 ? (uint32_t)delta * 4U / 10U : 0U;
        snprintf(out, len, "%s; 0x%03X %02X %02X %s +%u.%u s", phase, id, first,
                 second, bus, (unsigned)(since / 10U), (unsigned)(since % 10U));
    } else {
        snprintf(out, len, "%s; 0x%03X %02X %02X %s", phase, id, first, second, bus);
    }
    return true;
}

#else

void climate_tick(void) {}

fwd_result_t climate_fwd_hook(twai_message_t *msg, can_bus_t fwd_bus) {
    (void)msg;
    (void)fwd_bus;
    return FWD_PASSTHROUGH;
}

esp_err_t climate_request_start(uint8_t command) {
    (void)command;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t climate_request_start_temp(uint8_t command, int16_t setpoint_tenths,
                                     uint8_t options) {
    (void)command;
    (void)setpoint_tenths;
    (void)options;
    return ESP_ERR_NOT_SUPPORTED;
}

uint8_t climate_command_count(void) {
    return 0U;
}

const char *climate_command_label(uint8_t command) {
    (void)command;
    return "unknown";
}

bool climate_is_active(void) {
    return false;
}

void climate_rx_hook(const twai_message_t *msg, can_bus_t rx_bus) {
    (void)msg;
    (void)rx_bus;
}

bool climate_get_status(char *out, size_t len) {
    (void)out;
    (void)len;
    return false;
}

#endif
