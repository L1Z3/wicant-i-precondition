#pragma once

#include "MCP251XFD.h"

// The core has no RTOS dependencies. Its caller serializes all operations,
// including callbacks, with one lock. SPI and callbacks run in task context.
#define MCP251XFD_TX_CAPACITY 32
#define MCP251XFD_HW_TX_DEPTH 8

typedef enum {
    MCP251XFD_STATE_ACTIVE,
    MCP251XFD_STATE_WARNING,
    MCP251XFD_STATE_PASSIVE,
    MCP251XFD_STATE_BUS_OFF,
} mcp251xfd_state_t;

typedef struct {
    uint32_t id;
    uint8_t dlc;
    bool extended;
    bool rtr;
    uint8_t data[8];
} mcp251xfd_frame_t;

typedef struct {
    uint32_t oscillator_hz;
    uint32_t bitrate;
    uint16_t sample_point_permill;
    uint8_t tx_queue_depth;
    bool listen_only;
    bool loopback;
    bool one_shot;
} mcp251xfd_config_t;

typedef struct {
    void (*tx_done)(void *arg, const void *token, bool success);
    void (*rx_done)(void *arg, const mcp251xfd_frame_t *frame);
    void (*state_changed)(void *arg, mcp251xfd_state_t old, mcp251xfd_state_t state);
    void (*error)(void *arg, uint32_t diagnostic, bool arbitration_lost);
    void *arg;
} mcp251xfd_callbacks_t;

typedef struct {
    mcp251xfd_frame_t frame;
    const void *token;
    uint32_t sequence;
} mcp251xfd_pending_tx_t;

typedef struct {
    uint32_t con;
    uint32_t nbtcfg;
    uint32_t osc;
    uint32_t iocon;
    uint8_t valid; // Bits 0..3 correspond to the registers above; failed reads stay zero.
} mcp251xfd_registers_t;

typedef struct {
    const char *reason;
    uint32_t trec;
    uint32_t bdiag1;
    uint32_t bdiag1_seen; // Error flags accumulated since enable, before clearing hardware flags.
    uint16_t interrupts;
    bool trec_valid;
    bool bdiag1_valid;
    uint8_t queued;
    uint8_t loaded;
    mcp251xfd_registers_t registers; // Captured before fault handling enters configuration mode.
} mcp251xfd_diagnostics_t;

typedef struct {
    MCP251XFD device;
    mcp251xfd_config_t config;
    mcp251xfd_callbacks_t callbacks;
    mcp251xfd_diagnostics_t diagnostics;
    mcp251xfd_pending_tx_t tx[MCP251XFD_TX_CAPACITY];
    uint32_t next_sequence;
    uint32_t bus_errors;
    uint32_t rx_overruns;
    uint32_t rx_fd_dropped;
    uint8_t head;
    uint8_t count;
    uint8_t loaded;
    uint8_t tx_errors;
    uint8_t rx_errors;
    mcp251xfd_state_t state;
    bool running;
    bool faulted;
    bool filters_configured;
} mcp251xfd_core_t;

// Bit-time fields use the upstream library's register encoding (actual - 1).
eERRORRESULT mcp251xfd_calculate_timing(uint32_t clock_hz, uint32_t bitrate,
                                     uint16_t sample_point, MCP251XFD_BitTimeConfig *timing);
eERRORRESULT mcp251xfd_core_init(mcp251xfd_core_t *core);
eERRORRESULT mcp251xfd_core_enable(mcp251xfd_core_t *core);
// Always stops software activity and returns ownership of all accepted tokens,
// including when the controller cannot be reached. A faulted node must be recreated.
eERRORRESULT mcp251xfd_core_disable(mcp251xfd_core_t *core);
// Enter LPM after disabling. No SPI access is allowed afterwards until a fresh
// core_init() wakes and reconfigures the device; the caller discards this core.
eERRORRESULT mcp251xfd_core_sleep(mcp251xfd_core_t *core);
eERRORRESULT mcp251xfd_core_filter(mcp251xfd_core_t *core, uint8_t index,
                                 uint32_t id, uint32_t mask, bool extended);
eERRORRESULT mcp251xfd_core_enqueue(mcp251xfd_core_t *core,
                                  const mcp251xfd_frame_t *frame, const void *token);
eERRORRESULT mcp251xfd_core_service(mcp251xfd_core_t *core);
// Best-effort readback for bring-up logs. Call under the same lock as other core operations.
void mcp251xfd_core_read_registers(mcp251xfd_core_t *core, mcp251xfd_registers_t *registers);
