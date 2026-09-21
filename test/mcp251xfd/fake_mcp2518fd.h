#pragma once
#include "mcp251xfd_core.h"

// Register/RAM model at the SPI boundary. The production upstream driver runs
// unchanged above it; this model supplies controller behavior, not driver calls.
typedef struct {
    uint8_t memory[4096];
    unsigned tx_head, tx_tail, tx_count;
    unsigned rx_head, rx_tail, rx_count;
    unsigned tef_head, tef_tail, tef_count;
    unsigned mode;
    uint32_t spi_hz;
    uint32_t initial_spi_hz;
    unsigned transfers;
    unsigned fail_on_transfer;
    bool disconnected;
    bool freeze_mode;
    bool no_oscillator;
    bool pretend_mcp2517;
    bool low_power;
    unsigned wake_reads_remaining;
    uint32_t wire_ids[256];
    unsigned wire_count;
} fake_mcp2518fd_t;

void fake_init(fake_mcp2518fd_t *chip, mcp251xfd_core_t *core);
// Complete exactly one FIFO message, or exhaust one-shot attempts without TEF.
void fake_transmit(fake_mcp2518fd_t *chip, bool success);
void fake_receive(fake_mcp2518fd_t *chip, const mcp251xfd_frame_t *frame, bool fd);
uint32_t fake_read32(const fake_mcp2518fd_t *chip, unsigned address);
void fake_write32(fake_mcp2518fd_t *chip, unsigned address, uint32_t value);
void fake_bus_off(fake_mcp2518fd_t *chip, bool already_recovered);
