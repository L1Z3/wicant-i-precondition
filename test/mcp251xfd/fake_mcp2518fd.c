#include "fake_mcp2518fd.h"
#include <assert.h>
#include <string.h>

// Datasheet register offsets and message-object bit fields are deliberately
// encoded independently of upstream's C bitfields and serialization helpers.
enum { CON = 0x000, INT = 0x01c, TREC = 0x034, DIAG = 0x03c,
       TEFCON = 0x040, TEFSTA = 0x044, TEFUA = 0x048,
       TXCON = 0x05c, TXSTA = 0x060, TXUA = 0x064,
       RXCON = 0x068, RXSTA = 0x06c, RXUA = 0x070,
       TEF_RAM = 0x400, TX_RAM = 0x440, RX_RAM = 0x4c0, OSC = 0xe00 };

uint32_t fake_read32(const fake_mcp2518fd_t *chip, unsigned address)
{
    const uint8_t *p = &chip->memory[address];
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

void fake_write32(fake_mcp2518fd_t *chip, unsigned address, uint32_t value)
{
    for (unsigned i = 0; i < 4; i++) chip->memory[address + i] = value >> (8 * i);
}

static uint32_t encode_id(uint32_t id, bool extended)
{
    return extended ? (id >> 18) | ((id & 0x3ffff) << 11) : id;
}

static uint32_t decode_id(uint32_t id, bool extended)
{
    return extended ? ((id & 0x7ff) << 18) | ((id >> 11) & 0x3ffff) : id & 0x7ff;
}

static void reset_fifos(fake_mcp2518fd_t *chip)
{
    chip->tx_head = chip->tx_tail = chip->tx_count = 0;
    chip->rx_head = chip->rx_tail = chip->rx_count = 0;
    chip->tef_head = chip->tef_tail = chip->tef_count = 0;
    chip->memory[TXSTA] = chip->memory[RXSTA] = chip->memory[TEFSTA] = 0;
}

static void reset_chip(fake_mcp2518fd_t *chip)
{
    memset(chip->memory, 0, sizeof(chip->memory));
    reset_fifos(chip);
    chip->mode = 4;
    chip->memory[CON + 2] = 4 << 5;
    chip->memory[CON + 3] = 4;
}

static void refresh(fake_mcp2518fd_t *chip)
{
    chip->memory[OSC + 1] = chip->no_oscillator ? 0 : 4;
    if (chip->pretend_mcp2517) chip->memory[OSC] &= ~8u;
    chip->memory[CON + 2] = (chip->memory[CON + 2] & 0x1f) | chip->mode << 5;
    chip->memory[TXSTA] = (chip->memory[TXSTA] & 0xf0) | (chip->tx_count < 8 ? 1 : 0) | (!chip->tx_count ? 4 : 0);
    chip->memory[RXSTA] = (chip->memory[RXSTA] & 8) | (chip->rx_count ? 1 : 0) | (chip->rx_count == 32 ? 4 : 0);
    chip->memory[TEFSTA] = (chip->memory[TEFSTA] & 8) | (chip->tef_count ? 1 : 0) | (chip->tef_count == 8 ? 4 : 0);
    chip->memory[INT] = (chip->rx_count ? 2 : 0) | (chip->tef_count ? 16 : 0);
    chip->memory[INT + 1] = (chip->memory[INT + 1] & ~((1u << 2) | (1u << 3))) |
                           ((chip->memory[TXSTA] & 16) ? 4 : 0) | ((chip->memory[RXSTA] & 8) ? 8 : 0);
    fake_write32(chip, TEFUA, TEF_RAM - 0x400 + chip->tef_tail * 8);
    fake_write32(chip, TXUA, TX_RAM - 0x400 + chip->tx_head * 16);
    fake_write32(chip, RXUA, RX_RAM - 0x400 + chip->rx_tail * 16);
    chip->memory[TXSTA + 1] = chip->tx_tail;
}

static eERRORRESULT initialize_spi(void *arg, uint8_t select, uint32_t hz)
{
    (void)select;
    fake_mcp2518fd_t *chip = arg;
    if (!chip->initial_spi_hz) chip->initial_spi_hz = hz;
    chip->spi_hz = hz;
    return ERR_NONE;
}

static uint32_t current_ms(void)
{
    static uint32_t clock;
    return clock++;
}

static eERRORRESULT transfer(void *arg, uint8_t select, uint8_t *tx, uint8_t *rx, size_t size)
{
    (void)select;
    fake_mcp2518fd_t *chip = arg;
    chip->transfers++;
    if (chip->disconnected || chip->transfers == chip->fail_on_transfer) return ERR__SPI_COMM_ERROR;
    assert(size >= 2);
    unsigned command = tx[0] >> 4;
    unsigned address = (tx[0] & 15) * 256 + tx[1];
    assert(address + size - 2 <= sizeof(chip->memory));
    if (command == 0) {
        assert(size == 2 && address == 0);
        reset_chip(chip);
        return ERR_NONE;
    }
    refresh(chip);
    if (command == 3) {
        assert(rx);
        // The upstream driver uses the same TX/RX buffer.
        memset(rx, 0, 2);
        memcpy(rx + 2, chip->memory + address, size - 2);
        return ERR_NONE;
    }
    assert(command == 2 && !rx);
    memcpy(chip->memory + address, tx + 2, size - 2);
    if (address <= CON + 3 && address + size - 2 > CON + 3) {
        unsigned requested = chip->memory[CON + 3] & 7;
        if (!chip->freeze_mode && requested != chip->mode) {
            chip->mode = requested;
            reset_fifos(chip);
            chip->memory[TREC + 2] = requested == 4 ? 32 : 0;
            chip->memory[TEFCON + 1] = chip->memory[TXCON + 1] = chip->memory[RXCON + 1] = 0;
        }
    }
    if (size == 3) {
        uint8_t control = tx[2];
        if (address == TXCON + 1) {
            if (control & 4) {
                chip->tx_head = chip->tx_tail = chip->tx_count = 0;
                chip->memory[TXSTA] = 0;
                control = 0;
            }
            if (control & 1) {
                assert(chip->tx_count < 8);
                chip->tx_count++;
                chip->tx_head = (chip->tx_head + 1) % 8;
            }
            chip->memory[address] = control & 2;
        } else if (address == TEFCON + 1 && (control & 1)) {
            assert(chip->tef_count);
            chip->tef_count--;
            chip->tef_tail = (chip->tef_tail + 1) % 8;
            chip->memory[address] = 0;
        } else if (address == RXCON + 1 && (control & 1)) {
            assert(chip->rx_count);
            chip->rx_count--;
            chip->rx_tail = (chip->rx_tail + 1) % 32;
            chip->memory[address] = 0;
        }
    }
    refresh(chip);
    return ERR_NONE;
}

void fake_init(fake_mcp2518fd_t *chip, mcp251xfd_core_t *core)
{
    memset(chip, 0, sizeof(*chip));
    memset(core, 0, sizeof(*core));
    reset_chip(chip);
    core->device = (MCP251XFD){.InterfaceDevice = chip, .SPIClockSpeed = 10000000,
        .fnSPI_Init = initialize_spi, .fnSPI_Transfer = transfer, .fnGetCurrentms = current_ms};
    core->config = (mcp251xfd_config_t){.oscillator_hz = 40000000, .bitrate = 500000,
                                      .sample_point_permill = 875, .tx_queue_depth = 32};
}

void fake_transmit(fake_mcp2518fd_t *chip, bool success)
{
    assert(chip->tx_count && (chip->memory[TXCON + 1] & 2));
    assert(!(chip->memory[CON + 3] & 8)); // ABAT must have been released.
    unsigned source = TX_RAM + chip->tx_tail * 16;
    uint32_t control = fake_read32(chip, source + 4);
    if (!success) {
        chip->memory[TXCON + 1] = 0;
        chip->memory[TXSTA] |= 16 | 32;
        fake_write32(chip, DIAG, 1u << 18); // Missing ACK.
        return;
    }
    assert(chip->tef_count < 8);
    assert(chip->wire_count < 256);
    chip->wire_ids[chip->wire_count++] = decode_id(fake_read32(chip, source), (control & 16) != 0);
    unsigned destination = TEF_RAM + chip->tef_head * 8;
    memcpy(chip->memory + destination, chip->memory + source, 8);
    chip->tef_head = (chip->tef_head + 1) % 8;
    chip->tef_count++;
    chip->tx_tail = (chip->tx_tail + 1) % 8;
    chip->tx_count--;
    if (!chip->tx_count) chip->memory[TXCON + 1] = 0;
    refresh(chip);
}

void fake_receive(fake_mcp2518fd_t *chip, const mcp251xfd_frame_t *frame, bool fd)
{
    if (chip->rx_count == 32) {
        chip->memory[RXSTA] |= 8;
        return;
    }
    unsigned address = RX_RAM + chip->rx_head * 16;
    fake_write32(chip, address, encode_id(frame->id, frame->extended));
    fake_write32(chip, address + 4, frame->dlc | (frame->extended ? 16 : 0) | (frame->rtr ? 32 : 0) | (fd ? 128 : 0));
    memcpy(chip->memory + address + 8, frame->data, 8);
    chip->rx_head = (chip->rx_head + 1) % 32;
    chip->rx_count++;
    refresh(chip);
}

void fake_bus_off(fake_mcp2518fd_t *chip, bool already_recovered)
{
    chip->memory[TREC + 2] = already_recovered ? 0 : 32;
    chip->memory[INT + 1] |= 32;
    fake_write32(chip, DIAG, 1u << 23);
}
