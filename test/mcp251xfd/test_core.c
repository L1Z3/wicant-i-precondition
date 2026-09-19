#include "fake_mcp2518fd.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    fake_mcp2518fd_t chip;
    mcp251xfd_core_t core;
    int tokens[128];
    const void *completed[128];
    bool success[128];
    unsigned completed_count;
    mcp251xfd_frame_t received[128];
    unsigned received_count;
    unsigned bus_off_count;
    unsigned error_count;
    uint32_t diagnostic;
} fixture_t;

static void tx_done(void *arg, const void *token, bool success)
{
    fixture_t *f = arg;
    assert(f->completed_count < 128);
    for (unsigned i = 0; i < f->completed_count; i++) assert(f->completed[i] != token);
    f->completed[f->completed_count] = token;
    f->success[f->completed_count++] = success;
}

static void rx_done(void *arg, const mcp251xfd_frame_t *frame)
{
    fixture_t *f = arg;
    assert(f->received_count < 128);
    f->received[f->received_count++] = *frame;
}

static void state_changed(void *arg, mcp251xfd_state_t old, mcp251xfd_state_t state)
{
    fixture_t *f = arg;
    assert(old != state);
    if (state == MCP251XFD_STATE_BUS_OFF) f->bus_off_count++;
}

static void bus_error(void *arg, uint32_t diagnostic, bool arbitration_lost)
{
    fixture_t *f = arg;
    (void)arbitration_lost;
    f->error_count++;
    f->diagnostic = diagnostic;
}

static void prepare(fixture_t *f)
{
    memset(f, 0, sizeof(*f));
    fake_init(&f->chip, &f->core);
    f->core.callbacks = (mcp251xfd_callbacks_t){.tx_done = tx_done, .rx_done = rx_done,
        .state_changed = state_changed, .error = bus_error, .arg = f};
}

static void start(fixture_t *f)
{
    eERRORRESULT error = mcp251xfd_core_init(&f->core);
    if (error != ERR_NONE) fprintf(stderr, "init error %d after %u SPI transfers, mode %u\n", error, f->chip.transfers, f->chip.mode);
    assert(error == ERR_NONE);
    assert(mcp251xfd_core_enable(&f->core) == ERR_NONE);
    assert(f->chip.mode == (f->core.config.loopback ? 2u : f->core.config.listen_only ? 3u : 6u));
}

static mcp251xfd_frame_t make_frame(unsigned index)
{
    mcp251xfd_frame_t frame = {.id = 0x700 - index, .dlc = index % 9};
    for (unsigned i = 0; i < 8; i++) frame.data[i] = index + i;
    return frame;
}

static void enqueue(fixture_t *f, unsigned index)
{
    mcp251xfd_frame_t frame = make_frame(index);
    assert(mcp251xfd_core_enqueue(&f->core, &frame, &f->tokens[index]) == ERR_NONE);
}

static void test_timing(void)
{
    const uint32_t rates[] = {5000, 10000, 20000, 25000, 50000, 100000, 125000, 250000, 500000, 800000, 1000000};
    for (unsigned i = 0; i < sizeof(rates) / sizeof(rates[0]); i++) {
        MCP251XFD_BitTimeConfig timing;
        assert(mcp251xfd_calculate_timing(40000000, rates[i], 875, &timing) == ERR_NONE);
        uint32_t total = timing.NTSEG1 + timing.NTSEG2 + 3;
        assert(40000000 == rates[i] * (timing.NBRP + 1) * total);
        uint32_t sample = (timing.NTSEG1 + 2) * 1000 / total;
        assert(sample == (rates[i] == 800000 ? 880u : 875u));
        assert(timing.NBRP <= 255 && timing.NTSEG1 >= 1 && timing.NTSEG1 <= 255);
        assert(timing.NTSEG2 <= 127 && timing.NSJW <= timing.NTSEG2);
    }
    MCP251XFD_BitTimeConfig timing;
    assert(mcp251xfd_calculate_timing(40000000, 500000, 750, &timing) == ERR_NONE);
    assert((timing.NTSEG1 + 2) * 1000 / (timing.NTSEG1 + timing.NTSEG2 + 3) == 750);
    assert(mcp251xfd_calculate_timing(40000000, 333333, 875, &timing) == ERR__BITTIME_ERROR);
    assert(mcp251xfd_calculate_timing(40000000, 0, 875, &timing) == ERR__PARAMETER_ERROR);
    assert(mcp251xfd_calculate_timing(40000000, 500000, 1000, &timing) == ERR__PARAMETER_ERROR);
    assert(mcp251xfd_calculate_timing(40000000, 500000, 875, NULL) == ERR__PARAMETER_ERROR);
}

static void test_initialization(void)
{
    fixture_t f;
    prepare(&f);
    start(&f);
    assert(f.chip.initial_spi_hz == 1000000 && f.chip.spi_hz == 10000000);
    assert((fake_read32(&f.chip, 0xe00) & 0x11) == 0); // No PLL or SYSCLK divider.
    assert(fake_read32(&f.chip, 0x004) == 0x00440909); // 500 kbit/s, 87.5% at 40 MHz.
    assert((fake_read32(&f.chip, 0x000) & (1u << 19)) != 0); // TEF enabled.
    assert((fake_read32(&f.chip, 0x000) & (1u << 20)) == 0); // TXQ disabled.
    assert((fake_read32(&f.chip, 0x040) >> 24 & 31) == 7);
    assert((fake_read32(&f.chip, 0x05c) >> 24 & 31) == 7);
    assert((fake_read32(&f.chip, 0x068) >> 24 & 31) == 31);
    assert((fake_read32(&f.chip, 0x05c) & 0x80) != 0);
    assert((fake_read32(&f.chip, 0x068) & 0x80) == 0);
    assert(f.chip.memory[0x1d0] == 0x82); // Default accept-all points to RX FIFO2.
    assert(mcp251xfd_core_enable(&f.core) == ERR__NOT_READY);
    assert(mcp251xfd_core_disable(&f.core) == ERR_NONE);
    assert(f.chip.mode == 4 && f.core.count == 0);

    prepare(&f);
    f.core.config.bitrate = 5000;
    start(&f);
    uint32_t reg = fake_read32(&f.chip, 4);
    assert(((reg >> 24) + 1) * (((reg >> 16) & 255) + ((reg >> 8) & 127) + 3) * 5000 == 40000000);
}

static void test_fifo_order_and_ownership(void)
{
    fixture_t f;
    prepare(&f);
    start(&f);
    f.core.next_sequence = 0x7ffffc; // Exercise the 23-bit sequence wrap.
    for (unsigned i = 0; i < 32; i++) enqueue(&f, i);
    mcp251xfd_frame_t extra = make_frame(32);
    assert(mcp251xfd_core_enqueue(&f.core, &extra, &f.tokens[32]) == ERR__OUT_OF_MEMORY);
    assert(f.completed_count == 0); // Enqueued is not transmitted.
    assert(mcp251xfd_core_service(&f.core) == ERR_NONE);
    assert(f.chip.tx_count == 8 && f.core.loaded == 8 && !f.completed_count);
    for (unsigned batch = 0; batch < 4; batch++) {
        for (unsigned i = 0; i < 8; i++) fake_transmit(&f.chip, true);
        assert(f.chip.tef_count == 8);
        assert(mcp251xfd_core_service(&f.core) == ERR_NONE);
        assert(f.completed_count == (batch + 1) * 8);
    }
    assert(f.core.count == 0 && f.core.loaded == 0 && f.chip.tef_count == 0);
    for (unsigned i = 0; i < 32; i++) {
        assert(f.completed[i] == &f.tokens[i] && f.success[i]);
        assert(f.chip.wire_ids[i] == 0x700 - i); // Descending IDs retain submission order.
    }
    for (unsigned i = 32; i < 64; i++) enqueue(&f, i);
    assert(mcp251xfd_core_service(&f.core) == ERR_NONE);
    while (f.core.count) {
        fake_transmit(&f.chip, true);
        assert(mcp251xfd_core_service(&f.core) == ERR_NONE);
    }
    assert(f.completed_count == 64);
    for (unsigned i = 32; i < 64; i++) assert(f.completed[i] == &f.tokens[i] && f.success[i]);
}

static void test_one_shot(void)
{
    fixture_t f;
    prepare(&f);
    f.core.config.one_shot = true;
    start(&f);
    for (unsigned i = 0; i < 32; i++) enqueue(&f, i);
    assert(mcp251xfd_core_service(&f.core) == ERR_NONE);
    for (unsigned i = 0; i < 32; i++) {
        assert(f.core.loaded == 1 && f.chip.tx_count == 1);
        fake_transmit(&f.chip, false);
        assert(mcp251xfd_core_service(&f.core) == ERR_NONE);
        assert(f.completed_count == i + 1 && !f.success[i]);
        assert(f.completed[i] == &f.tokens[i]);
    }
    assert(!f.core.count && !f.core.loaded && !f.chip.tx_count && !f.bus_off_count);
    assert(f.error_count == 32 && (f.diagnostic & (1u << 18)));
    enqueue(&f, 32);
    assert(mcp251xfd_core_service(&f.core) == ERR_NONE);
    fake_transmit(&f.chip, true);
    assert(mcp251xfd_core_service(&f.core) == ERR_NONE);
    assert(f.completed[32] == &f.tokens[32] && f.success[32]);
}

static void test_rx_and_remote_frames(void)
{
    fixture_t f;
    prepare(&f);
    f.core.config.loopback = true;
    start(&f);
    const uint32_t ids[] = {0, 0x7ff, 0, 0x1fffffff};
    for (unsigned i = 0; i < 4; i++) {
        mcp251xfd_frame_t frame = make_frame(i);
        frame.id = ids[i];
        frame.extended = i >= 2;
        frame.dlc = 8;
        frame.rtr = i % 2;
        assert(mcp251xfd_core_enqueue(&f.core, &frame, &f.tokens[i]) == ERR_NONE);
        assert(mcp251xfd_core_service(&f.core) == ERR_NONE);
        unsigned slot = f.chip.tx_tail;
        if (frame.rtr) {
            for (unsigned b = 0; b < 8; b++) assert(f.chip.memory[0x440 + slot * 16 + 8 + b] == 0);
        }
        fake_transmit(&f.chip, true);
        fake_receive(&f.chip, &frame, false);
        assert(mcp251xfd_core_service(&f.core) == ERR_NONE);
        assert(f.chip.wire_ids[i] == ids[i]);
        assert(f.received[i].id == ids[i] && f.received[i].extended == frame.extended);
        assert(f.received[i].dlc == 8 && f.received[i].rtr == frame.rtr);
        if (!frame.rtr) assert(memcmp(f.received[i].data, frame.data, 8) == 0);
        else for (unsigned b = 0; b < 8; b++) assert(f.received[i].data[b] == 0);
    }
    mcp251xfd_frame_t fd = make_frame(0);
    fd.dlc = 15;
    fake_receive(&f.chip, &fd, true);
    assert(mcp251xfd_core_service(&f.core) == ERR_NONE);
    assert(f.received_count == 4 && f.core.rx_fd_dropped == 1);
    for (unsigned i = 0; i < 33; i++) fake_receive(&f.chip, &fd, false);
    assert(mcp251xfd_core_service(&f.core) == ERR_NONE);
    assert(f.core.rx_overruns == 1 && f.received_count == 36 && f.chip.rx_count == 0);
    assert(f.received[4].dlc == 8); // Classical DLC >8 maps to eight data bytes.
}

static void test_filters_and_listen_only(void)
{
    fixture_t f;
    prepare(&f);
    f.core.config.listen_only = true;
    assert(mcp251xfd_core_init(&f.core) == ERR_NONE);
    assert(mcp251xfd_core_filter(&f.core, 1, 0x123, 0x7f0, false) == ERR_NONE);
    assert((f.chip.memory[0x1d0] & 0x80) == 0); // No leftover accept-all bypass.
    assert(f.chip.memory[0x1d1] == 0x82);
    assert(fake_read32(&f.chip, 0x1f8) == 0x120); // Masked-off ID bits normalized.
    assert(fake_read32(&f.chip, 0x1fc) == (0x7f0 | 1u << 30));
    assert(mcp251xfd_core_filter(&f.core, 0, 0x1fffffff, 0x1fffffff, true) == ERR_NONE);
    assert(fake_read32(&f.chip, 0x1f0) == 0x5fffffff);
    assert(fake_read32(&f.chip, 0x1f4) == 0x5fffffff);
    assert(mcp251xfd_core_filter(&f.core, 32, 0, 0, false) == ERR__PARAMETER_ERROR);
    assert(mcp251xfd_core_filter(&f.core, 0, 0x800, 0x7ff, false) == ERR__PARAMETER_ERROR);
    assert(mcp251xfd_core_enable(&f.core) == ERR_NONE && f.chip.mode == 3);
    mcp251xfd_frame_t frame = make_frame(0);
    assert(mcp251xfd_core_enqueue(&f.core, &frame, &f.tokens[0]) == ERR__NOT_SUPPORTED);
    assert(mcp251xfd_core_filter(&f.core, 0, 0, 0, false) == ERR__NEED_CONFIG_MODE);
}

static void test_stop_and_faults(void)
{
    fixture_t f;
    for (unsigned mode = 0; mode < 6; mode++) {
        prepare(&f);
        start(&f);
        for (unsigned i = 0; i < 32; i++) enqueue(&f, i);
        assert(mcp251xfd_core_service(&f.core) == ERR_NONE);
        switch (mode) {
        case 0: assert(mcp251xfd_core_disable(&f.core) == ERR_NONE); break;
        case 1: fake_bus_off(&f.chip, false); break;
        case 2: fake_bus_off(&f.chip, true); break;
        case 3: f.chip.disconnected = true; break;
        case 4: f.chip.memory[0x044] |= 8; break; // TEF overflow.
        case 5:
            fake_transmit(&f.chip, true);
            fake_write32(&f.chip, 0x404, 0x12345u << 9); // Unknown completion sequence.
            break;
        }
        if (mode) {
            assert(mcp251xfd_core_service(&f.core) != ERR_NONE);
            assert(f.bus_off_count == 1 && f.core.faulted);
            assert(mcp251xfd_core_enable(&f.core) == ERR__NOT_READY);
        }
        assert(!f.core.running && !f.core.count && !f.core.loaded);
        assert(f.completed_count == 32);
        for (unsigned i = 0; i < 32; i++) assert(f.completed[i] == &f.tokens[i] && !f.success[i]);
        assert(mcp251xfd_core_service(&f.core) == ERR_NONE);
        assert(f.completed_count == 32); // No duplicate completions after stop.
    }
}

static void test_reenable_and_errors(void)
{
    fixture_t f;
    prepare(&f);
    start(&f);
    for (unsigned i = 0; i < 64; i++) {
        enqueue(&f, i);
        assert(mcp251xfd_core_service(&f.core) == ERR_NONE);
        assert(mcp251xfd_core_disable(&f.core) == ERR_NONE);
        assert(mcp251xfd_core_enable(&f.core) == ERR_NONE);
    }
    assert(f.completed_count == 64 && !f.bus_off_count);
    // Inject a failure at every SPI operation in initialization.
    unsigned init_transfers;
    prepare(&f);
    assert(mcp251xfd_core_init(&f.core) == ERR_NONE);
    init_transfers = f.chip.transfers;
    for (unsigned i = 1; i <= init_transfers; i++) {
        prepare(&f);
        f.chip.fail_on_transfer = i;
        assert(mcp251xfd_core_init(&f.core) != ERR_NONE);
        assert(!f.core.running);
    }
    prepare(&f);
    f.chip.no_oscillator = true;
    assert(mcp251xfd_core_init(&f.core) == ERR__DEVICE_TIMEOUT);
    prepare(&f);
    f.chip.pretend_mcp2517 = true;
    assert(mcp251xfd_core_init(&f.core) == ERR__UNKNOWN_DEVICE);
    prepare(&f);
    start(&f);
    f.chip.freeze_mode = true;
    enqueue(&f, 0);
    assert(mcp251xfd_core_disable(&f.core) == ERR__DEVICE_TIMEOUT);
    assert(f.core.faulted && !f.core.count && f.completed_count == 1);

    // Inject failures throughout a service pass that consumes a completion,
    // receives a frame, and submits the next frame. No token may leak or be
    // completed twice, whether hardware submission had happened or not.
    unsigned service_transfers = 0;
    for (unsigned failure = 0; failure <= service_transfers; failure++) {
        prepare(&f);
        start(&f);
        enqueue(&f, 0);
        assert(mcp251xfd_core_service(&f.core) == ERR_NONE);
        fake_transmit(&f.chip, true);
        mcp251xfd_frame_t frame = make_frame(0);
        fake_receive(&f.chip, &frame, false);
        enqueue(&f, 1);
        unsigned before = f.chip.transfers;
        if (failure) f.chip.fail_on_transfer = before + failure;
        eERRORRESULT error = mcp251xfd_core_service(&f.core);
        if (!failure) {
            assert(error == ERR_NONE);
            service_transfers = f.chip.transfers - before;
        } else {
            assert(error != ERR_NONE);
            assert(!f.core.running && !f.core.count && f.completed_count == 2);
            assert(f.completed[0] == &f.tokens[0] && f.completed[1] == &f.tokens[1]);
            assert(!f.success[1]);
        }
    }
}

int main(void)
{
    test_timing();
    test_initialization();
    test_fifo_order_and_ownership();
    test_one_shot();
    test_rx_and_remote_frames();
    test_filters_and_listen_only();
    test_stop_and_faults();
    test_reenable_and_errors();
    puts("MCP2518FD core: timing, SPI initialization, FIFO order, TEF ownership, one-shot, RX/RTR, filters, and lifecycle tests passed");
    return 0;
}
