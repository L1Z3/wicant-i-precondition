#include "mcp251xfd_core.h"
#include <string.h>

#define TX_FIFO MCP251XFD_FIFO1
#define RX_FIFO MCP251XFD_FIFO2
#define SEQUENCE_MASK 0x7fffffu
#define MODE_TIMEOUT_MS 100u
#define INTERRUPTS (MCP251XFD_INT_RX_EVENT | MCP251XFD_INT_TEF_EVENT | \
                    MCP251XFD_INT_TX_ATTEMPTS_EVENT | MCP251XFD_INT_RX_OVERFLOW_EVENT | \
                    MCP251XFD_INT_BUS_ERROR_EVENT | MCP251XFD_INT_RX_INVALID_MESSAGE_EVENT | \
                    MCP251XFD_INT_SYSTEM_ERROR_EVENT | MCP251XFD_INT_RAM_ECC_EVENT)

#define TRY(call) do { eERRORRESULT error_ = (call); if (error_ != ERR_NONE) return error_; } while (0)

eERRORRESULT mcp251xfd_calculate_timing(uint32_t clock_hz, uint32_t bitrate,
                                     uint16_t sample_point, MCP251XFD_BitTimeConfig *timing)
{
    if (!timing || clock_hz < 2000000 || clock_hz > 40000000 ||
        bitrate < 5000 || bitrate > 1000000 || sample_point >= 1000) {
        return ERR__PARAMETER_ERROR;
    }
    if (!sample_point) sample_point = 875;
    uint32_t best_error = UINT32_MAX;
    uint32_t best_total = 1;
    memset(timing, 0, sizeof(*timing));
    for (uint32_t brp = 1; brp <= 256; brp++) {
        uint32_t divisor = bitrate * brp;
        if (clock_hz % divisor) continue;
        uint32_t total = clock_hz / divisor;
        if (total < 4 || total > 385) continue;
        for (uint32_t seg2 = 1; seg2 <= 128 && seg2 + 2 < total; seg2++) {
            uint32_t seg1 = total - seg2 - 1;
            if (seg1 > 256) continue;
            int32_t difference = (int32_t)((total - seg2) * 1000) - (int32_t)(sample_point * total);
            uint32_t error = difference < 0 ? (uint32_t)-difference : (uint32_t)difference;
            // Compare fractions without rounding; ties prefer the smaller BRP.
            if ((uint64_t)error * best_total >= (uint64_t)best_error * total) continue;
            best_error = error;
            best_total = total;
            timing->NBRP = brp - 1;
            timing->NTSEG1 = seg1 - 1;
            timing->NTSEG2 = seg2 - 1;
            timing->NSJW = (seg2 < seg1 ? seg2 : seg1) - 1;
        }
    }
    return best_error == UINT32_MAX ? ERR__BITTIME_ERROR : ERR_NONE;
}

static eERRORRESULT set_mode(mcp251xfd_core_t *core, eMCP251XFD_OperationMode mode)
{
    // Upstream's synchronous wait assumes >=125 kbit/s and times out at 7 ms.
    // Classical CAN at 5 kbit/s can take much longer to finish the current frame.
    TRY(MCP251XFD_RequestOperationMode(&core->device, mode, false));
    uint32_t start = core->device.fnGetCurrentms();
    for (;;) {
        eMCP251XFD_OperationMode actual;
        TRY(MCP251XFD_GetActualOperationMode(&core->device, &actual));
        if (actual == mode) break;
        if ((uint32_t)(core->device.fnGetCurrentms() - start) >= MODE_TIMEOUT_MS) return ERR__DEVICE_TIMEOUT;
    }
    // RequestOperationMode sets ABAT. Release it explicitly before future TX.
    uint8_t control;
    TRY(MCP251XFD_ReadSFR8(&core->device, RegMCP251XFD_CiCON + 3, &control));
    return MCP251XFD_WriteSFR8(&core->device, RegMCP251XFD_CiCON + 3,
                             control & ~MCP251XFD_CAN_CiCON8_ABAT);
}

static eERRORRESULT wait_fifo_reset(mcp251xfd_core_t *core)
{
    const uint16_t addresses[] = {RegMCP251XFD_CiTEFCON_CONTROL,
                                 RegMCP251XFD_CiFIFOCONm_CONTROL,
                                 RegMCP251XFD_CiFIFOCONm_CONTROL + MCP251XFD_FIFO_REG_SIZE};
    uint32_t start = core->device.fnGetCurrentms();
    for (unsigned i = 0; i < sizeof(addresses) / sizeof(addresses[0]); i++) {
        for (;;) {
            uint8_t control;
            TRY(MCP251XFD_ReadSFR8(&core->device, addresses[i], &control));
            if (!(control & MCP251XFD_CAN_CiFIFOCONm8_FRESET)) break;
            if ((uint32_t)(core->device.fnGetCurrentms() - start) >= MODE_TIMEOUT_MS) return ERR__DEVICE_TIMEOUT;
        }
    }
    return ERR_NONE;
}

static eERRORRESULT wake_clock(mcp251xfd_core_t *core)
{
    // Asserting CS wakes MCP2518FD from LPM and resets its registers/RAM.
    // Discard the first read, then wait at the safe SPI speed before accessing
    // the CAN registers. The wake-up transaction need not return valid data.
    uint8_t status;
    TRY(MCP251XFD_ReadSFR8(&core->device, RegMCP251XFD_OSC + 1, &status));
    uint32_t start = core->device.fnGetCurrentms();
    for (;;) {
        TRY(MCP251XFD_ReadSFR8(&core->device, RegMCP251XFD_OSC + 1, &status));
        if (status != UINT8_MAX && (status & MCP251XFD_SFR_OSC8_OSCRDY)) return ERR_NONE;
        if ((uint32_t)(core->device.fnGetCurrentms() - start) >= MODE_TIMEOUT_MS) return ERR__DEVICE_TIMEOUT;
    }
}

static void complete_head(mcp251xfd_core_t *core, bool success)
{
    const void *token = core->tx[core->head].token;
    core->tx[core->head].token = NULL;
    core->head = (core->head + 1) % MCP251XFD_TX_CAPACITY;
    core->count--;
    if (core->loaded) core->loaded--;
    if (core->callbacks.tx_done) core->callbacks.tx_done(core->callbacks.arg, token, success);
}

static void cancel_all(mcp251xfd_core_t *core)
{
    while (core->count) complete_head(core, false);
    core->loaded = 0;
}

static void change_state(mcp251xfd_core_t *core, mcp251xfd_state_t state)
{
    mcp251xfd_state_t old = core->state;
    core->state = state;
    if (old != state && core->callbacks.state_changed) {
        core->callbacks.state_changed(core->callbacks.arg, old, state);
    }
}

void mcp251xfd_core_read_registers(mcp251xfd_core_t *core, mcp251xfd_registers_t *registers)
{
    *registers = (mcp251xfd_registers_t){0};
    // IOCON's multi-byte WRITE erratum does not affect this read-only snapshot.
    const uint16_t addresses[] = {RegMCP251XFD_CiCON, RegMCP251XFD_CiNBTCFG,
                                  RegMCP251XFD_OSC, RegMCP251XFD_IOCON_DIRECTION};
    uint32_t *values[] = {&registers->con, &registers->nbtcfg, &registers->osc, &registers->iocon};
    for (unsigned i = 0; i < sizeof(addresses) / sizeof(addresses[0]); i++) {
        uint32_t value;
        if (MCP251XFD_ReadSFR32(&core->device, addresses[i], &value) == ERR_NONE) {
            *values[i] = value;
            registers->valid |= 1u << i;
        }
    }
}

static eERRORRESULT fault(mcp251xfd_core_t *core, eERRORRESULT error)
{
    core->running = false;
    core->faulted = true;
    core->diagnostics.queued = core->count;
    core->diagnostics.loaded = core->loaded;
    mcp251xfd_core_read_registers(core, &core->diagnostics.registers);
    // Best effort: even an unreachable controller cannot retain software tokens.
    MCP251XFD_ConfigureInterrupt(&core->device, MCP251XFD_INT_NO_EVENT);
    set_mode(core, MCP251XFD_CONFIGURATION_MODE);
    cancel_all(core);
    change_state(core, MCP251XFD_STATE_BUS_OFF);
    return error;
}

eERRORRESULT mcp251xfd_core_init(mcp251xfd_core_t *core)
{
    if (!core || !core->device.fnSPI_Init || !core->device.fnSPI_Transfer ||
        !core->device.fnGetCurrentms || core->config.tx_queue_depth == 0 ||
        core->config.tx_queue_depth > MCP251XFD_TX_CAPACITY ||
        (core->config.listen_only && core->config.loopback)) return ERR__PARAMETER_ERROR;
    MCP251XFD_BitTimeConfig timing;
    TRY(mcp251xfd_calculate_timing(core->config.oscillator_hz, core->config.bitrate,
                                 core->config.sample_point_permill, &timing));
    if (!core->device.SPIClockSpeed ||
        core->device.SPIClockSpeed > (core->config.oscillator_hz / 2) * 85 / 100) {
        return ERR__SPI_FREQUENCY_ERROR;
    }
    core->state = MCP251XFD_STATE_BUS_OFF;
    // Enter configuration using our longer wait before upstream's safe reset.
    TRY(core->device.fnSPI_Init(core->device.InterfaceDevice, core->device.SPI_ChipSelect,
                               MCP251XFD_DRIVER_SAFE_RESET_SPI_CLK));
    TRY(wake_clock(core));
    TRY(set_mode(core, MCP251XFD_CONFIGURATION_MODE));
    core->device.DriverConfig = MCP251XFD_DRIVER_SAFE_RESET | MCP251XFD_DRIVER_ENABLE_ECC |
                                MCP251XFD_DRIVER_INIT_SET_RAM_AT_0 | MCP251XFD_DRIVER_CLEAR_BUFFER_BEFORE_READ;
    MCP251XFD_Config config = {
        .OscFreq = core->config.oscillator_hz,
        .SysclkConfig = MCP251XFD_SYSCLK_IS_CLKIN,
        .ClkoPinConfig = MCP251XFD_CLKO_DivBy10,
        // Upstream initialization accepts >=125 kbit/s. This temporary timing
        // is replaced while still in configuration mode, before joining the bus.
        .NominalBitrate = 500000,
        .DataBitrate = MCP251XFD_NO_CANFD,
        .Bandwidth = MCP251XFD_NO_DELAY,
        .ControlFlags = MCP251XFD_CAN_RESTRICTED_RETRANS_ATTEMPTS | MCP251XFD_CANFD_BITRATE_SWITCHING_DISABLE,
        .GPIO0PinMode = MCP251XFD_PIN_AS_GPIO0_IN,
        .GPIO1PinMode = MCP251XFD_PIN_AS_GPIO1_IN,
        .INTsOutMode = MCP251XFD_PINS_OPENDRAIN_OUT,
        .TXCANOutMode = MCP251XFD_PINS_PUSHPULL_OUT,
        .SysInterruptFlags = MCP251XFD_INT_NO_EVENT,
    };
    TRY(Init_MCP251XFD(&core->device, &config));
    eMCP251XFD_Devices device;
    TRY(MCP251XFD_GetDeviceID(&core->device, &device, NULL, NULL));
    if (device != MCP2518FD) return ERR__UNKNOWN_DEVICE;
    TRY(MCP251XFD_SetBitTimeConfiguration(&core->device, &timing, true));
    // 704 bytes total: TEF 8x8, TX 8x16, RX 32x16. No TXQ: it sorts by ID.
    MCP251XFD_FIFO fifos[] = {
        {.Name = MCP251XFD_TEF, .Size = MCP251XFD_FIFO_8_MESSAGE_DEEP,
         .InterruptFlags = MCP251XFD_FIFO_EVENT_FIFO_NOT_EMPTY_INT | MCP251XFD_FIFO_OVERFLOW_INT},
        {.Name = TX_FIFO, .Size = MCP251XFD_FIFO_8_MESSAGE_DEEP, .Payload = MCP251XFD_PAYLOAD_8BYTE,
         .Direction = MCP251XFD_TRANSMIT_FIFO,
         .Attempts = core->config.one_shot ? MCP251XFD_DISABLE_ATTEMPS : MCP251XFD_UNLIMITED_ATTEMPTS,
         .InterruptFlags = MCP251XFD_FIFO_TX_ATTEMPTS_EXHAUSTED_INT},
        {.Name = RX_FIFO, .Size = MCP251XFD_FIFO_32_MESSAGE_DEEP, .Payload = MCP251XFD_PAYLOAD_8BYTE,
         .Direction = MCP251XFD_RECEIVE_FIFO,
         .InterruptFlags = MCP251XFD_FIFO_RECEIVE_FIFO_NOT_EMPTY_INT | MCP251XFD_FIFO_OVERFLOW_INT},
    };
    TRY(MCP251XFD_ConfigureFIFOList(&core->device, fifos, sizeof(fifos) / sizeof(fifos[0])));
    MCP251XFD_Filter accept_all = {
        .Filter = MCP251XFD_FILTER0, .EnableFilter = true,
        .Match = MCP251XFD_MATCH_SID_EID, .PointTo = RX_FIFO,
    };
    return MCP251XFD_ConfigureFilter(&core->device, &accept_all);
}

eERRORRESULT mcp251xfd_core_enable(mcp251xfd_core_t *core)
{
    if (core->running || core->faulted) return ERR__NOT_READY;
    eMCP251XFD_OperationMode mode = core->config.loopback ? MCP251XFD_INTERNAL_LOOPBACK_MODE :
                                  core->config.listen_only ? MCP251XFD_LISTEN_ONLY_MODE : MCP251XFD_NORMAL_CAN20_MODE;
    eERRORRESULT error = MCP251XFD_ClearBusDiagnostic(&core->device, true, true);
    if (error == ERR_NONE) error = set_mode(core, mode);
    if (error == ERR_NONE) error = wait_fifo_reset(core);
    if (error == ERR_NONE) error = MCP251XFD_ConfigureInterrupt(&core->device, INTERRUPTS);
    if (error != ERR_NONE) {
        // No state callback before enable returns: WiCAN has not published ON yet.
        core->faulted = true;
        MCP251XFD_ConfigureInterrupt(&core->device, MCP251XFD_INT_NO_EVENT);
        set_mode(core, MCP251XFD_CONFIGURATION_MODE);
        return error;
    }
    core->running = true;
    core->state = MCP251XFD_STATE_ACTIVE;
    core->bus_errors = 0;
    core->tx_errors = core->rx_errors = 0;
    core->diagnostics = (mcp251xfd_diagnostics_t){0};
    return ERR_NONE;
}

eERRORRESULT mcp251xfd_core_disable(mcp251xfd_core_t *core)
{
    core->running = false;
    eERRORRESULT error = MCP251XFD_ConfigureInterrupt(&core->device, MCP251XFD_INT_NO_EVENT);
    eERRORRESULT mode_error = set_mode(core, MCP251XFD_CONFIGURATION_MODE);
    if (error == ERR_NONE) error = mode_error;
    if (error != ERR_NONE) core->faulted = true;
    // Disabling cancels ownership, not a claim that every frame was unsent:
    // a just-completed frame may not have had its TEF event serviced yet.
    cancel_all(core);
    core->state = MCP251XFD_STATE_BUS_OFF;
    return error;
}

eERRORRESULT mcp251xfd_core_sleep(mcp251xfd_core_t *core)
{
    if (core->running || core->count) return ERR__NOT_READY;
    TRY(MCP251XFD_ConfigureInterrupt(&core->device, MCP251XFD_INT_NO_EVENT));
    TRY(set_mode(core, MCP251XFD_CONFIGURATION_MODE));
    // WiCAN wakes on its battery-check timer and recreates the node. Disable
    // CAN wake interrupts; LPM discards configuration and RAM on wake-up.
    TRY(MCP251XFD_ConfigureSleepMode(&core->device, true, MCP251XFD_NO_FILTER, false));
    // This must be the last SPI access: polling OPMOD would wake it again.
    return MCP251XFD_EnterSleepMode(&core->device);
}

eERRORRESULT mcp251xfd_core_filter(mcp251xfd_core_t *core, uint8_t index,
                                 uint32_t id, uint32_t mask, bool extended)
{
    if (core->running || core->faulted) return ERR__NEED_CONFIG_MODE;
    uint32_t id_mask = extended ? 0x1fffffffu : 0x7ffu;
    if (index >= 32 || (id & ~id_mask) || (mask & ~id_mask)) return ERR__PARAMETER_ERROR;
    if (!core->filters_configured) {
        TRY(MCP251XFD_DisableFilter(&core->device, MCP251XFD_FILTER0));
        core->filters_configured = true;
    }
    MCP251XFD_Filter filter = {
        .Filter = (eMCP251XFD_Filter)index, .EnableFilter = true,
        .Match = extended ? MCP251XFD_MATCH_ONLY_EID : MCP251XFD_MATCH_ONLY_SID,
        .PointTo = RX_FIFO, .AcceptanceID = id & mask, .AcceptanceMask = mask,
        .ExtendedID = extended,
    };
    return MCP251XFD_ConfigureFilter(&core->device, &filter);
}

eERRORRESULT mcp251xfd_core_enqueue(mcp251xfd_core_t *core,
                                  const mcp251xfd_frame_t *frame, const void *token)
{
    if (!frame || !token || frame->dlc > 8 || frame->id > (frame->extended ? 0x1fffffffu : 0x7ffu)) {
        return ERR__PARAMETER_ERROR;
    }
    if (!core->running) return ERR__NOT_READY;
    if (core->config.listen_only) return ERR__NOT_SUPPORTED;
    if (core->count == core->config.tx_queue_depth) return ERR__OUT_OF_MEMORY;
    unsigned tail = (core->head + core->count) % MCP251XFD_TX_CAPACITY;
    core->tx[tail] = (mcp251xfd_pending_tx_t){.frame = *frame, .token = token,
                                           .sequence = core->next_sequence++ & SEQUENCE_MASK};
    core->count++;
    return ERR_NONE;
}

static uint32_t read_le32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

static void write_le32(uint8_t *p, uint32_t value)
{
    for (unsigned i = 0; i < 4; i++) p[i] = value >> (i * 8);
}

static eERRORRESULT read_fifo_head(mcp251xfd_core_t *core, eMCP251XFD_FIFO fifo,
                                  uint8_t *status, uint16_t *address)
{
    // STA and UA are adjacent. Read them together instead of issuing a second
    // transaction for UA through the vendor's generic message helper. UA is
    // owned by software UINC, so it remains valid until we advance this FIFO.
    uint16_t reg = fifo == MCP251XFD_TEF ? RegMCP251XFD_CiTEFSTA :
                   RegMCP251XFD_CiFIFOSTAm + MCP251XFD_FIFO_REG_SIZE * (fifo - 1);
    uint8_t registers[8];
    TRY(MCP251XFD_ReadData(&core->device, reg, registers, sizeof(registers)));
    *status = registers[0];
    uint32_t offset = read_le32(registers + 4);
    unsigned size = fifo == MCP251XFD_TEF ? 8 : 16;
    if ((offset & 3) || offset > MCP251XFD_RAM_SIZE - size) return ERR__SPI_INVALID_DATA;
    *address = MCP251XFD_RAM_ADDR + offset;
    return ERR_NONE;
}

static eERRORRESULT drain_tef(mcp251xfd_core_t *core)
{
    // Never load more than eight frames without reading their TEF entries,
    // even if hardware TX FIFO space has already become available.
    for (unsigned i = 0; i < MCP251XFD_HW_TX_DEPTH; i++) {
        uint8_t status;
        uint16_t address;
        TRY(read_fifo_head(core, MCP251XFD_TEF, &status, &address));
        if (status & MCP251XFD_TEF_FIFO_OVERFLOW) {
            core->diagnostics.reason = "TEF overflow";
            return ERR__BUFFER_FULL;
        }
        if (!(status & MCP251XFD_TEF_FIFO_NOT_EMPTY)) return ERR_NONE;
        uint8_t object[8];
        TRY(MCP251XFD_ReadData(&core->device, address, object, sizeof(object)));
        TRY(MCP251XFD_UpdateFIFO(&core->device, MCP251XFD_TEF, false));
        MCP251XFD_CAN_TX_Message_Control control = {.T1 = read_le32(object + 4)};
        if (!core->loaded || !core->count || control.SEQ != core->tx[core->head].sequence) {
            core->diagnostics.reason = "TEF sequence mismatch";
            return ERR__SPI_INVALID_DATA;
        }
        complete_head(core, true);
    }
    return ERR_NONE;
}

static eERRORRESULT fill_tx(mcp251xfd_core_t *core)
{
    unsigned max_loaded = core->config.one_shot ? 1 : MCP251XFD_HW_TX_DEPTH;
    while (core->loaded < core->count && core->loaded < max_loaded) {
        uint8_t status;
        uint16_t address;
        TRY(read_fifo_head(core, TX_FIFO, &status, &address));
        // A one-shot attempt can fail between the service status read and here.
        if (status & MCP251XFD_TX_FIFO_ATTEMPTS_EXHAUSTED) break;
        if (!(status & MCP251XFD_TX_FIFO_NOT_FULL)) break;
        const mcp251xfd_pending_tx_t *tx = &core->tx[(core->head + core->loaded) % MCP251XFD_TX_CAPACITY];
        uint8_t object[16] = {0};
        write_le32(object, MCP251XFD_MessageIDtoObjectMessageIdentifier(tx->frame.id, tx->frame.extended, false));
        MCP251XFD_CAN_TX_Message_Control control = {
            .SEQ = tx->sequence, .DLC = tx->frame.dlc, .IDE = tx->frame.extended, .RTR = tx->frame.rtr,
        };
        write_le32(object + 4, control.T1);
        if (!tx->frame.rtr) memcpy(object + 8, tx->frame.data, tx->frame.dlc);
        unsigned size = 8 + ((tx->frame.dlc + 3) & ~3u);
        TRY(MCP251XFD_WriteData(&core->device, address, object, size));
        // A failed transfer may already have committed UINC/TXREQ. Never retry
        // ambiguously: fault() retires the tokens and requests recreation.
        TRY(MCP251XFD_UpdateFIFO(&core->device, TX_FIFO, true));
        core->loaded++;
    }
    return ERR_NONE;
}

static eERRORRESULT service(mcp251xfd_core_t *core)
{
    core->diagnostics.reason = "controller I/O";
    core->diagnostics.trec_valid = core->diagnostics.bdiag1_valid = false;
    setMCP251XFD_InterruptEvents events = 0;
    MCP251XFD_CiBDIAG1_Register diagnostic = {0};
    TRY(MCP251XFD_GetInterruptEvents(&core->device, &events));
    core->diagnostics.interrupts = events;
    // New events after this snapshot leave INT asserted for the next pass.
    // TX submissions should not poll empty receive/completion FIFOs each time.
    if (events & MCP251XFD_INT_TEF_EVENT) TRY(drain_tef(core));
    // Read counters and status together, and keep the triggering values before
    // cleanup changes mode (configuration mode itself sets the TXBO bit).
    uint8_t counters[12]; // TREC, BDIAG0, BDIAG1 are contiguous.
    TRY(MCP251XFD_ReadData(&core->device, RegMCP251XFD_CiTREC, counters, sizeof(counters)));
    uint32_t trec = read_le32(counters);
    core->diagnostics.trec = trec;
    core->diagnostics.trec_valid = true;
    core->tx_errors = trec >> 8;
    core->rx_errors = trec;
    eMCP251XFD_TXRXErrorStatus status = (trec >> 16) & 0x3f;
    diagnostic.CiBDIAG1 = read_le32(counters + 8);
    core->diagnostics.bdiag1 = diagnostic.CiBDIAG1;
    core->diagnostics.bdiag1_valid = true;
    core->diagnostics.bdiag1_seen |= diagnostic.CiBDIAG1 & 0xffff0000u;
    // TXBOERR latches a bus-off even if the controller recovered before polling.
    if ((status & MCP251XFD_TX_BUS_OFF_STATE) || diagnostic.Bits.TXBOERR) {
        core->diagnostics.reason = "bus-off status";
        return ERR__NOT_READY;
    }
    if (events & (MCP251XFD_INT_SYSTEM_ERROR_EVENT | MCP251XFD_INT_RAM_ECC_EVENT)) {
        core->diagnostics.reason = "controller system/ECC error";
        return ERR__SPI_INVALID_DATA;
    }
    change_state(core, (status & (MCP251XFD_TX_BUS_PASSIVE_STATE | MCP251XFD_RX_BUS_PASSIVE_STATE)) ? MCP251XFD_STATE_PASSIVE :
                       (status & (MCP251XFD_TX_WARNING_STATE | MCP251XFD_RX_WARNING_STATE)) ? MCP251XFD_STATE_WARNING : MCP251XFD_STATE_ACTIVE);
    setMCP251XFD_FIFOstatus tx_status = 0;
    TRY(MCP251XFD_GetFIFOStatus(&core->device, TX_FIFO, &tx_status));
    if (events & (MCP251XFD_INT_BUS_ERROR_EVENT | MCP251XFD_INT_RX_INVALID_MESSAGE_EVENT | MCP251XFD_INT_TX_ATTEMPTS_EVENT)) {
        core->bus_errors++;
        if (core->callbacks.error) {
            core->callbacks.error(core->callbacks.arg, diagnostic.CiBDIAG1,
                                  (tx_status & MCP251XFD_TX_FIFO_ARBITRATION_LOST) != 0);
        }
        TRY(MCP251XFD_ClearBusDiagnostic(&core->device, false, true));
        TRY(MCP251XFD_ClearInterruptEvents(&core->device, events & MCP251XFD_INT_CLEARABLE_FLAGS_MASK));
    }
    if (tx_status & MCP251XFD_TX_FIFO_ATTEMPTS_EXHAUSTED) {
        // Failed one-shot frames produce no TEF entry and remain at the FIFO
        // tail. Only one was loaded: resetting cannot discard another frame.
        if (!core->config.one_shot || core->loaded != 1) {
            core->diagnostics.reason = "unexpected TX attempts exhausted";
            return ERR__SPI_INVALID_DATA;
        }
        TRY(MCP251XFD_ResetFIFO(&core->device, TX_FIFO));
        complete_head(core, false);
    }
    // Refill before draining RX so a receive burst cannot leave the TX FIFO
    // idle. The depth still bounds pending TEF events and preserves wire order.
    TRY(fill_tx(core));
    if (!(events & (MCP251XFD_INT_RX_EVENT | MCP251XFD_INT_RX_OVERFLOW_EVENT))) return ERR_NONE;
    for (unsigned i = 0; i < 32; i++) {
        uint8_t rx_status;
        uint16_t address;
        TRY(read_fifo_head(core, RX_FIFO, &rx_status, &address));
        if (rx_status & MCP251XFD_RX_FIFO_OVERFLOW) {
            core->rx_overruns++;
            TRY(MCP251XFD_ClearFIFOEvents(&core->device, RX_FIFO, MCP251XFD_RX_FIFO_OVERFLOW));
        }
        if (!(rx_status & MCP251XFD_RX_FIFO_NOT_EMPTY)) break;
        mcp251xfd_frame_t frame = {0};
        uint8_t object[16];
        TRY(MCP251XFD_ReadData(&core->device, address, object, sizeof(object)));
        TRY(MCP251XFD_UpdateFIFO(&core->device, RX_FIFO, false));
        MCP251XFD_CAN_RX_Message_Control control = {.R1 = read_le32(object + 4)};
        if (control.FDF) {
            core->rx_fd_dropped++;
            continue;
        }
        frame.extended = control.IDE;
        frame.id = MCP251XFD_ObjectMessageIdentifierToMessageID(read_le32(object), frame.extended, false);
        frame.rtr = control.RTR;
        frame.dlc = control.DLC > 8 ? 8 : control.DLC;
        if (!frame.rtr) memcpy(frame.data, object + 8, frame.dlc);
        if (core->callbacks.rx_done) core->callbacks.rx_done(core->callbacks.arg, &frame);
    }
    return ERR_NONE;
}

eERRORRESULT mcp251xfd_core_service(mcp251xfd_core_t *core)
{
    if (!core->running) return ERR_NONE;
    eERRORRESULT error = service(core);
    return error == ERR_NONE ? ERR_NONE : fault(core, error);
}
