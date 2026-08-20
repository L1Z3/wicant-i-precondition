// Host-side framing and timing tests for the deadline-driven ISO-TP sender.
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "test_support.h"
#include "can.h"
#include "isotp_tx.h"

typedef struct {
    can_bus_t bus;
    twai_message_t msg;
} sent_t;

static sent_t sent[1024];
static size_t sent_count;
static int fail_send_number = -1;

esp_err_t can_send(can_bus_t bus, twai_message_t *message, TickType_t ticks_to_wait) {
    (void)ticks_to_wait;
    int send_number = (int)sent_count;
    if (sent_count < sizeof(sent) / sizeof(sent[0])) {
        sent[sent_count] = (sent_t){ .bus = bus, .msg = *message };
    }
    sent_count++;
    return send_number == fail_send_number ? -1 : 0;
}

static isotp_tx_t tx;

static void init_tx(void) {
    memset(&tx, 0, sizeof(tx));
    memset(sent, 0, sizeof(sent));
    sent_count = 0U;
    fail_send_number = -1;
    const isotp_tx_config_t config = {
        .bus = CAN_BUS_0,
        .tx_id = 0x6E0U,
        .flow_control_id = 0x6BEU,
        .flow_control_timeout_us = 1000000U,
        .can_send_wait_ticks = 1U,
        .max_wait_frames = 3U,
        .padding_byte = 0xAAU,
    };
    isotp_tx_init(&tx, "test-isotp", &config);
}

static twai_message_t flow_control(uint8_t status, uint8_t block_size, uint8_t stmin) {
    twai_message_t msg = {0};
    msg.identifier = 0x6BEU;
    msg.data_length_code = 8U;
    msg.data[0] = 0x30U | status;
    msg.data[1] = block_size;
    msg.data[2] = stmin;
    return msg;
}

static void test_single_frame(void) {
    init_tx();
    const uint8_t payload[] = {1U, 2U, 3U};
    CHECK(isotp_tx_start(&tx, payload, sizeof(payload)));
    CHECK(isotp_tx_busy(&tx));
    CHECK(sent_count == 0U);
    isotp_tx_tick(&tx);
    CHECK(sent_count == 1U);
    CHECK(sent[0].bus == CAN_BUS_0);
    CHECK(sent[0].msg.identifier == 0x6E0U);
    CHECK(sent[0].msg.data_length_code == 8U);
    const uint8_t expected[] = {0x03U, 1U, 2U, 3U, 0xAAU, 0xAAU, 0xAAU, 0xAAU};
    CHECK(memcmp(sent[0].msg.data, expected, sizeof(expected)) == 0);
    CHECK(!isotp_tx_busy(&tx));
    CHECK(isotp_tx_result(&tx) == ISOTP_TX_RESULT_SUCCESS);
    CHECK(isotp_tx_next_deadline_us(&tx) == ISOTP_TX_NO_DEADLINE);
}

static void test_flow_control_and_timing(void) {
    init_tx();
    uint8_t payload[30];
    for (size_t i = 0U; i < sizeof(payload); i++) payload[i] = (uint8_t)i;

    CHECK(isotp_tx_start(&tx, payload, sizeof(payload)));
    isotp_tx_tick(&tx);
    CHECK(sent_count == 1U);
    CHECK(sent[0].msg.data[0] == 0x10U);
    CHECK(sent[0].msg.data[1] == sizeof(payload));
    CHECK(memcmp(&sent[0].msg.data[2], payload, 6U) == 0);
    CHECK(isotp_tx_next_deadline_us(&tx) == fake_now + 1000000);

    twai_message_t wrong = flow_control(0U, 0U, 0U);
    wrong.identifier = 0x123U;
    CHECK(!isotp_tx_rx(&tx, &wrong, CAN_BUS_0));

    twai_message_t fc = flow_control(0U, 2U, 5U);
    CHECK(isotp_tx_rx(&tx, &fc, CAN_BUS_0));
    CHECK(isotp_tx_next_deadline_us(&tx) == fake_now);
    isotp_tx_tick(&tx);
    CHECK(sent_count == 2U);
    CHECK(sent[1].msg.data[0] == 0x21U);
    CHECK(memcmp(&sent[1].msg.data[1], &payload[6], 7U) == 0);

    isotp_tx_tick(&tx);
    CHECK(sent_count == 2U);
    fake_now += 4999;
    isotp_tx_tick(&tx);
    CHECK(sent_count == 2U);
    fake_now++;
    isotp_tx_tick(&tx);
    CHECK(sent_count == 3U);
    CHECK(sent[2].msg.data[0] == 0x22U);
    CHECK(isotp_tx_next_deadline_us(&tx) == fake_now + 1000000);

    // A new block may choose a different STmin. F1 means 100 us.
    fc = flow_control(0U, 0U, 0xF1U);
    CHECK(isotp_tx_rx(&tx, &fc, CAN_BUS_0));
    isotp_tx_tick(&tx);
    CHECK(sent_count == 4U);
    CHECK(sent[3].msg.data[0] == 0x23U);
    fake_now += 99;
    isotp_tx_tick(&tx);
    CHECK(sent_count == 4U);
    fake_now++;
    isotp_tx_tick(&tx);
    CHECK(sent_count == 5U);
    CHECK(sent[4].msg.data[0] == 0x24U);
    CHECK(!isotp_tx_busy(&tx));
    CHECK(isotp_tx_result(&tx) == ISOTP_TX_RESULT_SUCCESS);
}

static void test_sequence_rollover(void) {
    init_tx();
    uint8_t payload[125];
    for (size_t i = 0U; i < sizeof(payload); i++) payload[i] = (uint8_t)i;
    CHECK(isotp_tx_start(&tx, payload, sizeof(payload)));
    isotp_tx_tick(&tx);
    twai_message_t fc = flow_control(0U, 0U, 0U);
    CHECK(isotp_tx_rx(&tx, &fc, CAN_BUS_0));
    while (isotp_tx_busy(&tx)) {
        isotp_tx_tick(&tx);
    }
    CHECK(sent_count == 18U); // one FF + 17 CFs
    CHECK(sent[15].msg.data[0] == 0x2FU);
    CHECK(sent[16].msg.data[0] == 0x20U);
    CHECK(sent[17].msg.data[0] == 0x21U);
    CHECK(isotp_tx_result(&tx) == ISOTP_TX_RESULT_SUCCESS);
}

static void test_wait_overflow_timeout_and_errors(void) {
    uint8_t payload[8] = {0};

    init_tx();
    CHECK(isotp_tx_start(&tx, payload, sizeof(payload)));
    isotp_tx_tick(&tx);
    twai_message_t wait = flow_control(1U, 0U, 0U);
    for (int i = 0; i < 3; i++) {
        fake_now += 100;
        CHECK(isotp_tx_rx(&tx, &wait, CAN_BUS_0));
        CHECK(isotp_tx_busy(&tx));
    }
    CHECK(isotp_tx_rx(&tx, &wait, CAN_BUS_0));
    CHECK(!isotp_tx_busy(&tx));
    CHECK(isotp_tx_result(&tx) == ISOTP_TX_RESULT_TIMEOUT);

    init_tx();
    CHECK(isotp_tx_start(&tx, payload, sizeof(payload)));
    isotp_tx_tick(&tx);
    twai_message_t overflow = flow_control(2U, 0U, 0U);
    CHECK(isotp_tx_rx(&tx, &overflow, CAN_BUS_0));
    CHECK(isotp_tx_result(&tx) == ISOTP_TX_RESULT_OVERFLOW);

    init_tx();
    CHECK(isotp_tx_start(&tx, payload, sizeof(payload)));
    isotp_tx_tick(&tx);
    fake_now += 1000000;
    isotp_tx_tick(&tx);
    CHECK(isotp_tx_result(&tx) == ISOTP_TX_RESULT_TIMEOUT);

    init_tx();
    fail_send_number = 0;
    CHECK(isotp_tx_start(&tx, payload, sizeof(payload)));
    isotp_tx_tick(&tx);
    CHECK(isotp_tx_result(&tx) == ISOTP_TX_RESULT_SEND_ERROR);

    init_tx();
    CHECK(isotp_tx_start(&tx, payload, sizeof(payload)));
    isotp_tx_tick(&tx);
    twai_message_t malformed = flow_control(0U, 0U, 0U);
    malformed.data_length_code = 2U;
    CHECK(isotp_tx_rx(&tx, &malformed, CAN_BUS_0));
    CHECK(isotp_tx_result(&tx) == ISOTP_TX_RESULT_PROTOCOL_ERROR);
}

int main(void) {
    test_single_frame();
    test_flow_control_and_timing();
    test_sequence_rollover();
    test_wait_overflow_timeout_and_errors();
    return test_report("ISO-TP transmitter");
}
