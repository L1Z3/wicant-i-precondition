// host-test stub for ESP-IDF driver/twai.h: just the fields the code under
// test touches
#pragma once
#include <stdint.h>

typedef struct {
    uint32_t identifier;
    uint8_t data_length_code;
    uint8_t data[8];
    uint8_t extd;   // extended (29-bit) id
    uint8_t rtr;    // remote frame
    uint8_t self;   // loopback/self-reception request
} twai_message_t;
