#ifndef __PRECONDITION_H__
#define __PRECONDITION_H__

#include "driver/twai.h"

typedef enum {
    FWD_BLOCK,
    FWD_MODIFIED,
    FWD_PASSTHROUGH,
} fwd_result_t;

void precondition_can_rx_hook(twai_message_t *to_push);
fwd_result_t precondition_fwd_hook(twai_message_t *to_send);
void precondition_tick(void);
void button_press_action(twai_message_t *to_push, bool activation_button_state);

#endif
