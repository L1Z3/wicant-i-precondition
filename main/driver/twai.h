/*
 * precondition.c is kept identical to the wicant-i-precondition firmware,
 * where it includes the legacy driver/twai.h for twai_message_t. On v6 the
 * bus belongs to can_manager on the esp_twai node API, and the legacy driver
 * must not be linked alongside it. precondition.c only needs the frame type,
 * so this header, found first from main/, supplies the types alone.
 */
#pragma once

/* what the real header brings in ahead of its types; code written against it
   (hsm.h) relies on FreeRTOS.h arriving this way */
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "driver/twai_types_legacy.h"
