// host-test stub for esp_err.h: real error codes are only compared, never
// inspected, so the numeric values just have to stay distinct. "MUST stay in
// sync with real headers".
#pragma once
#include "can.h"  // esp_err_t

#ifndef ESP_OK
#define ESP_OK                 0
#define ESP_ERR_INVALID_STATE  0x103
#define ESP_ERR_NOT_SUPPORTED  0x106
#endif
