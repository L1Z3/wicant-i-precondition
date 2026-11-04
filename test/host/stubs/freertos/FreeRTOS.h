// host-test stub for freertos/FreeRTOS.h: basic types only
#pragma once
#include <stdint.h>
#include <assert.h>

typedef uint32_t TickType_t;
typedef int BaseType_t;
typedef unsigned int UBaseType_t;
#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define portMAX_DELAY UINT32_MAX
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
// 1 tick == 1 ms, so slcan_get_time()'s pdTICKS_TO_MS() is the identity
#define pdTICKS_TO_MS(t) ((uint32_t)(t))
#define configASSERT(x) assert(x)
