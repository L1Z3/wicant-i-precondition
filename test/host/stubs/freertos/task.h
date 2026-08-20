// Host-test stub for task APIs used by firmware workers. Tests drive the
// workers manually, so xTaskCreate records a non-NULL handle without running
// the infinite worker function.
#pragma once
#include <stdint.h>
#include "freertos/FreeRTOS.h"

typedef void *TaskHandle_t;
typedef void (*TaskFunction_t)(void *);

static inline BaseType_t xTaskCreate(TaskFunction_t task, const char *name,
                                    uint32_t stack_size, void *arg,
                                    UBaseType_t priority, TaskHandle_t *handle) {
    (void)task;
    (void)name;
    (void)stack_size;
    (void)arg;
    (void)priority;
    if (handle != NULL) {
        *handle = (TaskHandle_t)(uintptr_t)1U;
    }
    return pdPASS;
}

static inline BaseType_t xTaskNotifyGive(TaskHandle_t handle) {
    (void)handle;
    return pdPASS;
}

static inline uint32_t ulTaskNotifyTake(BaseType_t clear_on_exit,
                                       TickType_t ticks_to_wait) {
    (void)clear_on_exit;
    (void)ticks_to_wait;
    return 0U;
}
