// host-test stub for the task notification API used by precondition.c
#pragma once

#include "freertos/FreeRTOS.h"

typedef void *TaskHandle_t;
typedef void (*TaskFunction_t)(void *);

static inline BaseType_t xTaskCreate(TaskFunction_t function, const char *name,
        uint32_t stack_depth, void *arg, UBaseType_t priority,
        TaskHandle_t *handle) {
    (void)function;
    (void)name;
    (void)stack_depth;
    (void)arg;
    (void)priority;
    if (handle != NULL) {
        *handle = (TaskHandle_t)1;
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
    return 0;
}
