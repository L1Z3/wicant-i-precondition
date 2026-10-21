// host-test stub for the recursive mutex used by hsm.c
#pragma once

#include <pthread.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"

typedef void *SemaphoreHandle_t;

typedef struct {
    pthread_mutex_t mutex;
} fake_recursive_mutex_t;

static inline SemaphoreHandle_t fake_mutex_create(int type) {
    fake_recursive_mutex_t *m = malloc(sizeof(*m));
    if (m == NULL) {
        return NULL;
    }

    pthread_mutexattr_t attr;
    if (pthread_mutexattr_init(&attr) != 0) {
        free(m);
        return NULL;
    }
    if (pthread_mutexattr_settype(&attr, type) != 0
            || pthread_mutex_init(&m->mutex, &attr) != 0) {
        pthread_mutexattr_destroy(&attr);
        free(m);
        return NULL;
    }
    pthread_mutexattr_destroy(&attr);
    return m;
}

static inline SemaphoreHandle_t xSemaphoreCreateRecursiveMutex(void) {
    return fake_mutex_create(PTHREAD_MUTEX_RECURSIVE);
}

static inline SemaphoreHandle_t xSemaphoreCreateMutex(void) {
    return fake_mutex_create(PTHREAD_MUTEX_NORMAL);
}

static inline BaseType_t xSemaphoreTakeRecursive(SemaphoreHandle_t handle, TickType_t wait) {
    (void)wait;
    fake_recursive_mutex_t *m = handle;
    return pthread_mutex_lock(&m->mutex) == 0 ? pdTRUE : pdFALSE;
}

static inline BaseType_t xSemaphoreGiveRecursive(SemaphoreHandle_t handle) {
    fake_recursive_mutex_t *m = handle;
    return pthread_mutex_unlock(&m->mutex) == 0 ? pdTRUE : pdFALSE;
}

static inline BaseType_t xSemaphoreTake(SemaphoreHandle_t handle, TickType_t wait) {
    return xSemaphoreTakeRecursive(handle, wait);
}

static inline BaseType_t xSemaphoreGive(SemaphoreHandle_t handle) {
    return xSemaphoreGiveRecursive(handle);
}
