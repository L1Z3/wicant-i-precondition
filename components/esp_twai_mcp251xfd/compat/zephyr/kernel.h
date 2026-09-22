#pragma once

// Zephyr kernel primitives used by the vendored sources, on FreeRTOS.
#include <assert.h>
#include <errno.h>
#include <stdlib.h>
#include <zephyr/sys/util.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"

#define __ASSERT_NO_MSG(test) assert(test)
#define k_oops() abort()

// Timeouts are in FreeRTOS ticks, rounded up from microseconds.
typedef struct {
	TickType_t ticks;
} k_timeout_t;

#define K_NO_WAIT  ((k_timeout_t){0})
#define K_FOREVER  ((k_timeout_t){portMAX_DELAY})
#define K_USEC(us) ((k_timeout_t){(TickType_t)DIV_ROUND_UP((uint64_t)(us) * configTICK_RATE_HZ, 1000000)})
#define K_MSEC(ms) K_USEC((uint64_t)(ms) * 1000)

// Zephyr waits at least the requested time. The current tick is already
// partly over, so a finite wait needs one extra tick.
static inline TickType_t z_ticks(k_timeout_t timeout)
{
	if (timeout.ticks == 0 || timeout.ticks == portMAX_DELAY) {
		return timeout.ticks;
	}
	return timeout.ticks + 1;
}

static inline int32_t k_sleep(k_timeout_t timeout)
{
	vTaskDelay(z_ticks(timeout));
	return 0;
}

static inline void k_busy_wait(uint32_t usec)
{
	esp_rom_delay_us(usec);
}

// Zephyr mutexes are recursive; the driver relocks from nested helpers.
struct k_mutex {
	SemaphoreHandle_t handle;
	StaticSemaphore_t storage;
};

static inline int k_mutex_init(struct k_mutex *mutex)
{
	mutex->handle = xSemaphoreCreateRecursiveMutexStatic(&mutex->storage);
	return 0;
}

static inline int k_mutex_lock(struct k_mutex *mutex, k_timeout_t timeout)
{
	return xSemaphoreTakeRecursive(mutex->handle, z_ticks(timeout)) == pdTRUE ? 0 : -EAGAIN;
}

static inline int k_mutex_unlock(struct k_mutex *mutex)
{
	xSemaphoreGiveRecursive(mutex->handle);
	return 0;
}

struct k_sem {
	SemaphoreHandle_t handle;
	StaticSemaphore_t storage;
};

static inline int k_sem_init(struct k_sem *sem, unsigned int initial_count, unsigned int limit)
{
	sem->handle = xSemaphoreCreateCountingStatic(limit, initial_count, &sem->storage);
	return 0;
}

static inline int k_sem_take(struct k_sem *sem, k_timeout_t timeout)
{
	return xSemaphoreTake(sem->handle, z_ticks(timeout)) == pdTRUE ? 0 : -EAGAIN;
}

// Also given from the INT GPIO interrupt handler.
static inline void k_sem_give(struct k_sem *sem)
{
	if (xPortInIsrContext()) {
		BaseType_t woken = pdFALSE;
		xSemaphoreGiveFromISR(sem->handle, &woken);
		if (woken) {
			portYIELD_FROM_ISR();
		}
	} else {
		xSemaphoreGive(sem->handle);
	}
}

// Used only by can_common.c's message-queue filter helper.
struct k_msgq {
	QueueHandle_t handle;
};

static inline int k_msgq_put(struct k_msgq *msgq, const void *data, k_timeout_t timeout)
{
	return xQueueSend(msgq->handle, data, z_ticks(timeout)) == pdTRUE ? 0 : -ENOMSG;
}

// FreeRTOS allocates thread stacks; Zephyr's stack argument is ignored.
typedef uint8_t k_thread_stack_t;
typedef void (*k_thread_entry_t)(void *p1, void *p2, void *p3);

struct k_thread {
	TaskHandle_t handle;
	k_thread_entry_t entry;
	void *p1, *p2, *p3;
};
typedef struct k_thread *k_tid_t;

// Thread priorities are configured directly as FreeRTOS priorities.
#define K_PRIO_COOP(x) (x)

static inline void z_thread_entry(void *arg)
{
	struct k_thread *thread = arg;

	thread->entry(thread->p1, thread->p2, thread->p3);
	vTaskDelete(NULL);
}

static inline k_tid_t k_thread_create(struct k_thread *thread, k_thread_stack_t *stack,
				      size_t stack_size, k_thread_entry_t entry, void *p1,
				      void *p2, void *p3, int prio, uint32_t options,
				      k_timeout_t delay)
{
	(void)stack;
	(void)options;
	(void)delay;
	*thread = (struct k_thread){.entry = entry, .p1 = p1, .p2 = p2, .p3 = p3};
	// Zephyr's k_thread_create cannot fail.
	if (xTaskCreatePinnedToCore(z_thread_entry, COMPAT_K_THREAD_NAME, stack_size, thread, prio,
				    &thread->handle, COMPAT_K_THREAD_CORE) != pdPASS) {
		k_oops();
	}
	return thread;
}

static inline int k_thread_name_set(k_tid_t thread, const char *name)
{
	(void)thread;
	(void)name;
	return 0;
}
