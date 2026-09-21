#define _POSIX_C_SOURCE 200809L
#include "fake_platform.h"
#include "fake_mcp2518fd.h"
#include <pthread.h>
#include <stdatomic.h>
#include <time.h>
#include <errno.h>

struct fake_semaphore { pthread_mutex_t lock; pthread_cond_t changed; unsigned count, capacity; bool mutex; };
struct fake_events { pthread_mutex_t lock; pthread_cond_t changed; EventBits_t bits; };
struct fake_task { pthread_t thread; SemaphoreHandle_t notifications; void (*entry)(void *); void *arg; };
struct fake_spi { unsigned hz; bool acquired; };
static _Thread_local TaskHandle_t current_task;
static _Thread_local bool isr_context;
static atomic_int semaphore_count, event_count, task_count, spi_count;
static pthread_mutex_t hardware_lock = PTHREAD_MUTEX_INITIALIZER;
static fake_mcp2518fd_t chip;
static MCP251XFD_SPITransfer_Func hardware_transfer;
static void (*interrupt_handler)(void *);
static void *interrupt_arg;
static bool interrupt_enabled, fail_install;
static bool cs_held;
static unsigned spi_acquire_attempts, spi_fail_acquire;
static atomic_uint spi_reservations;

static struct timespec deadline(unsigned ms)
{
    struct timespec until;
    clock_gettime(CLOCK_REALTIME, &until);
    until.tv_sec += ms / 1000;
    until.tv_nsec += (ms % 1000) * 1000000L;
    if (until.tv_nsec >= 1000000000L) { until.tv_sec++; until.tv_nsec -= 1000000000L; }
    return until;
}

void platform_pause_ms(unsigned ms)
{
    struct timespec delay = {.tv_sec = ms / 1000, .tv_nsec = (ms % 1000) * 1000000L};
    nanosleep(&delay, NULL);
}

SemaphoreHandle_t xSemaphoreCreateCounting(unsigned capacity, unsigned initial)
{
    SemaphoreHandle_t sem = calloc(1, sizeof(*sem));
    assert(sem);
    pthread_mutex_init(&sem->lock, NULL);
    pthread_cond_init(&sem->changed, NULL);
    sem->capacity = capacity;
    sem->count = initial;
    atomic_fetch_add(&semaphore_count, 1);
    return sem;
}

SemaphoreHandle_t xSemaphoreCreateMutex(void)
{
    SemaphoreHandle_t sem = xSemaphoreCreateCounting(1, 1);
    sem->mutex = true;
    return sem;
}

SemaphoreHandle_t xSemaphoreCreateBinary(void) { return xSemaphoreCreateCounting(1, 0); }

int xSemaphoreTake(SemaphoreHandle_t sem, TickType_t timeout)
{
    assert(!isr_context);
    if (sem->mutex) { pthread_mutex_lock(&sem->lock); return pdTRUE; }
    pthread_mutex_lock(&sem->lock);
    struct timespec until = deadline(timeout);
    while (!sem->count) {
        if (!timeout || (timeout != portMAX_DELAY && pthread_cond_timedwait(&sem->changed, &sem->lock, &until) == ETIMEDOUT)) {
            pthread_mutex_unlock(&sem->lock);
            return pdFALSE;
        }
        if (timeout == portMAX_DELAY) pthread_cond_wait(&sem->changed, &sem->lock);
    }
    sem->count--;
    pthread_mutex_unlock(&sem->lock);
    return pdTRUE;
}

int xSemaphoreGive(SemaphoreHandle_t sem)
{
    if (sem->mutex) { pthread_mutex_unlock(&sem->lock); return pdTRUE; }
    pthread_mutex_lock(&sem->lock);
    bool room = sem->count < sem->capacity;
    if (room) sem->count++;
    pthread_cond_broadcast(&sem->changed);
    pthread_mutex_unlock(&sem->lock);
    return room;
}

void vSemaphoreDelete(SemaphoreHandle_t sem)
{
    assert(pthread_mutex_destroy(&sem->lock) == 0);
    assert(pthread_cond_destroy(&sem->changed) == 0);
    free(sem);
    atomic_fetch_sub(&semaphore_count, 1);
}

EventGroupHandle_t xEventGroupCreate(void)
{
    EventGroupHandle_t group = calloc(1, sizeof(*group));
    assert(group);
    pthread_mutex_init(&group->lock, NULL);
    pthread_cond_init(&group->changed, NULL);
    atomic_fetch_add(&event_count, 1);
    return group;
}

EventBits_t xEventGroupSetBits(EventGroupHandle_t group, EventBits_t bits)
{
    assert(!isr_context);
    pthread_mutex_lock(&group->lock);
    EventBits_t result = group->bits |= bits;
    pthread_cond_broadcast(&group->changed);
    pthread_mutex_unlock(&group->lock);
    return result;
}

EventBits_t xEventGroupClearBits(EventGroupHandle_t group, EventBits_t bits)
{
    pthread_mutex_lock(&group->lock);
    EventBits_t result = group->bits;
    group->bits &= ~bits;
    pthread_mutex_unlock(&group->lock);
    return result;
}

EventBits_t xEventGroupWaitBits(EventGroupHandle_t group, EventBits_t bits, int clear, int all, TickType_t timeout)
{
    assert(!clear && all);
    pthread_mutex_lock(&group->lock);
    struct timespec until = deadline(timeout);
    while ((group->bits & bits) != bits) {
        if (!timeout || (timeout != portMAX_DELAY && pthread_cond_timedwait(&group->changed, &group->lock, &until) == ETIMEDOUT)) break;
        if (timeout == portMAX_DELAY) pthread_cond_wait(&group->changed, &group->lock);
    }
    EventBits_t result = group->bits;
    pthread_mutex_unlock(&group->lock);
    return result;
}

void vEventGroupDelete(EventGroupHandle_t group)
{
    assert(pthread_mutex_destroy(&group->lock) == 0);
    assert(pthread_cond_destroy(&group->changed) == 0);
    free(group);
    atomic_fetch_sub(&event_count, 1);
}

static void *run_task(void *arg)
{
    current_task = arg;
    current_task->entry(current_task->arg);
    assert(false); // Worker must exit through vTaskDelete(NULL).
    return NULL;
}

BaseType_t xTaskCreatePinnedToCore(void (*entry)(void *), const char *name, unsigned stack_size,
                                  void *arg, unsigned priority, TaskHandle_t *handle, int core)
{
    (void)name; (void)stack_size; (void)priority; (void)core;
    TaskHandle_t task = calloc(1, sizeof(*task));
    assert(task);
    task->entry = entry;
    task->arg = arg;
    task->notifications = xSemaphoreCreateCounting(UINT32_MAX, 0);
    *handle = task;
    atomic_fetch_add(&task_count, 1);
    assert(pthread_create(&task->thread, NULL, run_task, task) == 0);
    pthread_detach(task->thread);
    return pdPASS;
}

void vTaskDelete(TaskHandle_t handle)
{
    assert(!handle && current_task); // Deleting a running worker externally is forbidden.
    vSemaphoreDelete(current_task->notifications);
    free(current_task);
    current_task = NULL;
    atomic_fetch_sub(&task_count, 1);
    pthread_exit(NULL);
}

void xTaskNotifyGive(TaskHandle_t handle) { assert(!isr_context); xSemaphoreGive(handle->notifications); }
void vTaskNotifyGiveFromISR(TaskHandle_t handle, BaseType_t *woken) { assert(isr_context); xSemaphoreGive(handle->notifications); *woken = pdFALSE; }
unsigned ulTaskNotifyTake(int clear, TickType_t timeout)
{
    assert(current_task && clear);
    SemaphoreHandle_t sem = current_task->notifications;
    if (!xSemaphoreTake(sem, timeout)) return 0;
    pthread_mutex_lock(&sem->lock);
    unsigned count = sem->count + 1;
    sem->count = 0;
    pthread_mutex_unlock(&sem->lock);
    return count;
}
TaskHandle_t xTaskGetCurrentTaskHandle(void) { return current_task; }
bool xPortInIsrContext(void) { return isr_context; }
void platform_enter_isr(bool is_isr) { isr_context = is_isr; }
int64_t esp_timer_get_time(void)
{
    struct timespec time;
    clock_gettime(CLOCK_MONOTONIC, &time);
    return (int64_t)time.tv_sec * 1000000 + time.tv_nsec / 1000;
}

esp_err_t spi_bus_add_device(spi_host_device_t bus, const spi_device_interface_config_t *config, spi_device_handle_t *device)
{
    (void)bus;
    *device = calloc(1, sizeof(**device));
    assert(*device);
    (*device)->hz = config->clock_speed_hz;
    atomic_fetch_add(&spi_count, 1);
    return ESP_OK;
}
esp_err_t spi_bus_remove_device(spi_device_handle_t device)
{
    assert(!device->acquired);
    // Device removal resets the CS pin. It must remain held if the chip is
    // asleep, otherwise a low glitch could undo the low-power request.
    assert(!chip.low_power || cs_held);
    free(device);
    atomic_fetch_sub(&spi_count, 1);
    return ESP_OK;
}
esp_err_t spi_device_acquire_bus(spi_device_handle_t device, TickType_t wait)
{
    assert(wait == portMAX_DELAY && !device->acquired);
    if (++spi_acquire_attempts == spi_fail_acquire) return ESP_FAIL;
    assert(atomic_fetch_add(&spi_reservations, 1) == 0);
    device->acquired = true;
    return ESP_OK;
}
void spi_device_release_bus(spi_device_handle_t device)
{
    assert(device->acquired && atomic_fetch_sub(&spi_reservations, 1) == 1);
    device->acquired = false;
}
esp_err_t spi_device_polling_transmit(spi_device_handle_t device, spi_transaction_t *transaction)
{
    assert(device && !isr_context);
    pthread_mutex_lock(&hardware_lock);
    assert(!cs_held);
    chip.spi_hz = device->hz;
    eERRORRESULT error = hardware_transfer(&chip, 0, (uint8_t *)transaction->tx_buffer,
                                           transaction->rx_buffer, transaction->length / 8);
    pthread_mutex_unlock(&hardware_lock);
    return error == ERR_NONE ? ESP_OK : ESP_FAIL;
}
esp_err_t gpio_config(const gpio_config_t *config) { (void)config; return ESP_OK; }
esp_err_t gpio_hold_en(gpio_num_t gpio) { assert(gpio == 18); cs_held = true; return ESP_OK; }
esp_err_t gpio_hold_dis(gpio_num_t gpio) { assert(gpio == 18); cs_held = false; return ESP_OK; }
esp_err_t gpio_intr_enable(gpio_num_t gpio)
{
    (void)gpio;
    pthread_mutex_lock(&hardware_lock);
    interrupt_enabled = true;
    pthread_mutex_unlock(&hardware_lock);
    return ESP_OK;
}
esp_err_t gpio_intr_disable(gpio_num_t gpio)
{
    (void)gpio;
    pthread_mutex_lock(&hardware_lock);
    interrupt_enabled = false;
    pthread_mutex_unlock(&hardware_lock);
    return ESP_OK;
}
esp_err_t gpio_isr_handler_add(gpio_num_t gpio, void (*handler)(void *), void *arg)
{
    (void)gpio;
    pthread_mutex_lock(&hardware_lock);
    if (!fail_install) { interrupt_handler = handler; interrupt_arg = arg; }
    pthread_mutex_unlock(&hardware_lock);
    return fail_install ? ESP_FAIL : ESP_OK;
}
esp_err_t gpio_isr_handler_remove(gpio_num_t gpio)
{
    (void)gpio;
    pthread_mutex_lock(&hardware_lock);
    interrupt_handler = NULL;
    interrupt_arg = NULL;
    pthread_mutex_unlock(&hardware_lock);
    return ESP_OK;
}
esp_err_t gpio_set_intr_type(gpio_num_t gpio, int type) { (void)gpio; (void)type; return ESP_OK; }
int gpio_get_level(gpio_num_t gpio)
{
    (void)gpio;
    pthread_mutex_lock(&hardware_lock);
    bool active = chip.tef_count || chip.rx_count || (chip.memory[0x060] & 16);
    pthread_mutex_unlock(&hardware_lock);
    return !active;
}

void platform_check_clean(void)
{
    for (unsigned i = 0; i < 100 && atomic_load(&task_count); i++) platform_pause_ms(1);
    assert(!atomic_load(&task_count) && !atomic_load(&semaphore_count) && !atomic_load(&event_count) && !atomic_load(&spi_count));
    assert(!interrupt_handler);
    assert(!atomic_load(&spi_reservations));
}
void platform_reset(void)
{
    platform_check_clean();
    mcp251xfd_core_t core;
    fake_init(&chip, &core);
    hardware_transfer = core.device.fnSPI_Transfer;
    fail_install = interrupt_enabled = false;
    cs_held = false;
    spi_acquire_attempts = spi_fail_acquire = 0;
}
bool platform_is_low_power(void)
{
    pthread_mutex_lock(&hardware_lock);
    bool asleep = chip.low_power;
    pthread_mutex_unlock(&hardware_lock);
    return asleep;
}
void platform_finish_tx(unsigned count)
{
    pthread_mutex_lock(&hardware_lock);
    for (unsigned i = 0; i < count; i++) fake_transmit(&chip, true);
    if (interrupt_enabled && interrupt_handler) {
        isr_context = true;
        interrupt_handler(interrupt_arg);
        isr_context = false;
    }
    pthread_mutex_unlock(&hardware_lock);
}
unsigned platform_pending_tx(void)
{
    pthread_mutex_lock(&hardware_lock);
    unsigned count = chip.tx_count;
    pthread_mutex_unlock(&hardware_lock);
    return count;
}
void platform_receive(uint32_t id, bool rtr)
{
    pthread_mutex_lock(&hardware_lock);
    mcp251xfd_frame_t frame = {.id = id, .extended = true, .rtr = rtr, .dlc = 8,
                               .data = {0, 1, 2, 3, 4, 5, 6, 7}};
    fake_receive(&chip, &frame, false);
    if (interrupt_enabled && interrupt_handler) {
        isr_context = true;
        interrupt_handler(interrupt_arg);
        isr_context = false;
    }
    pthread_mutex_unlock(&hardware_lock);
}
void platform_disconnect(void) { pthread_mutex_lock(&hardware_lock); chip.disconnected = true; pthread_mutex_unlock(&hardware_lock); }
void platform_fail_isr_install(void) { fail_install = true; }
void platform_fail_spi_acquire(unsigned attempt) { spi_fail_acquire = attempt; }
bool platform_spi_acquired(void) { return atomic_load(&spi_reservations) != 0; }
