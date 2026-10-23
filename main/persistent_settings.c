#include <stdbool.h>
#include <stdatomic.h>
#include <stdint.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"
#include "persistent_settings.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG __func__

#define PERSISTENT_SETTINGS_RETRY_US 5000000U  // retry failed flash writes every 5 seconds
#define PERSISTENT_SETTINGS_MAX_RETRIES 3U
#define PERSISTENT_SETTINGS_TASK_STACK_SIZE (3 * 1024)
// Prio 5 stays below precondition/CAN RX (8/7), but still gets time alongside
// normal network and CAN TX work so a busy datapath cannot starve flash saves.
#define PERSISTENT_SETTINGS_TASK_PRIORITY 5

// Add future NVS-mirrored u8 settings here, then expose a typed accessor below.
typedef enum {
    MIRRORED_U8_PRECONDITIONING_ENABLED,
    MIRRORED_U8_COUNT,
} mirrored_u8_id_t;

typedef struct {
    const char *nvs_namespace;
    const char *nvs_key;
    uint8_t default_value;

    atomic_uint desired_value;
    atomic_uint generation;
    uint8_t flash_mirror;
    uint8_t save_failures;
    unsigned int observed_generation;
    int64_t retry_at;
} mirrored_u8_setting_t;

static mirrored_u8_setting_t mirrored_u8_settings[MIRRORED_U8_COUNT] = {
    [MIRRORED_U8_PRECONDITIONING_ENABLED] = {
        .nvs_namespace = "precondition",
        .nvs_key = "persist_en",
        .default_value = 0U,
    },
};

static TaskHandle_t persistent_settings_task_handle = NULL;
static bool persistent_settings_initialized = false;

static uint8_t load_setting(const mirrored_u8_setting_t *setting) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(setting->nvs_namespace, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        if (err != ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to open %s/%s: %s", setting->nvs_namespace,
                     setting->nvs_key, esp_err_to_name(err));
        }
        return setting->default_value;
    }

    uint8_t stored_value = setting->default_value;
    err = nvs_get_u8(handle, setting->nvs_key, &stored_value);
    nvs_close(handle);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "Failed to read %s/%s: %s", setting->nvs_namespace,
                 setting->nvs_key, esp_err_to_name(err));
    }
    return err == ESP_OK ? stored_value : setting->default_value;
}

static bool save_setting(const mirrored_u8_setting_t *setting, uint8_t value) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(setting->nvs_namespace, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open %s/%s for writing: %s",
                 setting->nvs_namespace, setting->nvs_key, esp_err_to_name(err));
        return false;
    }

    err = nvs_set_u8(handle, setting->nvs_key, value);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save %s/%s: %s", setting->nvs_namespace,
                 setting->nvs_key, esp_err_to_name(err));
    }
    return err == ESP_OK;
}

static uint8_t get_u8(mirrored_u8_id_t id) {
    return (uint8_t)atomic_load_explicit(
            &mirrored_u8_settings[id].desired_value, memory_order_relaxed);
}

static void set_u8(mirrored_u8_id_t id, uint8_t value) {
    mirrored_u8_setting_t *setting = &mirrored_u8_settings[id];
    unsigned int previous = atomic_exchange_explicit(
            &setting->desired_value, value, memory_order_relaxed);
    if (previous == value) {
        return;
    }

    atomic_fetch_add_explicit(&setting->generation, 1U, memory_order_release);
    if (persistent_settings_task_handle != NULL) {
        xTaskNotifyGive(persistent_settings_task_handle);
    }
}

// Mirror one setting to flash. Failed writes retry until the budget for the
// current generation is spent. Zero means no retry is pending.
static int64_t flush_setting(mirrored_u8_setting_t *setting) {
    unsigned int generation = atomic_load_explicit(
            &setting->generation, memory_order_acquire);
    if (generation != setting->observed_generation) {
        setting->observed_generation = generation;
        setting->save_failures = 0U;
        setting->retry_at = 0;
    }

    uint8_t desired = (uint8_t)atomic_load_explicit(
            &setting->desired_value, memory_order_relaxed);
    int64_t now = esp_timer_get_time();
    if (desired == setting->flash_mirror
            || setting->save_failures > PERSISTENT_SETTINGS_MAX_RETRIES) {
        return 0;
    }
    if (now < setting->retry_at) {
        return setting->retry_at;
    }
    if (save_setting(setting, desired)) {
        setting->flash_mirror = desired;
        setting->save_failures = 0U;
        return 0;
    }

    setting->save_failures++;
    if (setting->save_failures > PERSISTENT_SETTINGS_MAX_RETRIES) {
        ESP_LOGE(TAG, "Giving up saving %s/%s after %u attempts",
                 setting->nvs_namespace, setting->nvs_key,
                 (unsigned)setting->save_failures);
        return 0;
    }
    setting->retry_at = now + PERSISTENT_SETTINGS_RETRY_US;
    return setting->retry_at;
}

static int64_t flush_pending_settings(void) {
    int64_t next_retry_at = 0;
    for (mirrored_u8_id_t id = 0; id < MIRRORED_U8_COUNT; id++) {
        int64_t retry_at = flush_setting(&mirrored_u8_settings[id]);
        if (retry_at != 0 && (next_retry_at == 0 || retry_at < next_retry_at)) {
            next_retry_at = retry_at;
        }
    }
    return next_retry_at;
}

static TickType_t persistent_wait_ticks(int64_t retry_at) {
    if (retry_at == 0) {
        return portMAX_DELAY;
    }

    int64_t remaining_us = retry_at - esp_timer_get_time();
    if (remaining_us <= 0) {
        return 0;
    }
    uint64_t remaining_ms = ((uint64_t)remaining_us + 999U) / 1000U;  // round up
    TickType_t wait = pdMS_TO_TICKS(remaining_ms);
    return wait == 0 ? 1 : wait;
}

static void persistent_settings_task(void *arg) {
    (void)arg;
    int64_t retry_at = 0;
    while (1) {
        ulTaskNotifyTake(pdTRUE, persistent_wait_ticks(retry_at));
        retry_at = flush_pending_settings();
    }
}

void persistent_settings_init(void) {
    if (persistent_settings_initialized) {
        return;
    }

    for (mirrored_u8_id_t id = 0; id < MIRRORED_U8_COUNT; id++) {
        mirrored_u8_setting_t *setting = &mirrored_u8_settings[id];
        uint8_t stored_value = load_setting(setting);
        atomic_init(&setting->desired_value, stored_value);
        atomic_init(&setting->generation, 0U);
        setting->flash_mirror = stored_value;
        setting->save_failures = 0U;
        setting->observed_generation = 0U;
        setting->retry_at = 0;
    }

    BaseType_t created = xTaskCreate(persistent_settings_task, "settings_nvs",
            PERSISTENT_SETTINGS_TASK_STACK_SIZE, NULL,
            PERSISTENT_SETTINGS_TASK_PRIORITY, &persistent_settings_task_handle);
    configASSERT(created == pdPASS);
    (void)created;
    persistent_settings_initialized = true;
}

bool persistent_settings_get_precon_enabled(void) {
    configASSERT(persistent_settings_initialized);
    return get_u8(MIRRORED_U8_PRECONDITIONING_ENABLED) != 0U;
}

void persistent_settings_set_precon_enabled(bool enabled) {
    configASSERT(persistent_settings_initialized);
    set_u8(MIRRORED_U8_PRECONDITIONING_ENABLED, enabled ? 1U : 0U);
}
