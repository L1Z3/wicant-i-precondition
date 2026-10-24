/**
 * @file main_precondition.c
 * @brief Manual battery preconditioning for E-GMP cars (Ioniq 5/6, EV6),
 *        from the wicant-i-precondition firmware.
 *
 * precondition.c and the modules it uses (hsm, isotp_tx, track_popup, beep,
 * persistent_settings, utf8_utf16_converter) are that firmware's, unchanged,
 * so a later version can be dropped in as is. This file supplies what they
 * expect from around them there:
 *
 *  - can_send() (can.h), on can_manager_send();
 *  - the config_server getters (config_server.h), from the "precondition"
 *    settings group below;
 *  - the two tasks. There, the CAN task fed every frame to
 *    precondition_can_rx_hook() and sent the modified copy
 *    precondition_fwd_hook() asked for, and a task of its own ran
 *    precondition_tick() every 40 ms. Here the frames come from a
 *    can_manager subscription instead; the state machine locks around both.
 *
 * The Pro has a single CAN interface, so this is that firmware's parallel
 * mode: the device sits on M-CAN behind the head unit, a modified copy goes
 * out alongside the original frame, and nothing can be blocked.
 *
 * Needs the native bus: can_manager.enabled, baud 500, silent off. A fresh
 * device gets that from main_precondition_can_defaults().
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "autopid.h"
#include "can_core.h"
#include "can_manager.h"
#include "dev_status_manager.h"
#include "settings_manager.h"

#include "beep.h"
#include "can.h"
#include "config_server.h"
#include "precondition.h"

#include "main_precondition.h"

static const char *TAG = "precondition";

/* precondition_tick() is written to be called every 40 ms */
#define PRECON_TICK_US          40000
/* 512 frames: about 220 ms of a busy M-CAN (~2300 frames/s) before
   can_manager starts dropping the oldest for us */
#define PRECON_Q_LEN            512
#define PRECON_RX_TASK_STACK    4096
#define PRECON_TICK_TASK_STACK  4096
/* where the old firmware ran them: frames at its CAN task's 7, the tick
   above that at 8; both below the bus's own receive task (10) */
#define PRECON_RX_TASK_PRIO     7
#define PRECON_TICK_TASK_PRIO   8
#define PRECON_RX_IDLE_MS       100
/* how often to check the subscription is still ours: can_manager_stop()
   (sleep) clears the subscriber table under us */
#define PRECON_SUB_CHECK_US     2000000
#define PRECON_PUB_US           1000000
#define PRECON_SUB_NAME         "precondition"

/* ---- settings ------------------------------------------------------------ */

/* Same names and order as the old firmware's web UI. The CSV is what the
   settings schema enforces, the table what the names mean. */
#define BUTTONS_CSV "disabled,sw_star,avn_star,avn_tuner_in,avn_vol_in," \
                    "ev6_avn_setup,sw_mode,sw_speak,sw_call,sw_vol_in," \
                    "sw_vol_up,sw_vol_down,sw_skip_up,sw_skip_down,sw_ok," \
                    "avn_map,avn_nav,avn_media,avn_tuner_up,avn_tuner_down"

static const struct
{
    const char *name;
    int8_t      id;
} BUTTONS[] =
{
    { "disabled",       BUTTON_DISABLED },
    { "sw_star",        SW_STAR },
    { "avn_star",       AVN_STAR },
    { "avn_tuner_in",   AVN_TUNER_IN },
    { "avn_vol_in",     AVN_VOL_IN },
    { "ev6_avn_setup",  EV6_AVN_SETUP },
    { "sw_mode",        SW_MODE },
    { "sw_speak",       SW_SPEAK },
    { "sw_call",        SW_CALL },
    { "sw_vol_in",      SW_VOL_IN },
    { "sw_vol_up",      SW_VOL_UP },
    { "sw_vol_down",    SW_VOL_DOWN },
    { "sw_skip_up",     SW_SKIP_UP },
    { "sw_skip_down",   SW_SKIP_DOWN },
    { "sw_ok",          SW_OK },
    { "avn_map",        AVN_MAP },
    { "avn_nav",        AVN_NAV },
    { "avn_media",      AVN_MEDIA },
    { "avn_tuner_up",   AVN_TUNER_UP },
    { "avn_tuner_down", AVN_TUNER_DOWN },
};

static const settings_field_t FIELDS[] =
{
    SETTINGS_STR_ENUM("mode", "once,continuous,persistent", "once"),
    SETTINGS_STR_ENUM("button", BUTTONS_CSV, "sw_star"),
    SETTINGS_STR_ENUM("press", "short,long", "short"),
    /* how the battery temperature is shown, here and on the dash */
    SETTINGS_STR_ENUM("temp_unit", "c,f", "c"),
};

static int8_t s_mode   = ONCE;
static int8_t s_button = SW_STAR;
static int8_t s_press  = PRESS_SHORT;
static bool   s_fahrenheit;

int8_t config_server_precon_mode(void)
{
    return s_mode;
}

int8_t config_server_precon_button(void)
{
    return s_button;
}

int8_t config_server_precon_press(void)
{
    return s_press;
}

bool config_server_temperature_fahrenheit(void)
{
    return s_fahrenheit;
}

/* MITM forwarding needs the dual-bus board; the Pro is always parallel */
int8_t config_server_get_fwd_en(void)
{
    return 0;
}

static const char *str_item(const cJSON *settings, const char *key)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(settings, key);

    return cJSON_IsString(item) ? item->valuestring : "";
}

static esp_err_t on_apply(const cJSON *settings)
{
    const char *mode = str_item(settings, "mode");
    const char *button = str_item(settings, "button");

    s_mode = ONCE;
    if (strcmp(mode, "continuous") == 0)
    {
        s_mode = CONTINUOUS;
    }
    else if (strcmp(mode, "persistent") == 0)
    {
        s_mode = PERSISTENT;
    }

    s_button = SW_STAR;
    for (size_t i = 0; i < sizeof(BUTTONS) / sizeof(BUTTONS[0]); i++)
    {
        if (strcmp(button, BUTTONS[i].name) == 0)
        {
            s_button = BUTTONS[i].id;
            break;
        }
    }

    s_press = (strcmp(str_item(settings, "press"), "long") == 0)
                  ? PRESS_LONG : PRESS_SHORT;
    s_fahrenheit = strcmp(str_item(settings, "temp_unit"), "f") == 0;
    return ESP_OK;
}

esp_err_t main_precondition_register_settings(void)
{
    static const settings_descriptor_t DESC =
    {
        .name = "precondition",
        .version = 1,
        .fields = FIELDS,
        .field_count = sizeof(FIELDS) / sizeof(FIELDS[0]),
        .on_apply = on_apply,
    };

    return settings_manager_register(&DESC);
}

esp_err_t main_precondition_can_defaults(void)
{
    /* Nobody has saved CAN settings yet: a fresh device, a factory reset, or
       a Pro that ran other firmware and never had the bus configured. Saved
       settings, the bus off included, are the user's and stay as they are. */
    if (settings_manager_is_saved("can_manager"))
    {
        return ESP_OK;
    }

    /* can_manager ships with the bus off, and preconditioning needs it on.
       set() replaces the whole group, but with nothing saved the other
       fields are still their defaults (baud 500, silent off), so they stay
       as they are. Saving before settings_manager_start() means this boot
       already uses it. */
    cJSON *in = cJSON_CreateObject();
    if (in == NULL || cJSON_AddBoolToObject(in, "enabled", true) == NULL)
    {
        cJSON_Delete(in);
        return ESP_ERR_NO_MEM;
    }

    char err[64] = "";
    esp_err_t r = settings_manager_set("can_manager", in, err, sizeof(err),
                                       NULL);
    cJSON_Delete(in);
    if (r != ESP_OK)
    {
        ESP_LOGW(TAG, "could not enable the CAN bus by default: %s",
                 err[0] != '\0' ? err : esp_err_to_name(r));
        return r;
    }
    ESP_LOGI(TAG, "fresh settings: CAN bus enabled for preconditioning");
    return ESP_OK;
}

/* ---- the bus ------------------------------------------------------------- */

esp_err_t can_send(can_bus_t bus, twai_message_t *message,
                   TickType_t ticks_to_wait)
{
    static uint32_t drop_cnt;
    esp_err_t err = ESP_ERR_INVALID_STATE;

    /* can_manager_send() bounds its own wait, and only waits at all when
       the node can't talk yet, so ask first rather than stall the task
       that also runs the tick */
    (void)ticks_to_wait;
    if (bus != CAN_BUS_0)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (can_manager_tx_ready())
    {
        err = can_manager_send(message->identifier, message->extd,
                               message->rtr, message->data,
                               message->data_length_code);
    }
    if (err != ESP_OK && (++drop_cnt % 64U) == 1U)
    {
        ESP_LOGW(TAG, "%lu frames not sent (%s)", (unsigned long)drop_cnt,
                 esp_err_to_name(err));
    }
    return err;
}

static QueueHandle_t s_q;
static StaticQueue_t s_q_buf;
static uint8_t s_q_store[PRECON_Q_LEN * sizeof(can_core_frame_t)]
    EXT_RAM_BSS_ATTR;
static int s_sub = -1;

static bool still_subscribed(void)
{
    can_manager_subscriber_t sub;

    /* after a can_manager_stop() our index is free, or someone else's */
    return s_sub >= 0 && can_manager_subscriber_get(s_sub, &sub)
           && sub.name != NULL && strcmp(sub.name, PRECON_SUB_NAME) == 0;
}

static void subscribe(void)
{
    static bool warned;
    int idx;

    if (still_subscribed())
    {
        return;
    }

    s_sub = -1;
    xQueueReset(s_q);
    esp_err_t err = can_manager_subscribe_queue(s_q, 0, 0, false, true,
                                                &idx);
    if (err == ESP_OK)
    {
        s_sub = idx;
        (void)can_manager_subscriber_name(idx, PRECON_SUB_NAME);
        warned = false;
        ESP_LOGI(TAG, "listening on the CAN bus");
    }
    else if (!warned)
    {
        warned = true;
        ESP_LOGW(TAG, "no CAN bus (%s): needs can_manager.enabled, "
                      "baud 500, silent off", esp_err_to_name(err));
    }
}

static void frame_in(const can_core_frame_t *f)
{
    twai_message_t msg = { 0 };

    msg.identifier = f->id;
    msg.extd = f->ext;
    msg.rtr = f->rtr;
    msg.data_length_code = f->dlc > 8 ? 8 : f->dlc;
    memcpy(msg.data, f->data, msg.data_length_code);

    precondition_can_rx_hook(&msg, CAN_BUS_0);

    /* parallel mode: the original is already on the wire, so only a
       modified copy alongside it has any effect */
    twai_message_t fwd = msg;
    if (precondition_fwd_hook(&fwd, CAN_BUS_0) == FWD_MODIFIED)
    {
        (void)can_send(CAN_BUS_0, &fwd, 1);
    }
}

/* The HV battery temperature and state of charge precondition.c decodes, as
   dashboard values. autopid carries them on to HA, the logger and rules. */
static void publish_battery(int64_t now)
{
    static int64_t published_at;
    static int64_t temp_at;
    static int64_t soc_at;
    precondition_temperature_t t;
    precondition_soc_t soc;

    if (now - published_at < PRECON_PUB_US)
    {
        return;
    }
    published_at = now;

    if (precondition_get_battery_temperature(&t) && t.updated_at_us != temp_at)
    {
        temp_at = t.updated_at_us;
        if (s_fahrenheit)
        {
            (void)autopid_publish_external("hv_batt_temp_min", "\xC2\xB0" "F",
                                           t.min_c * 9.0 / 5.0 + 32.0);
            (void)autopid_publish_external("hv_batt_temp_max", "\xC2\xB0" "F",
                                           t.max_c * 9.0 / 5.0 + 32.0);
        }
        else
        {
            (void)autopid_publish_external("hv_batt_temp_min", "\xC2\xB0" "C",
                                           t.min_c);
            (void)autopid_publish_external("hv_batt_temp_max", "\xC2\xB0" "C",
                                           t.max_c);
        }
    }
    if (precondition_get_battery_soc(&soc) && soc.updated_at_us != soc_at)
    {
        soc_at = soc.updated_at_us;
        (void)autopid_publish_external("hv_batt_soc", "%",
                                       soc.raw * BATTERY_SOC_SCALE);
    }
}

/* Frames in from can_manager. The tick runs in its own task, as it does in
   the old firmware; the state machine locks around both. */
static void precondition_rx_task(void *arg)
{
    int64_t next_sub_check = 0;
    can_core_frame_t f;

    (void)arg;
    for (;;)
    {
        int64_t now = esp_timer_get_time();

        if (now >= next_sub_check)
        {
            subscribe();
            next_sub_check = now + PRECON_SUB_CHECK_US;
        }
        publish_battery(now);

        if (xQueueReceive(s_q, &f, pdMS_TO_TICKS(PRECON_RX_IDLE_MS)) == pdTRUE)
        {
            frame_in(&f);
        }
    }
}

/* ---- the tick ------------------------------------------------------------ */

/* As in the old firmware's main.c: a one-shot esp_timer wakes a dedicated
   task, which ticks and rearms the timer, so at least 40 ms pass between
   ticks even when the task runs late. The esp_timer task is shared and high
   priority, so its callback does nothing but wake the worker. */
static TaskHandle_t s_tick_task;
static esp_timer_handle_t s_tick_timer;

static void tick_timer_cb(void *arg)
{
    (void)arg;
    if (s_tick_task != NULL)
    {
        xTaskNotifyGive(s_tick_task);
    }
}

static void precondition_tick_task(void *arg)
{
    (void)arg;
    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        /* sleep_manager clears AWAKE before it takes the bus down, and a
           wake is a reboot */
        if (dev_status_manager_is_set(DEV_STATUS_BIT_AWAKE))
        {
            precondition_tick();
        }
        (void)esp_timer_start_once(s_tick_timer, PRECON_TICK_US);
    }
}

esp_err_t main_precondition_start(void)
{
    static StaticTask_t rx_tcb;
    static StackType_t rx_stack[PRECON_RX_TASK_STACK];
    static StaticTask_t tick_tcb;
    static StackType_t tick_stack[PRECON_TICK_TASK_STACK];

    s_q = xQueueCreateStatic(PRECON_Q_LEN, sizeof(can_core_frame_t),
                             s_q_store, &s_q_buf);
    if (s_q == NULL)
    {
        return ESP_FAIL;
    }

    /* both read the settings applied above; precondition_init also starts
       the NVS writer and the track popup's ISO-TP worker */
    beep_init();
    precondition_init();

    const esp_timer_create_args_t timer_args =
    {
        .callback = tick_timer_cb,
        .name = "precondition",
    };
    if (esp_timer_create(&timer_args, &s_tick_timer) != ESP_OK)
    {
        return ESP_FAIL;
    }
    s_tick_task = xTaskCreateStatic(precondition_tick_task, "precondition",
                                    PRECON_TICK_TASK_STACK, NULL,
                                    PRECON_TICK_TASK_PRIO, tick_stack,
                                    &tick_tcb);
    if (s_tick_task == NULL
        || xTaskCreateStatic(precondition_rx_task, "precondition_rx",
                             PRECON_RX_TASK_STACK, NULL, PRECON_RX_TASK_PRIO,
                             rx_stack, &rx_tcb) == NULL)
    {
        return ESP_FAIL;
    }
    return esp_timer_start_once(s_tick_timer, PRECON_TICK_US);
}
