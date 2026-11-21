#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"
#include "cJSON.h"

#include "ble_button.h"
#include "ble_scan.h"
#include "config_server.h"
#include "precondition.h"

#define TAG "BLE_BUTTON"

#define BUTTON_APP_ID 0
#define BUTTON_SVC_UUID 0xFFE0
#define BUTTON_SVC_UUID_ALT 0xFFE3
#define SCAN_SECONDS 6
#define MAX_NOTIFY_CHARS 8
#define NOTIFY_DEBOUNCE_US (1500 * 1000)
#define NOTIFY_SETTLE_US (2500 * 1000)
#define CONNECT_TIMEOUT_US (20 * 1000 * 1000)
#define RECONNECT_DELAY_MS 3000
/* The iTAG advertises continuously even when untouched, and the advertisement
 * is byte-identical for idle and pressed.  A press is only visible as a GATT
 * notification on 0xFFE1 (value 0x01).  Keep a connection open and trigger on
 * that notification; reconnect on the periodic disconnects. */
#define NVS_NAMESPACE "ble_btn"
#define NVS_KEY_BDA "bda"
#define NVS_KEY_TYPE "bda_type"

typedef struct {
    uint8_t bda[6];
    uint8_t addr_type;
    char name[32];
    char uuid[160];
} found_dev_t;

typedef struct {
    uint16_t handle;
    bool indicate;
} notify_char_t;

static TaskHandle_t s_task;
static bool s_started;
static bool s_paused;
static volatile bool s_disconnect_req;
static bool s_learning;
static bool s_registered;
static bool s_connected;
static bool s_connecting;
static volatile bool s_closing;
static volatile bool s_cancel_open;
static bool s_subscribed;
static uint16_t s_gattc_if;
static uint16_t s_conn_id;
static uint8_t s_scan_round;
static int64_t s_connect_us;

static found_dev_t s_found;
static volatile bool s_found_flag;

static uint8_t s_bound_bda[6];
static uint8_t s_bound_type;
static bool s_have_bound;
static uint8_t s_connect_fails;

static uint16_t s_svc_start;
static uint16_t s_svc_end;
static bool s_svc_found;

static notify_char_t s_notify[MAX_NOTIFY_CHARS];
static uint8_t s_notify_count;
static int64_t s_subscribed_us;
static int64_t s_last_notify_us;
static int64_t s_last_trigger_us;
static int64_t s_last_seen_us;
static uint32_t s_press_count;
static uint8_t s_last_value[20];
static uint16_t s_last_len;
static char s_last_addr[18];

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

/* The identifier the user reported for the specific button.  It is only a
 * fast-path hint: association is still by service UUID.  Both byte orders are
 * accepted because the scan result and phone apps differ on display order. */
static const uint8_t s_hint_a[6] = {0xff, 0xff, 0x12, 0xb9, 0xe9, 0x10};
static const uint8_t s_hint_b[6] = {0x10, 0xe9, 0xb9, 0x12, 0xff, 0xff};

static void addr_to_str(const uint8_t bda[6], char out[18])
{
    snprintf(out, 18, "%02x:%02x:%02x:%02x:%02x:%02x",
             bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
}

static bool uuid_is_button_service(const esp_bt_uuid_t *u)
{
    if (u->len == ESP_UUID_LEN_16) {
        return u->uuid.uuid16 == BUTTON_SVC_UUID ||
               u->uuid.uuid16 == BUTTON_SVC_UUID_ALT;
    }
    if (u->len == ESP_UUID_LEN_32) {
        return u->uuid.uuid32 == BUTTON_SVC_UUID ||
               u->uuid.uuid32 == BUTTON_SVC_UUID_ALT;
    }
    if (u->len == ESP_UUID_LEN_128) {
        /* Bluedroid stores uuid128 little-endian (LSB first), so the 16-bit
         * part of 0000ffe0/ffe3-... sits at bytes 12-13. */
        return (u->uuid.uuid128[12] == 0xe0 || u->uuid.uuid128[12] == 0xe3) &&
               u->uuid.uuid128[13] == 0xff &&
               u->uuid.uuid128[14] == 0x00 && u->uuid.uuid128[15] == 0x00;
    }
    return false;
}

static bool uuid_text_has_button_service(const char *uuid)
{
    return uuid != NULL &&
           (strstr(uuid, "ffe0") != NULL || strstr(uuid, "ffe3") != NULL);
}

static void load_bound(void)
{
    nvs_handle_t h;
    s_have_bound = false;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    size_t len = sizeof(s_bound_bda);
    if (nvs_get_blob(h, NVS_KEY_BDA, s_bound_bda, &len) == ESP_OK && len == sizeof(s_bound_bda)) {
        s_have_bound = true;
        uint8_t type = 0;
        if (nvs_get_u8(h, NVS_KEY_TYPE, &type) == ESP_OK) {
            s_bound_type = type;
        }
    }
    nvs_close(h);
}

static void save_bound(const uint8_t bda[6], uint8_t addr_type)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGW(TAG, "nvs open failed; association not saved");
        return;
    }
    if (nvs_set_blob(h, NVS_KEY_BDA, bda, 6) != ESP_OK ||
        nvs_set_u8(h, NVS_KEY_TYPE, addr_type) != ESP_OK ||
        nvs_commit(h) != ESP_OK) {
        ESP_LOGW(TAG, "nvs write failed; association not saved");
    }
    nvs_close(h);
    memcpy(s_bound_bda, bda, 6);
    s_bound_type = addr_type;
    s_have_bound = true;
}

static void clear_bound(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_key(h, NVS_KEY_BDA);
        nvs_erase_key(h, NVS_KEY_TYPE);
        nvs_commit(h);
        nvs_close(h);
    }
    s_have_bound = false;
}

static void scan_cb(const ble_scan_report_t *r, void *ctx)
{
    (void)ctx;
    bool addr_match = s_have_bound && memcmp(r->bda, s_bound_bda, 6) == 0;
    bool hint_match = memcmp(r->bda, s_hint_a, 6) == 0 || memcmp(r->bda, s_hint_b, 6) == 0;
    bool uuid_match = uuid_text_has_button_service(r->uuid);
    if (!addr_match && !hint_match && !uuid_match) {
        return;
    }

    int64_t now = esp_timer_get_time();
    char astr[18];
    addr_to_str(r->bda, astr);
    s_last_seen_us = now;
    s_found_flag = true;

    portENTER_CRITICAL(&s_mux);
    memcpy(s_found.bda, r->bda, 6);
    s_found.addr_type = r->addr_type;
    strlcpy(s_found.name, r->name, sizeof(s_found.name));
    strlcpy(s_found.uuid, r->uuid, sizeof(s_found.uuid));
    portEXIT_CRITICAL(&s_mux);

    ESP_LOGI(TAG, "ad %s type %u rssi %d uuid '%s'", astr, r->addr_type, r->rssi, r->uuid);

    /* Learn, or associate with the first button service seen.  The press that
     * wakes the device is consumed by the association; after the connection is
     * up, every 0xFFE1 notification is a real press. */
    if (s_learning || !s_have_bound) {
        save_bound(r->bda, r->addr_type);
        s_learning = false;
        s_scan_round = 0;
        ESP_LOGI(TAG, "associated %s (addr_type %u, rssi %d)",
                 astr, r->addr_type, r->rssi);
        return;
    }

    /* A different 0xFFE0/0xFFE3 device is only adopted after a full round in
     * which the learned address was absent. */
    if (!addr_match && !hint_match) {
        if (s_scan_round == 0) {
            return;
        }
        save_bound(r->bda, r->addr_type);
        s_scan_round = 0;
        ESP_LOGI(TAG, "re-associated %s", astr);
    }
}

static void disconnect_now(void)
{
    if (s_connected && !s_closing) {
        s_closing = true;
        esp_err_t err = esp_ble_gattc_close(s_gattc_if, s_conn_id);
        if (err != ESP_OK) {
            s_closing = false;
            ESP_LOGW(TAG, "close failed: %s", esp_err_to_name(err));
        }
    } else if (s_connecting) {
        /* No conn_id exists until OPEN_EVT; cannot close a pending open. */
        s_cancel_open = true;
    }
}

static void enable_cccd(uint16_t char_handle)
{
    bool indicate = false;
    for (uint8_t i = 0; i < s_notify_count; i++) {
        if (s_notify[i].handle == char_handle) {
            indicate = s_notify[i].indicate;
            break;
        }
    }

    esp_bt_uuid_t descr_uuid;
    memset(&descr_uuid, 0, sizeof(descr_uuid));
    descr_uuid.len = ESP_UUID_LEN_16;
    descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

    esp_gattc_descr_elem_t descr;
    uint16_t count = 1;
    esp_gatt_status_t st = esp_ble_gattc_get_descr_by_char_handle(
        s_gattc_if, s_conn_id, char_handle, descr_uuid, &descr, &count);
    if (st != ESP_GATT_OK || count == 0) {
        ESP_LOGW(TAG, "CCCD not found for char handle %u (status %d)", char_handle, st);
        return;
    }

    uint8_t val[2] = {(uint8_t)(indicate ? 2 : 1), 0};
    esp_err_t ret = esp_ble_gattc_write_char_descr(
        s_gattc_if, s_conn_id, descr.handle, sizeof(val), val,
        ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "CCCD write failed: %s", esp_err_to_name(ret));
    }
}

static void discover_chars(void)
{
    uint16_t offset = 0;
    bool any = false;

    while (offset <= s_svc_end) {
        esp_gattc_char_elem_t chars[16];
        uint16_t count = 16;
        esp_gatt_status_t st = esp_ble_gattc_get_all_char(
            s_gattc_if, s_conn_id, s_svc_start, s_svc_end, chars, &count, offset);
        if (st != ESP_GATT_OK) {
            ESP_LOGW(TAG, "get_all_char failed at offset %u (status %d)", offset, st);
            break;
        }

        for (uint16_t i = 0; i < count; i++) {
            esp_gatt_char_prop_t props = chars[i].properties;
            if ((props & (ESP_GATT_CHAR_PROP_BIT_NOTIFY | ESP_GATT_CHAR_PROP_BIT_INDICATE)) == 0) {
                continue;
            }
            if (s_notify_count >= MAX_NOTIFY_CHARS) {
                break;
            }
            s_notify[s_notify_count].handle = chars[i].char_handle;
            s_notify[s_notify_count].indicate =
                (props & ESP_GATT_CHAR_PROP_BIT_INDICATE) &&
                !(props & ESP_GATT_CHAR_PROP_BIT_NOTIFY);
            s_notify_count++;
            any = true;

            esp_err_t ret = esp_ble_gattc_register_for_notify(
                s_gattc_if, s_bound_bda, chars[i].char_handle);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "register_for_notify(0x%04x) failed: %s",
                         chars[i].char_handle, esp_err_to_name(ret));
            }
        }

        if (count < 16) {
            break;
        }
        offset += count;
    }

    if (!any) {
        ESP_LOGW(TAG, "button service 0xFFE0/0xFFE3 has no notify/indicate characteristic");
        disconnect_now();
    }
}

static void handle_notify(esp_ble_gattc_cb_param_t *p)
{
    if (!s_subscribed) {
        return;
    }
    int64_t now = esp_timer_get_time();
    if (s_subscribed_us != 0 && now - s_subscribed_us < NOTIFY_SETTLE_US) {
        return; /* initial value pushed right after CCCD write */
    }
    if (p->notify.value_len == 0) {
        return;
    }
    if (now - s_last_notify_us < NOTIFY_DEBOUNCE_US) {
        return;
    }

    uint8_t value = p->notify.value[0];
    s_last_notify_us = now;
    s_last_len = p->notify.value_len > sizeof(s_last_value)
                     ? sizeof(s_last_value)
                     : p->notify.value_len;
    memcpy(s_last_value, p->notify.value, s_last_len);
    addr_to_str(p->notify.remote_bda, s_last_addr);

    if (value != 0x01) {
        ESP_LOGI(TAG, "notification 0x%02x ignored (not a press)", value);
        return;
    }

    s_last_trigger_us = now;
    s_press_count++;
    ESP_LOGI(TAG, "button press from %s (handle 0x%04x, count %u)",
             s_last_addr, p->notify.handle, (unsigned)s_press_count);
    precondition_toggle_request();
}

static void gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                     esp_ble_gattc_cb_param_t *p)
{
    switch (event) {
    case ESP_GATTC_REG_EVT:
        if (p->reg.status == ESP_GATT_OK) {
            s_gattc_if = gattc_if;
            s_registered = true;
            ESP_LOGI(TAG, "GATTC registered, interface %u", gattc_if);
        } else {
            ESP_LOGE(TAG, "GATTC registration failed: %d", p->reg.status);
        }
        break;

    case ESP_GATTC_OPEN_EVT:
        s_connecting = false;
        s_closing = false;
        if (p->open.status == ESP_GATT_OK) {
            s_conn_id = p->open.conn_id;
            s_connected = true;
            s_connect_fails = 0;
            if (s_cancel_open) {
                s_cancel_open = false;
                ESP_LOGI(TAG, "cancelled pending connect, conn_id %u", s_conn_id);
                disconnect_now();
                break;
            }
            s_svc_found = false;
            s_notify_count = 0;
            ESP_LOGI(TAG, "connected, conn_id %u", s_conn_id);
            esp_ble_gattc_search_service(gattc_if, s_conn_id, NULL);
        } else {
            s_connected = false;
            s_cancel_open = false;
            s_connect_fails++;
            ESP_LOGW(TAG, "connect failed: %d (attempt %u)", p->open.status, s_connect_fails);
        }
        break;

    case ESP_GATTC_SEARCH_RES_EVT:
        if (uuid_is_button_service(&p->search_res.srvc_id.uuid)) {
            s_svc_start = p->search_res.start_handle;
            s_svc_end = p->search_res.end_handle;
            s_svc_found = true;
            ESP_LOGI(TAG, "button service handles 0x%04x-0x%04x",
                     s_svc_start, s_svc_end);
        }
        break;

    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (p->search_cmpl.status == ESP_GATT_OK && s_svc_found) {
            discover_chars();
        } else {
            ESP_LOGW(TAG, "service discovery incomplete (status %d, found %d)",
                     p->search_cmpl.status, s_svc_found);
            disconnect_now();
        }
        break;

    case ESP_GATTC_REG_FOR_NOTIFY_EVT:
        if (p->reg_for_notify.status == ESP_GATT_OK) {
            enable_cccd(p->reg_for_notify.handle);
        } else {
            ESP_LOGW(TAG, "register_for_notify failed: %d", p->reg_for_notify.status);
        }
        break;

    case ESP_GATTC_WRITE_DESCR_EVT:
        if (p->write.status == ESP_GATT_OK) {
            s_subscribed = true;
            s_subscribed_us = esp_timer_get_time();
            ESP_LOGI(TAG, "notifications enabled");
        } else {
            ESP_LOGW(TAG, "CCCD write status %d", p->write.status);
        }
        break;

    case ESP_GATTC_NOTIFY_EVT:
        handle_notify(p);
        break;

    case ESP_GATTC_CLOSE_EVT:
    case ESP_GATTC_DISCONNECT_EVT:
        if (s_connected || s_connecting || s_closing) {
            ESP_LOGW(TAG, "disconnected");
        }
        s_connected = false;
        s_connecting = false;
        s_closing = false;
        s_subscribed = false;
        s_svc_found = false;
        s_notify_count = 0;
        s_subscribed_us = 0;
        break;

    default:
        break;
    }
}

static void connect_found(void)
{
    found_dev_t f;
    portENTER_CRITICAL(&s_mux);
    f = s_found;
    portEXIT_CRITICAL(&s_mux);

    char astr[18];
    addr_to_str(f.bda, astr);
    ESP_LOGI(TAG, "connecting %s (addr_type %u, name '%s')", astr, f.addr_type, f.name);

    if (s_learning || !s_have_bound || memcmp(s_bound_bda, f.bda, 6) != 0) {
        save_bound(f.bda, f.addr_type);
    }
    s_learning = false;
    s_scan_round = 0;

    s_cancel_open = false;
    s_connecting = true;
    s_connect_us = esp_timer_get_time();
    esp_err_t ret = esp_ble_gattc_open(s_gattc_if, f.bda,
                                       (esp_ble_addr_type_t)f.addr_type, true);
    if (ret != ESP_OK) {
        s_connecting = false;
        s_cancel_open = false;
        s_connect_fails++;
        ESP_LOGW(TAG, "open failed: %s (attempt %u)", esp_err_to_name(ret), s_connect_fails);
        vTaskDelay(pdMS_TO_TICKS(RECONNECT_DELAY_MS));
    }
}

static void button_task(void *arg)
{
    (void)arg;
    bool logged_disabled = false;
    while (!ble_scan_ensure_stack()) {
        if (!logged_disabled) {
            ESP_LOGW(TAG, "BLE stack unavailable (ble_status=enable?); listener idle");
            logged_disabled = true;
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    esp_err_t ret = esp_ble_gattc_register_callback(gattc_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GATTC callback registration failed: %s", esp_err_to_name(ret));
    }
    ret = esp_ble_gattc_app_register(BUTTON_APP_ID);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GATTC app registration failed: %s", esp_err_to_name(ret));
    }

    while (1) {
        if (s_disconnect_req) {
            s_disconnect_req = false;
            disconnect_now();
        }
        if (s_paused) {
            if (s_connected || s_connecting) {
                disconnect_now();
            }
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        if (!s_registered) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        if (s_connected) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        if (s_connecting) {
            if (esp_timer_get_time() - s_connect_us > CONNECT_TIMEOUT_US) {
                ESP_LOGW(TAG, "connect timeout (attempt %u)", s_connect_fails + 1);
                s_connect_fails++;
                s_connecting = false;
                s_cancel_open = true;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        s_found_flag = false;
        if (!ble_scan_start(SCAN_SECONDS, scan_cb, NULL)) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(SCAN_SECONDS * 1000));
        ble_scan_stop();
        vTaskDelay(pdMS_TO_TICKS(300));

        if (s_found_flag) {
            connect_found();
        } else {
            s_scan_round = 1; /* next pass accepts any 0xFFE0/0xFFE3 device */
            s_connect_fails = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

bool ble_button_init(void)
{
    if (s_started) {
        return true;
    }
    if (config_server_get_ble_config() != 0) {
        ESP_LOGW(TAG, "ble_status=enable owns the BLE stack; button listener disabled");
        return false;
    }
    load_bound();
    BaseType_t ok = xTaskCreate(button_task, "ble_button", 4096, NULL, 4, &s_task);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "task create failed");
        return false;
    }
    s_started = true;
    ESP_LOGI(TAG, "listener started; service UUID 0xFFE0/0xFFE3");
    return true;
}

void ble_button_set_paused(bool paused)
{
    s_paused = paused;
}

bool ble_button_paused(void)
{
    return s_paused;
}

void ble_button_learn(void)
{
    clear_bound();
    s_learning = true;
    s_scan_round = 0;
    s_paused = false;
    s_last_trigger_us = 0;
    s_last_seen_us = 0;
    s_press_count = 0;
    s_connect_fails = 0;
    s_found_flag = false;
    s_disconnect_req = true;
}

void ble_button_forget(void)
{
    clear_bound();
    s_learning = false;
    s_scan_round = 0;
    s_paused = true;
    s_disconnect_req = true;
}

char *ble_button_status_json(void)
{
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        return NULL;
    }

    char bound[18] = "";
    if (s_have_bound) {
        addr_to_str(s_bound_bda, bound);
    }

    cJSON_AddBoolToObject(root, "started", s_started);
    cJSON_AddBoolToObject(root, "paused", s_paused);
    cJSON_AddBoolToObject(root, "learning", s_learning);
    cJSON_AddBoolToObject(root, "registered", s_registered);
    cJSON_AddBoolToObject(root, "connected", s_connected);
    cJSON_AddBoolToObject(root, "subscribed", s_subscribed);
    cJSON_AddNumberToObject(root, "conn_id", s_conn_id);
    cJSON_AddStringToObject(root, "service_uuid", "0000ffe0-0000-1000-8000-00805f9b34fb");
    cJSON_AddStringToObject(root, "service_uuid_alt", "0000ffe3-0000-1000-8000-00805f9b34fb");
    cJSON_AddStringToObject(root, "bound_addr", bound);
    cJSON_AddBoolToObject(root, "paired", s_have_bound);
    cJSON_AddNumberToObject(root, "notify_chars", s_notify_count);
    cJSON_AddStringToObject(root, "trigger", "notification");
    cJSON_AddNumberToObject(root, "press_count", s_press_count);
    cJSON_AddNumberToObject(root, "connect_fails", s_connect_fails);
    if (s_last_trigger_us != 0) {
        cJSON_AddNumberToObject(root, "last_press_age_ms",
                                (double)(esp_timer_get_time() - s_last_trigger_us) / 1000.0);
    }
    if (s_last_seen_us != 0) {
        cJSON_AddNumberToObject(root, "last_seen_age_ms",
                                (double)(esp_timer_get_time() - s_last_seen_us) / 1000.0);
    }

    char hex[20 * 2 + 1] = "";
    size_t pos = 0;
    for (uint16_t i = 0; i < s_last_len && pos + 2 < sizeof(hex); i++) {
        pos += (size_t)snprintf(hex + pos, sizeof(hex) - pos, "%02x", s_last_value[i]);
    }
    cJSON_AddStringToObject(root, "last_addr", s_last_addr);
    cJSON_AddStringToObject(root, "last_value", hex);
    if (s_last_notify_us != 0) {
        cJSON_AddNumberToObject(root, "last_notify_age_ms",
                                (double)(esp_timer_get_time() - s_last_notify_us) / 1000.0);
    }

    char *out = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return out;
}
