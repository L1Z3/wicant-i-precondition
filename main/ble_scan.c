#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "cJSON.h"

#include "ble_scan.h"
#include "config_server.h"

#define TAG "BLE_SCAN"
#define MAX_DEVICES 64
#define AD_MAX 31
#define UUID_MAX 160

typedef struct {
    uint8_t bda[6];
    uint8_t addr_type;
    int rssi;
    uint32_t count;
    int64_t first_us;
    int64_t last_us;
    uint8_t adv[AD_MAX];
    uint8_t adv_len;
    uint8_t rsp[AD_MAX];
    uint8_t rsp_len;
    char name[32];
    char uuid[UUID_MAX];
} scan_dev_t;

static scan_dev_t s_devs[MAX_DEVICES];
static uint8_t s_dev_count;
static SemaphoreHandle_t s_lock;
static SemaphoreHandle_t s_scan_lock;
static bool s_stack_ready;
static bool s_scan_active;
static ble_scan_report_cb_t s_obs;
static void *s_obs_ctx;

static void ensure_lock(void)
{
    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutex();
    }
}

static bool ensure_scan_lock(void)
{
    if (s_scan_lock == NULL) {
        s_scan_lock = xSemaphoreCreateMutex();
    }
    return s_scan_lock != NULL;
}

static void addr_to_str(const uint8_t bda[6], char out[18])
{
    snprintf(out, 18, "%02x:%02x:%02x:%02x:%02x:%02x",
             bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
}

static char *to_hex(const uint8_t *buf, uint8_t len)
{
    char *out = malloc((size_t)len * 2 + 1);
    if (out == NULL) {
        return NULL;
    }
    for (uint8_t i = 0; i < len; i++) {
        snprintf(out + (size_t)i * 2, 3, "%02x", buf[i]);
    }
    out[(size_t)len * 2] = '\0';
    return out;
}

static void append_uuid(char *dst, const char *s)
{
    size_t used = strlen(dst);
    size_t add = strlen(s);
    if (used + 1 + add + 1 > UUID_MAX) {
        return;
    }
    if (used > 0) {
        dst[used++] = ',';
    }
    memcpy(dst + used, s, add);
    dst[used + add] = '\0';
}

/* Canonical 128-bit Bluetooth UUID strings. 16- and 32-bit service UUIDs are
 * expanded with the Bluetooth base UUID so they match what nRF Connect shows. */
static void uuid16_to_str(uint16_t u, char *out, size_t cap)
{
    snprintf(out, cap, "0000%04x-0000-1000-8000-00805f9b34fb", u);
}

static void uuid32_to_str(uint32_t u, char *out, size_t cap)
{
    snprintf(out, cap, "%08x-0000-1000-8000-00805f9b34fb", (unsigned)u);
}

static void uuid128_to_str(const uint8_t v[16], char *out, size_t cap)
{
    /* Advertising data stores the 128-bit UUID least-significant byte first. */
    snprintf(out, cap, "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
             v[15], v[14], v[13], v[12],
             v[11], v[10],
             v[9], v[8],
             v[7], v[6],
             v[5], v[4], v[3], v[2], v[1], v[0]);
}

/* Parse the AD structures: name (0x08/0x09), 16-bit UUIDs (0x02/0x03),
 * 32-bit UUIDs (0x04/0x05), 128-bit UUIDs (0x06/0x07). */
static void parse_ad(const uint8_t *data, uint8_t len, char *name, char *uuid)
{
    uint8_t i = 0;
    while (i + 1 < len) {
        uint8_t l = data[i];       /* AD length, includes the type byte */
        if (l == 0) {
            break;
        }
        if ((size_t)i + 1 + l > len) {
            break;
        }
        uint8_t type = data[i + 1];
        const uint8_t *v = data + i + 2;
        uint8_t vlen = (uint8_t)(l - 1);
        if (type == 0x09 || type == 0x08) {
            uint8_t n = (vlen < 31) ? vlen : 31;
            memcpy(name, v, n);
            name[n] = '\0';
            for (uint8_t k = n; k > 0; k--) {
                if (name[k - 1] == ' ' || name[k - 1] == '\0') {
                    name[k - 1] = '\0';
                } else {
                    break;
                }
            }
        } else if (type == 0x03 || type == 0x02) {
            for (uint8_t j = 0; j + 1 < vlen; j += 2) {
                uint16_t u = (uint16_t)v[j] | ((uint16_t)v[j + 1] << 8);
                char tmp[40];
                uuid16_to_str(u, tmp, sizeof(tmp));
                append_uuid(uuid, tmp);
            }
        } else if (type == 0x05 || type == 0x04) {
            for (uint8_t j = 0; j + 3 < vlen; j += 4) {
                uint32_t u = (uint32_t)v[j] | ((uint32_t)v[j + 1] << 8)
                             | ((uint32_t)v[j + 2] << 16) | ((uint32_t)v[j + 3] << 24);
                char tmp[40];
                uuid32_to_str(u, tmp, sizeof(tmp));
                append_uuid(uuid, tmp);
            }
        } else if (type == 0x07 || type == 0x06) {
            for (uint8_t j = 0; j + 15 < vlen; j += 16) {
                char tmp[40];
                uuid128_to_str(v + j, tmp, sizeof(tmp));
                append_uuid(uuid, tmp);
            }
        }
        i += (uint8_t)(1 + l);
    }
}

static void normalize_uuid(const char *src, char *out, size_t cap)
{
    size_t j = 0;
    for (size_t i = 0; src[i] != '\0' && j < cap - 1; i++) {
        unsigned char c = (unsigned char)src[i];
        if (isxdigit(c)) {
            out[j++] = (char)tolower(c);
        }
    }
    out[j] = '\0';
}

static bool name_equal(const char *a, const char *b)
{
    if (a == NULL || b == NULL || a[0] == '\0' || b[0] == '\0') {
        return false;
    }
    return strcmp(a, b) == 0;
}

static bool uuid_contains(const char *haystack, const char *needle)
{
    if (haystack == NULL || needle == NULL || needle[0] == '\0') {
        return false;
    }
    char h[UUID_MAX];
    char n[UUID_MAX];
    normalize_uuid(haystack, h, sizeof(h));
    normalize_uuid(needle, n, sizeof(n));
    return n[0] != '\0' && strstr(h, n) != NULL;
}

static void add_report(const struct ble_scan_result_evt_param *r)
{
    scan_dev_t d;
    memset(&d, 0, sizeof(d));
    memcpy(d.bda, r->bda, 6);
    d.addr_type = (uint8_t)r->ble_addr_type;
    d.rssi = r->rssi;
    d.count = 1;
    d.first_us = esp_timer_get_time();
    d.last_us = d.first_us;

    uint8_t adv_len = r->adv_data_len;
    if (adv_len > AD_MAX) {
        adv_len = AD_MAX;
    }
    d.adv_len = adv_len;
    if (adv_len > 0) {
        memcpy(d.adv, r->ble_adv, adv_len);
    }

    uint8_t rsp_len = r->scan_rsp_len;
    if (rsp_len > AD_MAX) {
        rsp_len = AD_MAX;
    }
    d.rsp_len = rsp_len;
    if (rsp_len > 0) {
        memcpy(d.rsp, r->ble_adv + adv_len, rsp_len);
    }

    char name_adv[32] = {0};
    char name_rsp[32] = {0};
    char uuid_adv[UUID_MAX] = {0};
    char uuid_rsp[UUID_MAX] = {0};
    parse_ad(d.adv, d.adv_len, name_adv, uuid_adv);
    if (rsp_len > 0) {
        parse_ad(d.rsp, d.rsp_len, name_rsp, uuid_rsp);
    }

    if (name_adv[0] != '\0') {
        strncpy(d.name, name_adv, sizeof(d.name) - 1);
        d.name[sizeof(d.name) - 1] = '\0';
    } else if (name_rsp[0] != '\0') {
        strncpy(d.name, name_rsp, sizeof(d.name) - 1);
        d.name[sizeof(d.name) - 1] = '\0';
    }
    if (uuid_adv[0] != '\0') {
        strncpy(d.uuid, uuid_adv, sizeof(d.uuid) - 1);
        d.uuid[sizeof(d.uuid) - 1] = '\0';
    } else if (uuid_rsp[0] != '\0') {
        strncpy(d.uuid, uuid_rsp, sizeof(d.uuid) - 1);
        d.uuid[sizeof(d.uuid) - 1] = '\0';
    }

    if (s_obs != NULL) {
        ble_scan_report_t rep = {
            .bda = {0},
            .addr_type = d.addr_type,
            .rssi = d.rssi,
            .adv = d.adv,
            .adv_len = d.adv_len,
            .rsp = d.rsp,
            .rsp_len = d.rsp_len,
            .name = d.name,
            .uuid = d.uuid,
        };
        memcpy(rep.bda, d.bda, sizeof(rep.bda));
        s_obs(&rep, s_obs_ctx);
    }

    ensure_lock();
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (uint8_t i = 0; i < s_dev_count; i++) {
        scan_dev_t *e = &s_devs[i];
        if (memcmp(e->bda, d.bda, 6) == 0) {
            e->count++;
            e->last_us = d.last_us;
            e->rssi = d.rssi;
            if (d.adv_len > 0) {
                e->adv_len = d.adv_len;
                memcpy(e->adv, d.adv, d.adv_len);
            }
            if (d.rsp_len > 0) {
                e->rsp_len = d.rsp_len;
                memcpy(e->rsp, d.rsp, d.rsp_len);
            }
            if (d.name[0] != '\0') {
                strncpy(e->name, d.name, sizeof(e->name) - 1);
                e->name[sizeof(e->name) - 1] = '\0';
            }
            if (d.uuid[0] != '\0') {
                strncpy(e->uuid, d.uuid, sizeof(e->uuid) - 1);
                e->uuid[sizeof(e->uuid) - 1] = '\0';
            }
            xSemaphoreGive(s_lock);
            return;
        }
    }
    if (s_dev_count < MAX_DEVICES) {
        s_devs[s_dev_count++] = d;
    }
    xSemaphoreGive(s_lock);
}

static void gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    if (event == ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT) {
        if (param->scan_param_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGW(TAG, "scan params not accepted: %d", param->scan_param_cmpl.status);
        }
        return;
    }
    if (event != ESP_GAP_BLE_SCAN_RESULT_EVT) {
        return;
    }
    if (param->scan_rst.search_evt != ESP_GAP_SEARCH_INQ_RES_EVT) {
        return;
    }
    add_report(&param->scan_rst);
}

static void stack_teardown(void)
{
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
    s_stack_ready = false;
}

static bool stack_ensure(void)
{
    if (s_stack_ready) {
        return true;
    }
    /* main.c treats any non-zero value (including -1) as "start the GATT
     * server", so refuse in that case too: ble.c then owns the GAP callback. */
    if (config_server_get_ble_config() != 0) {
        ESP_LOGW(TAG, "GATT server (ble_status=enable) owns the stack; scan refused");
        return false;
    }

    esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_bt_controller_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "controller init failed: %s", esp_err_to_name(ret));
        return false;
    }
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "controller enable failed: %s", esp_err_to_name(ret));
        esp_bt_controller_deinit();
        return false;
    }
    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "bluedroid init failed: %s", esp_err_to_name(ret));
        esp_bt_controller_disable();
        esp_bt_controller_deinit();
        return false;
    }
    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "bluedroid enable failed: %s", esp_err_to_name(ret));
        esp_bluedroid_deinit();
        esp_bt_controller_disable();
        esp_bt_controller_deinit();
        return false;
    }
    ret = esp_ble_gap_register_callback(gap_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "gap callback registration failed: %s", esp_err_to_name(ret));
        stack_teardown();
        return false;
    }

    esp_ble_scan_params_t params;
    memset(&params, 0, sizeof(params));
    params.scan_type = BLE_SCAN_TYPE_ACTIVE;
    params.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
    params.scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL;
    params.scan_interval = 160; /* 100 ms; units of 0.625 ms */
    params.scan_window = 80;    /* 50 ms; 50% duty cycle */
    params.scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE;

    ret = esp_ble_gap_set_scan_params(&params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "set scan params failed: %s", esp_err_to_name(ret));
        stack_teardown();
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
    s_stack_ready = true;
    ESP_LOGI(TAG, "BLE scan stack ready");
    return true;
}

bool ble_scan_stack_ready(void)
{
    return s_stack_ready;
}

bool ble_scan_ensure_stack(void)
{
    return stack_ensure();
}

bool ble_scan_start(uint32_t seconds, ble_scan_report_cb_t cb, void *ctx)
{
    if (!ensure_scan_lock()) {
        return false;
    }
    if (xSemaphoreTake(s_scan_lock, 0) != pdTRUE) {
        return false;
    }
    if (!stack_ensure()) {
        xSemaphoreGive(s_scan_lock);
        return false;
    }
    if (seconds == 0) {
        seconds = 1;
    }
    s_obs = cb;
    s_obs_ctx = ctx;
    esp_err_t ret = esp_ble_gap_start_scanning(seconds);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "start async scan failed: %s", esp_err_to_name(ret));
        s_obs = NULL;
        s_obs_ctx = NULL;
        xSemaphoreGive(s_scan_lock);
        return false;
    }
    s_scan_active = true;
    return true;
}

void ble_scan_stop(void)
{
    if (s_scan_lock == NULL || !s_scan_active) {
        return;
    }
    esp_ble_gap_stop_scanning();
    s_obs = NULL;
    s_obs_ctx = NULL;
    s_scan_active = false;
    xSemaphoreGive(s_scan_lock);
}

static bool collect(uint32_t ms)
{
    if (!stack_ensure()) {
        return false;
    }
    ensure_lock();
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_dev_count = 0;
    xSemaphoreGive(s_lock);

    uint32_t sec = (ms + 999) / 1000;
    if (sec == 0) {
        sec = 1;
    }
    esp_err_t ret = esp_ble_gap_start_scanning(sec);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "start scan failed: %s", esp_err_to_name(ret));
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(ms));
    esp_ble_gap_stop_scanning();
    vTaskDelay(pdMS_TO_TICKS(100));
    return true;
}

char *ble_scan_json(uint32_t ms, const char *name, const char *uuid)
{
    if (ms < 1000) {
        ms = 1000;
    }
    if (!ensure_scan_lock()) {
        return NULL;
    }
    if (xSemaphoreTake(s_scan_lock, 0) != pdTRUE) {
        ESP_LOGW(TAG, "scan busy (button listener is scanning)");
        return NULL;
    }
    if (!collect(ms)) {
        xSemaphoreGive(s_scan_lock);
        return NULL;
    }
    xSemaphoreGive(s_scan_lock);

    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        return NULL;
    }
    cJSON *arr = cJSON_CreateArray();
    if (arr == NULL) {
        cJSON_Delete(root);
        return NULL;
    }

    ensure_lock();
    xSemaphoreTake(s_lock, portMAX_DELAY);
    uint8_t total = s_dev_count;
    for (uint8_t i = 0; i < total; i++) {
        const scan_dev_t *d = &s_devs[i];
        bool matches = true;
        if (name != NULL && name[0] != '\0') {
            matches = name_equal(d->name, name);
        }
        if (matches && uuid != NULL && uuid[0] != '\0') {
            matches = uuid_contains(d->uuid, uuid);
        }
        if (!matches) {
            continue;
        }

        cJSON *o = cJSON_CreateObject();
        char astr[18];
        addr_to_str(d->bda, astr);
        cJSON_AddStringToObject(o, "addr", astr);
        cJSON_AddNumberToObject(o, "addr_type", d->addr_type);
        cJSON_AddNumberToObject(o, "rssi", d->rssi);
        cJSON_AddNumberToObject(o, "count", d->count);
        cJSON_AddStringToObject(o, "name", d->name);
        cJSON_AddStringToObject(o, "uuid", d->uuid);
        char *adv = to_hex(d->adv, d->adv_len);
        char *rsp = to_hex(d->rsp, d->rsp_len);
        cJSON_AddStringToObject(o, "adv", adv != NULL ? adv : "");
        cJSON_AddStringToObject(o, "scan_rsp", rsp != NULL ? rsp : "");
        cJSON_AddNumberToObject(o, "first_ms", (double)d->first_us / 1000.0);
        cJSON_AddNumberToObject(o, "last_ms", (double)d->last_us / 1000.0);
        free(adv);
        free(rsp);
        cJSON_AddItemToArray(arr, o);
    }
    xSemaphoreGive(s_lock);

    cJSON_AddNumberToObject(root, "total", total);
    cJSON_AddNumberToObject(root, "matched", cJSON_GetArraySize(arr));
    if (name != NULL) {
        cJSON_AddStringToObject(root, "filter_name", name);
    }
    if (uuid != NULL) {
        cJSON_AddStringToObject(root, "filter_uuid", uuid);
    }
    cJSON_AddItemToObject(root, "devices", arr);

    char *out = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return out;
}
