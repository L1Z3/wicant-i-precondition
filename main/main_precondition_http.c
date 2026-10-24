/**
 * @file main_precondition_http.c
 * @brief Preconditioning over HTTP: its status, the start/stop toggle and
 *        the dash popup. The web UI's Preconditioning page drives them.
 *
 * The old firmware's web UI had these on its home page (POST
 * /precondition_toggle, POST /track_popup, and fields in its status JSON).
 *
 *   GET  /api/precondition         status, see status_handler()
 *   POST /api/precondition/toggle  start, or stop, as the button in the car
 *   POST /api/precondition/popup   text/plain body, shown on the dash
 */
#include <stdio.h>
#include <string.h>

#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "http_server_manager.h"

#include "config_server.h"
#include "precondition.h"
#include "track_popup.h"
#include "utf8_utf16_converter.h"

#include "main_precondition.h"

static const char *TAG = "precondition";

static esp_err_t send_json(httpd_req_t *req, const char *status,
                           const char *body)
{
    if (status != NULL)
    {
        httpd_resp_set_status(req, status);
    }
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t send_error(httpd_req_t *req, const char *status,
                            const char *msg)
{
    char body[96];

    snprintf(body, sizeof(body), "{\"error\":\"%s\"}", msg);
    return send_json(req, status, body);
}

/* NULL for a model this file doesn't know yet: reported as not seen */
static const char *car_model_name(egmp_car_model_t model)
{
    switch (model)
    {
        case EGMP_CAR_IONIQ5: return "ioniq5";
        case EGMP_CAR_IONIQ6: return "ioniq6";
        case EGMP_CAR_EV6:    return "ev6";
        default:              return NULL;
    }
}

/* GET /api/precondition
 * {"requested":bool,"active":bool,"starting":bool,"car_ready":bool,
 *  "car_model":"ioniq5"|"ioniq6"|"ev6"|null,"temp_unit":"c"|"f",
 *  "battery_temp":{"min_c":n,"max_c":n,"age_ms":n}|null,
 *  "battery_soc":{"pct":n,"age_ms":n}|null}
 * car_model is what the status frame identified this boot; the battery
 * values are null until their frames have been seen. */
static esp_err_t status_handler(httpd_req_t *req)
{
    char body[320];
    char model[16] = "null";
    char temp[80] = "null";
    char soc[48] = "null";
    precondition_state_t st = { 0 };
    egmp_car_model_t car;
    precondition_temperature_t t;
    precondition_soc_t s;
    int64_t now = esp_timer_get_time();

    (void)precondition_get_state(&st);
    const char *car_name = precondition_get_car_model(&car)
                               ? car_model_name(car) : NULL;
    if (car_name != NULL)
    {
        snprintf(model, sizeof(model), "\"%s\"", car_name);
    }
    if (precondition_get_battery_temperature(&t))
    {
        snprintf(temp, sizeof(temp),
                 "{\"min_c\":%d,\"max_c\":%d,\"age_ms\":%lld}", t.min_c,
                 t.max_c, (long long)((now - t.updated_at_us) / 1000));
    }
    if (precondition_get_battery_soc(&s))
    {
        snprintf(soc, sizeof(soc), "{\"pct\":%.1f,\"age_ms\":%lld}",
                 s.raw * BATTERY_SOC_SCALE,
                 (long long)((now - s.updated_at_us) / 1000));
    }

    snprintf(body, sizeof(body),
             "{\"requested\":%s,\"active\":%s,\"starting\":%s,"
             "\"car_ready\":%s,\"car_model\":%s,\"temp_unit\":\"%s\","
             "\"battery_temp\":%s,\"battery_soc\":%s}",
             st.requested ? "true" : "false", st.active ? "true" : "false",
             st.starting ? "true" : "false",
             car_in_ready() ? "true" : "false", model,
             config_server_temperature_fahrenheit() ? "f" : "c", temp, soc);
    return send_json(req, NULL, body);
}

/* POST /api/precondition/toggle: the same as a press of the button in the
   car, so it starts preconditioning, or stops it when it is requested */
static esp_err_t toggle_handler(httpd_req_t *req)
{
    precondition_toggle_request();
    return send_json(req, NULL, "{\"ok\":true}");
}

/* POST /api/precondition/popup, the text as a text/plain body: shown on the
   dash as a track popup. 413 past the popup's 50 UTF-16 units, 503 when two
   popups are already waiting. */
static esp_err_t popup_handler(httpd_req_t *req)
{
    char text[TRACK_POPUP_MAX_TEXT_UTF8_BYTES + 1U];
    size_t received = 0U;
    int retries = 0;

    if (req->content_len == 0U)
    {
        return send_error(req, "400 Bad Request", "popup text cannot be empty");
    }
    if (req->content_len > TRACK_POPUP_MAX_TEXT_UTF8_BYTES)
    {
        return send_error(req, "413 Content Too Large",
                          "popup text is too long");
    }

    while (received < req->content_len)
    {
        int n = httpd_req_recv(req, text + received,
                               req->content_len - received);
        /* a slow client: give it a few more receive timeouts, as the
           other upload handlers do */
        if (n == HTTPD_SOCK_ERR_TIMEOUT && retries++ < 5)
        {
            continue;
        }
        if (n <= 0)
        {
            return send_error(req, "408 Request Timeout",
                              "popup text not received");
        }
        received += (size_t)n;
    }

    /* track_popup_show() takes a C string: an embedded NUL would cut the
       text short without saying so */
    if (memchr(text, '\0', received) != NULL)
    {
        return send_error(req, "400 Bad Request",
                          "popup text contains a NUL byte");
    }
    text[received] = '\0';

    /* the byte limit above lets through up to 150 ASCII characters; the
       dash takes 50 UTF-16 units (an emoji is two) */
    if (utf8_to_utf16((const utf8_t *)text, received, NULL, 0U)
        > TRACK_POPUP_MAX_TEXT_CODE_UNITS)
    {
        return send_error(req, "413 Content Too Large",
                          "popup text is too long");
    }

    if (!track_popup_show(text))
    {
        return send_error(req, "503 Service Unavailable", "popup busy");
    }
    return send_json(req, NULL, "{\"ok\":true}");
}

esp_err_t main_precondition_register_http(void)
{
    static const httpd_uri_t URIS[] =
    {
        { .uri = "/api/precondition", .method = HTTP_GET,
          .handler = status_handler },
        { .uri = "/api/precondition/toggle", .method = HTTP_POST,
          .handler = toggle_handler },
        { .uri = "/api/precondition/popup", .method = HTTP_POST,
          .handler = popup_handler },
    };
    esp_err_t err = http_server_manager_register_handlers(
        URIS, sizeof(URIS) / sizeof(URIS[0]));
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "/api/precondition registered");
    }
    return err;
}
