#include "esp_twai_mcp251xfd.h"
#include "esp_private/twai_interface.h"
#include "mcp251xfd_core.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#if !CONFIG_FREERTOS_UNICORE
#include "esp_ipc.h"
#endif
#include <string.h>

#define TAG "mcp251xfd"
#define IDLE_BIT BIT0

typedef struct {
    struct twai_node_base base;
    mcp251xfd_core_t core;
    spi_host_device_t spi_host;
    spi_device_handle_t spi;
    uint32_t spi_hz;
    gpio_num_t cs_gpio;
    gpio_num_t int_gpio;
    SemaphoreHandle_t lock;
    SemaphoreHandle_t tx_space;
    SemaphoreHandle_t worker_exited;
    EventGroupHandle_t events;
    TaskHandle_t worker;
    twai_event_callbacks_t callbacks;
    void *user_data;
    uint32_t timestamp_resolution_hz;
    twai_frame_t rx_cache;
    uint8_t rx_data[8];
    bool rx_pending;
    bool in_rx_callback;
    bool isr_installed;
    bool enabled;
    bool stopping;
} mcp251xfd_node_t;

static mcp251xfd_node_t *node_context(twai_node_handle_t node)
{
    // The interface is the first field.
    return (mcp251xfd_node_t *)node;
}

static esp_err_t to_esp_error(eERRORRESULT error)
{
    switch (error) {
    case ERR_NONE: return ESP_OK;
    case ERR__PARAMETER_ERROR:
    case ERR__BITTIME_ERROR:
    case ERR__SPI_FREQUENCY_ERROR: return ESP_ERR_INVALID_ARG;
    case ERR__NOT_READY:
    case ERR__NEED_CONFIG_MODE: return ESP_ERR_INVALID_STATE;
    case ERR__DEVICE_TIMEOUT:
    case ERR__SPI_TIMEOUT: return ESP_ERR_TIMEOUT;
    case ERR__NOT_SUPPORTED: return ESP_ERR_NOT_SUPPORTED;
    case ERR__OUT_OF_MEMORY: return ESP_ERR_NO_MEM;
    case ERR__NO_DEVICE_DETECTED:
    case ERR__UNKNOWN_DEVICE: return ESP_ERR_NOT_FOUND;
    default: return ESP_FAIL;
    }
}

static eERRORRESULT spi_init(void *arg, uint8_t chip_select, uint32_t hz)
{
    (void)chip_select;
    mcp251xfd_node_t *ctx = arg;
    if (ctx->spi && ctx->spi_hz == hz) return ERR_NONE;
    // Only called during creation, before the worker or GPIO ISR is installed.
    if (ctx->spi) {
        if (spi_bus_remove_device(ctx->spi) != ESP_OK) return ERR__SPI_CONFIG_ERROR;
        ctx->spi = NULL;
    }
    spi_device_interface_config_t config = {
        .clock_speed_hz = hz, .mode = 0,
        .spics_io_num = ctx->cs_gpio, .queue_size = 1,
    };
    if (spi_bus_add_device(ctx->spi_host, &config, &ctx->spi) != ESP_OK) return ERR__SPI_CONFIG_ERROR;
    ctx->spi_hz = hz;
    return ERR_NONE;
}

static eERRORRESULT spi_transfer(void *arg, uint8_t chip_select, uint8_t *tx, uint8_t *rx, size_t size)
{
    (void)chip_select;
    mcp251xfd_node_t *ctx = arg;
    spi_transaction_t transaction = {.length = size * 8, .tx_buffer = tx, .rx_buffer = rx};
    // One lock surrounds whole core operations, not just individual transfers.
    esp_err_t error = spi_device_polling_transmit(ctx->spi, &transaction);
    return error == ESP_OK ? ERR_NONE : error == ESP_ERR_TIMEOUT ? ERR__SPI_TIMEOUT : ERR__SPI_COMM_ERROR;
}

static uint32_t current_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void tx_done(void *arg, const void *token, bool success)
{
    mcp251xfd_node_t *ctx = arg;
    xSemaphoreGive(ctx->tx_space);
    if (!ctx->core.count) xEventGroupSetBits(ctx->events, IDLE_BIT);
    if (ctx->callbacks.on_tx_done) {
        twai_tx_done_event_data_t event = {.done_tx_frame = token, .is_tx_success = success};
        ctx->callbacks.on_tx_done(&ctx->base, &event, ctx->user_data);
    }
}

static void rx_done(void *arg, const mcp251xfd_frame_t *frame)
{
    mcp251xfd_node_t *ctx = arg;
    if (!ctx->callbacks.on_rx_done) return;
    uint64_t us = esp_timer_get_time();
    uint32_t resolution = ctx->timestamp_resolution_hz;
    ctx->rx_cache = (twai_frame_t){
        .header = {.id = frame->id, .dlc = frame->dlc, .ide = frame->extended, .rtr = frame->rtr,
                   .timestamp = (us / 1000000) * resolution + (us % 1000000) * resolution / 1000000},
        .buffer = ctx->rx_data, .buffer_len = frame->rtr ? 0 : frame->dlc,
    };
    memcpy(ctx->rx_data, frame->data, sizeof(ctx->rx_data));
    ctx->rx_pending = ctx->in_rx_callback = true;
    twai_rx_done_event_data_t event = {};
    ctx->callbacks.on_rx_done(&ctx->base, &event, ctx->user_data);
    ctx->in_rx_callback = ctx->rx_pending = false;
}

static twai_error_state_t twai_state(mcp251xfd_state_t state)
{
    switch (state) {
    case MCP251XFD_STATE_ACTIVE: return TWAI_ERROR_ACTIVE;
    case MCP251XFD_STATE_WARNING: return TWAI_ERROR_WARNING;
    case MCP251XFD_STATE_PASSIVE: return TWAI_ERROR_PASSIVE;
    default: return TWAI_ERROR_BUS_OFF;
    }
}

static void state_changed(void *arg, mcp251xfd_state_t old, mcp251xfd_state_t state)
{
    mcp251xfd_node_t *ctx = arg;
    if (state == MCP251XFD_STATE_BUS_OFF) gpio_intr_disable(ctx->int_gpio);
    if (ctx->callbacks.on_state_change) {
        twai_state_change_event_data_t event = {.old_sta = twai_state(old), .new_sta = twai_state(state)};
        ctx->callbacks.on_state_change(&ctx->base, &event, ctx->user_data);
    }
}

static void bus_error(void *arg, uint32_t diagnostic, bool arbitration_lost)
{
    mcp251xfd_node_t *ctx = arg;
    if (ctx->callbacks.on_error) {
        twai_error_event_data_t event = {.err_flags = {
            .arb_lost = arbitration_lost,
            .bit_err = (diagnostic & (MCP251XFD_CAN_CiBDIAG1_NBIT0ERR | MCP251XFD_CAN_CiBDIAG1_NBIT1ERR)) != 0,
            .ack_err = (diagnostic & MCP251XFD_CAN_CiBDIAG1_NACKERR) != 0,
            .form_err = (diagnostic & MCP251XFD_CAN_CiBDIAG1_NFORMERR) != 0,
            .stuff_err = (diagnostic & MCP251XFD_CAN_CiBDIAG1_NSTUFERR) != 0,
        }};
        ctx->callbacks.on_error(&ctx->base, &event, ctx->user_data);
    }
}

static void worker_task(void *arg)
{
    mcp251xfd_node_t *ctx = arg;
    for (;;) {
        // Polling also covers an already-low INT line or a missed falling edge.
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
        xSemaphoreTake(ctx->lock, portMAX_DELAY);
        if (ctx->stopping) {
            SemaphoreHandle_t exited = ctx->worker_exited;
            xSemaphoreGive(ctx->lock);
            // No further ctx access after signalling the joiner.
            xSemaphoreGive(exited);
            vTaskDelete(NULL);
            return;
        }
        if (ctx->enabled) {
            eERRORRESULT error = mcp251xfd_core_service(&ctx->core);
            if (error != ERR_NONE) ESP_LOGE(TAG, "controller stopped (error %d); recreate node to recover", (int)error);
            if (ctx->core.running && gpio_get_level(ctx->int_gpio) == 0) xTaskNotifyGive(ctx->worker);
        }
        xSemaphoreGive(ctx->lock);
    }
}

static void IRAM_ATTR gpio_isr(void *arg)
{
    mcp251xfd_node_t *ctx = arg;
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(ctx->worker, &woken);
    if (woken) portYIELD_FROM_ISR();
}

static esp_err_t node_enable(twai_node_handle_t node)
{
    mcp251xfd_node_t *ctx = node_context(node);
    xSemaphoreTake(ctx->lock, portMAX_DELAY);
    esp_err_t error = ctx->enabled ? ESP_ERR_INVALID_STATE : to_esp_error(mcp251xfd_core_enable(&ctx->core));
    if (error == ESP_OK) {
        error = gpio_intr_enable(ctx->int_gpio);
        if (error == ESP_OK) {
            ctx->enabled = true;
            xTaskNotifyGive(ctx->worker);
        } else {
            mcp251xfd_core_disable(&ctx->core);
        }
    }
    xSemaphoreGive(ctx->lock);
    return error;
}

static esp_err_t node_disable(twai_node_handle_t node)
{
    mcp251xfd_node_t *ctx = node_context(node);
    xSemaphoreTake(ctx->lock, portMAX_DELAY);
    gpio_intr_disable(ctx->int_gpio);
    ctx->enabled = false;
    esp_err_t error = to_esp_error(mcp251xfd_core_disable(&ctx->core));
    xEventGroupSetBits(ctx->events, IDLE_BIT);
    xSemaphoreGive(ctx->lock);
    return error;
}

#if !CONFIG_FREERTOS_UNICORE
static void isr_barrier(void *arg)
{
    (void)arg;
}
#endif

static void destroy_context(mcp251xfd_node_t *ctx)
{
    if (ctx->isr_installed) {
        gpio_intr_disable(ctx->int_gpio);
#if !CONFIG_FREERTOS_UNICORE
        // GPIO's dispatcher does not hold its registration spinlock while
        // invoking handlers. Disable the source, then let any ISR already
        // running on either core finish before removing its function/argument.
        for (unsigned cpu = 0; cpu < portNUM_PROCESSORS; cpu++) {
            ESP_ERROR_CHECK(esp_ipc_call_blocking(cpu, isr_barrier, NULL));
        }
#endif
        gpio_isr_handler_remove(ctx->int_gpio);
    }
    if (ctx->worker) {
        xSemaphoreTake(ctx->lock, portMAX_DELAY);
        ctx->stopping = true;
        xTaskNotifyGive(ctx->worker);
        xSemaphoreGive(ctx->lock);
        // Cooperative exit: never kill a task in SPI, a callback, or a mutex.
        xSemaphoreTake(ctx->worker_exited, portMAX_DELAY);
    }
    if (ctx->spi) spi_bus_remove_device(ctx->spi);
    if (ctx->events) vEventGroupDelete(ctx->events);
    if (ctx->worker_exited) vSemaphoreDelete(ctx->worker_exited);
    if (ctx->tx_space) vSemaphoreDelete(ctx->tx_space);
    if (ctx->lock) vSemaphoreDelete(ctx->lock);
    free(ctx);
}

static esp_err_t node_delete(twai_node_handle_t node)
{
    mcp251xfd_node_t *ctx = node_context(node);
    xSemaphoreTake(ctx->lock, portMAX_DELAY);
    bool enabled = ctx->enabled;
    xSemaphoreGive(ctx->lock);
    if (enabled) return ESP_ERR_INVALID_STATE;
    destroy_context(ctx);
    return ESP_OK;
}

static esp_err_t register_callbacks(twai_node_handle_t node, const twai_event_callbacks_t *callbacks, void *arg)
{
    mcp251xfd_node_t *ctx = node_context(node);
    xSemaphoreTake(ctx->lock, portMAX_DELAY);
    esp_err_t error = ctx->enabled ? ESP_ERR_INVALID_STATE : ESP_OK;
    if (error == ESP_OK) {
        ctx->callbacks = *callbacks;
        ctx->user_data = arg;
    }
    xSemaphoreGive(ctx->lock);
    return error;
}

static esp_err_t configure_filter(twai_node_handle_t node, uint8_t index, const twai_mask_filter_config_t *config)
{
    if (config->dual_filter || config->no_classic || config->num_of_ids > 1) return ESP_ERR_NOT_SUPPORTED;
    if (config->num_of_ids && !config->id_list) return ESP_ERR_INVALID_ARG;
    mcp251xfd_node_t *ctx = node_context(node);
    xSemaphoreTake(ctx->lock, portMAX_DELAY);
    esp_err_t error = ctx->enabled ? ESP_ERR_INVALID_STATE : to_esp_error(mcp251xfd_core_filter(
        &ctx->core, index, config->num_of_ids ? config->id_list[0] : config->id, config->mask, config->is_ext));
    xSemaphoreGive(ctx->lock);
    return error;
}

static esp_err_t configure_timing(twai_node_handle_t node, const twai_timing_advanced_config_t *config,
                                  const twai_timing_advanced_config_t *data_timing)
{
    if (!config || data_timing || config->ssp_offset || config->triple_sampling || config->clk_src) return ESP_ERR_NOT_SUPPORTED;
    if (config->quanta_resolution_hz || config->brp < 1 || config->brp > 256 ||
        config->prop_seg + config->tseg_1 < 2 || config->prop_seg + config->tseg_1 > 256 ||
        !config->tseg_2 || config->tseg_2 > 128 || !config->sjw || config->sjw > config->tseg_2) return ESP_ERR_INVALID_ARG;
    mcp251xfd_node_t *ctx = node_context(node);
    MCP251XFD_BitTimeConfig timing = {.NBRP = config->brp - 1,
        .NTSEG1 = config->prop_seg + config->tseg_1 - 1, .NTSEG2 = config->tseg_2 - 1, .NSJW = config->sjw - 1};
    xSemaphoreTake(ctx->lock, portMAX_DELAY);
    esp_err_t error = (ctx->enabled || ctx->core.faulted) ? ESP_ERR_INVALID_STATE :
        to_esp_error(MCP251XFD_SetBitTimeConfiguration(&ctx->core.device, &timing, true));
    xSemaphoreGive(ctx->lock);
    return error;
}

static esp_err_t transmit(twai_node_handle_t node, const twai_frame_t *frame, int timeout)
{
    if (xPortInIsrContext()) return ESP_ERR_NOT_SUPPORTED;
    if (timeout < -1 || frame->header.dlc > 8 || frame->buffer_len > 8 ||
        (!frame->header.rtr && frame->buffer_len < frame->header.dlc) ||
        (frame->buffer_len && !frame->buffer)) return ESP_ERR_INVALID_ARG;
    if (frame->header.fdf || frame->header.brs || frame->header.esi || frame->header.trigger_time) return ESP_ERR_NOT_SUPPORTED;
    mcp251xfd_node_t *ctx = node_context(node);
    // Fail fast while disabled/listening, even when the caller requested a wait.
    xSemaphoreTake(ctx->lock, portMAX_DELAY);
    bool running = ctx->core.running;
    bool listening = ctx->core.config.listen_only;
    xSemaphoreGive(ctx->lock);
    if (!running) return ESP_ERR_INVALID_STATE;
    if (listening) return ESP_ERR_NOT_SUPPORTED;
    TickType_t ticks = timeout == -1 ? portMAX_DELAY : pdMS_TO_TICKS(timeout);
    if (xSemaphoreTake(ctx->tx_space, ticks) != pdTRUE) return ESP_ERR_TIMEOUT;
    mcp251xfd_frame_t message = {.id = frame->header.id, .dlc = frame->header.dlc,
                               .extended = frame->header.ide, .rtr = frame->header.rtr};
    if (!message.rtr && message.dlc) memcpy(message.data, frame->buffer, message.dlc);
    xSemaphoreTake(ctx->lock, portMAX_DELAY);
    esp_err_t error = to_esp_error(mcp251xfd_core_enqueue(&ctx->core, &message, frame));
    if (error == ESP_OK) {
        xEventGroupClearBits(ctx->events, IDLE_BIT);
        xTaskNotifyGive(ctx->worker);
    } else {
        xSemaphoreGive(ctx->tx_space);
    }
    xSemaphoreGive(ctx->lock);
    return error;
}

static esp_err_t wait_tx_done(twai_node_handle_t node, int timeout)
{
    if (timeout < -1) return ESP_ERR_INVALID_ARG;
    mcp251xfd_node_t *ctx = node_context(node);
    TickType_t ticks = timeout == -1 ? portMAX_DELAY : pdMS_TO_TICKS(timeout);
    EventBits_t bits = xEventGroupWaitBits(ctx->events, IDLE_BIT, pdFALSE, pdTRUE, ticks);
    xSemaphoreTake(ctx->lock, portMAX_DELAY);
    esp_err_t error = !ctx->core.running ? ESP_ERR_INVALID_STATE : (bits & IDLE_BIT) ? ESP_OK : ESP_ERR_TIMEOUT;
    xSemaphoreGive(ctx->lock);
    return error;
}

static esp_err_t receive_frame(twai_node_handle_t node, twai_frame_t *frame)
{
    mcp251xfd_node_t *ctx = node_context(node);
    // RX cache belongs to the worker while it calls on_rx_done. No lock here:
    // that callback is already under the core operation lock.
    if (xPortInIsrContext() || xTaskGetCurrentTaskHandle() != ctx->worker ||
        !ctx->in_rx_callback || !ctx->rx_pending) return ESP_ERR_INVALID_STATE;
    size_t size = frame->buffer_len < ctx->rx_cache.buffer_len ? frame->buffer_len : ctx->rx_cache.buffer_len;
    if (size && !frame->buffer) return ESP_ERR_INVALID_ARG;
    if (size) memcpy(frame->buffer, ctx->rx_data, size);
    frame->header = ctx->rx_cache.header;
    frame->buffer_len = size;
    ctx->rx_pending = false;
    return ESP_OK;
}

static esp_err_t get_info(twai_node_handle_t node, twai_node_status_t *status, twai_node_record_t *record)
{
    mcp251xfd_node_t *ctx = node_context(node);
    xSemaphoreTake(ctx->lock, portMAX_DELAY);
    if (status) *status = (twai_node_status_t){.state = twai_state(ctx->core.state),
        .tx_error_count = ctx->core.tx_errors, .rx_error_count = ctx->core.rx_errors,
        .tx_queue_remaining = ctx->core.config.tx_queue_depth - ctx->core.count};
    if (record) *record = (twai_node_record_t){.bus_err_num = ctx->core.bus_errors};
    xSemaphoreGive(ctx->lock);
    return ESP_OK;
}

esp_err_t twai_new_node_mcp251xfd(spi_host_device_t bus, const twai_mcp251xfd_node_config_t *config,
                                twai_node_handle_t *node_ret)
{
    ESP_RETURN_ON_FALSE(config && node_ret, ESP_ERR_INVALID_ARG, TAG, "null configuration/handle");
    *node_ret = NULL;
    ESP_RETURN_ON_FALSE(GPIO_IS_VALID_GPIO(config->io_cfg.int_gpio) && GPIO_IS_VALID_OUTPUT_GPIO(config->io_cfg.cs_gpio) &&
                        config->io_cfg.int_gpio != config->io_cfg.cs_gpio, ESP_ERR_INVALID_ARG, TAG, "invalid GPIOs");
    ESP_RETURN_ON_FALSE(config->tx_queue_depth && config->tx_queue_depth <= MCP251XFD_TX_CAPACITY,
                        ESP_ERR_INVALID_ARG, TAG, "TX capacity must be 1..32");
    ESP_RETURN_ON_FALSE(config->fail_retry_cnt == -1 || config->fail_retry_cnt == 0,
                        ESP_ERR_NOT_SUPPORTED, TAG, "retry count must be -1 or 0");
    ESP_RETURN_ON_FALSE(!config->bit_timing.ssp_permill, ESP_ERR_NOT_SUPPORTED, TAG, "no FD timing");
    mcp251xfd_node_t *ctx = heap_caps_calloc(1, sizeof(*ctx), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    ESP_RETURN_ON_FALSE(ctx, ESP_ERR_NO_MEM, TAG, "no memory");
    esp_err_t error = ESP_ERR_NO_MEM;
    ctx->spi_host = bus;
    ctx->cs_gpio = config->io_cfg.cs_gpio;
    ctx->int_gpio = config->io_cfg.int_gpio;
    ctx->timestamp_resolution_hz = config->timestamp_resolution_hz;
    ctx->lock = xSemaphoreCreateMutex();
    ctx->tx_space = xSemaphoreCreateCounting(config->tx_queue_depth, config->tx_queue_depth);
    ctx->worker_exited = xSemaphoreCreateBinary();
    ctx->events = xEventGroupCreate();
    if (!ctx->lock || !ctx->tx_space || !ctx->worker_exited || !ctx->events) goto fail;
    xEventGroupSetBits(ctx->events, IDLE_BIT);
    ctx->core.device = (MCP251XFD){.InterfaceDevice = ctx, .SPIClockSpeed = config->spi_clock_hz,
        .fnSPI_Init = spi_init, .fnSPI_Transfer = spi_transfer, .fnGetCurrentms = current_ms};
    ctx->core.config = (mcp251xfd_config_t){.oscillator_hz = config->oscillator_hz,
        .bitrate = config->bit_timing.bitrate, .sample_point_permill = config->bit_timing.sp_permill,
        .tx_queue_depth = config->tx_queue_depth, .listen_only = config->flags.enable_listen_only,
        .loopback = config->flags.enable_loopback, .one_shot = config->fail_retry_cnt == 0};
    ctx->core.callbacks = (mcp251xfd_callbacks_t){.tx_done = tx_done, .rx_done = rx_done,
        .state_changed = state_changed, .error = bus_error, .arg = ctx};
    eERRORRESULT init_error = mcp251xfd_core_init(&ctx->core);
    error = to_esp_error(init_error);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "MCP2518FD initialization failed: %d", (int)init_error);
        goto fail;
    }
    gpio_config_t gpio = {.pin_bit_mask = BIT64(ctx->int_gpio), .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE, .intr_type = GPIO_INTR_DISABLE};
    error = gpio_config(&gpio);
    if (error != ESP_OK) goto fail;
    // Same CAN core/priority as MCP2515: above lwIP, below Wi-Fi.
#if CONFIG_FREERTOS_UNICORE
    const BaseType_t worker_core = 0;
#else
    const BaseType_t worker_core = 1;
#endif
    BaseType_t created = xTaskCreatePinnedToCore(worker_task, "mcp251xfd", 4096, ctx, 20, &ctx->worker, worker_core);
    if (created != pdPASS) { error = ESP_ERR_NO_MEM; goto fail; }
    error = gpio_isr_handler_add(ctx->int_gpio, gpio_isr, ctx);
    if (error != ESP_OK) goto fail;
    ctx->isr_installed = true;
    error = gpio_intr_disable(ctx->int_gpio);
    if (error != ESP_OK) goto fail;
    error = gpio_set_intr_type(ctx->int_gpio, GPIO_INTR_NEGEDGE);
    if (error != ESP_OK) goto fail;
    ctx->base = (struct twai_node_base){.enable = node_enable, .disable = node_disable, .del = node_delete,
        .config_mask_filter = configure_filter, .reconfig_timing = configure_timing,
        .transmit = transmit, .transmit_wait_done = wait_tx_done, .receive_isr = receive_frame,
        .register_cbs = register_callbacks, .get_info = get_info};
    *node_ret = &ctx->base;
    return ESP_OK;
fail:
    destroy_context(ctx);
    return error;
}
