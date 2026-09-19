#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <assert.h>

typedef int esp_err_t;
enum { ESP_OK, ESP_FAIL, ESP_ERR_INVALID_ARG, ESP_ERR_INVALID_STATE, ESP_ERR_TIMEOUT,
       ESP_ERR_NOT_SUPPORTED, ESP_ERR_NO_MEM, ESP_ERR_NOT_FOUND };
typedef int gpio_num_t;
typedef int spi_host_device_t;
typedef struct fake_spi *spi_device_handle_t;
typedef struct { unsigned length; const void *tx_buffer; void *rx_buffer; } spi_transaction_t;
typedef struct { int clock_speed_hz, mode, spics_io_num, queue_size; } spi_device_interface_config_t;
typedef struct { uint64_t pin_bit_mask; int mode, pull_up_en, pull_down_en, intr_type; } gpio_config_t;
#define GPIO_IS_VALID_GPIO(x) ((x) >= 0 && (x) <= 48)
#define GPIO_IS_VALID_OUTPUT_GPIO(x) GPIO_IS_VALID_GPIO(x)
enum { GPIO_MODE_INPUT, GPIO_PULLUP_ENABLE, GPIO_PULLDOWN_DISABLE,
       GPIO_INTR_DISABLE, GPIO_INTR_NEGEDGE };
typedef unsigned TickType_t;
typedef int BaseType_t;
typedef unsigned UBaseType_t;
typedef uint32_t EventBits_t;
typedef struct fake_semaphore *SemaphoreHandle_t;
typedef struct fake_events *EventGroupHandle_t;
typedef struct fake_task *TaskHandle_t;
#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define portMAX_DELAY UINT32_MAX
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#define BIT0 1u
#define BIT64(bit) (UINT64_C(1) << (bit))
#define IRAM_ATTR
#define MALLOC_CAP_INTERNAL 1
#define MALLOC_CAP_8BIT 2
#define CONFIG_FREERTOS_UNICORE 1
#define ESP_RETURN_ON_FALSE(test, error, ...) do { if (!(test)) return (error); } while (0)
static inline void fake_log(const char *tag, const char *format, ...) __attribute__((format(printf, 2, 3)));
static inline void fake_log(const char *tag, const char *format, ...) { (void)tag; (void)format; }
#define ESP_LOGE(...) fake_log(__VA_ARGS__)
#define ESP_LOGI(...) fake_log(__VA_ARGS__)
#define portYIELD_FROM_ISR() ((void)0)
static inline void *heap_caps_calloc(size_t n, size_t size, unsigned caps) { (void)caps; return calloc(n, size); }

typedef enum { TWAI_ERROR_ACTIVE, TWAI_ERROR_WARNING, TWAI_ERROR_PASSIVE, TWAI_ERROR_BUS_OFF } twai_error_state_t;
typedef struct { uint32_t bitrate; uint16_t sp_permill, ssp_permill; } twai_timing_basic_config_t;
typedef struct { int clk_src; uint32_t quanta_resolution_hz, brp;
    uint8_t prop_seg, tseg_1, tseg_2, sjw, ssp_offset; bool triple_sampling; } twai_timing_advanced_config_t;
typedef struct { uint32_t id; uint16_t dlc;
    uint32_t ide:1, rtr:1, fdf:1, brs:1, esi:1;
    union { uint64_t timestamp, trigger_time; }; } twai_frame_header_t;
typedef struct { twai_frame_header_t header; uint8_t *buffer; size_t buffer_len; } twai_frame_t;
typedef struct { union { uint32_t id; struct { uint32_t *id_list; uint32_t num_of_ids; }; };
    uint32_t mask; uint32_t is_ext:1, no_classic:1, no_fd:1, dual_filter:1; } twai_mask_filter_config_t;
typedef struct { twai_error_state_t state; uint16_t tx_error_count, rx_error_count;
    uint32_t tx_queue_remaining; } twai_node_status_t;
typedef struct { uint32_t bus_err_num; } twai_node_record_t;
typedef struct { bool is_tx_success; const twai_frame_t *done_tx_frame; } twai_tx_done_event_data_t;
typedef struct {} twai_rx_done_event_data_t;
typedef struct { twai_error_state_t old_sta, new_sta; } twai_state_change_event_data_t;
typedef struct { struct { uint32_t arb_lost:1, bit_err:1, form_err:1, stuff_err:1, ack_err:1; } err_flags; } twai_error_event_data_t;
typedef struct twai_node_base *twai_node_handle_t;
typedef struct {
    bool (*on_tx_done)(twai_node_handle_t, const twai_tx_done_event_data_t *, void *);
    bool (*on_rx_done)(twai_node_handle_t, const twai_rx_done_event_data_t *, void *);
    bool (*on_state_change)(twai_node_handle_t, const twai_state_change_event_data_t *, void *);
    bool (*on_error)(twai_node_handle_t, const twai_error_event_data_t *, void *);
} twai_event_callbacks_t;
struct twai_node_base {
    esp_err_t (*enable)(twai_node_handle_t);
    esp_err_t (*disable)(twai_node_handle_t);
    esp_err_t (*del)(twai_node_handle_t);
    esp_err_t (*config_mask_filter)(twai_node_handle_t, uint8_t, const twai_mask_filter_config_t *);
    esp_err_t (*reconfig_timing)(twai_node_handle_t, const twai_timing_advanced_config_t *, const twai_timing_advanced_config_t *);
    esp_err_t (*transmit)(twai_node_handle_t, const twai_frame_t *, int);
    esp_err_t (*transmit_wait_done)(twai_node_handle_t, int);
    esp_err_t (*receive_isr)(twai_node_handle_t, twai_frame_t *);
    esp_err_t (*register_cbs)(twai_node_handle_t, const twai_event_callbacks_t *, void *);
    esp_err_t (*get_info)(twai_node_handle_t, twai_node_status_t *, twai_node_record_t *);
};

SemaphoreHandle_t xSemaphoreCreateMutex(void);
SemaphoreHandle_t xSemaphoreCreateCounting(unsigned capacity, unsigned initial);
SemaphoreHandle_t xSemaphoreCreateBinary(void);
int xSemaphoreTake(SemaphoreHandle_t sem, TickType_t timeout);
int xSemaphoreGive(SemaphoreHandle_t sem);
void vSemaphoreDelete(SemaphoreHandle_t sem);
EventGroupHandle_t xEventGroupCreate(void);
EventBits_t xEventGroupSetBits(EventGroupHandle_t group, EventBits_t bits);
EventBits_t xEventGroupClearBits(EventGroupHandle_t group, EventBits_t bits);
EventBits_t xEventGroupWaitBits(EventGroupHandle_t group, EventBits_t bits, int clear, int all, TickType_t ticks);
void vEventGroupDelete(EventGroupHandle_t group);
BaseType_t xTaskCreatePinnedToCore(void (*entry)(void *), const char *name, unsigned stack_size,
                                  void *arg, unsigned priority, TaskHandle_t *handle, int core);
void vTaskDelete(TaskHandle_t handle);
void xTaskNotifyGive(TaskHandle_t handle);
void vTaskNotifyGiveFromISR(TaskHandle_t handle, BaseType_t *woken);
unsigned ulTaskNotifyTake(int clear, TickType_t timeout);
TaskHandle_t xTaskGetCurrentTaskHandle(void);
bool xPortInIsrContext(void);
int64_t esp_timer_get_time(void);
esp_err_t spi_bus_add_device(spi_host_device_t bus, const spi_device_interface_config_t *config, spi_device_handle_t *device);
esp_err_t spi_bus_remove_device(spi_device_handle_t device);
esp_err_t spi_device_polling_transmit(spi_device_handle_t device, spi_transaction_t *transaction);
esp_err_t gpio_config(const gpio_config_t *config);
esp_err_t gpio_intr_enable(gpio_num_t gpio);
esp_err_t gpio_intr_disable(gpio_num_t gpio);
esp_err_t gpio_isr_handler_add(gpio_num_t gpio, void (*handler)(void *), void *arg);
esp_err_t gpio_isr_handler_remove(gpio_num_t gpio);
esp_err_t gpio_set_intr_type(gpio_num_t gpio, int type);
int gpio_get_level(gpio_num_t gpio);

// Test controls; they serialize simulated hardware changes against SPI.
void platform_reset(void);
void platform_finish_tx(unsigned count);
void platform_receive(uint32_t id, bool rtr);
unsigned platform_pending_tx(void);
void platform_disconnect(void);
void platform_fail_isr_install(void);
void platform_enter_isr(bool is_isr);
void platform_check_clean(void);
void platform_pause_ms(unsigned ms);
