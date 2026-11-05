#include "hw_config.h"
#include "esp_mac.h"
#include "esp_log.h"
#include <stdio.h>

static const char *TAG = "hw_config";

// The compile-time pin map is the first thing to check when a new board does
// not come up; log every macro the drivers will use.
void hw_config_log(void)
{
	ESP_LOGI(TAG, "variant=%s (HARDWARE_VER=%d), CAN_BUS_COUNT=%d, "
			 "HW_HAS_MCP2515=%d, HW_HAS_MCP2518FD=%d, HW_HAS_CAR_ON_SENSE=%d",
			 HARDWARE_VERSION, HARDWARE_VER, CAN_BUS_COUNT, HW_HAS_MCP2515,
			 HW_HAS_MCP2518FD, HW_HAS_CAR_ON_SENSE);

#if HARDWARE_VER == WICAN_PROTO
	ESP_LOGI(TAG, "PROTO: bus0 TWAI TX=GPIO%d RX=GPIO%d (no STDBY)", TX_GPIO_NUM, RX_GPIO_NUM);
	ESP_LOGI(TAG, "PROTO: bus1 MCP2515 SPI%d SCLK=%d MOSI=%d MISO=%d CS=%d INT=%d RST=%d "
			 "clock=%lu osc=%lu",
			 MCP2515_SPI_HOST, MCP2515_SCLK_GPIO_NUM, MCP2515_MOSI_GPIO_NUM,
			 MCP2515_MISO_GPIO_NUM, MCP2515_CS_GPIO_NUM, MCP2515_INT_GPIO_NUM,
			 MCP2515_RST_GPIO_NUM, (unsigned long)MCP2515_SPI_CLOCK_HZ,
			 (unsigned long)MCP2515_OSCILLATOR_HZ);
#elif HARDWARE_VER == WICAN_EB_FD
	ESP_LOGI(TAG, "EB-FD: bus0 TWAI TX=GPIO%d RX=GPIO%d STDBY=GPIO%d (U2 RS)",
			 TX_GPIO_NUM, RX_GPIO_NUM, CAN_STDBY_GPIO_NUM);
	ESP_LOGI(TAG, "EB-FD: bus1 MCP2518FD SPI%d SCLK=%d MOSI=%d MISO=%d CS=%d INT=%d "
			 "STDBY=GPIO%d (TCAN3413 U4) clock=%lu osc=%lu",
			 MCP2518FD_SPI_HOST, MCP2518FD_SCLK_GPIO_NUM, MCP2518FD_MOSI_GPIO_NUM,
			 MCP2518FD_MISO_GPIO_NUM, MCP2518FD_CS_GPIO_NUM, MCP2518FD_INT_GPIO_NUM,
			 MCP2518FD_STDBY_GPIO_NUM, (unsigned long)MCP2518FD_SPI_CLOCK_HZ,
			 (unsigned long)MCP2518FD_OSCILLATOR_HZ);
#elif HARDWARE_VER == WICAN_V300
	ESP_LOGI(TAG, "V300: bus0 TWAI TX=GPIO%d RX=GPIO%d STDBY=GPIO%d",
			 TX_GPIO_NUM, RX_GPIO_NUM, CAN_STDBY_GPIO_NUM);
#endif
	ESP_LOGI(TAG, "LEDs: connected=GPIO%d active=GPIO%d pwr=GPIO%d (on=%d) | "
			 "VBAT=ADC_CH%d atten=%d scale=%lu/%lu offset=%.2f V",
			 CONNECTED_LED_GPIO_NUM, ACTIVE_LED_GPIO_NUM, PWR_LED_GPIO_NUM, LED_ON,
			 VBAT_ADC_CHANNEL, VBAT_ADC_ATTEN, (unsigned long)VBAT_SCALE_NUM,
			 (unsigned long)VBAT_SCALE_DEN, (double)VBAT_READ_OFFSET_V);
#if HW_HAS_CAR_ON_SENSE
	ESP_LOGI(TAG, "car-on sense: ADC_CH%d (divider/offset assumed equal to VBAT; "
			 "not separately calibrated)", V_CAR_ON_ADC_CHANNEL);
#else
	ESP_LOGI(TAG, "car-on sense: not present on this variant");
#endif
}

esp_err_t hw_config_get_device_id(char *uid)
{
    uint8_t derived_mac_addr[6];
    
    esp_read_mac(derived_mac_addr, ESP_MAC_WIFI_SOFTAP);

    sprintf(uid, "%02x%02x%02x%02x%02x%02x",
            derived_mac_addr[0], derived_mac_addr[1], derived_mac_addr[2],
            derived_mac_addr[3], derived_mac_addr[4], derived_mac_addr[5]);

    return ESP_OK;
}
