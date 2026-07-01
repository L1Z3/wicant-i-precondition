#pragma once

#include <stdint.h>
#include "driver/spi_master.h"
#include "hal/gpio_types.h"

#define HWCFG_CAN_BITRATE                    500000U
#define HWCFG_CAN_SAMPLE_POINT_PERMILL       875U
#define HWCFG_CAN_SELF_TEST_MODE             0

#define HWCFG_INTERNAL_CAN_RX_GPIO           GPIO_NUM_1
#define HWCFG_INTERNAL_CAN_TX_GPIO           GPIO_NUM_2

#define HWCFG_MCP2515_SPI_HOST               SPI2_HOST
#define HWCFG_MCP2515_INT_GPIO               GPIO_NUM_7
#define HWCFG_MCP2515_MISO_GPIO              GPIO_NUM_15
#define HWCFG_MCP2515_MOSI_GPIO              GPIO_NUM_16
#define HWCFG_MCP2515_SCLK_GPIO              GPIO_NUM_17
#define HWCFG_MCP2515_CS_GPIO                GPIO_NUM_18
#define HWCFG_MCP2515_RST_GPIO               GPIO_NUM_8
#define HWCFG_MCP2515_SPI_CLOCK_HZ           5000000U
#define HWCFG_MCP2515_OSCILLATOR_HZ          8000000U
#define HWCFG_YELLOW_LED_GPIO                GPIO_NUM_40    //active low
#define HWCFG_GREEN_LED_GPIO                 GPIO_NUM_41    //active low
#define HWCFG_BLUE_LED_GPIO                  GPIO_NUM_42    //active low

#define HWCFG_VBAT_ADC_CHANNEL              ADC_CHANNEL_3   //R1=62K, R2=6.2K voltage divider to measure VBAT on ADC3