#pragma once

#include <stdint.h>
#include "driver/spi_master.h"
#include "hal/gpio_types.h"

/* ── CAN bit timing (shared by both channels) ─────────────── */
#define HWCFG_CAN_BITRATE                    500000U
#define HWCFG_CAN_SAMPLE_POINT_PERMILL       875U
#define HWCFG_CAN_SELF_TEST_MODE             0

/* ── Internal CAN: ESP32-S3 TWAI + SN65HVD233 (U2) ────────── */
#define HWCFG_INTERNAL_CAN_RX_GPIO           GPIO_NUM_1     /* RXCAN */
#define HWCFG_INTERNAL_CAN_TX_GPIO           GPIO_NUM_2     /* TXCAN */
#define HWCFG_INTERNAL_CAN_STDBY_GPIO        GPIO_NUM_11    /* STDBY_1 -> SN65HVD233 RS, 10k pull-up; drive low for high-speed mode */

/* ── External CAN: MCP2518FD (U57) + TCAN3413 (U4) on SPI2 ─ */
#define HWCFG_MCP2518FD_SPI_HOST             SPI2_HOST
#define HWCFG_MCP2518FD_INT_GPIO             GPIO_NUM_7     /* MCP2518_0_INT (INT#) */
#define HWCFG_MCP2518FD_MISO_GPIO            GPIO_NUM_15    /* SDO */
#define HWCFG_MCP2518FD_MOSI_GPIO            GPIO_NUM_16    /* SDI */
#define HWCFG_MCP2518FD_SCLK_GPIO            GPIO_NUM_17    /* SCK */
#define HWCFG_MCP2518FD_CS_GPIO              GPIO_NUM_18    /* MCP2518_0_CS */
#define HWCFG_MCP2518FD_STDBY_GPIO           GPIO_NUM_12    /* STDBY_2 -> TCAN3413 STB, 10k pull-up; drive low to enable transceiver */
#define HWCFG_MCP2518FD_SPI_CLOCK_HZ         10000000U
#define HWCFG_MCP2518FD_OSCILLATOR_HZ        40000000U      /* X2 = 40 MHz crystal, PLL off -> SYSCLK 40 MHz */
#define HWCFG_MCP2518FD_DATA_BITRATE         2000000U       /* only used when HWCFG_MCP2518FD_ENABLE_FD = 1 */
#define HWCFG_MCP2518FD_ENABLE_FD            1              /* 0: CAN 2.0 mode (ECU simulator), 1: CAN FD mode w/ BRS */
#define HWCFG_MCP2518FD_LOOPBACK             0              /* 1: MCP2518FD internal loopback (self-test without a bus peer) */

/* ── LEDs (active low) ────────────────────────────────────── */
#define HWCFG_YELLOW_LED_GPIO                GPIO_NUM_40
#define HWCFG_GREEN_LED_GPIO                 GPIO_NUM_41
#define HWCFG_BLUE_LED_GPIO                  GPIO_NUM_42

/* ── Battery sense ────────────────────────────────────────── */
#define HWCFG_VBAT_ADC_CHANNEL               ADC_CHANNEL_3   /* IO4, R1=62K / R2=6.2K divider to measure VBAT */
