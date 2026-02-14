/*
 * Copyright (C) 2026 Lennart Lutz
 */

/**
 * @ingroup     boards
 * @{
 *
 * @file
 * @brief       Peripheral MCU configuration for the "Plant Hub v1.0.F411.233" board
 *
 * @author      Lennart Lutz <lennartlutz@t-online.de>
 */

#ifndef PERIPH_CONF_H
#define PERIPH_CONF_H

/* This board provides an LSE */
#ifndef CONFIG_BOARD_HAS_LSE
#define CONFIG_BOARD_HAS_LSE    1
#endif

/* This board provides an HSE */
#ifndef CONFIG_BOARD_HAS_HSE
#define CONFIG_BOARD_HAS_HSE    1
#endif

/* The HSE provides a 8MHz clock */
#ifndef CONFIG_CLOCK_HSE
#define CONFIG_CLOCK_HSE               MHZ(8)
#endif

#include "periph_cpu.h"
#include "clk_conf.h"
#include "cfg_i2c1_pb8_pb9.h"
#include "cfg_timer_tim2.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    UART configuration
 * @{
 */
static const uart_conf_t uart_config[] = {
    {
        .dev        = USART1,
        .rcc_mask   = RCC_APB2ENR_USART1EN,
        .rx_pin     = GPIO_PIN(PORT_A, 10),
        .tx_pin     = GPIO_PIN(PORT_A, 9),
        .rx_af      = GPIO_AF7,
        .tx_af      = GPIO_AF7,
        .bus        = APB2,
        .irqn       = USART1_IRQn,
    },
};

#define UART_0_ISR          (isr_usart1)
#define UART_NUMOF          ARRAY_SIZE(uart_config)
/** @} */

/** @name    PWM configuration
 * @{
 */
static const pwm_conf_t pwm_config[] = {
    {
        .dev      = TIM2,
        .rcc_mask = RCC_APB1ENR_TIM2EN,
        .chan     = { { .pin = GPIO_UNDEF,                    .cc_chan = 0 } },
        .af       = GPIO_AF1,
        .bus      = APB1
    }
};

#define PWM_NUMOF           ARRAY_SIZE(pwm_config)
/** @} */

/**
 * @name    SPI configuration
 * @{
 */
static const spi_conf_t spi_config[] = {
    {
        .dev      = SPI1,
        .mosi_pin = GPIO_PIN(PORT_A, 7),
        .miso_pin = GPIO_PIN(PORT_A, 6),
        .sclk_pin = GPIO_PIN(PORT_A, 5),
        .cs_pin   = SPI_CS_UNDEF, // 4
        .mosi_af  = GPIO_AF5,
        .miso_af  = GPIO_AF5,
        .sclk_af  = GPIO_AF5,
        .cs_af    = GPIO_AF5,
        .rccmask  = RCC_APB2ENR_SPI1EN,
        .apbbus   = APB2
    }
};

#define SPI_NUMOF           ARRAY_SIZE(spi_config)
/** @} */

/**
 * @name   ADC configuration
 * @{
 */
static const adc_conf_t adc_config[] = {
    {GPIO_PIN(PORT_B, 1), 0, 9},
    {GPIO_PIN(PORT_B, 0), 0, 8},
    {GPIO_PIN(PORT_C, 5), 0, 15},
    {GPIO_PIN(PORT_C, 4), 0, 14},
    {GPIO_PIN(PORT_A, 3), 0, 3},
    {GPIO_PIN(PORT_A, 2), 0, 2},
    {GPIO_PIN(PORT_A, 1), 0, 1},
    {GPIO_PIN(PORT_A, 0), 0, 0},
    {GPIO_PIN(PORT_C, 3), 0, 13},
    {GPIO_PIN(PORT_C, 2), 0, 12},
    {GPIO_PIN(PORT_C, 1), 0, 11},
    {GPIO_PIN(PORT_C, 0), 0, 10},
};

#define ADC_NUMOF           ARRAY_SIZE(adc_config)
/** @} */

/**
 * @name   Radio configuration
 *
 * The Plant Hub has an at86rf233 onboard. 
 * 
 * @{
 */
#define AT86RF2XX_PARAM_SPI        SPI_DEV(0)
#define AT86RF2XX_PARAM_CS         GPIO_PIN(PORT_A, 4)
#define AT86RF2XX_PARAM_INT        GPIO_PIN(PORT_B, 5)
#define AT86RF2XX_PARAM_SLEEP      GPIO_PIN(PORT_B, 6)
#define AT86RF2XX_PARAM_RESET      GPIO_PIN(PORT_B, 7)
/** @} */

/**
 * @name   EEPROM configuration
 *
 * The Plant Hub has an AT24C02 256 Byte EEPROM onboard. 
 * 
 * @{
 */
#define AT24CXXX_PARAM_I2C              I2C_DEV(0)
#define AT24CXXX_PARAM_ADDR             0x50
#define AT24CXXX_PARAM_PIN_WP           GPIO_PIN(PORT_B, 10)
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* PERIPH_CONF_H */
/** @} */
