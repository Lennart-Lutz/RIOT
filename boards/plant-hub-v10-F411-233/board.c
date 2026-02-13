/*
 * Copyright (C) 2026 Lennart Lutz
 */

/**
 * @ingroup     boards
 * @{
 *
 * @file
 * @brief       Board initialization code for the "Plant Hub v1.0.F411.233" board.
 *
 * @author      Lennart Lutz <lennartlutz@t-online.de>
 * 
 * @}
 */

#include "board.h"
#include "periph/gpio.h"

void board_init(void)
{
    // Initialize all LED pins as outputs and set them to high (off)
    gpio_init(RFST_LED_RED_PIN, GPIO_OUT);
    gpio_init(RFST_LED_GREEN_PIN, GPIO_OUT);
    gpio_init(RFST_LED_BLUE_PIN, GPIO_OUT);

    gpio_init(ST1_LED_RED_PIN, GPIO_OUT);
    gpio_init(ST1_LED_GREEN_PIN, GPIO_OUT);
    gpio_init(ST1_LED_BLUE_PIN, GPIO_OUT);

    gpio_init(ST2_LED_RED_PIN, GPIO_OUT);
    gpio_init(ST2_LED_GREEN_PIN, GPIO_OUT);
    gpio_init(ST2_LED_BLUE_PIN, GPIO_OUT);

    gpio_set(RFST_LED_RED_PIN);
    gpio_set(RFST_LED_GREEN_PIN);
    gpio_set(RFST_LED_BLUE_PIN);

    gpio_set(ST1_LED_RED_PIN);
    gpio_set(ST1_LED_GREEN_PIN);
    gpio_set(ST1_LED_BLUE_PIN);

    gpio_set(ST2_LED_RED_PIN);
    gpio_set(ST2_LED_GREEN_PIN);
    gpio_set(ST2_LED_BLUE_PIN);


    // Initialize all presence detection pins as inputs with pull-up resistors
    gpio_init(PRESENCE_PORT_0_PIN, GPIO_IN_PU);
    gpio_init(PRESENCE_PORT_1_PIN, GPIO_IN_PU);
    gpio_init(PRESENCE_PORT_2_PIN, GPIO_IN_PU);
    gpio_init(PRESENCE_PORT_3_PIN, GPIO_IN_PU);
    gpio_init(PRESENCE_PORT_4_PIN, GPIO_IN_PU);
    gpio_init(PRESENCE_PORT_5_PIN, GPIO_IN_PU);
    gpio_init(PRESENCE_PORT_6_PIN, GPIO_IN_PU);
    gpio_init(PRESENCE_PORT_7_PIN, GPIO_IN_PU);
    gpio_init(PRESENCE_PORT_8_PIN, GPIO_IN_PU);
    gpio_init(PRESENCE_PORT_9_PIN, GPIO_IN_PU);
    gpio_init(PRESENCE_PORT_10_PIN, GPIO_IN_PU);
    gpio_init(PRESENCE_PORT_11_PIN, GPIO_IN_PU);

    // Initialize all power control pins as outputs and set them to high (off)
    gpio_init(POWER_PORT_0_PIN, GPIO_OUT);
    gpio_set(POWER_PORT_0_PIN);

    gpio_init(POWER_PORT_1_PIN, GPIO_OUT);
    gpio_set(POWER_PORT_1_PIN);

    gpio_init(POWER_PORT_2_PIN, GPIO_OUT);
    gpio_set(POWER_PORT_2_PIN);

    gpio_init(POWER_PORT_3_PIN, GPIO_OUT);
    gpio_set(POWER_PORT_3_PIN);

    gpio_init(POWER_PORT_4_PIN, GPIO_OUT);
    gpio_set(POWER_PORT_4_PIN);

    gpio_init(POWER_PORT_5_PIN, GPIO_OUT);
    gpio_set(POWER_PORT_5_PIN);

    gpio_init(POWER_PORT_6_PIN, GPIO_OUT);
    gpio_set(POWER_PORT_6_PIN);

    gpio_init(POWER_PORT_7_PIN, GPIO_OUT);
    gpio_set(POWER_PORT_7_PIN);

    gpio_init(POWER_PORT_8_PIN, GPIO_OUT);
    gpio_set(POWER_PORT_8_PIN);

    gpio_init(POWER_PORT_9_PIN, GPIO_OUT);
    gpio_set(POWER_PORT_9_PIN);

    gpio_init(POWER_PORT_10_PIN, GPIO_OUT);
    gpio_set(POWER_PORT_10_PIN);

    gpio_init(POWER_PORT_11_PIN, GPIO_OUT);
    gpio_set(POWER_PORT_11_PIN);

    // Initialize calibration indicator LED pins as outputs and set them to high (off)
    gpio_init(CALIB_LED_RED_PIN, GPIO_OUT);
    gpio_set(CALIB_LED_RED_PIN);

    gpio_init(CALIB_LED_GREEN_PIN, GPIO_OUT);
    gpio_set(CALIB_LED_GREEN_PIN);

    gpio_init(CALIB_LED_BLUE_PIN, GPIO_OUT);
    gpio_set(CALIB_LED_BLUE_PIN);

    // Note: calibration buttons are initialized as inputs with pull-up resistors in the main application code

    return;
}

