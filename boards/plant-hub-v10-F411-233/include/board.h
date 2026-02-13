/*
 * Copyright (C) 2026 Lennart Lutz
 */

/**
 * @ingroup     boards
 *
 * @brief       Support for the "Plant Hub v1.0.F411.233" board.
 * @{
 *
 * @file
 * @brief       Pin definitions and board configuration options
 *
 * @author      Lennart Lutz <lennartlutz@t-online.de>
 */

#ifndef BOARD_H
#define BOARD_H

#include "cpu.h"
#include "periph_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

#define XTIMER_BACKOFF              (8)

/**
 * @name    Pin definitions and handlers
 * @{
 */

// Soil Moisture Sensor analog input pins
#define ANALOG_PORT_0               ADC_LINE(0)
#define ANALOG_PORT_1               ADC_LINE(1)
#define ANALOG_PORT_2               ADC_LINE(2)
#define ANALOG_PORT_3               ADC_LINE(3)
#define ANALOG_PORT_4               ADC_LINE(4)
#define ANALOG_PORT_5               ADC_LINE(5)
#define ANALOG_PORT_6               ADC_LINE(6)
#define ANALOG_PORT_7               ADC_LINE(7)
#define ANALOG_PORT_8               ADC_LINE(8)
#define ANALOG_PORT_9               ADC_LINE(9)
#define ANALOG_PORT_10              ADC_LINE(10)
#define ANALOG_PORT_11              ADC_LINE(11)

// Soil Moisture Sensor presence detection pins
#define PRESENCE_PORT_0_PIN         GPIO_PIN(PORT_D, 0)
#define PRESENCE_PORT_1_PIN         GPIO_PIN(PORT_C, 11)
#define PRESENCE_PORT_2_PIN         GPIO_PIN(PORT_E, 11)
#define PRESENCE_PORT_3_PIN         GPIO_PIN(PORT_E, 10)
#define PRESENCE_PORT_4_PIN         GPIO_PIN(PORT_E, 9)
#define PRESENCE_PORT_5_PIN         GPIO_PIN(PORT_E, 8)
#define PRESENCE_PORT_6_PIN         GPIO_PIN(PORT_E, 7)
#define PRESENCE_PORT_7_PIN         GPIO_PIN(PORT_E, 6)
#define PRESENCE_PORT_8_PIN         GPIO_PIN(PORT_E, 5)
#define PRESENCE_PORT_9_PIN         GPIO_PIN(PORT_E, 4)
#define PRESENCE_PORT_10_PIN        GPIO_PIN(PORT_E, 3)
#define PRESENCE_PORT_11_PIN        GPIO_PIN(PORT_E, 2)

// Soil Moisture Sensor power control pins
#define POWER_PORT_0_PIN            GPIO_PIN(PORT_C, 12)
#define POWER_PORT_1_PIN            GPIO_PIN(PORT_C, 10)
#define POWER_PORT_2_PIN            GPIO_PIN(PORT_A, 15)
#define POWER_PORT_3_PIN            GPIO_PIN(PORT_A, 12)
#define POWER_PORT_4_PIN            GPIO_PIN(PORT_A, 11)
#define POWER_PORT_5_PIN            GPIO_PIN(PORT_D, 14)
#define POWER_PORT_6_PIN            GPIO_PIN(PORT_D, 15)
#define POWER_PORT_7_PIN            GPIO_PIN(PORT_C, 6)
#define POWER_PORT_8_PIN            GPIO_PIN(PORT_C, 7)
#define POWER_PORT_9_PIN            GPIO_PIN(PORT_C, 8)
#define POWER_PORT_10_PIN           GPIO_PIN(PORT_C, 9)
#define POWER_PORT_11_PIN           GPIO_PIN(PORT_A, 8)

// RF Status RGB LED pins
#define RFST_LED_RED_PIN              GPIO_PIN(PORT_D, 7)
#define RFST_LED_GREEN_PIN            GPIO_PIN(PORT_E, 0)
#define RFST_LED_BLUE_PIN             GPIO_PIN(PORT_E, 1)

// Status 1 RGB LED pins
#define ST1_LED_RED_PIN               GPIO_PIN(PORT_D, 4)
#define ST1_LED_GREEN_PIN             GPIO_PIN(PORT_D, 5)
#define ST1_LED_BLUE_PIN              GPIO_PIN(PORT_D, 6)

// Status 2 RGB LED pins
#define ST2_LED_RED_PIN               GPIO_PIN(PORT_D, 1)
#define ST2_LED_GREEN_PIN             GPIO_PIN(PORT_D, 2)
#define ST2_LED_BLUE_PIN              GPIO_PIN(PORT_D, 3)

// Calibration buttons pins
#define CALIB_BUTTON_A_PIN            GPIO_PIN(PORT_B, 12)
#define CALIB_BUTTON_B_PIN            GPIO_PIN(PORT_B, 13)

// Calibration indicator LED RGB pins
#define CALIB_LED_RED_PIN              GPIO_PIN(PORT_E, 13)
#define CALIB_LED_GREEN_PIN            GPIO_PIN(PORT_E, 14)
#define CALIB_LED_BLUE_PIN             GPIO_PIN(PORT_E, 15)
/** @} */

/**
 * @brief   Initialize board specific hardware, including clock, LEDs and std-IO
 */
void board_init(void);

/**
 * @name    Define specific board identifiers
 * @{
 */
#define board_plant_hub // Common identifier
#define board_plant_hub_v10_f411_233 // Unique identifier
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */
/** @} */
