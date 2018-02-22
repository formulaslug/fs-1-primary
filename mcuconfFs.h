// Copyright (c) 2018 Formula Slug. All Rights Reserved.

#ifndef MCUCONF_FS_H
#define MCUCONF_FS_H

// Current MCU
#define STM32F4

/*********************************************************************
 * @brief Bit Timing Register - Baud Rate Prescalar (BTR_BRP) vals
 * @note Only 250k is confirmed working, others were originally
 *       recovered from docs for STM32F103xx reference manual
 ********************************************************************/
#ifdef STM32F3
#define CAN_BTR_BRP_125k 239
#define CAN_BTR_BRP_250k 7
#define CAN_BTR_BRP_500k 5
#define CAN_BTR_BRP_1M 2
#define CAN_BTR_BRP_1M5 1
#define CAN_BTR_BRP_3M 0
#endif  /* STM32F3 */

#ifdef STM32F4
#define CAN_BTR_BRP_125k 239
#define CAN_BTR_BRP_250k 13
#define CAN_BTR_BRP_500k 6
#define CAN_BTR_BRP_1M 2
#define CAN_BTR_BRP_1M5 1
#define CAN_BTR_BRP_3M 0
#endif  /* STM32F4 */

/*********************************************************************
 * @brief Pin mappings
 ********************************************************************/

#ifdef STM32F3
#define ARBITRARY_PORT_1 GPIOA
#define ARBITRARY_PIN_1 GPIOA_ARD_A1

#define RIGHT_THROTTLE_PORT GPIOA
#define RIGHT_THROTTLE_PIN GPIOA_ARD_A3

#define LEFT_THROTTLE_PORT GPIOA
#define LEFT_THROTTLE_PIN GPIOA_ARD_A0

#define ARBITRARY_PORT_2 GPIOA
#define ARBITRARY_PIN_2 GPIOA_ARD_D2

#define ARBITRARY_PORT_3 GPIOB
#define ARBITRARY_PIN_3 GPIOB_ARD_D3

#define STARTUP_SOUND_PORT GPIOB
#define STARTUP_SOUND_PIN GPIOB_ARD_D4

#define BRAKE_LIGHT_PORT GPIOB
#define BRAKE_LIGHT_PIN GPIOB_ARD_D5
#endif  /* STM32F3 */

#ifdef STM32F4
// Analog inputs
#define BRAKE_VALUE_PORT GPIOA
#define BRAKE_VALUE_PIN GPIOA_PIN1

#define RIGHT_THROTTLE_PORT GPIOA
#define RIGHT_THROTTLE_PIN GPIOA_PIN2

#define LEFT_THROTTLE_PORT GPIOA
#define LEFT_THROTTLE_PIN GPIOA_PIN3

// Digital inputs
#define TRI_STATE_SWITCH_UP_PORT GPIOB
#define TRI_STATE_SWITCH_UP_PIN GPIOB_PIN11

#define TRI_STATE_SWITCH_DOWN_PORT GPIOB
#define TRI_STATE_SWITCH_DOWN_PIN GPIOB_PIN12

#define DRIVE_MODE_BUTTON_PORT GPIOB
#define DRIVE_MODE_BUTTON_PIN GPIOB_PIN13

#define BSPD_FAULT_PORT GPIOB
#define BSPD_FAULT_PIN GPIOB_PIN14

// Digital outputs
#define IMD_FAULT_INDICATOR_PORT GPIOE
#define IMD_FAULT_INDICATOR_PIN GPIOE_PIN10

#define AMS_FAULT_INDICATOR_PORT GPIOE
#define AMS_FAULT_INDICATOR_PIN GPIOE_PIN12

#define BSPD_FAULT_INDICATOR_PORT GPIOE
#define BSPD_FAULT_INDICATOR_PIN GPIOE_PIN14

#define STARTUP_SOUND_PORT GPIOE
#define STARTUP_SOUND_PIN GPIOE_PIN8

#define BRAKE_LIGHT_PORT GPIOB
#define BRAKE_LIGHT_PIN GPIOB_PIN2

#define STARTUP_LED_PORT GPIOD
#define STARTUP_LED_PIN GPIOD_LED4

#define CAN1_STATUS_LED_PORT GPIOD
#define CAN1_STATUS_LED_PIN GPIOD_LED3

#define CAN2_STATUS_LED_PORT GPIOD
#define CAN2_STATUS_LED_PIN GPIOD_LED5

// CAN IO
#define CAN1_RX_PORT GPIOD
#define CAN1_RX_PIN GPIOD_PIN0

#define CAN1_TX_PORT GPIOD
#define CAN1_TX_PIN GPIOD_PIN1

#define CAN2_TX_PORT GPIOB
#define CAN2_TX_PIN GPIOB_SCL

#define CAN2_RX_PORT GPIOB
#define CAN2_RX_PIN GPIOB_PIN5

/*
 * @brief Pin/Port aliasing
 */
#define NEUTRAL_BUTTON_PORT TRI_STATE_SWITCH_UP_PORT
#define NUETRAL_BUTTON_PIN TRI_STATE_SWITCH_UP_PIN

#define DRIVE_BUTTON_PORT TRI_STATE_SWITCH_DOWN_PORT
#define DRIVE_BUTTON_PIN TRI_STATE_SWITCH_DOWN_PIN

#endif  /* STM32F4 */

#endif  /* MCUCONF_FS_H */
