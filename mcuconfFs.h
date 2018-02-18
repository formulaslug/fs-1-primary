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
#define ARBITRARY_PORT_1 GPIOA
#define ARBITRARY_PIN_1 GPIOA_PIN1

#define RIGHT_THROTTLE_PORT GPIOA
#define RIGHT_THROTTLE_PIN GPIOA_PIN2

#define LEFT_THROTTLE_PORT GPIOA
#define LEFT_THROTTLE_PIN GPIOA_PIN3

#define ARBITRARY_PORT_2 GPIOA
#define ARBITRARY_PIN_2 GPIOA_PIN8

#define ARBITRARY_PORT_3 GPIOA
#define ARBITRARY_PIN_3 GPIOA_PIN15

#define STARTUP_SOUND_PORT GPIOB
#define STARTUP_SOUND_PIN GPIOB_PIN0

#define BRAKE_LIGHT_PORT GPIOB
#define BRAKE_LIGHT_PIN GPIOB_PIN1

#define STARTUP_LED_PORT GPIOD
#define STARTUP_LED_PIN GPIOD_LED4

#define CAN_STATUS_LED_PORT GPIOD
#define CAN_STATUS_LED_PIN GPIOD_LED3

#define IMD_FAULT_INDICATOR_PORT GPIOE
#define IMD_FAULT_INDICATOR_PIN GPIOE_PIN10

#define BMS_FAULT_INDICATOR_PORT GPIOE
#define BMS_FAULT_INDICATOR_PIN GPIOE_PIN12

#define TEMP_FAULT_INDICATOR_PORT GPIOE
#define TEMP_FAULT_INDICATOR_PIN GPIOE_PIN14

// START CAN pins
#define CAN1_TX_PORT GPIOD
#define CAN1_TX_PIN GPIOD_PIN1

#define CAN1_RX_PORT GPIOD
#define CAN1_RX_PIN GPIOD_PIN0
// END CAN pins
#endif  /* STM32F3 */

#endif  /* MCUCONF_FS_H */
