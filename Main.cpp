// Copyright (c) 2018 Formula Slug. All Rights Reserved.

#include <array>
#include <mutex>

#include "CanBus.h"
#include "CanOpenPdo.h"
#include "Vehicle.h"
#include "ch.hpp"
#include "hal.h"
#include "pinconf.h"
#include "thread.h"

/**
 * If the range is ordered as (min, max), minimum input values map to 0.
 * If the range is ordered as (max, min), maximum input values map to 1.
 *
 * @param range Range of input value
 * @param input Value within range
 * @return Normalized value between 0 and 1 inclusive
 */
double normalize(const std::array<double, 2> range, double input) {
  return (input - range[0]) / (range[1] - range[0]);
}

void heartbeatThreadFunc(CanBus& canBus, chibios_rt::Mutex& canBusMut);
void inputProcThreadFunc(CanBus& canBus, chibios_rt::Mutex& canBusMut);
void canRxThreadFunc(CanBus& canBus, chibios_rt::Mutex& canBusMut);
void canTxThreadFunc(CanBus& canBus, chibios_rt::Mutex& canBusMut);

int main() {
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  // Configure input pins
  palSetPadMode(ARBITRARY_PORT_1, ARBITRARY_PIN_1, PAL_MODE_INPUT);
  palSetPadMode(RIGHT_THROTTLE_PORT, RIGHT_THROTTLE_PIN, PAL_MODE_INPUT);
  palSetPadMode(LEFT_THROTTLE_PORT, LEFT_THROTTLE_PIN, PAL_MODE_INPUT);
  palSetPadMode(ARBITRARY_PORT_2, ARBITRARY_PIN_2, PAL_MODE_INPUT_PULLUP);
  palSetPadMode(ARBITRARY_PORT_3, ARBITRARY_PIN_3, PAL_MODE_INPUT_PULLUP);

  // Turn off startup sound
  palSetPadMode(STARTUP_SOUND_PORT, STARTUP_SOUND_PIN, PAL_MODE_OUTPUT_PUSHPULL);
  palWritePad(STARTUP_SOUND_PORT, STARTUP_SOUND_PIN, PAL_LOW);

  // Turn off brake light
  palSetPadMode(BRAKE_LIGHT_PORT, BRAKE_LIGHT_PIN, PAL_MODE_OUTPUT_PUSHPULL);
  palWritePad(BRAKE_LIGHT_PORT, BRAKE_LIGHT_PIN, PAL_LOW);

  Vehicle vehicle;

  // Activate CAN driver 1 (PA11 = CANRX, PA12 = CANTX)
  CanBus canBus(0x680, CanBusBaudRate::k250k, true);
  chibios_rt::Mutex canBusMut;

  thread heartbeatThread(NORMALPRIO + 3, heartbeatThreadFunc, canBus,
                         canBusMut);

  thread inputProcThread(NORMALPRIO, inputProcThreadFunc, canBus, canBusMut);

  // Start receiver thread
  thread canRxThread(NORMALPRIO + 7, canRxThreadFunc, canBus, canBusMut);

  // Start transmitter thread
  thread canTxThread(NORMALPRIO + 7, canTxThreadFunc, canBus, canBusMut);

  // Indicate startup - blink then stay on
  palSetPadMode(STARTUP_LED_PORT, STARTUP_LED_PIN, PAL_MODE_OUTPUT_PUSHPULL);
  for (uint8_t i = 0; i < 2; i++) {
    palWritePad(STARTUP_LED_PORT, STARTUP_LED_PIN, PAL_LOW);
    chThdSleepMilliseconds(300);
    palWritePad(STARTUP_LED_PORT, STARTUP_LED_PIN, PAL_HIGH);
    chThdSleepMilliseconds(300);
  }

  while (1) {
    {
      std::lock_guard<chibios_rt::Mutex> lock(canBusMut);

      // print all transmitted messages
      canBus.printTxAll();
      // print all received messages
      canBus.printRxAll();
    }

#if 0
    vehicle.dynamics.throttleVoltage =
        analogRead(analogInputPins[kThrottleVoltage]);

    // Vehicle's main state machine (FSM)
    switch (vehicle.state) {
      case kLVStartup:
        // Perform kLVStartup functions HERE
        vehicle.state = kLVActive;
        break;
      case kLVActive:
        // Set LED feedback
        vehicle.ledStates[kBlue] = kLEDOn;
        vehicle.ledStates[kYellow] = kLEDOff;
        vehicle.ledStates[kRed] = kLEDOff;

        // Wait to move to kHVStartup
        if (digitalReadFast(buttonPins[kHVToggle]) == LOW) {
          vehicle.state = kHVStartup;
        }
        break;
      case kHVShutdown:
        // Perform kHVShutdown functions HERE

        // Transition to kLVActive
        vehicle.state = kLVActive;
        break;
      case kHVStartup:
        // Perform kLVStartup functions HERE

        vehicle.state = kHVActive;
        break;
      case kHVActive:
        // Set LED feedback
        vehicle.ledStates[kBlue] = kLEDOn;
        vehicle.ledStates[kYellow] = kLEDOn;
        vehicle.ledStates[kRed] = kLEDOff;

        // Wait to move to kRTDStartup until user input
        if (digitalReadFast(buttonPins[kRTDToggle]) == LOW) {
          vehicle.state = kRTDStartup;
        } else if (digitalReadFast(buttonPins[kHVToggle]) == LOW) {
          // Or move back to LV active
          vehicle.state = kHVShutdown;
        }
        break;
      case kRTDShutdown:
        // Perform kHVShutdown functions HERE

        vehicle.state = kHVActive;
        break;
      case kRTDStartup:
        // Perform kLVStartup functions HERE

        // Show entire system is hot
        vehicle.ledStates[kBlue] = kLEDOn;
        vehicle.ledStates[kYellow] = kLEDOn;
        vehicle.ledStates[kRed] = kLEDOn;

        vehicle.state = kRTDActive;
        break;
      case kRTDActive:
        // update current throttle voltage
        vehicle.dynamics.throttleVoltage =
            analogRead(analogInputPins[kThrottleVoltage]);

        // Show speed
        vehicle.ledStates[kSpeed] = ~vehicle.ledStates[kSpeed];

        // Wait to transition back
        if (digitalReadFast(buttonPins[kRTDToggle]) == LOW) {
          // Start moving back to HV_ACTIVE
          vehicle.ledStates[kSpeed] = kLEDOff;
          vehicle.dynamics.throttleVoltage = 1;
          vehicle.state = kRTDShutdown;
        }
        break;
    }
#endif

    chThdSleepMilliseconds(50);
  }
}

/**
 * @desc Performs period tasks every second
 */
void heartbeatThreadFunc(CanBus& canBus, chibios_rt::Mutex& canBusMut) {
  while (1) {
    // enqueue heartbeat message to g_canTxQueue
    const HeartbeatMessage heartbeatMessage(kCobIdNode3Heartbeat);
    {
      std::lock_guard<chibios_rt::Mutex> lock(canBusMut);
      canBus.queueTxMessage(heartbeatMessage);
    }

    chThdSleepMilliseconds(1000);
  }
}

void inputProcThreadFunc(CanBus& canBus, chibios_rt::Mutex& canBusMut) {
  double leftThrottle = 0;
  double rightThrottle = 0;
  double throttle = 0;
  bool driveButton = false;

  while (1) {
    // leftThrottle = normalize({500, 750}, analogRead(A0));
    // rightThrottle = normalize({550, 295}, analogRead(A3));
    throttle = (leftThrottle + rightThrottle) / 2;
    // driveButton = digitalReadFast(23);

    // enqueue throttle voltage periodically as well
    const ThrottleMessage throttleMessage(65536 * throttle, driveButton);
    {
      std::lock_guard<chibios_rt::Mutex> lock(canBusMut);
      canBus.queueTxMessage(throttleMessage);
    }

    chThdSleepMilliseconds(100);
  }
}

void canRxThreadFunc(CanBus& canBus, chibios_rt::Mutex& canBusMut) {
  event_listener_t el;

  chRegSetThreadName("CAN RX");
  chEvtRegister(&CAND1.rxfull_event, &el, 0);

  while (true) {
    if (chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(100)) == 0) {
      continue;
    }
    {
      std::lock_guard<chibios_rt::Mutex> lock(canBusMut);
      canBus.processRxMessages();
    }
  }

  chEvtUnregister(&CAND1.rxfull_event, &el);
}

void canTxThreadFunc(CanBus& canBus, chibios_rt::Mutex& canBusMut) {
  chRegSetThreadName("CAN TX");

  while (true) {
    {
      std::lock_guard<chibios_rt::Mutex> lock(canBusMut);
      canBus.send(0x00FF00FF55AA55AA);
      canBus.processTxMessages();
    }

    chThdSleepMilliseconds(50);
  }
}
