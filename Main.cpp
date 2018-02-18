// Copyright (c) 2018 Formula Slug. All Rights Reserved.

#include <array>
#include <mutex>

#include "CanBus.h"
#include "CanOpenPdo.h"
#include "Vehicle.h"
#include "ch.hpp"
#include "hal.h"
#include "mcuconfFs.h"

// TODO: Fix this garbage
#define CAN_BUS (*(CanBus*)((*(std::vector<void*>*)arg)[0]))
#define CAN_BUS_MUT *(chibios_rt::Mutex*)((*(std::vector<void*>*)arg)[1])

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

/**
 * @desc Performs period tasks every second
 */
static THD_WORKING_AREA(heartbeatThreadFuncWa, 128);
static THD_FUNCTION(heartbeatThreadFunc, arg) {
  chRegSetThreadName("NODE HEARTBEAT");

  while (1) {
    // enqueue heartbeat message to g_canTxQueue
    // TODO: Remove need for node ID param to heartbeat obj (passed
    //       during instantiation of CAN bus)
    const HeartbeatMessage heartbeatMessage(0x1);
    {
      std::lock_guard<chibios_rt::Mutex> lock(CAN_BUS_MUT);
      (CAN_BUS).queueTxMessage(heartbeatMessage);
    }
    // transmit node's (self) heartbeat every 1s
    chThdSleepMilliseconds(1000);
  }
}

/**
 * @desc Performs period tasks every second
 */
static THD_WORKING_AREA(throttleThreadFuncWa, 128);
static THD_FUNCTION(throttleThreadFunc, arg) {
  chRegSetThreadName("THROTTLE");

  while (1) {
    // enqueue heartbeat message to g_canTxQueue
    // TODO: Remove need for node ID param to heartbeat obj (passed
    //       during instantiation of CAN bus)
    // ThrottleMessage::ThrottleMessage(uint16_t throttleVoltage, bool forwardSwitch) {
    const ThrottleMessage throttleMessage(1, true);
    {
      std::lock_guard<chibios_rt::Mutex> lock(CAN_BUS_MUT);
      (CAN_BUS).queueTxMessage(throttleMessage);
    }
    // transmit node's (self) heartbeat every 1s
    chThdSleepMilliseconds(200);
  }
}


// static THD_WORKING_AREA(inputProcThreadFuncWa, 128);
// static THD_FUNCTION(inputProcThreadFunc, arg) {
//   double leftThrottle = 0;
//   double rightThrottle = 0;
//   double throttle = 0;
//   bool driveButton = false;
//
//   while (true) {
//     // leftThrottle = normalize({500, 750}, analogRead(A0));
//     // rightThrottle = normalize({550, 295}, analogRead(A3));
//     throttle = (leftThrottle + rightThrottle) / 2;
//     // driveButton = digitalReadFast(23);
//
//     // enqueue throttle voltage periodically as well
//     const ThrottleMessage throttleMessage(65536 * throttle, driveButton);
//     {
//       std::lock_guard<chibios_rt::Mutex> lock(CAN_BUS_MUT);
//       (CAN_BUS).queueTxMessage(throttleMessage);
//     }
//
//     chThdSleepMilliseconds(100);
//   }
// }


/*
 * CAN TX thread
 */
static THD_WORKING_AREA(canTxThreadFuncWa, 128);
static THD_FUNCTION(canTxThreadFunc, arg) {
  chRegSetThreadName("CAN TX");

  while (true) {
    {
      // Lock from simultaneous thread access
      std::lock_guard<chibios_rt::Mutex> lock(CAN_BUS_MUT);
      // Process all messages to transmit from the message transmission queue
      (CAN_BUS).processTxMessages();
    }
    // throttle back thread runloop to prevent overconsumption of resources
    chThdSleepMilliseconds(10);
  }
}

/*
 * CAN RX thread
 */
static THD_WORKING_AREA(canRxThreadFuncWa, 128);
static THD_FUNCTION(canRxThreadFunc, arg) {
  event_listener_t el;

  chRegSetThreadName("CAN RX");
  chEvtRegister(&CAND1.rxfull_event, &el, 0);

  while (true) {
    if (chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(100)) == 0) {
      continue;
    }
    {
      std::lock_guard<chibios_rt::Mutex> lock(CAN_BUS_MUT);
      (CAN_BUS).processRxMessages();
    }
  }

  chEvtUnregister(&CAND1.rxfull_event, &el);
}

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
  // Fault indicator lights
  palSetPadMode(IMD_FAULT_INDICATOR_PORT, IMD_FAULT_INDICATOR_PIN,
      PAL_MODE_OUTPUT_PUSHPULL);  // IMD
  palSetPadMode(BMS_FAULT_INDICATOR_PORT, BMS_FAULT_INDICATOR_PIN,
      PAL_MODE_OUTPUT_PUSHPULL);  // BMS
  palSetPadMode(TEMP_FAULT_INDICATOR_PORT, TEMP_FAULT_INDICATOR_PIN,
      PAL_MODE_OUTPUT_PUSHPULL);  // Temp

  // Init LED fault states to LOW
  palWritePad(IMD_FAULT_INDICATOR_PORT, IMD_FAULT_INDICATOR_PIN,
      PAL_LOW);  // IMD
  palWritePad(BMS_FAULT_INDICATOR_PORT, BMS_FAULT_INDICATOR_PIN,
      PAL_LOW);  // BMS
  palWritePad(TEMP_FAULT_INDICATOR_PORT, TEMP_FAULT_INDICATOR_PIN,
      PAL_LOW);  // Temp

  // Turn off startup sound
  palSetPadMode(STARTUP_SOUND_PORT, STARTUP_SOUND_PIN, PAL_MODE_OUTPUT_PUSHPULL);
  palWritePad(STARTUP_SOUND_PORT, STARTUP_SOUND_PIN, PAL_LOW);

  // Turn off brake light
  palSetPadMode(BRAKE_LIGHT_PORT, BRAKE_LIGHT_PIN, PAL_MODE_OUTPUT_PUSHPULL);
  palWritePad(BRAKE_LIGHT_PORT, BRAKE_LIGHT_PIN, PAL_LOW);

  Vehicle vehicle;

  // Activate CAN driver 1 (PA11 = CANRX, PA12 = CANTX)
  // CanBus canBus(0x680, CanBusBaudRate::k250k, true);
  CanBus canBus(kNodeIdPrimary, CanBusBaudRate::k250k, false);
  chibios_rt::Mutex canBusMut;

  // create void* compatible obj
  std::vector<void*> args = {&canBus, &canBusMut};

  // Indicate startup - blink then stay on
  palSetPadMode(STARTUP_LED_PORT, STARTUP_LED_PIN, PAL_MODE_OUTPUT_PUSHPULL);
  for (uint8_t i = 0; i < 2; i++) {
    palWritePad(STARTUP_LED_PORT, STARTUP_LED_PIN, PAL_LOW);
    chThdSleepMilliseconds(300);
    palWritePad(STARTUP_LED_PORT, STARTUP_LED_PIN, PAL_HIGH);
    chThdSleepMilliseconds(300);
  }

  // start the CAN TX/RX threads
  chThdCreateStatic(heartbeatThreadFuncWa, sizeof(heartbeatThreadFuncWa), NORMALPRIO,
                    heartbeatThreadFunc, &args);
  // chThdCreateStatic(inputProcThreadFuncWa, sizeof(inputProcThreadFuncWa), NORMALPRIO,
  //                   inputProcThreadFunc, &args);
  // start the CAN heartbeat thread
  chThdCreateStatic(canRxThreadFuncWa, sizeof(canRxThreadFuncWa),
                    NORMALPRIO, canRxThreadFunc, &args);
  // Start SPI thread
  chThdCreateStatic(canTxThreadFuncWa, sizeof(canTxThreadFuncWa), NORMALPRIO + 1,
                    canTxThreadFunc, &args);
  // Start Throttle thread
  chThdCreateStatic(throttleThreadFuncWa, sizeof(throttleThreadFuncWa), NORMALPRIO + 1,
                    throttleThreadFunc, &args);

  // TODO: Fault the system if it doesn't hear from the temp system
  //       within 3 seconds of booting up
  // TODO: Add fault states to vehicle obj
  CANRxFrame msg;
  uint8_t imdFaultPinState = PAL_LOW;
  uint8_t bmsFaultPinState = PAL_LOW;
  uint8_t tempFaultPinState = PAL_LOW;

  while (1) {
    // pop new message
    {
      std::lock_guard<chibios_rt::Mutex> lock(canBusMut);
      msg = canBus.dequeueRxMessage();
    }

    // TODO: Switch the temp system over to standard length IDs
    // switch on system transmitted form
    switch (msg.EID & kSysIdMask) {
      case kSysIdFs:
        // switch on node transmitted from
        switch (msg.EID & kNodeIdMask) {
          case kNodeIdCellTemp:
            // switch on function type
            switch (msg.EID & kFuncIdMask) {
              case kFuncIdFaultStatuses:
                // unpack fault states
                imdFaultPinState = (msg.data8[0] & 0x1) != 0 ? PAL_HIGH : PAL_LOW;
                bmsFaultPinState = (msg.data8[0] & 0x2) != 0 ? PAL_HIGH : PAL_LOW;
                tempFaultPinState = (msg.data8[0] & 0x4) != 0 ? PAL_HIGH : PAL_LOW;

                // drive LEDs (did-faulted == PAL_HIGH == LED_ON)
                palWritePad(IMD_FAULT_INDICATOR_PORT, IMD_FAULT_INDICATOR_PIN,
                    imdFaultPinState);
                palWritePad(BMS_FAULT_INDICATOR_PORT, BMS_FAULT_INDICATOR_PIN,
                    bmsFaultPinState);
                palWritePad(TEMP_FAULT_INDICATOR_PORT, TEMP_FAULT_INDICATOR_PIN,
                    tempFaultPinState);
                break;
              default:
                break;
            }
            break;
          default:
            break;
        }
        break;
      default:
        break;
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
