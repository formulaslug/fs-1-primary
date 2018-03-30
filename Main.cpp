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

constexpr uint32_t kMaxPot = 1023;
constexpr uint32_t kPotTolerance = 10;

#define ADC_GRP1_NUM_CHANNELS   1
#define ADC_GRP1_BUF_DEPTH      1

static adcsample_t samples1[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];

/*
 * ADC streaming callback.
 */
size_t nx = 0, ny = 0;
static void adcerrorcallback(ADCDriver *adcp, adcerror_t err) {

  (void)adcp;
  (void)err;
}

static const ADCConversionGroup adcgrpcfg1 = {
  FALSE,
  ADC_GRP1_NUM_CHANNELS,
  NULL,
  adcerrorcallback,
  0,                        /* CR1 */
  ADC_CR2_SWSTART,          /* CR2 */
  ADC_SMPR1_SMP_AN11(ADC_SAMPLE_3),
  0,                        /* SMPR2 */
  0,                        /* SQR1 */
  0,                        /* SQR2 */
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN6)
};

// STEERING_VALUE_PIN -> ADC12_IN6 (POT 1)
//
// BRAKE_VALUE_PIN -> ADC123_IN1 (POT 2)
//
// RIGHT_THROTTLE_PIN -> ADC123_IN2 (POT 1)
//
// LEFT_THROTTLE_PIN -> ADC123_IN3 (POT 1)


/*
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
 * @desc Performs period tasks every second for HV CAN bus
 */
static THD_WORKING_AREA(heartbeatHVThreadFuncWa, 128);
static THD_FUNCTION(heartbeatHVThreadFunc, arg) {
  chRegSetThreadName("NODE HEARTBEAT HV");

  while (1) {
    // enqueue heartbeat message to g_canTxQueue
    // TODO: Remove need for node ID param to heartbeat obj (passed
    //       during instantiation of CAN bus)
    const HeartbeatMessage heartbeatMessage(0x2);
    {
      std::lock_guard<chibios_rt::Mutex> lock(CAN_BUS_MUT);
      (CAN_BUS).queueTxMessage(heartbeatMessage);
    }
    // transmit node's (self) heartbeat every 1s
    chThdSleepMilliseconds(1000);
  }
}

/**
 * @desc Performs period tasks every second for LV CAN bus
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
    const ThrottleMessage throttleMessage(1);
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
 * CAN LV TX thread
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
 * CAN LV RX thread
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



/*
 * CAN HV TX thread
 */
static THD_WORKING_AREA(canTxHVThreadFuncWa, 128);
static THD_FUNCTION(canTxHVThreadFunc, arg) {
  chRegSetThreadName("CAN TX HV");

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
 * CAN HV RX thread
 */
static THD_WORKING_AREA(canRxHVThreadFuncWa, 128);
static THD_FUNCTION(canRxHVThreadFunc, arg) {
  event_listener_t el;

  chRegSetThreadName("CAN RX HV");
  chEvtRegister(&CAND2.rxfull_event, &el, 0);

  while (true) {
    if (chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(100)) == 0) {
      continue;
    }
    {
      std::lock_guard<chibios_rt::Mutex> lock(CAN_BUS_MUT);
      (CAN_BUS).processRxMessages();
    }
  }

  chEvtUnregister(&CAND2.rxfull_event, &el);
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
  adcStart(&ADCD1, NULL);
  adcStart(&ADCD2, NULL);
  adcStart(&ADCD3, NULL);

  // Pin initialization
  // Analog inputs
  palSetPadMode(STEERING_VALUE_PORT, STEERING_VALUE_PIN, PAL_MODE_INPUT_ANALOG);
  palSetPadMode(BRAKE_VALUE_PORT, BRAKE_VALUE_PIN, PAL_MODE_INPUT_ANALOG);
  palSetPadMode(RIGHT_THROTTLE_PORT, RIGHT_THROTTLE_PIN, PAL_MODE_INPUT_ANALOG);
  palSetPadMode(LEFT_THROTTLE_PORT, LEFT_THROTTLE_PIN, PAL_MODE_INPUT_ANALOG);
  // Digital inputs
  palSetPadMode(NEUTRAL_BUTTON_PORT, NEUTRAL_BUTTON_PIN,
      PAL_MODE_INPUT_PULLUP);  // IMD
  palSetPadMode(DRIVE_BUTTON_PORT, DRIVE_BUTTON_PIN,
      PAL_MODE_INPUT_PULLUP);  // AMS
  palSetPadMode(DRIVE_MODE_BUTTON_PORT, DRIVE_MODE_BUTTON_PIN,
      PAL_MODE_INPUT_PULLUP);  // BSPD
  palSetPadMode(BSPD_FAULT_PORT, BSPD_FAULT_PIN,
      PAL_MODE_INPUT_PULLUP);  // RTDS signal
  // Digital outputs
  palSetPadMode(IMD_FAULT_INDICATOR_PORT, IMD_FAULT_INDICATOR_PIN,
      PAL_MODE_OUTPUT_PUSHPULL);  // IMD
  palSetPadMode(AMS_FAULT_INDICATOR_PORT, AMS_FAULT_INDICATOR_PIN,
      PAL_MODE_OUTPUT_PUSHPULL);  // AMS
  palSetPadMode(BSPD_FAULT_INDICATOR_PORT, BSPD_FAULT_INDICATOR_PIN,
      PAL_MODE_OUTPUT_PUSHPULL);  // BSPD
  palSetPadMode(STARTUP_SOUND_PORT, STARTUP_SOUND_PIN,
      PAL_MODE_OUTPUT_PUSHPULL);  // RTDS signal
  palSetPadMode(BRAKE_LIGHT_PORT, BRAKE_LIGHT_PIN,
      PAL_MODE_OUTPUT_PUSHPULL);  // Brake light signal

  // Init LED states to LOW (including faults)
  palWritePad(IMD_FAULT_INDICATOR_PORT, IMD_FAULT_INDICATOR_PIN, PAL_LOW);
  palWritePad(AMS_FAULT_INDICATOR_PORT, AMS_FAULT_INDICATOR_PIN, PAL_LOW);
  palWritePad(BSPD_FAULT_INDICATOR_PORT, BSPD_FAULT_INDICATOR_PIN, PAL_LOW);
  palWritePad(STARTUP_SOUND_PORT, STARTUP_SOUND_PIN, PAL_LOW);
  palWritePad(BRAKE_LIGHT_PORT, BRAKE_LIGHT_PIN, PAL_LOW);

  Vehicle vehicle;

  CanBus canBus(kNodeIdPrimary, &CAND1, CanBusBaudRate::k250k, false);
  chibios_rt::Mutex canBusMut;

  CanBus canBusHV(kNodeIdPrimary, &CAND2, CanBusBaudRate::k250k, false);
  chibios_rt::Mutex canBusMutHV;

  // create void* compatible obj
  std::vector<void*> args = {&canBus, &canBusMut};
  std::vector<void*> canArgsHV = {&canBusHV, &canBusMutHV};

  // Indicate startup - blink then stay on
  for (uint8_t i = 0; i < 2; i++) {
    palWritePad(IMD_FAULT_INDICATOR_PORT, IMD_FAULT_INDICATOR_PIN,
        PAL_HIGH);  // IMD
    palWritePad(AMS_FAULT_INDICATOR_PORT, AMS_FAULT_INDICATOR_PIN,
        PAL_HIGH);  // AMS
    palWritePad(BSPD_FAULT_INDICATOR_PORT, BSPD_FAULT_INDICATOR_PIN,
        PAL_HIGH);  // BSPD
    palWritePad(STARTUP_SOUND_PORT, STARTUP_SOUND_PIN,
        PAL_HIGH);  // RTDS
    palWritePad(BRAKE_LIGHT_PORT, BRAKE_LIGHT_PIN,
        PAL_HIGH);  // Brake Light
    chThdSleepMilliseconds(200);
    palWritePad(IMD_FAULT_INDICATOR_PORT, IMD_FAULT_INDICATOR_PIN,
        PAL_LOW);  // IMD
    palWritePad(AMS_FAULT_INDICATOR_PORT, AMS_FAULT_INDICATOR_PIN,
        PAL_LOW);  // AMS
    palWritePad(BSPD_FAULT_INDICATOR_PORT, BSPD_FAULT_INDICATOR_PIN,
        PAL_LOW);  // Temp
    palWritePad(STARTUP_SOUND_PORT, STARTUP_SOUND_PIN,
        PAL_LOW);  // RTDS
    palWritePad(BRAKE_LIGHT_PORT, BRAKE_LIGHT_PIN,
        PAL_LOW);  // Brake Light
    chThdSleepMilliseconds(200);
  }

  // CAN LV threads
  chThdCreateStatic(canRxThreadFuncWa, sizeof(canRxThreadFuncWa),
                    NORMALPRIO, canRxThreadFunc, &args);
  chThdCreateStatic(canTxThreadFuncWa, sizeof(canTxThreadFuncWa), NORMALPRIO + 1,
                    canTxThreadFunc, &args);
  chThdCreateStatic(heartbeatThreadFuncWa, sizeof(heartbeatThreadFuncWa), NORMALPRIO,
                    heartbeatThreadFunc, &args);
  // chThdCreateStatic(throttleThreadFuncWa, sizeof(throttleThreadFuncWa), NORMALPRIO + 1,
  //                   throttleThreadFunc, &args);

  // CAN HV threads
  chThdCreateStatic(heartbeatHVThreadFuncWa, sizeof(heartbeatHVThreadFuncWa), NORMALPRIO,
                    heartbeatHVThreadFunc, &canArgsHV);
  chThdCreateStatic(canTxHVThreadFuncWa, sizeof(canTxHVThreadFuncWa), NORMALPRIO + 1,
                    canTxHVThreadFunc, &canArgsHV);
  chThdCreateStatic(canRxHVThreadFuncWa, sizeof(canRxHVThreadFuncWa),
                    NORMALPRIO, canRxHVThreadFunc, &canArgsHV);
  // chThdCreateStatic(throttleThreadFuncWa, sizeof(throttleThreadFuncWa), NORMALPRIO + 1,
  //                   throttleThreadFunc, &canArgsHV);

  // Old throttle (and other) thread
  // chThdCreateStatic(inputProcThreadFuncWa, sizeof(inputProcThreadFuncWa), NORMALPRIO,
  //                   inputProcThreadFunc, &args);

  // Start Throttle thread

  // TODO: Fault the system if it doesn't hear from the temp system
  //       within 3 seconds of booting up
  // TODO: Add fault states to vehicle obj
  CANRxFrame msg;
  uint8_t imdFaultPinState = PAL_LOW;
  uint8_t bmsFaultPinState = PAL_LOW;
  uint8_t tempFaultPinState = PAL_LOW;

  uint32_t prevBrakeValue = 0,
           prevRightThrottleValue = 0, prevLeftThrottleValue = 0;

  while (1) {
    // read all digital inputs
    auto neutralButton = palReadPad(NEUTRAL_BUTTON_PORT, NEUTRAL_BUTTON_PIN);
    auto driveButton = palReadPad(DRIVE_BUTTON_PORT, DRIVE_BUTTON_PIN);
    auto driveModeButton = palReadPad(DRIVE_MODE_BUTTON_PORT, DRIVE_MODE_BUTTON_PIN);
    auto bspdFault = palReadPad(BSPD_FAULT_PORT, BSPD_FAULT_PIN);

    adcConvert(&ADCD1, &adcgrpcfg1, samples1, ADC_GRP1_BUF_DEPTH);

    // // read all analog inputs
    // uint32_t brakeValue = palReadPad(BRAKE_VALUE_PORT, BRAKE_VALUE_PIN);
    // uint32_t rightThrottleValue = palReadPad(RIGHT_THROTTLE_PORT, RIGHT_THROTTLE_PIN);
    // uint32_t leftThrottleValue = palReadPad(LEFT_THROTTLE_PORT, LEFT_THROTTLE_PIN);

    // const HeartbeatMessage brakeMessage(static_cast<uint16_t>(0xf000 | samples1[0]));
    // {
    //   std::lock_guard<chibios_rt::Mutex> lock(canBusMutHV);
    //   canBusHV.queueTxMessage(brakeMessage);
    // }

    const ThrottleMessage throttleMessage(static_cast<uint16_t>(0xfff & samples1[0]));
    {
      std::lock_guard<chibios_rt::Mutex> lock(canBusMutHV);
      canBusHV.queueTxMessage(throttleMessage);
    }


    // chThdSleepMilliseconds(200);

    // // determine which pot value to use
    // uint32_t potValue = 0;
    // if (brakeValue - prevBrakeValue > 5 || brakeValue - prevBrakeValue < -5) {
    //   potValue = static_cast<uint32_t>(brakeValue);
    // } else {
    //   potValue = static_cast<uint32_t>(rightThrottleValue);
    // }
    //
    // // light a subset of LEDs 1-4 depending on rotation of that pot
    // bool led0 = (potValue >> 2) > (kMaxPot >> 2);            // > 1/4 max pot
    // bool led1 = (potValue >> 2) > (kMaxPot >> 1);            // > 2/4 max pot
    // bool led2 = (potValue >> 2) > (kMaxPot >> 2) * 3;        // > 3/4 max pot
    // bool led3 = (potValue >> 2) > (kMaxPot - kPotTolerance); // > 4/4 - 10 max pot
    //
    // // determine LED states
    // auto ledState0 = neutralButton && led0 ? PAL_HIGH : PAL_LOW;
    // auto ledState1 = driveButton && led1 ? PAL_HIGH : PAL_LOW;
    // auto ledState2 = driveModeButton && led2 ? PAL_HIGH : PAL_LOW;
    // auto ledState3 = bspdFault && led3 ? PAL_HIGH : PAL_LOW;
    //
    // set LEDs (in-order) corresponding to each of the four digital inputs
    // palWritePad(AMS_FAULT_INDICATOR_PORT, AMS_FAULT_INDICATOR_PIN,
    //     ledState0);  // AMS
    // palWritePad(IMD_FAULT_INDICATOR_PORT, IMD_FAULT_INDICATOR_PIN,
    //     ledState1);  // IMD
    // palWritePad(BSPD_FAULT_INDICATOR_PORT, BSPD_FAULT_INDICATOR_PIN,
    //     ledState2);  // BSPD
    // palWritePad(STARTUP_SOUND_PORT, STARTUP_SOUND_PIN,
    //     ledState3);  // RTDS signal


    palWritePad(AMS_FAULT_INDICATOR_PORT, AMS_FAULT_INDICATOR_PIN,
        neutralButton ? PAL_HIGH : PAL_LOW);  // AMS
    // palWritePad(IMD_FAULT_INDICATOR_PORT, IMD_FAULT_INDICATOR_PIN,
    //     driveButton ? PAL_HIGH : PAL_LOW);  // IMD
    // palWritePad(BSPD_FAULT_INDICATOR_PORT, BSPD_FAULT_INDICATOR_PIN,
    //     driveModeButton ? PAL_HIGH : PAL_LOW);  // BSPD
    palWritePad(STARTUP_SOUND_PORT, STARTUP_SOUND_PIN,
        bspdFault ? PAL_HIGH : PAL_LOW);  // RTDS signal

#if 0
    // handle new packet if available
    if (canBusHV.rxQueueSize() > 0 ) {
      msg = canBusHV.dequeueRxMessage();

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
                  palWritePad(AMS_FAULT_INDICATOR_PORT, AMS_FAULT_INDICATOR_PIN,
                      bmsFaultPinState);
                  palWritePad(BSPD_FAULT_INDICATOR_PORT, BSPD_FAULT_INDICATOR_PIN,
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
    }
#endif

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
