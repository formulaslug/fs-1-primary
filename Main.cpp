// Copyright (c) 2018 Formula Slug. All Rights Reserved.

#include <array>
#include <mutex>

#include "CanBus.h"
#include "CanOpenPdo.h"
#include "Vehicle.h"
#include "ch.hpp"
#include "hal.h"
#include "mcuconfFs.h"
#include "Event.h"
#include "EventQueue.h"
#include "CanChSubsys.h"

constexpr uint32_t kMaxPot = 4096; // out of 4096 -- 512 is 1/8 max throttle value
constexpr uint32_t kPotTolerance = 10;

#define ADC_GRP1_NUM_CHANNELS   2
#define ADC_GRP1_BUF_DEPTH      8

static adcsample_t samples1[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];

/*
 * ADC streaming callback.
 */
size_t nx = 0, ny = 0;
static void adcerrorcallback(ADCDriver *adcp, adcerror_t err) {

  (void)adcp;
  (void)err;
}

static void adcendcallback(ADCDriver *adcp, adcsample_t *buffer, size_t n) {
  (void)adcp;
  (void)buffer;
  (void)n;
  palWritePad(IMD_FAULT_INDICATOR_PORT, IMD_FAULT_INDICATOR_PIN,
      PAL_HIGH);  // IMD
}

static const ADCConversionGroup adcgrpcfg1 = {
  FALSE,
  ADC_GRP1_NUM_CHANNELS,
  adcendcallback,
  adcerrorcallback,
  0,                        /* CR1 */
  ADC_CR2_SWSTART,          /* CR2 */
  // SMPR1: Samples times for channels 10-17
  0,
  // SMPR2: Samples times for channels 0-9
  // @note ADC_SAMPLE_[X] is in cycles of the ADC's clock
  // @note ADC_SMPR2_SMP_AN[X] corresponds to ADC_CHANNEL_IN[X]
  ADC_SMPR2_SMP_AN1(ADC_SAMPLE_480) | ADC_SMPR2_SMP_AN2(ADC_SAMPLE_480),
  0,                        /* SQR1 */
  0,                        /* SQR2 */
  // SQR1: Conversion group sequence 1-6
  // @brief specify which channels, in which order, are sampled per
  //        conversion sequence
  // @note Use ADC_SQR3_SQ[X]_N to indicate sequence number of channel
  //       ADC_CHANNEL_IN[Y]
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN1) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN2)
};

// TODO: test brake and throttle together

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
static THD_WORKING_AREA(heartbeatHvThreadFuncWa, 128);
static THD_FUNCTION(heartbeatHvThreadFunc, canChSubsys) {
  chRegSetThreadName("NODE HEARTBEAT HV");

  while (1) {
    HeartbeatMessage heartbeatMessage(0x1);
    static_cast<CanChSubsys*>(canChSubsys)->startSend(heartbeatMessage);
    chThdSleepMilliseconds(1000);
  }
}

/**
 * @desc Performs period tasks every second for LV CAN bus
 */
static THD_WORKING_AREA(heartbeatLvThreadFuncWa, 128);
static THD_FUNCTION(heartbeatLvThreadFunc, canChSubsys) {
  chRegSetThreadName("NODE HEARTBEAT LV");

  while (1) {
    HeartbeatMessage heartbeatMessage(0x1);
    static_cast<CanChSubsys*>(canChSubsys)->startSend(heartbeatMessage);
    chThdSleepMilliseconds(1000);
  }
}

/**
 * @desc Performs period tasks every second
 */
// static THD_WORKING_AREA(throttleThreadFuncWa, 128);
// static THD_FUNCTION(throttleThreadFunc, canChSubsys) {
//   chRegSetThreadName("THROTTLE");
//
//   while (1) {
//     ThrottleMessage throttleMessage(1);
//     static_cast<CanChSubsys*>(canChSubsys)->startSend(throttleMessage);
//     chThdSleepMilliseconds(200);
//   }
// }

/*
 * CAN TX/RX threads for subsystem
 */
static THD_WORKING_AREA(canTxLvThreadFuncWa, 128);
static THD_FUNCTION(canTxLvThreadFunc, canChSubsys) {
  chRegSetThreadName("CAN TX LV");
  static_cast<CanChSubsys*>(canChSubsys)->runTxThread();
}

static THD_WORKING_AREA(canRxLvThreadFuncWa, 128);
static THD_FUNCTION(canRxLvThreadFunc, canChSubsys) {
  chRegSetThreadName("CAN RX LV");
  static_cast<CanChSubsys*>(canChSubsys)->runRxThread();
}

static THD_WORKING_AREA(canTxHvThreadFuncWa, 128);
static THD_FUNCTION(canTxHvThreadFunc, canChSubsys) {
  chRegSetThreadName("CAN TX HV");
  static_cast<CanChSubsys*>(canChSubsys)->runTxThread();
}

static THD_WORKING_AREA(canRxHvThreadFuncWa, 128);
static THD_FUNCTION(canRxHvThreadFunc, canChSubsys) {
  chRegSetThreadName("CAN RX HV");
  static_cast<CanChSubsys*>(canChSubsys)->runRxThread();
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
  // TODO: remember these are commented out
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

  // TODO: implement vehicle singleton
  Vehicle vehicle;

  // CanBus canBus(kNodeIdPrimary, &CAND1, CanBusBaudRate::k500k, false);
  CanBus canBusLv(kNodeIdPrimary, &CAND1, CanBusBaudRate::k1M, false);
  chibios_rt::Mutex canBusLvMut;

  // CanBus canBusHV(kNodeIdPrimary, &CAND2, CanBusBaudRate::k500k, false);
  CanBus canBusHv(kNodeIdPrimary, &CAND2, CanBusBaudRate::k1M, false);
  chibios_rt::Mutex canBusHvMut;

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


  /*
   * Create subsystems
   */
  EventQueue fsmEventQueue = EventQueue();
  CanChSubsys canLvChSubsys = CanChSubsys(canBusLv, canBusLvMut, fsmEventQueue);
  CanChSubsys canHvChSubsys = CanChSubsys(canBusHv, canBusHvMut, fsmEventQueue);

  /*
   * Create threads (many of which are driving subsystems)
   */
  chThdCreateStatic(canRxLvThreadFuncWa, sizeof(canRxLvThreadFuncWa),
                    NORMALPRIO, canRxLvThreadFunc, &canLvChSubsys);
  chThdCreateStatic(canTxLvThreadFuncWa, sizeof(canTxLvThreadFuncWa), NORMALPRIO + 1,
                    canTxLvThreadFunc, &canLvChSubsys);
  chThdCreateStatic(heartbeatLvThreadFuncWa, sizeof(heartbeatLvThreadFuncWa), NORMALPRIO,
                    heartbeatLvThreadFunc, &canLvChSubsys);
  chThdCreateStatic(heartbeatHvThreadFuncWa, sizeof(heartbeatHvThreadFuncWa), NORMALPRIO,
                    heartbeatHvThreadFunc, &canHvChSubsys);
  chThdCreateStatic(canTxHvThreadFuncWa, sizeof(canTxHvThreadFuncWa), NORMALPRIO + 1,
                    canTxHvThreadFunc, &canHvChSubsys);
  chThdCreateStatic(canRxHvThreadFuncWa, sizeof(canRxHvThreadFuncWa),
                    NORMALPRIO, canRxHvThreadFunc, &canHvChSubsys);
  // chThdCreateStatic(throttleThreadFuncWa, sizeof(throttleThreadFuncWa), NORMALPRIO + 1,
  //                   throttleThreadFunc, &canHvChSubsys);




  // TODO: Fault the system if it doesn't hear from the temp system
  //       within 3 seconds of booting up
  // TODO: Add fault states to vehicle obj
  // CANRxFrame msg;
  // uint8_t imdFaultPinState = PAL_LOW;
  // uint8_t bmsFaultPinState = PAL_LOW;
  // uint8_t tempFaultPinState = PAL_LOW;

  uint32_t throttleOutputs[2] = {};
  // uint32_t throttleInputs[11] = {};
  uint32_t throttleInputs[11] = {};
  uint8_t currentThrottleIndex = 0;
  int32_t prevThrottle = 0; // save for nonlinear threshold to update dash
  const uint8_t requiredDelta = 20;

  while (1) {

    if (fsmEventQueue.size() > 0) {
      // indicate received message
      palWritePad(IMD_FAULT_INDICATOR_PORT, IMD_FAULT_INDICATOR_PIN,
          PAL_HIGH);  // IMD
      palWritePad(AMS_FAULT_INDICATOR_PORT, AMS_FAULT_INDICATOR_PIN,
          PAL_HIGH);  // AMS
      palWritePad(BSPD_FAULT_INDICATOR_PORT, BSPD_FAULT_INDICATOR_PIN,
          PAL_HIGH);  // BSPD
    }

    // read all digital inputs
    // auto neutralButton = palReadPad(NEUTRAL_BUTTON_PORT, NEUTRAL_BUTTON_PIN);
    // auto driveButton = palReadPad(DRIVE_BUTTON_PORT, DRIVE_BUTTON_PIN);
    // auto driveModeButton = palReadPad(DRIVE_MODE_BUTTON_PORT, DRIVE_MODE_BUTTON_PIN);
    // auto bspdFault = palReadPad(BSPD_FAULT_PORT, BSPD_FAULT_PIN);

    adcConvert(&ADCD1, &adcgrpcfg1, samples1, ADC_GRP1_BUF_DEPTH);

    // scaling to be from vref of 2.9V (as it appears Vdd is staying
    // around on the discovery board when powered over USB)
    // uint32_t throttleValue = static_cast<uint16_t>(0xfff & samples1[0]);

    // fetch new input
    throttleInputs[currentThrottleIndex] = static_cast<uint16_t>(0xfff & samples1[1]);

    // shift outputs
    throttleOutputs[0] = throttleOutputs[1];

    // y(n) = y(n-1) + x(n)/N - x(n-N)/N
    uint8_t nextThrottleIndex = (currentThrottleIndex + 1) % 11;
    throttleOutputs[1] = throttleOutputs[0] + throttleInputs[currentThrottleIndex]/10 - throttleInputs[nextThrottleIndex]/10;

    if (throttleOutputs[1] < 130) {
      ThrottleMessage throttleMessage(0);
      canLvChSubsys.startSend(throttleMessage);
    } else {
      int32_t delta = (int32_t)throttleOutputs[1] - prevThrottle;
      if (delta >= requiredDelta || delta <= -1*requiredDelta) {
        ThrottleMessage throttleMessage(throttleOutputs[1]);
        canLvChSubsys.startSend(throttleMessage);
        prevThrottle = (int32_t)throttleOutputs[1];
      } else {
        ThrottleMessage throttleMessage(prevThrottle);
        canLvChSubsys.startSend(throttleMessage);
      }
    }

    // increment current index in circular buffer
    currentThrottleIndex  = (currentThrottleIndex + 1) % 11;

#if 0
    // reflect button states on LEDs
    palWritePad(AMS_FAULT_INDICATOR_PORT, AMS_FAULT_INDICATOR_PIN,
        neutralButton ? PAL_LOW : PAL_HIGH);  // AMS
    palWritePad(IMD_FAULT_INDICATOR_PORT, IMD_FAULT_INDICATOR_PIN,
        driveButton ? PAL_LOW : PAL_HIGH);  // IMD
    palWritePad(BSPD_FAULT_INDICATOR_PORT, BSPD_FAULT_INDICATOR_PIN,
        driveModeButton ? PAL_LOW : PAL_HIGH);  // BSPD
    palWritePad(STARTUP_SOUND_PORT, STARTUP_SOUND_PIN,
        bspdFault ? PAL_HIGH : PAL_LOW);  // RTDS signal

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

    chThdSleepMilliseconds(10); // update dash at ~ 66Hz
  }
}
