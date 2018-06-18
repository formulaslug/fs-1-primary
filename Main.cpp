// Copyright (c) 2018 Formula Slug. All Rights Reserved.

#include <array>
#include <mutex>

#include "AdcChSubsys.h"
#include "AnalogFilter.h"
#include "CanBus.h"
#include "CanChSubsys.h"
#include "CanOpenPdo.h"
#include "DigInChSubsys.h"
#include "Event.h"
#include "EventQueue.h"
#include "Vehicle.h"
#include "ch.hpp"
#include "hal.h"
#include "mcuconfFs.h"

constexpr uint32_t kMaxPot =
    4096;  // out of 4096 -- 512 is 1/8 max throttle value
constexpr uint32_t kPotTolerance = 10;

static virtual_timer_t vtLedBspd, vtLedStartup;

static void ledBspdOff(void* p) {
  (void)p;
  palClearPad(BSPD_FAULT_INDICATOR_PORT, BSPD_FAULT_INDICATOR_PIN);  // IMD
}

static void ledStartupOff(void* p) {
  (void)p;
  palClearPad(STARTUP_LED_PORT, STARTUP_LED_PIN);
}

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

/**
 * CAN TX/RX, LV/HV subsystem threads
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

/**
 * ADC subsystem thread
 */
static THD_WORKING_AREA(adcThreadFuncWa, 128);
static THD_FUNCTION(adcThreadFunc, adcChSubsys) {
  chRegSetThreadName("CAN RX HV");
  static_cast<AdcChSubsys*>(adcChSubsys)->runThread();
}

/**
 * Digital Input subsystem thread
 */
static THD_WORKING_AREA(digInThreadFuncWa, 128);
static THD_FUNCTION(digInThreadFunc, digInChSubsys) {
  chRegSetThreadName("CAN RX HV");
  static_cast<DigInChSubsys*>(digInChSubsys)->runThread();
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

  // Pin initialization
  // Analog inputs
  // palSetPadMode(STEERING_VALUE_PORT, STEERING_VALUE_PIN,
  // PAL_MODE_INPUT_ANALOG); palSetPadMode(BRAKE_VALUE_PORT, BRAKE_VALUE_PIN,
  // PAL_MODE_INPUT_ANALOG); palSetPadMode(RIGHT_THROTTLE_PORT,
  // RIGHT_THROTTLE_PIN, PAL_MODE_INPUT_ANALOG);
  // palSetPadMode(LEFT_THROTTLE_PORT, LEFT_THROTTLE_PIN,
  // PAL_MODE_INPUT_ANALOG); Digital inputs commented out to confirm that
  // subsystem is working palSetPadMode(NEUTRAL_BUTTON_PORT, NEUTRAL_BUTTON_PIN,
  //     PAL_MODE_INPUT_PULLUP);  // IMD
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
  palSetPadMode(STARTUP_LED_PORT, STARTUP_LED_PIN,
                PAL_MODE_OUTPUT_PUSHPULL);  // Brake light signal

  // Init LED states to LOW (including faults)
  palClearPad(IMD_FAULT_INDICATOR_PORT, IMD_FAULT_INDICATOR_PIN);
  palClearPad(AMS_FAULT_INDICATOR_PORT, AMS_FAULT_INDICATOR_PIN);
  palClearPad(BSPD_FAULT_INDICATOR_PORT, BSPD_FAULT_INDICATOR_PIN);
  palClearPad(STARTUP_SOUND_PORT, STARTUP_SOUND_PIN);
  palClearPad(BRAKE_LIGHT_PORT, BRAKE_LIGHT_PIN);
  palClearPad(STARTUP_LED_PORT, STARTUP_LED_PIN);

  // TODO: implement vehicle singleton
  Vehicle vehicle;

  CanBus canBusLv(kNodeIdPrimary, &CAND1, CanBusBaudRate::k1M, false);
  chibios_rt::Mutex canBusLvMut;

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

  /**
   * Create subsystems
   * @note Subsystems DO NOT RUN unless their run function(s) is/are
   *       called within static threads
   */
  EventQueue fsmEventQueue = EventQueue();

  CanChSubsys canLvChSubsys = CanChSubsys(canBusLv, canBusLvMut, fsmEventQueue);
  CanChSubsys canHvChSubsys = CanChSubsys(canBusHv, canBusHvMut, fsmEventQueue);

  AdcChSubsys adcChSubsys = AdcChSubsys(fsmEventQueue);

  DigInChSubsys digInChSubsys = DigInChSubsys(fsmEventQueue);

  /*
   * Create threads (many of which are driving subsystems)
   */
  chThdCreateStatic(canRxLvThreadFuncWa, sizeof(canRxLvThreadFuncWa),
                    NORMALPRIO, canRxLvThreadFunc, &canLvChSubsys);
  chThdCreateStatic(canTxLvThreadFuncWa, sizeof(canTxLvThreadFuncWa),
                    NORMALPRIO + 1, canTxLvThreadFunc, &canLvChSubsys);
  chThdCreateStatic(heartbeatLvThreadFuncWa, sizeof(heartbeatLvThreadFuncWa),
                    NORMALPRIO, heartbeatLvThreadFunc, &canLvChSubsys);
  chThdCreateStatic(heartbeatHvThreadFuncWa, sizeof(heartbeatHvThreadFuncWa),
                    NORMALPRIO, heartbeatHvThreadFunc, &canHvChSubsys);
  chThdCreateStatic(canTxHvThreadFuncWa, sizeof(canTxHvThreadFuncWa),
                    NORMALPRIO + 1, canTxHvThreadFunc, &canHvChSubsys);
  chThdCreateStatic(canRxHvThreadFuncWa, sizeof(canRxHvThreadFuncWa),
                    NORMALPRIO, canRxHvThreadFunc, &canHvChSubsys);
  // chThdCreateStatic(throttleThreadFuncWa, sizeof(throttleThreadFuncWa),
  // NORMALPRIO + 1,
  //                   throttleThreadFunc, &canHvChSubsys);
  chThdCreateStatic(adcThreadFuncWa, sizeof(adcThreadFuncWa), NORMALPRIO,
                    adcThreadFunc, &adcChSubsys);
  chThdCreateStatic(digInThreadFuncWa, sizeof(digInThreadFuncWa), NORMALPRIO,
                    digInThreadFunc, &digInChSubsys);

  adcChSubsys.addPin(Gpio::kA1);  // add brake input
  adcChSubsys.addPin(Gpio::kA2);  // add throttle input

  digInChSubsys.addPin(DigitalInput::kTriStateUp);

  // TODO: Fault the system if it doesn't hear from the temp system
  //       within 3 seconds of booting up
  //
  AnalogFilter throttleFilter = AnalogFilter();

  while (1) {
    // always deplete the queue to help ensure that events are
    // processed faster than they're generated
    while (fsmEventQueue.size() > 0) {
      Event e = fsmEventQueue.pop();

      if (e.type() == Event::Type::kDigInTransition) {
        palSetPad(BSPD_FAULT_INDICATOR_PORT, BSPD_FAULT_INDICATOR_PIN);  // IMD
        switch (e.digInState()) {
          case true:
            // rising edge
            // flash BSPD LED quickly
            chVTReset(&vtLedBspd);
            chVTSet(&vtLedBspd, TIME_MS2I(200), ledBspdOff, NULL);
            break;
          case false:
            // falling edge
            // flash BSPD LED slowly
            chVTReset(&vtLedBspd);
            chVTSet(&vtLedBspd, TIME_MS2I(50), ledBspdOff, NULL);
            break;
        }
      } else if (e.type() == Event::Type::kCanRx) {
        std::array<uint16_t, 8> canFrame = e.canFrame();
        uint32_t canEid = e.canEid();

        // handle packet types
        switch (canEid & kNodeIdMask) {
          case kNodeIdCellTemp:
            palSetPad(STARTUP_LED_PORT, STARTUP_LED_PIN);
            chVTReset(&vtLedStartup);
            chVTSet(&vtLedStartup, TIME_MS2I(50), ledStartupOff, NULL);
            break;
          default:
            break;
        }
      } else if (e.type() == Event::Type::kAdcConversion) {
        if (e.adcPin() == Gpio::kA2) {
          uint16_t throttle =
              throttleFilter.filterLms(static_cast<uint16_t>(e.adcValue()));

          // output non-zero if passed sensitivity margin
          if (throttle < 130) {
            ThrottleMessage throttleMessage(0);
            canLvChSubsys.startSend(throttleMessage);
          } else {
            ThrottleMessage throttleMessage(throttle);
            canLvChSubsys.startSend(throttleMessage);
          }
        } else if (e.adcPin() == Gpio::kA1) {
          // indicate received message
          ThrottleMessage throttleMessage(0x1000);
          canLvChSubsys.startSend(throttleMessage);
        }
      }
    }

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

    // TODO: use condition var to signal that events are present in the queue
    chThdSleepMilliseconds(
        1);  // must be fast enough to deplete event queue quickly enough
  }
}
