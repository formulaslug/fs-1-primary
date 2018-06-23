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

static virtual_timer_t vtLedBspd, vtLedStartup, vtLedCan2Status,
                       vtLedCan1Status;

// TODO: Implement
//       1) TimerChSubsys that supports periodic and one-off lambdas and funcs and/or send signals to provided threads
//       2) DigOutChSubsys that uses Timer subsys for async output control
//       3) Down the road, implement a thread abstraction on top of chibios
static void ledBspdOff(void* p) {
  (void)p;
  palClearPad(BSPD_FAULT_INDICATOR_PORT, BSPD_FAULT_INDICATOR_PIN);
}

static void ledStartupOff(void* p) {
  (void)p;
  palClearPad(STARTUP_LED_PORT, STARTUP_LED_PIN);
}

static void ledCan1StatusOff(void* p) {
  (void)p;
  palClearPad(CAN1_STATUS_LED_PORT, CAN1_STATUS_LED_PIN);
}

static void ledCan2StatusOff(void* p) {
  (void)p;
  palClearPad(CAN2_STATUS_LED_PORT, CAN2_STATUS_LED_PIN);
}

/**
 *********************************************************
 * START Uart Stuff
 *********************************************************
 */
constexpr char kStartByte = '1';
constexpr char kStopByte = '0';
constexpr uint8_t kUartOkMask = EVENT_MASK(1);
constexpr uint8_t kUartChMask = EVENT_MASK(4); // app not ready for char
static thread_t *uart1RxThread;
static char lostCharUart1;
static bool packetBufferRead = false; // do this the right way with signals or something

/*
 * This callback is invoked when a character is received but the application
 * was not ready to receive it, the character is passed as parameter.
 */
static void rxchar(UARTDriver *uartp, uint16_t c) {
  (void)uartp;
  eventmask_t events = kUartChMask;

  chSysLockFromISR();
  // queue this character for ingestion
  lostCharUart1 = c;
  chEvtSignalI(uart1RxThread, events);
  chSysUnlockFromISR();
}

/*
 * This callback is invoked when a receive buffer has been completely written.
 */
static void rxend(UARTDriver *uartp) {
  (void)uartp;
  eventmask_t events = kUartOkMask;

  chSysLockFromISR();
  // signal UART 1 RX to read
  chEvtSignalI(uart1RxThread, events);
  chSysUnlockFromISR();
}

/*
 * This callback is invoked when a transmission buffer has been completely
 * read by the driver.
 */
static void txend1(UARTDriver *uartp) {
  (void)uartp;
  chSysLockFromISR();
  packetBufferRead = true;
  chSysUnlockFromISR();
}

/*
 * This callback is invoked when a transmission has physically completed.
 */
static void txend2(UARTDriver *uartp) {
  (void)uartp;
}

// UART 1 driver configuration structure.
static UARTConfig uart_cfg_1 = {
  txend1,          // callback: transmission buffer completely read by the driver
  txend2,          // callback: a transmission has physically completed
  rxend,           // callback: a receive buffer has been completely written
  rxchar,          // callback: a character is received but the
                   //           application was not ready to receive it (param)
  NULL,            // callback: receive error w/ errors mask as param
  115200,          // Bit rate.
  0,               // Initialization value for the CR1 register.
  USART_CR2_LINEN, // Initialization value for the CR2 register.
  0                // Initialization value for the CR3 register.
};

/*
 * UART 1 RX Thread
 */
static THD_WORKING_AREA(uart1RxThreadFuncWa, 128);
static THD_FUNCTION(uart1RxThreadFunc, eventQueue) {
  uint16_t rxBuffer[16];

  while (true) {
    // start the first receive
    uartStopReceive(&UARTD3);
    uartStartReceive(&UARTD3, 1, rxBuffer);

    // waiting for any and all events (generated from callback)
    eventmask_t event = chEvtWaitAny(ALL_EVENTS);

    if (event) {

      if (event & kUartOkMask) {
        // handle regular byte
        uartStopReceive(&UARTD3);
        uartStartReceive(&UARTD3, 1, rxBuffer);
      }
      if (event & kUartChMask) {
        // handle lost char byte
        rxBuffer[0] = lostCharUart1;
      }

      // Push byte to FSM's event queue
      Event e = Event(Event::Type::kUartRx, rxBuffer[0]);

      // Todo: can't push directly from the callback, need
      //       to check if the lock is already acquired or
      //       temporarily stored data internal to the abstraction
      static_cast<EventQueue*>(eventQueue)->push(e);
    }
  }
}

/**
 *********************************************************
 * END Uart Stuff
 *********************************************************
 */

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
  // UART TX/RX
  palSetPadMode(GPIOC, 10, PAL_MODE_ALTERNATE(7)); // USART3_TX, PAL_MODE_OUTPUT_PUSHPULL
  palSetPadMode(GPIOC, 11, PAL_MODE_ALTERNATE(7)); // USART3_RX, PAL_MODE_OUTPUT_PUSHPULL
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
                PAL_MODE_OUTPUT_PUSHPULL);

  // Init LED states to LOW (including faults)
  // TODO: init to correct state
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

  // Start UART driver 1 (make sure before starting UART RX thread)
  // left off-moving uart start to before creating static thread
  uartStart(&UARTD3, &uart_cfg_1);
  uartStopSend(&UARTD3);

  char txPacketArray[6] = {'S','T','A','R','T', ' '};
  uartStartSend(&UARTD3, 6, txPacketArray);

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
  // TODO: Note: Uncomment adc after resolving lock-queue-caused
  //       race-condition from uart thread
  //chThdCreateStatic(adcThreadFuncWa, sizeof(adcThreadFuncWa), NORMALPRIO,
  //                  adcThreadFunc, &adcChSubsys);
  chThdCreateStatic(digInThreadFuncWa, sizeof(digInThreadFuncWa), NORMALPRIO,
                    digInThreadFunc, &digInChSubsys);
  // thread names necessary for signaling between threads
  uart1RxThread = chThdCreateStatic(uart1RxThreadFuncWa,
      sizeof(uart1RxThreadFuncWa), NORMALPRIO, uart1RxThreadFunc,
      &fsmEventQueue);

  adcChSubsys.addPin(Gpio::kA1);  // add brake input
  adcChSubsys.addPin(Gpio::kA2);  // add throttle input

  digInChSubsys.addPin(DigitalInput::kTriStateUp);

  // TODO: Fault the system if it doesn't hear from the temp system
  //       within 3 seconds of booting up
  AnalogFilter throttleFilter = AnalogFilter();

  // Indicate startup - blink then stay on
  for (uint8_t i = 0; i < 5; i++) {
    palSetPad(STARTUP_LED_PORT, STARTUP_LED_PIN);
    chThdSleepMilliseconds(50);
    palClearPad(STARTUP_LED_PORT, STARTUP_LED_PIN);
    chThdSleepMilliseconds(50);
  }

  palClearPad(CAN2_STATUS_LED_PORT, CAN2_STATUS_LED_PIN);
  chThdSleepMilliseconds(300);

  while (1) {
    // always deplete the queue to help ensure that events are
    // processed faster than they're generated
    while (fsmEventQueue.size() > 0) {
      Event e = fsmEventQueue.pop();


      if (e.type() == Event::Type::kUartRx) {
        uint8_t byteVal = e.getByte();
        uartStopSend(&UARTD3);
        char txPacketArray2[1];
        txPacketArray2[0] = (char)byteVal;
        uartStartSend(&UARTD3, 1, txPacketArray2);
      } else if (e.type() == Event::Type::kDigInTransition) {
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
