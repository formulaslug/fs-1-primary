// Copyright (c) 2018 Formula Slug. All Rights Reserved.

#include <array>
#include <mutex>

#include "Vehicle.h"
#include "ch.hpp"
#include "hal.h"
#include "pinconf.h"

// new chibios-subsystem wrapped includes
// @note WARNING experienced very odd issue with the UART abstraction where it
//       seemed to break the RT kernel when we created a vector inside
//       CircularBuffer.h/inc. Then it magically started working again after
//       reverting all changes we introduced to try and diagnose the issue
#include "chibios-subsys/subsystems/uart/Uart.h"

// @TODO get from chibios-subsys instead
#include "AdcChSubsys.h"

//#include "chibios-subsys/common/CircularBuffer.h"
//#include "chibios-subsys/common/Event.h"

/**
 * ADC subsystem thread
 * @TODO Get rid of this once the ADC subsystem no longer requires a dedicated
         thread to run (and is instead purely callback driven)
 */
static THD_WORKING_AREA(adcThreadFuncWa, 128);
static THD_FUNCTION(adcThreadFunc, adcChSubsys) {
  chRegSetThreadName("CAN RX HV");
  static_cast<AdcChSubsys*>(adcChSubsys)->runThread();
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

  // @TODO Set UART input modes (eventually should be done inside of the UART
  //       abstraction)
  palSetPadMode(GPIOC, 10, PAL_MODE_ALTERNATE(7)); // USART3_TX
  palSetPadMode(GPIOC, 11, PAL_MODE_ALTERNATE(7)); // USART3_RX

  EventQueue fsmEventQueue = EventQueue();

  // CAN (Put this inside the abstraction)
  palSetPadMode(CAN1_STATUS_LED_PORT, CAN1_STATUS_LED_PIN,
      PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(CAN2_STATUS_LED_PORT, CAN2_STATUS_LED_PIN,
      PAL_MODE_OUTPUT_PUSHPULL);

  // MISC I/O init
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

  // clear all dig outputs
  palClearPad(IMD_FAULT_INDICATOR_PORT, IMD_FAULT_INDICATOR_PIN);
  palClearPad(AMS_FAULT_INDICATOR_PORT, AMS_FAULT_INDICATOR_PIN);
  palClearPad(BSPD_FAULT_INDICATOR_PORT, BSPD_FAULT_INDICATOR_PIN);
  palClearPad(STARTUP_SOUND_PORT, STARTUP_SOUND_PIN);
  palClearPad(BRAKE_LIGHT_PORT, BRAKE_LIGHT_PIN);
  palClearPad(STARTUP_LED_PORT, STARTUP_LED_PIN);


  // set some of them
  palSetPad(IMD_FAULT_INDICATOR_PORT, IMD_FAULT_INDICATOR_PIN);
  palSetPad(AMS_FAULT_INDICATOR_PORT, AMS_FAULT_INDICATOR_PIN);
  palSetPad(BSPD_FAULT_INDICATOR_PORT, BSPD_FAULT_INDICATOR_PIN);
  palClearPad(BRAKE_LIGHT_PORT, BRAKE_LIGHT_PIN);
  palSetPad(CAN2_STATUS_LED_PORT, CAN2_STATUS_LED_PIN);
  palSetPad(CAN1_STATUS_LED_PORT, CAN1_STATUS_LED_PIN);

  // create the UART interface and start using it
  cal::Uart uart = cal::Uart(UartInterface::kD3, fsmEventQueue);

  // create the ADC interface and start using it
  AdcChSubsys adcChSubsys = AdcChSubsys(fsmEventQueue);
  // create its thread
  chThdCreateStatic(adcThreadFuncWa, sizeof(adcThreadFuncWa), NORMALPRIO,
                    adcThreadFunc, &adcChSubsys);
  // start the ADC readings
  adcChSubsys.addPin(Gpio::kA1);  // add brake input

  while (1) {
    // always deplete the queue to help ensure that events are
    // processed faster than they're generated
    while (fsmEventQueue.size() > 0) {
      Event e = fsmEventQueue.pop();

      switch (e.type()) {
        case Event::Type::kUartRx:
          {
            char byte = e.getByte();
            // test code
            if (byte == '1') {
              palTogglePad(BRAKE_LIGHT_PORT, BRAKE_LIGHT_PIN);
            }
            // send UART byte back to labtop
            uart.send(byte);
          }
          break;
        case Event::Type::kAdcConversion:
          {
            uint32_t reading = e.adcValue();
            uart.send(std::string("ADC reading:") + std::to_string(reading) + std::string("\r\n"));
          }
          break;
      }
    }

    // TODO: use condition var to signal that events are present in the queue
    chThdSleepMilliseconds(
        1);  // must be fast enough to deplete event queue quickly enough
  }
}
