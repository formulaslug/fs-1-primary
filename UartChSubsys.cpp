// Copyright (c) 2018 Formula Slug. All Rights Reserved.

#include "UartChSubsys.h"

#include <mutex>

#include "Gpio.h"
#include "Event.h"
#include "EventQueue.h"
#include "ch.h"
#include "hal.h"
#include "mcuconfFs.h"

/**
 * TODO: - Remove driver pointer from constructor
 *       - Add an addInterface function that takes a UartInterface
 *       - set/start interface with default config, add note about
 *         currently only supporting one interface (due to needing
 *         to figure out where to start the config that the driver
 *         needs a pointer to)
 *       - Add private pin mappings for each interface and notes about
 *         which interface uses which pins in the public section
 *         under the addInterface member func
 * @note: Must use m_uartMut when writing to UART interface
 */
UartChSubsys::UartChSubsys(EventQueue& eq) : m_eventQueue(eq) {}

/**
 * TODO: Use ui param to determine interface config, even if only one
 *       interface is supported
 * TODO: Support multiple interfaces -- need event queus and driver
 *       configs for each interface
 * TODO: Implement variable baud
 * @note Have not as of yet gotten chibios callbacks to work with
 *       subsystem abstractions. Error callbacks should be
 *       implemented.
 */
void UartChSubsys::addInterface(UartInterface ui) {
  // set default config
  // TODO: Initialize interface pins
  // TODO: Get UART callbacks working in order to signal the thread
  //       (read the article on this before continuing with
  //       implementation). Can also checkout how std::thread
  //       abstracts on top of pthreads, given that pthreads take a
  //       void pointer for the function to call
  m_uartConfig = {
    NULL,//txend1,          // callback: transmission buffer completely read
                     //           by the driver
    NULL,//txend2,          // callback: a transmission has physically completed
    rxend,//rxend,           // callback: a receive buffer has been completely
                     //           written
    NULL,//rxchar,          // callback: a character is received but the
                     //           application was not ready to receive
                     //           it (param)
    NULL,            // callback: receive error w/ errors mask as param
    115200,          // Bit rate.
    0,               // Initialization value for the CR1 register.
    USART_CR2_LINEN, // Initialization value for the CR2 register.
    0                // Initialization value for the CR3 register.
  };
  // set default driver (TODO: make it based on ui)
  m_uartp = &UARTD3;
  // start interface with default config
  uartStart(m_uartp, &m_uartConfig);
}

/**
 * TODO: The UART transmit here is actually non-blockin, which it
 *        shouldn't be as it defeats the purpose of acquiring a lock
 *        on the interface before transmitting. Get blocking version
 *        working, then implement non-blocking the right way.
 */
void UartChSubsys::send(char * str, uint16_t len) {
  std::lock_guard<chibios_rt::Mutex> lock(m_uartMut);
  // non-blocking transmit
  uartStopSend(m_uartp);
  uartStartSend(m_uartp, len, str);
}

/*
 * @brief subsystem run function for CAN TX called from within a
 *        ChibiOS static thread
 * TODO: Implement callbacks for RX
 */
void UartChSubsys::runRxThread() {
  uint16_t rxBuffer[11] = {};

  while (true) {
    // start the first receive
    uartStopReceive(m_uartp);
    uartStartReceive(m_uartp, 1, rxBuffer);

    Event e = Event(Event::Type::kUartRx, rxBuffer[0]);
    m_eventQueue.push(e);

    //std::vector<Event> events;
    //events.push_back(Event(Event::Type::kUartRx, rxBuffer[0]));
    //m_eventQueue.push(events);

    chThdSleepMilliseconds(100);

    //// waiting for any and all events (generated from callback)
    //eventmask_t event = chEvtWaitAny(ALL_EVENTS);

    //if (event) {

    //  if (event & kUartOkMask) {
    //    // handle regular byte
    //    uartStopReceive(&UARTD3);
    //    uartStartReceive(&UARTD3, 1, rxBuffer);
    //  }
    //  if (event & kUartChMask) {
    //    // handle lost char byte
    //    rxBuffer[0] = lostCharUart1;
    //  }

    //  // Push byte to FSM's event queue
    //  Event e = Event(Event::Type::kUartRx, rxBuffer[0]);

    //  // Todo: can't push directly from the callback, need
    //  //       to check if the lock is already acquired or
    //  //       temporarily stored data internal to the abstraction
    //  static_cast<EventQueue*>(eventQueue)->push(e);
    //}
  }
}

void UartChSubsys::rxend(UARTDriver *uartp) {
  (void)uartp;
  eventmask_t events = kUartOkMask;

  palSetPad(STARTUP_LED_PORT, STARTUP_LED_PIN);

  //chSysLockFromISR();
  //// signal UART 1 RX to read
  //chEvtSignalI(uart1RxThread, events);
  //chSysUnlockFromISR();
}

