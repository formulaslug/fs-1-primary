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
  // TODO: Get UART callbacks working in order to signal the thread
  m_uartConfig = {
    NULL,//txend1,          // callback: transmission buffer completely read
                     //           by the driver
    NULL,//txend2,          // callback: a transmission has physically completed
    NULL,//rxend,           // callback: a receive buffer has been completely
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
 */
void UartChSubsys::runRxThread() {
  chThdSleepMilliseconds(10000);
  //event_listener_t el;
  //chEvtRegister(&(m_canBus.m_canp->rxfull_event), &el, 0);

  //while (true) {
  //  if (chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(100)) == 0) {
  //    continue;
  //  }

  //  // receive any present messages from hardware
  //  {
  //    std::lock_guard<chibios_rt::Mutex> lock(m_canBusMut);
  //    m_canBus.processRxMessages();
  //  }

  //  // generate events from any received messages
  //  while (m_canBus.rxQueueSize() > 0) {
  //    // get CAN message
  //    CANRxFrame msg = m_canBus.dequeueRxMessage();
  //    // create event
  //    std::array<uint16_t, 8> frame = {0, 0, 0, 0, 0, 0, 0, 0};
  //    // Event e = Event();
  //    // write data bytes to event params
  //    for (int i = 0; i < 8; i++) {
  //      // pushing data bytes to event param vector in reverse order,
  //      // such that popping off stack will result in correct order
  //      frame[i] = msg.data8[i];
  //    }
  //    // write COBID to event params
  //    palSetPad(STARTUP_LED_PORT, STARTUP_LED_PIN);
  //    Event e = Event(Event::Type::kCanRx, msg.EID, frame);
  //    // push event
  //    m_eventQueue.push(e);
  //  }
  //}

  //chEvtUnregister(&(m_canBus.m_canp->rxfull_event), &el);
}
