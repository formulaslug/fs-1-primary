// Copyright (c) 2018 Formula Slug. All Rights Reserved.

#pragma once

#include <stdint.h>

#include <mutex>
#include <vector>

#include "Gpio.h"
#include "Event.h"
#include "EventQueue.h"
#include "ch.h"
#include "hal.h"

/**
 *
 * UART subsystem, sitting on top of the chibios UART driver. Only one
 * subsystem should be instantiated, and currently, only one driver
 * interface (UARTD3 is supported)
 *
 * @note The UART subsystem currently only supports one interface,
 *       which is currently hardcoded to UARTD3. Supporting mutliple
 *       interfaces requires moving constructor params to
 *       addInterface(...), figuring out how best to store driver
 *       config structs, and handling signals from each of the
 *       respective interfaces in the thread run function (and
 *       defining them with the callbacks passed to the config)
 */
class UartChSubsys {
 public:
  /**
   * @param eventQueue Reference to queue to send this subsystem's
   *        events to. The event queue notifies itself.
   */
  UartChSubsys(EventQueue& eq);

  /**
   * @brief Add a UART interface to the subsystem
   * TODO: To support multiple
   */
  void addInterface(UartInterface ui);

  /**
   * @brief Queue a TX UART Frame for transmission
   * TODO: Implement async transmit function, startSend with
   *       signaling to caller on completion
   */
  void send(char * str, uint16_t len);

  /**
   * @brief Inf loop that MUST be called within staticly created thread
   * TODO: Implement tx thread for async transmit
   */
  void runRxThread();

 private:
  UARTDriver *m_uartp;
  UARTConfig m_uartConfig;
  chibios_rt::Mutex m_uartMut;
  EventQueue& m_eventQueue;
};
