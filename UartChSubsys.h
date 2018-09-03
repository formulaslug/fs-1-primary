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
#include "CircularBuffer.h"

/**
 *
 * UART subsystem, sitting on top of the chibios UART driver. Only one
 * subsystem should be instantiated, and currently, only one driver
 * interface (UARTD3 is supported)
 *
 * @TODO Currently has limited throughput on event generation. This is
 *       possibly due to the fsm (event queue consumer) not being able
 *       to process the events quickly enough, although not sure why
 *       exactly that is causing things to break. The event queue is
 *       built on top of the CircularBuffer class, which should just
 *       overwrite elements when the buffer overflows. Either way,
 *       having a larger, fixed-size packet would improve things
 *       - - - - - - - - - - - - - - - - - - - - 
 *       OH, actually, it's definitely that IF the lock on a shared
 *       resource in one of the static callbacks cannot be immediately
 *       acquired, the kernel gets upset b/c this basically results in
 *       an ISR taking a lifetime to exit. This issue is only likely
 *       to occur at high throughput over serial RX
 *       - - - - - - - - - - - - - - - - - - - - 
 *       Yep, that was it. Now just need a mechanism to postpone event
 *       generation when the lock isn't successfully acquired
 *
 * @note The UART subsystem currently only supports one interface,
 *       which is currently hardcoded to UARTD3. Supporting mutliple
 *       interfaces requires moving constructor params to
 *       addInterface(...), figuring out how best to store driver
 *       config structs, and handling signals from each of the
 *       respective interfaces in the thread run function (and
 *       defining them with the callbacks passed to the config)
 */
class UartChSubsys 
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
   * @brief Queue a TX UART Frame for async transmission
   * @note The contents of str is immediately copied to memory
   *       allocated by the subsystem instance, and can therefore be
   *       forgotten about after send() exits
   * TODO: Implement async transmit function, startSend with
   *       signaling to caller on completion
   * TODO: Add interface "ui" to send params to specify which
   *       interface
   */
  void send(char * str, uint16_t len);

  /**
   * @brief Inf loop that MUST be called within staticly created thread
   * TODO: Implement tx thread for async transmit
   */
  void runRxThread();

  // TODO: Make private
  /**
   * @note below are callbacks used for each interface (currently just
   *       one). They are static so that their signatures match those
   *       required by ChibiOS and are still encapsulated by the class.
   *       To gain access to interface states, static vars are used in
   *       the class, that are essentially encapsulated globals. These
   *       globals are necessary because the callbacks do not provide
   *       an arg for arbitrary data (like the virtual timer
   *       callbacks), which could otherwise have been used to pass a
   *       reference to a class instance.
   */
  // @brief This callback fires when an RX software buffer has been
  //        completely written
  static void rxDone(UARTDriver *uartp);

  // @brief This callback fires when a TX software buffer has been
  //        completely written to hardware interface buffers
  static void txEmpty(UARTDriver *uartp);

  // @brief Return pointer to the instance of self associated with the
  //        passed driver
  // @note Since this is a subsystem, it probably shouldn't need or
  //       have multiple instances, but who knows (currently always
  //       returns the single subsys instance)
  static UartChSubsys *getDriversSubsys(UARTDriver *uartp);

  // Max length of messages in bytes. Anything longer than this many
  // bytes must be broken down into multiple messages
  static constexpr uint32_t kMaxMsgLen = 100;

 private:
  /*
   * @note To navigate the ChibiOS constraint of static UART callbacks
   *       with a predefined signature (no arbitrary params), and
   *       since this callbacks are provided a pointer to the
   *       UARTDriver struct, callbacks will be statically registered
   *       (and encapsulated) within this class's (hopefully private)
   *       members, allowing a lookup of the interface instance
   *       corresponding to the callback that was hit
   */
  // contains pointers to all registered UARTDrivers, their index in
  // this array corresponds to the index of their UartChSubsys pointer
  // in driverToSubsysLookup
  static std::array<UARTDriver *,4> registeredDrivers;

  // contains pointers to instances of this class, used for lookup of
  // members from static C-style callbacks
  static std::array<UartChSubsys *,4> driverToSubsysLookup;

  // UARTDriver UARTD3 specific fields (need to generalize)
  CircularBuffer<char> m_d3TxQueue{kMaxMsgLen*2};
  chibios_rt::Mutex m_d3TxQueueMut;
  bool m_d3IsReady = true;
  char m_txBuffer[UartChSubsys::kMaxMsgLen] = {};

  static constexpr uint8_t kUartOkMask = EVENT_MASK(1);
  static constexpr uint8_t kUartChMask = EVENT_MASK(4);
  uint16_t m_rxBuffer[11];
  UARTDriver *m_uartp;
  UARTConfig m_uartConfig;
  chibios_rt::Mutex m_uartMut;
  EventQueue& m_eventQueue;
};
