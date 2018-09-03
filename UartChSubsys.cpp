// Copyright (c) 2018 Formula Slug. All Rights Reserved.

#include "UartChSubsys.h"

#include <mutex>
#include <cstring>

#include "Gpio.h"
#include "Event.h"
#include "EventQueue.h"
#include "ch.h"
#include "hal.h"
#include "mcuconfFs.h"

// Definitions for static members
std::array<UARTDriver *,4> UartChSubsys::registeredDrivers = {};
std::array<UartChSubsys *,4> UartChSubsys::driverToSubsysLookup = {};

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
    &UartChSubsys::txEmpty,   //txend1,// callback: transmission buffer completely read
                     // by the driver
    NULL,   //txend2,// callback: a transmission has physically completed
    &UartChSubsys::rxDone, // callback: a receive buffer has been
                                  //completelywritten
    NULL,   //rxchar,// callback: a character is received but the
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
  // register driver statically on class for lookup in static callbacks
  UartChSubsys::registeredDrivers[0] = m_uartp;
    //UartChSubsys::registeredDrivers.size()] = m_uartp;
  UartChSubsys::driverToSubsysLookup[0] = this;
      //UartChSubsys::driverToSubsysLookup.size()] = this;
  // start interface with default config
  uartStart(m_uartp, &m_uartConfig);
}

/*
 * TODO: Consider implementing a faster (but less flexible) version,
         with less impact on the calling thread, that requires the
         calling thread to "permanently" allocate mem for the passed
         str
 */
void UartChSubsys::send(char * str, uint16_t len) {
  // start data transmit if interface is ready
  if (m_d3IsReady) {
    m_d3IsReady = false;
    // copy string contents to class memory
    std::memcpy(m_txBuffer, str, len);
    // start transmit from the copied memory
    uartStopSend(m_uartp);
    uartStartSend(m_uartp, len, m_txBuffer);
  } else {
    // TODO: This function ideally doesn't affect timing in calling
    //       thread, having occasional copying of entire block will
    //       certainly effect timing
    // otherwise, queue for later
    std::lock_guard<chibios_rt::Mutex> txQueueGuard(m_d3TxQueueMut);
    for (uint32_t i = 0; i < len; i++) {
      // push byte
      m_d3TxQueue.PushBack(*(str + i));
    }
  }
}

/*
 * @brief subsystem run function for CAN RX called from within a
 *        ChibiOS static thread
 * TODO: Use this thread to generate RX msg events and anything else
 *       that can't always be performed in the static callbacks
 */
void UartChSubsys::runRxThread() {
  uartStopReceive(m_uartp);
  uartStartReceive(m_uartp, 1, m_rxBuffer);

  // sleep forever
  chThdSleepMilliseconds(1000*60*60*24);
}

UartChSubsys * UartChSubsys::getDriversSubsys(UARTDriver *uartp) {
  UartChSubsys *uartChSubsys = nullptr;

  for (uint32_t i = 0; i < UartChSubsys::registeredDrivers.size(); i++) {
    UARTDriver *_uartp = UartChSubsys::registeredDrivers[i];
    if (_uartp == uartp) {
      // found driver's subsys instance
      uartChSubsys = UartChSubsys::driverToSubsysLookup[i];
    }
  }

  return uartChSubsys;
}

void UartChSubsys::rxDone(UARTDriver *uartp) {
  // find class pointer corresponding to this uart driver via static
  // lookup table
  UartChSubsys *_this = UartChSubsys::getDriversSubsys(uartp);

  if (_this != nullptr) {
    // push byte-read event to the subsystem's event consumer
    // @TODO fix this limit in event generation, limiting RX bytes
    //       that can be processed by the FSM
    Event e = Event(Event::Type::kUartRx, _this->m_rxBuffer[0]);
    // if the event queue cannot be immediately acquired, the push fails
    bool success = _this->m_eventQueue.tryPush(e);
    if (!success) {
      // @TODO did not immediately acquire lock, must postpone pushing
      //       the event to the queue until after the callback exits
      //       (maybe should just always do this)
      palTogglePad(STARTUP_LED_PORT, STARTUP_LED_PIN);
    }
    // start next rx
    uartStopReceive(uartp);
    uartStartReceive(uartp, 1, _this->m_rxBuffer);
  } else {
    // ERROR
    // ...
  }
}

void UartChSubsys::txEmpty(UARTDriver *uartp) {
  // this is called every time the UART interface finishes copying
  // over bytes from a software-level tx buffer (the one passed to
  // the async start send function)

  UartChSubsys *_this = UartChSubsys::getDriversSubsys(uartp);

  // if the queue is non-empty, acquire, copy up to kMaxMsgLen of its
  // content to the output buffer and start a new transmission
  if (_this->m_d3TxQueue.Size() > 0) {
    // acquire lock guard in current scope

    // @TODO Is this lock really necessary? What's wrong with the size
    //       changing between the corresponding instructions
    //std::lock_guard<chibios_rt::Mutex> lock(_this->m_d3TxQueueMut);

    // determine num tx bytes to transmit, clamping to kMaxMsgLen
    uint32_t txCount = _this->m_d3TxQueue.Size() > kMaxMsgLen
      ? kMaxMsgLen : _this->m_d3TxQueue.Size();
    // copy over msg contents
    for (uint32_t i = 0; i < txCount; i++) {
      _this->m_txBuffer[i] = _this->m_d3TxQueue.PopFront();
    }
    // start the tx
    uartStopSend(uartp);
    uartStartSend(uartp, txCount, _this->m_txBuffer);
  } else {
    // otherwise, clear the interface ready flag
    _this->m_d3IsReady = true;
  }
}
