#include <mutex>
#include "ch.h"
#include "hal.h"
#include "mcuconfFs.h"
#include "CanBus.h"
#include "Event.h"
#include "EventQueue.h"
#include "CanChSubsys.h"

CanChSubsys::CanChSubsys(CanBus& cb, chibios_rt::Mutex& cbMut,
    EventQueue& eq) : m_canBus(cb), m_canBusMut(cbMut),
  m_eventQueue(eq) {}

void CanChSubsys::startSend(CANTxFrame& msg) {
  std::lock_guard<chibios_rt::Mutex> lock(m_canBusMut);
  m_canBus.queueTxMessage(msg);
}

/*
 * @brief subsystem run function for CAN TX called from within a
 *        ChibiOS static thread
 */
void CanChSubsys::runTxThread() {
  while (true) {
    {
      // Lock from simultaneous thread access
      std::lock_guard<chibios_rt::Mutex> lock(m_canBusMut);
      // Process all messages to transmit from the message transmission queue
      m_canBus.processTxMessages();
    }
    // throttle back thread runloop to prevent overconsumption of resources
    chThdSleepMilliseconds(10);
  }
}

/*
 * @brief subsystem run function for CAN TX called from within a
 *        ChibiOS static thread
 */
void CanChSubsys::runRxThread() {
  event_listener_t el;
  chEvtRegister(&(m_canBus.m_canp->rxfull_event), &el, 0);

  while (true) {
    if (chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(100)) == 0) {
      continue;
    }
    // receive any present messages from hardware
    {
      std::lock_guard<chibios_rt::Mutex> lock(m_canBusMut);
      m_canBus.processRxMessages();
    }
    // generate events from any received messages
    CANRxFrame msg;
    while (m_canBus.rxQueueSize() > 0) {
      // get CAN message
      msg = m_canBus.dequeueRxMessage();
      // create event
      std::array<int32_t, 8> frame;
      // Event e = Event();
      // write data bytes to event params
      // TODO: just switch to c-style iteration, since using an array now
      while (msg.DLC > 0) {
        // pushing data bytes to event param vector in reverse order,
        // such that popping off stack will result in correct order
        // e.params.push_back(msg.data8[msg.DLC - 1]);
        frame[msg.DLC - 1] = msg.data8[msg.DLC - 1];
        msg.DLC--;
      }
      // write COBID to event params
      // e.params.push_back(msg.EID);
      palSetPad(STARTUP_LED_PORT, STARTUP_LED_PIN);
      Event e = Event(Event::Type::kCanRx, msg.EID, frame);
      // push event
      // m_eventQueue.push(e);
      m_eventQueue.push(e);
    }
  }

  chEvtUnregister(&(m_canBus.m_canp->rxfull_event), &el);
}
