// Copyright (c) 2018 Formula Slug. All Rights Reserved.

#pragma once

#include <stdint.h>

#include <mutex>
#include <vector>

#include "CanBus.h"
#include "Event.h"
#include "EventQueue.h"
#include "ch.h"
#include "hal.h"

/**
 *
 * CAN subsystem, sitting on top of the chibios CAN driver. Multiple
 * subsystems can be operating to handle multiple CAN interfaces
 *
 * @note Requires that runTxThread() and runRxThread() public member
 *       functions are called within chibios static threads
 */
class CanChSubsys {
 private:
  CanBus& m_canBus;
  chibios_rt::Mutex& m_canBusMut;
  EventQueue& m_eventQueue;

 public:
  // @param canBus Reference to canBus containing bus configuration
  //        and lower-level tx/rx utilities
  // @param eventQueue Reference to queue to send this subsystem's
  //        events to. The event queue notifies itself.
  CanChSubsys(CanBus& cb, chibios_rt::Mutex& cbMut, EventQueue& eq);

  // @brief Queue a TX CAN Frame for transmission
  void startSend(CANTxFrame& msg);

  // @brief Inf loop that MUST be called within created static thread
  void runTxThread();
  void runRxThread();
};
