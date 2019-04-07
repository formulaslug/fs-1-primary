// Copyright (c) 2018 Formula Slug. All Rights Reserved.

#pragma once

#include <stdint.h>

#include <vector>

#include "Event.h"
#include "EventQueue.h"
#include "Gpio.h"
#include "ch.h"
#include "hal.h"
#include "mcuconfFs.h"

/**
 *
 * TODO: Rewrite subsystem using the EXT external interrupt driver
 *
 */

/**
 *
 * Singleton class for an ADC subsystem, sitting on top of ChibiOS's
 * low-level ADC driver
 *
 * @note Requires that the runThread() public member function is called
 *       within the context of a chibios static thread
 */
class DigInChSubsys {
 public:
  explicit DigInChSubsys(EventQueue& eq);

  /**
   * Add a pin to the subsystem and start generating transition events
   * immediately
   * @param pin Gpio pin (port and pin number)
   */
  bool addPin(DigitalInput pin);

  bool removePin(DigitalInput pin);

  bool getSavedState(DigitalInput p);

  /**
   * Digital input subsystem run function
   * @note MUST be called from within a chibios static thread. The run
   *       function can be called any time after instantiation of the
   *       subsystem, but conversion events will only be generated
   *       once the run function is called.
   */
  void runThread();

 private:
  static constexpr uint16_t kMaxNumPins = 1;
  uint16_t m_numPins = 0;

  // @note true if pin is registered, false otherwise
  std::array<bool, kMaxNumPins> m_pins = {};

  // @note true is HIGH, false is LOW
  std::array<bool, kMaxNumPins> m_pinStates = {};

  bool m_subsysActive = false;

  EventQueue& m_eventQueue;

  // TODO: Implement sampling frequency per-pin. Simple solution is
  //       just a subsystem tick that meets sampling requirements of
  //       the fastest sampled pin
  uint32_t m_sampleClkMs = 25;

  void start();
  void stop();

  bool registered(DigitalInput p);

  bool getState(DigitalInput p);

  /**
   * @note Maps for constant-time lookup of attrs of the analog input
   */
  // map user's Gpio pin to a pin integer
  uint32_t kPinMap[kMaxNumPins] = {TRI_STATE_SWITCH_UP_PIN};

  // map user's Gpio pin to chibios port pointer
  stm32_gpio_t* kPortMap[kMaxNumPins] = {TRI_STATE_SWITCH_UP_PORT};

  uint32_t getPinNum(DigitalInput p);

  stm32_gpio_t* getPort(DigitalInput p);
};
