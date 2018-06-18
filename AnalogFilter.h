// Copyright (c) 2018 Formula Slug. All Rights Reserved.

#pragma once

#include <stdint.h>

#include <array>

#include "CircularBuffer.h"

/**
 *
 * Singleton class for an ADC subsystem, sitting on top of ChibiOS's
 * low-level ADC driver
 *
 * @note Requires that the runThread() public member function is called
 *       within the context of a chibios static thread
 */
class AnalogFilter {
 public:
  AnalogFilter();

  uint16_t filterLms(uint16_t sample);

 private:
  // TODO: implement with circular buffer abstraction
  // CircularBuffer<Event> m_samples{20};
  std::array<uint32_t, 11> m_inputs = {};
  std::array<uint32_t, 2> m_outputs = {};
  uint8_t m_currentIndex = 0;
};
