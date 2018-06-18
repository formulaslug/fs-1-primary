// Copyright (c) 2018 Formula Slug. All Rights Reserved.

#include "Event.h"

#include <array>

#include "Gpio.h"

Event::Event() {}

// type-based constructors
Event::Event(Type t, Gpio adcPin, uint32_t adcValue) : m_type(t) {
  m_params[0] = static_cast<uint32_t>(adcPin);
  m_params[1] = adcValue;
}
Event::Event(Type t, uint32_t canEid, std::array<uint16_t, 8> canFrame)
    : m_type(t) {
  m_params[0] = canEid;

  // set data frame (max 8 bytes)
  std::copy(canFrame.begin(), canFrame.begin() + 8, m_params.begin() + 1);
}
Event::Event(Type t, DigitalInput pin, bool currentState) : m_type(t) {
  m_params[0] = static_cast<uint16_t>(pin);
  m_params[1] = static_cast<uint16_t>(currentState);
}

Event::Type Event::type() { return m_type; }

/**
 * @note Due to a terrible, terrible machine, for which there's no
 *       time to upgrade the toolchain, the toolchain for
 *       (no std::variant), the following member function
 *       implementation are, essentially, supporting c-style
 *       polymorphism
 */

// ADC event member functions
Gpio Event::adcPin() { return static_cast<Gpio>(m_params[0]); }

uint32_t Event::adcValue() { return m_params[1]; }

// CAN event member functions
uint32_t Event::canEid() { return m_params[0]; }

std::array<uint16_t, 8> Event::canFrame() {
  std::array<uint16_t, 8> frame = {0, 0, 0, 0, 0, 0, 0, 0};

  std::copy(m_params.begin() + 1, m_params.begin() + 1 + 8, frame.begin());

  return frame;
}

// Digital Input event member functions
DigitalInput Event::digInPin() {
  return static_cast<DigitalInput>(m_params[0]);
}

bool Event::digInState() { return static_cast<bool>(m_params[1]); }
