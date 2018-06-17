// Copyright (c) 2018 Formula Slug. All Rights Reserved.

#pragma once

#include <vector>
#include <array>
#include <stdint.h>
#include "Gpio.h"

class Event {
  public:
    // Event types
    enum Type {
      kNone,
      kCanRx,
      kTimerTimeout,
      kAdcConversion,
      kDigInTransition
    };

    Event(Type t, Gpio adcPin, uint32_t adcValue);
    Event(Type t, uint32_t canEid, std::array<uint16_t, 8> canFrame);
    Event(Type t, DigitalInput pin, bool currentState);
    Event();

    Type type();

    // type-specific member functions (see note in source)
    Gpio adcPin();
    uint32_t adcValue();
    uint32_t canEid();
    std::array<uint16_t, 8> canFrame();
    DigitalInput digInPin();
    bool digInState();

  private:
    Type m_type = kNone;

    /**
     * @note Allocation of 320 bytes (10 32 bit integers), of course,
     *       overflows a 128 byte static thread workspace and breaks
     *       the kernel. Be careful with static thread workspace
     */
    std::array<uint16_t, 10> m_params;
};
