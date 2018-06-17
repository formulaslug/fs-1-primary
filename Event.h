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
      kAdcConversion
    };

    Event(Type t, Gpio adcPin, uint32_t adcValue);
    Event(Type t, uint32_t canEid, std::array<uint32_t, 8> canFrame);
    Event();

    Type type();

    // type-specific member functions (see note in source)
    Gpio adcPin();
    uint32_t adcValue();
    uint32_t canEid();
    std::array<uint32_t, 8> canFrame();


  private:
    Type m_type = kNone;

    // TODO: implement a better mechanism for storing abitrary data
    //       in event param. Should probably statically allocate event
    //       param memory, then read/write in polymorphic manner
    std::array<uint32_t, 10> m_params;
};
