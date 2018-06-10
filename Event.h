// Copyright (c) 2018 Formula Slug. All Rights Reserved.

#pragma once

#include <vector>
#include <stdint.h>

class Event {
  public:
    // Event types
    enum Type {
      kNone,
      kCanRx,
      kTimerTimeout
    };

    Type type = kNone;
    // TODO: implement a better mechanism for storing abitrary data
    //       in event param. Should probably statically allocate event
    //       param memory, then read/write in polymorphic manner
    std::vector<int32_t> params;

    Event();
    Event(Type t, std::vector<int32_t> p);
};
