// Copyright (c) 2018 Formula Slug. All Rights Reserved.

#pragma once

#include <mutex>
#include <vector>
#include <stdint.h>
#include "hal.h"
#include "ch.h"
#include "Event.h"
#include "EventQueue.h"

class AdcChSubsys {
  private:
    EventQueue& m_eventQueue;
  public:
    AdcChSubsys(EventQueue& eq);
};
