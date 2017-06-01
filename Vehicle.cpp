// Copyright (c) 2016-2017 Formula Slug. All Rights Reserved.

#include "Vehicle.h"

Vehicle::Vehicle() {
  for (auto& led : ledStates) {
    led = kLEDOff;
  }
  ledStates[kStatusLED] = kOn;
}
