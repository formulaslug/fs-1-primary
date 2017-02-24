// Copyright (c) Formula Slug 2016-2017. All Rights Reserved.

#include "Vehicle.h"

Vehicle::Vehicle() {
  for (auto& led : ledStates) {
    led = kLEDOff;
  }
  ledStates[kStatusLED] = kOn;
}
