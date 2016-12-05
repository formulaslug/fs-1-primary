// Copyright (c) Formula Slug 2016. All Rights Reserved.

#pragma once

#include <stdint.h>

#include <array>

constexpr uint8_t kNumLEDs = 5;
constexpr uint8_t kNumButtons = 2;
static constexpr uint8_t kOn = 1;
static constexpr uint8_t kOff = 0;

// Operating on all 8 bits so that can be notted "~"
static constexpr uint8_t kLEDOn = 0xff;
static constexpr uint8_t kLEDOff = 0x00;

enum States {
  kLVStartup,
  kLVActive,

  kHVStartup,
  kHVActive,
  kHVShutdown,

  kRTDStartup,
  kRTDActive,
  kRTDShutdown
};

enum DriveProfiles {
  kSafe,       // restricted performace (pre-op)
  kEndurance,  // competition ready: endurance mode
  kSport       // competition ready: insane mode?
};

enum Leds { kBlue, kYellow, kRed, kStatusLED, kSpeed };

enum Buttons { kHVToggle, kRTDToggle };

enum AnalogInputs { kThrottleVoltage };

class Vehicle {
 public:
  Vehicle();

  uint8_t state = kLVStartup;

  struct Dynamics {
    uint16_t throttleVoltage = 1;
    uint16_t speed;
    uint8_t maxSpeed = 60;
    uint8_t driveProfile = kSafe;
  };

  Dynamics dynamics;

  // LED values are 0x00 or 0xff to allow for bitwise not
  std::array<uint8_t, kNumLEDs> ledStates;
};
