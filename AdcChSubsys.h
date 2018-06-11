// Copyright (c) 2018 Formula Slug. All Rights Reserved.

#pragma once

#include <vector>
#include <stdint.h>
#include "hal.h"
#include "ch.h"
#include "Event.h"
#include "EventQueue.h"

/**
 *
 * Singleton class for an ADC subsystem, sitting on top of ChibiOS's
 * low-level ADC driver
 *
 * @note Requires that the runThread() public member function is called
 *       within the context of a chibios static thread
 */
class AdcChSubsys {
 public:
  enum class Gpio {
    kA1,
    kA2,
    kA3,
    kA6
  };

  explicit AdcChSubsys(EventQueue& eq);
  ~AdcChSubsys();

  /**
   * Add a pin to the subsystem and start generating conversion events
   * immediately
   * @param pin Gpio pin (port and pin number)
   * @param samplingFrequency sampling frequency of pin in Hz
   */
  bool addPin(AdcChSubsys::Gpio pin, uint32_t samplingFrequency);

  bool removePin(AdcChSubsys::Gpio pin);

  /**
   * ADC subsystem run function
   * @note MUST be called from within a chibios static thread. The run
   *       function can be called any time after instantiation of the
   *       subsystem, but conversion events will only be generated
   *       once the run function is called.
   */
  void runThread();

  /**
   * What SHOULD be private (need to figure out callback)
   */
  static constexpr uint32_t kNumGpio = 4;
  static constexpr uint32_t kSampleBuffDepth = 8;
  EventQueue& m_eventQueue;
  ADCConversionGroup m_adcConversionGroup;
  // NOTE: size in test hal is ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH
  adcsample_t m_samples[kNumGpio * kSampleBuffDepth];
  // runs at the fastest sampling frequency of all analog inputs
  float m_sampleClkHz = 100; // default of 100Hz
  uint32_t m_sampleClkMs = 1;
  systime_t samplePeriodCycles();
  // uint32_t samplePeriodCycles();

  bool m_ledOn = false;

 private:
  // states of all available analog pins
  bool m_pins[kNumGpio] = { false, false, false, false };
  uint8_t m_numPins = 0;
  bool m_subsysActive = false;


  uint32_t sampleClkUs();
  uint32_t sampleClkMs();
  uint32_t sampleClkS();
  // static void sampleClkCallback(void *p);

  void restart();
  void start();
  void stop();

  // chibios callbacks
  // ...

  /**
   * @note Maps for constant-time lookup of attrs of the analog input
   */

  // map user's Gpio pin to a pin integer
  static constexpr int32_t kPinMap[] = { 1, 2, 3, 6 };

  // map user's Gpio pin to a chibios channel number
  static constexpr int32_t kChannelMap[] = {
    ADC_CHANNEL_IN1,
    ADC_CHANNEL_IN2,
    ADC_CHANNEL_IN3,
    ADC_CHANNEL_IN6
  };

  // map user's Gpio pin to chibios port pointer
  static constexpr stm32_gpio_t* kPortMap[] = {
    GPIOA,
    GPIOA,
    GPIOA,
    GPIOA
  };
};
