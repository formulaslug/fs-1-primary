// Copyright (c) 2018 Formula Slug. All Rights Reserved.

#pragma once

#include <stdint.h>

#include <vector>

#include "chibios-subsys/common/Event.h"
#include "chibios-subsys/common/EventQueue.h"
#include "chibios-subsys/common/Gpio.h"
#include "ch.h"
#include "hal.h"

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
  explicit AdcChSubsys(EventQueue& eq);
  ~AdcChSubsys();

  /**
   * Add a pin to the subsystem and start generating conversion events
   * immediately
   * @param pin Gpio pin (port and pin number)
   * @param samplingFrequency sampling frequency of pin in Hz
   */
  bool addPin(Gpio pin);

  bool removePin(Gpio pin);

  /**
   * ADC subsystem run function
   * @note MUST be called from within a chibios static thread. The run
   *       function can be called any time after instantiation of the
   *       subsystem, but conversion events will only be generated
   *       once the run function is called.
   */
  void runThread();

 private:
  static constexpr uint32_t kNumGpio = 2;
  static constexpr uint32_t kMaxNumGpio = 4;
  static constexpr uint32_t kSampleBuffDepth = 8;

  // states of all available analog pins
  bool m_pins[kMaxNumGpio] = {false, false, false, false};
  uint8_t m_numPins = 0;
  bool m_subsysActive = false;

  EventQueue& m_eventQueue;
  ADCConversionGroup m_adcConversionGroup;

  // @note Size in test hal is ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH
  adcsample_t m_samples[kNumGpio * kSampleBuffDepth];

  // TODO: Implement sampling frequency per-pin. Simple solution is
  //       just a subsystem tick that meets sampling requirements of
  //       the fastest sampled pin
  uint32_t m_sampleClkMs = 15;

  void start();
  void stop();

  /**
   * @note Maps for constant-time lookup of attrs of the analog input
   */

  // map user's Gpio pin to a pin integer
  int32_t kPinMap[kMaxNumGpio] = {1, 2, 3, 6};

  // map user's Gpio pin to chibios port pointer
  stm32_gpio_t* kPortMap[kMaxNumGpio] = {GPIOA, GPIOA, GPIOA, GPIOA};

  // map user's Gpio pin to a sample time macro from chibios
  uint32_t kSampleChargeTimeMap[kMaxNumGpio] = {
      ADC_SMPR2_SMP_AN1(ADC_SAMPLE_480), ADC_SMPR2_SMP_AN2(ADC_SAMPLE_480),
      ADC_SMPR2_SMP_AN3(ADC_SAMPLE_480), ADC_SMPR2_SMP_AN4(ADC_SAMPLE_480)};

  /**
   *
   * Map user's Gpio pin to a conversion sequence number macro from
   *
   * @note Each added pin will have one slot in the conversion sequence
   * @note Channel numbers map directly to the pins and which ADC
   *       channels they can be read from
   */
  uint32_t kConversionSequenceMap[kMaxNumGpio] = {
      ADC_SQR3_SQ1_N(ADC_CHANNEL_IN1), ADC_SQR3_SQ2_N(ADC_CHANNEL_IN2),
      ADC_SQR3_SQ3_N(ADC_CHANNEL_IN3), ADC_SQR3_SQ4_N(ADC_CHANNEL_IN6)};
};
