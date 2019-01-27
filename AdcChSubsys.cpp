// Copyright (c) 2018 Formula Slug. All Rights Reserved.

#include "AdcChSubsys.h"

#include "chibios-subsys/common/Event.h"
#include "ch.hpp"
#include "hal.h"
#include "pinconf.h"

// TODO: Test system with all four ADC inputs
// BRAKE_VALUE_PIN -> ADC123_IN1 (POT 2)
//
// RIGHT_THROTTLE_PIN -> ADC123_IN2 (POT 1)
//
// LEFT_THROTTLE_PIN -> ADC123_IN3 (POT 1)
//
// STEERING_VALUE_PIN -> ADC12_IN6 (POT 1)

AdcChSubsys::AdcChSubsys(EventQueue& eq) : m_eventQueue(eq) {
  // update the conversion group configuration
  m_adcConversionGroup = {
      FALSE, 0, NULL, NULL, 0, /* CR1 */
      ADC_CR2_SWSTART,         /* CR2 */
      // SMPR1: Samples times for channels 10-17
      0,
      // SMPR2: Samples times for channels 0-9
      // @note ADC_SAMPLE_[X] is in cycles of the ADC's clock
      // @note ADC_SMPR2_SMP_AN[X] corresponds to ADC_CHANNEL_IN[X]
      // ADC_SMPR2_SMP_AN1(ADC_SAMPLE_480) | ADC_SMPR2_SMP_AN2(ADC_SAMPLE_480),
      0,  // ADC_SMPR2_SMP_AN1(ADC_SAMPLE_480) |
          // ADC_SMPR2_SMP_AN2(ADC_SAMPLE_480),
      0,  /* SQR1 */
      0,  /* SQR2 */
      // SQR1: Conversion group sequence 1-6
      // @brief specify which channels, in which order, are sampled per
      //        conversion sequence
      // @note Use ADC_SQR3_SQ[X]_N to indicate sequence number of channel
      //       ADC_CHANNEL_IN[Y]
      0  // ADC_SQR3_SQ1_N(ADC_CHANNEL_IN1) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN2)
  };

  /**
   *
   * Start ADC hardware subsystems
   *
   * @note ChibiOS ADC subsystems will not consume any processing
   *       until conversion are actually request, which only occurs
   *       once at least one pin is added to the subsystem
   * TODO: only start those that are required for the given pins
   */
  adcStart(&ADCD1, NULL);
  adcStart(&ADCD2, NULL);
  adcStart(&ADCD3, NULL);
}

/**
 * TODO: Implement sampling frequency input per-pin
 */
bool AdcChSubsys::addPin(Gpio pin) {
  // fail if pin already added
  if (m_pins[static_cast<uint32_t>(pin)]) return false;

  // stop the subsystem to synchronize configuration with sampling
  // thread (run function)
  stop();

  // set the pin mode
  palSetPadMode(kPortMap[static_cast<uint32_t>(pin)],
                kPinMap[static_cast<uint32_t>(pin)], PAL_MODE_INPUT_ANALOG);

  // update the conversion group configuration
  // increment the number of channels (2nd member of struct)
  m_adcConversionGroup.num_channels += 1;
  // add sample charge time config (8th member of struct)
  m_adcConversionGroup.smpr2 |=
      kSampleChargeTimeMap[static_cast<uint32_t>(pin)];
  // add conversion sequence config (11th member of struct)
  m_adcConversionGroup.sqr3 |=
      kConversionSequenceMap[static_cast<uint32_t>(pin)];

  // add the pin internally
  m_pins[static_cast<uint32_t>(pin)] = true;
  m_numPins++;

  // start the subsystem again with new configuration
  start();

  // return success
  return true;
}

void AdcChSubsys::runThread() {
  while (true) {
    if (m_subsysActive) {
      // Make the conversion
      // @note Hard-coded to ADC Driver 1 only
      adcConvert(&ADCD1, &m_adcConversionGroup, m_samples, kSampleBuffDepth);

      // package and post event
      std::vector<Event> events;
      events.push_back(
          Event(Event::Type::kAdcConversion, Gpio::kA1, 0xfff & m_samples[0]));
      events.push_back(
          Event(Event::Type::kAdcConversion, Gpio::kA2, 0xfff & m_samples[1]));
      m_eventQueue.push(events);
    }
    // sleep to yield processing until next sample of channels
    chThdSleepMilliseconds(m_sampleClkMs);
  }
}

bool AdcChSubsys::removePin(Gpio pin) {
  // return failure if pin wasn't added
  if (!m_pins[static_cast<uint32_t>(pin)]) return false;

  // stop the interface momentarily
  stop();

  // remove the pin internally
  m_pins[static_cast<uint32_t>(pin)] = false;
  m_numPins--;

  // update the conversion group configuration
  // increment the number of channels (2nd member of struct)
  m_adcConversionGroup.num_channels -= 1;
  // add sample charge time config (8th member of struct)
  m_adcConversionGroup.smpr2 ^=
      kSampleChargeTimeMap[static_cast<uint32_t>(pin)];
  // add conversion sequence config (11th member of struct)
  m_adcConversionGroup.sqr3 ^=
      kConversionSequenceMap[static_cast<uint32_t>(pin)];

  // start the interface with the new configuration (if at least
  // one pin is still registered in the subsystem)
  if (m_numPins > 0) start();

  // return success
  return false;
}

// TODO: implement full subsystem, then implement pin adding
void AdcChSubsys::start() { m_subsysActive = true; }

/**
 * @note Subsystem should be stopped before any configurations are
 *       changed. Otherwise, partial configuration will temporarily
 *       corrupt conversion events send to the user's event queue
 */
void AdcChSubsys::stop() {
  // TODO: tear down the subystem
  m_subsysActive = false;
}
