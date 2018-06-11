#include "AdcChSubsys.h"
#include "hal.h"
#include "ch.hpp"
#include "Event.h"
#include "mcuconfFs.h"

// static virtual_timer_t vSampleClk;
static const ADCConversionGroup g_adcConversionGroup = {
  FALSE,
  2,
  NULL,
  NULL,
  0,                        /* CR1 */
  ADC_CR2_SWSTART,          /* CR2 */
  // SMPR1: Samples times for channels 10-17
  0,
  // SMPR2: Samples times for channels 0-9
  // @note ADC_SAMPLE_[X] is in cycles of the ADC's clock
  // @note ADC_SMPR2_SMP_AN[X] corresponds to ADC_CHANNEL_IN[X]
  // ADC_SMPR2_SMP_AN1(ADC_SAMPLE_480) | ADC_SMPR2_SMP_AN2(ADC_SAMPLE_480),
  ADC_SMPR2_SMP_AN1(ADC_SAMPLE_3) | ADC_SMPR2_SMP_AN2(ADC_SAMPLE_3),
  0,                        /* SQR1 */
  0,                        /* SQR2 */
  // SQR1: Conversion group sequence 1-6
  // @brief specify which channels, in which order, are sampled per
  //        conversion sequence
  // @note Use ADC_SQR3_SQ[X]_N to indicate sequence number of channel
  //       ADC_CHANNEL_IN[Y]
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN1) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN2)
  // // use a circular buffer
  // false,
  // // num channels
  // 1,
  // // end conversion callback
  // NULL,
  // // error callback
  // NULL,
  // // CR1 register config
  // 0,
  // // CR2 register config
  // ADC_CR2_SWSTART,
  // // SMPR1: Samples times for channels 10-17
  // 0,
  // // SMPR2: Samples times for channels 0-9
  // // @note ADC_SAMPLE_[X] is in cycles of the ADC's clock
  // // @note ADC_SMPR2_SMP_AN[X] corresponds to ADC_CHANNEL_IN[X]
  // ADC_SMPR2_SMP_AN1(ADC_SAMPLE_480),// | ADC_SMPR2_SMP_AN2(ADC_SAMPLE_480),
  // // SQR1 register config
  // 0,
  // // SQR2
  // 0,
  // // SQR1: Conversion group sequence 1-6
  // // @brief specify which channels, in which order, are sampled per
  // //        conversion sequence
  // // @note Use ADC_SQR3_SQ[X]_N to indicate sequence number of channel
  // //       ADC_CHANNEL_IN[Y]
  // ADC_SQR3_SQ1_N(ADC_CHANNEL_IN1)// | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN2)
};
static adcsample_t g_samples[2 * 8];

// TODO: Test system with all four ADC inputs
// BRAKE_VALUE_PIN -> ADC123_IN1 (POT 2)
//
// RIGHT_THROTTLE_PIN -> ADC123_IN2 (POT 1)
//
// LEFT_THROTTLE_PIN -> ADC123_IN3 (POT 1)
//
// STEERING_VALUE_PIN -> ADC12_IN6 (POT 1)

AdcChSubsys::AdcChSubsys(EventQueue& eq) : m_eventQueue(eq) {
}

// @NOTE Removed in exchange for thread that can run blocking code
// void sampleClkCallback(void *p) {
//   // get reference to self
//   auto _this = static_cast<AdcChSubsys*>(p);
//
//   if (_this->m_ledOn) {
//     palClearPad(STARTUP_LED_PORT, STARTUP_LED_PIN);
//     _this->m_ledOn = false;
//   } else {
//     palSetPad(STARTUP_LED_PORT, STARTUP_LED_PIN);
//     _this->m_ledOn = true;
//   }
//
//   // Make the conversion
//   // NOTE: Hard-coded to ADC Driver 1 only
//   // adcConvert(&ADCD1, &(_this->m_adcConversionGroup), _this->m_samples,
//   //     _this->kSampleBuffDepth);
//   // adcConvert(&ADCD1, &g_adcConversionGroup, g_samples,
//   //     _this->kSampleBuffDepth);
//   adcConvert(&ADCD1, &g_adcConversionGroup, g_samples, 8);
//
//   // // post event
//   // Event e = Event();
//   // e.type = Event::kAdcConversion;
//   // e.params.push_back(_this->m_samples[0]);
//   // _this->m_eventQueue.push(e);
//
//   // restart virtual timer
//   chSysLockFromISR();
//   chVTSetI(&vSampleClk, _this->samplePeriodCycles(), sampleClkCallback, p);
//   chSysUnlockFromISR();
// }

bool AdcChSubsys::addPin(AdcChSubsys::Gpio pin,
    uint32_t samplingFrequency) {
  // fail if pin already added
  if (m_pins[static_cast<uint32_t>(pin)]) return false;

  // set the pin mode
  palSetPadMode(kPortMap[static_cast<uint32_t>(pin)],
      kPinMap[static_cast<uint32_t>(pin)], PAL_MODE_INPUT_ANALOG);

  // add the pin internally
  m_pins[static_cast<uint32_t>(pin)] = true;
  m_numPins++;

  // start the subsystem
  if (m_subsysActive) {
    restart();
  } else {
    start();
  }

  // return success
  return true;
}

void AdcChSubsys::runThread() {
  while (true) {
    if (m_ledOn) {
      palClearPad(STARTUP_LED_PORT, STARTUP_LED_PIN);
      m_ledOn = false;
    } else {
      palSetPad(STARTUP_LED_PORT, STARTUP_LED_PIN);
      m_ledOn = true;
    }

    // Make the conversion
    // NOTE: Hard-coded to ADC Driver 1 only
    // adcConvert(&ADCD1, &(_this->m_adcConversionGroup), _this->m_samples,
    //     _this->kSampleBuffDepth);
    // adcConvert(&ADCD1, &g_adcConversionGroup, g_samples,
    //     _this->kSampleBuffDepth);
    // !!!This one
    adcConvert(&ADCD1, &g_adcConversionGroup, g_samples, 8);

    // post event
    Event e = Event();
    e.type = Event::kAdcConversion;
    e.params.push_back(g_samples[0]);
    m_eventQueue.push(e);

    // restart timer
    chThdSleepMilliseconds(m_sampleClkMs);
  }
}

// TODO: implement pin tear down (include safety check)
bool AdcChSubsys::removePin(AdcChSubsys::Gpio pin) {
  // remove the pin internall
  m_pins[static_cast<uint32_t>(pin)] = false;
  m_numPins--;

  return false;
}

// TODO: implement full subsystem, then implement pin adding
void AdcChSubsys::start() {
  // initialize conversion group config
  // m_adcConversionGroup = {
  // g_adcConversionGroup = {
  //   // use a circular buffer
  //   false,
  //   // num channels
  //   m_numPins,
  //   // end conversion callback
  //   NULL,
  //   // error callback
  //   NULL,
  //   // CR1 register config
  //   0,
  //   // CR2 register config
  //   ADC_CR2_SWSTART,
  //   // SMPR1: Samples times for channels 10-17
  //   0,
  //   // SMPR2: Samples times for channels 0-9
  //   // @note ADC_SAMPLE_[X] is in cycles of the ADC's clock
  //   // @note ADC_SMPR2_SMP_AN[X] corresponds to ADC_CHANNEL_IN[X]
  //   ADC_SMPR2_SMP_AN1(ADC_SAMPLE_480),// | ADC_SMPR2_SMP_AN2(ADC_SAMPLE_480),
  //   // SQR1 register config
  //   0,
  //   // SQR2
  //   0,
  //   // SQR1: Conversion group sequence 1-6
  //   // @brief specify which channels, in which order, are sampled per
  //   //        conversion sequence
  //   // @note Use ADC_SQR3_SQ[X]_N to indicate sequence number of channel
  //   //       ADC_CHANNEL_IN[Y]
  //   ADC_SQR3_SQ1_N(ADC_CHANNEL_IN1)// | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN2)
  // };

  // start ADC
  // TODO: only start those that are required for given pins
  adcStart(&ADCD1, NULL);
  adcStart(&ADCD2, NULL);
  adcStart(&ADCD3, NULL);

  // @NOTE: dropped virtual timer for thread that sleeps in order to
  //        run blocking ADC conversion
  // start the virtual timer
  // chVTSet(&vSampleClk, samplePeriodCycles(), sampleClkCallback, this);
  // chVTSet(&m_vSampleClk, samplePeriodCycles(), AdcChSubsys::sampleClkCallback, this);

  m_subsysActive = true;
}

void AdcChSubsys::stop() {
  // TODO: tear down the subystem
  m_subsysActive = false;
}

void AdcChSubsys::restart() {
  stop();
  start();
}

uint32_t AdcChSubsys::sampleClkUs() {
  return static_cast<uint32_t>(1000000.0 / m_sampleClkHz);
}

uint32_t AdcChSubsys::sampleClkMs() {
  return static_cast<uint32_t>(1000.0 / m_sampleClkHz);
}

uint32_t AdcChSubsys::sampleClkS() {
  return static_cast<uint32_t>(1.0 / m_sampleClkHz);
}

systime_t AdcChSubsys::samplePeriodCycles() {
  return TIME_MS2I(m_sampleClkMs);
  uint32_t us = sampleClkUs();
  if (us < 1 || us > 999) {
    uint32_t ms = sampleClkMs();
    if (ms < 1 || ms > 999) {
      uint32_t s = sampleClkS();
      return TIME_S2I(s);
    } else {
      return TIME_MS2I(ms);
    }
  } else {
    return TIME_US2I(us);
  }
}
