#include "AnalogFilter.h"

AnalogFilter::AnalogFilter() {
}


/**
 * TODO: Allow analog filter to pass a lambda as the final constructor
 *       param that executes the filter given the params for num input
 *       samples and num output samples (or something along those
 *       lines). OR, just make subclasses for each filter type with
 *       variable #taps.
 */
uint16_t AnalogFilter::filterLms(uint16_t sample) {
  // fetch new input
  m_inputs[m_currentIndex] = sample;

  // shift outputs
  m_outputs[0] = m_outputs[1];

  // apply filter
  // TODO: try changing the divisions to floats then cast to int at the
  //       end
  // y(n) = y(n-1) + x(n)/N - x(n-N)/N
  uint8_t nextIndex = (m_currentIndex + 1) % 11;
  m_outputs[1] = m_outputs[0] + m_inputs[m_currentIndex]/10 - m_inputs[nextIndex]/10;

  // set new index
  m_currentIndex = nextIndex;

  // return the latest filter output
  return m_outputs[1];
}
