/*----------------------------------------------------------------------------*/
/* Copyright (c) 2015-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <vector>

#include <wpi/ArrayRef.h>
#include <wpi/STLExtras.h>

#include "circular_buffer.h"

namespace frc {

/**
 * This class implements a linear, digital filter. All types of FIR and IIR
 * filters are supported. Static factory methods are provided to create commonly
 * used types of filters.
 *
 * Filters are of the form:<br>
 *  y[n] = (b0 * x[n] + b1 * x[n-1] + … + bP * x[n-P]) -
 *         (a0 * y[n-1] + a2 * y[n-2] + … + aQ * y[n-Q])
 *
 * Where:<br>
 *  y[n] is the output at time "n"<br>
 *  x[n] is the input at time "n"<br>
 *  y[n-1] is the output from the LAST time step ("n-1")<br>
 *  x[n-1] is the input from the LAST time step ("n-1")<br>
 *  b0 … bP are the "feedforward" (FIR) gains<br>
 *  a0 … aQ are the "feedback" (IIR) gains<br>
 * IMPORTANT! Note the "-" sign in front of the feedback term! This is a common
 *            convention in signal processing.
 *
 * What can linear filters do? Basically, they can filter, or diminish, the
 * effects of undesirable input frequencies. High frequencies, or rapid changes,
 * can be indicative of sensor noise or be otherwise undesirable. A "low pass"
 * filter smooths out the signal, reducing the impact of these high frequency
 * components.  Likewise, a "high pass" filter gets rid of slow-moving signal
 * components, letting you detect large changes more easily.
 *
 * Example FRC applications of filters:
 *  - Getting rid of noise from an analog sensor input (note: the roboRIO's FPGA
 *    can do this faster in hardware)
 *  - Smoothing out joystick input to prevent the wheels from slipping or the
 *    robot from tipping
 *  - Smoothing motor commands so that unnecessary strain isn't put on
 *    electrical or mechanical components
 *  - If you use clever gains, you can make a PID controller out of this class!
 *
 * For more on filters, I highly recommend the following articles:<br>
 *  http://en.wikipedia.org/wiki/Linear_filter<br>
 *  http://en.wikipedia.org/wiki/Iir_filter<br>
 *  http://en.wikipedia.org/wiki/Fir_filter<br>
 *
 * Note 1: Get() should be called by the user on a known, regular period. You
 * can set up a Notifier to do this (look at the WPILib PIDController class), or
 * do it "inline" with code in a periodic function.
 *
 * Note 2: For ALL filters, gains are necessarily a function of frequency. If
 * you make a filter that works well for you at, say, 100Hz, you will most
 * definitely need to adjust the gains if you then want to run it at 200Hz!
 * Combining this with Note 1 - the impetus is on YOU as a developer to make
 * sure Get() gets called at the desired, constant frequency!
 */
class LinearFilter {
 public:
  /**
   * Create a linear FIR or IIR filter.
   *
   * @param source  The function that is used to get values
   * @param ffGains The "feed forward" or FIR gains
   * @param fbGains The "feed back" or IIR gains
   */
  LinearFilter(wpi::function_ref<double()> source,
               wpi::ArrayRef<double> ffGains, wpi::ArrayRef<double> fbGains);

  virtual ~LinearFilter() = default;

  // Static methods to create commonly used filters
  /**
   * Creates a one-pole IIR low-pass filter of the form:<br>
   *   y[n] = (1 - gain) * x[n] + gain * y[n-1]<br>
   * where gain = e<sup>-dt / T</sup>, T is the time constant in seconds
   *
   * This filter is stable for time constants greater than zero.
   *
   * @param source        The function that is used to get values
   * @param timeConstant  The discrete-time time constant in seconds
   * @param period        The period in seconds between samples taken by the
   *                      user
   */
  static LinearFilter SinglePoleIIR(wpi::function_ref<double()> source,
                                    double timeConstant, double period);

  /**
   * Creates a first-order high-pass filter of the form:<br>
   *   y[n] = gain * x[n] + (-gain) * x[n-1] + gain * y[n-1]<br>
   * where gain = e<sup>-dt / T</sup>, T is the time constant in seconds
   *
   * This filter is stable for time constants greater than zero.
   *
   * @param source       The function that is used to get values
   * @param timeConstant The discrete-time time constant in seconds
   * @param period       The period in seconds between samples taken by the user
   */
  static LinearFilter HighPass(wpi::function_ref<double()> source,
                               double timeConstant, double period);

  /**
   * Creates a K-tap FIR moving average filter of the form:<br>
   *   y[n] = 1/k * (x[k] + x[k-1] + … + x[0])
   *
   * This filter is always stable.
   *
   * @param source The function that is used to get values
   * @param taps   The number of samples to average over. Higher = smoother but
   *               slower
   */
  static LinearFilter MovingAverage(wpi::function_ref<double()> source,
                                    int taps);

  /**
   * Creates a K-tap LMS filter of the form:<br>
   *   y[n] = 1/k * x[n] + (-1/k) * x[n-k] + y[n-1]
   *
   * @param source  The function that is used to get values
   * @param inputs  The number of inputs to the filter
   * @param outputs The number of outputs to the filter
   */
  static LinearFilter LMS(wpi::function_ref<double()> source, int taps);

  /**
   * Reset the filter state.
   */
  void Reset();

  /**
   * Calculates the next value of the filter.
   *
   * @return The filtered value at this step
   */
  double Get();

 private:
  wpi::function_ref<double()> m_source;
  circular_buffer<double> m_inputs{0};
  circular_buffer<double> m_outputs{0};
  std::vector<double> m_inputGains;
  std::vector<double> m_outputGains;
};

}  // namespace frc
