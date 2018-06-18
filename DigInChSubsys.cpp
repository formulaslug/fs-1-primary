// Copyright (c) 2018 Formula Slug. All Rights Reserved.

#include "DigInChSubsys.h"

#include "Event.h"
#include "ch.hpp"
#include "hal.h"
#include "mcuconfFs.h"

DigInChSubsys::DigInChSubsys(EventQueue& eq) : m_eventQueue(eq) {}

/**
 * TODO: Implement sampling frequency input per-pin
 */
bool DigInChSubsys::addPin(DigitalInput pin) {
  // fail if pin already added
  if (registered(pin)) return false;

  // stop the subsystem to synchronize configuration with sampling
  // thread (run function)
  stop();

  // set the pin mode
  palSetPadMode(getPort(pin), getPinNum(pin), PAL_MODE_INPUT_PULLUP);

  // add the pin internally
  m_pins[static_cast<uint32_t>(pin)] = true;
  // init current state
  m_pinStates[static_cast<uint32_t>(pin)] = getState(pin);
  m_numPins++;

  // start the subsystem again with new configuration
  start();

  // return success
  return true;
}

void DigInChSubsys::runThread() {
  while (true) {
    if (m_subsysActive) {
      std::vector<Event> events;

      for (uint32_t i = 0; i < m_numPins; i++) {
        // check for transition in pin state
        bool currentState = getState(static_cast<DigitalInput>(i));

        // if pin changed states
        if (currentState != getSavedState(static_cast<DigitalInput>(i))) {
          // save change
          m_pinStates[i] = currentState;
          // queue event
          events.push_back(Event(Event::Type::kDigInTransition,
                                 static_cast<DigitalInput>(i), currentState));
        }
      }

      // post events if present
      if (events.size() > 0) {
        m_eventQueue.push(events);
      }
    }
    // sleep to yield processing until next sample of channels
    chThdSleepMilliseconds(m_sampleClkMs);
  }
}

bool DigInChSubsys::removePin(DigitalInput pin) {
  // return failure if pin wasn't added
  if (!registered(pin)) return false;

  // stop the interface momentarily
  stop();

  // remove the pin internally
  m_pins[static_cast<uint32_t>(pin)] = false;
  m_numPins--;

  // start the subsystem with the new configuration (if at least
  // one pin is still registered)
  if (m_numPins > 0) start();

  // return success
  return false;
}

/**
 * @note Public interface only permits access to subsystem's
 *       internally saved pin state, rather than pin value, in order
 *       to ensure consistent in user's implementation
 */
bool DigInChSubsys::getSavedState(DigitalInput p) {
  return m_pinStates[static_cast<uint32_t>(p)];
}

// TODO: implement full subsystem, then implement pin adding
void DigInChSubsys::start() { m_subsysActive = true; }

/**
 * @note Subsystem should be stopped before any configurations are
 *       changed. Otherwise, partial configuration will temporarily
 *       corrupt conversion events send to the user's event queue
 */
void DigInChSubsys::stop() {
  // TODO: tear down the subystem
  m_subsysActive = false;
}

bool DigInChSubsys::registered(DigitalInput p) {
  if (m_pins[static_cast<uint32_t>(p)]) {
    return true;
  } else {
    return false;
  }
}

uint32_t DigInChSubsys::getPinNum(DigitalInput p) {
  return kPinMap[static_cast<uint32_t>(p)];
}

stm32_gpio_t* DigInChSubsys::getPort(DigitalInput p) {
  return kPortMap[static_cast<uint32_t>(p)];
}

bool DigInChSubsys::getState(DigitalInput p) {
  return palReadPad(getPort(p), getPinNum(p)) != 0;
}
