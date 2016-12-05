#include <avr_emulation.h>

template <uint8_t DebounceCount>
ButtonTracker<DebounceCount>::ButtonTracker(uint32_t pin, bool logicHigh) {
  m_pin = pin;
  m_logicHigh = logicHigh;
}

template <uint8_t DebounceCount>
void ButtonTracker<DebounceCount>::update() {
  uint8_t mask = 0xFF >> (8 - DebounceCount);

  if ((m_stateBuf & mask) == mask) {
    m_wasPressed = true;
  }

  if ((m_stateBuf & mask) == 0) {
    m_wasPressed = false;
  }

  m_stateBuf = m_stateBuf << 1 | (digitalReadFast(m_pin) == m_logicHigh);

  if ((m_stateBuf & mask) == mask) {
    m_isPressed = true;
  }

  if ((m_stateBuf & mask) == 0) {
    m_isPressed = false;
  }
}

template <uint8_t DebounceCount>
bool ButtonTracker<DebounceCount>::pressed() const {
  return !m_wasPressed && m_isPressed;
}

template <uint8_t DebounceCount>
bool ButtonTracker<DebounceCount>::released() const {
  return m_wasPressed && !m_isPressed;
}

template <uint8_t DebounceCount>
bool ButtonTracker<DebounceCount>::held() const {
  return m_wasPressed && m_isPressed;
}
