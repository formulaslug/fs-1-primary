// Copyright (c) 2015-2017 Formula Slug. All Rights Reserved.

#pragma once

#include <algorithm>

template <class T>
CircularBuffer<T>::CircularBuffer(size_t size) : m_data(size) {}

/**
 * Push new value onto front of the buffer. The value at the back is overwritten
 * if the buffer is full.
 */
template <class T>
void CircularBuffer<T>::PushFront(T value) {
  if (m_data.size() == 0) {
    return;
  }

  m_front = ModuloDec(m_front);

  m_data[m_front] = value;

  if (m_length < m_data.size()) {
    m_length++;
  }
}

/**
 * Push new value onto back of the buffer. The value at the front is overwritten
 * if the buffer is full.
 */
template <class T>
void CircularBuffer<T>::PushBack(T value) {
  if (m_data.size() == 0) {
    return;
  }

  m_data[(m_front + m_length) % m_data.size()] = value;

  if (m_length < m_data.size()) {
    m_length++;
  } else {
    // Increment front if buffer is full to maintain size
    m_front = ModuloInc(m_front);
  }
}

/**
 * Pop value at front of buffer.
 */
template <class T>
T CircularBuffer<T>::PopFront() {
  // If there are no elements in the buffer, do nothing
  if (m_length == 0) {
    return T();
  }

  T& temp = m_data[m_front];
  m_front = ModuloInc(m_front);
  m_length--;
  return temp;
}

/**
 * Pop value at back of buffer.
 */
template <class T>
T CircularBuffer<T>::PopBack() {
  // If there are no elements in the buffer, do nothing
  if (m_length == 0) {
    return 0;
  }

  m_length--;
  return m_data[(m_front + m_length) % m_data.size()];
}

/**
 * Changes size of internal buffer.
 *
 * All grows and shrinks are performed at the back of the circular buffer.
 */
template <class T>
void CircularBuffer<T>::Resize(size_t size) {
  if (size > m_length) {
    while (size > m_length) {
      PushBack(0);
    }
  } else if (size < m_length) {
    while (size < m_length) {
      PopBack();
    }
  }
}

/**
 * Sets internal buffer contents to zero.
 */
template <class T>
void CircularBuffer<T>::Reset() {
  std::fill(m_data.begin(), m_data.end(), 0);
  m_front = 0;
  m_length = 0;
}

/**
 * @return Number of elements in the buffer.
 */
template <class T>
size_t CircularBuffer<T>::Size() const {
  return m_length;
}

/**
 * @return Size of the storage space currently allocated, expressed in terms
 *         of elements.
 */
template <class T>
size_t CircularBuffer<T>::Capacity() const {
  return m_data.size();
}

/**
 * @return element at index starting from front of buffer.
 */
template <class T>
T& CircularBuffer<T>::operator[](size_t index) {
  return m_data[(m_front + index) % m_data.size()];
}

/**
 * @return Element at index starting from front of buffer.
 */
template <class T>
const T& CircularBuffer<T>::operator[](size_t index) const {
  return m_data[(m_front + index) % m_data.size()];
}

/**
 * Increment an index modulo the length of the m_data buffer.
 */
template <class T>
size_t CircularBuffer<T>::ModuloInc(size_t index) {
  return (index + 1) % m_data.size();
}

/**
 * Decrement an index modulo the length of the m_data buffer.
 */
template <class T>
size_t CircularBuffer<T>::ModuloDec(size_t index) {
  if (index == 0) {
    return m_data.size() - 1;
  } else {
    return index - 1;
  }
}
