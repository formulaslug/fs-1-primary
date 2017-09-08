// Copyright (c) 2015-2017 Formula Slug. All Rights Reserved.

#pragma once

#include <cstddef>
#include <vector>

/**
 * This is a simple circular buffer so we don't need to "bucket brigade" copy
 * old values.
 */
template <class T>
class CircularBuffer {
 public:
  explicit CircularBuffer(size_t size);

  void PushFront(T value);
  void PushBack(T value);
  T PopFront();
  T PopBack();
  void Resize(size_t size);
  void Reset();
  size_t Size() const;
  size_t Capacity() const;

  T& operator[](size_t index);
  const T& operator[](size_t index) const;

 private:
  std::vector<T> m_data;

  // Index of element at front of buffer
  size_t m_front = 0;

  // Number of elements used in buffer
  size_t m_length = 0;

  size_t ModuloInc(size_t index);
  size_t ModuloDec(size_t index);
};

#include "CircularBuffer.inc"
