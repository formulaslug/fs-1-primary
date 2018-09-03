// Copyright (c) 2018 Formula Slug. All Rights Reserved.

#include "EventQueue.h"

#include <stdint.h>

#include <mutex>

#include "hal.h"

EventQueue::EventQueue() {}

Event EventQueue::pop() {
  // acquire lock guard in current scope
  std::lock_guard<chibios_rt::Mutex> lock(m_queueMut);
  // pop item from queue and return
  return m_queue.PopFront();
}

bool EventQueue::tryPush(Event e) {
  // acquire lock in current scope
  bool didAcquire = m_queueMut.tryLock();
  //std::unique_lock<chibios_rt::Mutex> queueLock(m_queueMut,
  //    std::try_to_lock);
  if (!didAcquire) {
    // don't push, indicate failure
    return false;
  } else {
    // push item, indicate success
    m_queue.PushBack(e);
    m_queueMut.unlock();
    return true;
  }
}

void EventQueue::push(Event e) {
  // acquire lock in current scope
  std::lock_guard<chibios_rt::Mutex> queueGuard(m_queueMut);
  // push item
  m_queue.PushBack(e);
}

void EventQueue::push(std::vector<Event> events) {
  // acquire lock in current scope
  std::lock_guard<chibios_rt::Mutex> queueGuard(m_queueMut);
  // push items
  for (Event e : events) {
    m_queue.PushBack(e);
  }
}

bool EventQueue::wait() { return false; }

size_t EventQueue::size() { return m_queue.Size(); }
