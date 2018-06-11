// Copyright (c) 2018 Formula Slug. All Rights Reserved.

#pragma once

#include <mutex>
#include <stdint.h>
#include "ch.hpp"
#include "hal.h"
#include "CircularBuffer.h"
#include "Event.h"

/*
 * @brief Thread-safe FIFO queue of events
 */
class EventQueue {
  private:
    CircularBuffer<Event> m_queue{20};
    chibios_rt::Mutex m_queueMut;
  public:
    EventQueue();

    // @brief thread-safe queue pop
    // TODO: Rename to, correct, dequeue
    Event pop();

    // @brief thread-safe queue push
    // TODO: Rename to, correct, enqueue
    void push(Event e);

    // ------------------------ WIP ------------------------
    // @brief Wait (block) until the queue becomes non-empty.
    // @param timeout US before call times out
    // @return True if queue is now non-empty, false if timed out
    // TODO: Implement using condition vars against the circ buffer's
    //       size
    bool wait();

    // @return The length of the event queue
    size_t size();
};
