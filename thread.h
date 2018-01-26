// Copyright (c) 2018 Formula Slug. All Rights Reserved.

#pragma once

#include <atomic>

#include "ch.hpp"

/**
 * Wrapper class around ChibiOS dynamic threads
 */
class thread {
 public:
  thread() = default;
  thread(thread&& rhs) noexcept;
  thread(const thread&) = delete;
  thread& operator=(const thread&) = delete;
  thread& operator=(thread&& rhs) noexcept;

  template <class Function, class... Args>
  thread(tprio_t priority, Function&& function, Args&&... args);

  virtual ~thread();

  using id = thread_t*;
  using native_handle_type = thread_t*;

  bool joinable() const noexcept;
  void join();
  void detach();
  thread::id get_id() const noexcept;
  thread::native_handle_type native_handle();

 private:
  thread_t* m_thread;
  THD_WORKING_AREA(m_workingArea, 128);
  std::atomic<bool> m_joinable{false};
};

#include "thread.inc"
