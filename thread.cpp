// Copyright (c) 2018 Formula Slug. All Rights Reserved.

#include <utility>

#include "thread.h"

thread::thread(thread&& rhs) noexcept { *this = std::move(rhs); }

thread& thread::operator=(thread&& rhs) noexcept {
  if (rhs.m_joinable == false) {
    chSysHalt("Can't move detached thread");
  }

  m_thread = rhs.m_thread;
  m_joinable = true;
  rhs.m_thread = nullptr;
  rhs.m_joinable = false;

  return *this;
}

thread::~thread() {
  chThdRelease(m_thread);
  if (m_joinable) {
    m_joinable = false;
  }
}

bool thread::joinable() const noexcept { return m_joinable; }

void thread::join() { chThdWait(m_thread); }

void thread::detach() { m_joinable = false; }

thread::id thread::get_id() const noexcept { return m_thread; }

thread::native_handle_type thread::native_handle() { return m_thread; }
