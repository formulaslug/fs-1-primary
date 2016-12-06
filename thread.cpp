// Copyright (c) Formula Slug 2016. All Rights Reserved.

#include "thread.h"

thread& thread::operator=(thread&& rhs) {
  m_thread = rhs.m_thread;
  m_joinable = rhs.m_joinable.load();
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
