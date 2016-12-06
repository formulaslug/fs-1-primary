// Copyright (c) Formula Slug 2016. All Rights Reserved.

#pragma once

#include <stdint.h>

#include <memory>
#include <utility>

#include "hal.h"

template <typename Callable>
struct State {
  State(Callable&& f) : func(std::forward<Callable>(f)) {}
  Callable func;
};

/**
 * Create and launch a thread.
 *
 * @param function The address of the function to run as the new task.
 * @param args A parameter pack of arguments to pass to the function.
 */
template <class Function, class... Args>
thread::thread(tprio_t priority, Function&& func, Args&&... args) {
  auto funcWrapper =
      std::bind(std::forward<Function>(func), std::forward<Args>(args)...);
  auto state =
      std::make_unique<State<decltype(funcWrapper)>>(std::move(funcWrapper));

  auto threadFunc = [](void* arg) {
    auto state = reinterpret_cast<State<decltype(funcWrapper)>*>(arg);
    state->func();
    delete state;
  };

  m_thread =
      chThdCreateStatic(m_workingArea,  // nullptr = default heap
                        sizeof(m_workingArea),  // Stack size
                        priority,
                        threadFunc,
                        state.get());
  if (m_thread == nullptr) {
    chSysHalt("Out of memory");
  }

  // thread has taken control of memory
  state.release();

  m_joinable = true;
}
