// Copyright (c) 2018 Formula Slug. All Rights Reserved.

#include <array>
#include <mutex>

#include "Event.h"
#include "EventQueue.h"
#include "Vehicle.h"
#include "ch.hpp"
#include "hal.h"
#include "mcuconfFs.h"

int main() {
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  // @TODO Set UART input modes (eventually should be done inside of the UART
  //       abstraction)

  EventQueue fsmEventQueue = EventQueue();

  while (1) {
    // always deplete the queue to help ensure that events are
    // processed faster than they're generated
    while (fsmEventQueue.size() > 0) {
      Event e = fsmEventQueue.pop();

      if (e.type() == Event::Type::kDigInTransition) {
        // ...
      }
    }

    // TODO: use condition var to signal that events are present in the queue
    chThdSleepMilliseconds(
        1);  // must be fast enough to deplete event queue quickly enough
  }
}
