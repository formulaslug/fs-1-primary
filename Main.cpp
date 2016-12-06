// Copyright (c) Formula Slug 2016. All Rights Reserved.

#include "CANopen.h"
#include "ch.h"
#include "hal.h"

// RX thread
static THD_FUNCTION(can_rx, p) {
  CANopen* canBus = static_cast<CANopen*>(p);

  event_listener_t el;
  CANRxFrame rxmsg;

  chRegSetThreadName("receiver");
  chEvtRegister(&CAND1.rxfull_event, &el, 0);
  while (true) {
    if (chEvtWaitAnyTimeout(ALL_EVENTS, MS2ST(100)) == 0) continue;
    while (canBus->recv(rxmsg)) {}
  }
  chEvtUnregister(&CAND1.rxfull_event, &el);
}

// TX thread
static THD_FUNCTION(can_tx, p) {
  CANopen* canBus = static_cast<CANopen*>(p);

  chRegSetThreadName("transmitter");

  while (true) {
    canBus->send(0x00FF00FF55AA55AA);
    chThdSleepMilliseconds(500);
  }
}

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

  // Activate CAN driver 1 (PA11 = CANRX, PA12 = CANTX)
  CANopen canBus(0x680, CANBaudRate::k250k, false);

  // Start transmitter thread
  static THD_WORKING_AREA(can_rx_wa, 256);
  chThdCreateStatic(can_rx_wa, sizeof(can_rx_wa), NORMALPRIO + 7, can_rx,
                    &canBus);

  // Start receiver thread
  static THD_WORKING_AREA(can_tx_wa, 256);
  chThdCreateStatic(can_tx_wa, sizeof(can_tx_wa), NORMALPRIO + 7, can_tx,
                    &canBus);

  while (true) {
    chThdSleepMilliseconds(500);
  }
}
