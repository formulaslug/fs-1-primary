#include "ch.h"
#include "hal.h"

/* baudrate = 36MHz / ((1 + BRP) * (3 + TS1 + TS2))
 * See STM32F103xx reference manual, 24.7.7 for info on CAN_BTR register.
 *
 * BRP   freq (Hz)
 * ---------------
 * 239   125k
 *  11   250k
 *   5   500k
 *   2     1M
 *   1   1.5M
 *   0     3M
 */
static constexpr CANConfig cancfg = {
  CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
  CAN_BTR_SJW(0) | CAN_BTR_TS2(5) | CAN_BTR_TS1(4) | CAN_BTR_BRP(11)
};

// RX thread
static THD_WORKING_AREA(can_rx_wa, 256);
static THD_FUNCTION(can_rx, p) {
  event_listener_t el;
  CANRxFrame rxmsg;

  static_cast<void>(p);
  chRegSetThreadName("receiver");
  chEvtRegister(&CAND1.rxfull_event, &el, 0);
  while (true) {
    if (chEvtWaitAnyTimeout(ALL_EVENTS, MS2ST(100)) == 0)
      continue;
    while (canReceive(&CAND1, CAN_ANY_MAILBOX, &rxmsg, TIME_IMMEDIATE) == MSG_OK) {
      /* Process message.*/
      palTogglePad(IOPORT3, GPIOC_LED);
    }
  }
  chEvtUnregister(&CAND1.rxfull_event, &el);
}

// TX thread
static THD_WORKING_AREA(can_tx_wa, 256);
static THD_FUNCTION(can_tx, p) {
  CANTxFrame txmsg;

  static_cast<void>(p);
  chRegSetThreadName("transmitter");
  txmsg.IDE = CAN_IDE_EXT;
  txmsg.EID = 0x01234567;
  txmsg.RTR = CAN_RTR_DATA;
  txmsg.DLC = 8;
  txmsg.data32[0] = 0x55AA55AA;
  txmsg.data32[1] = 0x00FF00FF;

  while (true) {
    canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));
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
  canStart(&CAND1, &cancfg);

  // Starting the transmitter and receiver threads
  chThdCreateStatic(can_rx_wa, sizeof(can_rx_wa), NORMALPRIO + 7, can_rx,
                    nullptr);
  chThdCreateStatic(can_tx_wa, sizeof(can_tx_wa), NORMALPRIO + 7, can_tx,
                    nullptr);

  while (true) {
    chThdSleepMilliseconds(500);
  }
}
