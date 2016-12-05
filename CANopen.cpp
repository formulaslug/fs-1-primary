// Copyright (c) Formula Slug 2016. All Rights Reserved.

#include "CANopen.h"

#include <cmath>
#include <cstdio>

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
constexpr CANConfig MakeConfig(CANBaudRate baud, bool loopback) {
  uint32_t btr = CAN_BTR_SJW(0) | CAN_BTR_TS2(5) | CAN_BTR_TS1(4);

  if (loopback) {
    btr |= CAN_BTR_LBKM;
  }

  switch (baud) {
  case CANBaudRate::k125k:
    btr |= CAN_BTR_BRP(239);
    break;
  case CANBaudRate::k250k:
    btr |= CAN_BTR_BRP(11);
    break;
  case CANBaudRate::k500k:
    btr |= CAN_BTR_BRP(5);
    break;
  case CANBaudRate::k1M:
    btr |= CAN_BTR_BRP(2);
    break;
  case CANBaudRate::k1M5:
    btr |= CAN_BTR_BRP(1);
    break;
  case CANBaudRate::k3M:
    btr |= CAN_BTR_BRP(0);
    break;
  }

  return {CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP, btr};
}

CANopen::CANopen(uint32_t id, CANBaudRate baud) {
  m_id = id;

  CANConfig config = MakeConfig(baud, true);
  canStart(&CAND1, &config);

  palWriteLine(LINE_LED_GREEN, PAL_LOW);
}

CANopen::~CANopen() { canStop(&CAND1); }

void CANopen::setFilters(std::initializer_list<uint32_t> filters) {
  CANFilter filterArray[filters.size()];

  size_t i = 0;
  for (auto filter : filters) {
    filterArray[i].mode = 1;  // mask mode
    filterArray[i].scale = 1;  // 32 bits mode
    filterArray[i].assignment = 0;  // must be 0 in this version of the driver

    // TODO: filters currently do nothing
    filterArray[i].register1 = filter;

    i++;
  }
}

bool CANopen::send(uint64_t data) {
  static CANTxFrame msg;

  msg.IDE = CAN_IDE_EXT;
  msg.EID = m_id;
  msg.RTR = CAN_RTR_DATA;
  msg.DLC = 8;
  msg.data32[0] = data & 0xFFFF;
  msg.data32[1] = data >> 32;

  return send(msg);
}

bool CANopen::send(const CANTxFrame& msg) {
  if (canTransmit(&CAND1, CAN_ANY_MAILBOX, &msg, MS2ST(100)) == MSG_OK) {
    palWriteLine(LINE_LED_GREEN, PAL_HIGH);
    return true;
  } else {
    palWriteLine(LINE_LED_GREEN, PAL_LOW);
    return false;
  }
}

bool CANopen::recv(CANRxFrame& msg) {
  return canReceive(&CAND1, CAN_ANY_MAILBOX, &msg, TIME_IMMEDIATE) == MSG_OK;
}

void CANopen::printTxMessage(const CANTxFrame& msg) const {
  std::printf("[CAN TX] COB-ID:");

  // Pad left of shorter ID with spaces
  for (uint32_t i = 0; i < 8 - std::log(msg.EID) / std::log(16); ++i) {
    std::printf(" ");
  }
  std::printf("0x");

  // Print the node's ID
  std::printf("%lx", msg.EID);

  std::printf("  data:");
  for (uint32_t i = 0; i < msg.DLC; ++i) {
    std::printf(" 0x");
    // Print every byte of message payload
    std::printf("%x", msg.data8[i]);
  }

  std::printf("\n");
}

void CANopen::printRxMessage(const CANRxFrame& msg) const {
  std::printf("[CAN RX] COB-ID:");

  // Pad left of shorter ID with spaces
  for (uint32_t i = 0; i < 8 - std::log(msg.EID) / std::log(16); ++i) {
    std::printf(" ");
  }
  std::printf("0x");

  // Print the node's ID
  std::printf("%lx", msg.EID);

  std::printf("  data:");
  for (uint32_t i = 0; i < msg.DLC; ++i) {
    std::printf(" 0x");
    // Print every byte of message payload
    std::printf("%x", msg.data8[i]);
  }

  std::printf("\n");
}

/**
 * @desc Transmits all enqueued messages, in g_canTxQueue, of type
 *       CAN_message_t. Enqueue them onto the transmit logs queue after so that
 *       they can be printed
 */
void CANopen::processTxMessages() {
  while (m_txQueue.Size() > 0) {
    // write message
    send(m_txQueue[0]);
    // enqueue them onto the logs queue
    m_txLogsQueue.PushBack(m_txQueue[0]);
    // dequeue new message
    m_txQueue.PopFront();
  }
}

/**
 * @desc Enqueue any messages apearing on the CAN bus
 */
void CANopen::processRxMessages() {
  static CANRxFrame rxMessageTmp;
  while (recv(rxMessageTmp)) {
    m_rxQueue.PushBack(rxMessageTmp);

    // TODO: figure out a way to remove this duplication
    m_rxLogsQueue.PushBack(rxMessageTmp);
  }
}

/**
 * @desc Prints over serial all messages currently in the tx logs queue
 */
void CANopen::printTxAll() {
  static CANTxFrame queueMessage;
  queueMessage = m_txLogsQueue.PopFront();
  while (queueMessage.EID) {
    // print
    printTxMessage(queueMessage);
    // dequeue another message
    queueMessage = m_txLogsQueue.PopFront();
  }
}

/**
 * @desc Prints over serial all messages currently in the rx queue
 */
void CANopen::printRxAll() {
  static CANRxFrame msg;
  msg = m_rxLogsQueue.PopFront();
  while (msg.EID) {
    // print
    printRxMessage(msg);
    // dequeue another message
    msg = m_rxLogsQueue.PopFront();
  }
}

/**
 * @desc Enqueues a packaged message to be transmitted over the CAN bus
 */
void CANopen::queueTxMessage(CANTxFrame msg) { m_txQueue.PushBack(msg); }

/**
 * @desc Dequeues a packaged message to be unpacked and used
 * @param msg The message at the front of the rx queue
 */
CANRxFrame CANopen::dequeueRxMessage() { return m_rxQueue.PopFront(); }

/**
 * @desc Gets the current size of the tx queue
 * @return The size
 */
uint8_t CANopen::txQueueSize() { return m_txQueue.Size(); }

/**
 * @desc Gets the current size of the rx queue
 * @return The size
 */
uint8_t CANopen::rxQueueSize() { return m_rxQueue.Size(); }
