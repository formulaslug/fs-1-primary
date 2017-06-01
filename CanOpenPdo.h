// Copyright (c) 2016-2017 Formula Slug. All Rights Reserved.

#pragma once

#include <stdint.h>

#include "ch.h"
#include "hal.h"

// COB-IDs: MAX LENGTH of 12 bits, only the LSB 12 should be used
constexpr uint32_t kCobid_TPDO5 = 0x241;  // including throttle voltage payload
constexpr uint32_t kCobid_node3Heartbeat = 0x003;
constexpr uint32_t kCobid_node4Heartbeat = 0x004;
constexpr uint32_t kCobid_p2s = 0x013;
constexpr uint32_t kCobid_s2p = 0x014;

// Payload constants
constexpr uint32_t kPayloadHeartbeat = 0x1;

struct HeartbeatMessage : public CANTxFrame {
  HeartbeatMessage(uint32_t id);
};

struct ThrottleMessage : public CANTxFrame {
  /**
   * @param throttleVoltage The current, cleaned throttle voltage to be sent to
   *                        Master
   * @param forwardSwitch If true, enables forward drive
   */
  ThrottleMessage(uint16_t throttleVoltage, bool forwardSwitch);
};

/**
 * TPDO sent from Teensy to master motor controller
 */
struct[[gnu::packed]] TPDO5 {
  uint16_t throttleInputVoltage;
  uint16_t maxBatteryDischargeCurrent;
  uint16_t maxBatteryRechargeCurrent;
  uint8_t forwardSwitch : 1;
  uint8_t driveSelect1Switch : 1;
  uint8_t driveSelect2Switch : 1;
  uint8_t reverseSwitch : 1;
  uint8_t padding : 4;
};
