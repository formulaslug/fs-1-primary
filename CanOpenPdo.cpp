// Copyright (c) Formula Slug 2016. All Rights Reserved.

#include "CanOpenPdo.h"

#include <cstring>

HeartbeatMessage::HeartbeatMessage(uint32_t id) {
  IDE = CAN_IDE_EXT;
  EID = id;
  RTR = CAN_RTR_DATA;
  DLC = 8;

  data8[0] = kPayloadHeartbeat >> 8;
  data8[1] = kPayloadHeartbeat;
}

ThrottleMessage::ThrottleMessage(uint16_t throttleVoltage, bool forwardSwitch) {
  IDE = CAN_IDE_EXT;
  EID = kCobid_TPDO5;
  RTR = CAN_RTR_DATA;
  DLC = 8;

  TPDO5 tpdo5;
  tpdo5.throttleInputVoltage = throttleVoltage;
  tpdo5.maxBatteryDischargeCurrent = 400;
  tpdo5.maxBatteryRechargeCurrent = 400;
  tpdo5.forwardSwitch = forwardSwitch;
  tpdo5.driveSelect1Switch = false;
  tpdo5.driveSelect2Switch = false;
  tpdo5.reverseSwitch = false;

  std::memcpy(data8, &tpdo5, sizeof(TPDO5));
}
