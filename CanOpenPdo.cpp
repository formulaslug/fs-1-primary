// Copyright (c) 2018 Formula Slug. All Rights Reserved.

#include "CanOpenPdo.h"

#include <cstring>

// TODO: Make COB-ID naming match CANOpen convention

HeartbeatMessage::HeartbeatMessage(uint16_t data) {
  IDE = CAN_IDE_EXT;
  EID = kSysIdFs | kNodeIdPrimary | kFuncIdHeartbeat;
  RTR = CAN_RTR_DATA;
  DLC = 2;

  data8[0] = (data >> 8) & 0xFF; // MSB (32's 3rd byte)
  data8[1] = data & 0xFF; // LSB (32's 4th byte)
}

// ThrottleMessage::ThrottleMessage(uint16_t throttleVoltage, bool forwardSwitch) {
//   IDE = CAN_IDE_EXT;
//   EID = kCobIdTPDO5;
//   RTR = CAN_RTR_DATA;
//   DLC = 8;
//
//   TPDO5 tpdo5;
//   tpdo5.throttleInputVoltage = throttleVoltage;
//   tpdo5.maxBatteryDischargeCurrent = 400;
//   tpdo5.maxBatteryRechargeCurrent = 400;
//   tpdo5.forwardSwitch = forwardSwitch;
//   tpdo5.driveSelect1Switch = false;
//   tpdo5.driveSelect2Switch = false;
//   tpdo5.reverseSwitch = false;
//
//   std::memcpy(data8, &tpdo5, sizeof(TPDO5));
// }

ThrottleMessage::ThrottleMessage(uint16_t throttleVoltage) {
  IDE = CAN_IDE_EXT;
  EID = kSysIdFs | kNodeIdPrimary | kFuncIdThrottleValue; //kCobIdTPDO5;
  RTR = CAN_RTR_DATA;
  DLC = 2;

  data8[0] = throttleVoltage & 0xFF; // MSB (32's 3rd byte) (left most byte in DVT)
  data8[1] = (throttleVoltage >> 8) & 0xFF; // LSB (32's 4th byte) (right most byte in DVT)

  // TPDO5 tpdo5;
  // tpdo5.throttleInputVoltage = throttleVoltage;
  // tpdo5.maxBatteryDischargeCurrent = 400;
  // tpdo5.maxBatteryRechargeCurrent = 400;
  // tpdo5.forwardSwitch = forwardSwitch;
  // tpdo5.driveSelect1Switch = false;
  // tpdo5.driveSelect2Switch = false;
  // tpdo5.reverseSwitch = false;

  // std::memcpy(data8, &tpdo5, sizeof(TPDO5));
}
