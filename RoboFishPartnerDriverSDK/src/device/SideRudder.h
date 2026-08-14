#pragma once

#include "bsp/ServoPwm.h"
#include "DeviceMutex.h"

class SideRudder {
 public:
  bool begin();

  void setAngle(int angle, TickType_t timeout);
  void center(TickType_t timeout);

 private:
  ServoPwm servo_;
  DeviceMutex mutex_;
};
