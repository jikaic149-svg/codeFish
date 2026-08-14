#pragma once

#include "bsp/ServoPwm.h"
#include "DeviceMutex.h"

class TailServos {
 public:
  bool begin();

  void setServo1(int angle, TickType_t timeout);
  void setServo2(int angle, TickType_t timeout);
  void setBoth(int angle1, int angle2, TickType_t timeout);
  void center(TickType_t timeout);

 private:
  void writeServo1(int angle);
  void writeServo2(int angle);

  ServoPwm servo1_;
  ServoPwm servo2_;
  DeviceMutex mutex_;
};
