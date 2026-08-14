#pragma once

#include <stdint.h>

#include "bsp/ServoPwm.h"
#include "DeviceMutex.h"

class Propulsion {
 public:
  Propulsion();

  bool begin();

  void setThrottle(int command, TickType_t timeout);
  void setSteering(int command, TickType_t timeout);
  void stop(TickType_t timeout);
  bool update(uint32_t nowMs, TickType_t timeout);

 private:
  void stopUnlocked();

  ServoPwm throttle_;
  ServoPwm steering_;
  DeviceMutex mutex_;
  int throttleCommand_;
  uint32_t lastThrottleCommandMs_;
};
