#pragma once

#include <stdint.h>

#include "DeviceMutex.h"

class PumpOutput {
 public:
  PumpOutput();

  bool begin();

  void setEnabled(bool enabled, TickType_t timeout);
  void stop(TickType_t timeout);
  bool update(uint32_t nowMs, TickType_t timeout);

 private:
  void writeUnlocked(bool enabled);

  DeviceMutex mutex_;
  bool enabled_;
  uint32_t startedMs_;
};
