#include "TailServos.h"

#include <algorithm>

#include "../config/sdk_config.h"

bool TailServos::begin() {
  if (!mutex_.begin()) return false;
  bool attached = true;
  attached &= servo1_.begin(GPIO_NUM_5, LEDC_CHANNEL_0, LEDC_TIMER_1, 500, 2500);
  attached &= servo2_.begin(GPIO_NUM_6, LEDC_CHANNEL_1, LEDC_TIMER_1, 500, 2500);
  return attached;
}

void TailServos::setServo1(int angle, TickType_t timeout) {
  ScopedDeviceLock lock(mutex_, timeout);
  if (!lock.acquired()) return;
  writeServo1(angle);
}

void TailServos::setServo2(int angle, TickType_t timeout) {
  ScopedDeviceLock lock(mutex_, timeout);
  if (!lock.acquired()) return;
  writeServo2(angle);
}

void TailServos::setBoth(int angle1, int angle2, TickType_t timeout) {
  ScopedDeviceLock lock(mutex_, timeout);
  if (!lock.acquired()) return;
  writeServo1(angle1);
  writeServo2(angle2);
}

void TailServos::center(TickType_t timeout) {
  setBoth(RF_TAIL_CENTER, RF_TAIL_CENTER, timeout);
}

void TailServos::writeServo1(int angle) {
  servo1_.writeAngle(
      std::clamp(angle, RF_TAIL_MIN_ANGLE, RF_TAIL_MAX_ANGLE));
}

void TailServos::writeServo2(int angle) {
  servo2_.writeAngle(
      std::clamp(angle, RF_TAIL_MIN_ANGLE, RF_TAIL_MAX_ANGLE));
}
