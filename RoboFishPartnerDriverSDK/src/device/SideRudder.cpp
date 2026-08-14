#include "SideRudder.h"

#include <algorithm>

#include "../config/sdk_config.h"

bool SideRudder::begin() {
  if (!mutex_.begin()) return false;
  return servo_.begin(GPIO_NUM_4, LEDC_CHANNEL_2, LEDC_TIMER_2, 500, 2500);
}

void SideRudder::setAngle(int angle, TickType_t timeout) {
  ScopedDeviceLock lock(mutex_, timeout);
  if (!lock.acquired()) return;
  servo_.writeAngle(
      std::clamp(angle, RF_PITCH_MIN_ANGLE, RF_PITCH_MAX_ANGLE));
}

void SideRudder::center(TickType_t timeout) {
  setAngle(RF_PITCH_CENTER, timeout);
}
