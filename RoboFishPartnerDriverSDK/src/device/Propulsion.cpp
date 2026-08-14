#include "Propulsion.h"

#include <algorithm>

#include "../config/sdk_config.h"
#include "esp_timer.h"

Propulsion::Propulsion()
    : throttleCommand_(RF_ESC_THROTTLE_NEUTRAL), lastThrottleCommandMs_(0) {}

bool Propulsion::begin() {
  if (!mutex_.begin()) return false;
  bool attached = true;
  attached &= throttle_.begin(GPIO_NUM_43, LEDC_CHANNEL_3, LEDC_TIMER_2,
                              RF_ESC_PWM_MIN_US, RF_ESC_PWM_MAX_US);
  attached &= steering_.begin(GPIO_NUM_44, LEDC_CHANNEL_4, LEDC_TIMER_3,
                              RF_ESC_PWM_MIN_US, RF_ESC_PWM_MAX_US);
  return attached;
}

void Propulsion::setThrottle(int command, TickType_t timeout) {
  ScopedDeviceLock lock(mutex_, timeout);
  if (!lock.acquired()) return;
  throttleCommand_ =
      std::clamp(command, RF_ESC_THROTTLE_MIN, RF_ESC_THROTTLE_MAX);
  throttle_.writeAngle(throttleCommand_);
  lastThrottleCommandMs_ =
      static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
}

void Propulsion::setSteering(int command, TickType_t timeout) {
  ScopedDeviceLock lock(mutex_, timeout);
  if (!lock.acquired()) return;
  steering_.writeAngle(
      std::clamp(command, RF_ESC_STEERING_MIN, RF_ESC_STEERING_MAX));
}

void Propulsion::stop(TickType_t timeout) {
  ScopedDeviceLock lock(mutex_, timeout);
  if (!lock.acquired()) return;
  stopUnlocked();
}

bool Propulsion::update(uint32_t nowMs, TickType_t timeout) {
  ScopedDeviceLock lock(mutex_, timeout);
  if (!lock.acquired()) return false;

  if (throttleCommand_ != RF_ESC_THROTTLE_NEUTRAL &&
      lastThrottleCommandMs_ != 0 &&
      nowMs - lastThrottleCommandMs_ >= RF_PROPULSION_TIMEOUT_MS) {
    stopUnlocked();
    return true;
  }
  return false;
}

void Propulsion::stopUnlocked() {
  throttleCommand_ = RF_ESC_THROTTLE_NEUTRAL;
  throttle_.writeAngle(RF_ESC_THROTTLE_NEUTRAL);
  steering_.writeAngle(RF_ESC_STEERING_CENTER);
  lastThrottleCommandMs_ = 0;
}
