#pragma once

#include <stdint.h>

#include "driver/gpio.h"
#include "driver/ledc.h"

// ESP-IDF LEDC owner for one 50 Hz servo/ESC signal. Timer 0 remains unused
// to preserve the resource boundary of the previous ESP32Servo build.
class ServoPwm {
 public:
  ServoPwm();

  static bool configureTimers();

  bool begin(gpio_num_t gpio, ledc_channel_t channel, ledc_timer_t timer,
             int minPulseUs, int maxPulseUs);
  bool writeAngle(int angle);

 private:
  uint32_t angleToDuty(int angle) const;

  ledc_channel_t channel_;
  int minPulseUs_;
  int maxPulseUs_;
  bool initialized_;
};
