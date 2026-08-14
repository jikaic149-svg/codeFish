#include "ServoPwm.h"

#include "esp_log.h"

namespace {

constexpr char kTag[] = "servo_pwm";
constexpr uint32_t kFrequencyHz = 50;
constexpr uint32_t kPeriodUs = 20000;
constexpr uint32_t kDutySteps = 1024;

bool configureTimer(ledc_timer_t timer) {
  const ledc_timer_config_t configuration = {
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .duty_resolution = LEDC_TIMER_10_BIT,
      .timer_num = timer,
      .freq_hz = kFrequencyHz,
      .clk_cfg = LEDC_AUTO_CLK,
      .deconfigure = false,
  };
  const esp_err_t error = ledc_timer_config(&configuration);
  if (error != ESP_OK) {
    ESP_LOGE(kTag, "LEDC timer %d configuration failed: %s",
             static_cast<int>(timer), esp_err_to_name(error));
    return false;
  }
  return true;
}

}  // namespace

ServoPwm::ServoPwm()
    : channel_(LEDC_CHANNEL_0),
      minPulseUs_(0),
      maxPulseUs_(0),
      initialized_(false) {}

bool ServoPwm::configureTimers() {
  bool ready = true;
  ready &= configureTimer(LEDC_TIMER_1);
  ready &= configureTimer(LEDC_TIMER_2);
  ready &= configureTimer(LEDC_TIMER_3);
  return ready;
}

bool ServoPwm::begin(gpio_num_t gpio, ledc_channel_t channel,
                     ledc_timer_t timer, int minPulseUs, int maxPulseUs) {
  channel_ = channel;
  minPulseUs_ = minPulseUs;
  maxPulseUs_ = maxPulseUs;

  const ledc_channel_config_t configuration = {
      .gpio_num = static_cast<int>(gpio),
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = channel,
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = timer,
      .duty = 0,
      .hpoint = 0,
      .flags = {.output_invert = 0},
  };
  const esp_err_t error = ledc_channel_config(&configuration);
  if (error != ESP_OK) {
    ESP_LOGE(kTag, "LEDC channel %d on GPIO%d failed: %s",
             static_cast<int>(channel), static_cast<int>(gpio),
             esp_err_to_name(error));
    return false;
  }

  initialized_ = true;
  return true;
}

bool ServoPwm::writeAngle(int angle) {
  if (!initialized_) return false;

  esp_err_t error =
      ledc_set_duty(LEDC_LOW_SPEED_MODE, channel_, angleToDuty(angle));
  if (error == ESP_OK) {
    error = ledc_update_duty(LEDC_LOW_SPEED_MODE, channel_);
  }
  if (error != ESP_OK) {
    ESP_LOGE(kTag, "LEDC channel %d update failed: %s",
             static_cast<int>(channel_), esp_err_to_name(error));
    return false;
  }
  return true;
}

uint32_t ServoPwm::angleToDuty(int angle) const {
  // Match ESP32Servo: integer Arduino map(), followed by a 10-bit floor.
  const int pulseUs =
      minPulseUs_ + (angle * (maxPulseUs_ - minPulseUs_)) / 180;
  return static_cast<uint32_t>(pulseUs) * kDutySteps / kPeriodUs;
}
