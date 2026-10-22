#include "bsp.h"
#include <algorithm>
#include <cmath>
#include "config/BoardConfig.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace board {
namespace {
namespace calibration = robofish::config::lora;
constexpr ledc_mode_t kSpeed = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t kTimer = LEDC_TIMER_0;
constexpr int kGpios[] = {config::kTailFrontGpio, config::kTailRearGpio,
    config::kPectoralLeftGpio, config::kPectoralRightGpio,
    config::kLeftEscGpio, config::kRightEscGpio};
bool ready = false;

uint32_t dutyForPulse(float pulseUs) {
  constexpr uint32_t levels = 1U << config::kPwmResolutionBits;
  return static_cast<uint32_t>(std::lround(pulseUs * config::kPwmFrequencyHz * levels / 1000000.0f));
}

float servoPulse(float angle, float minimum, float maximum) {
  angle = std::clamp(angle, minimum, maximum);
  return config::kServoMinimumUs + angle / 180.0f *
      (config::kServoMaximumUs - config::kServoMinimumUs);
}

float escPulse(int nominal, int direction) {
  const int percent = std::clamp(nominal, -100, 100) * direction;
  return calibration::kEscNeutralUs + percent * (percent >= 0 ?
      calibration::kEscMaximumUs - calibration::kEscNeutralUs :
      calibration::kEscNeutralUs - calibration::kEscMinimumUs) / 100.0f;
}

bool writePulse(unsigned channel, float pulseUs) {
  const auto output = static_cast<ledc_channel_t>(channel);
  return ledc_set_duty(kSpeed, output, dutyForPulse(pulseUs)) == ESP_OK &&
      ledc_update_duty(kSpeed, output) == ESP_OK;
}
}

bool begin() {
  if (ready) return true;
  const auto pump = static_cast<gpio_num_t>(config::kPumpGpio);
  esp_err_t error = ESP_OK;
  if constexpr (config::kPumpEnabled) {
    error = gpio_reset_pin(pump);
    if (error == ESP_OK) error = gpio_set_level(pump, config::kPumpActiveHigh ? 0 : 1);
    if (error == ESP_OK) error = gpio_set_direction(pump, GPIO_MODE_OUTPUT);
  }
  ledc_timer_config_t timer{};
  timer.speed_mode = kSpeed;
  timer.duty_resolution = LEDC_TIMER_14_BIT;
  timer.timer_num = kTimer;
  timer.freq_hz = config::kPwmFrequencyHz;
  timer.clk_cfg = LEDC_AUTO_CLK;
  if (error == ESP_OK) error = ledc_timer_config(&timer);
  const float neutral[] = {servoPulse(calibration::kTailFrontCenter, 0, 180),
      servoPulse(calibration::kTailRearCenter, 0, 180),
      servoPulse(calibration::kPectoralLeftCenter, 0, 180),
      servoPulse(calibration::kPectoralRightCenter, 0, 180),
      escPulse(0, 1), escPulse(0, 1)};
  unsigned configured = 0;
  for (unsigned i = 0; error == ESP_OK && i < 6; ++i) {
    ledc_channel_config_t channel{};
    channel.gpio_num = kGpios[i];
    channel.speed_mode = kSpeed;
    channel.channel = static_cast<ledc_channel_t>(i);
    channel.intr_type = LEDC_INTR_DISABLE;
    channel.timer_sel = kTimer;
    channel.duty = dutyForPulse(neutral[i]);
    error = ledc_channel_config(&channel);
    if (error == ESP_OK) ++configured;
  }
  if (error != ESP_OK) {
    for (unsigned i = 0; i < configured; ++i)
      ledc_stop(kSpeed, static_cast<ledc_channel_t>(i), 0);
    if constexpr (config::kPumpEnabled)
      gpio_set_level(pump, config::kPumpActiveHigh ? 0 : 1);
    ESP_LOGE("board", "Actuator initialization failed: %s", esp_err_to_name(error));
    return false;
  }
  ready = true;
  vTaskDelay(pdMS_TO_TICKS(config::kEscArmMs)); // 仅启动时中立解锁。
  return true;
}

uint8_t applyFrameMask(const robofish::ActuatorFrame &frame) {
  if (!ready || !std::isfinite(frame.tailFrontDegrees) ||
      !std::isfinite(frame.tailRearDegrees) || !std::isfinite(frame.pectoralLeftDegrees) ||
      !std::isfinite(frame.pectoralRightDegrees)) return 0;
  const float pulses[] = {
      servoPulse(calibration::kTailFrontCenter + calibration::kTailFrontDirection *
          (frame.tailFrontDegrees - calibration::kTailFrontCenter), calibration::kTailFrontMinimum, calibration::kTailFrontMaximum),
      servoPulse(calibration::kTailRearCenter + calibration::kTailRearDirection *
          (frame.tailRearDegrees - calibration::kTailRearCenter), calibration::kTailRearMinimum, calibration::kTailRearMaximum),
      servoPulse(frame.pectoralLeftDegrees, calibration::kPectoralLeftMinimum, calibration::kPectoralLeftMaximum),
      servoPulse(frame.pectoralRightDegrees, calibration::kPectoralRightMinimum, calibration::kPectoralRightMaximum),
      escPulse(frame.leftPropulsionPercent, calibration::kLeftEscDirection),
      escPulse(frame.rightPropulsionPercent, calibration::kRightEscDirection)};
  uint8_t mask = 0;
  for (unsigned i = 0; i < 6; ++i)
    if (writePulse(i, pulses[i])) mask |= static_cast<uint8_t>(1U << i);
  return mask;
}

bool setPump(bool enabled) {
  if constexpr (!config::kPumpEnabled) return !enabled;
  return ready && gpio_set_level(static_cast<gpio_num_t>(config::kPumpGpio),
      enabled == config::kPumpActiveHigh ? 1 : 0) == ESP_OK;
}

bool applyFrame(const robofish::ActuatorFrame &frame, bool pump) {
  const uint8_t mask = applyFrameMask(frame);
  const bool pumpWritten = setPump(pump);
  return mask == config::kAllActuatorsMask && pumpWritten;
}

uint32_t nowMs() { return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL); }
}
