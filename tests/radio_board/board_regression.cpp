#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "bsp.h"

static uint32_t duties[6]{};
static int configuredGpios[6]{};
static int failTimer = 0, failConfigure = -1, failDuty = -1, failUpdate = -1;
static unsigned gpioCalls = 0, stops = 0, delayTicks = 0, dutyCalls = 0;

esp_err_t gpio_reset_pin(gpio_num_t) { ++gpioCalls; return ESP_OK; }
esp_err_t gpio_set_level(gpio_num_t, unsigned) { ++gpioCalls; return ESP_OK; }
esp_err_t gpio_set_direction(gpio_num_t, int) { ++gpioCalls; return ESP_OK; }
void vTaskDelay(unsigned ticks) { delayTicks += ticks; }
int64_t esp_timer_get_time() { return 1234567; }
const char *esp_err_to_name(esp_err_t) { return "mock"; }
esp_err_t ledc_timer_config(const ledc_timer_config_t *timer) {
  assert(timer->freq_hz == 50 && timer->duty_resolution == 14 && timer->timer_num == 0);
  return failTimer;
}
esp_err_t ledc_channel_config(const ledc_channel_config_t *channel) {
  if (channel->channel == failConfigure) return -1;
  configuredGpios[channel->channel] = channel->gpio_num;
  duties[channel->channel] = channel->duty;
  return ESP_OK;
}
esp_err_t ledc_set_duty(ledc_mode_t, ledc_channel_t channel, uint32_t duty) {
  ++dutyCalls;
  if (channel == failDuty) return -1;
  duties[channel] = duty; return ESP_OK;
}
esp_err_t ledc_update_duty(ledc_mode_t, ledc_channel_t channel) {
  return channel == failUpdate ? -1 : ESP_OK;
}
esp_err_t ledc_stop(ledc_mode_t, ledc_channel_t, unsigned) { ++stops; return ESP_OK; }

static bool pulseIs(unsigned channel, double microseconds) {
  return std::abs(duties[channel] * 20000.0 / 16384.0 - microseconds) <= 0.62;
}

int main() {
  robofish::ActuatorFrame frame;
  assert(board::applyFrameMask(frame) == 0 && !board::setPump(true));
  failTimer = -1; assert(!board::begin()); failTimer = 0;
  failConfigure = 2; assert(!board::begin() && stops == 2); failConfigure = -1;
  assert(board::begin() && delayTicks == 2000 && gpioCalls == 0);
  const int expectedGpios[] = {5,6,4,3,43,8};
  for (unsigned i = 0; i < 6; ++i) assert(configuredGpios[i] == expectedGpios[i] && pulseIs(i,1500));
  assert(board::begin() && delayTicks == 2000 && board::nowMs() == 1234);
  frame.leftPropulsionPercent = frame.rightPropulsionPercent = 100;
  assert(board::applyFrameMask(frame) == 0x3f && pulseIs(4,2000) && pulseIs(5,1000));
  frame.leftPropulsionPercent = frame.rightPropulsionPercent = -100;
  assert(board::applyFrameMask(frame) == 0x3f && pulseIs(4,1000) && pulseIs(5,2000));
  frame.tailFrontDegrees = 110; frame.tailRearDegrees = 70;
  assert(board::applyFrameMask(frame) == 0x3f &&
      pulseIs(0,500+70*2000.0/180) && pulseIs(1,500+110*2000.0/180));
  frame.leftPropulsionPercent = std::numeric_limits<int>::max();
  frame.rightPropulsionPercent = std::numeric_limits<int>::min();
  frame.tailFrontDegrees = 0; frame.tailRearDegrees = 180;
  frame.pectoralLeftDegrees = 0; frame.pectoralRightDegrees = 180;
  assert(board::applyFrameMask(frame) == 0x3f && pulseIs(4,2000) && pulseIs(5,2000));
  assert(pulseIs(0,500+122*2000.0/180) && pulseIs(1,500+45*2000.0/180));
  assert(pulseIs(2,1000) && pulseIs(3,500+155*2000.0/180));
  frame.tailFrontDegrees = std::numeric_limits<float>::quiet_NaN();
  const unsigned calls = dutyCalls;
  assert(board::applyFrameMask(frame) == 0 && dutyCalls == calls);
  frame = {};
  failDuty = 4; assert(board::applyFrameMask(frame) == 0x2f); failDuty = -1;
  failUpdate = 5; assert(board::applyFrameMask(frame) == 0x1f); failUpdate = -1;
  frame.leftPropulsionPercent = frame.rightPropulsionPercent = 40;
  assert(board::applyFrameMask(frame) == 0x3f);
  frame.leftPropulsionPercent = frame.rightPropulsionPercent = 0;
  failDuty = 5;
  assert(board::applyFrameMask(frame) == 0x1f && pulseIs(4,1500) && pulseIs(5,1300));
  failDuty = -1;
  assert(board::applyFrameMask(frame) == 0x3f && pulseIs(4,1500) && pulseIs(5,1500));
  assert(board::setPump(false) && !board::setPump(true));
  assert(board::applyFrame(frame,false) && !board::applyFrame(frame,true) && gpioCalls == 0);
  puts("PASS: actual BSP PWM pins/limits/direction/partial writes, absent pump; no radio dependency");
}
