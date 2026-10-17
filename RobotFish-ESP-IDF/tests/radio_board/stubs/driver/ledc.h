#pragma once
#include "driver/uart.h"
using ledc_mode_t = int;
using ledc_timer_t = int;
using ledc_channel_t = int;
inline constexpr int LEDC_LOW_SPEED_MODE = 0, LEDC_TIMER_0 = 0;
inline constexpr int LEDC_TIMER_14_BIT = 14, LEDC_AUTO_CLK = 0, LEDC_INTR_DISABLE = 0;
struct ledc_timer_config_t { int speed_mode, duty_resolution, timer_num; unsigned freq_hz; int clk_cfg; };
struct ledc_channel_config_t {
  int gpio_num, speed_mode, channel, intr_type, timer_sel;
  uint32_t duty;
};
esp_err_t ledc_timer_config(const ledc_timer_config_t *);
esp_err_t ledc_channel_config(const ledc_channel_config_t *);
esp_err_t ledc_set_duty(ledc_mode_t, ledc_channel_t, uint32_t);
esp_err_t ledc_update_duty(ledc_mode_t, ledc_channel_t);
esp_err_t ledc_stop(ledc_mode_t, ledc_channel_t, unsigned);
