#pragma once
#include "driver/uart.h"
using gpio_num_t = int;
inline constexpr int GPIO_MODE_OUTPUT = 1;
esp_err_t gpio_reset_pin(gpio_num_t);
esp_err_t gpio_set_level(gpio_num_t, unsigned);
esp_err_t gpio_set_direction(gpio_num_t, int);
