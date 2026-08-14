#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <stdint.h>

#include "driver/rmt_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Hardware owner for the recovered 8-pixel GPIO7 GRB/800-kHz status strip.
// The RMT transaction is bounded to 24 bytes and never waits indefinitely.
class RgbStatusStrip {
 public:
  RgbStatusStrip();

  bool begin();
  bool setColor(uint8_t red, uint8_t green, uint8_t blue);
  bool ready() const;

 private:
  static constexpr size_t kPixelCount = 8;

  std::array<uint8_t, kPixelCount * 3> pixels_{};
  StaticSemaphore_t mutexStorage_{};
  SemaphoreHandle_t mutex_ = nullptr;
  rmt_channel_handle_t channel_ = nullptr;
  rmt_encoder_handle_t encoder_ = nullptr;
  std::atomic_bool ready_{false};
};
