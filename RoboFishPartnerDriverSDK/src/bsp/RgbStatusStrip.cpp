#include "RgbStatusStrip.h"

#include <algorithm>
#include <iterator>

#include "driver/rmt_common.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"

namespace {

constexpr char kTag[] = "rgb";
constexpr gpio_num_t kDataGpio = GPIO_NUM_7;
constexpr uint32_t kResolutionHz = 10000000;
constexpr uint8_t kBrightness = 40;

uint8_t scale(uint8_t value) {
  return static_cast<uint8_t>((static_cast<uint16_t>(value) * kBrightness +
                               127U) /
                              255U);
}

}  // namespace

RgbStatusStrip::RgbStatusStrip() = default;

bool RgbStatusStrip::begin() {
  if (ready_.load()) return true;
  mutex_ = xSemaphoreCreateMutexStatic(&mutexStorage_);
  if (!mutex_) return false;

  const rmt_tx_channel_config_t channelConfig = {
      .gpio_num = kDataGpio,
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = kResolutionHz,
      .mem_block_symbols = 64,
      .trans_queue_depth = 1,
      .intr_priority = 0,
      .flags = {.invert_out = 0,
                .with_dma = 0,
                .io_loop_back = 0,
                .io_od_mode = 0},
  };
  esp_err_t error = rmt_new_tx_channel(&channelConfig, &channel_);

  // WS2812/NeoPixel timing at 10 MHz: T0H=0.3 us, T0L=0.9 us,
  // T1H=0.9 us, T1L=0.3 us. The >50 us latch is provided by the bounded
  // completion wait before the next status update.
  const rmt_bytes_encoder_config_t encoderConfig = {
      .bit0 = {.duration0 = 3, .level0 = 1, .duration1 = 9, .level1 = 0},
      .bit1 = {.duration0 = 9, .level0 = 1, .duration1 = 3, .level1 = 0},
      .flags = {.msb_first = 1},
  };
  if (error == ESP_OK) {
    error = rmt_new_bytes_encoder(&encoderConfig, &encoder_);
  }
  if (error == ESP_OK) error = rmt_enable(channel_);
  if (error != ESP_OK) {
    if (encoder_) rmt_del_encoder(encoder_);
    if (channel_) rmt_del_channel(channel_);
    encoder_ = nullptr;
    channel_ = nullptr;
    ESP_LOGE(kTag, "GPIO7 status strip initialization failed: %s",
             esp_err_to_name(error));
    return false;
  }

  ready_.store(true);
  const bool cleared = setColor(0, 0, 0);
  ESP_LOGI(kTag, "8-pixel GRB/800-kHz status strip ready on GPIO7, brightness 40");
  return cleared;
}

bool RgbStatusStrip::setColor(uint8_t red, uint8_t green, uint8_t blue) {
  if (!ready_.load() ||
      xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) != pdTRUE) {
    return false;
  }

  const uint8_t grb[3] = {scale(green), scale(red), scale(blue)};
  for (size_t pixel = 0; pixel < kPixelCount; ++pixel) {
    std::copy(std::begin(grb), std::end(grb),
              pixels_.begin() + pixel * 3);
  }
  const rmt_transmit_config_t transmitConfig = {
      .loop_count = 0,
      .flags = {.eot_level = 0, .queue_nonblocking = 1},
  };
  esp_err_t error = rmt_transmit(channel_, encoder_, pixels_.data(),
                                 pixels_.size(), &transmitConfig);
  if (error == ESP_OK) error = rmt_tx_wait_all_done(channel_, 10);
  xSemaphoreGive(mutex_);
  return error == ESP_OK;
}

bool RgbStatusStrip::ready() const { return ready_.load(); }
