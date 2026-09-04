#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <stdint.h>

#include "config/sdk_config.h"
#include "control/MotionController.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Fixed 18-byte little-endian frame sent by VISION/yolo(PID).py:
// magic, version, type, payload_len, sequence, sender_timestamp_ms,
// power_percent, steering_percent (positive=left), brake, reserved,
// crc16-ccitt.
class VisionUdpService {
 public:
  explicit VisionUdpService(MotionController &motion);

  bool begin(uint16_t port);
  void stop();
  bool running() const { return running_.load(std::memory_order_acquire); }

 private:
  static void taskEntry(void *context);
  void taskLoop();
  bool handleFrame(const uint8_t *data, size_t length);
  static uint16_t crc16Ccitt(const uint8_t *data, size_t length);

  MotionController &motion_;
  int socket_ = -1;
  uint16_t port_ = 0;
  std::atomic_bool running_{false};
  std::atomic_bool stopRequested_{false};
  StaticTask_t taskStorage_{};
  std::array<StackType_t, RF_VISION_UDP_TASK_STACK_BYTES / sizeof(StackType_t)>
      taskStack_{};
  TaskHandle_t taskHandle_ = nullptr;
};
