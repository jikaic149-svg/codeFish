#pragma once
#include <stddef.h>
#include <stdint.h>
#include <cstring>
#include "config/messages/ImuStatus.h"

namespace robofish::imu {
// WIT 私有协议：0x55、类型、8 字节载荷、低 8 位累加校验；逐字节接收。
class Jy61p {
 public:
  void reset() { length_ = 0; }
  void consume(uint8_t byte, uint32_t now, Status &status) {
    if (!length_ && byte != 0x55) return;
    frame_[length_++] = byte;
    if (length_ != sizeof(frame_)) return;
    uint8_t sum = 0;
    for (unsigned i = 0; i < sizeof(frame_) - 1; ++i) sum += frame_[i];
    if (sum != frame_[10]) {
      ++status.checksumErrors;
      unsigned next = 1;
      while (next < sizeof(frame_) && frame_[next] != 0x55) ++next;
      length_ = sizeof(frame_) - next;
      std::memmove(frame_, frame_ + next, length_);
      return;
    }
    const auto vector = [&](float scale) {
      Vector3 value;
      float *axes[] = {&value.x, &value.y, &value.z};
      for (unsigned i = 0; i < 3; ++i) {
        int raw = frame_[2 + 2*i] | (frame_[3 + 2*i] << 8);
        if (raw >= 32768) raw -= 65536;
        *axes[i] = raw * scale / 32768.0f;
      }
      return value;
    };
    switch (frame_[1]) {
      case 0x51:
        status.acceleration = vector(16);
        status.accelerationMs = now; status.hasAcceleration = true; ++status.frames;
        break;
      case 0x52:
        status.angularVelocity = vector(2000);
        status.angularVelocityMs = now; status.hasAngularVelocity = true; ++status.frames;
        break;
      case 0x53:
        status.angles = vector(180);
        status.anglesMs = now; status.hasAngles = true; ++status.frames;
        break;
    }
    reset();
  }
 private:
  uint8_t frame_[11]{};
  size_t length_ = 0;
};
}
