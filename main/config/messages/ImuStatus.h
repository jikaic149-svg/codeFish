#pragma once
#include <stdint.h>

namespace robofish::imu {
struct Vector3 { float x = 0, y = 0, z = 0; };
// JY61P 原生坐标系；加速度 g、角速度 deg/s、欧拉角 deg。姿态控制需显式映射到鱼体坐标。
struct Status {
  uint32_t baud = 115200;
  Vector3 acceleration, angularVelocity, angles;
  uint32_t accelerationMs = 0, angularVelocityMs = 0, anglesMs = 0;
  uint32_t receivedBytes = 0, frames = 0, checksumErrors = 0, receiveErrors = 0;
  bool uartReady = false;
  bool hasAcceleration = false, hasAngularVelocity = false, hasAngles = false;
};
}
