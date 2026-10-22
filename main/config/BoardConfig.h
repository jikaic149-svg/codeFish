#pragma once
#include <stdint.h>

namespace board::config {
// 单 XIAO ESP32-S3；桌面 ino 与用户接口图共同确认的 GPIO。
inline constexpr int kTailFrontGpio = 5;      // D4 / DJ2
inline constexpr int kTailRearGpio = 6;       // D5 / DJ3
inline constexpr int kPectoralLeftGpio = 4;   // D3 / DJ1
inline constexpr int kPectoralRightGpio = 3;  // D2
inline constexpr int kLeftEscGpio = 43;      // D6 / PWM1
inline constexpr int kRightEscGpio = 8;      // D9 / PWM3
inline constexpr int kLoRaTxGpio = 1;        // D0 -> 模块 RXD
inline constexpr int kLoRaRxGpio = 2;        // D1 <- 模块 TXD
inline constexpr int kPumpGpio = 9;          // D10
inline constexpr bool kPumpEnabled = false;  // 用户确认未接水泵；不配置或驱动 GPIO9。
inline constexpr bool kPumpActiveHigh = false; // 未启用，沿用 ino 标记供接线时重新确认。
inline constexpr uint32_t kPwmFrequencyHz = 50;
inline constexpr unsigned kPwmResolutionBits = 14; // ESP32-S3 LEDC，约 1.22 us/LSB。
inline constexpr int kServoMinimumUs = 500;
inline constexpr int kServoMaximumUs = 2500;
inline constexpr uint32_t kEscArmMs = 2000;  // 上电中立解锁，期间只让出 CPU。
inline constexpr uint8_t kAllActuatorsMask = 0x3f;
}
