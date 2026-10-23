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
inline constexpr int kLoRaTxGpio = 2;        // D1 / PCB TX -> 模块 RXD
inline constexpr int kLoRaRxGpio = 1;        // D0 / PCB RX <- 模块 TXD
// 按用户现有接法配置：LoRa TXD 接 PCB RX，LoRa RXD 接 PCB TX，不必改线。
inline constexpr int kImuTxGpio = 7;         // D8 / RGB -> JY61P RX
inline constexpr int kImuRxGpio = 9;         // D10 / GPIO_IN1 <- JY61P TX
inline constexpr int kImuBaud = 115200;      // 用户确认当前 JY61P 为 115200，8N1。
inline constexpr int kImuReceiveBufferBytes = 512;
inline constexpr unsigned kImuReadBudgetBytes = 128; // 每 10 ms 最多读取 128 字节，不等待串口。
// GPIO9 同时连接 U16 LED / R1 1k 支路；实物改作 UART RX 前需隔离该支路。
inline constexpr int kPumpGpio = 9;          // D10
inline constexpr bool kPumpEnabled = false;  // 用户确认未接水泵；不配置或驱动 GPIO9。
static_assert(!kPumpEnabled, "GPIO9 is reserved for the JY61P UART RX");
inline constexpr bool kPumpActiveHigh = false; // 未启用，沿用 ino 标记供接线时重新确认。
inline constexpr uint32_t kPwmFrequencyHz = 50;
inline constexpr unsigned kPwmResolutionBits = 14; // ESP32-S3 LEDC，约 1.22 us/LSB。
inline constexpr int kServoMinimumUs = 500;
inline constexpr int kServoMaximumUs = 2500;
inline constexpr uint32_t kEscArmMs = 2000;  // 上电中立解锁，期间只让出 CPU。
inline constexpr uint8_t kAllActuatorsMask = 0x3f;
}
