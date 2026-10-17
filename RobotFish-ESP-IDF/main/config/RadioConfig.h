#pragma once
#include <stdint.h>

namespace robofish::config::lora {
inline constexpr int kBaud = 115200;  // E32 UART，8N1，模块保持外部透明模式配置。
inline constexpr int kFishNode = 1;
inline constexpr unsigned kMaxLineLength = 32;
inline constexpr uint32_t kAckDelayMs = 260;
}

namespace robofish::config::radio {
inline constexpr uint32_t kPollPeriodMs = 10;
inline constexpr int kReceiveBufferBytes = 512;
inline constexpr unsigned kReceiveEventCount = 16;
inline constexpr unsigned kReadBudgetBytes = 128;
inline constexpr unsigned kRecentTransactionCount = 4;
inline constexpr unsigned kMaximumSources = 4;
// 沿用旧 F4 接收策略：空口参数和遥控器静默窗未验证，默认不发送 ACK。
struct Timing {
  bool ackTxEnabled = false;
  bool peerWindowsVerified = false;
  bool moduleConfigurationVerified = false;
  unsigned packetBytes = 0;
  uint32_t packetIdleUs = 0;
  uint32_t replyDelayMs = lora::kAckDelayMs;
  uint32_t replyWindowMs = 0;
  uint32_t uartDrainTimeoutMs = 0;
  uint32_t rfReleaseBudgetMs = 0;
};
inline constexpr Timing kTiming{};
}
