#pragma once
#include <stdint.h>

namespace robofish::config::lora {
inline constexpr int kBaud = 115200;  // A39 UART，8N1；沿用已验证的定点配置及外部固定模式脚。
inline constexpr int kFishNode = 1;
inline constexpr unsigned kMaxLineLength = 80; // CRC-protected three-axis targets, including signed tenths.
inline constexpr uint32_t kLinkTimeoutMs = 3000; // Fresh LoRa commands/heartbeats retain priority.
inline constexpr uint32_t kAckDelayMs = 260;
}

namespace robofish::config::radio {
inline constexpr uint32_t kPollPeriodMs = 10;
inline constexpr int kReceiveBufferBytes = 512;
inline constexpr unsigned kReceiveEventCount = 16;
inline constexpr unsigned kReadBudgetBytes = 128;
inline constexpr unsigned kRecentTransactionCount = 4;
inline constexpr unsigned kMaximumSources = 4;
// 机器鱼只从 LoRa 接收；命令结果通过 Wi-Fi 快照回传，生产配置不发送 LoRa ACK。
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
