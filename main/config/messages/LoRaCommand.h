#pragma once

#include <stdint.h>
#include "config/messages/WifiControl.h"

namespace robofish::lora {

// 协议边界校验后交给唯一控制器的文本遥控命令；不是线上二进制结构体。
struct Command {
  uint16_t sequence = 0;
  uint8_t code = 0;
  bool hasKnob = false;
  uint16_t knob = 0;
  uint16_t source = 0;  // Existing third wire field; sequence is tracked per source.
  DriveMode mode = DriveMode::Prop;
  wifi::Axes axes{};
  float propulsionScalePercent = 100;
  wifi::YawPid yawPid{};
};

inline constexpr uint8_t kTarget = 20, kHeartbeat = 21, kMode = 22, kAttitudeOn = 23, kAttitudeOff = 24, kYawPid = 25;

enum class ControlCode : uint8_t { Accepted, Invalid, Unsupported, Busy, Maintenance,
                                 OutputRecovery, OutputFault };
struct ControlResult {
  Command command;
  ControlCode code = ControlCode::Invalid;
  uint32_t receivedMs = 0;
  bool available = false;
};

inline const char *controlCodeName(ControlCode code) {
  switch (code) {
    case ControlCode::Accepted: return "accepted";
    case ControlCode::Unsupported: return "pump_unavailable";
    case ControlCode::Busy: return "neutral_pending";
    case ControlCode::Maintenance: return "ota_locked";
    case ControlCode::OutputRecovery: return "output_recovery";
    case ControlCode::OutputFault: return "output_fault";
    default: return "invalid_command";
  }
}

}  // namespace robofish::lora
