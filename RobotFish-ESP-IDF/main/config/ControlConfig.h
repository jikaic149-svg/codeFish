#pragma once

#include <stdint.h>
#include "config/RuntimeConfig.h"

// Latest F405 allocation policy, with unrelated services and safety omitted.
namespace robofish::config::control {
inline constexpr uint32_t kPeriodMs = 20;
inline constexpr uint32_t kMaximumSlewElapsedMs = kPeriodMs;
inline constexpr float kPropulsionRatePercentPerSecond = 500.0f;
inline constexpr float kTurnForwardReductionFraction = 1.0f;
inline constexpr float kBrakeMaximumReversePercent = 50.0f;
inline constexpr float kFlowMinimumPercent = 100.0f; // 满潜浮沿用原档位造流。
inline constexpr float kHybridTailCommandScale = 1.0f; // 命令权重，不表示推力份额。
inline constexpr float kHybridPropulsionCommandScale = 1.0f;
inline constexpr float kTailForwardSpeedFraction = 0.8f;
inline constexpr float kPectoralLeftYawDegrees = -10.0f;
inline constexpr float kPectoralRightYawDegrees = 10.0f;
inline constexpr float kPectoralLeftBrakeDegrees = 5.0f;
inline constexpr float kPectoralRightBrakeDegrees = -5.0f;
inline constexpr float kPectoralLeftSurgeTrimDegrees = 0.0f;
inline constexpr float kPectoralRightSurgeTrimDegrees = 0.0f;
inline constexpr float kPectoralLeftTailTrimDegrees = 0.0f;
inline constexpr float kPectoralRightTailTrimDegrees = 0.0f;
inline constexpr float kDiveDragCompensationPercent = 0.0f;
}
