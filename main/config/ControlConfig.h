#pragma once

#include <stdint.h>
#include "config/RuntimeConfig.h"

// Latest F405 allocation policy, with unrelated services and safety omitted.
namespace robofish::config::control {
inline constexpr uint32_t kPeriodMs = 20;
inline constexpr uint32_t kMaximumSlewElapsedMs = kPeriodMs;
// Ordinary start/stop: 0 to 100% (or back to neutral) takes 500 ms.
inline constexpr float kPropulsionRatePercentPerSecond = 200.0f;
// Tail bias/amplitude and stationary steering/return speed, degrees/second.
// Keep the periodic gait's separate 300 deg/s budget so 1.5 Hz remains available.
inline constexpr float kTailTransitionDegreesPerSecond = 50.0f;
// 普通停止及换向过零按 200 %/s 减速：100% 到中立需 500 ms。
inline constexpr float kPropulsionStopRatePercentPerSecond = 200.0f;
inline constexpr float kTurnForwardReductionFraction = 1.0f;
inline constexpr float kBrakeMaximumReversePercent = 50.0f;
inline constexpr float kFlowMinimumPercent = 100.0f; // 满潜浮沿用原档位造流。
inline constexpr float kHybridTailCommandScale = 1.0f; // 命令权重，不表示推力份额。
inline constexpr float kHybridPropulsionCommandScale = 1.0f;
inline constexpr float kTailForwardSpeedFraction = 0.8f;
// Fins follow the selected heave state only; yaw is handled by tail/props.
inline constexpr float kPectoralLeftYawDegrees = 0.0f;
inline constexpr float kPectoralRightYawDegrees = 0.0f;
// Servo-angle offsets for +100% body roll request. Zero until physically calibrated.
inline constexpr float kPectoralLeftRollDegrees = 0.0f;
inline constexpr float kPectoralRightRollDegrees = 0.0f;
inline constexpr float kPectoralLeftBrakeDegrees = 5.0f;
inline constexpr float kPectoralRightBrakeDegrees = -5.0f;
inline constexpr float kPectoralLeftSurgeTrimDegrees = 0.0f;
inline constexpr float kPectoralRightSurgeTrimDegrees = 0.0f;
inline constexpr float kPectoralLeftTailTrimDegrees = 0.0f;
inline constexpr float kPectoralRightTailTrimDegrees = 0.0f;
inline constexpr float kDiveDragCompensationPercent = 0.0f;
}
