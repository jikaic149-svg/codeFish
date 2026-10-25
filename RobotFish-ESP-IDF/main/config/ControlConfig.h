#pragma once

#include <stdint.h>
#include "config/RuntimeConfig.h"

// Latest F405 allocation policy, with unrelated services and safety omitted.
namespace robofish::config::control {
// HeadingTarget only: the speed loop supplies both tail gait and common props;
// yaw adds tail bias without increasing gait amplitude/frequency or adding a floor.
// Equal command percentages match the measured hybrid 10/10, 15/15 and 20/20
// recipes; they do not imply equal thrust. Ratio [0,1], zero surge means zero gait.
inline constexpr float kHeadingTailDriveRatio = 1.0f;
// Increase requested oscillation amplitude, independently of frequency and bias.
// Applies to heading control and explicit HYBRID calibration (not old baseline).
// Existing actuator travel/rate limits still bound actual joint commands.
inline constexpr float kHybridTailAmplitudeGain = 1.5f;
static_assert(kHybridTailAmplitudeGain >= 1 && kHybridTailAmplitudeGain <= 2);
// Optional differential gain [0,1] and cap [0,100]%; remaining common-drive
// headroom also limits it. Neither changes the existing yaw PID or tail gain.
inline constexpr float kHeadingPropDiffGain = 0.0f;
inline constexpr float kHeadingPropDiffLimitPercent = 0.0f;
inline constexpr const char *kHeadingAllocationProfile = "hybrid_gait_tail_bias";
static_assert(kHeadingTailDriveRatio >= 0 && kHeadingTailDriveRatio <= 1 &&
              kHeadingPropDiffGain >= 0 && kHeadingPropDiffGain <= 1 &&
              kHeadingPropDiffLimitPercent >= 0 && kHeadingPropDiffLimitPercent <= 100);
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
