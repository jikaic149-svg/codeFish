#pragma once

// Motion-only calibration from the latest F405 firmware. No ADC or radio driver.

namespace robofish::config::lora {
inline constexpr int kPectoralLeftCenter = 90;
inline constexpr int kPectoralRightCenter = 90;
inline constexpr int kTailFrontCenter = 90;
inline constexpr int kTailRearCenter = 90;
// User-confirmed joint-relative mechanical limits, with 5 degrees reserved
// independently at each side. The rear angle is relative to the front segment.
inline constexpr int kTailFrontMechanicalDegrees = 37;
inline constexpr int kTailRearMechanicalDegrees = 50;
inline constexpr int kTailMarginDegrees = 5;
inline constexpr int kTailFrontMinimum = kTailFrontCenter - kTailFrontMechanicalDegrees + kTailMarginDegrees;
inline constexpr int kTailFrontMaximum = kTailFrontCenter + kTailFrontMechanicalDegrees - kTailMarginDegrees;
inline constexpr int kTailRearMinimum = kTailRearCenter - kTailRearMechanicalDegrees + kTailMarginDegrees;
inline constexpr int kTailRearMaximum = kTailRearCenter + kTailRearMechanicalDegrees - kTailMarginDegrees;
inline constexpr int kPectoralLeftMinimum = 45;
inline constexpr int kPectoralLeftMaximum = 145;
inline constexpr int kPectoralRightMinimum = 35;
inline constexpr int kPectoralRightMaximum = 155;
inline constexpr int kPectoralLeftUp = 140;
inline constexpr int kPectoralRightUp = 40;
inline constexpr int kPectoralLeftDown = 50;
inline constexpr int kPectoralRightDown = 130;
inline constexpr float kPectoralScale = 0.85f;
inline constexpr float kServoStepPer10Ms = 0.9f; // Fin start/return: 90 deg/s, 42 degrees in about 0.47 s.
inline constexpr float kTailStepPer10Ms = 3.0f; // 尾摆 300 deg/s，支持提高后的摆频。
inline constexpr float kTailStaticSteeringDegrees = 32.0f; // Front joint's usable travel.
inline constexpr float kTailAmplitudeDegrees = 25.0f;
inline constexpr float kTailTurnBiasDegrees = 24.0f; // Jointly scaled with swing inside each joint's limits.
inline constexpr float kHybridTurnTailDrivePercent = 35.0f; // Tail participation at full yaw, including zero surge.
// Gait maximum, Hz; frequency uses sqrt(max(tail drive, turn demand)).
// Amplitude still follows tail drive, including Hybrid's turn reduction.
// 25 degrees at 1.5 Hz requires 235.62 deg/s before amplitude transitions.
inline constexpr float kTailForwardMaximumFrequencyHz = 1.5f;
inline constexpr float kRearTailAmplitudeRatio = 0.90f;
inline constexpr float kBionicPhaseLagDegrees = 60.0f;
inline constexpr float kHybridPhaseLagDegrees = 72.0f;
inline constexpr int kForwardDirection = 1;
inline constexpr int kTurnDirection = -1;
inline constexpr int kLeftEscDirection = 1;
inline constexpr int kRightEscDirection = -1;
inline constexpr int kTailSteeringDirection = 1;
inline constexpr int kTailFrontDirection = -1;
inline constexpr int kTailRearDirection = -1;
inline constexpr int kGearMinimumPercent = 50;
inline constexpr int kGearMaximumPercent = 100;
inline constexpr int kEscMinimumUs = 1000;
inline constexpr int kEscNeutralUs = 1500;
inline constexpr int kEscMaximumUs = 2000;
}  // namespace robofish::config::lora
