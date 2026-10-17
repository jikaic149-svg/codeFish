#pragma once

// Motion-only calibration from the latest F405 firmware. No ADC or radio driver.
#define RF_TAIL_MIN_ANGLE 20
#define RF_TAIL_MAX_ANGLE 152

namespace robofish::config::lora {
inline constexpr int kPectoralLeftCenter = 90;
inline constexpr int kPectoralRightCenter = 90;
inline constexpr int kTailFrontCenter = 90;
inline constexpr int kTailRearCenter = 90;
inline constexpr int kPectoralLeftMinimum = 45;
inline constexpr int kPectoralLeftMaximum = 145;
inline constexpr int kPectoralRightMinimum = 35;
inline constexpr int kPectoralRightMaximum = 155;
inline constexpr int kPectoralLeftUp = 140;
inline constexpr int kPectoralRightUp = 40;
inline constexpr int kPectoralLeftDown = 50;
inline constexpr int kPectoralRightDown = 130;
inline constexpr float kPectoralScale = 0.85f;
inline constexpr float kServoStepPer10Ms = 2.5f;
inline constexpr float kTailStaticSteeringDegrees = 40.0f;
inline constexpr float kTailAmplitudeDegrees = 25.0f;
inline constexpr float kTailTurnBiasDegrees = 18.0f;
// Straight forward gait maximum, Hz; scaled with sqrt(effective tail request).
// 25 degrees at 1.25 Hz requires 196.35 deg/s before amplitude transitions.
inline constexpr float kTailForwardMaximumFrequencyHz = 1.25f;
inline constexpr float kRearTailAmplitudeRatio = 0.90f;
inline constexpr float kBionicPhaseLagDegrees = 60.0f;
inline constexpr float kHybridPhaseLagDegrees = 72.0f;
inline constexpr int kForwardDirection = 1;
inline constexpr int kTurnDirection = -1;
inline constexpr int kLeftEscDirection = 1;
inline constexpr int kRightEscDirection = -1;
inline constexpr int kTailSteeringDirection = 1;
inline constexpr int kTailFrontDirection = 1;
inline constexpr int kTailRearDirection = 1;
inline constexpr int kGearMinimumPercent = 50;
inline constexpr int kGearMaximumPercent = 100;
inline constexpr int kEscMinimumUs = 1000;
inline constexpr int kEscNeutralUs = 1500;
inline constexpr int kEscMaximumUs = 2000;
}  // namespace robofish::config::lora
