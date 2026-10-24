#pragma once
#include "APP/Algorithm/AttitudeControl.h"
#include "config/ControlConfig.h"

namespace robofish::config::attitude {
// User measured: sensor yaw increases left, pitch decreases nose-up, roll
// increases right. The 2026-10-23 backend log shows resting roll near +/-180
// (e.g. -175.75), so use FRD {+1,+2,+3}, not the level-roll-zero {+1,-2,-3}.
// Log pose is not a measured level calibration; retain zero trims below.
// Signed sensor axes: +/-1(X), +/-2(Y), +/-3(Z). Zero means unconfirmed.
inline constexpr int kBodyXFromSensor = 1;
inline constexpr int kBodyYFromSensor = 2;
inline constexpr int kBodyZFromSensor = 3;
// +1: positive body request produces positive body torque; -1: opposite.
// Pitch positive = nose up, roll = right side down, yaw = right turn.
// Existing UP raises the nose; positive yaw requests turn right.
inline constexpr int kPitchCommandSign = 1;
inline constexpr int kRollCommandSign = 0;
inline constexpr int kYawCommandSign = 1;
// Initial PD gains, not pool-tuned: command %/degree and command %/(degree/second).
inline constexpr float kPitchKp = 1.0f, kPitchKd = 0.10f;
inline constexpr float kRollKp = 1.0f, kRollKd = 0.10f;
inline constexpr float kYawKp = 1.0f, kYawKd = 0.10f;
inline constexpr float kPitchTrimDegrees = 0, kRollTrimDegrees = 0;
inline constexpr float kMaximumCorrectionPercent = 20; // Existing allocator limits/slew still apply.
inline constexpr uint32_t kMaximumSampleAgeMs = 200; // Both angle and rate must be fresh.

inline robofish::attitude::Settings makeSettings() {
  robofish::attitude::Settings settings;
  settings.sensorToBody[0] = kBodyXFromSensor;
  settings.sensorToBody[1] = kBodyYFromSensor;
  settings.sensorToBody[2] = kBodyZFromSensor;
  settings.pitchCommandSign = kPitchCommandSign;
  // Roll also needs calibrated differential fin geometry in ControlConfig.h.
  settings.rollCommandSign = config::control::kPectoralLeftRollDegrees != 0 ||
      config::control::kPectoralRightRollDegrees != 0 ? kRollCommandSign : 0;
  settings.yawCommandSign = kYawCommandSign;
  settings.pitchKp = kPitchKp; settings.pitchKd = kPitchKd;
  settings.rollKp = kRollKp; settings.rollKd = kRollKd;
  settings.yawKp = kYawKp; settings.yawKd = kYawKd;
  settings.pitchTrimDegrees = kPitchTrimDegrees; settings.rollTrimDegrees = kRollTrimDegrees;
  settings.maximumCorrectionPercent = kMaximumCorrectionPercent;
  settings.maximumAgeMs = kMaximumSampleAgeMs;
  return settings;
}
}  // namespace robofish::config::attitude
