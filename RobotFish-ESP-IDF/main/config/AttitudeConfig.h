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
// User reconfirmed the original PID polarity is correct; retain that direction.
inline constexpr int kYawCommandSign = 1;
// Three-axis PID is available, but only yaw reaches real actuators in this test.
// Tilt controllers run in preview only. Roll also requires fin geometry/sign calibration.
inline constexpr bool kPitchOutputEnabled = false, kRollOutputEnabled = false;
inline constexpr float kPitchKp = 1, kPitchKi = .02f, kPitchKd = .10f;
inline constexpr float kRollKp = 1, kRollKi = .02f, kRollKd = .10f;
// Initial PID, not water-tuned. Units: %/deg, %/(deg*s), %*s/deg.
// Each gain accepts 0..100, with 0.001 resolution over LoRa.
// The UI changes RAM values; reboot restores these defaults, loop remains OFF.
// Outside the deadband, nominal P demand: 10deg -> 25%, 20deg -> 50%.
// I/D, body-axis projection and actuator slew still affect instantaneous output.
inline constexpr float kYawKp = 2.5f, kYawKi = 0.02f, kYawKd = 0.10f;
// Symmetric yaw deadband, degrees (>=0). Clear I/D and turn demand inside it;
// retain the heading reference and independently requested forward motion.
inline constexpr float kYawDeadbandDegrees = 1.0f;
// Closed-loop turn authority, unitless (>0): scale propeller differential and
// tail steering together. Existing per-actuator and mechanical limits apply.
inline constexpr float kYawActuatorGain = 1.5f;
// Yaw PID updates at 10 Hz; the existing gait/PWM scheduler is independent.
// This is a command update period, not a claim that the servo has reached its target.
inline constexpr uint32_t kYawPidPeriodMs = 100;
// Software secondary filtering: two first-order stages, EACH at 4 Hz (>0).
// Both the yaw controller and displayed Euler angles use this result. Raw
// measurements remain available. More smoothing adds phase delay in water.
inline constexpr float kAngleFilterStageHz = 4.0f;
inline constexpr float kPitchTrimDegrees = 0, kRollTrimDegrees = 0;
inline constexpr float kMaximumCorrectionPercent = 20; // Pitch/roll preview bounds; outputs remain off.
// Yaw uses the allocator's full normalized command range, in percent (0 < limit <= 100).
// Mechanical travel, slew and per-prop +/-100% limits remain owned by the allocator/BSP.
inline constexpr float kMaximumYawCorrectionPercent = 100;
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
  settings.pitchKi=kPitchKi; settings.pitchEnabled=kPitchOutputEnabled;
  settings.rollKp = kRollKp; settings.rollKd = kRollKd;
  settings.rollKi=kRollKi; settings.rollEnabled=kRollOutputEnabled;
  settings.yawKp = kYawKp; settings.yawKi = kYawKi; settings.yawKd = kYawKd;
  settings.yawDeadbandDegrees = kYawDeadbandDegrees;
  settings.pitchTrimDegrees = kPitchTrimDegrees; settings.rollTrimDegrees = kRollTrimDegrees;
  settings.maximumCorrectionPercent = kMaximumCorrectionPercent;
  settings.maximumYawCorrectionPercent = kMaximumYawCorrectionPercent;
  settings.maximumAgeMs = kMaximumSampleAgeMs;
  settings.yawPeriodMs = kYawPidPeriodMs;
  return settings;
}
}  // namespace robofish::config::attitude
