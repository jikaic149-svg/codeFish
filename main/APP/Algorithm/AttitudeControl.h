#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace robofish::attitude {

struct Vector { float x = 0, y = 0, z = 0; };
struct Sample {
  Vector anglesDegrees, ratesDps;
  uint32_t anglesMs = 0, ratesMs = 0;
  bool ready = false, hasAngles = false, hasRates = false;
};
struct Settings {
  // Body FRD: X forward, Y right, Z down. Signed sensor axes, +/-1..3.
  int sensorToBody[3]{};
  int pitchCommandSign = 0, rollCommandSign = 0, yawCommandSign = 0; // 0 = response not calibrated.
  float pitchKp = 0, pitchKd = 0, rollKp = 0, rollKd = 0, yawKp = 0, yawKd = 0;
  float pitchTrimDegrees = 0, rollTrimDegrees = 0;
  float maximumCorrectionPercent = 0;
  uint32_t maximumAgeMs = 0;
};
enum class Reason : uint8_t { Disabled, Uncalibrated, WaitingMotion, NoImu, Stale,
                              Invalid, ManualPitch, ManualYaw, HeadingUnavailable, Active };
inline const char *reasonName(Reason reason) {
  switch (reason) {
    case Reason::Disabled: return "disabled";
    case Reason::Uncalibrated: return "uncalibrated";
    case Reason::WaitingMotion: return "waiting_motion";
    case Reason::NoImu: return "imu_unavailable";
    case Reason::Stale: return "imu_stale";
    case Reason::Invalid: return "invalid_sample";
    case Reason::ManualPitch: return "manual_pitch";
    case Reason::ManualYaw: return "manual_yaw";
    case Reason::HeadingUnavailable: return "heading_unavailable";
    default: return "active";
  }
}
struct Result {
  Reason reason = Reason::Disabled;
  bool requested = false, bodyValid = false;
  bool pitchCalibrated = false, rollCalibrated = false, yawCalibrated = false;
  bool pitchActive = false, rollActive = false, yawActive = false;
  bool headingValid = false, headingLocked = false, manualYaw = false;
  bool hasSample = false;
  float rollDegrees = 0, pitchDegrees = 0;
  float rollRateDps = 0, pitchRateDps = 0;
  float rollCorrectionPercent = 0, pitchCorrectionPercent = 0;
  float yawDegrees = 0, yawRateDps = 0, yawProjection = 0;
  float headingTargetDegrees = 0, yawCorrectionPercent = 0;
  uint32_t sampleMs = 0; // Oldest measurement actually used by this control tick.
};
struct HeadingState {
  float targetDegrees = 0;
  uint32_t generation = 0;
  bool locked = false;
};

inline bool validFrame(const Settings &settings) {
  int matrix[3][3]{};
  unsigned used = 0;
  for (unsigned row = 0; row < 3; ++row) {
    const int axis = settings.sensorToBody[row];
    if (!axis || axis < -3 || axis > 3) return false;
    const unsigned column = static_cast<unsigned>(axis < 0 ? -axis : axis) - 1;
    if (used & (1u << column)) return false;
    used |= 1u << column;
    matrix[row][column] = axis > 0 ? 1 : -1;
  }
  const int determinant = matrix[0][0] * (matrix[1][1]*matrix[2][2] - matrix[1][2]*matrix[2][1]) -
      matrix[0][1] * (matrix[1][0]*matrix[2][2] - matrix[1][2]*matrix[2][0]) +
      matrix[0][2] * (matrix[1][0]*matrix[2][1] - matrix[1][1]*matrix[2][0]);
  return determinant == 1; // A reflection is not a physical sensor mounting.
}
inline bool calibratedSign(int sign) { return sign == 1 || sign == -1; }
inline bool validSettings(const Settings &s) {
  return validFrame(s) && s.maximumAgeMs > 0 && s.maximumAgeMs < 0x80000000u &&
      std::isfinite(s.maximumCorrectionPercent) && s.maximumCorrectionPercent > 0 && s.maximumCorrectionPercent <= 100 &&
      std::isfinite(s.pitchKp) && s.pitchKp >= 0 && s.pitchKp <= 100 &&
      std::isfinite(s.pitchKd) && s.pitchKd >= 0 && s.pitchKd <= 100 &&
      std::isfinite(s.rollKp) && s.rollKp >= 0 && s.rollKp <= 100 &&
      std::isfinite(s.rollKd) && s.rollKd >= 0 && s.rollKd <= 100 &&
      std::isfinite(s.yawKp) && s.yawKp >= 0 && s.yawKp <= 100 &&
      std::isfinite(s.yawKd) && s.yawKd >= 0 && s.yawKd <= 100 &&
      std::isfinite(s.pitchTrimDegrees) && std::abs(s.pitchTrimDegrees) <= 30 &&
      std::isfinite(s.rollTrimDegrees) && std::abs(s.rollTrimDegrees) <= 30;
}
inline Vector toBody(const Settings &settings, Vector sensor) {
  const float values[] = {sensor.x, sensor.y, sensor.z};
  float body[3]{};
  for (unsigned row = 0; row < 3; ++row) {
    const int axis = settings.sensorToBody[row];
    body[row] = (axis > 0 ? 1 : -1) * values[(axis > 0 ? axis : -axis) - 1];
  }
  return {body[0], body[1], body[2]};
}
inline bool finite(Vector value) {
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

// Stateless PD: angle error supplies restoring demand, measured body rate damps it.
// The caller owns the motion lease and must pass motionAllowed=false on any STOP.
inline Result evaluate(const Settings &s, const Sample &sample, uint32_t nowMs,
                       bool requested, bool motionAllowed, bool manualPitch) {
  Result result;
  result.requested = requested;
  result.hasSample = sample.hasAngles && sample.hasRates;
  if (result.hasSample) result.sampleMs = nowMs - std::max(uint32_t(nowMs-sample.anglesMs), uint32_t(nowMs-sample.ratesMs));
  const bool settingsValid = validSettings(s);
  result.pitchCalibrated = settingsValid && calibratedSign(s.pitchCommandSign) && s.pitchKp > 0;
  result.rollCalibrated = settingsValid && calibratedSign(s.rollCommandSign) && s.rollKp > 0;
  result.yawCalibrated = settingsValid && calibratedSign(s.yawCommandSign) && s.yawKp > 0;
  result.reason = !requested ? Reason::Disabled : !result.pitchCalibrated && !result.rollCalibrated && !result.yawCalibrated ?
      Reason::Uncalibrated : !motionAllowed ? Reason::WaitingMotion : Reason::Active;
  if (!settingsValid) return result;
  const auto unavailable = [&](Reason reason) {
    if (result.reason == Reason::Active) result.reason = reason;
    return result;
  };
  if (!sample.ready || !sample.hasAngles || !sample.hasRates) {
    return unavailable(Reason::NoImu);
  }
  if (uint32_t(nowMs - sample.anglesMs) > s.maximumAgeMs ||
      uint32_t(nowMs - sample.ratesMs) > s.maximumAgeMs) {
    return unavailable(Reason::Stale);
  }
  if (!finite(sample.anglesDegrees) || !finite(sample.ratesDps) ||
      std::abs(sample.anglesDegrees.x) > 180 || std::abs(sample.anglesDegrees.y) > 90 ||
      std::abs(sample.ratesDps.x) > 2000 || std::abs(sample.ratesDps.y) > 2000 || std::abs(sample.ratesDps.z) > 2000) {
    return unavailable(Reason::Invalid);
  }
  constexpr float radians = 0.017453292519943295f;
  const float roll = sample.anglesDegrees.x * radians, pitch = sample.anglesDegrees.y * radians;
  // WIT Z-Y-X angles describe world-up in sensor coordinates. Rotate the gravity
  // vector, not Euler components: swapping angles is wrong for a rotated mount.
  const Vector down = toBody(s, {std::sin(pitch), -std::sin(roll)*std::cos(pitch), -std::cos(roll)*std::cos(pitch)});
  const Vector rates = toBody(s, sample.ratesDps);
  result.rollDegrees = std::atan2(down.y, down.z) / radians;
  result.pitchDegrees = std::atan2(-down.x, std::hypot(down.y, down.z)) / radians;
  result.rollRateDps = rates.x;
  result.pitchRateDps = rates.y;
  result.bodyValid = true;
  const float horizontalSquared = down.y*down.y + down.z*down.z;
  if (horizontalSquared > 0.000001f) {
    const float yaw = sample.anglesDegrees.z * radians;
    const float cr=std::cos(roll), sr=std::sin(roll), cp=std::cos(pitch), sp=std::sin(pitch);
    const float cy=std::cos(yaw), sy=std::sin(yaw);
    const float sensorForwardX[] = {cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr};
    const float sensorForwardY[] = {sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr};
    const int axis=s.sensorToBody[0], column=(axis>0?axis:-axis)-1;
    const int sign=axis>0?1:-1;
    // Local heading, clockwise positive. No magnetic-north reference is implied.
    result.yawDegrees=std::atan2(-sign*sensorForwardY[column],sign*sensorForwardX[column])/radians;
    result.yawRateDps=(rates.y*down.y+rates.z*down.z)/horizontalSquared;
    result.yawProjection=down.z;
    result.headingValid=true;
  }
  // Mapped pose remains visible for bench calibration even with assistance off.
  if (result.reason != Reason::Active) return result;
  result.pitchActive = result.pitchCalibrated && !manualPitch;
  result.rollActive = result.rollCalibrated;
  const auto correction = [&](float angle, float trim, float rate, float kp, float kd, int sign) {
    const float error = std::remainder(trim - angle, 360.0f);
    return std::clamp(sign * (kp * error - kd * rate), -s.maximumCorrectionPercent, s.maximumCorrectionPercent);
  };
  if (result.pitchActive) result.pitchCorrectionPercent = correction(result.pitchDegrees, s.pitchTrimDegrees,
      result.pitchRateDps, s.pitchKp, s.pitchKd, s.pitchCommandSign);
  if (result.rollActive) result.rollCorrectionPercent = correction(result.rollDegrees, s.rollTrimDegrees,
      result.rollRateDps, s.rollKp, s.rollKd, s.rollCommandSign);
  result.reason = result.pitchActive || result.rollActive ? Reason::Active : Reason::ManualPitch;
  return result;
}

// Keep a relative heading only within one continuous, authorized forward segment.
// Manual steering, stale/invalid data and STOP drop the old reference. Re-enabling
// assistance also relatches, even if OFF/ON were consumed within one control tick.
inline void holdHeading(const Settings &s, Result &result, HeadingState &state,
                        bool manualYaw, uint32_t generation) {
  if (state.generation != generation) state={0,generation,false};
  result.yawActive=result.headingLocked=false;
  result.yawCorrectionPercent=result.headingTargetDegrees=0;
  result.manualYaw=manualYaw;
  const bool eligible=result.reason==Reason::Active || result.reason==Reason::ManualPitch;
  if (!eligible || !result.yawCalibrated || !result.headingValid || manualYaw) {
    state.locked=false;
    if (eligible && !result.pitchActive && !result.rollActive && result.yawCalibrated)
      result.reason=manualYaw?Reason::ManualYaw:Reason::HeadingUnavailable;
    return;
  }
  if (!state.locked) { state.targetDegrees=result.yawDegrees; state.locked=true; }
  result.headingLocked=result.yawActive=true;
  result.headingTargetDegrees=state.targetDegrees;
  const float error=std::remainder(state.targetDegrees-result.yawDegrees,360.0f);
  const float correction=s.yawCommandSign*(s.yawKp*error-s.yawKd*result.yawRateDps)*result.yawProjection;
  result.yawCorrectionPercent=std::clamp(correction,-s.maximumCorrectionPercent,s.maximumCorrectionPercent);
  result.reason=Reason::Active;
}
}  // namespace robofish::attitude
