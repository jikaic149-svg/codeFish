#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include "APP/Algorithm/AnglePid.h"

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
  float pitchKp = 0, pitchKd = 0, rollKp = 0, rollKd = 0, yawKp = 0, yawKi = 0, yawKd = 0;
  float pitchKi = 0, rollKi = 0;
  bool pitchEnabled = false, rollEnabled = false;
  float pitchTrimDegrees = 0, rollTrimDegrees = 0;
  float maximumCorrectionPercent = 0;
  float maximumYawCorrectionPercent = 0;
  uint32_t maximumAgeMs = 0;
  uint32_t yawPeriodMs = 0;
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
  float yawErrorDegrees = 0, yawP = 0, yawI = 0, yawD = 0;
  float pitchPreviewPercent = 0, rollPreviewPercent = 0;
  bool tiltPreviewValid = false;
  uint32_t sampleMs = 0; // Oldest measurement actually used by this control tick.
};
struct HeadingState {
  float targetDegrees = 0;
  uint32_t generation = 0;
  bool locked = false;
  PidState pid{};
  float correctionPercent = 0;
};
struct TiltState { PidState pitch, roll; uint32_t generation = 0; };

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
      s.yawPeriodMs > 0 && s.yawPeriodMs < 0x80000000u &&
      std::isfinite(s.maximumCorrectionPercent) && s.maximumCorrectionPercent > 0 && s.maximumCorrectionPercent <= 100 &&
      std::isfinite(s.maximumYawCorrectionPercent) && s.maximumYawCorrectionPercent > 0 && s.maximumYawCorrectionPercent <= 100 &&
      std::isfinite(s.pitchKp) && s.pitchKp >= 0 && s.pitchKp <= 100 &&
      std::isfinite(s.pitchKd) && s.pitchKd >= 0 && s.pitchKd <= 100 &&
      std::isfinite(s.pitchKi) && s.pitchKi >= 0 && s.pitchKi <= 100 &&
      std::isfinite(s.rollKp) && s.rollKp >= 0 && s.rollKp <= 100 &&
      std::isfinite(s.rollKd) && s.rollKd >= 0 && s.rollKd <= 100 &&
      std::isfinite(s.rollKi) && s.rollKi >= 0 && s.rollKi <= 100 &&
      std::isfinite(s.yawKp) && s.yawKp >= 0 && s.yawKp <= 100 &&
      std::isfinite(s.yawKi) && s.yawKi >= 0 && s.yawKi <= 100 &&
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
// One signed shortest turn in [-180, 180). A half-turn always chooses -180,
// independent of whether the sensor encoded the boundary as +180 or -180.
inline float wrapDegrees(float degrees) {
  const float wrapped = std::remainder(degrees, 360.0f);
  return wrapped >= 180.0f ? wrapped - 360.0f : wrapped;
}

struct AngleFilter {
  Vector first, second;
  uint32_t sampleMs = 0;
  bool initialized = false;
};
// Two cascaded low-pass stages, after the sensor's own processing. Roll/yaw
// interpolate along the shortest arc; averaging +179 and -179 must stay near 180.
// Repeated samples do not advance time, and a measurement gap starts fresh.
inline Sample filterAngles(Sample sample, AngleFilter &state, float stageHz, uint32_t gapMs) {
  const auto angles=sample.anglesDegrees;
  if (!sample.ready || !sample.hasAngles || !finite(angles) ||
      std::abs(angles.x)>180 || std::abs(angles.y)>90 || std::abs(angles.z)>180 ||
      !std::isfinite(stageHz) || stageHz<=0) {
    state.initialized=false; return sample;
  }
  const uint32_t elapsed=sample.anglesMs-state.sampleMs;
  if (!state.initialized || elapsed>gapMs) {
    state.first=state.second=angles; state.initialized=true;
  } else if (elapsed) {
    const float step=6.28318530718f*stageHz*elapsed*.001f;
    const float decay=std::exp(-step),alpha=-std::expm1(-step);
    const auto update=[&](float &first,float &second,float input,bool circular) {
      const float e1=circular?wrapDegrees(input-first):input-first;
      const float e2=circular?wrapDegrees(input-second):input-second;
      // Exact constant-input update of the two cascaded states, using real dt.
      second+=alpha*e2-step*decay*e1;
      first+=alpha*e1;
      if (circular) { first=wrapDegrees(first); second=wrapDegrees(second); }
    };
    update(state.first.x,state.second.x,angles.x,true);
    update(state.first.y,state.second.y,angles.y,false);
    update(state.first.z,state.second.z,angles.z,true);
  }
  state.sampleMs=sample.anglesMs;
  sample.anglesDegrees=state.second;
  return sample;
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
  result.yawCalibrated = settingsValid && calibratedSign(s.yawCommandSign);
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
      std::abs(sample.anglesDegrees.x) > 180 || std::abs(sample.anglesDegrees.y) > 90 || std::abs(sample.anglesDegrees.z) > 180 ||
      std::abs(sample.ratesDps.x) > 2000 || std::abs(sample.ratesDps.y) > 2000 || std::abs(sample.ratesDps.z) > 2000) {
    return unavailable(Reason::Invalid);
  }
  constexpr float radians = 0.017453292519943295f;
  const float roll = sample.anglesDegrees.x * radians, pitch = sample.anglesDegrees.y * radians;
  // WIT Z-Y-X angles describe world-up in sensor coordinates. Rotate the gravity
  // vector, not Euler components: swapping angles is wrong for a rotated mount.
  const Vector down = toBody(s, {std::sin(pitch), -std::sin(roll)*std::cos(pitch), -std::cos(roll)*std::cos(pitch)});
  const Vector rates = toBody(s, sample.ratesDps);
  result.rollDegrees = wrapDegrees(std::atan2(down.y, down.z) / radians);
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
    result.yawDegrees=wrapDegrees(std::atan2(-sign*sensorForwardY[column],sign*sensorForwardX[column])/radians);
    result.yawRateDps=(rates.y*down.y+rates.z*down.z)/horizontalSquared;
    result.yawProjection=down.z;
    result.headingValid=true;
  }
  // Mapped pose remains visible for bench calibration even with assistance off.
  if (result.reason != Reason::Active) return result;
  result.pitchActive = s.pitchEnabled && result.pitchCalibrated && !manualPitch;
  result.rollActive = s.rollEnabled && result.rollCalibrated;
  const auto correction = [&](float angle, float trim, float rate, float kp, float kd, int sign) {
    const float error = wrapDegrees(trim - angle);
    return std::clamp(sign * (kp * error - kd * rate), -s.maximumCorrectionPercent, s.maximumCorrectionPercent);
  };
  if (result.pitchActive) result.pitchCorrectionPercent = correction(result.pitchDegrees, s.pitchTrimDegrees,
      result.pitchRateDps, s.pitchKp, s.pitchKd, s.pitchCommandSign);
  if (result.rollActive) result.rollCorrectionPercent = correction(result.rollDegrees, s.rollTrimDegrees,
      result.rollRateDps, s.rollKp, s.rollKd, s.rollCommandSign);
  result.reason = result.pitchActive || result.rollActive ? Reason::Active : Reason::ManualPitch;
  return result;
}

// Same PID for pitch/roll, with independent integrals and manual-pitch priority.
// Production keeps both output gates OFF. Preview values are BODY demands,
// not calibrated fin angles/torques. Unknown roll geometry remains unactuated.
inline void updateTiltPid(const Settings &s, Result &result, TiltState &state,
                          bool manualPitch, uint32_t generation, uint32_t nowMs) {
  const bool eligible=result.bodyValid && result.requested &&
      (result.reason==Reason::Active || result.reason==Reason::ManualPitch);
  if (!eligible || generation!=state.generation) state={{},{},generation};
  result.tiltPreviewValid=eligible;
  result.pitchPreviewPercent=result.rollPreviewPercent=0;
  if (!eligible) return;
  if (manualPitch) state.pitch={};
  else {
    updateAnglePid(state.pitch,wrapDegrees(s.pitchTrimDegrees-result.pitchDegrees),nowMs,
        s.yawPeriodMs,s.pitchKp,s.pitchKi,s.pitchKd,s.maximumCorrectionPercent);
    result.pitchPreviewPercent=state.pitch.outputPercent;
  }
  updateAnglePid(state.roll,wrapDegrees(s.rollTrimDegrees-result.rollDegrees),nowMs,
      s.yawPeriodMs,s.rollKp,s.rollKi,s.rollKd,s.maximumCorrectionPercent);
  result.rollPreviewPercent=state.roll.outputPercent;
  if (result.pitchActive) result.pitchCorrectionPercent=s.pitchCommandSign*result.pitchPreviewPercent;
  if (result.rollActive) result.rollCorrectionPercent=s.rollCommandSign*result.rollPreviewPercent;
}

// Keep a relative heading while authorized, including ordinary zero-surge holds.
// Manual steering, stale/invalid data and STOP drop the old reference. Re-enabling
// assistance also relatches, even if OFF/ON were consumed within one control tick.
inline void holdHeading(const Settings &s, Result &result, HeadingState &state,
                        bool manualYaw, uint32_t generation, uint32_t nowMs) {
  if (state.generation != generation) state={0,generation,false};
  result.yawActive=result.headingLocked=false;
  result.yawCorrectionPercent=result.headingTargetDegrees=0;
  result.yawErrorDegrees=result.yawP=result.yawI=result.yawD=0;
  result.manualYaw=manualYaw;
  const bool eligible=result.reason==Reason::Active || result.reason==Reason::ManualPitch;
  if (!eligible || !result.yawCalibrated || !result.headingValid || manualYaw) {
    state={0,generation,false};
    if (eligible && !result.pitchActive && !result.rollActive && result.yawCalibrated)
      result.reason=manualYaw?Reason::ManualYaw:Reason::HeadingUnavailable;
    return;
  }
  const bool first=!state.locked;
  if (first) { state.targetDegrees=result.yawDegrees; state.locked=true; }
  result.headingLocked=result.yawActive=true;
  result.headingTargetDegrees=state.targetDegrees;
  const float error=wrapDegrees(state.targetDegrees-result.yawDegrees);
  result.yawErrorDegrees=error;
  const bool updated=updateAnglePid(state.pid,error,nowMs,s.yawPeriodMs,s.yawKp,s.yawKi,s.yawKd,s.maximumYawCorrectionPercent);
  result.yawP=state.pid.pPercent; result.yawI=state.pid.integralPercent; result.yawD=state.pid.dPercent;
  if (updated) {
    const float correction=s.yawCommandSign*(result.yawP+result.yawI+result.yawD)*result.yawProjection;
    state.correctionPercent=std::clamp(correction,-s.maximumYawCorrectionPercent,s.maximumYawCorrectionPercent);
  }
  result.yawCorrectionPercent=state.correctionPercent;
  result.reason=Reason::Active;
}
}  // namespace robofish::attitude
