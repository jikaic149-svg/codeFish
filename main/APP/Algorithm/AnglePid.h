#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace robofish::attitude {
struct PidState {
  float integralPercent = 0, previousErrorDegrees = 0;
  uint32_t previousMs = 0;
  float pPercent = 0, dPercent = 0, outputPercent = 0;
  bool initialized = false;
};

// The caller supplies a shortest angle error. No deadband; all three attitude
// axes use this same bounded PID and their actual measurement/control clock.
inline bool updateAnglePid(PidState &state, float error, uint32_t nowMs,
                           uint32_t periodMs, float kp, float ki, float kd, float limit) {
  const bool first=!state.initialized;
  const uint32_t elapsed=nowMs-state.previousMs;
  if (!first && elapsed<periodMs) return false;
  const float dt=first?0:elapsed*.001f;
  float delta=std::remainder(error-state.previousErrorDegrees,360.0f);
  if (delta>=180) delta-=360;
  state.pPercent=kp*error;
  state.dPercent=!first && dt>0?kd*delta/dt:0;
  if (ki==0) state.integralPercent=0;
  const float integral=std::clamp(state.integralPercent+ki*error*dt,-limit,limit);
  const float candidate=state.pPercent+integral+state.dPercent;
  if ((candidate<=limit || error<0) && (candidate>=-limit || error>0)) state.integralPercent=integral;
  state.outputPercent=std::clamp(state.pPercent+state.integralPercent+state.dPercent,-limit,limit);
  state.previousErrorDegrees=error; state.previousMs=nowMs; state.initialized=true;
  return true;
}
} // namespace robofish::attitude
