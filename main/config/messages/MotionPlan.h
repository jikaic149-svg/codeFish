#pragma once

#include "config/messages/ActuatorFrame.h"
#include "config/messages/MotionTypes.h"

namespace robofish::motion {

// Body requests are percentages, not measured forces, rates or attitude targets.
// +surge is forward, +yaw is right; pitch/roll mapping is explicitly calibrated.
struct Request {
  DriveMode mode = DriveMode::Idle;
  float surgePercent = 0;
  float yawPercent = 0;
  float pitchPercent = 0;
  float rollPercent = 0;
  bool enabled = false;
  bool allowFlowAssist = false;
  bool immediate = false;  // Existing immediate stop/failsafe path; bypass slew only.
  float outputScale = 1;
  float propulsionScale = 1; // [0,1], applies after prop mixing; leaves tail and fins unchanged.
  // Preserve independent tail/prop drive demands from existing upstream controls.
  // Nodes do not know whether the caller is a radio, vision or manual source.
  bool independentDrives = false;
  float tailDrivePercent = 0;
  float propulsionDrivePercent = 0;
  bool tailEnabled = true;
  bool propulsionEnabled = true;
  bool independentTailYaw = false;
  float tailYawPercent = 0;
  float brakePercent = 0;  // Independent [0,100], never an authorization or STOP.
};

enum Limit : uint32_t {
  PropulsionLimited = 1,
  TailLimited = 2,
  PectoralLimited = 4,
  ReverseUnsupported = 8,
  InvalidInput = 16,
  TailRateLimited = 32,
  PropulsionRateLimited = 64,
  PectoralRateLimited = 128
};

struct Result {
  ActuatorFrame targetFrame;     // Geometrically feasible, before per-tick slew.
  ActuatorFrame candidateFrame;  // Nominal output offered to Device/BSP, never committed here.
  double phaseRad = 0;
  float commonPropulsionPercent = 0;
  float differentialPropulsionPercent = 0;
  float tailLoad = 0;
  float pectoralCorrectionScale = 1;
  uint32_t limitedReasons = 0;
  bool unsupportedReverse = false;
};

}  // namespace robofish::motion
