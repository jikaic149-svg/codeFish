#pragma once

#include "config/ControlConfig.h"
#include "config/messages/MotionTypes.h"
#include "APP/Algorithm/GaitPhase.h"

namespace robofish::motion {

struct JointGeometry {
  float centerDegrees = 0;
  float minimumDegrees = 0;
  float maximumDegrees = 0;
};
struct Geometry {
  JointGeometry tailFront;
  JointGeometry tailRear;
  JointGeometry pectoralLeft;
  JointGeometry pectoralRight;
};

struct TailRecipe {
  float amplitudeDegrees = 0;
  float rearAmplitudeRatio = 0;
  float phaseLagRad = 0;  // rear = sin(sharedPhase - lag)
  float turnBiasDegrees = 0;
  float staticSteeringDegrees = 0;
  float rateDegreesPerSecond = 0;
  float transitionRateDegreesPerSecond = 0; // Bias/envelope and static start/return speed.
  float forwardMaximumFrequencyHz = 0;
  float forwardSpeedFraction = 0;  // (0,1): gait; remainder shared by bias/amplitude.
};
struct PropulsionRecipe {
  float commandScale = 1;
  float yawGain = 1;
  float ratePercentPerSecond = 0;
  float stopRatePercentPerSecond = 0;
  float brakeMaximumReversePercent = 0;
};
struct PectoralRecipe {
  // Offsets from each geometric center, in the existing absolute-angle encoding.
  float leftPositivePitchDegrees = 0;
  float rightPositivePitchDegrees = 0;
  float leftNegativePitchDegrees = 0;
  float rightNegativePitchDegrees = 0;
  float leftRollDegrees = 0;
  float rightRollDegrees = 0;
  float leftYawDegrees = 0;
  float rightYawDegrees = 0;
  float leftBrakeDegrees = 0;
  float rightBrakeDegrees = 0;
  float leftSurgeTrimDegrees = 0;
  float rightSurgeTrimDegrees = 0;
  float leftTailTrimDegrees = 0;
  float rightTailTrimDegrees = 0;
  float rateDegreesPerSecond = 0;
};
struct Recipe {
  TailRecipe tail;
  PropulsionRecipe propulsion;
  PectoralRecipe pectoral;
  float flowMinimumPercent = 0;
  float diveDragCompensationPercent = 0;
  uint32_t maximumSlewElapsedMs = 0;
  float turnForwardReductionFraction = 0;  // [0,1], reduction at full yaw.
};

}  // namespace robofish::motion

// One hardware profile for all sources. Legacy SDK virtual throttle/steering
// angles and 86-degree tail centers no longer select different physical gaits.
namespace robofish::config::motion {

inline robofish::motion::Geometry makeGeometry() {
  return {
      {lora::kTailFrontCenter, lora::kTailFrontMinimum, lora::kTailFrontMaximum},
      {lora::kTailRearCenter, lora::kTailRearMinimum, lora::kTailRearMaximum},
      {lora::kPectoralLeftCenter, lora::kPectoralLeftMinimum, lora::kPectoralLeftMaximum},
      {lora::kPectoralRightCenter, lora::kPectoralRightMinimum, lora::kPectoralRightMaximum}};
}

inline robofish::motion::Recipe makeRecipe(DriveMode mode) {
  robofish::motion::Recipe recipe;
  const bool hybrid = mode == DriveMode::Hybrid;
  recipe.tail.amplitudeDegrees = lora::kTailAmplitudeDegrees *
      (hybrid ? config::control::kHybridTailCommandScale : 1);
  recipe.tail.forwardMaximumFrequencyHz = lora::kTailForwardMaximumFrequencyHz;
  recipe.tail.forwardSpeedFraction = config::control::kTailForwardSpeedFraction;
  recipe.tail.rearAmplitudeRatio = lora::kRearTailAmplitudeRatio;
  recipe.tail.phaseLagRad = (mode == DriveMode::Bionic ?
      lora::kBionicPhaseLagDegrees : lora::kHybridPhaseLagDegrees) *
      static_cast<float>(algorithm::kTwoPi / 360);
  // These are nominal yaw-to-tail geometric gains, not ESC wiring polarity.
  const float tailYawSign = lora::kTurnDirection * lora::kTailSteeringDirection;
  recipe.tail.turnBiasDegrees = lora::kTailTurnBiasDegrees * tailYawSign;
  recipe.tail.staticSteeringDegrees = lora::kTailStaticSteeringDegrees * tailYawSign;
  recipe.tail.rateDegreesPerSecond = lora::kTailStepPer10Ms * 100;
  recipe.tail.transitionRateDegreesPerSecond = config::control::kTailTransitionDegreesPerSecond;
  recipe.propulsion.commandScale = hybrid ? config::control::kHybridPropulsionCommandScale : 1;
  // FRD: the left prop is at negative y, so positive (right) yaw needs T_L>T_R.
  // Do not reuse the old radio-specific steering sign for this geometric law.
  recipe.propulsion.yawGain = 1;
  recipe.propulsion.ratePercentPerSecond = config::control::kPropulsionRatePercentPerSecond;
  recipe.propulsion.stopRatePercentPerSecond = config::control::kPropulsionStopRatePercentPerSecond;
  recipe.propulsion.brakeMaximumReversePercent = config::control::kBrakeMaximumReversePercent;
  const auto offset = [](int center, int endpoint) {
    // Preserve the existing 132/48, 56/124 endpoint rounding.
    return static_cast<float>(static_cast<int>((endpoint - center) * lora::kPectoralScale));
  };
  recipe.pectoral.leftPositivePitchDegrees = offset(lora::kPectoralLeftCenter, lora::kPectoralLeftUp);
  recipe.pectoral.rightPositivePitchDegrees = offset(lora::kPectoralRightCenter, lora::kPectoralRightUp);
  recipe.pectoral.leftNegativePitchDegrees = offset(lora::kPectoralLeftCenter, lora::kPectoralLeftDown);
  recipe.pectoral.rightNegativePitchDegrees = offset(lora::kPectoralRightCenter, lora::kPectoralRightDown);
  // Roll geometry remains uncalibrated: its gains are explicitly zero.
  recipe.pectoral.leftYawDegrees = config::control::kPectoralLeftYawDegrees;
  recipe.pectoral.rightYawDegrees = config::control::kPectoralRightYawDegrees;
  recipe.pectoral.leftBrakeDegrees = config::control::kPectoralLeftBrakeDegrees;
  recipe.pectoral.rightBrakeDegrees = config::control::kPectoralRightBrakeDegrees;
  recipe.pectoral.leftSurgeTrimDegrees = config::control::kPectoralLeftSurgeTrimDegrees;
  recipe.pectoral.rightSurgeTrimDegrees = config::control::kPectoralRightSurgeTrimDegrees;
  recipe.pectoral.leftTailTrimDegrees = config::control::kPectoralLeftTailTrimDegrees;
  recipe.pectoral.rightTailTrimDegrees = config::control::kPectoralRightTailTrimDegrees;
  recipe.pectoral.rateDegreesPerSecond = lora::kServoStepPer10Ms * 100;
  recipe.flowMinimumPercent = config::control::kFlowMinimumPercent;
  recipe.diveDragCompensationPercent = config::control::kDiveDragCompensationPercent;
  recipe.maximumSlewElapsedMs = config::control::kMaximumSlewElapsedMs;
  recipe.turnForwardReductionFraction = config::control::kTurnForwardReductionFraction;
  return recipe;
}

}  // namespace robofish::config::motion
