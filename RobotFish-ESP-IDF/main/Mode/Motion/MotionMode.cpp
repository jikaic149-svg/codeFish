#include "Mode/Motion/MotionMode.h"

#include <algorithm>
#include <cmath>
#include <initializer_list>

#include "Device/Actuator/PectoralMotion.h"
#include "Device/Actuator/PropulsionMotion.h"
#include "Device/Actuator/TailMotion.h"

namespace robofish::motion {
namespace {
bool finiteValues(std::initializer_list<float> values) {
  for (const float value : values) if (!std::isfinite(value)) return false;
  return true;
}
bool validJoint(const JointGeometry &joint) {
  return finiteValues({joint.centerDegrees, joint.minimumDegrees, joint.maximumDegrees}) &&
      joint.minimumDegrees <= joint.centerDegrees && joint.centerDegrees <= joint.maximumDegrees;
}
bool validGeometry(const Geometry &geometry) {
  return validJoint(geometry.tailFront) && validJoint(geometry.tailRear) &&
      validJoint(geometry.pectoralLeft) && validJoint(geometry.pectoralRight);
}
bool validRecipe(const Recipe &recipe) {
  const auto &tail = recipe.tail;
  const auto &prop = recipe.propulsion;
  const auto &fin = recipe.pectoral;
  return finiteValues({tail.amplitudeDegrees, tail.rearAmplitudeRatio,
      tail.phaseLagRad, tail.turnBiasDegrees, tail.staticSteeringDegrees, tail.rateDegreesPerSecond,
      tail.forwardMaximumFrequencyHz, tail.forwardSpeedFraction,
      prop.commandScale, prop.yawGain, prop.ratePercentPerSecond, prop.brakeMaximumReversePercent,
      fin.leftPositivePitchDegrees, fin.rightPositivePitchDegrees,
      fin.leftNegativePitchDegrees, fin.rightNegativePitchDegrees,
      fin.leftRollDegrees, fin.rightRollDegrees, fin.leftYawDegrees, fin.rightYawDegrees,
      fin.leftBrakeDegrees, fin.rightBrakeDegrees,
      fin.leftSurgeTrimDegrees, fin.rightSurgeTrimDegrees,
      fin.leftTailTrimDegrees, fin.rightTailTrimDegrees, fin.rateDegreesPerSecond,
      recipe.flowMinimumPercent, recipe.diveDragCompensationPercent,
      recipe.turnForwardReductionFraction}) &&
      tail.amplitudeDegrees >= 0 && tail.rateDegreesPerSecond >= 0 &&
      tail.forwardMaximumFrequencyHz >= 0 &&
      tail.forwardSpeedFraction > 0 && tail.forwardSpeedFraction < 1 &&
      prop.commandScale >= 0 && prop.ratePercentPerSecond >= 0 && fin.rateDegreesPerSecond >= 0 &&
      prop.brakeMaximumReversePercent >= 0 && prop.brakeMaximumReversePercent <= 100 &&
      recipe.turnForwardReductionFraction >= 0 && recipe.turnForwardReductionFraction <= 1 &&
      recipe.flowMinimumPercent >= 0 && recipe.flowMinimumPercent <= 100 &&
      recipe.diveDragCompensationPercent >= 0;
}
bool validRequest(const Request &request) {
  return finiteValues({request.surgePercent, request.yawPercent, request.pitchPercent,
      request.rollPercent, request.tailDrivePercent, request.propulsionDrivePercent,
      request.tailYawPercent, request.outputScale, request.brakePercent}) &&
      request.outputScale >= 0 && request.outputScale <= 1 &&
      request.brakePercent >= 0 && request.brakePercent <= 100 &&
      (request.mode == DriveMode::Idle || request.mode == DriveMode::Bionic ||
       request.mode == DriveMode::Prop || request.mode == DriveMode::Hybrid);
}
bool validHistory(const ActuatorFrame &frame, const Geometry &geometry) {
  const auto within = [](float value, const JointGeometry &joint) {
    return std::isfinite(value) && value >= joint.minimumDegrees && value <= joint.maximumDegrees;
  };
  return within(frame.tailFrontDegrees, geometry.tailFront) &&
      within(frame.tailRearDegrees, geometry.tailRear) &&
      within(frame.pectoralLeftDegrees, geometry.pectoralLeft) &&
      within(frame.pectoralRightDegrees, geometry.pectoralRight) &&
      frame.leftPropulsionPercent >= -100 && frame.leftPropulsionPercent <= 100 &&
      frame.rightPropulsionPercent >= -100 && frame.rightPropulsionPercent <= 100;
}
ActuatorFrame neutralFrame(const Geometry &geometry) {
  ActuatorFrame frame;
  frame.tailFrontDegrees = geometry.tailFront.centerDegrees;
  frame.tailRearDegrees = geometry.tailRear.centerDegrees;
  frame.pectoralLeftDegrees = geometry.pectoralLeft.centerDegrees;
  frame.pectoralRightDegrees = geometry.pectoralRight.centerDegrees;
  return frame;
}
float percent(float value) { return std::clamp(value, -100.0f, 100.0f); }
}  // namespace

Result allocate(const Request &input, const Recipe &recipe,
                const Geometry &geometry, const ActuatorFrame &lastCommitted,
                State &state, uint32_t nowMs, uint32_t elapsedMs) {
  Result result;
  const bool geometryValid = validGeometry(geometry);
  if (!geometryValid || !validRecipe(recipe) || !validRequest(input) ||
      !validHistory(lastCommitted, geometry) ||
      (state.gait.initialized && !std::isfinite(state.gait.radians)) ||
      !std::isfinite(state.tailAmplitudeDegrees) || state.tailAmplitudeDegrees < 0 ||
      !std::isfinite(state.tailBiasDegrees) || !std::isfinite(state.propulsionDirection) ||
      (state.propulsionDirection != 0 && std::abs(state.propulsionDirection) != 1)) {
    // Invalid data is explicit and never reaches sin(), clamp(lo>hi), or Device/BSP as NaN.
    // Invalid geometry has no usable neutral; return the existing frame defaults.
    result.targetFrame = geometryValid ? neutralFrame(geometry) : ActuatorFrame{};
    result.candidateFrame = result.targetFrame;
    result.limitedReasons = InvalidInput;
    result.phaseRad = std::isfinite(state.gait.radians) ? state.gait.radians : 0;
    return result;
  }

  Request request = input;
  request.surgePercent = percent(request.surgePercent);
  request.yawPercent = percent(request.yawPercent);
  request.pitchPercent = percent(request.pitchPercent);
  request.rollPercent = percent(request.rollPercent);
  request.tailDrivePercent = percent(request.tailDrivePercent);
  request.propulsionDrivePercent = percent(request.propulsionDrivePercent);
  request.tailYawPercent = percent(request.tailYawPercent);
  if (request.mode == DriveMode::Idle) request.enabled = false;

  const bool tailDriveActive = request.tailEnabled &&
      (request.mode == DriveMode::Bionic || request.mode == DriveMode::Hybrid);
  const bool propDriveActive = request.propulsionEnabled &&
      (request.mode == DriveMode::Prop || request.mode == DriveMode::Hybrid);
  const float requestedTail = tailDriveActive ?
      (request.independentDrives ? request.tailDrivePercent : request.surgePercent) : 0;
  const float requestedProp = propDriveActive ?
      (request.independentDrives ? request.propulsionDrivePercent : request.surgePercent) : 0;
  // Only drives that this mode will actually execute establish its direction.
  // A disabled/stale prop request cannot turn a pure-tail forward action into
  // an unsupported reverse, nor can Prop's unused tail drive disable its rudder.
  const bool backward = requestedTail < 0 || requestedProp < 0;
  if (request.enabled && request.mode == DriveMode::Bionic && backward) {
    // Reject the whole unsupported pure-tail action. No old horizontal request,
    // yaw-driven oscillation, pitch request, or flow assist survives this request.
    request.enabled = false;
    request.allowFlowAssist = false;
    result.unsupportedReverse = true;
    result.limitedReasons |= ReverseUnsupported;
  } else if (request.enabled && backward) {
    // Prop/Hybrid reverse is provided by the reversible props. A static tail
    // steering law in reversed flow has not been calibrated for this machine.
    request.tailEnabled = false;
    request.allowFlowAssist = false;
  }
  if (!request.enabled) {
    request.allowFlowAssist = false;
    request.surgePercent = request.yawPercent = 0;
    request.pitchPercent = request.rollPercent = 0;
    request.tailDrivePercent = request.propulsionDrivePercent = 0;
    request.tailYawPercent = 0;
    request.brakePercent = 0;
  }

  bool flowAssisted = false;
  if (request.enabled && !backward && request.brakePercent == 0) {
    const float minimum = request.allowFlowAssist && request.pitchPercent != 0 ?
        std::abs(request.pitchPercent) * recipe.flowMinimumPercent * 0.01f : 0;
    const float drag = std::abs(request.pitchPercent) * recipe.diveDragCompensationPercent * 0.01f;
    const auto assistedDrive = [&](float drive, bool active) {
      if (!active) return drive;
      if (drive < minimum) {
        drive = minimum;
        flowAssisted = true;
      }
      return drive > 0 ? std::min(100.0f, drive + drag) : drive;
    };
    if (request.independentDrives) {
      // Each active node retains its own upstream contribution. In particular,
      // body.surge is descriptive here and must not overwrite a 20/80 split.
      request.tailDrivePercent = assistedDrive(request.tailDrivePercent, tailDriveActive);
      request.propulsionDrivePercent = assistedDrive(request.propulsionDrivePercent, propDriveActive);
    } else {
      request.surgePercent = assistedDrive(request.surgePercent, tailDriveActive || propDriveActive);
    }
  }

  float tailDrive = request.independentDrives ? request.tailDrivePercent : request.surgePercent;
  float propDrive = request.independentDrives ? request.propulsionDrivePercent : request.surgePercent;
  const float tailYaw = request.independentTailYaw ? request.tailYawPercent : request.yawPercent;
  if (!request.independentDrives && request.mode == DriveMode::Bionic && tailDrive == 0)
    tailDrive = std::abs(tailYaw);
  // Retain the non-braking requested direction, never the opposing brake output.
  // This is an open-loop intent estimate; no measured stopping speed is implied.
  if (propDriveActive && propDrive != 0 &&
      (request.brakePercent == 0 || state.propulsionDirection == 0))
    state.propulsionDirection = std::copysign(1.0f, propDrive);
  if (state.propulsionDirection == 0 && request.brakePercent > 0) {
    const int previousCommon = lastCommitted.leftPropulsionPercent + lastCommitted.rightPropulsionPercent;
    state.propulsionDirection = previousCommon < 0 ? -1.0f : 1.0f;
  }
  const float yawFraction = std::abs(request.yawPercent) * 0.01f;
  const float forwardScale = 1 - recipe.turnForwardReductionFraction * yawFraction * yawFraction;
  if (request.mode == DriveMode::Hybrid && tailDrive > 0) tailDrive *= forwardScale;
  if (propDrive > 0) propDrive *= forwardScale;
  // Normalize source choices once; no zero-drive fallback may restart a braked tail.
  request.independentDrives = true;
  request.tailDrivePercent = tailDrive * (1 - request.brakePercent * 0.01f);
  request.propulsionDrivePercent = propDrive;

  const uint32_t slewMs = std::min(elapsedMs, recipe.maximumSlewElapsedMs);
  const auto motion = device::tail::schedule(request, recipe.tail, state.tailAmplitudeDegrees);
  result.limitedReasons |= motion.limits;
  const float minimumBias = std::max(geometry.tailFront.minimumDegrees - geometry.tailFront.centerDegrees,
      geometry.tailRear.minimumDegrees - geometry.tailRear.centerDegrees);
  const float maximumBias = std::min(geometry.tailFront.maximumDegrees - geometry.tailFront.centerDegrees,
      geometry.tailRear.maximumDegrees - geometry.tailRear.centerDegrees);
  const float desiredBias = std::clamp(motion.biasDegrees, minimumBias, maximumBias);
  if (desiredBias != motion.biasDegrees) result.limitedReasons |= TailLimited;
  const bool periodic = motion.amplitudeDegrees > 0 && motion.frequencyHz > 0;
  const float transitionBudget = recipe.tail.rateDegreesPerSecond * slewMs * 0.001f *
      (periodic ? 1 - recipe.tail.forwardSpeedFraction : 1);
  const float biasChange = std::clamp(desiredBias - state.tailBiasDegrees,
      -transitionBudget, transitionBudget);
  if (!request.enabled || !request.tailEnabled) state.tailBiasDegrees = 0;
  else state.tailBiasDegrees = request.immediate ? desiredBias : state.tailBiasDegrees + biasChange;
  if (state.tailBiasDegrees != desiredBias) result.limitedReasons |= TailRateLimited;
  if (motion.amplitudeDegrees > 0 && motion.frequencyHz > 0) {
    const bool starting = !state.gait.initialized;
    // One parent-owned clock for both joints, including uint32 tick wraparound.
    result.phaseRad = algorithm::advanceGait(state.gait, nowMs, motion.frequencyHz);
    if (!request.immediate) {
      const float envelopeStep = std::max(0.0f, transitionBudget - std::abs(biasChange)) /
          std::max(1.0f, std::abs(recipe.tail.rearAmplitudeRatio));
      state.tailAmplitudeDegrees = starting ? 0 : state.tailAmplitudeDegrees +
          std::clamp(motion.amplitudeDegrees - state.tailAmplitudeDegrees,
                     -envelopeStep, envelopeStep);
      if (state.tailAmplitudeDegrees != motion.amplitudeDegrees)
        result.limitedReasons |= TailRateLimited;
    } else {
      state.tailAmplitudeDegrees = motion.amplitudeDegrees;
    }
  } else {
    // STOP/disabled/reverse has no residual periodic motion; the next gait restarts at zero.
    state.gait = {};
    state.tailAmplitudeDegrees = 0;
  }
  const auto tail = device::tail::allocate(request, recipe.tail, geometry, lastCommitted,
      result.phaseRad, state.tailAmplitudeDegrees, state.tailBiasDegrees, slewMs);
  const auto propulsion = device::propulsion::allocate(request, recipe.propulsion, lastCommitted, slewMs,
      state.propulsionDirection == 0 ? 1 : state.propulsionDirection);
  const auto pectoral = device::pectoral::allocate(request, recipe.pectoral, geometry, lastCommitted,
      (propulsion.targetLeft + propulsion.targetRight) * 0.005f, tail.load, slewMs);

  result.targetFrame = neutralFrame(geometry);
  result.targetFrame.tailFrontDegrees = tail.targetFront;
  result.targetFrame.tailRearDegrees = tail.targetRear;
  result.targetFrame.pectoralLeftDegrees = pectoral.targetLeft;
  result.targetFrame.pectoralRightDegrees = pectoral.targetRight;
  result.targetFrame.leftPropulsionPercent = propulsion.targetLeft;
  result.targetFrame.rightPropulsionPercent = propulsion.targetRight;
  result.targetFrame.flowAssisted = flowAssisted;
  result.candidateFrame = result.targetFrame;
  result.candidateFrame.tailFrontDegrees = tail.front;
  result.candidateFrame.tailRearDegrees = tail.rear;
  result.candidateFrame.pectoralLeftDegrees = pectoral.left;
  result.candidateFrame.pectoralRightDegrees = pectoral.right;
  result.candidateFrame.leftPropulsionPercent = propulsion.left;
  result.candidateFrame.rightPropulsionPercent = propulsion.right;
  result.commonPropulsionPercent = propulsion.common;
  result.differentialPropulsionPercent = propulsion.differential;
  result.tailLoad = tail.load;
  result.pectoralCorrectionScale = pectoral.correctionScale;
  result.limitedReasons |= tail.limits | propulsion.limits | pectoral.limits;
  // Finite individual coefficients can still overflow when added together.
  // Keep that failure explicit rather than offering a non-finite PWM target.
  if (!validHistory(result.targetFrame, geometry) || !validHistory(result.candidateFrame, geometry)) {
    result.targetFrame = result.candidateFrame = neutralFrame(geometry);
    result.commonPropulsionPercent = result.differentialPropulsionPercent = result.tailLoad = 0;
    result.pectoralCorrectionScale = 1;
    result.limitedReasons |= InvalidInput;
  }
  return result;
}

}  // namespace robofish::motion
