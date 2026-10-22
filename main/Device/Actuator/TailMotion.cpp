#include "Device/Actuator/TailMotion.h"

#include <algorithm>
#include <cmath>

namespace robofish::device::tail {
namespace {
float approach(float previous, float target, float step) {
  return previous + std::clamp(target - previous, -step, step);
}
float boundedCenter(const robofish::motion::JointGeometry &joint, float offset, uint32_t &limits) {
  const float requested = joint.centerDegrees + offset;
  const float result = std::clamp(requested, joint.minimumDegrees, joint.maximumDegrees);
  if (result != requested) limits |= robofish::motion::TailLimited;
  return result;
}
}  // namespace

Motion schedule(const robofish::motion::Request &request, const robofish::motion::TailRecipe &recipe, float previousAmplitudeDegrees) {
  Motion motion;
  if (!request.enabled || !request.tailEnabled) return motion;
  const bool oscillating = request.mode == DriveMode::Bionic || request.mode == DriveMode::Hybrid;
  const float yaw = request.independentTailYaw ? request.tailYawPercent : request.yawPercent;
  motion.biasDegrees = yaw * 0.01f *
      (oscillating ? recipe.turnBiasDegrees : recipe.staticSteeringDegrees);
  if (!oscillating) return motion;
  float drive = request.independentDrives ? request.tailDrivePercent : request.surgePercent;
  if (!request.independentDrives && request.mode == DriveMode::Bionic && drive == 0)
    drive = std::abs(yaw);
  const float effective = std::clamp(drive * 0.01f * request.outputScale, 0.0f, 1.0f);
  const float scale = std::sqrt(effective);
  motion.amplitudeDegrees = recipe.amplitudeDegrees * scale;
  // Turning reduces common drive/amplitude, but must not also slow the gait.
  // Keep strength scaling and the existing angular-speed budget for both joints.
  const float turn = std::clamp(std::abs(yaw) * 0.01f * request.outputScale, 0.0f, 1.0f);
  motion.frequencyHz = effective > 0 ?
      recipe.forwardMaximumFrequencyHz * std::sqrt(std::max(effective, turn)) : 0;
  if (motion.amplitudeDegrees > 0) {
    const double peakSpeed = algorithm::kTwoPi * motion.frequencyHz *
        std::max(motion.amplitudeDegrees, previousAmplitudeDegrees) *
        std::max(1.0f, std::abs(recipe.rearAmplitudeRatio));
    const double budget = recipe.rateDegreesPerSecond * recipe.forwardSpeedFraction;
    if (peakSpeed > budget) {
      motion.frequencyHz *= static_cast<float>(budget / peakSpeed);
      motion.limits |= robofish::motion::TailRateLimited;
    }
  }
  return motion;
}

Output allocate(const robofish::motion::Request &request, const robofish::motion::TailRecipe &recipe,
                const robofish::motion::Geometry &geometry, const ActuatorFrame &lastCommitted,
                double phaseRad, float amplitudeDegrees, float biasDegrees, uint32_t slewElapsedMs) {
  Output output;
  output.targetFront = geometry.tailFront.centerDegrees;
  output.targetRear = geometry.tailRear.centerDegrees;
  if (request.enabled && request.tailEnabled) {
    const bool oscillating = request.mode == DriveMode::Bionic || request.mode == DriveMode::Hybrid;
    if (oscillating) {
      // Share joint travel between steering bias and oscillation. Reserving all
      // travel for bias first can make full steering appear to freeze the tail.
      float scale = 1.0f;
      const auto limit = [&](const robofish::motion::JointGeometry &joint, float amplitude) {
        const float high = biasDegrees + amplitude, low = biasDegrees - amplitude;
        if (high > 0) scale = std::min(scale, (joint.maximumDegrees - joint.centerDegrees) / high);
        if (low < 0) scale = std::min(scale, (joint.centerDegrees - joint.minimumDegrees) / -low);
      };
      limit(geometry.tailFront, amplitudeDegrees);
      limit(geometry.tailRear, amplitudeDegrees * std::abs(recipe.rearAmplitudeRatio));
      if (scale < 1) output.limits |= robofish::motion::TailLimited;
      biasDegrees *= scale;
      amplitudeDegrees *= scale;
    }
    const float frontCenter = boundedCenter(geometry.tailFront, biasDegrees, output.limits);
    const float rearCenter = boundedCenter(geometry.tailRear, biasDegrees, output.limits);
    output.targetFront = frontCenter;
    output.targetRear = rearCenter;
    if (oscillating) {
      float available = std::min(frontCenter - geometry.tailFront.minimumDegrees,
                                 geometry.tailFront.maximumDegrees - frontCenter);
      if (recipe.rearAmplitudeRatio != 0) {
        const float rearAvailable = std::min(rearCenter - geometry.tailRear.minimumDegrees,
            geometry.tailRear.maximumDegrees - rearCenter) / std::abs(recipe.rearAmplitudeRatio);
        available = std::min(available, rearAvailable);
      }
      const float amplitude = std::min(amplitudeDegrees, std::max(0.0f, available));
      if (amplitude != amplitudeDegrees) output.limits |= robofish::motion::TailLimited;
      output.load = recipe.amplitudeDegrees > 0 ? amplitude / recipe.amplitudeDegrees : 0;
      output.targetFront += amplitude * static_cast<float>(std::sin(phaseRad));
      output.targetRear += amplitude * recipe.rearAmplitudeRatio *
          static_cast<float>(std::sin(phaseRad - recipe.phaseLagRad));
    }
  }
  // Floating-point roundoff must not leak an out-of-range endpoint to Device/BSP.
  output.targetFront = std::clamp(output.targetFront,
      geometry.tailFront.minimumDegrees, geometry.tailFront.maximumDegrees);
  output.targetRear = std::clamp(output.targetRear,
      geometry.tailRear.minimumDegrees, geometry.tailRear.maximumDegrees);
  const bool swinging = request.enabled && request.tailEnabled && amplitudeDegrees > 0 &&
      (request.mode == DriveMode::Bionic || request.mode == DriveMode::Hybrid);
  const float rate = swinging ? recipe.rateDegreesPerSecond :
      std::min(recipe.rateDegreesPerSecond, recipe.transitionRateDegreesPerSecond);
  const float step = rate * slewElapsedMs * 0.001f;
  output.front = request.immediate ? output.targetFront :
      approach(lastCommitted.tailFrontDegrees, output.targetFront, step);
  output.rear = request.immediate ? output.targetRear :
      approach(lastCommitted.tailRearDegrees, output.targetRear, step);
  if (output.front != output.targetFront || output.rear != output.targetRear)
    output.limits |= robofish::motion::TailRateLimited;
  return output;
}

}  // namespace robofish::device::tail
