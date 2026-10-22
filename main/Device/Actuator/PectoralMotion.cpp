#include "Device/Actuator/PectoralMotion.h"

#include <algorithm>
#include <cmath>

#include "APP/Algorithm/PairedLimit.h"

namespace robofish::device::pectoral {
namespace {
float approach(float previous, float target, float step) {
  return previous + std::clamp(target - previous, -step, step);
}
}  // namespace

Output allocate(const robofish::motion::Request &request, const robofish::motion::PectoralRecipe &recipe,
                const robofish::motion::Geometry &geometry, const ActuatorFrame &lastCommitted,
                float commonPropulsionFraction, float tailLoad, uint32_t slewElapsedMs) {
  Output output;
  float baseLeft = geometry.pectoralLeft.centerDegrees;
  float baseRight = geometry.pectoralRight.centerDegrees;
  float deltaLeft = 0;
  float deltaRight = 0;
  if (request.enabled) {
    const float pitchFraction = std::abs(request.pitchPercent) * 0.01f;
    baseLeft += pitchFraction * (request.pitchPercent >= 0 ?
        recipe.leftPositivePitchDegrees : recipe.leftNegativePitchDegrees);
    baseRight += pitchFraction * (request.pitchPercent >= 0 ?
        recipe.rightPositivePitchDegrees : recipe.rightNegativePitchDegrees);
    deltaLeft = request.rollPercent * 0.01f * recipe.leftRollDegrees +
        std::max(0.0f, -request.yawPercent) * 0.01f * recipe.leftYawDegrees +
        commonPropulsionFraction * recipe.leftSurgeTrimDegrees + tailLoad * recipe.leftTailTrimDegrees;
    deltaRight = request.rollPercent * 0.01f * recipe.rightRollDegrees +
        std::max(0.0f, request.yawPercent) * 0.01f * recipe.rightYawDegrees +
        commonPropulsionFraction * recipe.rightSurgeTrimDegrees + tailLoad * recipe.rightTailTrimDegrees;
  }
  const float boundedLeft = std::clamp(baseLeft,
      geometry.pectoralLeft.minimumDegrees, geometry.pectoralLeft.maximumDegrees);
  const float boundedRight = std::clamp(baseRight,
      geometry.pectoralRight.minimumDegrees, geometry.pectoralRight.maximumDegrees);
  if (boundedLeft != baseLeft || boundedRight != baseRight) output.limits |= robofish::motion::PectoralLimited;
  // robofish::motion::Request strengths weight the offsets; saturation priority is pitch, brake, yaw.
  const float brake = request.enabled ? request.brakePercent * 0.01f : 0;
  const float brakeLeft = brake * recipe.leftBrakeDegrees;
  const float brakeRight = brake * recipe.rightBrakeDegrees;
  const float brakeScale = algorithm::pairedOffsetScale(boundedLeft, brakeLeft,
      geometry.pectoralLeft.minimumDegrees, geometry.pectoralLeft.maximumDegrees,
      boundedRight, brakeRight,
      geometry.pectoralRight.minimumDegrees, geometry.pectoralRight.maximumDegrees);
  const float brakedLeft = boundedLeft + brakeScale * brakeLeft;
  const float brakedRight = boundedRight + brakeScale * brakeRight;
  output.correctionScale = algorithm::pairedOffsetScale(brakedLeft, deltaLeft,
      geometry.pectoralLeft.minimumDegrees, geometry.pectoralLeft.maximumDegrees,
      brakedRight, deltaRight,
      geometry.pectoralRight.minimumDegrees, geometry.pectoralRight.maximumDegrees);
  output.targetLeft = std::clamp(brakedLeft + output.correctionScale * deltaLeft,
      geometry.pectoralLeft.minimumDegrees, geometry.pectoralLeft.maximumDegrees);
  output.targetRight = std::clamp(brakedRight + output.correctionScale * deltaRight,
      geometry.pectoralRight.minimumDegrees, geometry.pectoralRight.maximumDegrees);
  if (brakeScale < 1 || output.correctionScale < 1) output.limits |= robofish::motion::PectoralLimited;
  output.correctionScale = std::min(brakeScale, output.correctionScale);
  const float step = recipe.rateDegreesPerSecond * slewElapsedMs * 0.001f;
  output.left = request.immediate ? output.targetLeft :
      approach(lastCommitted.pectoralLeftDegrees, output.targetLeft, step);
  output.right = request.immediate ? output.targetRight :
      approach(lastCommitted.pectoralRightDegrees, output.targetRight, step);
  if (output.left != output.targetLeft || output.right != output.targetRight)
    output.limits |= robofish::motion::PectoralRateLimited;
  return output;
}

}  // namespace robofish::device::pectoral
