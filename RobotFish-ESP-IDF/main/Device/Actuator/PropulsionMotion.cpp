#include "Device/Actuator/PropulsionMotion.h"

#include <algorithm>
#include <cmath>

namespace robofish::device::propulsion {
namespace {
int approach(int previous, int target, float step, float stopStep) {
  // A normal polarity change must successfully submit neutral before reversing.
  if ((previous > 0 && target < 0) || (previous < 0 && target > 0)) target = 0;
  if (target == 0) step = stopStep;
  // Integer nominal commands have 1% resolution. Floor the allowed step so
  // truncation toward zero cannot exceed the rate limit while decelerating.
  // ponytail: sub-1% steps hold; use a float nominal frame if finer ramps are needed.
  const int wholeStep = static_cast<int>(std::min(200.0f, std::floor(step)));
  return previous + std::clamp(target - previous, -wholeStep, wholeStep);
}
}  // namespace

Output allocate(const robofish::motion::Request &request, const robofish::motion::PropulsionRecipe &recipe,
                const ActuatorFrame &lastCommitted, uint32_t slewElapsedMs,
                float brakingDirection) {
  Output output;
  if (request.enabled && request.propulsionEnabled &&
      (request.mode == DriveMode::Prop || request.mode == DriveMode::Hybrid)) {
    const float drive = request.independentDrives ? request.propulsionDrivePercent : request.surgePercent;
    const float brake = request.brakePercent * 0.01f;
    const float requestedCommon = drive * recipe.commandScale * (1 - brake) -
        brakingDirection * recipe.brakeMaximumReversePercent * brake;
    float common = std::clamp(requestedCommon, -100.0f, 100.0f);
    const float requestedDifferential = request.yawPercent * recipe.yawGain;
    float differential;
    if (brake > 0) {
      // The braking cap applies to EACH opposing prop, not only their average.
      const float lower = brakingDirection > 0 ? -recipe.brakeMaximumReversePercent : -100;
      const float upper = brakingDirection > 0 ? 100 : recipe.brakeMaximumReversePercent;
      common = std::clamp(common, lower, upper);
      // Preserve explicit braking; yaw may only consume its remaining authority.
      const float margin = std::min(common - lower, upper - common);
      differential = std::clamp(requestedDifferential, -margin, margin);
    } else {
      // Steering has priority over ordinary common drive, including at full throttle.
      differential = std::clamp(requestedDifferential, -100.0f, 100.0f);
      const float margin = 100.0f - std::abs(differential);
      common = std::clamp(common, -margin, margin);
    }
    if (common != requestedCommon || differential != requestedDifferential)
      output.limits |= robofish::motion::PropulsionLimited;
    output.targetLeft = static_cast<int>((common + differential) * request.outputScale * request.propulsionScale);
    output.targetRight = static_cast<int>((common - differential) * request.outputScale * request.propulsionScale);
    if (request.propulsionScale < 1) output.limits |= robofish::motion::PropulsionLimited;
  }
  const float step = recipe.ratePercentPerSecond * slewElapsedMs * 0.001f;
  const float stopStep = recipe.stopRatePercentPerSecond * slewElapsedMs * 0.001f;
  output.left = request.immediate ? output.targetLeft :
      approach(lastCommitted.leftPropulsionPercent, output.targetLeft, step, stopStep);
  output.right = request.immediate ? output.targetRight :
      approach(lastCommitted.rightPropulsionPercent, output.targetRight, step, stopStep);
  const int limit = static_cast<int>(100 * request.propulsionScale);
  output.left = std::clamp(output.left, -limit, limit);
  output.right = std::clamp(output.right, -limit, limit);
  if (output.left != output.targetLeft || output.right != output.targetRight)
    output.limits |= robofish::motion::PropulsionRateLimited;
  output.common = (output.left + output.right) * 0.5f;
  output.differential = (output.left - output.right) * 0.5f;
  return output;
}

}  // namespace robofish::device::propulsion
