#pragma once

#include "config/MotionAllocationConfig.h"
#include "config/messages/MotionPlan.h"

namespace robofish::device::pectoral {

struct Output {
  float targetLeft = 0;
  float targetRight = 0;
  float left = 0;
  float right = 0;
  float correctionScale = 1;
  uint32_t limits = 0;
};

// Feed-forward values are passed by the parent. This leaf never calls siblings.
Output allocate(const robofish::motion::Request &request, const robofish::motion::PectoralRecipe &recipe,
                const robofish::motion::Geometry &geometry, const ActuatorFrame &lastCommitted,
                float commonPropulsionFraction, float tailLoad, uint32_t slewElapsedMs);

}  // namespace robofish::device::pectoral
