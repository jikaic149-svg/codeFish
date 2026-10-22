#pragma once

#include "config/MotionAllocationConfig.h"
#include "config/messages/MotionPlan.h"

namespace robofish::device::tail {

struct Motion {
  float amplitudeDegrees = 0;
  float frequencyHz = 0;
  float biasDegrees = 0;
  uint32_t limits = 0;
};

// Pure scheduling; the parent owns phase integration and the startup envelope.
Motion schedule(const robofish::motion::Request &request, const robofish::motion::TailRecipe &recipe, float previousAmplitudeDegrees);

struct Output {
  float targetFront = 0;
  float targetRear = 0;
  float front = 0;
  float rear = 0;
  float load = 0;
  uint32_t limits = 0;
};

Output allocate(const robofish::motion::Request &request, const robofish::motion::TailRecipe &recipe,
                const robofish::motion::Geometry &geometry, const ActuatorFrame &lastCommitted,
                double phaseRad, float amplitudeDegrees, float biasDegrees, uint32_t slewElapsedMs);

}  // namespace robofish::device::tail
