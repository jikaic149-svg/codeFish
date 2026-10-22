#pragma once

#include "config/MotionAllocationConfig.h"
#include "config/messages/MotionPlan.h"

namespace robofish::device::propulsion {

struct Output {
  int targetLeft = 0;
  int targetRight = 0;
  int left = 0;
  int right = 0;
  float common = 0;
  float differential = 0;
  uint32_t limits = 0;
};

Output allocate(const robofish::motion::Request &request, const robofish::motion::PropulsionRecipe &recipe,
                const ActuatorFrame &lastCommitted, uint32_t slewElapsedMs,
                float brakingDirection);

}  // namespace robofish::device::propulsion
