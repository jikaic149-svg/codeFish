#pragma once

#include "config/MotionAllocationConfig.h"
#include "config/messages/MotionPlan.h"
#include "APP/Algorithm/GaitPhase.h"

namespace robofish::motion {

// The owner keeps one clock and one amplitude envelope. Successful nominal
// outputs are supplied separately after Device/BSP reports its per-channel written mask.
struct State {
  algorithm::GaitPhase gait;
  float tailAmplitudeDegrees = 0;
  float tailBiasDegrees = 0;
  float propulsionDirection = 0;  // Last non-braking drive sign, not measured speed.
};
// Compose the selected motion behavior: tail / propulsion / pectoral. It is the sole
// caller of actuator target planners; Task performs physical frame submission.
// All output angles remain nominal; electrical installation polarity is Device/BSP's job.
Result allocate(const Request &request, const Recipe &recipe,
                const Geometry &geometry, const ActuatorFrame &lastCommitted,
                State &state, uint32_t nowMs, uint32_t elapsedMs);

}  // namespace robofish::motion
