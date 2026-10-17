#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

#include "Mode/Motion/MotionMode.h"

using namespace robofish;
static unsigned checks = 0;

static void check(bool value, const char *label) {
  ++checks;
  if (!value) { std::fprintf(stderr, "FAIL: %s\n", label); std::exit(1); }
}

static void commit(ActuatorFrame &history, const ActuatorFrame &candidate, unsigned mask) {
  if (mask & 1) history.tailFrontDegrees = candidate.tailFrontDegrees;
  if (mask & 2) history.tailRearDegrees = candidate.tailRearDegrees;
  if (mask & 4) history.pectoralLeftDegrees = candidate.pectoralLeftDegrees;
  if (mask & 8) history.pectoralRightDegrees = candidate.pectoralRightDegrees;
  if (mask & 16) history.leftPropulsionPercent = candidate.leftPropulsionPercent;
  if (mask & 32) history.rightPropulsionPercent = candidate.rightPropulsionPercent;
}

static motion::Result allocate(const motion::Request &request, ActuatorFrame &history,
    motion::State &state, uint32_t now, uint32_t dt, unsigned writtenMask = 63) {
  const auto result = motion::allocate(request, config::motion::makeRecipe(request.mode),
      config::motion::makeGeometry(), history, state, now, dt);
  commit(history, result.candidateFrame, writtenMask);
  return result;
}

static void behaviorChecks() {
  motion::Request request;
  request.enabled = true;
  request.mode = DriveMode::Hybrid;
  request.surgePercent = 100;
  ActuatorFrame history;
  motion::State state;
  auto result = allocate(request, history, state, 5000, 20);
  check(result.phaseRad == 0, "new gait starts at zero, independently of uptime");
  check(history.leftPropulsionPercent == 10 && history.rightPropulsionPercent == 10,
      "forward outputs remain nominal and rise by 10 percent per 20 ms");
  for (unsigned tick = 1; tick < 20; ++tick) allocate(request, history, state, 5000 + 20 * tick, 20);
  check(history.leftPropulsionPercent == 100 && history.rightPropulsionPercent == 100,
      "both nominal propellers reach forward target");
  request.surgePercent = -100;
  bool submittedNeutral = false;
  for (unsigned tick = 0; tick < 24; ++tick) {
    const int previous = history.leftPropulsionPercent;
    result = allocate(request, history, state, 5400 + 20 * tick, 20);
    if (previous > 0) check(history.leftPropulsionPercent >= 0, "reverse passes through submitted neutral");
    if (history.leftPropulsionPercent == 0) submittedNeutral = true;
    if (history.leftPropulsionPercent < 0) check(submittedNeutral, "negative drive follows a neutral tick");
    check(result.targetFrame.tailFrontDegrees == 90 && result.targetFrame.tailRearDegrees == 90,
        "reverse Hybrid drive disables uncalibrated tail motion");
  }
  check(history.leftPropulsionPercent == -100 && history.rightPropulsionPercent == -100,
      "reverse reaches both nominal propellers");

  request.mode = DriveMode::Bionic;
  history = {}; state = {};
  result = allocate(request, history, state, 10000, 20);
  check(result.unsupportedReverse && (result.limitedReasons & motion::ReverseUnsupported),
      "pure tail reverse is explicitly unsupported");
  check(history.leftPropulsionPercent == 0 && history.rightPropulsionPercent == 0 &&
      history.tailFrontDegrees == 90 && history.tailRearDegrees == 90,
      "unsupported pure tail reverse has no residual motion");

  request.mode = DriveMode::Hybrid;
  request.surgePercent = 60; request.yawPercent = 60;
  request.pitchPercent = 100; request.allowFlowAssist = true;
  request.brakePercent = 100;
  history = {}; state = {};
  result = allocate(request, history, state, 12000, 20);
  check(result.targetFrame.leftPropulsionPercent >= -50 && result.targetFrame.rightPropulsionPercent >= -50,
      "brake cap applies independently to both opposing propellers");
  check(!result.targetFrame.flowAssisted, "braking does not re-enable flow assistance");
  check(std::abs(result.targetFrame.pectoralLeftDegrees - 137) < 0.001f &&
      std::abs(result.targetFrame.pectoralRightDegrees - 49) < 0.001f,
      "combined pitch, brake and inner fin yaw retain the F405 recipe");

  request.brakePercent = 0; request.pitchPercent = request.yawPercent = 0;
  request.mode = DriveMode::Prop; request.surgePercent = 100;
  history = {}; state = {};
  result = allocate(request, history, state, 14000, 200, 32);
  check(history.leftPropulsionPercent == 0 && history.rightPropulsionPercent == 10,
      "only successful channels advance output history");
  result = allocate(request, history, state, 14020, 20);
  check(result.candidateFrame.leftPropulsionPercent == 10 && result.candidateFrame.rightPropulsionPercent == 20,
      "next slew starts from each channel's last successful output");

  request.immediate = true; request.enabled = false;
  result = allocate(request, history, state, 14040, 20);
  check(history.leftPropulsionPercent == 0 && history.rightPropulsionPercent == 0 &&
      history.tailFrontDegrees == 90 && history.tailRearDegrees == 90,
      "immediate disabled request is neutral without ordinary slew");
  request.enabled = true; request.surgePercent = std::numeric_limits<float>::quiet_NaN();
  result = allocate(request, history, state, 14060, 20);
  check((result.limitedReasons & motion::InvalidInput) && history.leftPropulsionPercent == 0,
      "non-finite input is rejected before reaching PWM");
}

static void record(unsigned scenario, unsigned tick, const motion::Result &result) {
  const auto &f = result.candidateFrame;
  const auto &t = result.targetFrame;
  std::printf("%u,%u,%.9g,%.9g,%.9g,%.9g,%d,%d,%d,%.17g,%.9g,%.9g,%u,%d,%.9g,%.9g,%d,%d\n",
      scenario, tick, f.tailFrontDegrees, f.tailRearDegrees, f.pectoralLeftDegrees,
      f.pectoralRightDegrees, f.leftPropulsionPercent, f.rightPropulsionPercent,
      int(f.flowAssisted), result.phaseRad, result.commonPropulsionPercent,
      result.differentialPropulsionPercent, result.limitedReasons, int(result.unsupportedReverse),
      t.tailFrontDegrees, t.tailRearDegrees, t.leftPropulsionPercent, t.rightPropulsionPercent);
}

int main() {
  behaviorChecks();
  unsigned scenario = 0;
  constexpr unsigned masks[] = {63, 31, 47, 0, 61, 59, 55, 63};
  constexpr uint32_t periods[] = {20, 5, 35, 20, 0, 20, 40, 20};
  for (auto mode : {DriveMode::Bionic, DriveMode::Prop, DriveMode::Hybrid})
    for (float surge : {-100.0f, -20.0f, 0.0f, 20.0f, 100.0f})
      for (float yaw : {-100.0f, 0.0f, 100.0f})
        for (float pitch : {-100.0f, 0.0f, 100.0f})
          for (float gear : {0.5f, 0.75f, 1.0f})
            for (float brake : {0.0f, 100.0f})
              for (bool independent : {false, true}) {
                motion::State state;
                ActuatorFrame history;
                motion::Request request;
                request.mode = mode; request.enabled = true;
                request.surgePercent = surge; request.yawPercent = yaw;
                request.pitchPercent = pitch; request.outputScale = gear;
                request.brakePercent = brake; request.allowFlowAssist = true;
                request.independentDrives = independent;
                request.tailDrivePercent = surge * 0.2f;
                request.propulsionDrivePercent = surge * 0.8f;
                request.independentTailYaw = independent; request.tailYawPercent = -yaw;
                uint32_t now = 0xffffff00u;
                for (unsigned tick = 0; tick < 32; ++tick) {
                  const auto elapsed = periods[tick % 8]; now += elapsed;
                  if (tick == 16) {
                    request.surgePercent *= -1; request.tailDrivePercent *= -1;
                    request.propulsionDrivePercent *= -1;
                  }
                  if (tick == 31) { request.enabled = false; request.immediate = true; }
                  const auto result = allocate(request, history, state, now, elapsed, masks[tick % 8]);
                  check(std::isfinite(result.candidateFrame.tailFrontDegrees) &&
                      result.candidateFrame.tailFrontDegrees >= RF_TAIL_MIN_ANGLE &&
                      result.candidateFrame.tailFrontDegrees <= RF_TAIL_MAX_ANGLE &&
                      std::abs(result.candidateFrame.leftPropulsionPercent) <= 100 &&
                      std::abs(result.candidateFrame.rightPropulsionPercent) <= 100,
                      "combined trajectory stays within hard actuator limits");
                  record(scenario, tick, result);
                }
                ++scenario;
              }
  std::fprintf(stderr, "PASS: %u behavior checks, %u scenarios / %u control ticks\n",
      checks, scenario, scenario * 32);
}
