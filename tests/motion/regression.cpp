#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

#include "Mode/Motion/MotionMode.h"
#include "Device/Actuator/TailMotion.h"

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
  check(history.leftPropulsionPercent == 4 && history.rightPropulsionPercent == 4,
      "forward outputs remain nominal and rise by 4 percent per 20 ms");
  for (unsigned tick = 1; tick < 25; ++tick) allocate(request, history, state, 5000 + 20 * tick, 20);
  check(history.leftPropulsionPercent == 100 && history.rightPropulsionPercent == 100,
      "both nominal propellers reach forward target");
  request.surgePercent = -100;
  bool submittedNeutral = false;
  for (unsigned tick = 0; tick < 50; ++tick) {
    const int previous = history.leftPropulsionPercent;
    result = allocate(request, history, state, 5500 + 20 * tick, 20);
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
      std::abs(result.targetFrame.pectoralRightDegrees - 43) < 0.001f,
      "pitch and brake stay within geometry without yaw changing fin state");

  request = {}; request.enabled = true; request.mode = DriveMode::Hybrid;
  request.surgePercent = 100;
  const auto recipe = config::motion::makeRecipe(request.mode);
  const auto gait = device::tail::schedule(request, recipe.tail, 25);
  check(std::abs(gait.frequencyHz - 1.5f) < 0.001f,
      "raised 1.5 Hz full forward gait fits the tail speed budget");
  request.yawPercent = 100;
  const auto turning = device::tail::schedule(request, recipe.tail, 25);
  check(std::abs(turning.biasDegrees) == 24,
      "full turning gait has 24 degree tail bias within front-joint reserve");
  request.mode = DriveMode::Prop;
  history = {}; state = {}; request.immediate = true;
  result = allocate(request, history, state, 13000, 20);
  check(std::abs(history.tailFrontDegrees - 90) == 32 &&
      history.pectoralLeftDegrees == 90 && history.pectoralRightDegrees == 90,
      "prop mode respects 32 degree usable rudder and keeps normal fins level");
  request.yawPercent = -100;
  result = allocate(request, history, state, 13020, 20);
  check(history.pectoralLeftDegrees == 90 && history.pectoralRightDegrees == 90,
      "opposite yaw also leaves normal fins level");

  history = {}; state = {}; request = {};
  request.mode = DriveMode::Prop; request.enabled = true; request.surgePercent = 100;
  request.immediate = true;
  allocate(request, history, state, 13200, 20);
  request.enabled = false; request.immediate = false;
  for (unsigned tick = 1; tick <= 25; ++tick) {
    result = allocate(request, history, state, 13200 + 20 * tick, 20);
    check(history.leftPropulsionPercent == 100 - int(4 * tick) &&
        history.rightPropulsionPercent == 100 - int(4 * tick),
        "ordinary disabled allocation ramps 100 percent to neutral over 500 ms");
  }

  request.brakePercent = 0; request.pitchPercent = request.yawPercent = 0;
  request.mode = DriveMode::Prop; request.surgePercent = 100;
  request.enabled = true;
  history = {}; state = {};
  result = allocate(request, history, state, 14000, 200, 32);
  check(history.leftPropulsionPercent == 0 && history.rightPropulsionPercent == 4,
      "only successful channels advance output history");
  result = allocate(request, history, state, 14020, 20);
  check(result.candidateFrame.leftPropulsionPercent == 4 && result.candidateFrame.rightPropulsionPercent == 8,
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

static void servoTransitionChecks() {
  for (float yaw : {-100.0f, 100.0f}) for (float pitch : {-100.0f, 100.0f}) {
    motion::Request request;
    request.mode = DriveMode::Prop; request.enabled = true;
    request.yawPercent = yaw; request.pitchPercent = pitch;
    ActuatorFrame history;
    motion::State state;
    for (unsigned tick = 0; tick < 100; ++tick) {
      if (tick == 50) request.enabled = false;
      const auto previous = history;
      const auto result = allocate(request, history, state, 20000 + tick * 20, 20);
      check(std::abs(history.tailFrontDegrees - previous.tailFrontDegrees) <= 1.0001f &&
          std::abs(history.tailRearDegrees - previous.tailRearDegrees) <= 1.0001f,
          "both stationary tail joints start and return at no more than 50 degrees/second");
      check(std::abs(history.pectoralLeftDegrees - previous.pectoralLeftDegrees) <= 1.8001f &&
          std::abs(history.pectoralRightDegrees - previous.pectoralRightDegrees) <= 1.8001f,
          "both pectoral fins start and return at no more than 90 degrees/second");
      if (tick == 49) check(history.tailFrontDegrees == result.targetFrame.tailFrontDegrees &&
          history.tailRearDegrees == result.targetFrame.tailRearDegrees &&
          history.pectoralLeftDegrees == result.targetFrame.pectoralLeftDegrees &&
          history.pectoralRightDegrees == result.targetFrame.pectoralRightDegrees,
          "soft startup still reaches all four commanded servo positions");
      if (tick == 50) check(history.tailFrontDegrees != 90 && history.tailRearDegrees != 90 &&
          history.pectoralLeftDegrees != 90 && history.pectoralRightDegrees != 90,
          "ordinary release does not jump any moving servo directly to neutral");
    }
    check(history.tailFrontDegrees == 90 && history.tailRearDegrees == 90 &&
        history.pectoralLeftDegrees == 90 && history.pectoralRightDegrees == 90,
        "ordinary servo release reaches neutral after bounded return");
  }
  motion::Request request;
  request.mode = DriveMode::Bionic; request.enabled = true; request.surgePercent = 100;
  ActuatorFrame history;
  motion::State state;
  for (unsigned tick = 0; tick < 26; ++tick) {
    const float before = state.tailAmplitudeDegrees;
    allocate(request, history, state, 30000 + tick * 20, 20);
    check(state.tailAmplitudeDegrees - before <= 1.0001f,
        "tail swimming amplitude grows gradually while preserving the 1.5 Hz gait");
    if (tick == 10) check(state.tailAmplitudeDegrees <= 10.0001f,
        "tail swing cannot immediately reach full amplitude on startup");
  }
  check(std::abs(state.tailAmplitudeDegrees - 25) < 0.001f,
      "tail amplitude reaches full strength after the 500 ms envelope");
  const auto previous = history;
  request.enabled = false;
  allocate(request, history, state, 30520, 20);
  check(std::abs(history.tailFrontDegrees - previous.tailFrontDegrees) <= 1.0001f &&
      std::abs(history.tailRearDegrees - previous.tailRearDegrees) <= 1.0001f &&
      history.tailFrontDegrees != 90,
      "swimming tail release returns gradually from its actual last joint positions");
}

static void fullTurnChecks() {
  for (auto mode : {DriveMode::Bionic, DriveMode::Hybrid})
    for (float surge : {0.0f, 100.0f}) for (float yaw : {-100.0f, 100.0f}) {
      motion::Request request; request.mode=mode; request.enabled=true;
      request.surgePercent=surge; request.yawPercent=yaw;
      ActuatorFrame history; motion::State state;
      float minimum=180, maximum=0;
      for (unsigned tick=0;tick<250;++tick) {
        const double previousPhase = state.gait.radians;
        const auto result=allocate(request,history,state,tick*20,20);
        check(history.tailFrontDegrees>=58 && history.tailFrontDegrees<=122 &&
              history.tailRearDegrees>=45 && history.tailRearDegrees<=135,
              "full-strength turn remains inside both five-degree mechanical reserves");
        if(tick>100){minimum=std::min(minimum,history.tailFrontDegrees);maximum=std::max(maximum,history.tailFrontDegrees);}
        check(!(result.limitedReasons & motion::InvalidInput), "full turn keeps valid joint history");
        if (tick > 100) check(std::abs(std::remainder(result.phaseRad - previousPhase,
            algorithm::kTwoPi) / (0.02 * algorithm::kTwoPi) - 1.5) < 0.001,
            "full turn retains a 1.5 Hz gait despite Hybrid forward-drive reduction");
      }
      check(maximum-minimum>15, "maximum turn retains useful tail oscillation instead of freezing");
    }
}

static void finStateChecks() {
  for (auto mode : {DriveMode::Bionic, DriveMode::Prop, DriveMode::Hybrid})
    for (float surge : {0.0f, 60.0f}) for (float yaw : {-75.0f, 0.0f, 75.0f}) {
      motion::Request request; request.mode = mode; request.enabled = true;
      request.surgePercent = surge; request.yawPercent = yaw;
      ActuatorFrame history, levelHistory;
      motion::State state, levelState;
      for (unsigned tick = 0; tick < 120; ++tick) {
        request.pitchPercent = tick < 40 ? 100 : tick < 80 ? -100 : 0;
        auto level = request; level.pitchPercent = 0;
        const auto result = allocate(request, history, state, tick * 20, 20);
        allocate(level, levelHistory, levelState, tick * 20, 20);
        check(history.tailFrontDegrees == levelHistory.tailFrontDegrees &&
            history.tailRearDegrees == levelHistory.tailRearDegrees &&
            history.leftPropulsionPercent == levelHistory.leftPropulsionPercent &&
            history.rightPropulsionPercent == levelHistory.rightPropulsionPercent &&
            !result.targetFrame.flowAssisted,
            "changing independent fin state cannot change tail or propeller demand");
        if (surge == 0 && yaw == 0) check(history.tailFrontDegrees == 90 &&
            history.tailRearDegrees == 90 && history.leftPropulsionPercent == 0 &&
            history.rightPropulsionPercent == 0,
            "up and down alone only move the two fins in every drive mode");
      }
      check(history.pectoralLeftDegrees == 90 &&
          history.pectoralRightDegrees == 90, "normal state levels both fins smoothly");
    }
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
  servoTransitionChecks();
  fullTurnChecks();
  finStateChecks();
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
                      result.candidateFrame.tailFrontDegrees >= config::lora::kTailFrontMinimum &&
                      result.candidateFrame.tailFrontDegrees <= config::lora::kTailFrontMaximum &&
                      result.candidateFrame.tailRearDegrees >= config::lora::kTailRearMinimum &&
                      result.candidateFrame.tailRearDegrees <= config::lora::kTailRearMaximum &&
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
