#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include "ControlState.h"
#include "Service/Wifi/ControlProtocol.h"

using namespace robofish;
static unsigned checks = 0;
static void check(bool ok, const char *message) {
  ++checks;
  if (!ok) { std::fprintf(stderr, "FAIL: %s\n", message); std::exit(1); }
}
static wifi::Command parse(const char *fields) {
  const std::string text = std::string("{\"action\":\"calibration_target\",\"session\":\"fixture\",\"sequence\":1,") + fields + "}";
  wifi::Command command;
  check(wifi::parseControlJson(text.data(), text.size(), command), "valid independent command parses");
  return command;
}
static motion::Result allocate(const motion::Request &request, bool independent = true, float amplitudeGain = 1) {
  auto recipe = config::motion::makeRecipe(request.mode);
  recipe.tail.amplitudeDegrees *= amplitudeGain;
  if (independent) recipe.turnForwardReductionFraction = 0;
  auto immediate = request;
  immediate.immediate = true;
  motion::State state;
  return motion::allocate(immediate, recipe, config::motion::makeGeometry(), {}, state, 20, 20);
}
int main() {
  ControlState owner;
  wifi::Command command;
  std::strcpy(command.session, "fixture");
  command.action = wifi::Action::Acquire;
  check(owner.accept(command, 0).code == wifi::Code::Accepted, "acquire existing Wi-Fi session");
  owner.stopPending = false;
  owner.setAttitude(true);
  const auto pid = owner.yawPid;
  command = parse("\"mode\":3,\"tail_drive_pct\":0,\"tail_bias_pct\":40,\"prop_surge_pct\":0,\"prop_diff_pct\":40");
  check(owner.accept(command, 0).code == wifi::Code::Accepted && owner.calibrationActive &&
      !owner.attitudeMotionAllowed() && !owner.headingTargetActive, "fixed calibration pauses feedback");
  check(owner.target.preserveTailBias, "explicit hybrid calibration uses the current heading steering priority");
  check(owner.yawPid.kp == pid.kp && owner.yawPid.ki == pid.ki && owner.yawPid.kd == pid.kd,
      "calibration preserves PID parameters");
  auto result = allocate(owner.target);
  check(result.tailDrivePercent == 0 && result.tailAmplitudeDegrees == 0 && result.tailFrequencyHz == 0,
      "zero forward drive never gets a yaw-driven tail gait");
  check(result.targetFrame.tailFrontDegrees != 90 && result.targetFrame.leftPropulsionPercent == 40 &&
      result.targetFrame.rightPropulsionPercent == -40, "zero surge retains tail bias and independent differential");
  command.sequence = 2;
  command.calibration = {20, 0, 30, 0, false};
  check(owner.accept(command, 0).code == wifi::Code::Accepted, "independent mixed propulsion accepts");
  result = allocate(owner.target);
  check(result.tailDrivePercent == 20 && result.commonPropulsionPercent == 30 && result.differentialPropulsionPercent == 0,
      "tail and common propulsion retain their independent setpoints");
  const auto straightFrequency = result.tailFrequencyHz;
  command.sequence = 3;
  command.calibration.tailBiasPercent = 40;
  command.calibration.propDiffPercent = 40;
  check(owner.accept(command, 0).code == wifi::Code::Accepted, "independent mixed steering accepts");
  result = allocate(owner.target);
  check(result.tailDrivePercent == 20 && result.tailFrequencyHz == straightFrequency &&
      result.commonPropulsionPercent == 30 && result.differentialPropulsionPercent == 40,
      "steering neither adds propulsion nor reduces explicit common drive");
  command.sequence = 4;
  command.mode = DriveMode::Prop;
  command.calibration = {0, 0, 20, -30, false};
  check(owner.accept(command, 0).code == wifi::Code::Accepted, "pure prop calibration accepts");
  check(!owner.target.preserveTailBias, "pure prop calibration retains the old allocation policy");
  result = allocate(owner.target);
  check(result.targetFrame.tailFrontDegrees == 90 && result.targetFrame.tailRearDegrees == 90 &&
      result.commonPropulsionPercent == 20 && result.differentialPropulsionPercent == -30,
      "prop trial leaves tail neutral");
  command.sequence = 5;
  command.mode = DriveMode::Bionic;
  command.calibration = {30, -20, 0, 0, false};
  check(owner.accept(command, 0).code == wifi::Code::Accepted, "pure tail calibration accepts");
  check(!owner.target.preserveTailBias, "pure tail calibration retains the old allocation policy");
  result = allocate(owner.target);
  check(result.commonPropulsionPercent == 0 && result.differentialPropulsionPercent == 0 && result.tailDrivePercent == 30,
      "tail trial leaves both props neutral");
  command.sequence = 6;
  command.action = wifi::Action::Mode;
  command.mode = DriveMode::Hybrid;
  check(owner.accept(command, 0).code == wifi::Code::Accepted && !owner.calibrationActive && !owner.leaseActive,
      "mode change clears calibration without starting motion");
  command.action = wifi::Action::CalibrationTarget;
  command.sequence = 7;
  command.calibration = {0, 100, 0, 100, true};
  check(owner.accept(command, 0).code == wifi::Code::Accepted, "explicit baseline accepts");
  check(owner.tailAmplitudeGain() == 1.0f, "old baseline keeps its original gait amplitude");
  check(!owner.target.preserveTailBias, "old baseline keeps shared bias and amplitude scaling");
  result = allocate(owner.target, false);
  check(result.tailDrivePercent == config::lora::kHybridTurnTailDrivePercent,
      "baseline explicitly preserves old 35 percent yaw tail rule");
  check(std::abs(result.effectiveTailBiasDegrees) < config::lora::kTailTurnBiasDegrees &&
      result.effectiveTailAmplitudeDegrees > 0, "old baseline still shares joint travel between steering and gait");
  command.sequence = 8;
  command.action = wifi::Action::Target;
  command.axes = {10, 0, 0};
  check(owner.accept(command, 0).code == wifi::Code::Accepted && !owner.calibrationActive && !owner.target.fixedCalibration,
      "manual target removes fixed calibration state");
  check(!owner.target.preserveTailBias, "manual target clears heading-specific steering priority");
  command.action = wifi::Action::CalibrationTarget;
  command.sequence = 9;
  check(owner.accept(command, 0).code == wifi::Code::Accepted, "restart calibration");
  command.action = wifi::Action::Stop;
  command.sequence = 10;
  check(owner.accept(command, 0).code == wifi::Code::Accepted && !owner.calibrationActive &&
      !owner.leaseActive && !owner.target.enabled, "STOP clears calibration and all output authorization");
  owner.stopPending = false;
  owner.headingFeedbackValid = true;
  owner.headingSampleMs = 0;
  command.action = wifi::Action::HeadingTarget;
  command.sequence = 11;
  command.axes = {};
  check(owner.accept(command, 0).code == wifi::Code::Accepted, "automatic zero-drive heading accepts");
  auto automatic = owner.target;
  check(std::strcmp(config::control::kHeadingAllocationProfile, "hybrid_gait_tail_bias") == 0,
      "heading capability names the hybrid gait recipe");
  for (const float surge : {0.0f, 10.0f, 15.0f, 20.0f, 30.0f, 100.0f}) {
    owner.startHeading(0, surge, 100, 0);
    float gaitFrequency = -1, gaitAmplitude = -1;
    for (const float yaw : {-100.0f, 0.0f, 100.0f}) {
      automatic = owner.target;
      automatic.yawPercent = yaw;
      automatic.immediate = true;
      auto recipe = config::motion::makeRecipe(automatic.mode);
      const auto originalRecipe = recipe;
      recipe.tail.amplitudeDegrees *= owner.tailAmplitudeGain();
      check(owner.tailAmplitudeGain() == 1.5f, "heading uses the requested fifty-percent amplitude boost");
      owner.applyHeadingAllocation(automatic, recipe);
      motion::State state;
      result = motion::allocate(automatic, recipe, config::motion::makeGeometry(), {}, state, 20, 20);
      check(automatic.preserveTailBias && std::abs(result.effectiveTailBiasDegrees + yaw * .24f) < .001f,
          "heading keeps the requested tail steering center after travel limits");
      if (surge == 0)
        check(result.effectiveTailAmplitudeDegrees == 0, "zero surge has no effective periodic tail motion");
      if (surge == 20 && std::abs(yaw) == 100) {
        check(std::abs(result.effectiveTailAmplitudeDegrees - 8) < .001f,
            "boosted twenty-percent gait retains eight-degree swing at full twenty-four-degree bias");
        auto legacy = automatic;
        legacy.preserveTailBias = false;
        motion::State legacyState;
        const auto shared = motion::allocate(legacy, recipe, config::motion::makeGeometry(), {}, legacyState, 20, 20);
        const float sharedScale = 32.0f / (24.0f + result.tailAmplitudeDegrees);
        check(std::abs(std::abs(shared.effectiveTailBiasDegrees) - 24.0f * sharedScale) < .001f &&
            std::abs(shared.effectiveTailAmplitudeDegrees - result.tailAmplitudeDegrees * sharedScale) < .001f,
            "non-priority requests retain the exact previous shared scaling formula");
      }
      check(result.tailDrivePercent == surge && ((surge == 0 && result.tailFrequencyHz == 0 &&
          result.tailAmplitudeDegrees == 0) || (surge > 0 && result.tailFrequencyHz > 0 && result.tailAmplitudeDegrees > 0)),
          "heading surge drives hybrid gait and zero surge cancels all periodic propulsion");
      if (gaitFrequency < 0) { gaitFrequency = result.tailFrequencyHz; gaitAmplitude = result.tailAmplitudeDegrees; }
      check(result.tailFrequencyHz == gaitFrequency && result.tailAmplitudeDegrees == gaitAmplitude,
          "yaw changes neither gait frequency nor requested amplitude at a fixed surge");
      check(result.commonPropulsionPercent == surge && result.differentialPropulsionPercent == 0,
          "heading correction preserves calibrated common propulsion without differential");
      if (surge > 0 && surge <= 20 && yaw == 0) {
        auto referenceRecipe = originalRecipe;
        owner.applyHeadingAllocation(automatic, referenceRecipe);
        motion::State referenceState;
        const auto reference = motion::allocate(automatic, referenceRecipe, config::motion::makeGeometry(), {}, referenceState, 20, 20);
        check(result.tailAmplitudeDegrees >= reference.tailAmplitudeDegrees * 1.49f,
            "low-speed heading gait has visibly greater requested amplitude");
        check(result.tailFrequencyHz == reference.tailFrequencyHz,
            "normal low-speed tail frequency is not raised by the amplitude gain");
      }
      check(result.tailBiasPercent == yaw && (surge != 0 ||
          result.targetFrame.tailFrontDegrees == config::lora::kTailFrontCenter - yaw * .01f * config::lora::kTailTurnBiasDegrees),
          "heading retains original bias polarity and zero-surge static steering");
      const auto geometry = config::motion::makeGeometry();
      const float initialFront = result.targetFrame.tailFrontDegrees;
      result = motion::allocate(automatic, recipe, geometry, result.candidateFrame, state, 270, 20);
      check((surge > 0) == (std::abs(result.targetFrame.tailFrontDegrees - initialFront) > .01f),
          "positive heading surge really moves tail over time; zero surge never swings");
      check(result.targetFrame.tailFrontDegrees >= geometry.tailFront.minimumDegrees &&
          result.targetFrame.tailFrontDegrees <= geometry.tailFront.maximumDegrees &&
          result.targetFrame.tailRearDegrees >= geometry.tailRear.minimumDegrees &&
          result.targetFrame.tailRearDegrees <= geometry.tailRear.maximumDegrees,
          "hybrid gait with full steering remains inside both joint limits");
    }
  }
  command.mode = DriveMode::Hybrid;
  command.calibration = {20, 100, 20, 0, false};
  owner.startCalibration(command);
  result = allocate(owner.target, true, owner.tailAmplitudeGain());
  check(result.effectiveTailBiasDegrees == -24 && result.effectiveTailAmplitudeDegrees == 8,
      "current hybrid calibration reproduces the heading recipe's preserved bias and bounded swing");
  for (const char *fields : {
      "\"mode\":3,\"tail_drive_pct\":-1,\"tail_bias_pct\":0,\"prop_surge_pct\":0,\"prop_diff_pct\":0",
      "\"mode\":3,\"tail_drive_pct\":0,\"tail_bias_pct\":0,\"prop_surge_pct\":0,\"prop_diff_pct\":101",
      "\"mode\":3,\"tail_drive_pct\":0,\"tail_bias_pct\":0,\"prop_surge_pct\":0,\"prop_diff_pct\":0,\"baseline\":1",
      "\"mode\":3,\"tail_drive_pct\":10,\"tail_bias_pct\":0,\"prop_surge_pct\":20,\"prop_diff_pct\":0,\"baseline\":true",
      "\"mode\":1,\"tail_drive_pct\":20,\"tail_bias_pct\":0,\"prop_surge_pct\":10,\"prop_diff_pct\":0",
      "\"mode\":2,\"tail_drive_pct\":20,\"tail_bias_pct\":0,\"prop_surge_pct\":10,\"prop_diff_pct\":0"}) {
    const std::string text = std::string("{\"action\":\"calibration_target\",\"session\":\"fixture\",\"sequence\":1,") + fields + "}";
    check(!wifi::parseControlJson(text.data(), text.size(), command), "invalid or ambiguous calibration output rejects");
  }
  std::printf("PASS: %u dynamics calibration protocol, takeover and allocator checks\n", checks);
}
