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
static motion::Result allocate(const motion::Request &request, bool independent = true) {
  auto recipe = config::motion::makeRecipe(request.mode);
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
  result = allocate(owner.target);
  check(result.targetFrame.tailFrontDegrees == 90 && result.targetFrame.tailRearDegrees == 90 &&
      result.commonPropulsionPercent == 20 && result.differentialPropulsionPercent == -30,
      "prop trial leaves tail neutral");
  command.sequence = 5;
  command.mode = DriveMode::Bionic;
  command.calibration = {30, -20, 0, 0, false};
  check(owner.accept(command, 0).code == wifi::Code::Accepted, "pure tail calibration accepts");
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
  result = allocate(owner.target, false);
  check(result.tailDrivePercent == config::lora::kHybridTurnTailDrivePercent,
      "baseline explicitly preserves old 35 percent yaw tail rule");
  command.sequence = 8;
  command.action = wifi::Action::Target;
  command.axes = {10, 0, 0};
  check(owner.accept(command, 0).code == wifi::Code::Accepted && !owner.calibrationActive && !owner.target.fixedCalibration,
      "manual target removes fixed calibration state");
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
  for (const float surge : {0.0f, 30.0f, 100.0f}) {
    owner.startHeading(0, surge, 100, 0);
    for (const float yaw : {-100.0f, 0.0f, 100.0f}) {
      automatic = owner.target;
      automatic.yawPercent = yaw;
      automatic.immediate = true;
      auto recipe = config::motion::makeRecipe(automatic.mode);
      owner.applyHeadingAllocation(automatic, recipe);
      motion::State state;
      result = motion::allocate(automatic, recipe, config::motion::makeGeometry(), {}, state, 20, 20);
      check(result.tailDrivePercent == 0 && result.tailFrequencyHz == 0 && result.tailAmplitudeDegrees == 0,
          "heading uses static tail bias without hidden forward gait");
      check(result.commonPropulsionPercent == surge && result.differentialPropulsionPercent == 0,
          "heading correction preserves calibrated common propulsion without differential");
      check(result.tailBiasPercent == yaw &&
          result.targetFrame.tailFrontDegrees == config::lora::kTailFrontCenter - yaw * .01f * config::lora::kTailTurnBiasDegrees,
          "heading retains original left/right tail polarity and 24-degree maximum attack angle");
    }
  }
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
