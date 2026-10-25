#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "ControlState.h"
using namespace robofish;
static unsigned checks=0;
static void check(bool ok,const char *message) {
  ++checks;if (!ok) { std::fprintf(stderr,"FAIL: %s\n",message);std::exit(1); }
}
int main() {
  ControlState state;state.stopPending=false;state.headingFeedbackValid=true;state.headingSampleMs=20;
  lora::Command command;command.code=lora::kHeadingTarget;command.mode=DriveMode::Hybrid;
  command.headingCwDegrees=-90;command.axes.surge=35;
  check(state.accept(command,20) && state.headingTargetActive && state.attitudeEnabled &&
      state.target.mode==DriveMode::Hybrid && state.target.surgePercent==35,"LoRa angle/surge target enables native hybrid control");
  auto generation=state.attitudeGeneration;command.headingCwDegrees=-80;
  check(state.accept(command,40) && state.attitudeGeneration==generation,"streaming references do not reset PID generation");
  state.headingFeedbackValid=false;
  check(!state.accept(command,60) && state.loraResult.code==lora::ControlCode::HeadingUnavailable &&
      state.headingTargetCwDegrees==-80,"unavailable heading rejects a new reference without changing the active goal");
  state.headingFeedbackValid=true;state.headingSampleMs=0;
  check(!state.accept(command,240),"stale cached IMU cannot authorize a new heading goal");
  command.code=lora::kTarget;command.axes={0,40,0};
  check(state.accept(command,260) && !state.headingTargetActive,"manual steering takes over explicit heading reference");
  state.stop();check(!state.attitudeEnabled && !state.headingTargetActive && !state.leaseActive,"STOP removes target and PID authorization");
  ControlState wifiState;wifiState.stopPending=false;wifiState.headingFeedbackValid=true;wifiState.headingSampleMs=100;
  wifi::Command request;request.action=wifi::Action::Acquire;request.receivedMs=100;std::strcpy(request.session,"fixture");
  check(wifiState.accept(request,100).code==wifi::Code::Accepted,"Wi-Fi acquires ordered session");
  wifiState.stopPending=false; // Successful neutral write by the existing owner barrier.
  request.action=wifi::Action::HeadingTarget;request.sequence=1;request.headingCwDegrees=179;request.axes.surge=0;
  check(wifiState.accept(request,100).code==wifi::Code::Accepted && wifiState.headingTargetActive &&
      wifiState.attitudeMotionAllowed(),"zero-surge Wi-Fi angle target remains a local closed loop");
  request.sequence=2;request.receivedMs=320;
  check(wifiState.accept(request,320).code==wifi::Code::HeadingUnavailable,"Wi-Fi stale IMU rejects goal with specific status");
  request.action=wifi::Action::Stop;
  check(wifiState.accept(request,320).code==wifi::Code::Accepted && !wifiState.attitudeEnabled,"full stop still works without heading");
  std::printf("PASS: %u explicit heading authorization/state checks\n",checks);
}
