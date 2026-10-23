#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include "ControlState.h"
#include "Service/LoRa/LoraProtocol.h"
using namespace robofish;
static wifi::Command request(wifi::Action action, uint32_t sequence, uint32_t now) {
  wifi::Command c; c.action=action; c.sequence=sequence; c.receivedMs=now;
  std::strcpy(c.session,"test"); c.axes={35,-40,50}; return c;
}
int main() {
  lora::Command target;
  const std::string golden="101,1,23120,65535,20,3,-350,400,-1000,750,2991";
  assert(lora::parseLine(golden.c_str(),target));
  assert(target.code==lora::kTarget && target.source==23120 && target.sequence==65535 &&
      target.mode==DriveMode::Hybrid && target.axes.surge==-35 && target.axes.yaw==40 &&
      target.axes.heave==-100 && target.propulsionScalePercent==75);
  for (unsigned i=0;i<golden.rfind(',');++i) {
    auto changed=golden; changed[i]=changed[i]=='9'?'8':'9';
    lora::Command untouched; untouched.sequence=123;
    assert(!lora::parseLine(changed.c_str(),untouched) && untouched.sequence==123);
  }
  lora::Command heartbeat;
  assert(lora::parseLine("102,1,23120,0,62587",heartbeat) && heartbeat.code==lora::kHeartbeat);
  lora::LineDecoder decoder; lora::Command decoded;
  for(char c:golden) assert(decoder.consume(c,decoded)==lora::ParseResult::Incomplete);
  assert(decoder.consume('\n',decoded)==lora::ParseResult::Accepted && decoded.axes.heave==-100);
  for(uint32_t start : {0u,0xfffffff0u}) {
    ControlState state; state.stopPending=false;
    assert(state.accept(target,start) && state.leaseActive);
    auto acquired=state.accept(request(wifi::Action::Acquire,0,start+10),start+10);
    assert(acquired.code==wifi::Code::Accepted && state.leaseActive && !state.stopPending);
    assert(state.accept(request(wifi::Action::Target,1,start+2999),start+2999).code==wifi::Code::LoRaActive);
    assert(state.loraHealthy(start+2999) && !state.loraHealthy(start+3000));
    assert(state.accept(request(wifi::Action::Target,1,start+3000),start+3000).code==wifi::Code::Accepted);
    assert(state.wifiOwner && state.target.surgePercent==35);
    const auto lease=state.lastMotionMs;
    assert(state.accept(heartbeat,start+3100));
    assert(!state.wifiOwner && !state.leaseActive && state.stopPending && state.target.surgePercent==0);
    assert(state.lastMotionMs==lease); // Heartbeat cannot renew motion or restore an old goal.
    assert(state.accept(request(wifi::Action::Target,2,start+3101),start+3101).code==wifi::Code::LoRaActive);
    assert(!state.accept(target,start+3102)); // Neutral barrier precedes restored LoRa motion.
    state.stopPending=false;
    assert(state.accept(target,start+3120) && state.leaseActive && !state.wifiOwner);
    assert(state.accept(heartbeat,start+6000) && state.lastMotionMs==start+3120);
    assert(state.expire(start+6120) && state.loraHealthy(start+6120));
    auto stop=request(wifi::Action::Stop,2,start+6121);
    assert(state.accept(stop,start+6121).code==wifi::Code::Accepted && state.stopPending);
    state.maintenance=true;
    assert(!state.accept(target,start+6140));
    assert(state.accept(heartbeat,start+6150) && !state.leaseActive);
    assert(state.accept(request(wifi::Action::Target,3,start+6160),start+6160).code==wifi::Code::Maintenance);
  }
  ControlState switcher; switcher.stopPending=false;
  assert(switcher.accept(target,0)); assert(switcher.accept(heartbeat,100));
  assert(switcher.accept(request(wifi::Action::Acquire,0,3090),3090).code==wifi::Code::Accepted);
  // A later heartbeat has not renewed the original motion lease.
  assert(switcher.expire(3000));
  switcher.stopPending=false;
  assert(switcher.accept(request(wifi::Action::Target,1,3100),3100).code==wifi::Code::Accepted);
  for(unsigned i=0;i<5;++i) switcher.observeOutput(31);
  assert(switcher.outputRecovery && !switcher.leaseActive);
  assert(switcher.accept(request(wifi::Action::Target,2,3120),3120).code==wifi::Code::OutputRecovery);
  assert(!switcher.accept(target,3130));
  switcher.observeOutput(63); switcher.stopPending=false;
  assert(switcher.accept(request(wifi::Action::Target,2,6200),6200).code==wifi::Code::SessionConflict);
  lora::Command attitudeOn,attitudeOff;
  assert(lora::parseLine("101,1,23120,42,23,3,0,0,0,1000,7010",attitudeOn) && attitudeOn.code==lora::kAttitudeOn);
  assert(lora::parseLine("101,1,23120,42,24,3,0,0,0,1000,27492",attitudeOff) && attitudeOff.code==lora::kAttitudeOff);
  ControlState radioAttitude; radioAttitude.stopPending=false;
  assert(radioAttitude.accept(attitudeOn,10) && radioAttitude.attitudeEnabled && !radioAttitude.leaseActive);
  assert(radioAttitude.accept(target,20) && radioAttitude.leaseActive);
  assert(radioAttitude.accept(attitudeOff,2000) && !radioAttitude.attitudeEnabled && radioAttitude.lastMotionMs==20);
  assert(radioAttitude.expire(3020));
  ControlState wifiAttitude; wifiAttitude.stopPending=false;
  assert(wifiAttitude.accept(request(wifi::Action::Acquire,0,0),0).code==wifi::Code::Accepted);
  const auto enabled=wifiAttitude.accept(request(wifi::Action::AttitudeOn,1,10),10);
  assert(enabled.code==wifi::Code::Accepted && enabled.attitudeEnabled && !enabled.leaseActive);
  assert(wifiAttitude.accept(request(wifi::Action::Target,2,20),20).code==wifi::Code::Accepted);
  assert(wifiAttitude.accept(request(wifi::Action::AttitudeOff,3,2000),2000).code==wifi::Code::Accepted);
  assert(!wifiAttitude.attitudeEnabled && wifiAttitude.lastMotionMs==20 && wifiAttitude.leaseActive);
  assert(wifiAttitude.accept(request(wifi::Action::AttitudeOn,3,2001),2001).code==wifi::Code::StaleSequence);
  assert(wifiAttitude.expire(3020));
  std::puts("PASS: ordered LoRa/Wi-Fi attitude switches never start or renew motion; replay rejected");
  std::puts("PASS: CRC targets, 3s LoRa priority/wrap, fallback, recovery neutral barrier, heartbeat/lease separation, OTA and output-fault ownership");
}
