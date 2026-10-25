#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <string>
#include <vector>
#include "network.h"
#include "driver/uart.h"
#include "config/BoardConfig.h"
#include "Service/LoRa/LoraProtocol.h"
#include "Device/Imu/Jy61p.h"

extern "C" void app_main();
static unsigned checks = 0, queueCount = 0;
static uint32_t now = 0;
static uint32_t boardClockOffset = 0;
static std::string scenario = "normal";
static QueueHandle_t values, commands, otaRequests, otaResults;
static HostQueue events[3];
static std::deque<uint8_t> rx[3];
static bool uartReady[3]{};
struct Write { uint32_t at; robofish::ActuatorFrame frame; uint8_t mask; };
static std::vector<Write> writes;
static void check(bool ok, const char *label) {
  ++checks;
  if (!ok) { std::fprintf(stderr, "FAIL at %u ms (%s): %s\n", now, scenario.c_str(), label); std::abort(); }
}
QueueHandle_t xQueueCreateStatic(unsigned capacity, unsigned size, uint8_t *, StaticQueue_t *storage) {
  ++queueCount; storage->capacity = capacity; storage->itemBytes = size; return storage;
}
int xQueueSend(QueueHandle_t q, const void *item, unsigned wait) {
  check(wait == 0, "queue publication is nonblocking");
  if (q->entries.size() == q->capacity) return 0;
  const auto *bytes = static_cast<const uint8_t *>(item);
  q->entries.emplace_back(bytes, bytes + q->itemBytes); return pdTRUE;
}
int xQueueReceive(QueueHandle_t q, void *out, unsigned wait) {
  check(wait == 0, "queue consumption is nonblocking");
  if (q->entries.empty()) return 0;
  if(q==commands && scenario=="clock_race") boardClockOffset=1;
  std::memcpy(out,q->entries.front().data(),q->itemBytes); q->entries.pop_front(); return pdTRUE;
}
int xQueueReset(QueueHandle_t q) { q->entries.clear(); return pdTRUE; }
int xQueueOverwrite(QueueHandle_t q, const void *item) {
  check(q->capacity == 1, "latest-value queue"); xQueueReset(q); return xQueueSend(q,item,0);
}
int xQueuePeek(QueueHandle_t q, void *out, unsigned wait) {
  check(wait == 0, "snapshot read is nonblocking");
  if (q->entries.empty()) return 0;
  std::memcpy(out,q->entries.front().data(),q->itemBytes); return pdTRUE;
}
const char *esp_err_to_name(esp_err_t) { return "mock"; }
esp_err_t uart_param_config(uart_port_t port,const uart_config_t *c) {
  check(port == 1 || port == 2, "separate hardware UARTs");
  check(c->baud_rate == 115200 && c->data_bits == 8 &&
      c->parity == 0 && c->stop_bits == 1, "LoRa and JY61P 115200, both 8N1");
  return ESP_OK;
}
esp_err_t uart_set_pin(uart_port_t port,int tx,int receive,int,int) {
  check(tx == (port == 1 ? 2 : 7) && receive == (port == 1 ? 1 : 9), "PCB-verified UART pin mapping");
  return ESP_OK;
}
esp_err_t uart_driver_install(uart_port_t port,int buffer,int,int count,QueueHandle_t *out,int) {
  check(buffer == 512 && count == 16 && out, "bounded independent receive buffers/events");
  events[port].capacity = count; events[port].itemBytes = sizeof(uart_event_t);
  *out = &events[port]; uartReady[port] = true; return ESP_OK;
}
esp_err_t uart_driver_delete(uart_port_t port) { uartReady[port] = false; return ESP_OK; }
esp_err_t uart_flush_input(uart_port_t port) { rx[port].clear(); return ESP_OK; }
int uart_read_bytes(uart_port_t port,void *out,size_t capacity,unsigned wait) {
  check(wait == 0 && capacity <= 128, "bounded UART poll never blocks");
  const size_t count = std::min(capacity,rx[port].size());
  for (size_t i=0;i<count;++i) { static_cast<uint8_t *>(out)[i] = rx[port].front(); rx[port].pop_front(); }
  return static_cast<int>(count);
}
int uart_tx_chars(uart_port_t,const char *,size_t) { check(false,"LoRa/IMU must never transmit"); return -1; }
esp_err_t uart_wait_tx_done(uart_port_t,unsigned) { return ESP_OK; }
static void send(unsigned sequence,unsigned code,int knob = 1000) {
  char line[64]; std::snprintf(line,sizeof(line),"100,1,0,%u,%u,%d\n",sequence,code,knob);
  for (const char *p=line;*p;++p) rx[1].push_back(static_cast<uint8_t>(*p));
}
static void postWifi(robofish::wifi::Action action,uint32_t sequence) {
  robofish::wifi::Command command;
  command.action=action; command.sequence=sequence; command.receivedMs=now+(scenario=="clock_race"?1:0);
  std::strcpy(command.session,"hold-fixture");
  command.axes={100,0,100};
  check(xQueueSend(commands,&command,0)==pdTRUE,"Wi-Fi hold fixture fits command queue");
}
static std::vector<uint8_t> packet(uint8_t type,int x,int y,int z) {
  std::vector<uint8_t> p{0x55,type};
  for (int value : {x,y,z,0}) { p.push_back(value & 255); p.push_back((static_cast<unsigned>(value) >> 8) & 255); }
  uint8_t sum=0; for(auto b:p) sum+=b; p.push_back(sum); return p;
}
static bool fullRangeScenario() { return scenario=="yaw_full" || scenario=="wifi_yaw_full"; }
static bool headingScenario() { return scenario=="heading_target" || scenario=="wifi_heading_target"; }
static bool idleScenario() { return scenario=="yaw_idle" || scenario=="wifi_yaw_idle" || fullRangeScenario() || headingScenario(); }
static bool wifiIdleScenario() { return scenario=="wifi_yaw_idle" || scenario=="wifi_yaw_full" || scenario=="wifi_heading_target"; }
static void idleCommand(unsigned sequence,unsigned code,float surge=0,float yaw=0,float heading=0) {
  if(wifiIdleScenario()) {
    robofish::wifi::Command c;
    using robofish::wifi::Action;
    c.action=code==robofish::lora::kHeadingTarget?Action::HeadingTarget:code==robofish::lora::kTarget?Action::Target:code==17?Action::Stop:
        code==robofish::lora::kAttitudeOn?Action::AttitudeOn:Action::AttitudeOff;
    c.sequence=sequence; c.receivedMs=now; c.axes={surge,yaw,0};
    c.headingCwDegrees=heading;
    std::strcpy(c.session,"hold-fixture");
    check(xQueueSend(commands,&c,0)==pdTRUE,"idle command fits bounded Wi-Fi FIFO");
  } else {
    char body[80],line[96];
    std::snprintf(body,sizeof(body),"101,1,0,%u,%u,3,%d,%d,0,1000",sequence,code,
        static_cast<int>(code==robofish::lora::kHeadingTarget?heading*100:surge*10),
        static_cast<int>(code==robofish::lora::kHeadingTarget?surge*10:yaw*10));
    uint16_t crc=0xffff;
    for(const char *p=body;*p;++p) {
      crc^=static_cast<uint16_t>(static_cast<uint8_t>(*p))<<8;
      for(unsigned bit=0;bit<8;++bit) crc=static_cast<uint16_t>((crc<<1)^((crc&0x8000)?0x1021:0));
    }
    std::snprintf(line,sizeof(line),"%s,%u\n",body,crc);
    for(const char *p=line;*p;++p) rx[1].push_back(static_cast<uint8_t>(*p));
  }
}
static void idleImu() {
  if(headingScenario()) {
    for(auto b:packet(0x53,-32768,0,0)) rx[2].push_back(b);
    for(auto b:packet(0x52,0,0,0)) rx[2].push_back(b);
    return;
  }
  if(fullRangeScenario()) {
    const float yaw=now<200?0:now<400?10:now<1100?21:-21;
    for(auto b:packet(0x53,-32768,0,static_cast<int>(std::lround(yaw*32768/180)))) rx[2].push_back(b);
    for(auto b:packet(0x52,0,0,0)) rx[2].push_back(b);
    return;
  }
  if(now>=4240 && now<4500) return; // Exercise the real sensor freshness gate.
  const float yaw=now<200?0:now<600?10:now<1600?-10:now<2200?-15:
      now<3100?-5:now<3900?5:now<4500?15:-20;
  for(auto b:packet(0x53,-32768,0,static_cast<int>(std::lround(yaw*32768/180)))) rx[2].push_back(b);
  for(auto b:packet(0x52,0,0,0)) rx[2].push_back(b);
}
namespace board {
bool begin() { return true; }
uint32_t nowMs() { return now+boardClockOffset; }
uint8_t applyFrameMask(const robofish::ActuatorFrame &frame) {
  const bool failure = (scenario == "recovery" && now >= 60 && now < 240) ||
      (scenario == "fault" && now >= 60 && now < 400) || (scenario == "normal" && now == 1200);
  const uint8_t mask = failure ? 31 : 63;
  writes.push_back({now,frame,mask}); return mask;
}
}
bool networkBegin(QueueHandle_t snapshots,QueueHandle_t requests,QueueHandle_t,QueueHandle_t stops,QueueHandle_t stopped) {
  values=snapshots; commands=requests; otaRequests=stops; otaResults=stopped;
  check(queueCount==5 && uartReady[1] && uartReady[2],"both UARTs initialized, bounded fallback, telemetry and OTA queues");
  if(idleScenario()) {
    if(wifiIdleScenario()) postWifi(robofish::wifi::Action::Acquire,0);
    else if(!headingScenario()) idleCommand(1,robofish::lora::kAttitudeOn);
    idleImu(); return true;
  }
  if(scenario=="wifi_hold" || scenario=="clock_race") {
    postWifi(robofish::wifi::Action::Acquire,0);
    postWifi(robofish::wifi::Action::Target,1);
  } else {
    if (scenario=="hold") send(1,6);
    send(scenario=="hold"?2:1,2);
  }
  for(auto b:packet(0x53,8192,-8192,16384)) rx[2].push_back(b);
  if(scenario=="attitude")
    for(const char *p="101,1,23120,42,23,3,0,0,0,1000,7010\n";*p;++p) rx[1].push_back(static_cast<uint8_t>(*p));
  return true;
}
void networkConfirmStartup(bool ready) { check(ready,"startup confirms LoRa and successful neutral output"); }
TickType_t xTaskGetTickCount() { return now; }
struct Finished {};
static void headingStep(const MotionSnapshot &s) {
  if(now==100) check(s.control.headingTargetActive && s.attitude.yawActive &&
      std::abs(s.attitude.headingTargetDegrees-30)<.01f && s.control.target.surgePercent==25 &&
      s.written.leftPropulsionPercent>s.written.rightPropulsionPercent,"actual owner executes absolute yaw and surge target on S3");
  if(now==700) check(s.control.headingTargetActive && s.attitude.yawActive &&
      std::abs(s.attitude.headingTargetDegrees+20)<.01f && s.attitude.yawD==0,
      "moving reference changes native target without setpoint derivative kick");
  if(now==900) check(s.control.lastMotionMs==600 && s.attitude.yawActive && s.attitude.yawCorrectionPercent<0,
      "local angle PID keeps running between PC updates");
  if(now==1100) check(!s.control.headingTargetActive && s.attitude.manualYaw && !s.attitude.yawActive,
      "manual yaw overrides the streamed reference through the actual owner");
  if(now>=1420) check(!s.control.attitudeEnabled && !s.control.headingTargetActive && !s.attitude.yawActive &&
      s.written.leftPropulsionPercent==0 && s.written.rightPropulsionPercent==0,
      "STOP disarms explicit reference and neutralizes native outputs");
  if(now==1480) throw Finished{};
}
static void fullRangeStep(const MotionSnapshot &s) {
  check(s.control.target.surgePercent==0 && !s.control.leaseActive,"full-authority yaw uses no translational target");
  if(now==1000 || now==2400) {
    const int expected=(now==1000?100:-100)*robofish::config::attitude::kYawCommandSign;
    check(s.attitude.yawActive && s.attitude.yawCorrectionPercent==expected &&
        s.written.leftPropulsionPercent==expected && s.written.rightPropulsionPercent==-expected,
        "local IMU yaw reaches full +/-100% opposed propellers through the actual owner");
    check(std::abs(s.attitude.yawErrorDegrees)>20 && std::abs(s.attitude.yawErrorDegrees)<22,
        "twenty-degree-scale local heading errors reach full authority");
  }
  const auto geometry=robofish::config::motion::makeGeometry();
  check(s.written.tailFrontDegrees>=geometry.tailFront.minimumDegrees &&
      s.written.tailFrontDegrees<=geometry.tailFront.maximumDegrees &&
      s.written.tailRearDegrees>=geometry.tailRear.minimumDegrees &&
      s.written.tailRearDegrees<=geometry.tailRear.maximumDegrees,
      "full-authority yaw preserves both tail mechanical limits");
  if(now%20==0 && now>0 && now<2500) {
    const auto &a=writes[writes.size()-2].frame;
    const auto &b=writes.back().frame;
    check(std::abs(a.leftPropulsionPercent-b.leftPropulsionPercent)<=4 &&
        std::abs(a.rightPropulsionPercent-b.rightPropulsionPercent)<=4,
        "full-authority yaw preserves per-tick output slew");
    check(a.leftPropulsionPercent*b.leftPropulsionPercent>=0 && a.rightPropulsionPercent*b.rightPropulsionPercent>=0,
        "full-authority yaw submits neutral before propeller reversal");
  }
  if(now>=2520) check(!s.control.attitudeEnabled && !s.attitude.yawActive &&
      s.written.leftPropulsionPercent==0 && s.written.rightPropulsionPercent==0 &&
      s.written.tailFrontDegrees==90 && s.written.tailRearDegrees==90,
      "STOP immediately neutralizes full-authority yaw and remains disarmed");
  if(now==2580) throw Finished{};
}
static void idleStep(const MotionSnapshot &s) {
  using namespace robofish::lora;
  if(now==100) check(s.attitude.yawActive && s.attitude.headingLocked && !s.control.leaseActive &&
      s.control.target.surgePercent==0 && s.written.leftPropulsionPercent==0 && s.written.rightPropulsionPercent==0,
      "enable at rest locks heading with zero initial thrust and no forward target");
  if(now==500 || now==980) {
    const bool right=(now==500?1:-1)*robofish::config::attitude::kYawCommandSign>0;
    check(s.attitude.yawActive && (right?s.attitude.yawCorrectionPercent>0:s.attitude.yawCorrectionPercent<0) &&
        (right?s.written.leftPropulsionPercent>0:s.written.leftPropulsionPercent<0) &&
        s.written.leftPropulsionPercent==-s.written.rightPropulsionPercent,
        "both idle yaw error signs reach opposed propellers with zero common surge");
    check(s.written.tailFrontDegrees!=90 || s.written.tailRearDegrees!=90,"idle yaw also reaches tail allocation");
    check(!s.attitude.pitchActive && !s.attitude.rollActive &&
        s.written.pectoralLeftDegrees==90 && s.written.pectoralRightDegrees==90,"idle yaw never connects tilt preview outputs");
  }
  if(now==1400) check(s.attitude.yawActive && std::abs(s.attitude.headingTargetDegrees)<.01f &&
      s.control.target.surgePercent==0,"forward then ordinary zero target retains the original heading and active PID");
  if(now==1800) check(!s.attitude.yawActive && !s.attitude.headingLocked && s.attitude.yawCorrectionPercent==0,
      "manual stationary steering has priority over PID");
  if(now==1800) check(s.control.target.yawPercent==40 &&
      s.allocation.targetFrame.leftPropulsionPercent>s.allocation.targetFrame.rightPropulsionPercent,
      "manual right steering preserves its existing target mixer direction");
  // The preceding PID can still be turning left; allow the existing ESC ramp
  // to submit neutral before checking the delivered manual right-turn output.
  if(now==1960) check(s.written.leftPropulsionPercent>s.written.rightPropulsionPercent,
      "manual right steering reaches the original output direction after neutral-crossing slew");
  if(now==2100) check(s.attitude.yawActive && std::abs(s.attitude.headingTargetDegrees-15)<.5f,
      "manual steering release captures the new heading at zero surge");
  const bool neutral=s.written.leftPropulsionPercent==0 && s.written.rightPropulsionPercent==0 &&
      s.written.tailFrontDegrees==90 && s.written.tailRearDegrees==90 &&
      s.written.pectoralLeftDegrees==90 && s.written.pectoralRightDegrees==90;
  if(now==2800) check(!s.control.attitudeEnabled && !s.attitude.yawActive && neutral,"switching hold OFF centers idle actuators");
  if(now>=3320 && now<=3700) check(!s.control.attitudeEnabled && !s.attitude.yawActive &&
      !s.attitude.headingLocked && neutral,"explicit STOP remains disarmed despite subsequent fresh IMU samples");
  if(now==4140) check(s.attitude.yawActive && s.attitude.yawCorrectionPercent*robofish::config::attitude::kYawCommandSign>0 &&
      s.written.leftPropulsionPercent*robofish::config::attitude::kYawCommandSign>0,"a fresh explicit enable can resume calibrated idle hold");
  if(now==4480) check(!s.attitude.yawActive && !s.attitude.headingLocked &&
      s.attitude.reason==robofish::attitude::Reason::Stale &&
      s.written.leftPropulsionPercent==0 && s.written.rightPropulsionPercent==0,
      "stale local IMU stops idle correction and drops the old reference");
  if(now==4560) check(s.attitude.yawActive && std::abs(s.attitude.headingTargetDegrees-20)<.02f,
      "fresh IMU after a gap reanchors locally without a PC control update");
  if(now==4760) {
    bool stopped=false;
    check(s.control.maintenance && !s.control.attitudeEnabled && !s.attitude.yawActive && neutral &&
        xQueueReceive(otaResults,&stopped,0)==pdTRUE && stopped,"OTA still requires successful six-channel neutral output");
    throw Finished{};
  }
}
void vTaskDelayUntil(TickType_t *wake,TickType_t period) {
  MotionSnapshot s; check(xQueuePeek(values,&s,0)==pdTRUE,"HTTP receives copied snapshots");
  check(s.loraReady && s.imu.uartReady && !s.lora.txEnabled,"LoRa receive-only, IMU UART ready");
  if(idleScenario()) {
    check(period==10 && writes.back().at==now/20*20,"idle hold preserves the original 10/20 ms owner cadence");
    if(headingScenario()) headingStep(s); else if(fullRangeScenario()) fullRangeStep(s); else idleStep(s);
    *wake+=period; now=*wake;
    if(now%20==0) idleImu();
    if(headingScenario()) {
      if(now==30) idleCommand(1,robofish::lora::kHeadingTarget,25,0,30);
      if(now==600) idleCommand(2,robofish::lora::kHeadingTarget,25,0,-20);
      if(now==1000) idleCommand(3,robofish::lora::kTarget,0,40);
      if(now==1200) idleCommand(4,robofish::lora::kHeadingTarget,0,0,179);
      if(now==1400) idleCommand(5,17);
      return;
    }
    if(now==30 && wifiIdleScenario()) idleCommand(1,robofish::lora::kAttitudeOn);
    if(fullRangeScenario()) {
      if(now==2500) idleCommand(2,17);
      return;
    }
    if(now==1000) idleCommand(2,robofish::lora::kTarget,35);
    if(now==1200) idleCommand(3,robofish::lora::kTarget);
    if(now==1600) idleCommand(4,robofish::lora::kTarget,0,40);
    if(now==2000) idleCommand(5,robofish::lora::kTarget);
    if(now==2400) idleCommand(6,robofish::lora::kAttitudeOff);
    if(now==2860) idleCommand(7,robofish::lora::kAttitudeOn);
    if(now==3300) idleCommand(8,17);
    if(now==3760) idleCommand(9,robofish::lora::kAttitudeOn);
    if(now==4700) { const bool stop=true; check(xQueueSend(otaRequests,&stop,0)==pdTRUE,"idle OTA stop fits existing queue"); }
    return;
  }
  check(s.imu.hasAngles && s.imu.angles.x==45 && s.imu.angles.y==-45 &&
      s.imu.angles.z==90,"actual UART2 bytes decode into native-axis angles");
  check(period==10 && writes.back().at==now/20*20,"original 10/20 ms cadence");
  if (scenario=="normal") {
    if(now==500) check(s.written.leftPropulsionPercent==100,"original 500 ms startup ramp");
    if(now==620) check(s.control.softStopping && s.written.leftPropulsionPercent==96,"ordinary LoRa STOP ramps");
    if(now==1030) check(!s.control.leaseActive && s.written.leftPropulsionPercent==0 &&
        s.written.rightPropulsionPercent==0 && s.lora.duplicates==1 && s.lora.stale==1,"SAFE, duplicate and stale replay barrier");
    if(now==1200) check(s.control.maintenance && otaResults->entries.empty(),"OTA waits through partial neutral write");
    if(now==1240) { bool ok=false; check(xQueueReceive(otaResults,&ok,0)==pdTRUE && ok &&
        s.control.loraResult.code==robofish::lora::ControlCode::Maintenance,"OTA returns success only after full neutral and rejects LoRa"); throw Finished{}; }
  } else if (scenario=="clock_race") {
    check(s.nowMs>=s.control.lastMotionMs,"published snapshot is not older than its accepted command");
    if(now==50) {
      check(s.wifi.accepted==2 && s.wifi.rejected==0 && s.control.leaseActive && s.wifi.sequence==1,
            "HTTP published after loop time capture is accepted using current owner time");
      throw Finished{};
    }
  } else if (scenario=="hold" || scenario=="wifi_hold") {
    if(now>=1000 && now<=30000) check(s.control.leaseActive && !s.control.expired &&
        s.written.leftPropulsionPercent==100 && s.written.rightPropulsionPercent==100 &&
        s.written.pectoralLeftDegrees!=90 && s.written.pectoralRightDegrees!=90 &&
        s.control.lastMotionMs==0,"one accepted target holds thrust and fins for 30 seconds without renewal");
    if(now==30000 && scenario=="hold") check(s.lora.duplicates==1,"duplicate radio target stays rejected");
    if(now==30000 && scenario=="wifi_hold") check(s.wifi.rejected==1,"duplicate Wi-Fi target stays rejected");
    if(now==30040) {
      check(!s.control.leaseActive && s.control.target.surgePercent==0 && s.control.target.pitchPercent==0 &&
          s.written.leftPropulsionPercent==0 && s.written.rightPropulsionPercent==0 &&
          s.written.tailFrontDegrees==90 && s.written.tailRearDegrees==90 &&
          s.written.pectoralLeftDegrees==90 && s.written.pectoralRightDegrees==90,
          "explicit STOP still clears held motion and immediately submits six-channel neutral");
      throw Finished{};
    }
  } else if (scenario=="attitude") {
    if(now<600) check(s.control.attitudeEnabled,"LoRa attitude request reaches the owner");
    if(now>=20 && now<600) check(s.attitude.requested && s.attitude.reason==(s.control.attitudeMotionAllowed()?
        robofish::attitude::Reason::NoImu:robofish::attitude::Reason::WaitingMotion) &&
        !s.attitude.pitchActive && !s.attitude.rollActive && !s.attitude.yawActive && !s.attitude.headingLocked,
        "angles without measured rates cannot drive feedback even with a mapped mount");
    if(now==500) check(s.written.leftPropulsionPercent==100,"enabling assistance did not alter manual thrust");
    if(now==620) {
      check(!s.control.leaseActive && !s.control.attitudeEnabled && s.written.leftPropulsionPercent==0 &&
        s.written.rightPropulsionPercent==0 && s.written.pectoralLeftDegrees==90 &&
        s.written.pectoralRightDegrees==90,"STOP retains full neutral authority with attitude requested");
      throw Finished{};
    }
  } else {
    if(now==150) check(s.control.outputRecovery && !s.control.leaseActive,"existing five-failure neutral recovery");
    if(now==170) check(s.control.loraResult.code==robofish::lora::ControlCode::OutputRecovery,"motion rejected during recovery");
    if(scenario=="recovery" && now==270) check(!s.control.leaseActive && !s.control.outputRecovery,"recovery does not replay old command");
    if(scenario=="recovery" && now==310) { check(s.control.leaseActive,"new sequence explicitly resumes recovered outputs"); throw Finished{}; }
    if(scenario=="fault" && now==440) { check(s.control.outputFault && !s.control.leaseActive &&
        s.control.loraResult.code==robofish::lora::ControlCode::OutputFault,"15 failures latch despite later successful output"); throw Finished{}; }
  }
  *wake+=period; now=*wake;
  if(scenario=="normal") {
    if(now==610) send(2,1);
    if(now==710) send(3,9);
    if(now==1010) { send(4,17); send(4,17); send(0,2); }
    if(now==1190) { const bool stop=true; check(xQueueSend(otaRequests,&stop,0)==pdTRUE,"OTA stop published"); }
    if(now==1230) send(5,2);
  } else if(scenario=="hold") {
    if(now==1000) send(2,2);
    if(now==30010) send(3,17);
  } else if(scenario=="wifi_hold") {
    if(now==1000) postWifi(robofish::wifi::Action::Target,1);
    if(now==30010) postWifi(robofish::wifi::Action::Stop,2);
  } else if(scenario=="attitude") {
    if(now==600) send(2,17);
  } else {
    if(now==160) send(2,2);
    if(now==260) send(1,2);
    if(now==280 && scenario=="recovery") send(3,2);
    if(now==430 && scenario=="fault") send(3,2);
  }
}
static void protocolChecks() {
  robofish::lora::Command headingWire;
  check(robofish::lora::parseLine("101,1,23120,42,26,3,-17955,350,0,1000,6570",headingWire) &&
      headingWire.code==robofish::lora::kHeadingTarget && std::abs(headingWire.headingCwDegrees+179.55f)<.001f &&
      headingWire.axes.surge==35,"PC golden angle frame decodes with hundredth-degree and tenth-percent units");
  using namespace robofish::lora;
  Command c;
  check(parseLine("100,1,0,65535,2,1000",c) && c.sequence==65535,"wire protocol retained");
  check(!parseLine("100,2,0,1,2",c) && !parseLine("100,1,0,1,20",c),"destination and command validation");
  check(sequenceAfter(0,65535) && !sequenceAfter(32768,0),"16-bit sequence wrap and half-range rejection");
  LineDecoder decoder;
  for(unsigned i=0;i<robofish::config::lora::kMaxLineLength+1;++i) decoder.consume('1',c);
  check(decoder.consume('\n',c)==ParseResult::Rejected,"overlong line discarded");
  ControlState state; state.stopPending=false;
  check(state.accept(Command{1,6,true,1000},0) && state.accept(Command{2,2,false,0},10),"independent fin hold plus forward");
  check(state.target.pitchPercent==100 && state.target.surgePercent==100,"fin command survives horizontal motion");
  check(state.accept(Command{3,1,false,0},20) && state.target.pitchPercent==100 &&
      state.target.surgePercent==0,"ordinary STOP preserves fin hold");
  check(state.accept(Command{4,7,false,0},30) && state.target.pitchPercent==0 &&
      state.accept(Command{5,7,false,0},40) && state.target.pitchPercent==-100,"original UP to center to DOWN mapping");
  check(!state.accept(Command{6,8,true,1000},50) &&
      state.loraResult.code==ControlCode::Unsupported,"absent pump command rejected");
  check(state.accept(Command{7,17,false,0},60) && state.stopPending && !state.leaseActive,"SAFE immediate neutral barrier");
  check(!state.accept(Command{8,2,false,0},61) && state.loraResult.code==ControlCode::Busy,"fresh motion cannot skip neutral barrier");
  robofish::imu::Jy61p imu; robofish::imu::Status status;
  for(auto b:packet(0x51,2048,-2048,32767)) imu.consume(b,1,status);
  for(auto b:packet(0x52,16384,-16384,0)) imu.consume(b,2,status);
  check(status.acceleration.x==1 && status.acceleration.y==-1 &&
      status.angularVelocity.x==1000 && status.angularVelocity.y==-1000,"signed IMU units and scale");
  auto bad=packet(0x53,8192,0,0); bad[10]^=1;
  for(auto b:bad) imu.consume(b,3,status);
  check(!status.hasAngles && status.checksumErrors==1,"bad IMU checksum never updates angles");
  for(auto b:packet(0x53,8192,-8192,16384)) imu.consume(b,4,status);
  check(status.frames==3 && status.hasAngles && status.angles.z==90,"IMU resynchronizes after corruption");
  imu.consume(0x55,5,status); imu.reset();
  for(auto b:packet(0x53,0,0,-16384)) imu.consume(b,6,status);
  check(status.angles.z==-90 && status.anglesMs==6,"stream interruption resets partial IMU frame");
}
int main(int argc,char **argv) {
  if(argc>1) scenario=argv[1];
  protocolChecks();
  try { app_main(); } catch(const Finished &) { std::printf("PASS: %u LoRa/IMU owner checks (%s)\n",checks,scenario.c_str()); return 0; }
  check(false,"fixture must complete");
}
