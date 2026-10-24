#pragma once
#include <cmath>
#include <cstring>
#include "Mode/Motion/MotionMode.h"
#include "config/messages/WifiControl.h"
#include "config/AppConfig.h"
#include "config/RadioConfig.h"
#include "config/AttitudeConfig.h"
#include "config/messages/LoRaCommand.h"

// One owner: the 10 ms application loop arbitrates LoRa and queued HTTP requests.
struct ControlState {
  robofish::motion::Request target = [] {
    robofish::motion::Request value;
    value.mode = DriveMode::Prop;
    return value;
  }();
  robofish::wifi::Status wifi;
  robofish::lora::ControlResult loraResult;
  uint16_t knob = 0; // 原遥控器旋钮 0..1000，对应 50..100% 档位。
  uint32_t lastMotionMs = 0;
  uint32_t lastLoRaMs = 0;
  bool loRaSeen = false, wifiOwner = false;
  bool loraHealthy(uint32_t now) const {
    return loRaSeen && uint32_t(now - lastLoRaMs) < robofish::config::lora::kLinkTimeoutMs;
  }
  const char *controlSource() const { return wifiOwner ? "wifi" : loRaSeen ? "lora" : "none"; }
  bool leaseActive = false;
  bool expired = false;
  bool stopPending = true;
  bool maintenance = false;
  bool softStopping = false;
  bool attitudeEnabled = false; // Explicit yaw-hold authorization, including zero surge.
  uint32_t attitudeGeneration = 0;
  robofish::wifi::YawPid yawPid{robofish::config::attitude::kYawKp,
      robofish::config::attitude::kYawKi,robofish::config::attitude::kYawKd};
  static bool validPid(const robofish::wifi::YawPid &pid) {
    return std::isfinite(pid.kp) && pid.kp>=0 && pid.kp<=100 &&
        std::isfinite(pid.ki) && pid.ki>=0 && pid.ki<=100 &&
        std::isfinite(pid.kd) && pid.kd>=0 && pid.kd<=100;
  }
  uint8_t outputFailureStreak = 0;
  uint8_t failedMask = 0;
  bool outputRecovery = false;
  bool outputFault = false;

  bool attitudeMotionAllowed() const {
    // Zero targets retain heading hold. Manual yaw yields in holdHeading();
    // reverse, explicit STOP, OTA and output faults retain their priority.
    return attitudeEnabled && target.surgePercent >= 0 &&
        !expired && !maintenance && !stopPending && !outputRecovery && !outputFault;
  }
  void setMode(DriveMode mode) { target.mode=attitudeEnabled?DriveMode::Hybrid:mode; }
  void setAttitude(bool enabled) {
    // Reuse the neutral barrier to center a tail previously driven by idle hold.
    if (attitudeEnabled && !enabled && !leaseActive) stopPending = true;
    attitudeEnabled=enabled; ++attitudeGeneration;
    if (enabled) setMode(DriveMode::Hybrid); // Tail and both props share the yaw demand.
  }

  // The owner calls this after each actual BSP submission; API success is not
  // physical position feedback. Recovery cannot resurrect a previous session.
  void observeOutput(uint8_t writtenMask) {
    if (writtenMask == 63) {
      outputFailureStreak = failedMask = 0;
      outputRecovery = false;
      return; // A latched fault survives even a later successful neutral write.
    }
    failedMask = static_cast<uint8_t>((~writtenMask) & 63);
    if (outputFailureStreak < appconfig::kOutputFaultFailures) ++outputFailureStreak;
    if (outputFailureStreak >= appconfig::kOutputNeutralFailures && !outputRecovery) {
      stop();
      outputRecovery = true;
      std::memset(wifi.session, 0, sizeof(wifi.session));
      wifi.sequence = 0;
    }
    if (outputFailureStreak >= appconfig::kOutputFaultFailures) outputFault = true;
  }
  const char *safetyState() const {
    if (outputFault) return "OUTPUT_FAULT";
    if (outputRecovery) return "OUTPUT_RECOVERY";
    if (maintenance) return "OTA_LOCKED";
    if (outputFailureStreak) return "OUTPUT_WARNING";
    if (expired) return "LEASE_EXPIRED";
    if (attitudeMotionAllowed()) return "RUNNING";
    if (softStopping) return "STOPPING";
    return leaseActive ? "RUNNING" : "WAIT_COMMAND";
  }

  void clearTarget() {
    const auto mode = target.mode;
    target = {};
    target.mode = mode;
    leaseActive = false;
    softStopping = false;
    knob = 0;
  }
  void stop() {
    clearTarget();
    attitudeEnabled = false; // Neutral acknowledgement must not restart idle hold.
    ++attitudeGeneration;
    expired = false;
    stopPending = true;
  }
  bool expire(uint32_t now) {
    if (!appconfig::kLeaseMs || !leaseActive || uint32_t(now - lastMotionMs) < appconfig::kLeaseMs) return false;
    clearTarget();
    expired = true;
    return true;
  }
  bool accept(const robofish::lora::Command &command, uint32_t now) {
    using robofish::lora::ControlCode;
    using namespace robofish::lora;
    if (command.code < 1 || command.code > kYawPid ||
        (command.hasKnob && command.knob > 1000)) return false;
    if (command.code == kYawPid && !validPid(command.yawPid)) return false;
    if (command.code == kTarget && (!validAxis(command.axes.surge) || !validAxis(command.axes.yaw) ||
        !validAxis(command.axes.heave) || !std::isfinite(command.propulsionScalePercent) ||
        command.propulsionScalePercent < 0 || command.propulsionScalePercent > 100)) return false;
    if ((command.code == kTarget || command.code == kMode) &&
        command.mode != DriveMode::Prop && command.mode != DriveMode::Bionic && command.mode != DriveMode::Hybrid)
      return false;
    loRaSeen = true; lastLoRaMs = now;
    // Returning primary control clears the fallback goal before any new motion.
    if (wifiOwner) { stop(); wifiOwner = false; }
    if (command.code == kHeartbeat) return true; // Liveness never renews the motion lease.
    loraResult = {command, ControlCode::Invalid, now, true};
    const auto finish = [&](ControlCode code) {
      loraResult.code = code;
      return code == ControlCode::Accepted;
    };
    if (maintenance) return finish(ControlCode::Maintenance);
    if (command.code == 8 || command.code == 18 || command.code == 19)
      return finish(ControlCode::Unsupported);
    if (command.code != 17 && command.code != 1) {
      if (outputFault) return finish(ControlCode::OutputFault);
      if (outputRecovery) return finish(ControlCode::OutputRecovery);
      if (stopPending) return finish(ControlCode::Busy);
    }
    if (command.code == 17) {
      stop(); lastMotionMs = now;
      return finish(ControlCode::Accepted);
    }
    expire(now);
    if (command.code == kYawPid) { yawPid=command.yawPid; return finish(ControlCode::Accepted); }
    if (command.code == kAttitudeOn || command.code == kAttitudeOff) {
      setAttitude(command.code == kAttitudeOn);
      return finish(ControlCode::Accepted);
    }
    if (command.code == kMode) { setMode(command.mode); return finish(ControlCode::Accepted); }
    if (command.code == kTarget) {
      target = {};
      setMode(command.mode);
      target.surgePercent = command.axes.surge;
      target.yawPercent = command.axes.yaw;
      target.pitchPercent = command.axes.heave;
      target.propulsionScale = command.propulsionScalePercent * 0.01f;
      target.outputScale = 1;
      target.allowFlowAssist = false;
      target.enabled = leaseActive = true;
      softStopping = command.axes.surge == 0 && command.axes.yaw == 0 && command.axes.heave == 0;
      expired = false; lastMotionMs = now;
      return finish(ControlCode::Accepted);
    }
    if (command.hasKnob) knob = command.knob;
    target.outputScale = (robofish::config::lora::kGearMinimumPercent +
        (robofish::config::lora::kGearMaximumPercent - robofish::config::lora::kGearMinimumPercent) *
        knob / 1000.0f) * 0.01f;
    // 模式设置本身不启动运动、不续租。顺序沿用原 ino：螺旋桨 -> 仿生 -> 混合。
    if (command.code == 5 || (command.code >= 14 && command.code <= 16)) {
      setMode(command.code == 14 ? DriveMode::Prop : command.code == 15 ? DriveMode::Bionic :
          command.code == 16 ? DriveMode::Hybrid : target.mode == DriveMode::Prop ? DriveMode::Bionic :
          target.mode == DriveMode::Bionic ? DriveMode::Hybrid : DriveMode::Prop);
      return finish(ControlCode::Accepted);
    }
    switch (command.code) {
      case 1: target.surgePercent = target.yawPercent = 0; break;
      case 2: target.surgePercent = 100; target.yawPercent = 0; break;
      case 3: target.surgePercent = 0; target.yawPercent = -100; break;
      case 4: target.surgePercent = 0; target.yawPercent = 100; break;
      case 6: target.pitchPercent = 100; break;
      case 7: target.pitchPercent = target.pitchPercent > 0 ? 0 : -100; break;
      case 9: target.surgePercent = -100; target.yawPercent = 0; break;
      case 10: target.surgePercent = 100; target.yawPercent = -100; break;
      case 11: target.surgePercent = target.yawPercent = 100; break;
      case 12: target.surgePercent = target.yawPercent = -100; break;
      case 13: target.surgePercent = -100; target.yawPercent = 100; break;
      default: return finish(ControlCode::Invalid);
    }
    // 输出恢复/急停的中立屏障不能被普通停止帧解除或续上旧运动。
    if (outputRecovery || outputFault || stopPending) {
      stop();
      return finish(ControlCode::Accepted);
    }
    target.allowFlowAssist = false; // 浮沉只调整鱼鳍，与当前固件一致。
    target.propulsionScale = 1;
    target.enabled = leaseActive = true;
    softStopping = target.surgePercent == 0 && target.yawPercent == 0 && target.pitchPercent == 0;
    expired = false;
    lastMotionMs = now;
    return finish(ControlCode::Accepted);
  }
  static bool validSession(const char (&session)[robofish::wifi::kSessionCapacity]) {
    for (unsigned i = 0; i < sizeof(session); ++i) {
      const auto c = static_cast<unsigned char>(session[i]);
      if (c == 0) return i != 0;
      if (c < 32 || c > 126) return false;
    }
    return false;
  }
  static bool validAxis(float value) {
    return std::isfinite(value) && value >= -100 && value <= 100;
  }
  robofish::wifi::Result result(const robofish::wifi::Command &command,
                               robofish::wifi::Code code) {
    robofish::wifi::Result value;
    value.requestId = command.requestId;
    value.code = wifi.lastCode = code;
    if (code == robofish::wifi::Code::Accepted) ++wifi.accepted;
    else ++wifi.rejected;
    std::memcpy(value.session, wifi.session, sizeof(value.session));
    value.sequence = wifi.sequence;
    value.mode = target.mode;
    value.axes = {target.surgePercent, target.yawPercent, target.pitchPercent};
    value.leaseActive = leaseActive;
    value.stopPending = stopPending;
    value.attitudeEnabled = attitudeEnabled;
    return value;
  }
  robofish::wifi::Result accept(const robofish::wifi::Command &command, uint32_t now) {
    using namespace robofish::wifi;
    if (maintenance) return result(command, Code::Maintenance);
    if (!validSession(command.session)) return result(command, Code::Invalid);
    // The HTTP caller only waits this long. A request left behind a failed
    // neutral-output barrier must not acquire control or start motion later.
    if (uint32_t(now - command.receivedMs) >= appconfig::kCommandResultTimeoutMs)
      return result(command, Code::Expired);
    if (command.action != Action::Stop) {
      if (outputFault) return result(command, Code::OutputFault);
      if (outputRecovery) return result(command, Code::OutputRecovery);
    }
    if (command.action == Action::Acquire) {
      if (std::strcmp(command.session, wifi.session) != 0) {
        if (!loraHealthy(now)) stop();
        std::memcpy(wifi.session, command.session, sizeof(wifi.session));
        wifi.sequence = 0;
      }
      return result(command, Code::Accepted);
    }
    if (command.action != Action::Target && command.action != Action::Mode &&
        command.action != Action::Stop && command.action != Action::AttitudeOn &&
        command.action != Action::AttitudeOff && command.action != Action::YawPid) return result(command, Code::Invalid);
    if (command.action == Action::YawPid && !validPid(command.yawPid)) return result(command, Code::Invalid);
    if (std::strcmp(command.session, wifi.session) != 0)
      return result(command, Code::SessionConflict);
    if (command.sequence == 0 || command.sequence <= wifi.sequence)
      return result(command, Code::StaleSequence);
    if (command.action != Action::Stop && loraHealthy(now))
      return result(command, Code::LoRaActive);
    if (command.action == Action::Target && stopPending && loRaSeen)
      return result(command, Code::NeutralPending);
    if (command.action == Action::Target) {
      if (!validAxis(command.axes.surge) || !validAxis(command.axes.yaw) ||
          !validAxis(command.axes.heave) || !std::isfinite(command.propulsionScalePercent) ||
          command.propulsionScalePercent < 0 || command.propulsionScalePercent > 100)
        return result(command, Code::Invalid);
    }
    if (command.action == Action::Mode && command.mode != DriveMode::Bionic &&
        command.mode != DriveMode::Prop && command.mode != DriveMode::Hybrid)
      return result(command, Code::Invalid);

    expire(now);
    if (command.action != Action::Stop && !wifiOwner && leaseActive) {
      stop();
      return result(command, Code::NeutralPending);
    }
    wifiOwner = true;
    wifi.sequence = command.sequence;
    if (command.action == Action::Stop) {
      stop();
    } else if (command.action == Action::Target && command.axes.surge == 0 &&
        command.axes.yaw == 0 && command.axes.heave == 0) {
      // Ordinary key/button release ramps the current successful ESC outputs
      // toward neutral. Explicit STOP, takeover and OTA remain immediate;
      // lease expiry stops thrust immediately and slews the servos to center.
      clearTarget();
      expired = false;
      softStopping = leaseActive = true;
      lastMotionMs = command.receivedMs;
    } else if (command.action == Action::YawPid) {
      yawPid = command.yawPid; // Settings never start/renew motion or change the held target.
    } else if (command.action == Action::AttitudeOn || command.action == Action::AttitudeOff) {
      setAttitude(command.action == Action::AttitudeOn);
    } else if (command.action == Action::Mode) {
      setMode(command.mode); // Configuration alone never starts or renews motion.
    } else {
      const auto mode = target.mode;
      target = {};
      target.mode = mode;
      target.surgePercent = command.axes.surge;
      target.yawPercent = command.axes.yaw;
      target.pitchPercent = command.axes.heave;
      target.propulsionScale = command.propulsionScalePercent * 0.01f;
      // Heave is an independent fin position: it never creates surge or tail drive.
      target.allowFlowAssist = false;
      target.outputScale = 1; // Wi-Fi axes are already scaled percentages.
      target.enabled = leaseActive = true;
      softStopping = false;
      expired = false;
      lastMotionMs = command.receivedMs;
    }
    return result(command, Code::Accepted);
  }
};
