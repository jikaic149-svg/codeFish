#pragma once
#include <cmath>
#include <cstring>
#include "Mode/Motion/MotionMode.h"
#include "config/messages/WifiControl.h"
#include "config/AppConfig.h"

// One owner: the 10 ms application loop. HTTP only sends copied commands.
struct ControlState {
  robofish::motion::Request target = [] {
    robofish::motion::Request value;
    value.mode = DriveMode::Prop;
    return value;
  }();
  robofish::wifi::Status wifi;
  uint32_t lastMotionMs = 0;
  bool leaseActive = false;
  bool expired = false;
  bool stopPending = true;
  bool maintenance = false;
  bool softStopping = false;
  uint8_t outputFailureStreak = 0;
  uint8_t failedMask = 0;
  bool outputRecovery = false;
  bool outputFault = false;

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
    if (softStopping) return "STOPPING";
    return leaseActive ? "RUNNING" : "WAIT_COMMAND";
  }

  void clearTarget() {
    const auto mode = target.mode;
    target = {};
    target.mode = mode;
    leaseActive = false;
    softStopping = false;
  }
  void stop() {
    clearTarget();
    expired = false;
    stopPending = true;
  }
  bool expire(uint32_t now) {
    if (!leaseActive || uint32_t(now - lastMotionMs) < appconfig::kLeaseMs) return false;
    clearTarget();
    expired = true;
    return true;
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
        stop();
        std::memcpy(wifi.session, command.session, sizeof(wifi.session));
        wifi.sequence = 0;
      }
      return result(command, Code::Accepted);
    }
    if (command.action != Action::Target && command.action != Action::Mode &&
        command.action != Action::Stop) return result(command, Code::Invalid);
    if (std::strcmp(command.session, wifi.session) != 0)
      return result(command, Code::SessionConflict);
    if (command.sequence == 0 || command.sequence <= wifi.sequence)
      return result(command, Code::StaleSequence);
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
    } else if (command.action == Action::Mode) {
      target.mode = command.mode; // Configuration alone never starts or renews motion.
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
