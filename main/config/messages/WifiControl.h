#pragma once
#include <stdint.h>
#include "config/messages/MotionTypes.h"

namespace robofish::wifi {
inline constexpr unsigned kSessionCapacity = 65;
inline constexpr unsigned kQueueDepth = 8;

enum class Action : uint8_t { Acquire, Target, Mode, Stop, AttitudeOn, AttitudeOff };
enum class Code : uint8_t { Accepted, Invalid, SessionConflict, StaleSequence, Expired, Maintenance,
                          OutputRecovery, OutputFault, LoRaActive, NeutralPending };
struct Axes { float surge = 0, yaw = 0, heave = 0; };

// HTTP owns publication; the 10 ms control owner consumes at most eight copies.
// Percentages are body requests. receivedMs is the original HTTP receive time.
struct Command {
  uint32_t requestId = 0, sequence = 0, receivedMs = 0;
  char session[kSessionCapacity]{};
  Action action = Action::Stop;
  DriveMode mode = DriveMode::Prop;
  Axes axes;
  float propulsionScalePercent = 100; // Optional target field; manual clients default to full range.
};
struct Result {
  uint32_t requestId = 0;
  Code code = Code::Invalid;
  char session[kSessionCapacity]{};
  uint32_t sequence = 0;
  DriveMode mode = DriveMode::Prop;
  Axes axes;
  bool leaseActive = false, stopPending = false, attitudeEnabled = false;
};
struct Status {
  char session[kSessionCapacity]{};
  uint32_t sequence = 0, accepted = 0, rejected = 0, resultDrops = 0;
  Code lastCode = Code::Accepted;
};
} // namespace robofish::wifi
