#pragma once
#include <stdint.h>

namespace appconfig {
// From the actual ino constants; comments in that sketch contain older SSIDs.
inline constexpr char kSsid[] = "RobotFishLOAR1";
inline constexpr char kWifiPassword[] = "12345678";
inline constexpr char kOtaUser[] = "fish";
inline constexpr char kOtaPassword[] = "fishota";
inline constexpr uint32_t kPollMs = 10;
inline constexpr uint32_t kControlMs = 20;
// Motion hold timeout, milliseconds: 0 keeps the last accepted target until a
// new target or STOP. Event-driven handsets do not repeat unchanged commands.
inline constexpr uint32_t kLeaseMs = 0;
// Consecutive incomplete six-channel PWM submissions, at the 20 ms output period.
// 1..4: warning; 5 (~100 ms): neutral/re-authorize; 15 (~300 ms): latched stop.
inline constexpr uint8_t kOutputNeutralFailures = 5;
inline constexpr uint8_t kOutputFaultFailures = 15;
static_assert(kOutputNeutralFailures > 0 && kOutputFaultFailures > kOutputNeutralFailures);
inline constexpr uint32_t kOtaStopTimeoutMs = 300;
inline constexpr uint32_t kCommandResultTimeoutMs = 150;
inline constexpr unsigned kMaximumImageBytes = 0x330000;
}
