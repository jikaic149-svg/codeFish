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
inline constexpr uint32_t kLeaseMs = 3000;
inline constexpr uint32_t kOtaStopTimeoutMs = 300;
inline constexpr uint32_t kCommandResultTimeoutMs = 150;
inline constexpr unsigned kMaximumImageBytes = 0x330000;
}
