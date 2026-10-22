#pragma once
#include <stddef.h>
#include "config/messages/WifiControl.h"

namespace robofish::wifi {
inline constexpr size_t kMaximumControlJsonBytes = 512;
// Parse and validate before enqueueing. Invalid input leaves command untouched.
bool parseControlJson(const char *data, size_t length, Command &command);
}
