#pragma once
#include <stdint.h>

enum class DriveMode : uint8_t { Idle = 0, Bionic, Prop, Hybrid };
enum class MotionAction : uint8_t { Stopped = 0, Forward, Backward, Left, Right };
