#pragma once
#include <stdint.h>
#include "config/messages/ActuatorFrame.h"

// 仅由主控制循环持有和调用；HTTP/OTA 通过队列请求该循环操作。
namespace board {
bool begin();
bool applyFrame(const robofish::ActuatorFrame &frame, bool pump);
// 位 0..5：前尾、后尾、左胸、右胸、左 ESC、右 ESC。
// 六路 LEDC 更新不是原子事务；置位只表示该通道 API 写入成功。
uint8_t applyFrameMask(const robofish::ActuatorFrame &frame);
bool setPump(bool enabled);
uint32_t nowMs();
}
