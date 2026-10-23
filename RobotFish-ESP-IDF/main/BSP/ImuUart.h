#pragma once
#include <stddef.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

namespace robofish::bsp {
// UART2 唯一 owner 为主控制循环；只接收，不向 JY61P 写配置或校准命令。
class ImuUart {
 public:
  bool begin();
  int read(uint8_t *buffer, size_t capacity); // 0 无数据，-1 流中断；不等待。
#ifdef ROBOTFISH_USB_DIAGNOSTICS
  bool setTestBaud(unsigned baud); // 仅 USB 联调时改变 MCU 接收速率，不写传感器。
#endif
 private:
  bool interrupted();
  bool ready_ = false;
  QueueHandle_t events_ = nullptr;
};
}
