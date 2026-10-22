#pragma once
#include <stddef.h>
#include <stdint.h>

namespace robofish::bsp {
enum class UartTxState : uint8_t { Idle, Busy, Error };
struct UartWriteResult { int bytes = 0; bool error = false; };

// UART1 唯一 owner；read/TX/stop 均由主控制循环调用。
class LoRaUart {
 public:
  bool begin();
  int read(uint8_t *buffer, size_t capacity); // -1：流中断，解码器丢残行到换行。
  int writeAvailable(const char *data, size_t length);
  UartTxState txState() const;
  UartWriteResult tryWriteFrame(const char *data, size_t length);
  void stop();
  bool ready() const { return ready_; }

 private:
  bool ready_ = false;
};
}
