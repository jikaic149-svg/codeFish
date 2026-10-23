#include "BSP/ImuUart.h"
#include "config/BoardConfig.h"
#include "driver/uart.h"
#include "esp_log.h"

namespace robofish::bsp {
namespace {
constexpr uart_port_t kPort = UART_NUM_2;
constexpr unsigned kEventCount = 16;
}
bool ImuUart::begin() {
  if (ready_) return true;
  uart_config_t config{};
  config.baud_rate = board::config::kImuBaud;
  config.data_bits = UART_DATA_8_BITS;
  config.parity = UART_PARITY_DISABLE;
  config.stop_bits = UART_STOP_BITS_1;
  config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  config.source_clk = UART_SCLK_DEFAULT;
  esp_err_t error = uart_param_config(kPort, &config);
  if (error == ESP_OK) error = uart_set_pin(kPort, board::config::kImuTxGpio,
      board::config::kImuRxGpio, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  if (error == ESP_OK) error = uart_driver_install(kPort,
      board::config::kImuReceiveBufferBytes, 0, kEventCount, &events_, 0);
  ready_ = error == ESP_OK;
  if (!ready_) ESP_LOGE("imu_uart", "UART2 initialization failed: %s", esp_err_to_name(error));
  return ready_;
}
bool ImuUart::interrupted() {
  uart_event_t event{};
  bool error = false;
  for (unsigned i = 0; events_ && i < kEventCount &&
       xQueueReceive(events_, &event, 0) == pdTRUE; ++i) {
    if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL ||
        event.type == UART_FRAME_ERR || event.type == UART_PARITY_ERR || event.type == UART_BREAK)
      error = true;
  }
  if (error) { uart_flush_input(kPort); xQueueReset(events_); }
  return error;
}
int ImuUart::read(uint8_t *buffer, size_t capacity) {
  if (!ready_ || !buffer || !capacity) return 0;
  if (interrupted()) return -1;
  const int count = uart_read_bytes(kPort, buffer, capacity, 0);
  return interrupted() ? -1 : count;
}
#ifdef ROBOTFISH_USB_DIAGNOSTICS
bool ImuUart::setTestBaud(unsigned baud) {
  if (!ready_ || uart_set_baudrate(kPort, baud) != ESP_OK) return false;
  uart_flush_input(kPort);
  xQueueReset(events_);
  return true;
}
#endif
}
