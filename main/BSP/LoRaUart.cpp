#include "BSP/LoRaUart.h"
#include "config/BoardConfig.h"
#include "config/RadioConfig.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

namespace robofish::bsp {
namespace {
constexpr uart_port_t kPort = UART_NUM_1;
QueueHandle_t receiveEvents = nullptr;

bool receiveInterrupted() {
  uart_event_t event{};
  bool interrupted = false;
  for (unsigned i = 0; receiveEvents && i < config::radio::kReceiveEventCount &&
       xQueueReceive(receiveEvents, &event, 0) == pdTRUE; ++i) {
    if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL ||
        event.type == UART_FRAME_ERR || event.type == UART_PARITY_ERR ||
        event.type == UART_BREAK) interrupted = true;
  }
  if (interrupted) {
    uart_flush_input(kPort);
    xQueueReset(receiveEvents);
  }
  return interrupted;
}
}

bool LoRaUart::begin() {
  if (ready_) return true;
  uart_config_t configuration{};
  configuration.baud_rate = config::lora::kBaud;
  configuration.data_bits = UART_DATA_8_BITS;
  configuration.parity = UART_PARITY_DISABLE;
  configuration.stop_bits = UART_STOP_BITS_1;
  configuration.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  configuration.source_clk = UART_SCLK_DEFAULT;
  esp_err_t error = uart_param_config(kPort, &configuration);
  if (error == ESP_OK) error = uart_set_pin(kPort,
      board::config::kLoRaTxGpio, board::config::kLoRaRxGpio,
      UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  if (error == ESP_OK) error = uart_driver_install(kPort,
      config::radio::kReceiveBufferBytes, 0, config::radio::kReceiveEventCount,
      &receiveEvents, 0);
  ready_ = error == ESP_OK;
  if (!ready_) ESP_LOGE("lora_uart", "UART1 initialization failed: %s", esp_err_to_name(error));
  return ready_;
}

int LoRaUart::read(uint8_t *buffer, size_t capacity) {
  if (!ready_ || !buffer || !capacity) return 0;
  if (receiveInterrupted()) return -1;
  const int count = uart_read_bytes(kPort, buffer, capacity, 0);
  // 捕捉读取期间到达的错误，禁止把缺字节前后的数据拼成命令。
  return receiveInterrupted() ? -1 : count;
}

int LoRaUart::writeAvailable(const char *data, size_t length) {
  return ready_ && data && length ? uart_tx_chars(kPort, data, length) : 0;
}

UartTxState LoRaUart::txState() const {
  if (!ready_) return UartTxState::Error;
  const esp_err_t state = uart_wait_tx_done(kPort, 0);
  return state == ESP_OK ? UartTxState::Idle :
      state == ESP_ERR_TIMEOUT ? UartTxState::Busy : UartTxState::Error;
}

UartWriteResult LoRaUart::tryWriteFrame(const char *data, size_t length) {
  if (!ready_ || !data || !length || length > UART_HW_FIFO_LEN(kPort)) return {0, true};
  const auto state = txState();
  if (state != UartTxState::Idle) return {0, state == UartTxState::Error};
  const int written = uart_tx_chars(kPort, data, length);
  return {written > 0 ? written : 0, written < 0};
}

void LoRaUart::stop() {
  if (ready_) uart_driver_delete(kPort);
  ready_ = false;
  receiveEvents = nullptr;
}
}
