#pragma once
#include <stddef.h>
#include <stdint.h>
#include "freertos/queue.h"

using esp_err_t = int;
using uart_port_t = int;
inline constexpr int ESP_OK = 0, ESP_ERR_TIMEOUT = 1;
inline constexpr int UART_NUM_1 = 1, UART_NUM_2 = 2, UART_DATA_8_BITS = 8;
inline constexpr int UART_PARITY_DISABLE = 0, UART_STOP_BITS_1 = 1;
inline constexpr int UART_HW_FLOWCTRL_DISABLE = 0, UART_SCLK_DEFAULT = 0;
inline constexpr int UART_PIN_NO_CHANGE = -1;
#define UART_HW_FIFO_LEN(port) 128
struct uart_config_t {
  int baud_rate, data_bits, parity, stop_bits, flow_ctrl, source_clk;
};
enum uart_event_type_t {
  UART_DATA, UART_FIFO_OVF, UART_BUFFER_FULL, UART_FRAME_ERR,
  UART_PARITY_ERR, UART_BREAK
};
struct uart_event_t { uart_event_type_t type; };
const char *esp_err_to_name(esp_err_t);
esp_err_t uart_param_config(uart_port_t, const uart_config_t *);
esp_err_t uart_set_pin(uart_port_t, int, int, int, int);
esp_err_t uart_driver_install(uart_port_t, int, int, int, QueueHandle_t *, int);
esp_err_t uart_driver_delete(uart_port_t);
esp_err_t uart_flush_input(uart_port_t);
int uart_read_bytes(uart_port_t, void *, size_t, unsigned);
int uart_tx_chars(uart_port_t, const char *, size_t);
esp_err_t uart_wait_tx_done(uart_port_t, unsigned);
