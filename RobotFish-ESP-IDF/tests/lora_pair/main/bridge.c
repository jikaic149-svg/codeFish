#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

// Independent bench firmware. Owns UART1 and USB only; no actuator outputs.
// MD0/MD1 are strapped externally, never driven by this firmware.
#define RADIO_UART UART_NUM_1
#define RADIO_TX 1
#define RADIO_RX 2
#define MAX_TX 96

static char command[2 * MAX_TX + 8];
static uint8_t tx[MAX_TX], rx[128], usb[64];
static char output[2 * sizeof(rx) + 4];
static const char hex[] = "0123456789ABCDEF";

static void reply(const char *text)
{
    usb_serial_jtag_write_bytes(text, strlen(text), pdMS_TO_TICKS(100));
}

static int nibble(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static void execute(void)
{
    if (!strcmp(command, "P")) {
        uint32_t baud = 0;
        uart_get_baudrate(RADIO_UART, &baud);
        snprintf(output, sizeof(output), "READY LORA_BRIDGE TX=1 RX=2 BAUD=%lu RX_LEVEL=%d\n",
                 (unsigned long)baud, gpio_get_level(RADIO_RX));
        reply(output);
        return;
    }
    if (!strcmp(command, "L1") || !strcmp(command, "L0")) {
        bool enabled = command[1] == '1';
        if (uart_set_loop_back(RADIO_UART, enabled) != ESP_OK) {
            reply("ERR LOOPBACK\n");
            return;
        }
        uart_flush_input(RADIO_UART);
        reply(enabled ? "OK L1\n" : "OK L0\n");
        return;
    }
    if (!strcmp(command, "B9600") || !strcmp(command, "B115200")) {
        int baud = !strcmp(command, "B9600") ? 9600 : 115200;
        if (uart_set_baudrate(RADIO_UART, baud) != ESP_OK) {
            reply("ERR BAUD\n");
            return;
        }
        uart_flush_input(RADIO_UART);
        reply(baud == 9600 ? "OK B9600\n" : "OK B115200\n");
        return;
    }
    if (strncmp(command, "T ", 2)) {
        reply("ERR COMMAND\n");
        return;
    }
    size_t length = strlen(command + 2);
    if (!length || length % 2 || length > 2 * MAX_TX) {
        reply("ERR LENGTH\n");
        return;
    }
    for (size_t i = 0; i < length / 2; ++i) {
        int high = nibble(command[2 + i * 2]);
        int low = nibble(command[3 + i * 2]);
        if (high < 0 || low < 0) {
            reply("ERR HEX\n");
            return;
        }
        tx[i] = (uint8_t)((high << 4) | low);
    }
    // Nonblocking FIFO write. Host sends one bounded request at a time.
    int written = uart_tx_chars(RADIO_UART, (const char *)tx, length / 2);
    reply(written >= 0 && (size_t)written == length / 2 ? "OK T\n" : "ERR TX_PARTIAL\n");
}

void app_main(void)
{
    const uart_config_t uart = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(RADIO_UART, 1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(RADIO_UART, &uart));
    ESP_ERROR_CHECK(uart_set_pin(RADIO_UART, RADIO_TX, RADIO_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    usb_serial_jtag_driver_config_t usb_config = {
        .tx_buffer_size = 2048,
        .rx_buffer_size = 1024,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_config));
    reply("READY LORA_BRIDGE TX=1 RX=2\n");
    size_t used = 0;
    bool overflow = false;
    for (;;) {
        int count = usb_serial_jtag_read_bytes(usb, sizeof(usb), pdMS_TO_TICKS(20));
        for (int i = 0; i < count; ++i) {
            if (usb[i] == '\r') continue;
            if (usb[i] == '\n') {
                command[used] = 0;
                if (overflow) reply("ERR LINE\n");
                else if (used) execute();
                used = 0;
                overflow = false;
            } else if (!overflow && used < sizeof(command) - 1) {
                command[used++] = (char)usb[i];
            } else {
                overflow = true;
            }
        }
        count = uart_read_bytes(RADIO_UART, rx, sizeof(rx), 0);
        if (count > 0) {
            output[0] = 'R'; output[1] = ' ';
            for (int i = 0; i < count; ++i) {
                output[2 + i * 2] = hex[rx[i] >> 4];
                output[3 + i * 2] = hex[rx[i] & 15];
            }
            output[2 + count * 2] = '\n';
            output[3 + count * 2] = 0;
            reply(output);
        }
    }
}
