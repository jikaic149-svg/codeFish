#include <algorithm>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <deque>
#include <cmath>
#include <limits>
#include <string>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "bsp.h"
#include "ControlState.h"
#include "Service/LoRa/LoRaService.h"

struct HostQueue { std::deque<uart_event_t> events; };
static HostQueue queue;
static std::deque<uint8_t> rx;
static std::string tx;
static bool readError = false, errorDuringRead = false;
static esp_err_t txState = ESP_OK;
static unsigned flushes = 0;
static uint32_t duties[6]{};
static int configuredGpios[6]{};
static int failTimer = 0, failConfigure = -1, failDuty = -1, failUpdate = -1;
static unsigned gpioCalls = 0, stops = 0, delayTicks = 0, dutyCalls = 0;

esp_err_t gpio_reset_pin(gpio_num_t) { ++gpioCalls; return ESP_OK; }
esp_err_t gpio_set_level(gpio_num_t, unsigned) { ++gpioCalls; return ESP_OK; }
esp_err_t gpio_set_direction(gpio_num_t, int) { ++gpioCalls; return ESP_OK; }
void vTaskDelay(unsigned ticks) { delayTicks += ticks; }
int64_t esp_timer_get_time() { return 1234567; }
esp_err_t ledc_timer_config(const ledc_timer_config_t *timer) {
  assert(timer->freq_hz == 50 && timer->duty_resolution == 14 && timer->timer_num == 0);
  return failTimer;
}
esp_err_t ledc_channel_config(const ledc_channel_config_t *channel) {
  if (channel->channel == failConfigure) return -1;
  configuredGpios[channel->channel] = channel->gpio_num;
  duties[channel->channel] = channel->duty;
  return ESP_OK;
}
esp_err_t ledc_set_duty(ledc_mode_t, ledc_channel_t channel, uint32_t duty) {
  ++dutyCalls;
  if (channel == failDuty) return -1;
  duties[channel] = duty; return ESP_OK;
}
esp_err_t ledc_update_duty(ledc_mode_t, ledc_channel_t channel) {
  return channel == failUpdate ? -1 : ESP_OK;
}
esp_err_t ledc_stop(ledc_mode_t, ledc_channel_t, unsigned) { ++stops; return ESP_OK; }

static bool pulseIs(unsigned channel, double microseconds) {
  return std::abs(duties[channel] * 20000.0 / 16384.0 - microseconds) <= 0.62;
}

static void checkBoard() {
  robofish::ActuatorFrame frame;
  assert(board::applyFrameMask(frame) == 0 && !board::setPump(true));
  failTimer = -1; assert(!board::begin()); failTimer = 0;
  failConfigure = 2; assert(!board::begin() && stops == 2); failConfigure = -1;
  assert(board::begin() && delayTicks == 2000 && gpioCalls == 0);
  const int expectedGpios[] = {5,6,4,3,43,8};
  for (unsigned i = 0; i < 6; ++i) assert(configuredGpios[i] == expectedGpios[i] && pulseIs(i,1500));
  assert(board::begin() && delayTicks == 2000 && board::nowMs() == 1234);
  frame.leftPropulsionPercent = frame.rightPropulsionPercent = 100;
  assert(board::applyFrameMask(frame) == 0x3f && pulseIs(4,2000) && pulseIs(5,1000));
  frame.leftPropulsionPercent = frame.rightPropulsionPercent = -100;
  assert(board::applyFrameMask(frame) == 0x3f && pulseIs(4,1000) && pulseIs(5,2000));
  frame.leftPropulsionPercent = std::numeric_limits<int>::max();
  frame.rightPropulsionPercent = std::numeric_limits<int>::min();
  frame.tailFrontDegrees = 0; frame.tailRearDegrees = 180;
  frame.pectoralLeftDegrees = 0; frame.pectoralRightDegrees = 180;
  assert(board::applyFrameMask(frame) == 0x3f && pulseIs(4,2000) && pulseIs(5,2000));
  assert(pulseIs(0,500+122*2000.0/180) && pulseIs(1,500+45*2000.0/180));
  assert(pulseIs(2,1000) && pulseIs(3,500+155*2000.0/180));
  frame.tailFrontDegrees = std::numeric_limits<float>::quiet_NaN();
  const unsigned calls = dutyCalls;
  assert(board::applyFrameMask(frame) == 0 && dutyCalls == calls);
  frame = {};
  failDuty = 4; assert(board::applyFrameMask(frame) == 0x2f); failDuty = -1;
  failUpdate = 5; assert(board::applyFrameMask(frame) == 0x1f); failUpdate = -1;
  frame.leftPropulsionPercent = frame.rightPropulsionPercent = 40;
  assert(board::applyFrameMask(frame) == 0x3f);
  frame.leftPropulsionPercent = frame.rightPropulsionPercent = 0;
  failDuty = 5;
  assert(board::applyFrameMask(frame) == 0x1f && pulseIs(4,1500) && pulseIs(5,1300));
  failDuty = -1;
  assert(board::applyFrameMask(frame) == 0x3f && pulseIs(4,1500) && pulseIs(5,1500));
  assert(board::setPump(false) && !board::setPump(true));
  assert(board::applyFrame(frame,false) && !board::applyFrame(frame,true) && gpioCalls == 0);
}

int xQueueReceive(QueueHandle_t q, void *out, unsigned) {
  if (q->events.empty()) return 0;
  *static_cast<uart_event_t *>(out) = q->events.front();
  q->events.pop_front();
  return 1;
}
int xQueueReset(QueueHandle_t q) { q->events.clear(); return 1; }
const char *esp_err_to_name(esp_err_t) { return "mock"; }
esp_err_t uart_param_config(uart_port_t port, const uart_config_t *config) {
  assert(port == UART_NUM_1 && config->baud_rate == 115200);
  assert(config->data_bits == UART_DATA_8_BITS && config->parity == UART_PARITY_DISABLE);
  return ESP_OK;
}
esp_err_t uart_set_pin(uart_port_t, int txPin, int rxPin, int, int) {
  assert(txPin == 2 && rxPin == 1); return ESP_OK;
}
esp_err_t uart_driver_install(uart_port_t, int, int, int size, QueueHandle_t *out, int) {
  assert(size > 0 && out); *out = &queue; return ESP_OK;
}
esp_err_t uart_driver_delete(uart_port_t) { rx.clear(); queue.events.clear(); return ESP_OK; }
esp_err_t uart_flush_input(uart_port_t) { ++flushes; rx.clear(); return ESP_OK; }
int uart_read_bytes(uart_port_t, void *out, size_t capacity, unsigned) {
  if (readError) { readError = false; return -1; }
  const size_t count = std::min(capacity, rx.size());
  for (size_t i = 0; i < count; ++i) {
    static_cast<uint8_t *>(out)[i] = rx.front(); rx.pop_front();
  }
  if (errorDuringRead) { errorDuringRead = false; queue.events.push_back({UART_FRAME_ERR}); }
  return static_cast<int>(count);
}
int uart_tx_chars(uart_port_t, const char *data, size_t length) {
  tx.assign(data, length); return static_cast<int>(length);
}
esp_err_t uart_wait_tx_done(uart_port_t, unsigned) { return txState; }

struct Sink { unsigned applied = 0; bool accept = true; robofish::lora::Command last{}; };
static bool submit(const robofish::lora::Command &command, void *context) {
  auto &sink = *static_cast<Sink *>(context);
  if (!sink.accept) return false;
  ++sink.applied; sink.last = command; return true;
}
static void receive(const char *line) { while (*line) rx.push_back(static_cast<uint8_t>(*line++)); }

struct ControlSink { ControlState state; uint32_t now = 0; };
static bool submitControl(const robofish::lora::Command &command, void *context) {
  auto &sink = *static_cast<ControlSink *>(context);
  return sink.state.accept(command,sink.now);
}

static void checkRadioHold() {
  using namespace robofish::lora;
  ControlSink sink;
  sink.state.stopPending=false; // The real owner confirms startup neutral before accepting motion.
  LoRaService service(submitControl,&sink);
  assert(service.begin());
  auto send = [&](uint32_t now, unsigned sequence, unsigned code, int knob = -1) {
    char line[64];
    std::snprintf(line,sizeof(line),"100,1,0,%u,%u,%d\n",sequence,code,knob);
    sink.now = now; receive(line); service.poll(now);
  };
  send(100,1,2,1000);
  assert(sink.state.leaseActive && sink.state.target.surgePercent == 100);
  send(3099,1,2,1000);
  assert(sink.state.lastMotionMs == 100 && service.status().accepted == 1);
  assert(!sink.state.expire(3099) && !sink.state.expire(3100));
  send(3101,1,2,1000);
  assert(sink.state.leaseActive && !sink.state.expired && sink.state.target.surgePercent==100);
  send(3102,2,6);
  assert(sink.state.leaseActive && sink.state.target.pitchPercent == 100 &&
      sink.state.target.surgePercent == 100 && sink.state.knob == 1000);
  send(3200,3,1);
  assert(sink.state.lastMotionMs == 3200 && sink.state.target.pitchPercent == 100 &&
      !sink.state.target.allowFlowAssist);
  send(5000,4,16,1000);
  assert(sink.state.lastMotionMs == 3200 && !sink.state.expire(6200));
  send(6500,5,14,1000);
  assert(sink.state.leaseActive && sink.state.target.pitchPercent == 100);
  send(6501,6,3);
  assert(sink.state.leaseActive && sink.state.target.yawPercent == -100 &&
      sink.state.target.pitchPercent == 100 && sink.state.target.surgePercent == 0);
  send(6600,7,17);
  assert(!sink.state.leaseActive && sink.state.stopPending && sink.state.knob == 0);
  send(6601,6,3);
  assert(!sink.state.leaseActive);
  service.stop(); service.poll(6602);

  ControlState wrap;
  wrap.stopPending=false;
  assert(wrap.accept(Command{0,2,false,0},0xfffffff0U));
  assert(!wrap.expire(uint32_t(0xfffffff0U + 2999U)) &&
      !wrap.expire(uint32_t(0xfffffff0U + 30000U)) && wrap.target.surgePercent==100);
}

int main() {
  checkBoard();
  using namespace robofish::lora;
  Sink sink;
  LoRaService service(submit, &sink);
  assert(service.begin() && !service.status().txEnabled);
  uint32_t now = 0;
  auto send = [&](unsigned source, unsigned sequence, unsigned code) {
    char line[64];
    std::snprintf(line, sizeof(line), "100,1,%u,%u,%u,1000\n", source, sequence, code);
    receive(line); service.poll(++now);
  };
  send(0,100,2); send(0,101,17); send(0,99,2);
  assert(sink.applied == 2 && sink.last.code == 17);
  send(0,101,17); assert(sink.applied == 2 && service.status().duplicates == 1);
  send(0,101,2); assert(sink.applied == 2);
  for (unsigned sequence = 102; sequence < 108; ++sequence) send(0,sequence,17);
  unsigned applied = sink.applied;
  send(0,100,2); assert(sink.applied == applied); // 水位不会随近期缓存淘汰。
  send(1,100,2); assert(sink.applied == applied + 1 && sink.last.source == 1);
  send(2,65534,2); send(2,65535,17); send(2,0,2);
  assert(sink.last.source == 2 && sink.last.sequence == 0);
  applied = sink.applied;
  send(2,65535,2); send(2,32768,2); assert(sink.applied == applied);
  send(3,10,17); applied = sink.applied;
  send(4,1,2); assert(sink.applied == applied); // 源表满，不驱逐已有历史。
  sink.accept = false; send(0,108,2); assert(sink.applied == applied);
  sink.accept = true; send(0,108,17); assert(sink.applied == applied + 1);

  // 用实际 LoRaUart 实现验证所有 ESP-IDF 接收错误均打断并丢弃残行。
  unsigned sequence = 109;
  for (auto type : {UART_FIFO_OVF, UART_BUFFER_FULL, UART_FRAME_ERR, UART_PARITY_ERR, UART_BREAK}) {
    char prefix[48]; std::snprintf(prefix, sizeof(prefix), "100,1,0,%u,", sequence);
    receive(prefix); service.poll(++now);
    queue.events.push_back({type}); service.poll(++now);
    applied = sink.applied;
    receive("2,1000\n"); service.poll(++now); assert(sink.applied == applied);
    send(0,sequence++,17); assert(sink.applied == applied + 1);
  }
  assert(flushes == 5);
  receive("100,1,0,114,"); service.poll(++now);
  readError = true; service.poll(++now); applied = sink.applied;
  receive("2,1000\n"); service.poll(++now); assert(sink.applied == applied);
  send(0,114,17); assert(sink.applied == applied + 1);
  receive("100,1,0,115,2,1000\n"); errorDuringRead = true;
  applied = sink.applied; service.poll(++now); assert(sink.applied == applied);
  receive("\n"); service.poll(++now); send(0,115,17);
  assert(sink.applied == applied + 1);
  queue.events.push_back({UART_DATA}); send(0,116,17);
  assert(sink.applied == applied + 2);
  service.stop(); service.poll(++now);

  // ACK 时序仅在主机模拟中明确配置；默认生产发射保持关闭。
  robofish::config::radio::Timing timing{true,true,true,64,100,10,100,20,30};
  Sink ackSink; LoRaService ackService(submit, &ackSink, timing);
  assert(ackService.begin());
  receive("100,1,65535,1,5,1000\n"); ackService.poll(0); ackService.poll(10); ackService.poll(11);
  const std::string expected = "910,65535,1,1,65535,5,1\n";
  assert(tx == expected);
  ackService.poll(12); ackService.poll(42);
  receive("100,1,65535,1,5,1000\n"); ackService.poll(100); ackService.poll(110); ackService.poll(111);
  assert(ackSink.applied == 1 && ackService.status().duplicates == 1 && tx == expected);
  ackService.stop(); ackService.poll(112);

  Command command{42,8,true,100};
  assert(command.source == 0); // 既有四字段聚合初始化保持兼容。
  assert(!parseLine("100,1,65536,1,2", command) && command.sequence == 42);
  assert(!parseLine("100,1,-1,1,2", command));
  assert(!parseLine("100,1,1,65536,2", command));
  assert(parseLine("100,1,65535,65535,19,1000", command) && command.source == 65535);
  assert(sequenceAfter(0,65535) && !sequenceAfter(32768,0));
  checkRadioHold();
  puts("PASS: PWM pins/limits/direction/partial writes, absent pump, source ordering, STOP replay barrier, wrap, source limit, ACK addressing, UART resynchronization, held radio commands");
}
