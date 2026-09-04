#include "VisionUdpService.h"

#include <cerrno>
#include <cstring>
#include <sys/time.h>
#include <unistd.h>

#include "config/sdk_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"

namespace {

constexpr char kTag[] = "vision_udp";
constexpr uint8_t kMagic = 0xA5;
constexpr uint8_t kVersion = 1;
constexpr uint8_t kCommandType = 1;
constexpr size_t kFrameBytes = 18;

uint32_t readLe32(const uint8_t *data) {
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) |
         (static_cast<uint32_t>(data[3]) << 24);
}

uint16_t readLe16(const uint8_t *data) {
  return static_cast<uint16_t>(data[0]) |
         (static_cast<uint16_t>(data[1]) << 8);
}

}  // namespace

VisionUdpService::VisionUdpService(MotionController &motion) : motion_(motion) {}

bool VisionUdpService::begin(uint16_t port) {
  if (running_.load(std::memory_order_acquire)) return true;
  port_ = port;
  socket_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (socket_ < 0) {
    ESP_LOGE(kTag, "socket failed: errno=%d", errno);
    return false;
  }
  int reuse = 1;
  setsockopt(socket_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
  timeval timeout = {};
  timeout.tv_sec = 0;
  timeout.tv_usec = 100000;
  setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

  sockaddr_in address = {};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_ANY);
  address.sin_port = htons(port_);
  if (bind(socket_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
    ESP_LOGE(kTag, "bind UDP port %u failed: errno=%d", port_, errno);
    ::close(socket_);
    socket_ = -1;
    return false;
  }

  stopRequested_.store(false, std::memory_order_release);
  taskHandle_ = xTaskCreateStaticPinnedToCore(
      taskEntry, "VisionUdp", taskStack_.size(), this,
      RF_VISION_UDP_TASK_PRIORITY, taskStack_.data(), &taskStorage_,
      RF_VISION_UDP_TASK_CORE);
  if (!taskHandle_) {
    ESP_LOGE(kTag, "failed to create UDP task");
    ::close(socket_);
    socket_ = -1;
    return false;
  }
  running_.store(true, std::memory_order_release);
  ESP_LOGI(kTag, "vision UDP listening on port %u", port_);
  return true;
}

void VisionUdpService::stop() {
  stopRequested_.store(true, std::memory_order_release);
  const int socket = socket_;
  if (socket >= 0) {
    ::shutdown(socket, SHUT_RDWR);
    ::close(socket);
    socket_ = -1;
  }
  if (taskHandle_ && taskHandle_ != xTaskGetCurrentTaskHandle()) {
    for (int i = 0; i < 20 && running_.load(std::memory_order_acquire); ++i) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
  taskHandle_ = nullptr;
  running_.store(false, std::memory_order_release);
}

void VisionUdpService::taskEntry(void *context) {
  static_cast<VisionUdpService *>(context)->taskLoop();
  vTaskDelete(nullptr);
}

void VisionUdpService::taskLoop() {
  std::array<uint8_t, 64> buffer = {};
  while (!stopRequested_.load(std::memory_order_acquire)) {
    const int count = recvfrom(socket_, buffer.data(), buffer.size(), 0, nullptr, nullptr);
    if (count < 0) {
      if (errno == EWOULDBLOCK || errno == EAGAIN || errno == EINTR) continue;
      if (!stopRequested_.load(std::memory_order_acquire)) {
        ESP_LOGW(kTag, "recvfrom failed: errno=%d", errno);
      }
      break;
    }
    handleFrame(buffer.data(), static_cast<size_t>(count));
  }
  running_.store(false, std::memory_order_release);
}

bool VisionUdpService::handleFrame(const uint8_t *data, size_t length) {
  if (!data || length != kFrameBytes || data[0] != kMagic ||
      data[1] != kVersion || data[2] != kCommandType || data[3] != 12 ||
      data[14] > 1 || data[15] != 0) {
    return false;
  }
  if (readLe16(data + 16) != crc16Ccitt(data, 16)) return false;
  const uint32_t sequence = readLe32(data + 4);
  // Sender timestamp is intentionally carried for diagnostics.  PC and MCU
  // clocks are not synchronized, so freshness is enforced by the MCU receive
  // lease (RF_VISION_COMMAND_TIMEOUT_MS), not by comparing this field.
  const uint8_t power = data[12];
  // Wire polarity is positive=left; MotionController performs the one
  // translation required by the legacy HTTP actuator contract.
  const int8_t steering = static_cast<int8_t>(data[13]);
  const bool brake = data[14] != 0;
  const VisionControlResult result = motion_.applyVisionUdp(
      sequence, power, static_cast<int>(steering), brake);
  return result == VisionControlResult::Accepted;
}

uint16_t VisionUdpService::crc16Ccitt(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t index = 0; index < length; ++index) {
    crc ^= static_cast<uint16_t>(data[index]) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}
