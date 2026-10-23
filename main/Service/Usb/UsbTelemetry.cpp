#include "Service/Usb/UsbTelemetry.h"
#include "network.h"
#include <cstdio>
#include <unistd.h>
#include "freertos/task.h"

namespace {
// ESP-IDF Xtensa 的 StackType_t 为 uint8_t，静态栈大小和 API 参数均以字节计。
StaticTask_t taskStorage;
StackType_t stack[4096];
void observe(void *context) {
  const auto snapshots = static_cast<QueueHandle_t>(context);
  MotionSnapshot s;
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    if (xQueuePeek(snapshots, &s, 0) == pdTRUE) {
      const auto &r = s.lora;
      const auto &i = s.imu;
      const auto &c = s.control.loraResult;
      std::printf("\nUSBSTAT {\"ms\":%lu,\"lora_ready\":%u,\"accepted\":%lu,"
          "\"rejected\":%lu,\"duplicates\":%lu,\"stale\":%lu,\"lora_errors\":%lu,"
          "\"source\":%u,\"sequence\":%u,\"code\":%u,\"result\":%u,\"rf_tx\":%u,"
          "\"imu_ready\":%u,\"imu_baud\":%lu,\"imu_bytes\":%lu,\"imu_frames\":%lu,\"imu_crc_errors\":%lu,"
          "\"imu_rx_errors\":%lu,\"angles_valid\":%u,\"roll\":%.3f,\"pitch\":%.3f,"
          "\"yaw\":%.3f,\"angle_age_ms\":%lu,\"written_mask\":%u,\"left\":%d,\"right\":%d,"
          "\"attitude_enabled\":%u,\"attitude_reason\":\"%s\",\"pitch_correction\":%.3f,\"roll_correction\":%.3f,"
          "\"attitude_body_valid\":%u,\"attitude_age_ms\":%lu,"
          "\"heading_locked\":%u,\"heading_target\":%.3f,\"yaw_correction\":%.3f,"
          "\"stack_free\":%u}\n",
          static_cast<unsigned long>(s.nowMs), s.loraReady,
          static_cast<unsigned long>(r.accepted), static_cast<unsigned long>(r.rejected),
          static_cast<unsigned long>(r.duplicates), static_cast<unsigned long>(r.stale),
          static_cast<unsigned long>(r.receiveErrors), c.command.source, c.command.sequence,
          c.command.code, static_cast<unsigned>(c.code), r.txEnabled,
          i.uartReady, static_cast<unsigned long>(i.baud), static_cast<unsigned long>(i.receivedBytes), static_cast<unsigned long>(i.frames),
          static_cast<unsigned long>(i.checksumErrors), static_cast<unsigned long>(i.receiveErrors),
          i.hasAngles, i.angles.x, i.angles.y, i.angles.z,
          static_cast<unsigned long>(s.nowMs - i.anglesMs), s.writtenMask,
          s.written.leftPropulsionPercent, s.written.rightPropulsionPercent,
          s.control.attitudeEnabled, robofish::attitude::reasonName(s.attitude.reason),
          s.attitude.pitchCorrectionPercent, s.attitude.rollCorrectionPercent,
          s.attitude.bodyValid, static_cast<unsigned long>(s.attitude.hasSample ? uint32_t(s.nowMs-s.attitude.sampleMs) : 0),
          s.attitude.headingLocked, s.attitude.headingTargetDegrees, s.attitude.yawCorrectionPercent,
          static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)));
    }
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(100));
  }
}
}
bool usbTelemetryBegin(QueueHandle_t snapshots) {
  // 独立 CPU1 低优先级观察任务：只读现有快照，stdio 不进入运动控制路径。
  return snapshots && xTaskCreateStaticPinnedToCore(observe, "usb_observe", sizeof(stack),
      snapshots, 1, stack, &taskStorage, 1) != nullptr;
}
unsigned usbTelemetryTestBaud() {
  // USB console 的默认 VFS 为非阻塞读。只接受主机显式发来的单字符诊断请求。
  char command = 0;
  if (read(STDIN_FILENO, &command, 1) != 1) return 0;
  switch (command) {
    case '9': return 9600;
    case '2': return 19200;
    case '3': return 38400;
    case '5': return 57600;
    case '1': return 115200;
    case '4': return 230400;
    default: return 0;
  }
}
