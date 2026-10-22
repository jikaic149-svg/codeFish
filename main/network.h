#pragma once
#include "ControlState.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

struct MotionSnapshot {
  ControlState control;
  robofish::ActuatorFrame attempted;
  robofish::ActuatorFrame written;
  robofish::motion::Result allocation;
  robofish::wifi::Status wifi;
  uint32_t nowMs = 0;
  uint32_t attempts = 0;
  uint32_t failures = 0;
  uint32_t periodMs = 0;
  uint8_t writtenMask = 0;
};

// Latest-value telemetry, bounded command/result FIFO, and one OTA stop transaction.
// HTTP is the only producer; the application loop is the only control-state owner.
bool networkBegin(QueueHandle_t snapshots, QueueHandle_t commands,
                  QueueHandle_t commandResults, QueueHandle_t stopRequests,
                  QueueHandle_t stopResults);
void networkConfirmStartup(bool ready);
