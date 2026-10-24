#pragma once
#include "ControlState.h"
#include "config/messages/LoRaLinkStatus.h"
#include "config/messages/ImuStatus.h"
#include "APP/Algorithm/AttitudeControl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

struct MotionSnapshot {
  ControlState control;
  robofish::ActuatorFrame attempted;
  robofish::ActuatorFrame written;
  robofish::motion::Result allocation;
  robofish::wifi::Status wifi;
  robofish::lora::LinkStatus lora;
  robofish::imu::Status imu;
  robofish::attitude::Result attitude;
  robofish::imu::Vector3 filteredAngles;
  uint32_t filteredAnglesMs = 0;
  bool filteredAnglesValid = false;
  bool loraReady = false;
  uint32_t nowMs = 0;
  uint32_t attempts = 0;
  uint32_t failures = 0;
  uint32_t periodMs = 0;
  uint8_t writtenMask = 0;
};

// Latest snapshot, bounded Wi-Fi fallback transactions and one OTA stop transaction.
// The application loop owns both UARTs, link arbitration and all actuator updates.
bool networkBegin(QueueHandle_t snapshots, QueueHandle_t commands, QueueHandle_t commandResults,
                  QueueHandle_t stopRequests, QueueHandle_t stopResults);
void networkConfirmStartup(bool ready);
