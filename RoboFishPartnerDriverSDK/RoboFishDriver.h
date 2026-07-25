#pragma once

#include <Arduino.h>
#include <ESP32Servo.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Thin hardware driver only. It deliberately contains no forward/backward,
// steering, gait, CPG, demo, depth-control or autonomous swimming algorithm.
class RoboFishDriver {
 public:
  RoboFishDriver();

  bool begin();
  void update();

  void setTailServo1(int angle);
  void setTailServo2(int angle);
  void setTailServos(int angle1, int angle2);
  void setPitchServo(int angle);
  void setPump(bool enabled);
  void setEscThrottle(int command);
  void setEscSteering(int command);

  void centerTail();
  void stopPropulsion();
  void emergencyStop();

  // Called by the OTA service before flash writing. Once locked, actuator
  // commands are ignored until the device reboots into the new firmware.
  void prepareForOta();

  bool ready() const;
  bool otaLocked() const;

 private:
  bool takeLock(TickType_t timeout = pdMS_TO_TICKS(100));
  void giveLock();
  void writePumpUnlocked(bool enabled);
  void stopPropulsionUnlocked();
  void emergencyStopUnlocked();

  Servo tailServo1_;
  Servo tailServo2_;
  Servo pitchServo_;
  Servo escThrottle_;
  Servo escSteering_;
  SemaphoreHandle_t mutex_;

  volatile bool ready_;
  volatile bool otaLocked_;
  bool pumpEnabled_;
  int throttleCommand_;
  uint32_t lastThrottleCommandMs_;
  uint32_t pumpStartedMs_;
};

