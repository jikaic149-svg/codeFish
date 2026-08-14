#pragma once

#include <atomic>

#include "freertos/FreeRTOS.h"

#include "Propulsion.h"
#include "PumpOutput.h"
#include "SideRudder.h"
#include "TailServos.h"

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
  bool prepareForOta();

  bool ready() const;
  bool otaLocked() const;

 private:
  class CommandLease {
   public:
    explicit CommandLease(RoboFishDriver &driver);
    ~CommandLease();

    bool entered() const;

   private:
    RoboFishDriver &driver_;
    bool entered_;
  };

  bool commandsAccepted() const;
  bool tryEnterCommand();
  void leaveCommand();
  void stopAllOutputs();

  TailServos tail_;
  SideRudder sideRudder_;
  Propulsion propulsion_;
  PumpOutput pump_;

  std::atomic_bool ready_;
  std::atomic_bool otaLocked_;
  std::atomic_uint32_t activeCommands_;
};
