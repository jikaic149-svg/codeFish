#pragma once

#include "bsp/RgbStatusStrip.h"
#include "camera/CameraService.h"
#include "control/MotionController.h"
#include "device/RoboFishDriver.h"
#include "partner/PartnerScriptEngine.h"
#include "web/ProductWebService.h"
#include "web/VisionUdpService.h"

class RoboFishRuntime {
 public:
  RoboFishRuntime();

  bool begin();
  void update();

  RoboFishDriver &driver();

 private:
  static bool stopActiveRuntimeForOta();
  static bool partnerCommand(void *context,
                             const robofish::partner::Command &command);
  static bool partnerFailsafe(void *context);
  static void networkLinkState(uint8_t apClientCount, bool staConnected,
                               void *context);
  bool applyPartnerCommand(const robofish::partner::Command &command);
  void updateRgbStatus();

  RoboFishDriver driver_;
  MotionController motion_;
  CameraService camera_;
  RgbStatusStrip rgb_;
  robofish::partner::ScriptEngine partner_;
  ProductWebService web_;
  VisionUdpService visionUdp_;
  bool rgbStateKnown_ = false;
  bool rgbStartupFailed_ = false;
  bool rgbConnected_ = false;
  bool rgbFailsafe_ = false;
  DriveMode rgbMode_ = DriveMode::Idle;
  static RoboFishRuntime *activeRuntime_;
};
