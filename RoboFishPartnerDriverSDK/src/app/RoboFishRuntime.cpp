#include "RoboFishRuntime.h"

#include <array>

#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "../config/sdk_config.h"
#include "../service/OtaService.h"
#include "../web/WebAssets.h"

namespace {

constexpr char kTag[] = "runtime";

}  // namespace

RoboFishRuntime *RoboFishRuntime::activeRuntime_ = nullptr;

RoboFishRuntime::RoboFishRuntime()
    : driver_(),
      motion_(driver_),
      camera_(),
      rgb_(),
      partner_({partnerCommand, partnerFailsafe, this}),
      web_(motion_, partner_, camera_),
      visionUdp_(motion_) {}

bool RoboFishRuntime::begin() {
  activeRuntime_ = this;

  const bool rgbReady = rgb_.begin();
  if (rgbReady) {
    // Blue is reserved for the runtime startup sequence.
    rgb_.setColor(0, 0, 255);
  } else {
    ESP_LOGE(kTag, "RGB status strip failed to start");
  }

  vTaskDelay(pdMS_TO_TICKS(RF_BOOT_SERIAL_DELAY_MS));
  ESP_LOGI(kTag, "RoboFish FPV Controller v4.0.0 Partner Edition");

  const bool nvsReady = otaServiceInitializeNvs();
  if (!nvsReady) ESP_LOGE(kTag, "NVS initialization failed");

  const esp_err_t cameraError = camera_.begin();
  if (cameraError != ESP_OK) {
    ESP_LOGE(kTag, "Camera initialization failed: %s",
             esp_err_to_name(cameraError));
    // Match the recovered startup contract: motion is not armed when the
    // camera hardware is unavailable. The HTTP service still starts so the
    // failure is observable and recoverable by OTA.
  }

  bool driverReady = driver_.begin();
  if (!driverReady) {
    ESP_LOGE(kTag,
             "Driver initialization failed. Check board selection and wiring.");
  }

  const bool motionReady = driverReady && cameraError == ESP_OK && motion_.begin();
  if (!motionReady) ESP_LOGE(kTag, "Motion controller is not armed");
  if (motionReady && nvsReady) {
    nvs_handle_t driveConfig = 0;
    uint8_t escReversed = 0;
    uint8_t steeringReversed = 0;
    uint8_t tailReversed = 0;
    if (nvs_open("drive_cfg", NVS_READONLY, &driveConfig) == ESP_OK) {
      nvs_get_u8(driveConfig, "esc_rev", &escReversed);
      nvs_get_u8(driveConfig, "steer_rev", &steeringReversed);
      nvs_get_u8(driveConfig, "tail_rev", &tailReversed);
      nvs_close(driveConfig);
    }
    motion_.configureDirection(escReversed != 0, steeringReversed != 0,
                               tailReversed != 0);
  }

  std::array<char, 192> partnerError = {};
  const auto partnerResult =
      nvsReady ? partner_.begin(partnerError.data(), partnerError.size())
               : robofish::partner::Result::kStorageError;
  const bool partnerReady = partnerResult == robofish::partner::Result::kOk;
  if (!partnerReady) {
    ESP_LOGE(kTag, "Partner engine failed: %s",
             partnerError[0] ? partnerError.data()
                             : robofish::partner::resultText(partnerResult));
  }

  OtaServiceConfig otaConfig = {
      RF_AP_SSID,
      RF_AP_PASSWORD,
      RF_STA_SSID,
      RF_STA_PASSWORD,
      RF_OTA_USERNAME,
      RF_OTA_PASSWORD,
      networkLinkState,
      this,
  };
  bool otaReady = otaServiceBegin(otaConfig, stopActiveRuntimeForOta);
  if (!otaReady) {
    ESP_LOGE(kTag, "Web OTA service failed to start");
  }


  httpd_handle_t server = otaServiceHttpServer();
  bool routesReady = server != nullptr;
  if (routesReady) {
    routesReady = roboFishRegisterWebAssetRoutes(server) == ESP_OK;
  }
  if (routesReady) {
    routesReady = camera_.registerControlRoutes(server, true) == ESP_OK;
  }
  if (routesReady) routesReady = web_.registerRoutes(server) == ESP_OK;
  if (!routesReady) ESP_LOGE(kTag, "Recovered product routes failed to start");

  const bool visionUdpReady = motionReady && visionUdp_.begin(RF_VISION_UDP_PORT);
  if (!visionUdpReady) ESP_LOGE(kTag, "Vision UDP service failed to start");

  const bool streamReady = cameraError == ESP_OK &&
                           camera_.startStreamServer(80) == ESP_OK;
  if (!streamReady) ESP_LOGE(kTag, "Camera stream server is unavailable");

  const bool runtimeReady =
      nvsReady && cameraError == ESP_OK && driverReady && motionReady &&
      rgbReady && partnerReady && otaReady && routesReady && streamReady &&
      visionUdpReady;
  rgbStartupFailed_ = !runtimeReady;
  rgbStateKnown_ = false;
  if (rgbStartupFailed_) {
    // Red is reserved for a runtime that did not finish starting.
    rgb_.setColor(255, 0, 0);
  } else {
    updateRgbStatus();
  }

  return runtimeReady;
}

void RoboFishRuntime::update() {
  driver_.update();
  motion_.update();
  partner_.tick(static_cast<uint32_t>(esp_timer_get_time() / 1000ULL));
  updateRgbStatus();
}

RoboFishDriver &RoboFishRuntime::driver() {
  return driver_;
}

bool RoboFishRuntime::stopActiveRuntimeForOta() {
  if (!activeRuntime_) return false;
  activeRuntime_->partner_.stop();
  activeRuntime_->motion_.cancelDemo(false);
  activeRuntime_->motion_.automationSafeStop();
  activeRuntime_->visionUdp_.stop();
  return activeRuntime_->driver_.prepareForOta();
}

bool RoboFishRuntime::partnerCommand(
    void *context, const robofish::partner::Command &command) {
  RoboFishRuntime *runtime = static_cast<RoboFishRuntime *>(context);
  return runtime && runtime->applyPartnerCommand(command);
}

bool RoboFishRuntime::partnerFailsafe(void *context) {
  RoboFishRuntime *runtime = static_cast<RoboFishRuntime *>(context);
  return !runtime || runtime->motion_.failsafeActive();
}

void RoboFishRuntime::networkLinkState(uint8_t apClientCount,
                                       bool staConnected, void *context) {
  RoboFishRuntime *runtime = static_cast<RoboFishRuntime *>(context);
  if (runtime) runtime->motion_.setLinkState(apClientCount, staConnected);
}

bool RoboFishRuntime::applyPartnerCommand(
    const robofish::partner::Command &command) {
  using robofish::partner::CommandType;
  switch (command.type) {
    case CommandType::kSafeStop:
      return motion_.automationSafeStop();
    case CommandType::kModeBionic:
      return motion_.applyAutomationCommand(DriveMode::Bionic);
    case CommandType::kModeProp:
      return motion_.applyAutomationCommand(DriveMode::Prop);
    case CommandType::kModeHybrid:
      return motion_.applyAutomationCommand(DriveMode::Hybrid);
    case CommandType::kForward:
      return motion_.applyAutomationAction(MotionAction::Forward,
                                           command.power, command.strength);
    case CommandType::kBackward:
      return motion_.applyAutomationAction(MotionAction::Backward,
                                           command.power, command.strength);
    case CommandType::kLeft:
      return motion_.applyAutomationAction(MotionAction::Left, command.power,
                                           command.strength);
    case CommandType::kRight:
      return motion_.applyAutomationAction(MotionAction::Right, command.power,
                                           command.strength);
    case CommandType::kStop:
      return motion_.applyAutomationAction(MotionAction::Stopped);
    case CommandType::kPitch:
      return motion_.applyAutomationPitch(command.angle);
    case CommandType::kPumpOn:
      return motion_.applyAutomationPump(true);
    case CommandType::kPumpOff:
      return motion_.applyAutomationPump(false);
  }
  return false;
}

void RoboFishRuntime::updateRgbStatus() {
  if (!rgb_.ready() || rgbStartupFailed_) return;
  MotionStatus status = {};
  if (!motion_.getStatus(status)) return;
  const bool connected = status.apClientCount != 0 || status.staConnected;
  if (rgbStateKnown_ && status.failsafeActive == rgbFailsafe_ &&
      status.mode == rgbMode_ && connected == rgbConnected_) {
    return;
  }
  rgbStateKnown_ = true;
  rgbFailsafe_ = status.failsafeActive;
  rgbMode_ = status.mode;
  rgbConnected_ = connected;
  if (status.failsafeActive) {
    rgb_.setColor(255, 120, 0);
  } else if (status.mode == DriveMode::Idle) {
    if (connected) {
      rgb_.setColor(0, 255, 255);
    } else {
      rgb_.setColor(0, 0, 255);
    }
  } else if (status.mode == DriveMode::Bionic) {
    rgb_.setColor(0, 255, 0);
  } else if (status.mode == DriveMode::Prop) {
    rgb_.setColor(255, 0, 0);
  } else {
    rgb_.setColor(128, 0, 128);
  }
}
