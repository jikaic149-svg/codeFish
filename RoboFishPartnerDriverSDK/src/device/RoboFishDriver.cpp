#include "RoboFishDriver.h"

#include "../config/sdk_config.h"
#include "bsp/ServoPwm.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/task.h"

namespace {
constexpr char kTag[] = "driver";
}

RoboFishDriver::CommandLease::CommandLease(RoboFishDriver &driver)
    : driver_(driver), entered_(driver_.tryEnterCommand()) {}

RoboFishDriver::CommandLease::~CommandLease() {
  if (entered_) driver_.leaveCommand();
}

bool RoboFishDriver::CommandLease::entered() const {
  return entered_;
}

RoboFishDriver::RoboFishDriver()
    : ready_(false), otaLocked_(false), activeCommands_(0) {}

bool RoboFishDriver::begin() {
  if (ready_.load()) return true;

  // LEDC Timer 0 is intentionally left for other ESP32 peripherals.
  bool attached = ServoPwm::configureTimers();
  attached &= tail_.begin();
  attached &= sideRudder_.begin();
  attached &= propulsion_.begin();
  attached &= pump_.begin();

  stopAllOutputs();
  ready_.store(attached);

  ESP_LOGI(kTag, "Tail1 GPIO%d, Tail2 GPIO%d, Pitch GPIO%d",
           RF_TAIL_SERVO_1_PIN, RF_TAIL_SERVO_2_PIN, RF_PITCH_SERVO_PIN);
  ESP_LOGI(kTag, "Pump GPIO%d, ESC throttle GPIO%d, ESC steering GPIO%d",
           RF_PUMP_PIN, RF_ESC_THROTTLE_PIN, RF_ESC_STEERING_PIN);
  ESP_LOGI(kTag, "Initialization %s", attached ? "OK" : "FAILED");
  return attached;
}

bool RoboFishDriver::commandsAccepted() const {
  return ready_.load() && !otaLocked_.load();
}

bool RoboFishDriver::tryEnterCommand() {
  if (!commandsAccepted()) return false;

  activeCommands_.fetch_add(1);
  if (!commandsAccepted()) {
    activeCommands_.fetch_sub(1);
    return false;
  }
  return true;
}

void RoboFishDriver::leaveCommand() {
  activeCommands_.fetch_sub(1);
}

void RoboFishDriver::setTailServo1(int angle) {
  CommandLease command(*this);
  if (!command.entered()) return;
  tail_.setServo1(angle, pdMS_TO_TICKS(100));
}

void RoboFishDriver::setTailServo2(int angle) {
  CommandLease command(*this);
  if (!command.entered()) return;
  tail_.setServo2(angle, pdMS_TO_TICKS(100));
}

void RoboFishDriver::setTailServos(int angle1, int angle2) {
  CommandLease command(*this);
  if (!command.entered()) return;
  tail_.setBoth(angle1, angle2, pdMS_TO_TICKS(100));
}

void RoboFishDriver::setPitchServo(int angle) {
  CommandLease command(*this);
  if (!command.entered()) return;
  sideRudder_.setAngle(angle, pdMS_TO_TICKS(100));
}

void RoboFishDriver::setPump(bool enabled) {
  CommandLease command(*this);
  if (!command.entered()) return;
  pump_.setEnabled(enabled, pdMS_TO_TICKS(100));
}

void RoboFishDriver::setEscThrottle(int command) {
  CommandLease commandLease(*this);
  if (!commandLease.entered()) return;
  propulsion_.setThrottle(command, pdMS_TO_TICKS(100));
}

void RoboFishDriver::setEscSteering(int command) {
  CommandLease commandLease(*this);
  if (!commandLease.entered()) return;
  propulsion_.setSteering(command, pdMS_TO_TICKS(100));
}

void RoboFishDriver::centerTail() {
  CommandLease command(*this);
  if (!command.entered()) return;
  tail_.center(pdMS_TO_TICKS(100));
}

void RoboFishDriver::stopPropulsion() {
  if (!ready_.load()) return;
  propulsion_.stop(pdMS_TO_TICKS(100));
}

void RoboFishDriver::emergencyStop() {
  if (!ready_.load()) return;
  stopAllOutputs();
}

void RoboFishDriver::stopAllOutputs() {
  tail_.center(pdMS_TO_TICKS(100));
  sideRudder_.center(pdMS_TO_TICKS(100));
  propulsion_.stop(pdMS_TO_TICKS(100));
  pump_.stop(pdMS_TO_TICKS(100));
}

bool RoboFishDriver::prepareForOta() {
  otaLocked_.store(true);
  const TickType_t started = xTaskGetTickCount();
  const TickType_t timeout = pdMS_TO_TICKS(200);
  while (activeCommands_.load() != 0) {
    if (xTaskGetTickCount() - started >= timeout) {
      ESP_LOGE(kTag, "Timed out waiting for actuator commands to finish");
      return false;
    }
    vTaskDelay(1);
  }

  if (ready_.load()) emergencyStop();
  ESP_LOGI(kTag, "Actuators stopped and locked for OTA");
  return true;
}

void RoboFishDriver::update() {
  CommandLease command(*this);
  if (!command.entered()) return;
  const uint32_t now =
      static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);

  if (propulsion_.update(now, 0)) {
    ESP_LOGW(kTag, "Propulsion timeout -> neutral");
  }

  if (pump_.update(now, 0)) {
    ESP_LOGW(kTag, "Pump timeout -> off");
  }
}

bool RoboFishDriver::ready() const {
  return ready_.load();
}

bool RoboFishDriver::otaLocked() const {
  return otaLocked_.load();
}
