#include <algorithm>
#include "bsp.h"
#include "network.h"
#include "config/AppConfig.h"
#include "config/BoardConfig.h"
#include "Service/LoRa/LoRaService.h"
#include "BSP/ImuUart.h"
#include "Device/Imu/Jy61p.h"
#include "config/AttitudeConfig.h"
#include "esp_log.h"
#include "freertos/task.h"
#ifdef ROBOTFISH_USB_DIAGNOSTICS
#include "Service/Usb/UsbTelemetry.h"
#endif

namespace {
ControlState control;
robofish::motion::State gait;
MotionSnapshot snapshot;
StaticQueue_t snapshotsStorage, requestsStorage, resultsStorage;
StaticQueue_t commandsStorage, commandResultsStorage;
alignas(MotionSnapshot) uint8_t snapshotsBytes[sizeof(MotionSnapshot)];
alignas(robofish::wifi::Command) uint8_t commandsBytes[sizeof(robofish::wifi::Command) * robofish::wifi::kQueueDepth];
alignas(robofish::wifi::Result) uint8_t commandResultsBytes[sizeof(robofish::wifi::Result) * robofish::wifi::kQueueDepth];
uint8_t requestsBytes[sizeof(bool)], resultsBytes[sizeof(bool)];
QueueHandle_t snapshots, stopRequests, stopResults;
QueueHandle_t commands, commandResults;
bool stopAcknowledged = false;
bool submitLoRa(const robofish::lora::Command &command, void *context) {
  return static_cast<ControlState *>(context)->accept(command, board::nowMs());
}
robofish::lora::LoRaService radio(submitLoRa, &control);
robofish::bsp::ImuUart imuUart;
robofish::imu::Jy61p imuDecoder;
robofish::attitude::HeadingState heading;

void pollImu(uint32_t now) {
  uint8_t bytes[board::config::kImuReadBudgetBytes];
  const int count = imuUart.read(bytes, sizeof(bytes));
  if (count < 0) { ++snapshot.imu.receiveErrors; imuDecoder.reset(); return; }
  snapshot.imu.receivedBytes += static_cast<unsigned>(count);
  for (int i = 0; i < count; ++i) imuDecoder.consume(bytes[i], now, snapshot.imu);
}

void merge(robofish::ActuatorFrame &old, const robofish::ActuatorFrame &next, uint8_t mask) {
  if (mask & 1) old.tailFrontDegrees = next.tailFrontDegrees;
  if (mask & 2) old.tailRearDegrees = next.tailRearDegrees;
  if (mask & 4) old.pectoralLeftDegrees = next.pectoralLeftDegrees;
  if (mask & 8) old.pectoralRightDegrees = next.pectoralRightDegrees;
  if (mask & 16) old.leftPropulsionPercent = next.leftPropulsionPercent;
  if (mask & 32) old.rightPropulsionPercent = next.rightPropulsionPercent;
  if (mask == 63) old.flowAssisted = next.flowAssisted;
}

void tick(uint32_t now, uint32_t elapsed) {
  control.expire(now);
  const auto &imu = snapshot.imu;
  const robofish::attitude::Sample sample{
      {imu.angles.x, imu.angles.y, imu.angles.z},
      {imu.angularVelocity.x, imu.angularVelocity.y, imu.angularVelocity.z},
      imu.anglesMs, imu.angularVelocityMs, imu.uartReady, imu.hasAngles, imu.hasAngularVelocity};
  const auto attitudeSettings = robofish::config::attitude::makeSettings();
  snapshot.attitude = robofish::attitude::evaluate(attitudeSettings,
      sample, now, control.attitudeEnabled, control.attitudeMotionAllowed(), control.target.pitchPercent != 0);
  robofish::attitude::holdHeading(attitudeSettings, snapshot.attitude,
      heading, control.target.yawPercent != 0, control.attitudeGeneration);
  auto frame = snapshot.written;
  // Lease expiry stops thrust immediately; the existing disabled-target
  // allocator still slews all four servos from their last successful angles.
  if (control.expired) frame.leftPropulsionPercent = frame.rightPropulsionPercent = 0;
  if (control.maintenance || control.stopPending || control.outputRecovery || control.outputFault) {
    frame = {};
    gait = {};
    snapshot.allocation = {};
  } else if (!control.leaseActive && !control.expired) {
    frame.leftPropulsionPercent = frame.rightPropulsionPercent = 0;
    frame.flowAssisted = false;
    gait = {};
    snapshot.allocation = {};
  } else {
    auto request = control.target;
    if (snapshot.attitude.pitchActive) request.pitchPercent = std::clamp(
        request.pitchPercent + snapshot.attitude.pitchCorrectionPercent, -100.0f, 100.0f);
    if (snapshot.attitude.rollActive) request.rollPercent = std::clamp(
        request.rollPercent + snapshot.attitude.rollCorrectionPercent, -100.0f, 100.0f);
    if (snapshot.attitude.yawActive) request.yawPercent = std::clamp(
        request.yawPercent + snapshot.attitude.yawCorrectionPercent, -100.0f, 100.0f);
    snapshot.allocation = robofish::motion::allocate(request,
        robofish::config::motion::makeRecipe(control.target.mode),
        robofish::config::motion::makeGeometry(), frame,
        gait, now, std::min(elapsed, appconfig::kControlMs));
    frame = snapshot.allocation.candidateFrame;
  }
  snapshot.attempted = frame;
  snapshot.writtenMask = board::applyFrameMask(frame);
  merge(snapshot.written, frame, snapshot.writtenMask);
  control.observeOutput(snapshot.writtenMask);
  if (control.softStopping && snapshot.writtenMask == 63 &&
      snapshot.written.leftPropulsionPercent == 0 && snapshot.written.rightPropulsionPercent == 0 &&
      snapshot.written.tailFrontDegrees == robofish::config::lora::kTailFrontCenter &&
      snapshot.written.tailRearDegrees == robofish::config::lora::kTailRearCenter &&
      snapshot.written.pectoralLeftDegrees == robofish::config::lora::kPectoralLeftCenter &&
      snapshot.written.pectoralRightDegrees == robofish::config::lora::kPectoralRightCenter) {
    control.softStopping = control.leaseActive = false;
  }
  snapshot.periodMs = elapsed;
  ++snapshot.attempts;
  if (snapshot.writtenMask != 63) ++snapshot.failures;
  else {
    if (frame.leftPropulsionPercent == 0 && frame.rightPropulsionPercent == 0)
      control.stopPending = false;
    if (control.maintenance && !stopAcknowledged) {
      const bool success = true;
      xQueueOverwrite(stopResults, &success);
      stopAcknowledged = true;
    }
  }
}
}

extern "C" void app_main() {
  snapshots = xQueueCreateStatic(1, sizeof(MotionSnapshot), snapshotsBytes, &snapshotsStorage);
  stopRequests = xQueueCreateStatic(1, sizeof(bool), requestsBytes, &requestsStorage);
  stopResults = xQueueCreateStatic(1, sizeof(bool), resultsBytes, &resultsStorage);
  commands = xQueueCreateStatic(robofish::wifi::kQueueDepth, sizeof(robofish::wifi::Command), commandsBytes, &commandsStorage);
  commandResults = xQueueCreateStatic(robofish::wifi::kQueueDepth, sizeof(robofish::wifi::Result), commandResultsBytes, &commandResultsStorage);
  if (!snapshots || !stopRequests || !stopResults || !commands || !commandResults || !board::begin()) {
    ESP_LOGE("robotfish", "Queue/PWM initialization failed");
    return;
  }
  tick(board::nowMs(), appconfig::kControlMs);
  snapshot.loraReady = radio.begin();
  snapshot.imu.uartReady = imuUart.begin();
  snapshot.imu.baud = board::config::kImuBaud;
  snapshot.control = control;
  xQueueOverwrite(snapshots, &snapshot);
  const bool networkReady = networkBegin(snapshots, commands, commandResults, stopRequests, stopResults);
  networkConfirmStartup(networkReady && snapshot.loraReady && snapshot.writtenMask == 63);
#ifdef ROBOTFISH_USB_DIAGNOSTICS
  if (!usbTelemetryBegin(snapshots)) ESP_LOGE("robotfish", "USB telemetry task initialization failed");
#endif
  uint32_t lastControl = board::nowMs();
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    const uint32_t now = board::nowMs();
    bool stop = false;
    if (xQueueReceive(stopRequests, &stop, 0) == pdTRUE && stop) {
      control.stop();
      control.maintenance = true;
      stopAcknowledged = false;
    }
#ifdef ROBOTFISH_USB_DIAGNOSTICS
    const unsigned testBaud = usbTelemetryTestBaud();
    if (testBaud && imuUart.setTestBaud(testBaud)) {
      imuDecoder.reset();
      snapshot.imu.baud = testBaud;
      snapshot.imu.hasAngles = snapshot.imu.hasAcceleration = snapshot.imu.hasAngularVelocity = false;
    }
#endif
    radio.poll(now);
    pollImu(now);
    // Both transports submit to this owner; HTTP never touches actuators.
    for (unsigned processed = 0; processed < robofish::wifi::kQueueDepth &&
         (!control.stopPending || control.outputRecovery || control.outputFault); ++processed) {
      robofish::wifi::Command command;
      if (xQueueReceive(commands, &command, 0) != pdTRUE) break;
      // HTTP can publish after this loop captured `now`. Re-read the clock
      // after dequeue so an otherwise fresh command cannot underflow its age.
      const auto result = control.accept(command, board::nowMs());
      if (xQueueSend(commandResults, &result, 0) != pdTRUE) ++control.wifi.resultDrops;
    }
    const uint32_t controlNow = board::nowMs();
    if (uint32_t(controlNow - lastControl) >= appconfig::kControlMs) {
      tick(controlNow, controlNow - lastControl);
      lastControl = controlNow;
    }
    snapshot.nowMs = controlNow;
    snapshot.control = control;
    snapshot.wifi = control.wifi;
    snapshot.lora = radio.status();
    xQueueOverwrite(snapshots, &snapshot);
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(appconfig::kPollMs));
  }
}
