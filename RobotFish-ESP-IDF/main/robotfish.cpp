#include <algorithm>
#include "bsp.h"
#include "network.h"
#include "config/AppConfig.h"
#include "esp_log.h"
#include "freertos/task.h"

namespace {
ControlState control;
robofish::motion::State gait;
MotionSnapshot snapshot;
StaticQueue_t snapshotsStorage, commandsStorage, commandResultsStorage, requestsStorage, resultsStorage;
alignas(MotionSnapshot) uint8_t snapshotsBytes[sizeof(MotionSnapshot)];
alignas(robofish::wifi::Command) uint8_t commandsBytes[
    sizeof(robofish::wifi::Command) * robofish::wifi::kQueueDepth];
alignas(robofish::wifi::Result) uint8_t commandResultsBytes[
    sizeof(robofish::wifi::Result) * robofish::wifi::kQueueDepth];
uint8_t requestsBytes[sizeof(bool)], resultsBytes[sizeof(bool)];
QueueHandle_t snapshots, commands, commandResults, stopRequests, stopResults;
bool stopAcknowledged = false;

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
  auto frame = snapshot.written;
  if (control.maintenance || control.stopPending) {
    frame = {};
    gait = {};
    snapshot.allocation = {};
  } else if (control.expired || !control.leaseActive) {
    // Same F405 lease exit: no thrust, last successful fin angles held.
    frame.leftPropulsionPercent = frame.rightPropulsionPercent = 0;
    frame.flowAssisted = false;
    gait = {};
    snapshot.allocation = {};
  } else {
    snapshot.allocation = robofish::motion::allocate(control.target,
        robofish::config::motion::makeRecipe(control.target.mode),
        robofish::config::motion::makeGeometry(), snapshot.written,
        gait, now, std::min(elapsed, appconfig::kControlMs));
    frame = snapshot.allocation.candidateFrame;
  }
  snapshot.attempted = frame;
  snapshot.writtenMask = board::applyFrameMask(frame);
  merge(snapshot.written, frame, snapshot.writtenMask);
  snapshot.periodMs = elapsed;
  ++snapshot.attempts;
  if (snapshot.writtenMask != 63) ++snapshot.failures;
  else {
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
  commands = xQueueCreateStatic(robofish::wifi::kQueueDepth,
      sizeof(robofish::wifi::Command), commandsBytes, &commandsStorage);
  commandResults = xQueueCreateStatic(robofish::wifi::kQueueDepth,
      sizeof(robofish::wifi::Result), commandResultsBytes, &commandResultsStorage);
  stopRequests = xQueueCreateStatic(1, sizeof(bool), requestsBytes, &requestsStorage);
  stopResults = xQueueCreateStatic(1, sizeof(bool), resultsBytes, &resultsStorage);
  if (!snapshots || !commands || !commandResults || !stopRequests || !stopResults || !board::begin()) {
    ESP_LOGE("robotfish", "Queue/PWM initialization failed");
    return;
  }
  tick(board::nowMs(), appconfig::kControlMs);
  snapshot.control = control;
  xQueueOverwrite(snapshots, &snapshot);
  const bool networkReady = networkBegin(snapshots, commands, commandResults, stopRequests, stopResults);
  networkConfirmStartup(networkReady && snapshot.writtenMask == 63);
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
    // A STOP/acquire is an output barrier: later targets wait for one complete,
    // successful neutral frame. No HTTP callback can write an actuator directly.
    for (unsigned processed = 0; processed < robofish::wifi::kQueueDepth &&
         !control.stopPending; ++processed) {
      robofish::wifi::Command command;
      if (xQueueReceive(commands, &command, 0) != pdTRUE) break;
      const auto result = control.accept(command, now);
      if (xQueueSend(commandResults, &result, 0) != pdTRUE) ++control.wifi.resultDrops;
    }
    if (uint32_t(now - lastControl) >= appconfig::kControlMs) {
      tick(now, now - lastControl);
      lastControl = now;
    }
    snapshot.nowMs = now;
    snapshot.control = control;
    snapshot.wifi = control.wifi;
    xQueueOverwrite(snapshots, &snapshot);
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(appconfig::kPollMs));
  }
}
