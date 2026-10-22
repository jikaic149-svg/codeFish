#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <vector>
#include "bsp.h"
#include "network.h"

using namespace robofish::wifi;
extern "C" void app_main();
static unsigned checks = 0;
static uint32_t now = 0;
static bool deadlineScenario = false;
static bool leaseScenario = false;
static bool emergencyScenario = false;
static bool faultScenario = false;
static robofish::ActuatorFrame beforeExpiry;
static QueueHandle_t values, commands, results, otaRequests, otaResults;
struct Write { uint32_t at; robofish::ActuatorFrame frame; uint8_t mask; };
static std::vector<Write> writes;
static void check(bool ok, const char *label) {
  ++checks;
  if (!ok) { std::fprintf(stderr, "FAIL at %u ms: %s\n", now, label); std::exit(1); }
}
QueueHandle_t xQueueCreateStatic(unsigned capacity, unsigned itemBytes, uint8_t *, StaticQueue_t *storage) {
  storage->capacity = capacity; storage->itemBytes = itemBytes; return storage;
}
int xQueueSend(QueueHandle_t queue, const void *item, unsigned wait) {
  check(wait == 0, "owner queue publication never blocks");
  if (queue->entries.size() == queue->capacity) return 0;
  const auto *bytes = static_cast<const uint8_t *>(item);
  queue->entries.emplace_back(bytes, bytes + queue->itemBytes); return pdTRUE;
}
int xQueueReceive(QueueHandle_t queue, void *out, unsigned wait) {
  check(wait == 0, "owner queue consumption never blocks");
  if (queue->entries.empty()) return 0;
  std::memcpy(out, queue->entries.front().data(), queue->itemBytes);
  queue->entries.pop_front(); return pdTRUE;
}
int xQueueOverwrite(QueueHandle_t queue, const void *item) {
  check(queue->capacity == 1, "overwrite is reserved for latest snapshots or OTA result");
  queue->entries.clear(); return xQueueSend(queue, item, 0);
}
int xQueuePeek(QueueHandle_t queue, void *out, unsigned wait) {
  check(wait == 0, "snapshot peeks never block");
  if (queue->entries.empty()) return 0;
  std::memcpy(out, queue->entries.front().data(), queue->itemBytes); return pdTRUE;
}
static Command command(Action action, uint32_t sequence, uint32_t id, uint32_t received,
                       const char *session = "owner-test") {
  Command value;
  value.action = action; value.sequence = sequence; value.requestId = id; value.receivedMs = received;
  std::strcpy(value.session, session);
  if (action == Action::Target) value.axes.surge = 100;
  return value;
}
static void post(Action action, uint32_t sequence, uint32_t id, uint32_t received,
                 const char *session = "owner-test") {
  const auto value = command(action, sequence, id, received, session);
  check(xQueueSend(commands, &value, 0) == pdTRUE, "fixture command fits bounded FIFO");
}
namespace board {
bool begin() { return true; }
uint32_t nowMs() { return now; }
uint8_t applyFrameMask(const robofish::ActuatorFrame &frame) {
  const bool fail = faultScenario ? now >= 60 && now < 360 :
      deadlineScenario ? now >= 60 && now < 240 :
      !leaseScenario && (now == 640 || now == 2440);
  const uint8_t mask = leaseScenario && now == 3020 ? 62 : fail ? 31 : 63;
  writes.push_back({now, frame, mask}); return mask;
}
}
bool networkBegin(QueueHandle_t snapshots, QueueHandle_t requests, QueueHandle_t replies,
                  QueueHandle_t stops, QueueHandle_t stopped) {
  values = snapshots; commands = requests; results = replies; otaRequests = stops; otaResults = stopped;
  check(commands->capacity == 8 && results->capacity == 8 &&
      commands->itemBytes == sizeof(Command) && results->itemBytes == sizeof(Result),
      "actual application creates bounded static command/result queues");
  post(Action::Acquire, 0, 1, 0);
  auto target = command(Action::Target, 1, 2, 0);
  if (leaseScenario) target.axes = {60, 25, 50};
  check(xQueueSend(commands, &target, 0) == pdTRUE, "initial target fits FIFO");
  return true;
}
void networkConfirmStartup(bool ready) { check(ready, "startup no longer requires LoRa hardware"); }
TickType_t xTaskGetTickCount() { return now; }
struct Finished {};
static void deadlineStep(const MotionSnapshot &snapshot) {
  if (now == 130) check(snapshot.control.stopPending && !snapshot.control.outputRecovery &&
      commands->entries.size() == 3, "neutral barrier holds requests during transient output warnings");
  if (now == 150) check(snapshot.control.outputRecovery && snapshot.wifi.rejected == 3 &&
      commands->entries.empty() && snapshot.wifi.lastCode == Code::OutputRecovery,
      "output recovery promptly rejects queued commands instead of silently replaying them later");
  if (now == 230) check(snapshot.control.stopPending && snapshot.wifi.sequence == 0 &&
      snapshot.wifi.session[0] == 0 && snapshot.control.outputRecovery,
      "persistent output failures retain neutral demand and invalidate the old session");
  if (now == 240) check(!snapshot.control.stopPending && snapshot.wifi.sequence == 0,
      "long neutral-output failure clears only after complete neutral output");
  if (now == 250) check(snapshot.wifi.sequence == 0 && snapshot.wifi.rejected == 3 &&
      snapshot.wifi.session[0] == 0 && !snapshot.control.outputRecovery &&
      snapshot.control.target.mode == DriveMode::Prop && !snapshot.control.leaseActive &&
      snapshot.wifi.lastCode == Code::OutputRecovery,
      "recovery never automatically resumes the discarded session or queued motion");
  if (now == 270) check(!snapshot.control.leaseActive && snapshot.wifi.sequence == 0 &&
      snapshot.wifi.accepted == 4, "fresh authorization alone leaves recovered hardware neutral");
  if (now == 290) check(snapshot.control.leaseActive && snapshot.wifi.sequence == 1 &&
      snapshot.wifi.accepted == 5, "fresh target after authorization explicitly resumes movement");
  if (now == 300) {
    for (uint32_t id = 1; id <= 8; ++id) {
      Result result;
      check(xQueueReceive(results, &result, 0) == pdTRUE && result.requestId == id &&
          result.code == (id >= 4 && id <= 6 ? Code::OutputRecovery : Code::Accepted),
          "recovery rejections preserve original command request IDs");
    }
    check(snapshot.wifi.resultDrops == 0, "extended barrier loses no control result");
    for (const auto &write : writes) {
      check(write.at % 20 == 0, "extended barrier retains 20 ms output cadence");
      if (write.at >= 60 && write.at < 280)
        check(write.frame.leftPropulsionPercent == 0 && write.frame.rightPropulsionPercent == 0,
            "timed-out commands cannot create propulsion when neutral output finally recovers");
    }
    throw Finished{};
  }
}
static void faultStep(const MotionSnapshot &snapshot) {
  if (now == 130) check(snapshot.control.outputFailureStreak == 4 &&
      !snapshot.control.outputRecovery && snapshot.control.leaseActive,
      "four failed submissions remain a warning before the neutral threshold");
  if (now == 140) check(snapshot.control.outputRecovery && !snapshot.control.outputFault &&
      !snapshot.control.leaseActive && snapshot.wifi.session[0] == 0,
      "fifth failed submission discards motion and the active session");
  if (now >= 160) check(snapshot.attempted.leftPropulsionPercent == 0 &&
      snapshot.attempted.rightPropulsionPercent == 0 &&
      snapshot.attempted.tailFrontDegrees == 90 && snapshot.attempted.tailRearDegrees == 90 &&
      snapshot.attempted.pectoralLeftDegrees == 90 && snapshot.attempted.pectoralRightDegrees == 90,
      "every subsequent recovery or fault frame requests all six neutral channels");
  if (now == 330) check(!snapshot.control.outputFault, "fault does not latch before fifteen failures");
  if (now == 340) check(snapshot.control.outputFault &&
      snapshot.control.outputFailureStreak == 15, "fifteenth failed submission latches output fault");
  if (now >= 360) check(snapshot.control.outputFault && !snapshot.control.outputRecovery &&
      snapshot.written.leftPropulsionPercent == 0 && snapshot.written.rightPropulsionPercent == 0 &&
      !snapshot.control.leaseActive, "successful neutral output does not release the latched stop");
  if (now == 400) {
    check(snapshot.wifi.rejected == 2 && snapshot.wifi.lastCode == Code::OutputFault &&
        snapshot.wifi.session[0] == 0 && snapshot.wifi.sequence == 0,
        "new acquire and target remain explicitly rejected after latched fault");
    check(snapshot.wifi.resultDrops == 0, "fault reporting loses no request results");
    throw Finished{};
  }
}
void vTaskDelayUntil(TickType_t *wake, TickType_t period) {
  check(period == 10, "the actual application owner retains its 10 ms period");
  MotionSnapshot snapshot;
  check(xQueuePeek(values, &snapshot, 0) == pdTRUE, "owner publishes copied telemetry each loop");
  if (faultScenario) {
    faultStep(snapshot);
    *wake += period; now = *wake;
    if (now == 380) post(Action::Acquire, 0, 3, now);
    if (now == 390) post(Action::Target, 2, 4, now);
    return;
  }
  if (emergencyScenario) {
    if (now == 600) check(snapshot.written.leftPropulsionPercent == 100 &&
        snapshot.written.rightPropulsionPercent == 100, "emergency fixture reaches full thrust");
    if (now >= 620) check(snapshot.written.leftPropulsionPercent == 0 &&
        snapshot.written.rightPropulsionPercent == 0 && !snapshot.control.leaseActive,
        "explicit STOP is immediate and does not use the ordinary release ramp");
    if (now == 660) throw Finished{};
    *wake += period; now = *wake;
    if (now == 610) post(Action::Stop, 2, 3, now);
    return;
  }
  if (deadlineScenario) {
    deadlineStep(snapshot);
    *wake += period; now = *wake;
    if (now == 50) {
      post(Action::Acquire, 0, 3, now, "other");
      post(Action::Target, 1, 4, now, "other");
      auto acquire = command(Action::Acquire, 0, 5, now);
      std::strcpy(acquire.session, "late-page-uuid");
      check(xQueueSend(commands, &acquire, 0) == pdTRUE, "late acquire fits FIFO");
      auto mode = command(Action::Mode, 2, 6, now, "other"); mode.mode = DriveMode::Hybrid;
      check(xQueueSend(commands, &mode, 0) == pdTRUE, "late mode fits FIFO");
    }
    if (now == 260) post(Action::Acquire, 0, 7, now, "other");
    if (now == 280) post(Action::Target, 1, 8, now, "other");
    return;
  }
  if (leaseScenario) {
    if (now == 2980) {
      beforeExpiry = snapshot.written;
      check(beforeExpiry.tailFrontDegrees != 90 && beforeExpiry.tailRearDegrees != 90 &&
          beforeExpiry.pectoralLeftDegrees != 90 && beforeExpiry.pectoralRightDegrees != 90,
          "lease fixture offsets all four servos before disconnect");
    }
    if (now == 3000) check(snapshot.control.expired && !snapshot.control.leaseActive &&
        snapshot.written.leftPropulsionPercent == 0 && snapshot.written.rightPropulsionPercent == 0,
        "three-second expiry immediately requests neutral thrust");
    if (now >= 3000 && now <= 3500) check(snapshot.control.expired &&
        !snapshot.control.leaseActive && snapshot.written.leftPropulsionPercent == 0 &&
        snapshot.written.rightPropulsionPercent == 0,
        "expired control cannot renew propulsion while servos return to center");
    if (now == 3500) {
      check(snapshot.written.tailFrontDegrees == 90 && snapshot.written.tailRearDegrees == 90 &&
          snapshot.written.pectoralLeftDegrees == 90 && snapshot.written.pectoralRightDegrees == 90,
          "all four disconnected servos finish centered without a new command");
      auto previous = beforeExpiry;
      for (const auto &write : writes) {
        if (write.at < 3000) continue;
        const float angles[] = {write.frame.tailFrontDegrees, write.frame.tailRearDegrees,
            write.frame.pectoralLeftDegrees, write.frame.pectoralRightDegrees};
        float *prior[] = {&previous.tailFrontDegrees, &previous.tailRearDegrees,
            &previous.pectoralLeftDegrees, &previous.pectoralRightDegrees};
        for (unsigned channel = 0; channel < 4; ++channel) {
          check(std::abs(angles[channel] - 90) <= std::abs(*prior[channel] - 90) &&
              (angles[channel] - 90) * (*prior[channel] - 90) >= 0,
              "every servo returns monotonically without crossing center after disconnect");
          check(std::abs(angles[channel] - *prior[channel]) <= (channel < 2 ? 1.001f : 1.801f),
              "disconnect recentering keeps the existing servo slew limit and successful-write history");
          if (write.mask & (1 << channel)) *prior[channel] = angles[channel];
        }
      }
    }
    if (now == 3540) {
      check(snapshot.control.leaseActive && !snapshot.control.expired && snapshot.wifi.sequence == 2,
          "only a fresh explicit target resumes expired movement");
      throw Finished{};
    }
    *wake += period; now = *wake;
    if (now == 3520) post(Action::Target, 2, 3, now);
    return;
  }
  if (now == 10) check(snapshot.control.stopPending && snapshot.wifi.sequence == 0,
      "target behind acquire waits for successful neutral output");
  if (now == 30) check(snapshot.wifi.sequence == 1 && snapshot.control.leaseActive,
      "target begins after acquire neutral output");
  if (now == 200) check(snapshot.written.leftPropulsionPercent == 36 &&
      snapshot.written.rightPropulsionPercent == 36,
      "owner applies soft startup instead of immediately reaching requested full thrust");
  if (now == 600) check(snapshot.written.leftPropulsionPercent == 100 &&
      snapshot.written.rightPropulsionPercent == 100, "fixture reaches full speed before STOP");
  if (now == 620) check(snapshot.control.softStopping && !snapshot.control.stopPending &&
      snapshot.control.leaseActive && snapshot.written.leftPropulsionPercent == 96 &&
      snapshot.written.rightPropulsionPercent == 96, "ordinary release starts the requested gradual deceleration");
  if (now == 640) check(snapshot.written.leftPropulsionPercent == 92 &&
      snapshot.written.rightPropulsionPercent == 96,
      "failed right ESC write cannot advance its deceleration history");
  if (now == 660) check(snapshot.written.leftPropulsionPercent == 88 &&
      snapshot.written.rightPropulsionPercent == 92,
      "each propeller resumes deceleration from its own successful output");
  if (now == 730) check(snapshot.control.leaseActive && !snapshot.control.stopPending &&
      snapshot.wifi.sequence == 3, "fresh reverse target can interrupt ordinary ramp-down without HTTP expiry");
  if (now == 1620) check(snapshot.written.leftPropulsionPercent == -100 &&
      snapshot.written.rightPropulsionPercent == -100, "reverse resumes only after each propeller reaches neutral");
  if (now == 1640) check(snapshot.written.leftPropulsionPercent == -96 &&
      snapshot.written.rightPropulsionPercent == -96, "reverse movement also stops gradually");
  if (now == 2120) check(!snapshot.control.stopPending &&
      snapshot.written.leftPropulsionPercent == 0 && snapshot.written.rightPropulsionPercent == 0,
      "full reverse to neutral finishes after exactly 500 ms");
  if (now == 2360) check(!snapshot.control.stopPending && !snapshot.control.leaseActive &&
      snapshot.written.leftPropulsionPercent == 0 && snapshot.written.rightPropulsionPercent == 0 &&
      commands->entries.size() == 1, "new-session takeover retains an immediate neutral output barrier");
  if (now == 2380) check(snapshot.control.leaseActive && snapshot.wifi.sequence == 1,
      "new session's target starts only after its neutral barrier");
  if (now == 2440) check(snapshot.control.maintenance && snapshot.control.stopPending &&
      !snapshot.control.leaseActive && otaResults->entries.empty(),
      "OTA requires immediate neutral and waits for a failed right ESC write");
  if (now == 2480) {
    bool stopped = false;
    check(xQueueReceive(otaResults, &stopped, 0) == pdTRUE && stopped,
        "OTA acknowledgement is emitted only after a complete neutral output");
    check(snapshot.wifi.resultDrops == 0 && results->entries.size() == 8 &&
        snapshot.wifi.accepted == 8 && snapshot.wifi.rejected == 2,
        "all command results remain visible without silent FIFO loss");
    for (uint32_t id = 3; id <= 10; ++id) {
      Result result;
      check(xQueueReceive(results, &result, 0) == pdTRUE && result.requestId == id,
          "results preserve request IDs across STOP barriers and rejected commands");
    }
    check(writes.size() == 125, "only the original 20 ms writer submits six-channel frames");
    robofish::ActuatorFrame previous;
    for (size_t i = 0; i < writes.size(); ++i) {
      check(writes[i].at == i * 20, "HTTP queue activity does not add output writes or alter cadence");
      const auto &write = writes[i];
      if (write.at >= 40 && write.at <= 600)
        check(std::abs(write.frame.leftPropulsionPercent - previous.leftPropulsionPercent) <= 4 &&
            std::abs(write.frame.rightPropulsionPercent - previous.rightPropulsionPercent) <= 4,
            "ordinary startup obeys the four-percent bound through the actual output owner");
      if ((write.at >= 620 && write.at <= 700) || (write.at >= 1640 && write.at <= 2120))
        check(std::abs(write.frame.leftPropulsionPercent - previous.leftPropulsionPercent) <= 4 &&
            std::abs(write.frame.rightPropulsionPercent - previous.rightPropulsionPercent) <= 4,
            "ordinary stopping obeys the four-percent bound on every successful-channel history");
      if (write.at >= 720 && write.at <= 1620)
        check((previous.leftPropulsionPercent <= 0 || write.frame.leftPropulsionPercent >= 0) &&
            (previous.rightPropulsionPercent <= 0 || write.frame.rightPropulsionPercent >= 0),
            "reversal never crosses a propeller's neutral without submitting it first");
      if (write.at == 20 || write.at == 2360 || write.at >= 2440)
        check(writes[i].frame.leftPropulsionPercent == 0 && writes[i].frame.rightPropulsionPercent == 0,
            "acquire and OTA frames retain immediate neutral propulsion");
      if (write.mask & 16) previous.leftPropulsionPercent = write.frame.leftPropulsionPercent;
      if (write.mask & 32) previous.rightPropulsionPercent = write.frame.rightPropulsionPercent;
    }
    throw Finished{};
  }
  *wake += period; now = *wake;
  if (now == 600) {
    Result result;
    check(xQueueReceive(results, &result, 0) == pdTRUE && result.requestId == 1,
        "client consumes acquire result");
    check(xQueueReceive(results, &result, 0) == pdTRUE && result.requestId == 2,
        "client consumes initial target result");
  }
  if (now == 610) {
    auto released = command(Action::Target, 2, 3, now); released.axes = {};
    check(xQueueSend(commands, &released, 0) == pdTRUE, "ordinary release fits FIFO");
  }
  if (now == 720) {
    auto reverse = command(Action::Target, 3, 4, now); reverse.axes.surge = -100;
    check(xQueueSend(commands, &reverse, 0) == pdTRUE, "fresh reverse target fits FIFO during ramp-down");
  }
  if (now == 1630) {
    auto released = command(Action::Target, 4, 5, now); released.axes = {};
    check(xQueueSend(commands, &released, 0) == pdTRUE, "reverse release fits FIFO");
  }
  if (now == 2160) post(Action::Target, 5, 6, now);
  if (now == 2350) {
    post(Action::Acquire, 0, 7, now, "takeover");
    post(Action::Target, 1, 8, now, "takeover");
  }
  if (now == 2430) {
    post(Action::Target, 2, 9, now, "takeover");
    const bool stop = true;
    check(xQueueSend(otaRequests, &stop, 0) == pdTRUE, "OTA stop request fits its dedicated static queue");
  }
  if (now == 2470) post(Action::Target, 3, 10, now, "takeover");
}
int main(int argc, char **argv) {
  deadlineScenario = argc > 1 && std::strcmp(argv[1], "deadlines") == 0;
  leaseScenario = argc > 1 && std::strcmp(argv[1], "lease") == 0;
  emergencyScenario = argc > 1 && std::strcmp(argv[1], "emergency") == 0;
  faultScenario = argc > 1 && std::strcmp(argv[1], "fault") == 0;
  try { app_main(); } catch (const Finished &) {
    std::printf("PASS: %u actual-owner checks (%s)\n", checks, faultScenario ?
        "output warning, recovery, six-channel neutral retry and latched fault" : emergencyScenario ?
        "explicit emergency STOP bypasses ordinary soft-stop ramp" : leaseScenario ?
        "3s immediate thrust stop, four-servo gradual recentering, partial writes, fresh target resumption" : deadlineScenario ?
        "extended neutral-write failure, queued-command rejection, session invalidation, explicit fresh resumption" :
        "10/20ms cadence, gradual ordinary release/reversal, partial writes, target resumption, acquire/OTA neutral barriers");
    return 0;
  }
  check(false, "actual owner must complete fixture schedule");
}
