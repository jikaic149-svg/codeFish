#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include "bsp.h"
#include "network.h"

using namespace robofish::wifi;
extern "C" void app_main();
static unsigned checks = 0;
static uint32_t now = 0;
static bool deadlineScenario = false;
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
static Command command(Action action, uint32_t sequence, uint32_t id, uint32_t received) {
  Command value;
  value.action = action; value.sequence = sequence; value.requestId = id; value.receivedMs = received;
  std::strcpy(value.session, "owner-test");
  if (action == Action::Target) value.axes.surge = 100;
  return value;
}
static void post(Action action, uint32_t sequence, uint32_t id, uint32_t received) {
  const auto value = command(action, sequence, id, received);
  check(xQueueSend(commands, &value, 0) == pdTRUE, "fixture command fits bounded FIFO");
}
namespace board {
bool begin() { return true; }
uint32_t nowMs() { return now; }
uint8_t applyFrameMask(const robofish::ActuatorFrame &frame) {
  const bool fail = deadlineScenario ? now >= 60 && now < 240 : now == 60 || now == 120;
  const uint8_t mask = fail ? 31 : 63;
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
  post(Action::Target, 1, 2, 0);
  return true;
}
void networkConfirmStartup(bool ready) { check(ready, "startup no longer requires LoRa hardware"); }
TickType_t xTaskGetTickCount() { return now; }
struct Finished {};
static void deadlineStep(const MotionSnapshot &snapshot) {
  if (now == 230) check(snapshot.control.stopPending && snapshot.wifi.sequence == 4 &&
      commands->entries.size() == 3, "long neutral-output failure retains later requests without executing them");
  if (now == 240) check(!snapshot.control.stopPending && snapshot.wifi.sequence == 4,
      "long neutral-output failure clears only after complete neutral output");
  if (now == 250) check(snapshot.wifi.sequence == 4 && snapshot.wifi.rejected == 3 &&
      std::strcmp(snapshot.wifi.session, "owner-test") == 0 &&
      snapshot.control.target.mode == DriveMode::Prop && !snapshot.control.leaseActive &&
      snapshot.wifi.lastCode == Code::Expired,
      "target, acquire and mode left behind an HTTP timeout never execute after output recovers");
  if (now == 270) check(snapshot.control.leaseActive && snapshot.wifi.sequence == 7 &&
      snapshot.wifi.accepted == 4, "a fresh later target may explicitly resume after old requests expire");
  if (now == 280) {
    for (uint32_t id = 1; id <= 7; ++id) {
      Result result;
      check(xQueueReceive(results, &result, 0) == pdTRUE && result.requestId == id &&
          result.code == (id >= 4 && id <= 6 ? Code::Expired : Code::Accepted),
          "late commands report explicit expiry with the original request ID");
    }
    check(snapshot.wifi.resultDrops == 0, "extended barrier loses no control result");
    for (const auto &write : writes) {
      check(write.at % 20 == 0, "extended barrier retains 20 ms output cadence");
      if (write.at >= 60 && write.at < 260)
        check(write.frame.leftPropulsionPercent == 0 && write.frame.rightPropulsionPercent == 0,
            "timed-out commands cannot create propulsion when neutral output finally recovers");
    }
    throw Finished{};
  }
}
void vTaskDelayUntil(TickType_t *wake, TickType_t period) {
  check(period == 10, "the actual application owner retains its 10 ms period");
  MotionSnapshot snapshot;
  check(xQueuePeek(values, &snapshot, 0) == pdTRUE, "owner publishes copied telemetry each loop");
  if (deadlineScenario) {
    deadlineStep(snapshot);
    *wake += period; now = *wake;
    if (now == 50) {
      post(Action::Stop, 4, 3, now);
      post(Action::Target, 5, 4, now);
      auto acquire = command(Action::Acquire, 0, 5, now);
      std::strcpy(acquire.session, "late-page-uuid");
      check(xQueueSend(commands, &acquire, 0) == pdTRUE, "late acquire fits FIFO");
      auto mode = command(Action::Mode, 6, 6, now); mode.mode = DriveMode::Hybrid;
      check(xQueueSend(commands, &mode, 0) == pdTRUE, "late mode fits FIFO");
    }
    if (now == 260) post(Action::Target, 7, 7, now);
    return;
  }
  if (now == 10) check(snapshot.control.stopPending && snapshot.wifi.sequence == 0,
      "target behind acquire waits for successful neutral output");
  if (now == 30) check(snapshot.wifi.sequence == 1 && snapshot.control.leaseActive,
      "target begins after acquire neutral output");
  if (now == 70) check(snapshot.control.stopPending && snapshot.wifi.sequence == 4 &&
      commands->entries.size() == 2, "failed STOP output holds both older and newer queued targets");
  if (now == 80) check(!snapshot.control.stopPending && snapshot.wifi.sequence == 4,
      "STOP clears only after all six neutral writes succeed");
  if (now == 90) check(snapshot.wifi.sequence == 5 && snapshot.control.leaseActive &&
      snapshot.wifi.rejected == 1, "old target after STOP is rejected while fresh target may resume");
  if (now == 110) check(snapshot.wifi.sequence == 5 && snapshot.wifi.lastCode == Code::Expired,
      "queued target cannot renew an already elapsed receive deadline");
  if (now == 130) check(snapshot.control.maintenance && snapshot.control.stopPending &&
      !snapshot.control.leaseActive && commands->entries.size() == 1 && otaResults->entries.empty(),
      "OTA wins over ordinary target and waits through partial neutral failure");
  if (now == 150) check(snapshot.control.maintenance && !snapshot.control.leaseActive &&
      snapshot.wifi.lastCode == Code::Maintenance && snapshot.wifi.sequence == 5,
      "ordinary FIFO cannot restart motion after OTA lock");
  if (now == 160) {
    bool stopped = false;
    check(xQueueReceive(otaResults, &stopped, 0) == pdTRUE && stopped,
        "OTA acknowledgement is emitted only after a complete neutral output");
    check(snapshot.wifi.resultDrops == 0 && results->entries.size() == 7 &&
        snapshot.wifi.accepted == 4 && snapshot.wifi.rejected == 3,
        "all command results remain visible without silent FIFO loss");
    for (uint32_t id = 1; id <= 7; ++id) {
      Result result;
      check(xQueueReceive(results, &result, 0) == pdTRUE && result.requestId == id,
          "results preserve request IDs across STOP barriers and rejected commands");
    }
    check(writes.size() == 9, "only the original 20 ms writer submits six-channel frames");
    for (size_t i = 0; i < writes.size(); ++i) {
      check(writes[i].at == i * 20, "HTTP queue activity does not add output writes or alter cadence");
      if (writes[i].at == 20 || writes[i].at == 60 || writes[i].at == 80 || writes[i].at >= 120)
        check(writes[i].frame.leftPropulsionPercent == 0 && writes[i].frame.rightPropulsionPercent == 0,
            "acquire, STOP retry and OTA frames request zero propulsion");
    }
    throw Finished{};
  }
  *wake += period; now = *wake;
  if (now == 50) {
    post(Action::Stop, 4, 3, now);
    post(Action::Target, 3, 4, now);
    post(Action::Target, 5, 5, now);
  }
  if (now == 110) post(Action::Target, 6, 6, now - 3000);
  if (now == 120) {
    post(Action::Target, 7, 7, now);
    const bool stop = true;
    check(xQueueSend(otaRequests, &stop, 0) == pdTRUE, "OTA stop request fits its dedicated static queue");
  }
}
int main(int argc, char **) {
  deadlineScenario = argc > 1;
  try { app_main(); } catch (const Finished &) {
    std::printf("PASS: %u actual-owner checks (%s)\n", checks, deadlineScenario ?
        "extended neutral-write failure, HTTP request deadlines, stale takeover/mode rejection, explicit fresh resumption" :
        "10/20ms cadence, STOP/acquire neutral barriers, partial writes, FIFO ordering/deadlines, OTA priority and results");
    return 0;
  }
  check(false, "actual owner must complete fixture schedule");
}
