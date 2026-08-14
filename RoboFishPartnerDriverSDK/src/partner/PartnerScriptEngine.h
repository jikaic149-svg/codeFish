#pragma once

#include <cstddef>
#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace robofish::partner {

constexpr size_t kMaxJsonBytes = 3500;
constexpr size_t kMaxSteps = 32;
constexpr size_t kMaxNameBytes = 47;

enum class Action : uint8_t {
  kModeBionic = 1,
  kModeProp,
  kModeHybrid,
  kForward,
  kBackward,
  kLeft,
  kRight,
  kStop,
  kPitch,
  kPumpOn,
  kPumpOff,
};

// kSafeStop is emitted before every step, after every timed step and when a
// run is stopped. The adapter must stop propulsion and turn the pump off; it
// must not reset pitch unless the application deliberately chooses to do so.
enum class CommandType : uint8_t {
  kSafeStop,
  kModeBionic,
  kModeProp,
  kModeHybrid,
  kForward,
  kBackward,
  kLeft,
  kRight,
  kStop,
  kPitch,
  kPumpOn,
  kPumpOff,
};

struct Command {
  CommandType type = CommandType::kSafeStop;
  uint8_t power = 55;
  uint8_t strength = 40;
  int16_t angle = 90;
};

using CommandCallback = bool (*)(void *context, const Command &command);
using FailsafeCallback = bool (*)(void *context);

struct Callbacks {
  CommandCallback command = nullptr;
  FailsafeCallback failsafeActive = nullptr;
  void *context = nullptr;
};

enum class Result : uint8_t {
  kOk,
  kInvalidArgument,
  kInvalidJson,
  kStorageError,
  kNoValidScript,
  kBlockedByFailsafe,
  kCommandRejected,
  kBufferTooSmall,
};

struct Step {
  Action action = Action::kStop;
  uint32_t durationMs = 0;
  int16_t angle = 90;
  uint8_t power = 55;
  uint8_t strength = 40;
};

struct Status {
  bool loaded = false;
  bool running = false;
  char name[kMaxNameBytes + 1] = {};
  uint8_t step = 0;   // 1-based while running, otherwise 0.
  uint8_t steps = 0;
  uint8_t pass = 0;   // 1-based while running, otherwise 0.
  uint8_t repeat = 0;
  uint32_t remainingMs = 0;
};

class ScriptEngine {
 public:
  explicit ScriptEngine(const Callbacks &callbacks);

  // Loads the persisted script from NVS namespace "partner_cfg". A missing,
  // oversized or invalid value falls back to the recovered built-in script.
  Result begin(char *error, size_t errorCapacity);

  // Performs strict validation before committing the original JSON text to
  // NVS. Saving a valid script safely stops any active run first.
  Result validateAndSaveJson(const char *json, size_t length, char *error,
                             size_t errorCapacity);

  Result copyJson(char *output, size_t outputCapacity,
                  size_t *written = nullptr) const;
  Result start(char *error, size_t errorCapacity);
  void stop();

  // Non-blocking state-machine update. nowMs must use one monotonic 32-bit
  // millisecond timebase; unsigned subtraction makes comparisons wrap-safe.
  void tick(uint32_t nowMs);

  Status status(uint32_t nowMs) const;
  Result copyStatusJson(uint32_t nowMs, char *output, size_t outputCapacity,
                        size_t *written = nullptr) const;
  bool running() const;

 private:
  struct ParsedScript {
    char name[kMaxNameBytes + 1] = {};
    uint8_t repeat = 0;
    uint8_t stepCount = 0;
    uint32_t onePassDurationMs = 0;
    Step steps[kMaxSteps] = {};
  };

  Result parse(const char *json, size_t length, ParsedScript &parsed,
               char *error, size_t errorCapacity) const;
  bool failsafeActive() const;
  bool emit(const Command &command) const;
  bool emitSafeStop() const;
  bool startCurrentStep(uint32_t nowMs);
  bool takeLock(TickType_t timeout) const;
  void giveLock() const;
  void stopLocked();
  Status statusLocked(uint32_t nowMs) const;
  static Command commandForStep(const Step &step);

  Callbacks callbacks_;
  mutable StaticSemaphore_t lockStorage_;
  mutable SemaphoreHandle_t lock_ = nullptr;
  ParsedScript script_;
  ParsedScript stagingScript_;
  char json_[kMaxJsonBytes + 1] = {};
  char stagingJson_[kMaxJsonBytes + 1] = {};
  size_t jsonLength_ = 0;
  bool loaded_ = false;
  bool running_ = false;
  bool stepStarted_ = false;
  uint8_t stepIndex_ = 0;
  uint8_t passIndex_ = 0;
  uint32_t stepStartedMs_ = 0;
};

const char *resultText(Result result);

}  // namespace robofish::partner
