#include "PartnerScriptEngine.h"

#include <algorithm>
#include <cctype>
#include <climits>
#include <cstdio>
#include <cstring>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"

namespace robofish::partner {
namespace {

constexpr char kTag[] = "partner";
constexpr char kNvsNamespace[] = "partner_cfg";
constexpr char kNvsScriptKey[] = "script";
constexpr char kVersion[] = "4.0.0-partner";
constexpr TickType_t kApiLockTimeout = pdMS_TO_TICKS(50);
constexpr TickType_t kTickLockTimeout = pdMS_TO_TICKS(1);

constexpr char kDefaultScript[] = R"JSON({
  "name": "Partner demo",
  "repeat": 1,
  "steps": [
    {"action": "mode_bionic"},
    {"action": "forward", "power": 55, "duration_ms": 3000},
    {"action": "stop", "duration_ms": 800},
    {"action": "left", "power": 45, "strength": 40, "duration_ms": 1200},
    {"action": "pitch", "angle": 110, "duration_ms": 1200},
    {"action": "stop", "duration_ms": 500}
  ]
})JSON";

void setError(char *output, size_t capacity, const char *text) {
  if (!output || capacity == 0) return;
  std::snprintf(output, capacity, "%s", text ? text : "");
}

bool elapsed(uint32_t now, uint32_t started, uint32_t duration) {
  return static_cast<uint32_t>(now - started) >= duration;
}

bool isJsonSpace(char value) {
  return value == ' ' || value == '\t' || value == '\r' || value == '\n';
}

bool isNameCharacter(unsigned char value) {
  // The recovered parser accepts printable ASCII except JSON-sensitive quote
  // and backslash. UTF-8 non-ASCII bytes are accepted and copied verbatim.
  return value >= 0x80 || (value >= 0x20 && value != '"' && value != '\\');
}

class JsonCursor {
 public:
  JsonCursor(const char *input, size_t length)
      : begin_(input), current_(input), end_(input + length) {}

  void skipWhitespace() {
    while (current_ < end_ && isJsonSpace(*current_)) ++current_;
  }

  bool consume(char expected) {
    skipWhitespace();
    if (current_ >= end_ || *current_ != expected) return false;
    ++current_;
    return true;
  }

  bool atEnd() {
    skipWhitespace();
    return current_ == end_;
  }

  size_t offset() const { return static_cast<size_t>(current_ - begin_); }

  bool parseString(char *output, size_t capacity, size_t *length = nullptr) {
    skipWhitespace();
    if (current_ >= end_ || *current_++ != '"') return false;

    size_t count = 0;
    while (current_ < end_) {
      const unsigned char value = static_cast<unsigned char>(*current_++);
      if (value == '"') {
        if (capacity > 0) output[count] = '\0';
        if (length) *length = count;
        return true;
      }
      if (value == '\\' || value < 0x20 || count + 1 >= capacity) return false;
      output[count++] = static_cast<char>(value);
    }
    return false;
  }

  bool parseInteger(int32_t &value) {
    skipWhitespace();
    if (current_ >= end_) return false;

    bool negative = false;
    if (*current_ == '-') {
      negative = true;
      ++current_;
    }
    if (current_ >= end_ || !std::isdigit(static_cast<unsigned char>(*current_))) {
      return false;
    }
    if (*current_ == '0' && current_ + 1 < end_ &&
        std::isdigit(static_cast<unsigned char>(current_[1]))) {
      return false;
    }

    int64_t magnitude = 0;
    while (current_ < end_ &&
           std::isdigit(static_cast<unsigned char>(*current_))) {
      magnitude = magnitude * 10 + (*current_++ - '0');
      if (magnitude > static_cast<int64_t>(INT32_MAX) + 1) return false;
    }
    if (current_ < end_ &&
        (*current_ == '.' || *current_ == 'e' || *current_ == 'E')) {
      return false;
    }

    const int64_t signedValue = negative ? -magnitude : magnitude;
    if (signedValue < INT32_MIN || signedValue > INT32_MAX) return false;
    value = static_cast<int32_t>(signedValue);
    return true;
  }

 private:
  const char *begin_;
  const char *current_;
  const char *end_;
};

bool same(const char *left, const char *right) {
  return std::strcmp(left, right) == 0;
}

bool parseAction(const char *text, Action &action) {
  struct Entry {
    const char *name;
    Action action;
  };
  constexpr Entry kActions[] = {
      {"mode_bionic", Action::kModeBionic}, {"mode_prop", Action::kModeProp},
      {"mode_hybrid", Action::kModeHybrid}, {"forward", Action::kForward},
      {"backward", Action::kBackward},       {"left", Action::kLeft},
      {"right", Action::kRight},             {"stop", Action::kStop},
      {"pitch", Action::kPitch},             {"pump_on", Action::kPumpOn},
      {"pump_off", Action::kPumpOff},
  };
  for (const auto &entry : kActions) {
    if (same(text, entry.name)) {
      action = entry.action;
      return true;
    }
  }
  return false;
}

bool isMotion(Action action) {
  return action == Action::kForward || action == Action::kBackward ||
         action == Action::kLeft || action == Action::kRight;
}

}  // namespace

ScriptEngine::ScriptEngine(const Callbacks &callbacks) : callbacks_(callbacks) {
  lock_ = xSemaphoreCreateMutexStatic(&lockStorage_);
}

Result ScriptEngine::begin(char *error, size_t errorCapacity) {
  if (!takeLock(kApiLockTimeout)) {
    setError(error, errorCapacity, "Partner engine is busy");
    return Result::kCommandRejected;
  }
  stopLocked();
  loaded_ = false;

  bool useDefault = true;
  nvs_handle_t handle = 0;
  const esp_err_t openError = nvs_open(kNvsNamespace, NVS_READONLY, &handle);
  if (openError == ESP_OK) {
    size_t storedSize = 0;
    esp_err_t storageError = nvs_get_str(handle, kNvsScriptKey, nullptr,
                                         &storedSize);
    if (storageError == ESP_OK && storedSize > 1 &&
        storedSize <= sizeof(json_)) {
      storageError = nvs_get_str(handle, kNvsScriptKey, json_, &storedSize);
      if (storageError == ESP_OK) {
        const size_t length = storedSize - 1;
        char validationError[96] = {};
        if (parse(json_, length, stagingScript_, validationError,
                  sizeof(validationError)) == Result::kOk) {
          script_ = stagingScript_;
          jsonLength_ = length;
          loaded_ = true;
          useDefault = false;
        } else {
          ESP_LOGW(kTag, "Stored script invalid (%s); loading default",
                   validationError);
        }
      }
    } else if (storageError == ESP_OK && storedSize > sizeof(json_)) {
      ESP_LOGW(kTag, "Stored script is too large; loading default");
    } else if (storageError != ESP_ERR_NVS_NOT_FOUND) {
      ESP_LOGW(kTag, "Cannot read stored script: %s",
               esp_err_to_name(storageError));
    }
    nvs_close(handle);
  } else if (openError != ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGW(kTag, "Cannot open persistent storage: %s",
             esp_err_to_name(openError));
  }

  if (useDefault) {
    constexpr size_t defaultLength = sizeof(kDefaultScript) - 1;
    const Result result =
        parse(kDefaultScript, defaultLength, stagingScript_, error,
              errorCapacity);
    if (result != Result::kOk) {
      ESP_LOGE(kTag, "Internal default script invalid: %s",
               error ? error : resultText(result));
      giveLock();
      return result;
    }
    std::memcpy(json_, kDefaultScript, defaultLength + 1);
    jsonLength_ = defaultLength;
    script_ = stagingScript_;
    loaded_ = true;
  }

  setError(error, errorCapacity, "");
  ESP_LOGI(kTag, "Loaded '%s' (%u steps, repeat %u)", script_.name,
           script_.stepCount, script_.repeat);
  giveLock();
  return Result::kOk;
}

Result ScriptEngine::validateAndSaveJson(const char *json, size_t length,
                                         char *error,
                                         size_t errorCapacity) {
  if (!takeLock(kApiLockTimeout)) {
    setError(error, errorCapacity, "Partner engine is busy");
    return Result::kCommandRejected;
  }
  const Result validation =
      parse(json, length, stagingScript_, error, errorCapacity);
  if (validation != Result::kOk) {
    giveLock();
    return validation;
  }

  nvs_handle_t handle = 0;
  esp_err_t storageError =
      nvs_open(kNvsNamespace, NVS_READWRITE, &handle);
  if (storageError == ESP_OK) {
    // nvs_set_str() needs NUL-terminated input. Reuse the fixed member buffer
    // instead of placing a 3501-byte temporary on the task stack. The parsed
    // runtime script and loaded_ are committed only after NVS succeeds, so a
    // storage failure leaves the previous executable configuration intact.
    std::memcpy(stagingJson_, json, length);
    stagingJson_[length] = '\0';
    storageError = nvs_set_str(handle, kNvsScriptKey, stagingJson_);
    if (storageError == ESP_OK) storageError = nvs_commit(handle);
    nvs_close(handle);
  }
  if (storageError != ESP_OK) {
    setError(error, errorCapacity, "Cannot save script to persistent storage");
    giveLock();
    return Result::kStorageError;
  }

  stopLocked();
  std::memcpy(json_, stagingJson_, length + 1);
  json_[length] = '\0';
  jsonLength_ = length;
  script_ = stagingScript_;
  loaded_ = true;
  setError(error, errorCapacity, "");
  ESP_LOGI(kTag, "Saved '%s' (%u steps)", script_.name, script_.stepCount);
  giveLock();
  return Result::kOk;
}

Result ScriptEngine::copyJson(char *output, size_t outputCapacity,
                              size_t *written) const {
  if (!takeLock(kApiLockTimeout)) return Result::kCommandRejected;
  if (written) *written = jsonLength_;
  if (!loaded_) {
    giveLock();
    return Result::kNoValidScript;
  }
  if (!output || outputCapacity <= jsonLength_) {
    giveLock();
    return Result::kBufferTooSmall;
  }
  std::memcpy(output, json_, jsonLength_ + 1);
  giveLock();
  return Result::kOk;
}

Result ScriptEngine::start(char *error, size_t errorCapacity) {
  if (!takeLock(kApiLockTimeout)) {
    setError(error, errorCapacity, "Partner engine is busy");
    return Result::kCommandRejected;
  }
  if (!loaded_) {
    setError(error, errorCapacity, "No valid script loaded");
    giveLock();
    return Result::kNoValidScript;
  }
  if (failsafeActive()) {
    setError(error, errorCapacity, "Blocked by failsafe");
    giveLock();
    return Result::kBlockedByFailsafe;
  }

  stopLocked();
  running_ = true;
  stepStarted_ = false;
  stepIndex_ = 0;
  passIndex_ = 0;
  stepStartedMs_ = 0;
  if (!startCurrentStep(
          static_cast<uint32_t>(esp_timer_get_time() / 1000ULL))) {
    stopLocked();
    setError(error, errorCapacity, "Command rejected");
    giveLock();
    return Result::kCommandRejected;
  }
  setError(error, errorCapacity, "");
  ESP_LOGI(kTag, "Started '%s'", script_.name);
  giveLock();
  return Result::kOk;
}

void ScriptEngine::stop() {
  if (!takeLock(kApiLockTimeout)) return;
  stopLocked();
  giveLock();
}

void ScriptEngine::stopLocked() {
  const bool wasRunning = running_;
  running_ = false;
  stepStarted_ = false;
  stepIndex_ = 0;
  passIndex_ = 0;
  stepStartedMs_ = 0;
  emitSafeStop();
  if (wasRunning) ESP_LOGI(kTag, "Script stopped");
}

void ScriptEngine::tick(uint32_t nowMs) {
  if (!takeLock(kTickLockTimeout)) return;
  if (!running_) {
    giveLock();
    return;
  }
  if (failsafeActive()) {
    stopLocked();
    giveLock();
    return;
  }

  if (!stepStarted_) {
    if (!startCurrentStep(nowMs)) stopLocked();
    giveLock();
    return;
  }

  const Step &current = script_.steps[stepIndex_];
  if (!elapsed(nowMs, stepStartedMs_, current.durationMs)) {
    giveLock();
    return;
  }

  emitSafeStop();
  ++stepIndex_;
  if (stepIndex_ >= script_.stepCount) {
    stepIndex_ = 0;
    ++passIndex_;
    if (passIndex_ >= script_.repeat) {
      running_ = false;
      stepStarted_ = false;
      stepStartedMs_ = 0;
      ESP_LOGI(kTag, "Completed '%s'", script_.name);
      giveLock();
      return;
    }
  }

  if (!startCurrentStep(nowMs)) stopLocked();
  giveLock();
}

Status ScriptEngine::status(uint32_t nowMs) const {
  if (!takeLock(kApiLockTimeout)) return {};
  const Status output = statusLocked(nowMs);
  giveLock();
  return output;
}

Status ScriptEngine::statusLocked(uint32_t nowMs) const {
  Status output = {};
  output.loaded = loaded_;
  output.running = running_;
  if (!loaded_) return output;

  std::memcpy(output.name, script_.name, sizeof(output.name));
  output.steps = script_.stepCount;
  output.repeat = script_.repeat;
  if (!running_) return output;

  output.step = static_cast<uint8_t>(stepIndex_ + 1);
  output.pass = static_cast<uint8_t>(passIndex_ + 1);
  if (stepStarted_ && stepIndex_ < script_.stepCount) {
    const uint32_t duration = script_.steps[stepIndex_].durationMs;
    const uint32_t used = static_cast<uint32_t>(nowMs - stepStartedMs_);
    output.remainingMs = used < duration ? duration - used : 0;
  }
  return output;
}

Result ScriptEngine::copyStatusJson(uint32_t nowMs, char *output,
                                    size_t outputCapacity,
                                    size_t *written) const {
  if (!output || outputCapacity == 0) return Result::kBufferTooSmall;
  if (!takeLock(kApiLockTimeout)) return Result::kCommandRejected;
  const Status state = statusLocked(nowMs);
  const int count = std::snprintf(
      output, outputCapacity,
      "{\"version\":\"%s\",\"running\":%s,\"name\":\"%s\","
      "\"step\":%u,\"steps\":%u,\"pass\":%u,\"repeat\":%u,"
      "\"remaining_ms\":%lu}",
      kVersion, state.running ? "true" : "false", state.name, state.step,
      state.steps, state.pass, state.repeat,
      static_cast<unsigned long>(state.remainingMs));
  if (count < 0 || static_cast<size_t>(count) >= outputCapacity) {
    if (written) *written = count > 0 ? static_cast<size_t>(count) : 0;
    giveLock();
    return Result::kBufferTooSmall;
  }
  if (written) *written = static_cast<size_t>(count);
  giveLock();
  return Result::kOk;
}

bool ScriptEngine::running() const {
  if (!takeLock(kApiLockTimeout)) return false;
  const bool value = running_;
  giveLock();
  return value;
}

Result ScriptEngine::parse(const char *json, size_t length,
                           ParsedScript &parsed, char *error,
                           size_t errorCapacity) const {
  if (!json || length < 1 || length > kMaxJsonBytes) {
    setError(error, errorCapacity, "Script size must be 1..3500 bytes");
    return Result::kInvalidArgument;
  }

  JsonCursor cursor(json, length);
  if (!cursor.consume('{')) {
    setError(error, errorCapacity, "Top level must be an object");
    return Result::kInvalidJson;
  }

  bool haveName = false;
  bool haveRepeat = false;
  bool haveSteps = false;
  bool firstTopField = true;
  while (true) {
    cursor.skipWhitespace();
    if (cursor.consume('}')) break;
    if (!firstTopField && !cursor.consume(',')) {
      setError(error, errorCapacity, "Missing comma at top level");
      return Result::kInvalidJson;
    }
    firstTopField = false;

    char key[20] = {};
    if (!cursor.parseString(key, sizeof(key)) || !cursor.consume(':')) {
      setError(error, errorCapacity, "Invalid top-level field");
      return Result::kInvalidJson;
    }

    if (same(key, "name")) {
      size_t nameLength = 0;
      if (haveName || !cursor.parseString(parsed.name, sizeof(parsed.name),
                                           &nameLength) ||
          nameLength == 0) {
        setError(error, errorCapacity, "name is too long or invalid");
        return Result::kInvalidJson;
      }
      for (size_t i = 0; i < nameLength; ++i) {
        if (!isNameCharacter(static_cast<unsigned char>(parsed.name[i]))) {
          setError(error, errorCapacity,
                   "name contains unsupported characters");
          return Result::kInvalidJson;
        }
      }
      haveName = true;
    } else if (same(key, "repeat")) {
      int32_t repeat = 0;
      if (haveRepeat || !cursor.parseInteger(repeat) || repeat < 1 ||
          repeat > 10) {
        setError(error, errorCapacity, "repeat must be 1..10");
        return Result::kInvalidJson;
      }
      parsed.repeat = static_cast<uint8_t>(repeat);
      haveRepeat = true;
    } else if (same(key, "steps")) {
      if (haveSteps || !cursor.consume('[')) {
        setError(error, errorCapacity, "steps must be an array");
        return Result::kInvalidJson;
      }
      haveSteps = true;
      bool firstStep = true;
      while (true) {
        cursor.skipWhitespace();
        if (cursor.consume(']')) break;
        if (!firstStep && !cursor.consume(',')) {
          setError(error, errorCapacity, "Missing comma between steps");
          return Result::kInvalidJson;
        }
        firstStep = false;
        if (parsed.stepCount >= kMaxSteps) {
          setError(error, errorCapacity, "Too many steps (maximum 32)");
          return Result::kInvalidJson;
        }
        if (!cursor.consume('{')) {
          setError(error, errorCapacity, "Each step must be an object");
          return Result::kInvalidJson;
        }

        Step step = {};
        bool haveAction = false;
        bool haveDuration = false;
        bool haveAngle = false;
        bool havePower = false;
        bool haveStrength = false;
        bool firstStepField = true;
        while (true) {
          cursor.skipWhitespace();
          if (cursor.consume('}')) break;
          if (!firstStepField && !cursor.consume(',')) {
            setError(error, errorCapacity, "Missing comma inside step");
            return Result::kInvalidJson;
          }
          firstStepField = false;

          char field[20] = {};
          if (!cursor.parseString(field, sizeof(field)) ||
              !cursor.consume(':')) {
            setError(error, errorCapacity, "Invalid step field");
            return Result::kInvalidJson;
          }
          if (same(field, "action")) {
            char actionText[20] = {};
            if (haveAction ||
                !cursor.parseString(actionText, sizeof(actionText)) ||
                !parseAction(actionText, step.action)) {
              setError(error, errorCapacity, "Unknown or missing action");
              return Result::kInvalidJson;
            }
            haveAction = true;
          } else {
            int32_t value = 0;
            if (!cursor.parseInteger(value)) {
              setError(error, errorCapacity,
                       "Step parameters must be integers");
              return Result::kInvalidJson;
            }
            if (same(field, "duration_ms") && !haveDuration && value >= 0 &&
                value <= 30000) {
              step.durationMs = static_cast<uint32_t>(value);
              haveDuration = true;
            } else if (same(field, "angle") && !haveAngle && value >= 45 &&
                       value <= 135) {
              step.angle = static_cast<int16_t>(value);
              haveAngle = true;
            } else if (same(field, "power") && !havePower && value >= 20 &&
                       value <= 70) {
              step.power = static_cast<uint8_t>(value);
              havePower = true;
            } else if (same(field, "strength") && !haveStrength &&
                       value >= 10 && value <= 70) {
              step.strength = static_cast<uint8_t>(value);
              haveStrength = true;
            } else if (same(field, "duration_ms")) {
              setError(error, errorCapacity,
                       "duration_ms must be 0..30000");
              return Result::kInvalidJson;
            } else if (same(field, "angle")) {
              setError(error, errorCapacity, "pitch angle must be 45..135");
              return Result::kInvalidJson;
            } else if (same(field, "power")) {
              setError(error, errorCapacity, "power must be 20..70");
              return Result::kInvalidJson;
            } else if (same(field, "strength")) {
              setError(error, errorCapacity, "strength must be 10..70");
              return Result::kInvalidJson;
            } else {
              setError(error, errorCapacity, "Unknown step field");
              return Result::kInvalidJson;
            }
          }
        }

        if (!haveAction) {
          setError(error, errorCapacity, "Unknown or missing action");
          return Result::kInvalidJson;
        }
        if (isMotion(step.action) && step.durationMs < 100) {
          setError(error, errorCapacity,
                   "Motion duration_ms must be at least 100");
          return Result::kInvalidJson;
        }
        if (step.action == Action::kPumpOn &&
            (step.durationMs < 100 || step.durationMs > 10000)) {
          setError(error, errorCapacity,
                   "pump_on duration_ms must be 100..10000");
          return Result::kInvalidJson;
        }
        if (step.action == Action::kPitch && step.durationMs < 100) {
          setError(error, errorCapacity,
                   "pitch duration_ms must be at least 100");
          return Result::kInvalidJson;
        }
        parsed.onePassDurationMs += step.durationMs;
        parsed.steps[parsed.stepCount++] = step;
      }
      if (parsed.stepCount == 0) {
        setError(error, errorCapacity, "steps cannot be empty");
        return Result::kInvalidJson;
      }
    } else {
      setError(error, errorCapacity, "Unknown top-level field");
      return Result::kInvalidJson;
    }
  }

  if (!cursor.atEnd()) {
    setError(error, errorCapacity,
             "Unexpected content after JSON object");
    return Result::kInvalidJson;
  }
  if (!haveSteps) {
    setError(error, errorCapacity, "Missing steps array");
    return Result::kInvalidJson;
  }
  if (!haveName) std::snprintf(parsed.name, sizeof(parsed.name), "Partner script");
  if (!haveRepeat) parsed.repeat = 1;

  const uint64_t total =
      static_cast<uint64_t>(parsed.onePassDurationMs) * parsed.repeat;
  if (total < 1 || total > 300000) {
    setError(error, errorCapacity,
             "Total duration must be 1..300000 ms");
    return Result::kInvalidJson;
  }
  setError(error, errorCapacity, "");
  return Result::kOk;
}

bool ScriptEngine::failsafeActive() const {
  return callbacks_.failsafeActive &&
         callbacks_.failsafeActive(callbacks_.context);
}

bool ScriptEngine::emit(const Command &command) const {
  return callbacks_.command && callbacks_.command(callbacks_.context, command);
}

bool ScriptEngine::emitSafeStop() const {
  return emit(Command{CommandType::kSafeStop, 100, 100, 90});
}

bool ScriptEngine::startCurrentStep(uint32_t nowMs) {
  if (!emitSafeStop()) return false;
  const Command command = commandForStep(script_.steps[stepIndex_]);
  if (!emit(command)) return false;
  stepStartedMs_ = nowMs;
  stepStarted_ = true;
  return true;
}

bool ScriptEngine::takeLock(TickType_t timeout) const {
  return lock_ && xSemaphoreTake(lock_, timeout) == pdTRUE;
}

void ScriptEngine::giveLock() const {
  if (lock_) xSemaphoreGive(lock_);
}

Command ScriptEngine::commandForStep(const Step &step) {
  Command output = {};
  output.power = step.power;
  output.strength = step.strength;
  output.angle = step.angle;
  switch (step.action) {
    case Action::kModeBionic:
      output.type = CommandType::kModeBionic;
      break;
    case Action::kModeProp:
      output.type = CommandType::kModeProp;
      break;
    case Action::kModeHybrid:
      output.type = CommandType::kModeHybrid;
      break;
    case Action::kForward:
      output.type = CommandType::kForward;
      break;
    case Action::kBackward:
      output.type = CommandType::kBackward;
      break;
    case Action::kLeft:
      output.type = CommandType::kLeft;
      break;
    case Action::kRight:
      output.type = CommandType::kRight;
      break;
    case Action::kStop:
      output.type = CommandType::kStop;
      break;
    case Action::kPitch:
      output.type = CommandType::kPitch;
      break;
    case Action::kPumpOn:
      output.type = CommandType::kPumpOn;
      break;
    case Action::kPumpOff:
      output.type = CommandType::kPumpOff;
      break;
  }
  return output;
}

const char *resultText(Result result) {
  switch (result) {
    case Result::kOk:
      return "OK";
    case Result::kInvalidArgument:
      return "Invalid argument";
    case Result::kInvalidJson:
      return "Invalid JSON";
    case Result::kStorageError:
      return "Persistent storage error";
    case Result::kNoValidScript:
      return "No valid script loaded";
    case Result::kBlockedByFailsafe:
      return "Blocked by failsafe";
    case Result::kCommandRejected:
      return "Command rejected";
    case Result::kBufferTooSmall:
      return "Buffer too small";
  }
  return "Unknown result";
}

}  // namespace robofish::partner
