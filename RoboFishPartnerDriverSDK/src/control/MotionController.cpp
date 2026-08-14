#include "MotionController.h"

#include <algorithm>
#include <cmath>

#include "config/sdk_config.h"
#include "esp_log.h"
#include "esp_timer.h"

namespace {

constexpr char kTag[] = "motion";

constexpr uint32_t kControlPeriodMs = 20;
constexpr uint32_t kLinkCheckPeriodMs = 500;
constexpr uint32_t kFailsafeArmDelayMs = 15000;
constexpr uint32_t kDisconnectTriggerMs = 3000;

constexpr uint32_t kDemoForwardMs = 4000;
constexpr uint32_t kDemoDiveMs = 3500;
constexpr uint32_t kDemoSurfaceMs = 4000;
constexpr uint32_t kDemoStopMs = 1200;

constexpr float kTwoPi = 6.28318530717958647692f;
constexpr float kTail2AmplitudeRatio = 0.9f;
constexpr int kEscDriveBaseline = 88;

uint32_t nowMs() {
  return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
}

uint8_t clampPercent(uint8_t value) {
  return std::min<uint8_t>(value, 100);
}

int roundedPercent(int gain, uint8_t percent) {
  return (gain * static_cast<int>(percent) + 50) / 100;
}

float slew(float current, float target, float step) {
  if (current < target) return std::min(current + step, target);
  if (current > target) return std::max(current - step, target);
  return current;
}

int slew(int current, int target, int step) {
  if (current < target) return std::min(current + step, target);
  if (current > target) return std::max(current - step, target);
  return current;
}

int slewThrottle(int current, int target, bool slowAcceleration) {
  if (!slowAcceleration) {
    return slew(current, target, RF_ESC_THROTTLE_SLEW_DEG_PER_TICK);
  }

  const int currentDelta = current - RF_ESC_THROTTLE_NEUTRAL;
  const int targetDelta = target - RF_ESC_THROTTLE_NEUTRAL;
  if (targetDelta == 0) {
    return slew(current, target, RF_ESC_THROTTLE_SLEW_DEG_PER_TICK);
  }
  if ((currentDelta < 0 && targetDelta > 0) ||
      (currentDelta > 0 && targetDelta < 0)) {
    return slew(current, RF_ESC_THROTTLE_NEUTRAL,
                RF_ESC_THROTTLE_SLEW_DEG_PER_TICK);
  }

  const bool accelerating = std::abs(targetDelta) > std::abs(currentDelta);
  return slew(current, target,
              accelerating ? RF_HYBRID_ESC_ACCEL_SLEW_DEG_PER_TICK
                           : RF_ESC_THROTTLE_SLEW_DEG_PER_TICK);
}

}  // namespace

MotionController::MotionController(RoboFishDriver &driver)
    : driver_(driver),
      stateMutexStorage_{},
      stateMutex_(nullptr),
      controlTaskStorage_{},
      controlTaskStack_{},
      controlTaskHandle_(nullptr),
      visionTaskStorage_{},
      visionTaskStack_{},
      visionTaskHandle_(nullptr),
      started_(false),
      state_{DriveMode::Idle,
             MotionAction::Stopped,
             DemoState::Idle,
             100,
             100,
             RF_PITCH_CENTER,
             true,
             false,
             true,
             0,
             false,
             0,
             false,
             false,
             false,
             false,
             0,
             0,
             0,
             0,
             false,
             0,
             0,
             false,
             false,
             0.0f,
             0.0f,
             0.0f,
             0.0f,
             0.0f,
             0.0f,
             0,
             static_cast<float>(RF_TAIL_CENTER),
             static_cast<float>(RF_TAIL_CENTER),
             static_cast<float>(RF_TAIL_CENTER),
             static_cast<float>(RF_TAIL_CENTER),
             RF_ESC_THROTTLE_NEUTRAL,
             RF_ESC_STEERING_CENTER,
             RF_ESC_THROTTLE_NEUTRAL,
             RF_ESC_STEERING_CENTER} {}

bool MotionController::begin() {
  if (started_.load()) return true;
  if (!driver_.ready() || driver_.otaLocked()) return false;

  stateMutex_ = xSemaphoreCreateMutexStatic(&stateMutexStorage_);
  if (!stateMutex_) {
    ESP_LOGE(kTag, "Failed to create state mutex");
    return false;
  }

  const uint32_t startedAt = nowMs();
  if (!takeState(kCommandLockTimeout)) return false;
  state_.bootTimeMs = startedAt;
  state_.lastLinkCheckMs = startedAt;
  giveState();

  controlTaskHandle_ = xTaskCreateStaticPinnedToCore(
      controlTaskEntry, "MotionControl",
      sizeof(controlTaskStack_) / sizeof(controlTaskStack_[0]), this, 2,
      controlTaskStack_, &controlTaskStorage_, 1);
  if (!controlTaskHandle_) {
    ESP_LOGE(kTag, "Failed to create static control task");
    return false;
  }

  visionTaskHandle_ = xTaskCreateStaticPinnedToCore(
      visionTaskEntry, "VisionControl", kVisionTaskStackBytes / sizeof(StackType_t),
      this, 2, visionTaskStack_, &visionTaskStorage_, 1);
  if (!visionTaskHandle_) {
    ESP_LOGE(kTag, "Failed to create static vision control task");
    return false;
  }

  started_.store(true);
  ESP_LOGI(kTag,
           "Motion control started: Core 1, priority 2, period %lu ms, "
           "static stack %u bytes",
           static_cast<unsigned long>(kControlPeriodMs),
           static_cast<unsigned>(sizeof(controlTaskStack_)));
  ESP_LOGI(kTag,
           "Failsafe armed after %lu ms; triggers after %lu ms without AP/STA "
           "link",
           static_cast<unsigned long>(kFailsafeArmDelayMs),
           static_cast<unsigned long>(kDisconnectTriggerMs));
  ESP_LOGI(kTag,
           "Vision control task running: Core 1, priority 2, period %lu ms, "
           "timeout %lu ms, static stack %u bytes",
           static_cast<unsigned long>(kControlPeriodMs),
           static_cast<unsigned long>(RF_VISION_COMMAND_TIMEOUT_MS),
           static_cast<unsigned>(sizeof(visionTaskStack_)));
  return true;
}

bool MotionController::takeState(TickType_t timeout) const {
  return stateMutex_ && xSemaphoreTake(stateMutex_, timeout) == pdTRUE;
}

void MotionController::giveState() const {
  xSemaphoreGive(stateMutex_);
}

bool MotionController::commandAllowed() const {
  return started_.load() && driver_.ready() && !driver_.otaLocked();
}

bool MotionController::onOperatorCommandLocked() {
  const bool leftFailsafe = state_.failsafeActive;
  state_.failsafeActive = false;
  state_.visionControlActive = false;
  state_.visionPidEnabled = false;
  state_.visionPidPending = false;
  if (state_.demoState != DemoState::Idle) cancelDemoLocked(false);
  return leftFailsafe;
}

bool MotionController::automationAllowedLocked() const {
  return !state_.failsafeActive && state_.demoState == DemoState::Idle;
}

void MotionController::stopMotionLocked() {
  state_.action = MotionAction::Stopped;
  state_.tail1Target = static_cast<float>(RF_TAIL_CENTER);
  state_.tail2Target = static_cast<float>(RF_TAIL_CENTER);
  state_.escThrottleTarget = RF_ESC_THROTTLE_NEUTRAL;
  state_.escSteeringTarget = RF_ESC_STEERING_CENTER;
}

bool MotionController::setMode(DriveMode mode) {
  if (!commandAllowed()) return false;
  if (mode != DriveMode::Bionic && mode != DriveMode::Prop &&
      mode != DriveMode::Hybrid) {
    return false;
  }
  if (!takeState(kCommandLockTimeout)) return false;
  const bool leftFailsafe = onOperatorCommandLocked();
  state_.mode = mode;
  stopMotionLocked();
  giveState();

  if (leftFailsafe) ESP_LOGI(kTag, "Operator command cleared failsafe");
  ESP_LOGI(kTag, "Drive mode: %s", modeName(mode));
  return true;
}

bool MotionController::cycleMode(DriveMode *newMode) {
  if (!commandAllowed() || !takeState(kCommandLockTimeout)) return false;
  const bool leftFailsafe = onOperatorCommandLocked();
  switch (state_.mode) {
    case DriveMode::Idle:
      state_.mode = DriveMode::Bionic;
      break;
    case DriveMode::Bionic:
      state_.mode = DriveMode::Prop;
      break;
    case DriveMode::Prop:
      state_.mode = DriveMode::Hybrid;
      break;
    default:
      state_.mode = DriveMode::Bionic;
      break;
  }
  stopMotionLocked();
  const DriveMode selected = state_.mode;
  if (newMode) *newMode = selected;
  giveState();

  if (leftFailsafe) ESP_LOGI(kTag, "Operator command cleared failsafe");
  ESP_LOGI(kTag, "Drive mode: %s", modeName(selected));
  return true;
}

bool MotionController::setAction(MotionAction action, uint8_t powerPercent,
                                 uint8_t strengthPercent) {
  if (!commandAllowed() || !takeState(kCommandLockTimeout)) return false;
  if (state_.mode == DriveMode::Idle) {
    giveState();
    ESP_LOGW(kTag, "Motion command rejected until a drive mode is selected");
    return false;
  }
  const bool leftFailsafe = onOperatorCommandLocked();
  state_.action = action;
  state_.powerPercent = clampPercent(powerPercent);
  state_.turnStrengthPercent = clampPercent(strengthPercent);
  giveState();
  if (leftFailsafe) ESP_LOGI(kTag, "Operator command cleared failsafe");
  return true;
}

bool MotionController::forward(uint8_t powerPercent) {
  return setAction(MotionAction::Forward, powerPercent, 0);
}

bool MotionController::backward(uint8_t powerPercent) {
  return setAction(MotionAction::Backward, powerPercent, 0);
}

bool MotionController::left(uint8_t powerPercent, uint8_t strengthPercent) {
  return setAction(MotionAction::Left, powerPercent, strengthPercent);
}

bool MotionController::right(uint8_t powerPercent, uint8_t strengthPercent) {
  return setAction(MotionAction::Right, powerPercent, strengthPercent);
}

bool MotionController::stop() {
  if (!commandAllowed() || !takeState(kCommandLockTimeout)) return false;
  const bool leftFailsafe = onOperatorCommandLocked();
  stopMotionLocked();
  state_.pumpEnabled = false;
  state_.pumpStartedMs = 0;
  state_.pumpDirty = true;
  giveState();
  if (leftFailsafe) ESP_LOGI(kTag, "Operator command cleared failsafe");
  return true;
}

bool MotionController::setPitch(int angle) {
  if (!commandAllowed() || !takeState(kCommandLockTimeout)) return false;
  const bool leftFailsafe = onOperatorCommandLocked();
  state_.pitchAngle =
      std::clamp(angle, RF_PITCH_MIN_ANGLE, RF_PITCH_MAX_ANGLE);
  state_.pitchDirty = true;
  giveState();
  if (leftFailsafe) ESP_LOGI(kTag, "Operator command cleared failsafe");
  return true;
}

bool MotionController::setPump(bool enabled) {
  if (!commandAllowed() || !takeState(kCommandLockTimeout)) return false;
  const bool leftFailsafe = onOperatorCommandLocked();
  state_.pumpEnabled = enabled;
  state_.pumpStartedMs = enabled ? nowMs() : 0;
  state_.pumpDirty = true;
  giveState();
  if (leftFailsafe) ESP_LOGI(kTag, "Operator command cleared failsafe");
  return true;
}

bool MotionController::configureDirection(bool escReversed,
                                          bool steeringReversed,
                                          bool tailReversed) {
  if (!commandAllowed() || !takeState(kCommandLockTimeout)) return false;
  state_.escReversed = escReversed;
  state_.steeringReversed = steeringReversed;
  state_.tailReversed = tailReversed;
  state_.visionControlActive = false;
  state_.visionPidEnabled = false;
  state_.visionPidPending = false;
  stopMotionLocked();
  giveState();
  ESP_LOGI(kTag, "Drive direction config: esc=%u steering=%u tail=%u",
           escReversed, steeringReversed, tailReversed);
  return true;
}

bool MotionController::setTailReversed(bool reversed) {
  if (!commandAllowed() || !takeState(kCommandLockTimeout)) return false;
  const bool leftFailsafe = onOperatorCommandLocked();
  state_.tailReversed = reversed;
  stopMotionLocked();
  giveState();
  if (leftFailsafe) ESP_LOGI(kTag, "Operator command cleared failsafe");
  ESP_LOGI(kTag, "Tail steering direction: %s",
           reversed ? "REVERSED" : "NORMAL");
  return true;
}

bool MotionController::tailReversed() const {
  if (!started_.load() || !takeState(kCommandLockTimeout)) return false;
  const bool reversed = state_.tailReversed;
  giveState();
  return reversed;
}

bool MotionController::applyAutomationCommand(DriveMode mode) {
  if (!commandAllowed() ||
      (mode != DriveMode::Bionic && mode != DriveMode::Prop &&
       mode != DriveMode::Hybrid) ||
      !takeState(kCommandLockTimeout)) {
    return false;
  }
  if (!automationAllowedLocked()) {
    giveState();
    return false;
  }
  state_.visionControlActive = false;
  state_.visionPidEnabled = false;
  state_.visionPidPending = false;
  state_.mode = mode;
  stopMotionLocked();
  giveState();
  return true;
}

bool MotionController::applyAutomationAction(MotionAction action,
                                             uint8_t powerPercent,
                                             uint8_t strengthPercent) {
  if (!commandAllowed() || !takeState(kCommandLockTimeout)) return false;
  if (!automationAllowedLocked()) {
    giveState();
    return false;
  }
  if (state_.mode == DriveMode::Idle && action != MotionAction::Stopped) {
    giveState();
    return false;
  }
  state_.visionControlActive = false;
  state_.visionPidEnabled = false;
  state_.visionPidPending = false;
  state_.action = action;
  state_.powerPercent = clampPercent(powerPercent);
  state_.turnStrengthPercent = clampPercent(strengthPercent);
  if (action == MotionAction::Stopped) stopMotionLocked();
  giveState();
  return true;
}

bool MotionController::applyAutomationPitch(int angle) {
  if (!commandAllowed() || !takeState(kCommandLockTimeout)) return false;
  if (!automationAllowedLocked()) {
    giveState();
    return false;
  }
  state_.visionControlActive = false;
  state_.visionPidEnabled = false;
  state_.visionPidPending = false;
  state_.pitchAngle =
      std::clamp(angle, RF_PITCH_MIN_ANGLE, RF_PITCH_MAX_ANGLE);
  state_.pitchDirty = true;
  giveState();
  return true;
}

bool MotionController::applyAutomationPump(bool enabled) {
  if (!commandAllowed() || !takeState(kCommandLockTimeout)) return false;
  if (!automationAllowedLocked()) {
    giveState();
    return false;
  }
  state_.visionControlActive = false;
  state_.visionPidEnabled = false;
  state_.visionPidPending = false;
  state_.pumpEnabled = enabled;
  state_.pumpStartedMs = enabled ? nowMs() : 0;
  state_.pumpDirty = true;
  giveState();
  return true;
}

bool MotionController::automationSafeStop() {
  if (!commandAllowed() || !takeState(kCommandLockTimeout)) return false;
  state_.visionControlActive = false;
  state_.visionPidEnabled = false;
  state_.visionPidPending = false;
  stopMotionLocked();
  state_.pumpEnabled = false;
  state_.pumpStartedMs = 0;
  state_.pumpDirty = true;
  giveState();
  return true;
}

VisionControlResult MotionController::applyVisionControl(
    uint32_t sequence, uint8_t powerPercent, int steeringPercent) {
  if (!commandAllowed() || powerPercent > 70 || steeringPercent < -70 ||
      steeringPercent > 70 || !takeState(kCommandLockTimeout)) {
    return VisionControlResult::Rejected;
  }

  if (state_.visionControlActive) {
    const uint32_t delta = sequence - state_.visionSequence;
    if (delta == 0 || delta >= 0x80000000u) {
      giveState();
      return VisionControlResult::StaleSequence;
    }
  }

  onOperatorCommandLocked();
  state_.visionControlActive = true;
  state_.visionPidEnabled = false;
  state_.visionSequence = sequence;
  state_.lastVisionCommandMs = nowMs();
  state_.mode = DriveMode::Prop;
  state_.powerPercent = powerPercent;
  state_.turnStrengthPercent =
      static_cast<uint8_t>(std::abs(steeringPercent));

  if (powerPercent == 0) {
    stopMotionLocked();
  } else if (steeringPercent < 0) {
    state_.action = MotionAction::Left;
  } else if (steeringPercent > 0) {
    state_.action = MotionAction::Right;
  } else {
    state_.action = MotionAction::Forward;
  }
  giveState();
  return VisionControlResult::Accepted;
}

VisionControlResult MotionController::applyVisionPid(uint32_t sequence,
                                                     float xErrorM,
                                                     float yErrorM) {
  if (!commandAllowed() || !std::isfinite(xErrorM) ||
      !std::isfinite(yErrorM) || std::abs(xErrorM) > 5.0f || yErrorM < 0.0f ||
      yErrorM > 5.0f || !takeState(kCommandLockTimeout)) {
    return VisionControlResult::Rejected;
  }

  if (state_.visionControlActive) {
    const uint32_t delta = sequence - state_.visionSequence;
    if (delta == 0 || delta >= 0x80000000u) {
      giveState();
      return VisionControlResult::StaleSequence;
    }
  }

  const bool startingPid = !state_.visionPidEnabled;
  onOperatorCommandLocked();
  state_.visionControlActive = true;
  state_.visionPidEnabled = true;
  state_.visionPidPending = true;
  state_.visionSequence = sequence;
  state_.lastVisionCommandMs = nowMs();
  state_.mode = DriveMode::Prop;
  state_.visionXErrorM = xErrorM;
  state_.visionYErrorM = yErrorM;
  if (startingPid) {
    state_.visionXIntegral = 0.0f;
    state_.visionYIntegral = 0.0f;
    state_.visionXPreviousErrorM = xErrorM;
    state_.visionYPreviousErrorM = yErrorM;
    state_.visionPidPreviousMs = state_.lastVisionCommandMs;
  }
  giveState();
  return VisionControlResult::Accepted;
}

void MotionController::cancelDemoLocked(bool returnToSafePose) {
  state_.demoState = DemoState::Idle;
  if (returnToSafePose) {
    stopMotionLocked();
    state_.pitchAngle = RF_PITCH_UI_NEUTRAL;
    state_.pitchDirty = true;
  }
}

DemoRequestResult MotionController::requestDemo() {
  if (!commandAllowed() || !takeState(kCommandLockTimeout)) {
    return DemoRequestResult::Unavailable;
  }
  if (state_.demoState != DemoState::Idle) {
    cancelDemoLocked(true);
    giveState();
    ESP_LOGI(kTag, "Demo cancelled");
    return DemoRequestResult::Cancelled;
  }
  if (state_.failsafeActive) {
    giveState();
    ESP_LOGW(kTag, "Demo blocked by failsafe");
    return DemoRequestResult::BlockedFailsafe;
  }

  state_.visionControlActive = false;
  state_.visionPidEnabled = false;
  state_.visionPidPending = false;

  if (state_.mode != DriveMode::Bionic) {
    state_.mode = DriveMode::Bionic;
    stopMotionLocked();
    ESP_LOGI(kTag, "Demo selected BIONIC mode");
  }
  state_.demoState = DemoState::Forward;
  state_.demoStateStartedMs = nowMs();
  state_.pitchAngle = RF_PITCH_UI_NEUTRAL;
  state_.pitchDirty = true;
  state_.action = MotionAction::Forward;
  giveState();
  ESP_LOGI(kTag, "Demo started: Forward -> Dive -> Surface -> Stop");
  return DemoRequestResult::Started;
}

bool MotionController::cancelDemo(bool returnToSafePose) {
  if (!commandAllowed() || !takeState(kCommandLockTimeout)) return false;
  const bool wasRunning = state_.demoState != DemoState::Idle;
  cancelDemoLocked(returnToSafePose);
  giveState();
  if (wasRunning) ESP_LOGI(kTag, "Demo cancelled");
  return wasRunning;
}

void MotionController::setLinkState(uint8_t apClientCount,
                                    bool staConnected) {
  if (!started_.load() || !takeState(kCommandLockTimeout)) return;
  state_.apClientCount = apClientCount;
  state_.staConnected = staConnected;
  giveState();
}

bool MotionController::triggerFailsafeLocked() {
  if (state_.failsafeActive) return false;
  state_.failsafeActive = true;
  state_.visionControlActive = false;
  state_.visionPidEnabled = false;
  state_.visionPidPending = false;
  cancelDemoLocked(false);
  stopMotionLocked();
  state_.pitchAngle = RF_PITCH_MIN_ANGLE;
  state_.pitchDirty = true;
  state_.pumpEnabled = false;
  state_.pumpStartedMs = 0;
  state_.pumpDirty = true;
  return true;
}

bool MotionController::triggerFailsafe(const char *reason) {
  if (!started_.load() || !takeState(kCommandLockTimeout)) return false;
  const bool triggered = triggerFailsafeLocked();
  giveState();
  if (triggered) {
    ESP_LOGE(kTag, "Failsafe triggered (%s): stop and pitch to 45 degrees",
             reason ? reason : "external request");
  }
  return triggered;
}

void MotionController::update() {
  if (!started_.load() || !takeState(kCommandLockTimeout)) return;
  const uint32_t now = nowMs();
  bool failsafeTriggered = false;
  DemoState enteredDemoState = DemoState::Idle;
  bool demoFinished = false;

  if (state_.pumpEnabled && state_.pumpStartedMs != 0 &&
      now - state_.pumpStartedMs >= RF_PUMP_MAX_ON_MS) {
    state_.pumpEnabled = false;
    state_.pumpStartedMs = 0;
    state_.pumpDirty = true;
  }

  if (now - state_.lastLinkCheckMs >= kLinkCheckPeriodMs) {
    state_.lastLinkCheckMs = now;
    if (now - state_.bootTimeMs >= kFailsafeArmDelayMs) {
      const bool linkPresent = state_.apClientCount != 0 || state_.staConnected;
      if (linkPresent) {
        state_.disconnectedSinceMs = 0;
      } else if (state_.disconnectedSinceMs == 0) {
        state_.disconnectedSinceMs = now;
      } else if (!state_.failsafeActive &&
                 now - state_.disconnectedSinceMs > kDisconnectTriggerMs) {
        failsafeTriggered = triggerFailsafeLocked();
      }
    }
  }

  if (state_.failsafeActive) {
    state_.pitchAngle = RF_PITCH_MIN_ANGLE;
    state_.pitchDirty = true;
    if (state_.demoState != DemoState::Idle) cancelDemoLocked(false);
  } else if (state_.demoState != DemoState::Idle) {
    const uint32_t elapsed = now - state_.demoStateStartedMs;
    switch (state_.demoState) {
      case DemoState::Forward:
        if (elapsed >= kDemoForwardMs) {
          state_.demoState = DemoState::Dive;
          state_.demoStateStartedMs = now;
          state_.pitchAngle = RF_PITCH_MAX_ANGLE;
          state_.pitchDirty = true;
          state_.action = MotionAction::Forward;
          enteredDemoState = DemoState::Dive;
        }
        break;
      case DemoState::Dive:
        if (elapsed >= kDemoDiveMs) {
          state_.demoState = DemoState::Surface;
          state_.demoStateStartedMs = now;
          stopMotionLocked();
          state_.pitchAngle = RF_PITCH_MIN_ANGLE;
          state_.pitchDirty = true;
          enteredDemoState = DemoState::Surface;
        }
        break;
      case DemoState::Surface:
        if (elapsed >= kDemoSurfaceMs) {
          state_.demoState = DemoState::Stop;
          state_.demoStateStartedMs = now;
          stopMotionLocked();
          state_.pitchAngle = RF_PITCH_UI_NEUTRAL;
          state_.pitchDirty = true;
          enteredDemoState = DemoState::Stop;
        }
        break;
      case DemoState::Stop:
        if (elapsed >= kDemoStopMs) {
          state_.demoState = DemoState::Idle;
          demoFinished = true;
        }
        break;
      default:
        state_.demoState = DemoState::Idle;
        break;
    }
  }
  giveState();

  if (failsafeTriggered) {
    ESP_LOGE(kTag,
             "Failsafe triggered (AP and STA disconnected): stop and pitch to "
             "45 degrees");
  }
  if (enteredDemoState == DemoState::Dive) ESP_LOGI(kTag, "Demo -> DIVE");
  if (enteredDemoState == DemoState::Surface)
    ESP_LOGI(kTag, "Demo -> SURFACE");
  if (enteredDemoState == DemoState::Stop) ESP_LOGI(kTag, "Demo -> STOP");
  if (demoFinished) ESP_LOGI(kTag, "Demo finished");
}

void MotionController::controlTaskEntry(void *context) {
  static_cast<MotionController *>(context)->controlTask();
}

void MotionController::controlTask() {
  ESP_LOGI(kTag, "20 ms control task running");
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    if (!driver_.otaLocked()) controlTick(nowMs());
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(kControlPeriodMs));
  }
}

void MotionController::visionTaskEntry(void *context) {
  static_cast<MotionController *>(context)->visionTask();
}

void MotionController::visionTask() {
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    if (!driver_.otaLocked()) visionTick(nowMs());
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(kControlPeriodMs));
  }
}

void MotionController::visionTick(uint32_t now) {
  if (!takeState(kTickLockTimeout)) return;
  bool timedOut = false;
  if (state_.visionControlActive && state_.visionPidEnabled &&
      state_.visionPidPending) {
    const float dt = std::clamp(
        (now - state_.visionPidPreviousMs) * 0.001f, 0.02f, 0.20f);
    state_.visionPidPreviousMs = now;
    state_.visionPidPending = false;

    const float xError = state_.visionXErrorM;
    const float yError = state_.visionYErrorM;
    state_.visionXIntegral = std::clamp(
        state_.visionXIntegral + xError * dt,
        -RF_VISION_PID_INTEGRAL_LIMIT, RF_VISION_PID_INTEGRAL_LIMIT);
    state_.visionYIntegral = std::clamp(
        state_.visionYIntegral + yError * dt, 0.0f,
        RF_VISION_PID_INTEGRAL_LIMIT);

    const float xDerivative =
        (xError - state_.visionXPreviousErrorM) / dt;
    const float yDerivative =
        (yError - state_.visionYPreviousErrorM) / dt;
    state_.visionXPreviousErrorM = xError;
    state_.visionYPreviousErrorM = yError;

    float steeringOutput =
        RF_VISION_X_KP * xError +
        RF_VISION_X_KI * state_.visionXIntegral +
        RF_VISION_X_KD * xDerivative;
    float powerOutput =
        RF_VISION_Y_KP * yError +
        RF_VISION_Y_KI * state_.visionYIntegral +
        RF_VISION_Y_KD * yDerivative;
    if (std::abs(xError) <= RF_VISION_PID_DEADBAND_M) steeringOutput = 0.0f;
    if (yError <= RF_VISION_PID_DEADBAND_M) powerOutput = 0.0f;

    const int steeringPercent = static_cast<int>(std::lround(
        std::clamp(steeringOutput, -70.0f, 70.0f)));
    const int powerPercent = static_cast<int>(std::lround(
        std::clamp(powerOutput, 0.0f, 70.0f)));
    state_.powerPercent = static_cast<uint8_t>(powerPercent);
    state_.turnStrengthPercent =
        static_cast<uint8_t>(std::abs(steeringPercent));
    if (powerPercent == 0) {
      stopMotionLocked();
    } else if (steeringPercent > 0) {
      // Preserve the polarity already verified by the existing PC tracker:
      // positive cross-track error commands a left turn.
      state_.action = MotionAction::Left;
    } else if (steeringPercent < 0) {
      state_.action = MotionAction::Right;
    } else {
      state_.action = MotionAction::Forward;
    }
  }
  if (state_.visionControlActive &&
      now - state_.lastVisionCommandMs >= RF_VISION_COMMAND_TIMEOUT_MS) {
    state_.visionControlActive = false;
    state_.visionPidEnabled = false;
    state_.visionPidPending = false;
    state_.visionXIntegral = 0.0f;
    state_.visionYIntegral = 0.0f;
    stopMotionLocked();
    timedOut = true;
  }
  giveState();
  if (timedOut) ESP_LOGW(kTag, "Vision control task timeout: propulsion stopped");
}

void MotionController::controlTick(uint32_t now) {
  if (!takeState(kTickLockTimeout)) return;

  const bool tailMode = state_.mode == DriveMode::Bionic ||
                        state_.mode == DriveMode::Hybrid;
  const bool propMode = state_.mode == DriveMode::Prop ||
                        state_.mode == DriveMode::Hybrid;
  const bool moving = state_.action != MotionAction::Stopped;
  const bool reverse = state_.action == MotionAction::Backward;
  const bool turning = state_.action == MotionAction::Left ||
                       state_.action == MotionAction::Right;
  float turnSign = state_.action == MotionAction::Left ? -1.0f : 1.0f;
  if (state_.tailReversed) turnSign = -turnSign;

  state_.tail1Target = static_cast<float>(RF_TAIL_CENTER);
  state_.tail2Target = static_cast<float>(RF_TAIL_CENTER);
  state_.escThrottleTarget = RF_ESC_THROTTLE_NEUTRAL;
  state_.escSteeringTarget = RF_ESC_STEERING_CENTER;

  if (moving && tailMode) {
    const bool hybrid = state_.mode == DriveMode::Hybrid;
    const float amplitudeScale = hybrid ? 58.0f : 50.0f;
    const float phaseRate =
        hybrid ? RF_HYBRID_TAIL_FREQUENCY_HZ
               : RF_BIONIC_TAIL_FREQUENCY_HZ;
    const float phaseLagCycles =
        hybrid ? RF_HYBRID_TAIL_PHASE_LAG_CYCLES
               : RF_BIONIC_TAIL_PHASE_LAG_CYCLES;
    const float amplitude =
        amplitudeScale * static_cast<float>(state_.powerPercent) / 100.0f;
    const float direction = reverse ? -1.0f : 1.0f;
    const float phase = std::fmod(static_cast<float>(now) * 0.001f *
                                      phaseRate * kTwoPi,
                                  kTwoPi);
    float bias = 0.0f;
    if (turning) {
      const float turnScale = hybrid ? RF_HYBRID_TAIL_TURN_BIAS_DEG
                                     : RF_BIONIC_TAIL_TURN_BIAS_DEG;
      bias = turnSign * turnScale *
             static_cast<float>(state_.turnStrengthPercent) / 100.0f;
    }

    // High-confidence approximation: the binary proves sin/fmod, the listed
    // phase/amplitude constants, a 0.9 secondary mix and turn asymmetry, but
    // symbol stripping prevents recovery of the exact original expression.
    // Direction polarity must still be verified on an unloaded mechanism.
    state_.tail1Target = std::clamp(
        static_cast<float>(RF_TAIL_CENTER) +
            direction * amplitude * std::sin(phase) + bias,
        0.0f, 180.0f);
    state_.tail2Target = std::clamp(
        static_cast<float>(RF_TAIL_CENTER) +
            direction * amplitude * kTail2AmplitudeRatio *
                std::sin(phase + phaseLagCycles * kTwoPi) +
            bias,
        0.0f, 180.0f);
  }

  if (moving && propMode) {
    const bool hybrid = state_.mode == DriveMode::Hybrid;
    bool escBackward = reverse;
    if (state_.escReversed) escBackward = !escBackward;
    if (escBackward) {
      state_.escThrottleTarget =
          kEscDriveBaseline - roundedPercent(hybrid ? 42 : 34,
                                             state_.powerPercent);
    } else {
      state_.escThrottleTarget =
          kEscDriveBaseline + roundedPercent(hybrid ? 44 : 36,
                                             state_.powerPercent);
    }
    if (turning) {
      const int steeringOffset =
          roundedPercent(hybrid ? RF_HYBRID_PROP_TURN_MAX_OFFSET_DEG
                                : RF_PROP_TURN_MAX_OFFSET_DEG,
                         state_.turnStrengthPercent);
      bool steerLeft = state_.action == MotionAction::Left;
      if (state_.steeringReversed) steerLeft = !steerLeft;
      state_.escSteeringTarget = RF_ESC_STEERING_CENTER +
                                 (steerLeft ? -steeringOffset
                                            : steeringOffset);
    }
  }

  const float tailStep =
      state_.mode == DriveMode::Hybrid
          ? RF_HYBRID_TAIL_SLEW_DEG_PER_TICK
          : RF_BIONIC_TAIL_SLEW_DEG_PER_TICK;
  state_.tail1Output =
      slew(state_.tail1Output, state_.tail1Target, tailStep);
  state_.tail2Output =
      slew(state_.tail2Output, state_.tail2Target, tailStep);
  state_.escThrottleOutput = slewThrottle(
      state_.escThrottleOutput, state_.escThrottleTarget,
      state_.mode == DriveMode::Hybrid && moving);
  state_.escSteeringOutput =
      slew(state_.escSteeringOutput, state_.escSteeringTarget,
           RF_ESC_STEERING_SLEW_DEG_PER_TICK);

  const int tail1 = static_cast<int>(std::lround(state_.tail1Output));
  const int tail2 = static_cast<int>(std::lround(state_.tail2Output));
  const int throttle = state_.escThrottleOutput;
  const int steering = state_.escSteeringOutput;
  const bool writePitch = state_.pitchDirty || state_.failsafeActive;
  const int pitch = state_.pitchAngle;
  const bool writePump = state_.pumpDirty;
  const bool pump = state_.pumpEnabled;
  state_.pitchDirty = false;
  state_.pumpDirty = false;
  giveState();

  driver_.setTailServos(tail1, tail2);
  driver_.setEscThrottle(throttle);
  driver_.setEscSteering(steering);
  if (writePitch) driver_.setPitchServo(pitch);
  if (writePump) driver_.setPump(pump);
}

bool MotionController::getStatus(MotionStatus &status) const {
  if (!started_.load() || !takeState(kCommandLockTimeout)) return false;
  status = {state_.mode,
            state_.action,
            state_.demoState,
            state_.powerPercent,
            state_.turnStrengthPercent,
            state_.pitchAngle,
            state_.pumpEnabled,
            state_.failsafeActive,
            state_.apClientCount,
            state_.staConnected,
            static_cast<int>(std::lround(state_.tail1Output)),
            static_cast<int>(std::lround(state_.tail2Output)),
            state_.escThrottleOutput,
            state_.escSteeringOutput};
  giveState();
  return true;
}

bool MotionController::failsafeActive() const {
  if (!started_.load() || !takeState(kCommandLockTimeout)) return false;
  const bool active = state_.failsafeActive;
  giveState();
  return active;
}

bool MotionController::demoRunning() const {
  if (!started_.load() || !takeState(kCommandLockTimeout)) return false;
  const bool running = state_.demoState != DemoState::Idle;
  giveState();
  return running;
}

const char *MotionController::modeName(DriveMode mode) {
  switch (mode) {
    case DriveMode::Idle:
      return "IDLE";
    case DriveMode::Bionic:
      return "BIONIC";
    case DriveMode::Prop:
      return "PROP";
    case DriveMode::Hybrid:
      return "HYBRID";
    default:
      return "UNKNOWN";
  }
}

const char *MotionController::demoResultText(DemoRequestResult result) {
  switch (result) {
    case DemoRequestResult::Started:
      return "DEMO_STARTED";
    case DemoRequestResult::Cancelled:
      return "DEMO_CANCELLED";
    case DemoRequestResult::BlockedFailsafe:
      return "DEMO_BLOCKED_FAILSAFE";
    default:
      return "DEMO_UNAVAILABLE";
  }
}
