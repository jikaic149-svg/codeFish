#pragma once

#include <atomic>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "device/RoboFishDriver.h"

enum class DriveMode : uint8_t {
  Idle = 0,
  Bionic = 1,
  Prop = 2,
  Hybrid = 3,
};

enum class MotionAction : uint8_t {
  Stopped = 0,
  Forward,
  Backward,
  Left,
  Right,
};

enum class DemoState : uint8_t {
  Idle = 0,
  Forward,
  Dive,
  Surface,
  Stop,
};

enum class DemoRequestResult : uint8_t {
  Started,
  Cancelled,
  BlockedFailsafe,
  Unavailable,
};

enum class VisionControlResult : uint8_t {
  Accepted,
  StaleSequence,
  Rejected,
};

enum class VisionBrakePhase : uint8_t {
  Cruise = 0,
  Neutralizing,
  NeutralDwell,
  ReverseBrake,
  Hold,
};

struct MotionStatus {
  DriveMode mode;
  MotionAction action;
  DemoState demoState;
  uint8_t powerPercent;
  uint8_t turnStrengthPercent;
  int pitchAngle;
  bool pumpEnabled;
  bool failsafeActive;
  uint8_t apClientCount;
  bool staConnected;
  int tail1Output;
  int tail2Output;
  int escThrottleOutput;
  int escSteeringOutput;
  uint32_t visionAppliedSequence;
};

// Restored application-level motion coordinator for the recovered CameraTest
// firmware. RoboFishDriver remains the sole hardware owner.
//
// Task contract:
// - one Core 1 output task, priority 2, 20 ms period, 4096-byte static stack;
// - one Core 1 vision-lease task, priority 2, 20 ms period, 2048-byte static
//   stack; it owns visual-command expiry but never writes hardware directly;
// - each period computes at most one actuator update and never busy-waits;
// - commands only update bounded state under a finite-time mutex;
// - update() advances the 10 ms-class Demo/failsafe state machines.
class MotionController {
 public:
  explicit MotionController(RoboFishDriver &driver);

  bool begin();
  void update();

  bool setMode(DriveMode mode);
  bool cycleMode(DriveMode *newMode = nullptr);

  bool forward(uint8_t powerPercent = 55);
  bool backward(uint8_t powerPercent = 55);
  bool left(uint8_t powerPercent = 55,
            uint8_t strengthPercent = 100);
  bool right(uint8_t powerPercent = 55,
             uint8_t strengthPercent = 100);
  bool stop();

  bool setPitch(int angle);
  bool setPump(bool enabled);
  bool configureDirection(bool escReversed, bool steeringReversed,
                          bool tailReversed);
  bool setTailReversed(bool reversed);
  bool tailReversed() const;

  // Executes one Partner-script command without applying manual-control
  // semantics. In particular it never clears failsafe and never cancels the
  // script that owns the command. The whole state transition is made under
  // the controller mutex so the script cannot partially update motion.
  bool applyAutomationCommand(DriveMode mode);
  bool applyAutomationAction(MotionAction action, uint8_t powerPercent = 55,
                             uint8_t strengthPercent = 40);
  bool applyAutomationPitch(int angle);
  bool applyAutomationPump(bool enabled);

  // Partner/script cleanup: stop propulsion and turn the pump off while
  // preserving pitch, Demo ownership and failsafe state.
  bool automationSafeStop();

  // Applies the legacy HTTP vision command in HYBRID mode. powerPercent drives
  // both tail amplitude and propeller power; signed steeringPercent keeps the
  // recovered HTTP polarity (negative=left, positive=right).
  VisionControlResult applyVisionControl(uint32_t sequence,
                                         uint8_t powerPercent,
                                         int steeringPercent,
                                         bool brakeRequested = false);

  // UDP tracking uses the same actuator mixer as the legacy HTTP command, but
  // its wire polarity is positive=left to match the PC path-control law. The
  // sign translation is performed here and UDP is rejected while an
  // HTTP/manual priority lease is active.
  VisionControlResult applyVisionUdp(uint32_t sequence,
                                     uint8_t powerPercent,
                                     int steeringPercent,
                                     bool brakeRequested);
  void noteHttpPriority();

  // Enqueues the latest calibrated visual errors for the VisionControl task.
  // xErrorM is signed lateral error; yErrorM is remaining forward distance.
  // Speed and upcoming curve severity shape independent tail/propeller power;
  // brakeRequested enters the non-blocking soft reverse-brake state machine.
  VisionControlResult applyVisionPid(uint32_t sequence, float xErrorM,
                                     float yErrorM, float speedMps,
                                     float curveSeverity,
                                     bool brakeRequested);

  DemoRequestResult requestDemo();
  bool cancelDemo(bool returnToSafePose = true);

  // Network service supplies link facts; MotionController owns all timing.
  void setLinkState(uint8_t apClientCount, bool staConnected);
  bool triggerFailsafe(const char *reason);

  bool getStatus(MotionStatus &status) const;
  bool failsafeActive() const;
  bool demoRunning() const;

  static const char *modeName(DriveMode mode);
  static const char *demoResultText(DemoRequestResult result);

 private:
  struct State {
    DriveMode mode;
    MotionAction action;
    DemoState demoState;
    uint8_t powerPercent;
    uint8_t turnStrengthPercent;
    int pitchAngle;
    bool pitchDirty;
    bool pumpEnabled;
    bool pumpDirty;
    uint32_t pumpStartedMs;
    bool failsafeActive;
    uint8_t apClientCount;
    bool staConnected;
    bool escReversed;
    bool steeringReversed;
    bool tailReversed;
    uint32_t bootTimeMs;
    uint32_t lastLinkCheckMs;
    uint32_t disconnectedSinceMs;
    uint32_t demoStateStartedMs;
    bool visionControlActive;
    uint32_t visionSequence;
    uint32_t visionAppliedSequence;
    uint32_t lastVisionCommandMs;
    bool visionPidEnabled;
    bool visionPidPending;
    float visionXErrorM;
    float visionYErrorM;
    float visionXIntegral;
    float visionYIntegral;
    float visionXPreviousErrorM;
    float visionYPreviousErrorM;
    uint32_t visionPidPreviousMs;
    float tail1Target;
    float tail2Target;
    float tail1Output;
    float tail2Output;
    int escThrottleTarget;
    int escSteeringTarget;
    int escThrottleOutput;
    int escSteeringOutput;
    uint8_t visionTailPowerPercent;
    uint8_t visionPropPowerPercent;
    float visionSpeedMps;
    float visionCurveSeverity;
    bool visionBrakeRequested;
    VisionBrakePhase visionBrakePhase;
    uint32_t visionBrakePhaseStartedMs;
    bool visionPropReverse;
    uint32_t visionHttpPriorityUntilMs;
  };

  static constexpr uint32_t kControlTaskStackBytes = 4096;
  static constexpr uint32_t kVisionTaskStackBytes = 2048;
  static constexpr TickType_t kCommandLockTimeout = pdMS_TO_TICKS(10);
  static constexpr TickType_t kTickLockTimeout = pdMS_TO_TICKS(2);

  static void controlTaskEntry(void *context);
  void controlTask();
  static void visionTaskEntry(void *context);
  void visionTask();
  void visionTick(uint32_t nowMs);
  void controlTick(uint32_t nowMs);

  bool takeState(TickType_t timeout) const;
  void giveState() const;
  bool commandAllowed() const;
  bool setAction(MotionAction action, uint8_t powerPercent,
                 uint8_t strengthPercent);
  bool onOperatorCommandLocked();
  bool automationAllowedLocked() const;
  VisionControlResult applyVisionCommandLocked(uint32_t sequence,
                                               uint8_t powerPercent,
                                               int steeringPercent,
                                               bool brakeRequested);
  void stopMotionLocked();
  void resetVisionBrakeLocked();
  void cancelDemoLocked(bool returnToSafePose);
  bool triggerFailsafeLocked();

  RoboFishDriver &driver_;
  mutable StaticSemaphore_t stateMutexStorage_;
  mutable SemaphoreHandle_t stateMutex_;
  StaticTask_t controlTaskStorage_;
  alignas(16) StackType_t controlTaskStack_[kControlTaskStackBytes /
                                             sizeof(StackType_t)];
  TaskHandle_t controlTaskHandle_;
  StaticTask_t visionTaskStorage_;
  alignas(16) StackType_t visionTaskStack_[kVisionTaskStackBytes /
                                           sizeof(StackType_t)];
  TaskHandle_t visionTaskHandle_;
  std::atomic_bool started_;
  State state_;
};
