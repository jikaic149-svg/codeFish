#include "RoboFishDriver.h"

#include "ESP32PWM.h"
#include "sdk_config.h"

RoboFishDriver::RoboFishDriver()
    : mutex_(nullptr),
      ready_(false),
      otaLocked_(false),
      pumpEnabled_(false),
      throttleCommand_(RF_ESC_THROTTLE_NEUTRAL),
      lastThrottleCommandMs_(0),
      pumpStartedMs_(0) {}

bool RoboFishDriver::begin() {
  if (ready_) return true;
  mutex_ = xSemaphoreCreateMutex();
  if (!mutex_) {
    Serial.println("[DRIVER] Cannot create actuator mutex");
    return false;
  }

  // Timer 0 is intentionally left for other ESP32 peripherals.
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  bool attached = true;
  attached &= tailServo1_.attach(RF_TAIL_SERVO_1_PIN, 500, 2500) != 0;
  attached &= tailServo2_.attach(RF_TAIL_SERVO_2_PIN, 500, 2500) != 0;
  attached &= pitchServo_.attach(RF_PITCH_SERVO_PIN, 500, 2500) != 0;
  attached &= escThrottle_.attach(RF_ESC_THROTTLE_PIN,
                                  RF_ESC_PWM_MIN_US, RF_ESC_PWM_MAX_US) != 0;
  attached &= escSteering_.attach(RF_ESC_STEERING_PIN,
                                  RF_ESC_PWM_MIN_US, RF_ESC_PWM_MAX_US) != 0;

  pinMode(RF_PUMP_PIN, OUTPUT);
  ready_ = attached;
  if (!takeLock()) return false;
  emergencyStopUnlocked();
  giveLock();

  Serial.printf("[DRIVER] Tail1 GPIO%d, Tail2 GPIO%d, Pitch GPIO%d\n",
                RF_TAIL_SERVO_1_PIN, RF_TAIL_SERVO_2_PIN, RF_PITCH_SERVO_PIN);
  Serial.printf("[DRIVER] Pump GPIO%d, ESC throttle GPIO%d, ESC steering GPIO%d\n",
                RF_PUMP_PIN, RF_ESC_THROTTLE_PIN, RF_ESC_STEERING_PIN);
  Serial.printf("[DRIVER] Initialization %s\n", ready_ ? "OK" : "FAILED");
  return ready_;
}

bool RoboFishDriver::takeLock(TickType_t timeout) {
  return mutex_ && xSemaphoreTake(mutex_, timeout) == pdTRUE;
}

void RoboFishDriver::giveLock() {
  if (mutex_) xSemaphoreGive(mutex_);
}

void RoboFishDriver::setTailServo1(int angle) {
  if (!ready_ || otaLocked_ || !takeLock()) return;
  tailServo1_.write(constrain(angle, RF_TAIL_MIN_ANGLE, RF_TAIL_MAX_ANGLE));
  giveLock();
}

void RoboFishDriver::setTailServo2(int angle) {
  if (!ready_ || otaLocked_ || !takeLock()) return;
  tailServo2_.write(constrain(angle, RF_TAIL_MIN_ANGLE, RF_TAIL_MAX_ANGLE));
  giveLock();
}

void RoboFishDriver::setTailServos(int angle1, int angle2) {
  if (!ready_ || otaLocked_ || !takeLock()) return;
  tailServo1_.write(constrain(angle1, RF_TAIL_MIN_ANGLE, RF_TAIL_MAX_ANGLE));
  tailServo2_.write(constrain(angle2, RF_TAIL_MIN_ANGLE, RF_TAIL_MAX_ANGLE));
  giveLock();
}

void RoboFishDriver::setPitchServo(int angle) {
  if (!ready_ || otaLocked_ || !takeLock()) return;
  pitchServo_.write(constrain(angle, RF_PITCH_MIN_ANGLE, RF_PITCH_MAX_ANGLE));
  giveLock();
}

void RoboFishDriver::writePumpUnlocked(bool enabled) {
  digitalWrite(RF_PUMP_PIN,
               enabled ? (RF_PUMP_ACTIVE_HIGH ? HIGH : LOW)
                       : (RF_PUMP_ACTIVE_HIGH ? LOW : HIGH));
  pumpEnabled_ = enabled;
  pumpStartedMs_ = enabled ? millis() : 0;
}

void RoboFishDriver::setPump(bool enabled) {
  if (!ready_ || otaLocked_ || !takeLock()) return;
  writePumpUnlocked(enabled);
  giveLock();
}

void RoboFishDriver::setEscThrottle(int command) {
  if (!ready_ || otaLocked_ || !takeLock()) return;
  throttleCommand_ = constrain(command, RF_ESC_THROTTLE_MIN, RF_ESC_THROTTLE_MAX);
  escThrottle_.write(throttleCommand_);
  lastThrottleCommandMs_ = millis();
  giveLock();
}

void RoboFishDriver::setEscSteering(int command) {
  if (!ready_ || otaLocked_ || !takeLock()) return;
  escSteering_.write(constrain(command, RF_ESC_STEERING_MIN, RF_ESC_STEERING_MAX));
  giveLock();
}

void RoboFishDriver::centerTail() {
  setTailServos(RF_TAIL_CENTER, RF_TAIL_CENTER);
}

void RoboFishDriver::stopPropulsionUnlocked() {
  throttleCommand_ = RF_ESC_THROTTLE_NEUTRAL;
  escThrottle_.write(RF_ESC_THROTTLE_NEUTRAL);
  escSteering_.write(RF_ESC_STEERING_CENTER);
  lastThrottleCommandMs_ = 0;
}

void RoboFishDriver::stopPropulsion() {
  if (!ready_ || !takeLock()) return;
  stopPropulsionUnlocked();
  giveLock();
}

void RoboFishDriver::emergencyStopUnlocked() {
  tailServo1_.write(RF_TAIL_CENTER);
  tailServo2_.write(RF_TAIL_CENTER);
  pitchServo_.write(RF_PITCH_CENTER);
  stopPropulsionUnlocked();
  writePumpUnlocked(false);
}

void RoboFishDriver::emergencyStop() {
  if (!ready_ || !takeLock()) return;
  emergencyStopUnlocked();
  giveLock();
}

void RoboFishDriver::prepareForOta() {
  if (!ready_ || !takeLock(pdMS_TO_TICKS(1000))) return;
  emergencyStopUnlocked();
  otaLocked_ = true;
  giveLock();
  Serial.println("[DRIVER] Actuators stopped and locked for OTA");
}

void RoboFishDriver::update() {
  if (!ready_ || otaLocked_ || !takeLock(0)) return;
  uint32_t now = millis();

  if (throttleCommand_ != RF_ESC_THROTTLE_NEUTRAL &&
      lastThrottleCommandMs_ != 0 &&
      now - lastThrottleCommandMs_ >= RF_PROPULSION_TIMEOUT_MS) {
    stopPropulsionUnlocked();
    Serial.println("[DRIVER] Propulsion timeout -> neutral");
  }

  if (pumpEnabled_ && pumpStartedMs_ != 0 &&
      now - pumpStartedMs_ >= RF_PUMP_MAX_ON_MS) {
    writePumpUnlocked(false);
    Serial.println("[DRIVER] Pump timeout -> off");
  }
  giveLock();
}

bool RoboFishDriver::ready() const {
  return ready_;
}

bool RoboFishDriver::otaLocked() const {
  return otaLocked_;
}

