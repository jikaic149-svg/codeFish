/*
 * RoboFish Partner Driver SDK 1.0.0
 *
 * This distributable source version intentionally provides only:
 *   1. Low-level tail/pitch servo, ESC and pump drivers.
 *   2. A local Wi-Fi AP and background Web OTA service.
 *   3. Conservative output limits and simple hardware timeouts.
 *
 * It intentionally does NOT contain camera streaming, CPG, gait generation,
 * forward/backward/left/right actions, demo sequences, autonomous control or
 * the proprietary swimming parameters from the full RoboFish firmware.
 */

#include "src/app/RoboFishRuntime.h"
#include "src/config/sdk_config.h"

RoboFishRuntime runtime;
RoboFishDriver &fish = runtime.driver();

void setup() {
  runtime.begin();
}

void loop() {
  // Keep the basic driver safety timeouts active.
  runtime.update();

  // Partner development starts here. Examples of individual hardware calls:
  //
  // fish.setTailServo1(86);        // first tail joint
  // fish.setTailServo2(86);        // second tail joint
  // fish.setTailServos(70, 100);   // set both joints, no gait generation
  // fish.setPitchServo(45);        // float/up command
  // fish.setPitchServo(90);        // neutral
  // fish.setPitchServo(135);       // dive/down command
  // fish.setPump(true);            // automatically turns off after 10 seconds
  // fish.setEscThrottle(90);       // neutral; refresh non-neutral commands <2s
  // fish.setEscSteering(88);       // steering center
  // fish.emergencyStop();          // center/stop every actuator
  //
  // To make the fish swim, write the required time-varying joint coordination,
  // propulsion control and safety state machine here.

  delay(RF_MAIN_LOOP_DELAY_MS);
}
