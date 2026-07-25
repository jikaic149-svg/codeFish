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

#include "OtaService.h"
#include "RoboFishDriver.h"
#include "sdk_config.h"

RoboFishDriver fish;

void stopForOta() {
  fish.prepareForOta();
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\nRoboFish Partner Driver SDK 1.0.0");

  if (!fish.begin()) {
    Serial.println("Driver initialization failed. Check board selection and wiring.");
  }

  OtaServiceConfig otaConfig = {
    RF_AP_SSID,
    RF_AP_PASSWORD,
    RF_STA_SSID,
    RF_STA_PASSWORD,
    RF_OTA_USERNAME,
    RF_OTA_PASSWORD
  };
  if (!otaServiceBegin(otaConfig, stopForOta)) {
    Serial.println("Web OTA service failed to start.");
  }

  Serial.println("No swimming algorithm is included.");
  Serial.println("Implement your actuator timing in loop().");
}

void loop() {
  // Keep the basic driver safety timeouts active.
  fish.update();

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

  delay(10);
}

