#pragma once

#include <Arduino.h>

typedef void (*OtaStartCallback)();

struct OtaServiceConfig {
  const char *apSsid;
  const char *apPassword;
  const char *staSsid;
  const char *staPassword;
  const char *username;
  const char *password;
};

// Starts Wi-Fi and a dedicated background Web OTA task. Keeping OTA outside
// loop() makes it less likely that a slow partner algorithm blocks updates.
bool otaServiceBegin(const OtaServiceConfig &config, OtaStartCallback onStart);

