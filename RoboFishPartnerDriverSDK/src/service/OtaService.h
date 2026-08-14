#pragma once

#include <stdint.h>

#include "esp_http_server.h"

typedef bool (*OtaStartCallback)();
typedef void (*OtaLinkStateCallback)(uint8_t apClientCount,
                                     bool staConnected, void *context);

struct OtaServiceConfig {
  const char *apSsid;
  const char *apPassword;
  const char *staSsid;
  const char *staPassword;
  const char *username;
  const char *password;
  OtaLinkStateCallback onLinkState;
  void *linkStateContext;
};

// Initializes the single default NVS partition used by Wi-Fi, Partner scripts
// and recovered drive configuration. Call once before any NVS consumer.
bool otaServiceInitializeNvs();

// Starts the ESP-IDF Wi-Fi AP/optional STA and HTTP Web OTA service.
bool otaServiceBegin(const OtaServiceConfig &config, OtaStartCallback onStart);

// The application registers recovered product routes on this single port-80
// server. This avoids two HTTP owners racing for the same TCP port.
httpd_handle_t otaServiceHttpServer();
