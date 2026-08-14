#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_http_server.h"

// Read-only view over one resource recovered from CameraTest.ino.bin.
// The backing bytes are linked directly from reverse_analysis/web_assets;
// callers must not modify or free them.
struct RoboFishWebAsset {
  const uint8_t *data;
  size_t size;
  const char *contentType;
};

RoboFishWebAsset roboFishDriverPageAsset();
RoboFishWebAsset roboFishPartnerPageAsset();
RoboFishWebAsset roboFishManifestAsset();
RoboFishWebAsset roboFishServiceWorkerAsset();

// Registers only the recovered static resources. Business API routes such as
// /control, /partner_script, /get_ip and /ota_upload remain owned by their
// respective services.
//
// Routes:
//   GET /gpio
//   GET /partner
//   GET /manifest.json
//   GET /sw.js
esp_err_t roboFishRegisterWebAssetRoutes(httpd_handle_t server);
