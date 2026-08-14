#include "WebAssets.h"

#include <limits>

namespace {

// These symbols are emitted by ESP-IDF EMBED_TXTFILES. Keep the original
// recovered files as the single source of truth instead of copying or editing
// the HTML/JS/JSON in C++.
extern "C" {
extern const uint8_t driverPageStart[]
    asm("_binary_html_01_0x0000100b_html_start");
extern const uint8_t driverPageEnd[]
    asm("_binary_html_01_0x0000100b_html_end");
extern const uint8_t partnerPageStart[]
    asm("_binary_html_02_0x0001f6ad_html_start");
extern const uint8_t partnerPageEnd[]
    asm("_binary_html_02_0x0001f6ad_html_end");
extern const uint8_t manifestStart[]
    asm("_binary_manifest_01_0x00000d57_json_start");
extern const uint8_t manifestEnd[]
    asm("_binary_manifest_01_0x00000d57_json_end");
extern const uint8_t serviceWorkerStart[]
    asm("_binary_service_worker_01_0x00000bc5_js_start");
extern const uint8_t serviceWorkerEnd[]
    asm("_binary_service_worker_01_0x00000bc5_js_end");
}

RoboFishWebAsset textAsset(const uint8_t *start, const uint8_t *end,
                           const char *contentType) {
  // EMBED_TXTFILES appends exactly one NUL byte which is not part of the
  // recovered resource and must not be sent over HTTP.
  const size_t linkedBytes = static_cast<size_t>(end - start);
  return {start, linkedBytes == 0 ? 0 : linkedBytes - 1, contentType};
}

esp_err_t sendAsset(httpd_req_t *request, const RoboFishWebAsset &asset) {
  if (!request || !asset.data || asset.size == 0 || !asset.contentType ||
      asset.size > static_cast<size_t>(std::numeric_limits<ssize_t>::max())) {
    return ESP_ERR_INVALID_ARG;
  }
  esp_err_t error = httpd_resp_set_type(request, asset.contentType);
  if (error != ESP_OK) return error;
  return httpd_resp_send(request,
                         reinterpret_cast<const char *>(asset.data),
                         static_cast<ssize_t>(asset.size));
}

esp_err_t driverPageHandler(httpd_req_t *request) {
  return sendAsset(request, roboFishDriverPageAsset());
}

esp_err_t partnerPageHandler(httpd_req_t *request) {
  return sendAsset(request, roboFishPartnerPageAsset());
}

esp_err_t manifestHandler(httpd_req_t *request) {
  return sendAsset(request, roboFishManifestAsset());
}

esp_err_t serviceWorkerHandler(httpd_req_t *request) {
  return sendAsset(request, roboFishServiceWorkerAsset());
}

}  // namespace

RoboFishWebAsset roboFishDriverPageAsset() {
  return textAsset(driverPageStart, driverPageEnd,
                   "text/html; charset=utf-8");
}

RoboFishWebAsset roboFishPartnerPageAsset() {
  return textAsset(partnerPageStart, partnerPageEnd,
                   "text/html; charset=utf-8");
}

RoboFishWebAsset roboFishManifestAsset() {
  return textAsset(manifestStart, manifestEnd,
                   "application/manifest+json; charset=utf-8");
}

RoboFishWebAsset roboFishServiceWorkerAsset() {
  return textAsset(serviceWorkerStart, serviceWorkerEnd,
                   "application/javascript; charset=utf-8");
}

esp_err_t roboFishRegisterWebAssetRoutes(httpd_handle_t server) {
  if (!server) return ESP_ERR_INVALID_ARG;

  const httpd_uri_t routes[] = {
      {.uri = "/gpio",
       .method = HTTP_GET,
       .handler = driverPageHandler,
       .user_ctx = nullptr},
      {.uri = "/partner",
       .method = HTTP_GET,
       .handler = partnerPageHandler,
       .user_ctx = nullptr},
      {.uri = "/manifest.json",
       .method = HTTP_GET,
       .handler = manifestHandler,
       .user_ctx = nullptr},
      {.uri = "/sw.js",
       .method = HTTP_GET,
       .handler = serviceWorkerHandler,
       .user_ctx = nullptr},
  };

  for (const httpd_uri_t &route : routes) {
    const esp_err_t error = httpd_register_uri_handler(server, &route);
    if (error != ESP_OK) return error;
  }
  return ESP_OK;
}
