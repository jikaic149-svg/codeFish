#pragma once

#include <cstddef>
#include <cstdint>

#include "PartnerScriptEngine.h"
#include "esp_err.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace robofish::partner {

struct NetworkStatus {
  bool staConnected = false;
  char staIp[16] = "0.0.0.0";
  char staSsid[33] = {};
  char apIp[16] = "0.0.0.0";
};

using ReadNetworkStatusCallback = bool (*)(void *context,
                                           NetworkStatus &status);
using SaveStaCredentialsCallback = bool (*)(void *context, const char *ssid,
                                            const char *password);
using RequestRestartCallback = void (*)(void *context);

struct HttpCallbacks {
  ReadNetworkStatusCallback readNetworkStatus = nullptr;
  SaveStaCredentialsCallback saveStaCredentials = nullptr;
  RequestRestartCallback requestRestart = nullptr;
  void *context = nullptr;
};

// Registers the Partner and recovered network-status APIs on an HTTP server
// owned by the application. The service owns no task and performs no dynamic
// allocation; its instance must outlive the registered HTTP server.
class HttpService {
 public:
  HttpService(ScriptEngine &engine, const HttpCallbacks &callbacks);

  esp_err_t registerRoutes(httpd_handle_t server,
                           bool includeNetworkRoutes = true);

 private:
  static esp_err_t scriptGetHandler(httpd_req_t *request);
  static esp_err_t scriptPostHandler(httpd_req_t *request);
  static esp_err_t runHandler(httpd_req_t *request);
  static esp_err_t stopHandler(httpd_req_t *request);
  static esp_err_t statusHandler(httpd_req_t *request);
  static esp_err_t getIpHandler(httpd_req_t *request);
  static esp_err_t saveWifiHandler(httpd_req_t *request);

  esp_err_t sendScript(httpd_req_t *request);
  esp_err_t saveScript(httpd_req_t *request);
  esp_err_t startScript(httpd_req_t *request);
  esp_err_t stopScript(httpd_req_t *request);
  esp_err_t sendStatus(httpd_req_t *request);
  esp_err_t sendNetworkStatus(httpd_req_t *request);
  esp_err_t saveWifi(httpd_req_t *request);

  bool takeWorkspace();
  void giveWorkspace();

  ScriptEngine &engine_;
  HttpCallbacks callbacks_;
  StaticSemaphore_t workspaceLockStorage_{};
  SemaphoreHandle_t workspaceLock_ = nullptr;
  char json_[kMaxJsonBytes + 1] = {};
  char response_[384] = {};
  char query_[513] = {};
};

}  // namespace robofish::partner
