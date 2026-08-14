#include "PartnerHttpService.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>

#include "esp_log.h"
#include "esp_timer.h"

namespace robofish::partner {
namespace {

constexpr char kTag[] = "partner_http";
constexpr TickType_t kWorkspaceLockTimeout = pdMS_TO_TICKS(50);

esp_err_t send(httpd_req_t *request, const char *status,
               const char *contentType, const char *body,
               ssize_t length = HTTPD_RESP_USE_STRLEN) {
  if (!request || !status || !contentType || !body) return ESP_ERR_INVALID_ARG;
  esp_err_t error = httpd_resp_set_status(request, status);
  if (error == ESP_OK) error = httpd_resp_set_type(request, contentType);
  if (error == ESP_OK) error = httpd_resp_send(request, body, length);
  return error;
}

esp_err_t sendJsonError(httpd_req_t *request, const char *status,
                        const char *message) {
  std::array<char, 192> body = {};
  const int length = std::snprintf(body.data(), body.size(),
                                   "{\"success\":false,\"error\":\"%s\"}",
                                   message ? message : "Unknown error");
  if (length < 0 || static_cast<size_t>(length) >= body.size()) {
    return send(request, "500 Internal Server Error", "application/json",
                "{\"success\":false,\"error\":\"Internal error\"}");
  }
  return send(request, status, "application/json; charset=utf-8",
              body.data(), length);
}

bool decodeComponent(const char *source, char *destination, size_t capacity) {
  if (!source || !destination || capacity == 0) return false;
  auto hexValue = [](char digit) -> int {
    if (digit >= '0' && digit <= '9') return digit - '0';
    if (digit >= 'a' && digit <= 'f') return digit - 'a' + 10;
    if (digit >= 'A' && digit <= 'F') return digit - 'A' + 10;
    return -1;
  };

  size_t output = 0;
  for (size_t input = 0; source[input] != '\0'; ++input) {
    unsigned char value = static_cast<unsigned char>(source[input]);
    if (value == '+') {
      value = ' ';
    } else if (value == '%') {
      if (source[input + 1] == '\0' || source[input + 2] == '\0') return false;
      const int high = hexValue(source[input + 1]);
      const int low = hexValue(source[input + 2]);
      if (high < 0 || low < 0) return false;
      value = static_cast<unsigned char>((high << 4) | low);
      input += 2;
      if (value == 0) return false;
    }
    if (output + 1 >= capacity) return false;
    destination[output++] = static_cast<char>(value);
  }
  destination[output] = '\0';
  return true;
}

const char *statusForResult(Result result) {
  switch (result) {
    case Result::kInvalidArgument:
    case Result::kInvalidJson:
      return "400 Bad Request";
    case Result::kBlockedByFailsafe:
    case Result::kNoValidScript:
    case Result::kCommandRejected:
      return "409 Conflict";
    default:
      return "500 Internal Server Error";
  }
}

HttpService *serviceFrom(httpd_req_t *request) {
  return request ? static_cast<HttpService *>(request->user_ctx) : nullptr;
}

esp_err_t registerRoute(httpd_handle_t server, const char *uri,
                        httpd_method_t method,
                        esp_err_t (*handler)(httpd_req_t *), void *context) {
  const httpd_uri_t route = {
      .uri = uri,
      .method = method,
      .handler = handler,
      .user_ctx = context,
  };
  return httpd_register_uri_handler(server, &route);
}

}  // namespace

HttpService::HttpService(ScriptEngine &engine, const HttpCallbacks &callbacks)
    : engine_(engine), callbacks_(callbacks) {
  workspaceLock_ = xSemaphoreCreateMutexStatic(&workspaceLockStorage_);
}

esp_err_t HttpService::registerRoutes(httpd_handle_t server,
                                      bool includeNetworkRoutes) {
  if (!server || !workspaceLock_) return ESP_ERR_INVALID_ARG;
  struct Route {
    const char *uri;
    httpd_method_t method;
    esp_err_t (*handler)(httpd_req_t *);
  };
  constexpr Route kPartnerRoutes[] = {
      {"/partner_script", HTTP_GET, scriptGetHandler},
      {"/partner_script", HTTP_POST, scriptPostHandler},
      {"/partner_run", HTTP_POST, runHandler},
      {"/partner_stop", HTTP_POST, stopHandler},
      {"/partner_status", HTTP_GET, statusHandler},
  };
  for (const Route &route : kPartnerRoutes) {
    const esp_err_t error =
        registerRoute(server, route.uri, route.method, route.handler, this);
    if (error != ESP_OK) return error;
  }
  if (!includeNetworkRoutes) return ESP_OK;
  esp_err_t error =
      registerRoute(server, "/get_ip", HTTP_GET, getIpHandler, this);
  if (error == ESP_OK) {
    error = registerRoute(server, "/save_wifi", HTTP_GET, saveWifiHandler,
                          this);
  }
  return error;
}

bool HttpService::takeWorkspace() {
  return workspaceLock_ &&
         xSemaphoreTake(workspaceLock_, kWorkspaceLockTimeout) == pdTRUE;
}

void HttpService::giveWorkspace() {
  if (workspaceLock_) xSemaphoreGive(workspaceLock_);
}

esp_err_t HttpService::scriptGetHandler(httpd_req_t *request) {
  HttpService *service = serviceFrom(request);
  return service ? service->sendScript(request) : ESP_ERR_INVALID_ARG;
}

esp_err_t HttpService::scriptPostHandler(httpd_req_t *request) {
  HttpService *service = serviceFrom(request);
  return service ? service->saveScript(request) : ESP_ERR_INVALID_ARG;
}

esp_err_t HttpService::runHandler(httpd_req_t *request) {
  HttpService *service = serviceFrom(request);
  return service ? service->startScript(request) : ESP_ERR_INVALID_ARG;
}

esp_err_t HttpService::stopHandler(httpd_req_t *request) {
  HttpService *service = serviceFrom(request);
  return service ? service->stopScript(request) : ESP_ERR_INVALID_ARG;
}

esp_err_t HttpService::statusHandler(httpd_req_t *request) {
  HttpService *service = serviceFrom(request);
  return service ? service->sendStatus(request) : ESP_ERR_INVALID_ARG;
}

esp_err_t HttpService::getIpHandler(httpd_req_t *request) {
  HttpService *service = serviceFrom(request);
  return service ? service->sendNetworkStatus(request) : ESP_ERR_INVALID_ARG;
}

esp_err_t HttpService::saveWifiHandler(httpd_req_t *request) {
  HttpService *service = serviceFrom(request);
  return service ? service->saveWifi(request) : ESP_ERR_INVALID_ARG;
}

esp_err_t HttpService::sendScript(httpd_req_t *request) {
  if (!takeWorkspace()) {
    return sendJsonError(request, "503 Service Unavailable",
                         "Partner service is busy");
  }
  size_t length = 0;
  const Result result = engine_.copyJson(json_, sizeof(json_), &length);
  esp_err_t error = ESP_OK;
  if (result == Result::kOk) {
    error = send(request, "200 OK", "application/json; charset=utf-8", json_,
                 static_cast<ssize_t>(length));
  } else {
    error = sendJsonError(request, statusForResult(result), resultText(result));
  }
  giveWorkspace();
  return error;
}

esp_err_t HttpService::saveScript(httpd_req_t *request) {
  if (!request || request->content_len < 1 ||
      request->content_len > kMaxJsonBytes) {
    return sendJsonError(request, "400 Bad Request",
                         "Script must be 1..3500 bytes");
  }
  if (!takeWorkspace()) {
    return sendJsonError(request, "503 Service Unavailable",
                         "Partner service is busy");
  }

  size_t received = 0;
  while (received < request->content_len) {
    const int count = httpd_req_recv(request, json_ + received,
                                     request->content_len - received);
    if (count <= 0) {
      giveWorkspace();
      return sendJsonError(request, "400 Bad Request",
                           "Incomplete request body");
    }
    received += static_cast<size_t>(count);
  }
  json_[received] = '\0';

  std::array<char, 128> validationError = {};
  const Result result = engine_.validateAndSaveJson(
      json_, received, validationError.data(), validationError.size());
  esp_err_t error = ESP_OK;
  if (result == Result::kOk) {
    error = send(request, "200 OK", "application/json; charset=utf-8",
                 "{\"success\":true,\"message\":\"Script validated and "
                 "saved\"}");
  } else {
    error = sendJsonError(request, statusForResult(result),
                          validationError[0] ? validationError.data()
                                             : resultText(result));
  }
  giveWorkspace();
  return error;
}

esp_err_t HttpService::startScript(httpd_req_t *request) {
  std::array<char, 96> startMessage = {};
  const Result result =
      engine_.start(startMessage.data(), startMessage.size());
  if (result != Result::kOk) {
    return sendJsonError(request, "409 Conflict",
                         startMessage[0] ? startMessage.data()
                                         : resultText(result));
  }
  const Status state =
      engine_.status(static_cast<uint32_t>(esp_timer_get_time() / 1000ULL));
  std::array<char, 128> body = {};
  const int length = std::snprintf(
      body.data(), body.size(),
      "{\"success\":true,\"message\":\"Started: %s\"}", state.name);
  if (length < 0 || static_cast<size_t>(length) >= body.size()) {
    return sendJsonError(request, "500 Internal Server Error",
                         "Internal error");
  }
  return send(request, "200 OK", "application/json; charset=utf-8",
              body.data(), length);
}

esp_err_t HttpService::stopScript(httpd_req_t *request) {
  engine_.stop();
  return send(request, "200 OK", "application/json; charset=utf-8",
              "{\"success\":true,\"message\":\"Script safely stopped\"}");
}

esp_err_t HttpService::sendStatus(httpd_req_t *request) {
  if (!takeWorkspace()) {
    return sendJsonError(request, "503 Service Unavailable",
                         "Partner service is busy");
  }
  size_t length = 0;
  const Result result = engine_.copyStatusJson(
      static_cast<uint32_t>(esp_timer_get_time() / 1000ULL), response_,
      sizeof(response_), &length);
  esp_err_t error = ESP_OK;
  if (result == Result::kOk) {
    error = send(request, "200 OK", "application/json; charset=utf-8",
                 response_, static_cast<ssize_t>(length));
  } else {
    error = sendJsonError(request, statusForResult(result), resultText(result));
  }
  giveWorkspace();
  return error;
}

esp_err_t HttpService::sendNetworkStatus(httpd_req_t *request) {
  if (!callbacks_.readNetworkStatus) {
    return sendJsonError(request, "503 Service Unavailable",
                         "Network service unavailable");
  }
  if (!takeWorkspace()) {
    return sendJsonError(request, "503 Service Unavailable",
                         "Partner service is busy");
  }
  NetworkStatus status = {};
  if (!callbacks_.readNetworkStatus(callbacks_.context, status)) {
    giveWorkspace();
    return sendJsonError(request, "500 Internal Server Error",
                         "Cannot read network status");
  }
  const int length = std::snprintf(
      response_, sizeof(response_),
      "{\"sta_connected\":%d,\"sta_ip\":\"%s\",\"sta_ssid\":\"%s\","
      "\"ap_ip\":\"%s\",\"version\":\"4.0.0-partner\"}",
      status.staConnected ? 1 : 0, status.staIp, status.staSsid, status.apIp);
  if (length < 0 || static_cast<size_t>(length) >= sizeof(response_)) {
    giveWorkspace();
    return sendJsonError(request, "500 Internal Server Error",
                         "Network status too large");
  }
  const esp_err_t error =
      send(request, "200 OK", "application/json; charset=utf-8", response_,
           length);
  giveWorkspace();
  return error;
}

esp_err_t HttpService::saveWifi(httpd_req_t *request) {
  if (!callbacks_.saveStaCredentials || !callbacks_.requestRestart) {
    return sendJsonError(request, "503 Service Unavailable",
                         "Network configuration unavailable");
  }
  const size_t queryLength = httpd_req_get_url_query_len(request);
  if (queryLength == 0 || queryLength >= sizeof(query_)) {
    return sendJsonError(request, "400 Bad Request",
                         "Missing or oversized WiFi parameters");
  }
  if (!takeWorkspace()) {
    return sendJsonError(request, "503 Service Unavailable",
                         "Partner service is busy");
  }

  std::array<char, 193> encodedSsid = {};
  std::array<char, 385> encodedPassword = {};
  std::array<char, 33> ssid = {};
  std::array<char, 65> password = {};
  const bool valid =
      httpd_req_get_url_query_str(request, query_, sizeof(query_)) == ESP_OK &&
      httpd_query_key_value(query_, "ssid", encodedSsid.data(),
                            encodedSsid.size()) == ESP_OK &&
      httpd_query_key_value(query_, "pass", encodedPassword.data(),
                            encodedPassword.size()) == ESP_OK &&
      decodeComponent(encodedSsid.data(), ssid.data(), ssid.size()) &&
      decodeComponent(encodedPassword.data(), password.data(),
                      password.size()) &&
      ssid[0] != '\0' &&
      (password[0] == '\0' || std::strlen(password.data()) >= 8);
  if (!valid) {
    std::fill(password.begin(), password.end(), '\0');
    giveWorkspace();
    return sendJsonError(request, "400 Bad Request",
                         "Invalid SSID or password");
  }

  const bool saved = callbacks_.saveStaCredentials(
      callbacks_.context, ssid.data(), password.data());
  std::fill(password.begin(), password.end(), '\0');
  std::fill(encodedPassword.begin(), encodedPassword.end(), '\0');
  giveWorkspace();
  if (!saved) {
    return sendJsonError(request, "500 Internal Server Error",
                         "WiFi save failed");
  }

  const esp_err_t response = send(request, "200 OK", "text/plain",
                                  "WiFi Saved. Rebooting...");
  ESP_LOGI(kTag, "Saved STA credentials; restart requested");
  callbacks_.requestRestart(callbacks_.context);
  return response;
}

}  // namespace robofish::partner
