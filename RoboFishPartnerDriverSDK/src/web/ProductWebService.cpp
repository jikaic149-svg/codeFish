#include "ProductWebService.h"

#include <array>
#include <cerrno>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <cmath>

#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"
#include "config/sdk_config.h"
#include "web/WebAssets.h"

namespace {

constexpr char kTag[] = "product_web";
constexpr char kDriveNamespace[] = "drive_cfg";
constexpr char kTailReverseKey[] = "tail_rev";

bool saveTailReversed(bool reversed) {
  nvs_handle_t handle = 0;
  esp_err_t error = nvs_open(kDriveNamespace, NVS_READWRITE, &handle);
  if (error == ESP_OK) error = nvs_set_u8(handle, kTailReverseKey, reversed);
  if (error == ESP_OK) error = nvs_commit(handle);
  if (handle) nvs_close(handle);
  return error == ESP_OK;
}

ProductWebService *serviceFrom(httpd_req_t *request) {
  return request ? static_cast<ProductWebService *>(request->user_ctx)
                 : nullptr;
}

esp_err_t send(httpd_req_t *request, const char *status,
               const char *contentType, const char *body) {
  httpd_resp_set_status(request, status);
  httpd_resp_set_type(request, contentType);
  httpd_resp_set_hdr(request, "Cache-Control", "no-store");
  return httpd_resp_send(request, body, HTTPD_RESP_USE_STRLEN);
}

esp_err_t sendJsonError(httpd_req_t *request, const char *status,
                        const char *message) {
  std::array<char, 224> response = {};
  std::snprintf(response.data(), response.size(),
                "{\"success\":false,\"error\":\"%s\"}", message);
  return send(request, status, "application/json; charset=utf-8",
              response.data());
}

uint32_t nowMs() {
  return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
}

}  // namespace

ProductWebService::ProductWebService(
    MotionController &motion, robofish::partner::ScriptEngine &partner,
    CameraService &camera)
    : motion_(motion), partner_(partner), camera_(camera) {
  camera_.setControlFallback(controlFallback, this);
}

esp_err_t ProductWebService::registerRoutes(httpd_handle_t server) {
  if (!server) return ESP_ERR_INVALID_ARG;
  const httpd_uri_t routes[] = {
      {.uri = "/", .method = HTTP_GET, .handler = rootHandler, .user_ctx = this},
      {.uri = "/partner_script", .method = HTTP_GET, .handler = partnerScriptGetHandler, .user_ctx = this},
      {.uri = "/partner_script", .method = HTTP_POST, .handler = partnerScriptPostHandler, .user_ctx = this},
      {.uri = "/partner_run", .method = HTTP_POST, .handler = partnerRunHandler, .user_ctx = this},
      {.uri = "/partner_stop", .method = HTTP_POST, .handler = partnerStopHandler, .user_ctx = this},
      {.uri = "/partner_status", .method = HTTP_GET, .handler = partnerStatusHandler, .user_ctx = this},
      {.uri = "/vision_control", .method = HTTP_GET, .handler = visionControlHandler, .user_ctx = this},
      {.uri = "/vision_pid", .method = HTTP_GET, .handler = visionPidHandler, .user_ctx = this},
      {.uri = "/generate_204", .method = HTTP_GET, .handler = captivePortalHandler, .user_ctx = this},
      {.uri = "/gen_204", .method = HTTP_GET, .handler = captivePortalHandler, .user_ctx = this},
      {.uri = "/hotspot-detect.html", .method = HTTP_GET, .handler = captivePortalHandler, .user_ctx = this},
      {.uri = "/ncsi.txt", .method = HTTP_GET, .handler = captivePortalHandler, .user_ctx = this},
      {.uri = "/connecttest.txt", .method = HTTP_GET, .handler = captivePortalHandler, .user_ctx = this},
      {.uri = "/fwlink", .method = HTTP_GET, .handler = captivePortalHandler, .user_ctx = this},
  };
  for (const httpd_uri_t &route : routes) {
    const esp_err_t error = httpd_register_uri_handler(server, &route);
    if (error != ESP_OK) return error;
  }
  return ESP_OK;
}

esp_err_t ProductWebService::rootHandler(httpd_req_t *request) {
  httpd_resp_set_status(request, "302 Found");
  httpd_resp_set_hdr(request, "Location", "/gpio");
  return httpd_resp_send(request, nullptr, 0);
}

esp_err_t ProductWebService::controlFallback(httpd_req_t *request,
                                             const char *variable, int value,
                                             void *context) {
  ProductWebService *service = static_cast<ProductWebService *>(context);
  return service ? service->dispatchMotionControl(request, variable, value)
                 : ESP_ERR_INVALID_ARG;
}

esp_err_t ProductWebService::dispatchMotionControl(httpd_req_t *request,
                                                   const char *variable,
                                                   int value) {
  static constexpr const char *kMotionVariables[] = {
      "servo_forward", "servo_backward", "servo_left", "servo_right",
      "servo_stop", "pitch", "pump", "mode", "demo",
      "tail_reverse",
  };
  bool recognized = false;
  for (const char *known : kMotionVariables) {
    if (std::strcmp(variable, known) == 0) {
      recognized = true;
      break;
    }
  }
  if (!recognized) {
    return sendJsonError(request, "404 Not Found", "Unknown control variable");
  }

  // Match the recovered contract: a valid manual command stops automation
  // before it changes actuator state.
  stopPartnerForManualControl();

  bool accepted = false;
  const char *response = "";
  MotionStatus motionStatus = {};
  const bool hasMotionStatus = motion_.getStatus(motionStatus);
  const bool turningCommand = std::strcmp(variable, "servo_left") == 0 ||
                              std::strcmp(variable, "servo_right") == 0;
  const uint8_t manualPower =
      hasMotionStatus && motionStatus.mode == DriveMode::Prop
          ? (turningCommand ? RF_MANUAL_PROP_TURN_POWER_PERCENT
                            : RF_MANUAL_PROP_POWER_PERCENT)
          : RF_MANUAL_TAIL_POWER_PERCENT;
  if (std::strcmp(variable, "servo_forward") == 0) {
    accepted = motion_.forward(manualPower);
  } else if (std::strcmp(variable, "servo_backward") == 0) {
    accepted = motion_.backward(manualPower);
  } else if (std::strcmp(variable, "servo_left") == 0) {
    accepted = motion_.left(manualPower);
  } else if (std::strcmp(variable, "servo_right") == 0) {
    accepted = motion_.right(manualPower);
  } else if (std::strcmp(variable, "servo_stop") == 0) {
    accepted = motion_.stop();
  } else if (std::strcmp(variable, "pitch") == 0) {
    accepted = value >= 45 && value <= 135 && motion_.setPitch(value);
  } else if (std::strcmp(variable, "pump") == 0) {
    accepted = (value == 0 || value == 1) && motion_.setPump(value != 0);
  } else if (std::strcmp(variable, "mode") == 0) {
    DriveMode mode = DriveMode::Bionic;
    accepted = motion_.cycleMode(&mode);
    response = accepted ? MotionController::modeName(mode) : "";
  } else if (std::strcmp(variable, "demo") == 0) {
    const DemoRequestResult result = motion_.requestDemo();
    accepted = result != DemoRequestResult::Unavailable;
    response = MotionController::demoResultText(result);
  } else if (std::strcmp(variable, "tail_reverse") == 0) {
    const bool reversed = value < 0 ? !motion_.tailReversed() : value != 0;
    accepted = saveTailReversed(reversed) &&
               motion_.setTailReversed(reversed);
    response = accepted ? (reversed ? "TAIL_REVERSED" : "TAIL_NORMAL") : "";
  }

  if (!accepted) {
    return sendJsonError(request, "409 Conflict", "Control command rejected");
  }
  return send(request, "200 OK", "text/plain; charset=utf-8", response);
}

void ProductWebService::stopPartnerForManualControl() {
  if (partner_.running()) partner_.stop();
}

esp_err_t ProductWebService::visionControlHandler(httpd_req_t *request) {
  ProductWebService *service = serviceFrom(request);
  return service ? service->applyVisionControl(request) : ESP_ERR_INVALID_ARG;
}

esp_err_t ProductWebService::visionPidHandler(httpd_req_t *request) {
  ProductWebService *service = serviceFrom(request);
  return service ? service->applyVisionPid(request) : ESP_ERR_INVALID_ARG;
}

esp_err_t ProductWebService::applyVisionPid(httpd_req_t *request) {
  const size_t queryLength = httpd_req_get_url_query_len(request);
  if (queryLength == 0 || queryLength > 127) {
    return sendJsonError(request, "400 Bad Request", "Invalid query length");
  }

  std::array<char, 128> query = {};
  std::array<char, 16> sequenceText = {};
  std::array<char, 24> xText = {};
  std::array<char, 24> yText = {};
  if (httpd_req_get_url_query_str(request, query.data(), query.size()) !=
          ESP_OK ||
      httpd_query_key_value(query.data(), "seq", sequenceText.data(),
                            sequenceText.size()) != ESP_OK ||
      httpd_query_key_value(query.data(), "x_error", xText.data(),
                            xText.size()) != ESP_OK ||
      httpd_query_key_value(query.data(), "y_error", yText.data(),
                            yText.size()) != ESP_OK) {
    return sendJsonError(request, "400 Bad Request",
                         "Required: seq, x_error, y_error");
  }

  auto parseSequence = [](const char *text, uint32_t *value) -> bool {
    if (!text || !text[0] || !value) return false;
    errno = 0;
    char *end = nullptr;
    const unsigned long parsed = std::strtoul(text, &end, 10);
    if (errno != 0 || !end || *end != '\0' || parsed > 0x7fffffffu) {
      return false;
    }
    *value = static_cast<uint32_t>(parsed);
    return true;
  };
  auto parseFloat = [](const char *text, float *value) -> bool {
    if (!text || !text[0] || !value) return false;
    errno = 0;
    char *end = nullptr;
    const float parsed = std::strtof(text, &end);
    if (errno != 0 || !end || *end != '\0' || !std::isfinite(parsed)) {
      return false;
    }
    *value = parsed;
    return true;
  };

  uint32_t sequence = 0;
  float xError = 0.0f;
  float yError = 0.0f;
  if (!parseSequence(sequenceText.data(), &sequence) ||
      !parseFloat(xText.data(), &xError) || !parseFloat(yText.data(), &yError) ||
      std::abs(xError) > 5.0f || yError < 0.0f || yError > 5.0f) {
    return sendJsonError(request, "400 Bad Request",
                         "x_error must be +/-5 m; y_error must be 0..5 m");
  }

  stopPartnerForManualControl();
  const VisionControlResult result =
      motion_.applyVisionPid(sequence, xError, yError);
  if (result == VisionControlResult::StaleSequence) {
    return sendJsonError(request, "409 Conflict", "Stale sequence");
  }
  if (result != VisionControlResult::Accepted) {
    return sendJsonError(request, "409 Conflict", "Control command rejected");
  }

  std::array<char, 192> response = {};
  std::snprintf(response.data(), response.size(),
                "{\"ok\":true,\"seq\":%lu,\"x_error\":%.4f,"
                "\"y_error\":%.4f,\"pid\":\"mcu_xy\"}",
                static_cast<unsigned long>(sequence), xError, yError);
  return send(request, "200 OK", "application/json; charset=utf-8",
              response.data());
}

esp_err_t ProductWebService::applyVisionControl(httpd_req_t *request) {
  const size_t queryLength = httpd_req_get_url_query_len(request);
  if (queryLength == 0 || queryLength > 127) {
    return sendJsonError(request, "400 Bad Request", "Invalid query length");
  }

  std::array<char, 128> query = {};
  std::array<char, 16> sequenceText = {};
  std::array<char, 8> powerText = {};
  std::array<char, 8> steeringText = {};
  if (httpd_req_get_url_query_str(request, query.data(), query.size()) !=
          ESP_OK ||
      httpd_query_key_value(query.data(), "seq", sequenceText.data(),
                            sequenceText.size()) != ESP_OK ||
      httpd_query_key_value(query.data(), "power", powerText.data(),
                            powerText.size()) != ESP_OK ||
      httpd_query_key_value(query.data(), "steering", steeringText.data(),
                            steeringText.size()) != ESP_OK) {
    return sendJsonError(request, "400 Bad Request",
                         "Required: seq, power, steering");
  }

  auto parseInteger = [](const char *text, long minimum, long maximum,
                         long *value) -> bool {
    if (!text || !text[0] || !value) return false;
    errno = 0;
    char *end = nullptr;
    const long parsed = std::strtol(text, &end, 10);
    if (errno != 0 || !end || *end != '\0' || parsed < minimum ||
        parsed > maximum) {
      return false;
    }
    *value = parsed;
    return true;
  };

  long sequence = 0;
  long power = 0;
  long steering = 0;
  if (!parseInteger(sequenceText.data(), 0, 2147483647L, &sequence) ||
      !parseInteger(powerText.data(), 0, 70, &power) ||
      !parseInteger(steeringText.data(), -70, 70, &steering)) {
    return sendJsonError(request, "400 Bad Request",
                         "seq=0..2147483647, power=0..70, steering=-70..70");
  }

  stopPartnerForManualControl();
  const VisionControlResult result = motion_.applyVisionControl(
      static_cast<uint32_t>(sequence), static_cast<uint8_t>(power),
      static_cast<int>(steering));
  if (result == VisionControlResult::StaleSequence) {
    return sendJsonError(request, "409 Conflict", "Stale sequence");
  }
  if (result != VisionControlResult::Accepted) {
    return sendJsonError(request, "409 Conflict", "Control command rejected");
  }

  std::array<char, 160> response = {};
  std::snprintf(response.data(), response.size(),
                "{\"ok\":true,\"seq\":%ld,\"mode\":\"PROP\","
                "\"power\":%ld,\"steering\":%ld}",
                sequence, power, steering);
  return send(request, "200 OK", "application/json; charset=utf-8",
              response.data());
}

esp_err_t ProductWebService::partnerScriptGetHandler(httpd_req_t *request) {
  ProductWebService *service = serviceFrom(request);
  return service ? service->sendPartnerScript(request) : ESP_ERR_INVALID_ARG;
}

esp_err_t ProductWebService::sendPartnerScript(httpd_req_t *request) {
  scriptBuffer_.fill('\0');
  size_t length = 0;
  const auto result = partner_.copyJson(scriptBuffer_.data(),
                                        scriptBuffer_.size(), &length);
  if (result != robofish::partner::Result::kOk) {
    return sendJsonError(request, "500 Internal Server Error",
                         robofish::partner::resultText(result));
  }
  httpd_resp_set_type(request, "application/json");
  httpd_resp_set_hdr(request, "Cache-Control", "no-store");
  return httpd_resp_send(request, scriptBuffer_.data(),
                         static_cast<ssize_t>(length));
}

esp_err_t ProductWebService::partnerScriptPostHandler(httpd_req_t *request) {
  ProductWebService *service = serviceFrom(request);
  return service ? service->savePartnerScript(request) : ESP_ERR_INVALID_ARG;
}

esp_err_t ProductWebService::savePartnerScript(httpd_req_t *request) {
  if (request->content_len < 1 ||
      request->content_len > robofish::partner::kMaxJsonBytes) {
    return sendJsonError(request, "400 Bad Request",
                         "Script must be 1..3500 bytes");
  }
  scriptBuffer_.fill('\0');
  size_t received = 0;
  while (received < request->content_len) {
    const int count = httpd_req_recv(request, scriptBuffer_.data() + received,
                                     request->content_len - received);
    if (count <= 0) {
      return sendJsonError(request, "400 Bad Request",
                           "Incomplete request body");
    }
    received += static_cast<size_t>(count);
  }
  std::array<char, 192> error = {};
  const auto result = partner_.validateAndSaveJson(
      scriptBuffer_.data(), received, error.data(), error.size());
  if (result != robofish::partner::Result::kOk) {
    return sendJsonError(request, "400 Bad Request",
                         error[0] ? error.data()
                                  : robofish::partner::resultText(result));
  }
  return send(request, "200 OK", "application/json; charset=utf-8",
              "{\"success\":true,\"message\":\"Script validated and saved\"}");
}

esp_err_t ProductWebService::partnerRunHandler(httpd_req_t *request) {
  ProductWebService *service = serviceFrom(request);
  return service ? service->runPartner(request) : ESP_ERR_INVALID_ARG;
}

esp_err_t ProductWebService::runPartner(httpd_req_t *request) {
  std::array<char, 192> error = {};
  const auto result = partner_.start(error.data(), error.size());
  if (result != robofish::partner::Result::kOk) {
    return sendJsonError(request, "409 Conflict",
                         error[0] ? error.data()
                                  : robofish::partner::resultText(result));
  }
  const auto status = partner_.status(nowMs());
  std::array<char, 128> response = {};
  const int length = std::snprintf(
      response.data(), response.size(),
      "{\"success\":true,\"message\":\"Started: %s\"}", status.name);
  if (length <= 0 || static_cast<size_t>(length) >= response.size()) {
    return sendJsonError(request, "500 Internal Server Error",
                         "Partner response too large");
  }
  return send(request, "200 OK", "application/json; charset=utf-8",
              response.data());
}

esp_err_t ProductWebService::partnerStopHandler(httpd_req_t *request) {
  ProductWebService *service = serviceFrom(request);
  return service ? service->stopPartner(request) : ESP_ERR_INVALID_ARG;
}

esp_err_t ProductWebService::stopPartner(httpd_req_t *request) {
  partner_.stop();
  return send(request, "200 OK", "application/json; charset=utf-8",
              "{\"success\":true,\"message\":\"Script safely stopped\"}");
}

esp_err_t ProductWebService::partnerStatusHandler(httpd_req_t *request) {
  ProductWebService *service = serviceFrom(request);
  return service ? service->sendPartnerStatus(request) : ESP_ERR_INVALID_ARG;
}

esp_err_t ProductWebService::sendPartnerStatus(httpd_req_t *request) {
  std::array<char, 320> status = {};
  size_t length = 0;
  const auto result =
      partner_.copyStatusJson(nowMs(), status.data(), status.size(), &length);
  if (result != robofish::partner::Result::kOk) {
    return sendJsonError(request, "500 Internal Server Error",
                         robofish::partner::resultText(result));
  }
  httpd_resp_set_type(request, "application/json");
  httpd_resp_set_hdr(request, "Cache-Control", "no-store");
  return httpd_resp_send(request, status.data(), static_cast<ssize_t>(length));
}

esp_err_t ProductWebService::captivePortalHandler(httpd_req_t *request) {
  return rootHandler(request);
}
