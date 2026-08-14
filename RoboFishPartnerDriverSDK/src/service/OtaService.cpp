#include "OtaService.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <cstdio>
#include <cstring>

#include "../config/sdk_config.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mbedtls/base64.h"
#include "nvs.h"
#include "nvs_flash.h"

namespace {

constexpr char kTag[] = "ota";
constexpr size_t kReceiveBufferBytes = 4096;
constexpr size_t kMaxMultipartHeaderBytes = 2048;
constexpr size_t kMaxBoundaryBytes = 128;
constexpr size_t kMultipartMarkerBytes = kMaxBoundaryBytes + 9;
constexpr size_t kOtaPrefixBytes =
    kMaxMultipartHeaderBytes + kReceiveBufferBytes;

struct OtaWorkspace {
  std::array<char, 513> authorization;
  std::array<unsigned char, 384> decodedAuthorization;
  std::array<char, 256> contentType;
  std::array<char, kOtaPrefixBytes> data;
  std::array<char, kMultipartMarkerBytes> firstLine;
  std::array<char, kMultipartMarkerBytes> footer;
  std::array<char, kMultipartMarkerBytes> receivedFooter;
  std::array<char, kMaxBoundaryBytes + 1> boundary;
};

httpd_handle_t server = nullptr;
esp_netif_t *apNetif = nullptr;
esp_netif_t *staNetif = nullptr;
OtaServiceConfig serviceConfig = {};
OtaStartCallback otaStartCallback = nullptr;
std::atomic_bool staConnected{false};
std::atomic_uint_fast8_t apClientCount{0};
std::atomic_bool nvsInitialized{false};
OtaWorkspace otaWorkspace = {};

constexpr char kWifiNamespace[] = "wifi_config";
constexpr char kWifiSsidKey[] = "ssid";
constexpr char kWifiPasswordKey[] = "pass";
std::array<char, 33> savedStaSsid = {};
std::array<char, 65> savedStaPassword = {};

constexpr char kUploadPage[] = R"HTML(
<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>RoboFish OTA</title>
<style>body{max-width:620px;margin:40px auto;padding:0 18px;background:#07151d;color:#eaf7fa;font-family:system-ui}form{background:#102630;border:1px solid #2b5364;border-radius:14px;padding:24px}input,button{width:100%;margin-top:14px;padding:12px}button{background:#39d2a0;border:0;border-radius:8px;font-weight:700}</style></head>
<body><form method="POST" action="/update" enctype="multipart/form-data"><h2>RoboFish Web OTA</h2><p>请选择 ESP-IDF 生成的应用固件 <code>.bin</code>。上传开始时所有执行器会停止并锁定。</p><input type="file" name="firmware" accept=".bin" required><button type="submit">上传并重启</button></form></body></html>
)HTML";

esp_err_t sendText(httpd_req_t *request, const char *status,
                   const char *contentType, const char *body) {
  httpd_resp_set_status(request, status);
  httpd_resp_set_type(request, contentType);
  return httpd_resp_send(request, body, HTTPD_RESP_USE_STRLEN);
}

esp_err_t sendOtaFailure(httpd_req_t *request) {
  return sendText(request, "500 Internal Server Error",
                  "text/plain; charset=utf-8",
                  "OTA失败，请查看串口日志后重试。");
}

bool equalsIgnoreCase(const char *left, const char *right, size_t count) {
  for (size_t i = 0; i < count; ++i) {
    if (std::tolower(static_cast<unsigned char>(left[i])) !=
        std::tolower(static_cast<unsigned char>(right[i]))) {
      return false;
    }
  }
  return true;
}

bool authenticated(httpd_req_t *request) {
  constexpr char kPrefix[] = "Basic ";
  const size_t headerLength =
      httpd_req_get_hdr_value_len(request, "Authorization");
  if (headerLength < sizeof(kPrefix) - 1 || headerLength > 512) return false;

  std::fill(otaWorkspace.authorization.begin(),
            otaWorkspace.authorization.end(), '\0');
  if (httpd_req_get_hdr_value_str(request, "Authorization",
                                  otaWorkspace.authorization.data(),
                                  otaWorkspace.authorization.size()) !=
      ESP_OK) {
    return false;
  }
  if (!equalsIgnoreCase(otaWorkspace.authorization.data(), kPrefix,
                        sizeof(kPrefix) - 1)) {
    return false;
  }

  const char *encoded =
      otaWorkspace.authorization.data() + sizeof(kPrefix) - 1;
  const size_t encodedLength = std::strlen(encoded);
  std::fill(otaWorkspace.decodedAuthorization.begin(),
            otaWorkspace.decodedAuthorization.end(), 0);
  size_t decodedLength = 0;
  if (mbedtls_base64_decode(
          otaWorkspace.decodedAuthorization.data(),
          otaWorkspace.decodedAuthorization.size() - 1, &decodedLength,
          reinterpret_cast<const unsigned char *>(encoded), encodedLength) !=
      0) {
    std::fill(otaWorkspace.authorization.begin(),
              otaWorkspace.authorization.end(), '\0');
    std::fill(otaWorkspace.decodedAuthorization.begin(),
              otaWorkspace.decodedAuthorization.end(), 0);
    return false;
  }

  const size_t usernameLength = std::strlen(serviceConfig.username);
  const size_t passwordLength = std::strlen(serviceConfig.password);
  const bool matches =
      decodedLength == usernameLength + 1 + passwordLength &&
      std::memcmp(otaWorkspace.decodedAuthorization.data(),
                  serviceConfig.username, usernameLength) == 0 &&
      otaWorkspace.decodedAuthorization[usernameLength] == ':' &&
      std::memcmp(otaWorkspace.decodedAuthorization.data() + usernameLength +
                      1,
                  serviceConfig.password, passwordLength) == 0;
  std::fill(otaWorkspace.authorization.begin(),
            otaWorkspace.authorization.end(), '\0');
  std::fill(otaWorkspace.decodedAuthorization.begin(),
            otaWorkspace.decodedAuthorization.end(), 0);
  return matches;
}

esp_err_t requireAuthentication(httpd_req_t *request) {
  if (authenticated(request)) return ESP_OK;
  httpd_resp_set_status(request, "401 Unauthorized");
  httpd_resp_set_type(request, "text/plain; charset=utf-8");
  httpd_resp_set_hdr(request, "WWW-Authenticate",
                     "Basic realm=\"RoboFish OTA\"");
  httpd_resp_send(request, "Authentication required.", HTTPD_RESP_USE_STRLEN);
  return ESP_ERR_INVALID_STATE;
}

size_t extractBoundary(const char *contentType, char *boundary,
                       size_t boundaryCapacity) {
  constexpr char kBoundaryKey[] = "boundary=";
  const char *cursor = contentType;
  while (*cursor) {
    const size_t remaining = std::strlen(cursor);
    if (remaining >= sizeof(kBoundaryKey) - 1 &&
        equalsIgnoreCase(cursor, kBoundaryKey,
                         sizeof(kBoundaryKey) - 1)) {
      cursor += sizeof(kBoundaryKey) - 1;
      bool quoted = *cursor == '"';
      if (quoted) ++cursor;
      const char *end = cursor;
      while (*end && ((quoted && *end != '"') ||
                      (!quoted && *end != ';' &&
                       !std::isspace(static_cast<unsigned char>(*end))))) {
        ++end;
      }
      const size_t length = static_cast<size_t>(end - cursor);
      if (length > 0 && length < boundaryCapacity) {
        std::memcpy(boundary, cursor, length);
        boundary[length] = '\0';
        return length;
      }
      return 0;
    }
    ++cursor;
  }
  return 0;
}

bool containsBytes(const char *data, size_t dataLength, const char *needle) {
  const size_t needleLength = std::strlen(needle);
  return needleLength <= dataLength &&
         std::search(data, data + dataLength, needle,
                     needle + needleLength) != data + dataLength;
}

bool containsFirmwareDisposition(const char *header, size_t headerLength) {
  if (!containsBytes(header, headerLength, "Content-Disposition:") &&
      !containsBytes(header, headerLength, "content-disposition:")) {
    return false;
  }
  return containsBytes(header, headerLength, "name=\"firmware\"") ||
         containsBytes(header, headerLength, "name=firmware");
}

int receiveBody(httpd_req_t *request, char *buffer, size_t length) {
  int received = httpd_req_recv(request, buffer, length);
  if (received == HTTPD_SOCK_ERR_TIMEOUT) {
    // The HTTP server already enforces a finite receive timeout. Do not retry.
    ESP_LOGE(kTag, "OTA receive timeout");
  }
  return received;
}

esp_err_t getIpHandler(httpd_req_t *request) {
  esp_netif_ip_info_t apInfo = {};
  esp_netif_ip_info_t staInfo = {};
  if (apNetif) esp_netif_get_ip_info(apNetif, &apInfo);
  if (staNetif) esp_netif_get_ip_info(staNetif, &staInfo);

  std::array<char, 384> response = {};
  std::snprintf(
      response.data(), response.size(),
      "{\"sta_connected\":%d,\"sta_ip\":\"" IPSTR
      "\",\"sta_ssid\":\"%s\",\"ap_ip\":\"" IPSTR
      "\",\"version\":\"4.0.0-partner\"}",
      staConnected.load(std::memory_order_relaxed) ? 1 : 0,
      IP2STR(&staInfo.ip), savedStaSsid.data(), IP2STR(&apInfo.ip));
  return sendText(request, "200 OK", "application/json", response.data());
}

bool decodeUrlComponent(const char *source, char *destination,
                        size_t capacity) {
  if (!source || !destination || capacity == 0) return false;
  size_t output = 0;
  for (size_t input = 0; source[input] != '\0'; ++input) {
    unsigned char value = static_cast<unsigned char>(source[input]);
    if (value == '+') {
      value = ' ';
    } else if (value == '%') {
      const char high = source[++input];
      const char low = high == '\0' ? '\0' : source[++input];
      auto hex = [](char digit) -> int {
        if (digit >= '0' && digit <= '9') return digit - '0';
        if (digit >= 'a' && digit <= 'f') return digit - 'a' + 10;
        if (digit >= 'A' && digit <= 'F') return digit - 'A' + 10;
        return -1;
      };
      const int highValue = hex(high);
      const int lowValue = hex(low);
      if (highValue < 0 || lowValue < 0) return false;
      value = static_cast<unsigned char>((highValue << 4) | lowValue);
      if (value == 0) return false;
    }
    if (output + 1 >= capacity) return false;
    destination[output++] = static_cast<char>(value);
  }
  destination[output] = '\0';
  return true;
}

esp_err_t saveWifiHandler(httpd_req_t *request) {
  const size_t queryLength = httpd_req_get_url_query_len(request);
  if (queryLength == 0 || queryLength > 512) {
    return sendText(request, "400 Bad Request", "text/plain",
                    "Missing or oversized WiFi parameters");
  }

  std::array<char, 513> query = {};
  std::array<char, 193> encodedSsid = {};
  std::array<char, 385> encodedPassword = {};
  std::array<char, 33> ssid = {};
  std::array<char, 65> password = {};
  if (httpd_req_get_url_query_str(request, query.data(), query.size()) !=
          ESP_OK ||
      httpd_query_key_value(query.data(), "ssid", encodedSsid.data(),
                            encodedSsid.size()) != ESP_OK ||
      httpd_query_key_value(query.data(), "pass", encodedPassword.data(),
                            encodedPassword.size()) != ESP_OK ||
      !decodeUrlComponent(encodedSsid.data(), ssid.data(), ssid.size()) ||
      !decodeUrlComponent(encodedPassword.data(), password.data(),
                          password.size()) ||
      ssid[0] == '\0') {
    return sendText(request, "400 Bad Request", "text/plain",
                    "Invalid SSID or password");
  }

  nvs_handle_t nvs = 0;
  esp_err_t error = nvs_open(kWifiNamespace, NVS_READWRITE, &nvs);
  if (error == ESP_OK) error = nvs_set_str(nvs, kWifiSsidKey, ssid.data());
  if (error == ESP_OK) {
    error = nvs_set_str(nvs, kWifiPasswordKey, password.data());
  }
  if (error == ESP_OK) error = nvs_commit(nvs);
  if (nvs) nvs_close(nvs);
  std::fill(password.begin(), password.end(), '\0');
  std::fill(encodedPassword.begin(), encodedPassword.end(), '\0');
  if (error != ESP_OK) {
    ESP_LOGE(kTag, "Saving STA credentials failed: %s",
             esp_err_to_name(error));
    return sendText(request, "500 Internal Server Error", "text/plain",
                    "WiFi save failed");
  }

  const esp_err_t response =
      sendText(request, "200 OK", "text/plain", "WiFi Saved. Rebooting...");
  vTaskDelay(pdMS_TO_TICKS(500));
  esp_restart();
  return response;
}

esp_err_t updatePageHandler(httpd_req_t *request) {
  if (requireAuthentication(request) != ESP_OK) return ESP_OK;
  return sendText(request, "200 OK", "text/html; charset=utf-8",
                  kUploadPage);
}

esp_err_t updateHandler(httpd_req_t *request) {
  if (requireAuthentication(request) != ESP_OK) return ESP_OK;

  std::fill(otaWorkspace.contentType.begin(), otaWorkspace.contentType.end(),
            '\0');
  if (httpd_req_get_hdr_value_str(request, "Content-Type",
                                  otaWorkspace.contentType.data(),
                                  otaWorkspace.contentType.size()) != ESP_OK ||
      !equalsIgnoreCase(otaWorkspace.contentType.data(),
                        "multipart/form-data", 19)) {
    ESP_LOGE(kTag, "OTA request is not multipart/form-data");
    return sendOtaFailure(request);
  }

  const size_t boundaryLength =
      extractBoundary(otaWorkspace.contentType.data(),
                      otaWorkspace.boundary.data(),
                      otaWorkspace.boundary.size());
  if (boundaryLength == 0 || request->content_len == 0) {
    ESP_LOGE(kTag, "Invalid multipart boundary or length");
    return sendOtaFailure(request);
  }

  const int firstLineLength =
      std::snprintf(otaWorkspace.firstLine.data(),
                    otaWorkspace.firstLine.size(), "--%s\r\n",
                    otaWorkspace.boundary.data());
  const int footerLength =
      std::snprintf(otaWorkspace.footer.data(), otaWorkspace.footer.size(),
                    "\r\n--%s--\r\n", otaWorkspace.boundary.data());
  if (firstLineLength <= 0 || footerLength <= 0 ||
      static_cast<size_t>(firstLineLength) >= otaWorkspace.firstLine.size() ||
      static_cast<size_t>(footerLength) >= otaWorkspace.footer.size()) {
    ESP_LOGE(kTag, "Multipart markers are too long");
    return sendOtaFailure(request);
  }

  size_t prefixLength = 0;
  size_t consumed = 0;
  size_t payloadOffset = kOtaPrefixBytes;

  while (consumed < request->content_len &&
         prefixLength <= kMaxMultipartHeaderBytes) {
    const size_t wanted =
        std::min({kReceiveBufferBytes, request->content_len - consumed,
                  otaWorkspace.data.size() - prefixLength});
    if (wanted == 0) break;
    const int count = receiveBody(request, otaWorkspace.data.data() +
                                               prefixLength,
                                  wanted);
    if (count <= 0) {
      ESP_LOGE(kTag, "Multipart header receive failed");
      return sendOtaFailure(request);
    }
    prefixLength += static_cast<size_t>(count);
    consumed += static_cast<size_t>(count);

    constexpr char kHeaderEnd[] = "\r\n\r\n";
    const char *end = std::search(otaWorkspace.data.data(),
                                  otaWorkspace.data.data() + prefixLength,
                                  kHeaderEnd,
                                  kHeaderEnd + sizeof(kHeaderEnd) - 1);
    if (end != otaWorkspace.data.data() + prefixLength) {
      payloadOffset = static_cast<size_t>(end - otaWorkspace.data.data()) +
                      sizeof(kHeaderEnd) - 1;
      break;
    }
  }

  if (payloadOffset == kOtaPrefixBytes ||
      payloadOffset > kMaxMultipartHeaderBytes ||
      prefixLength < static_cast<size_t>(firstLineLength) ||
      std::memcmp(otaWorkspace.data.data(), otaWorkspace.firstLine.data(),
                  static_cast<size_t>(firstLineLength)) != 0) {
    ESP_LOGE(kTag, "Malformed multipart preamble");
    return sendOtaFailure(request);
  }

  if (!containsFirmwareDisposition(
          otaWorkspace.data.data() + firstLineLength,
          payloadOffset - static_cast<size_t>(firstLineLength))) {
    ESP_LOGE(kTag, "Multipart field 'firmware' was not found");
    return sendOtaFailure(request);
  }

  if (request->content_len <
      payloadOffset + static_cast<size_t>(footerLength)) {
    ESP_LOGE(kTag, "Multipart body is shorter than its envelope");
    return sendOtaFailure(request);
  }
  const size_t firmwareLength =
      request->content_len - payloadOffset - static_cast<size_t>(footerLength);
  if (firmwareLength == 0 || firmwareLength > RF_OTA_MAX_IMAGE_BYTES) {
    ESP_LOGE(kTag, "OTA image size is invalid: %u",
             static_cast<unsigned>(firmwareLength));
    return sendOtaFailure(request);
  }

  const esp_partition_t *partition = esp_ota_get_next_update_partition(nullptr);
  if (!partition || partition->type != ESP_PARTITION_TYPE_APP ||
      partition->size < firmwareLength) {
    ESP_LOGE(kTag, "No suitable OTA partition");
    return sendOtaFailure(request);
  }

  if (otaStartCallback && !otaStartCallback()) {
    ESP_LOGE(kTag, "Failed to stop and lock actuators for OTA");
    return sendOtaFailure(request);
  }
  ESP_LOGI(kTag, "Upload start: %u bytes -> %s",
           static_cast<unsigned>(firmwareLength), partition->label);

  esp_ota_handle_t otaHandle = 0;
  bool otaBegun = false;
  bool otaEnded = false;
  esp_err_t error = esp_ota_begin(partition, firmwareLength, &otaHandle);
  if (error == ESP_OK) otaBegun = true;

  size_t written = 0;
  if (error == ESP_OK) {
    const size_t buffered = prefixLength - payloadOffset;
    const size_t payloadBytes = std::min(buffered, firmwareLength);
    if (payloadBytes > 0) {
      error = esp_ota_write(otaHandle,
                            otaWorkspace.data.data() + payloadOffset,
                            payloadBytes);
      if (error == ESP_OK) written += payloadBytes;
    }
  }

  while (error == ESP_OK && written < firmwareLength) {
    const size_t wanted =
        std::min(kReceiveBufferBytes, firmwareLength - written);
    const int count = receiveBody(request, otaWorkspace.data.data(), wanted);
    if (count <= 0) {
      error = ESP_FAIL;
      break;
    }
    consumed += static_cast<size_t>(count);
    error = esp_ota_write(otaHandle, otaWorkspace.data.data(),
                          static_cast<size_t>(count));
    if (error == ESP_OK) written += static_cast<size_t>(count);
  }

  size_t receivedFooterLength = 0;
  if (error == ESP_OK) {
    const size_t buffered = prefixLength - payloadOffset;
    if (buffered > firmwareLength) {
      const size_t footerBytes =
          std::min(buffered - firmwareLength,
                   static_cast<size_t>(footerLength));
      std::memcpy(otaWorkspace.receivedFooter.data(),
                  otaWorkspace.data.data() + payloadOffset + firmwareLength,
                  footerBytes);
      receivedFooterLength = footerBytes;
    }
    while (receivedFooterLength < static_cast<size_t>(footerLength)) {
      const size_t wanted =
          static_cast<size_t>(footerLength) - receivedFooterLength;
      const int count = receiveBody(
          request,
          otaWorkspace.receivedFooter.data() + receivedFooterLength, wanted);
      if (count <= 0) {
        error = ESP_FAIL;
        break;
      }
      receivedFooterLength += static_cast<size_t>(count);
      consumed += static_cast<size_t>(count);
    }
  }

  if (error == ESP_OK &&
      (receivedFooterLength != static_cast<size_t>(footerLength) ||
       std::memcmp(otaWorkspace.receivedFooter.data(),
                   otaWorkspace.footer.data(),
                   static_cast<size_t>(footerLength)) != 0)) {
    ESP_LOGE(kTag, "Multipart footer mismatch");
    error = ESP_FAIL;
  }
  if (error == ESP_OK && consumed != request->content_len) {
    ESP_LOGE(kTag, "Multipart content length mismatch");
    error = ESP_FAIL;
  }

  if (error == ESP_OK) {
    error = esp_ota_end(otaHandle);
    otaEnded = true;
  }
  if (error == ESP_OK) {
    error = esp_ota_set_boot_partition(partition);
  }

  if (error != ESP_OK) {
    if (otaBegun && !otaEnded) esp_ota_abort(otaHandle);
    ESP_LOGE(kTag, "OTA failed after %u bytes: %s",
             static_cast<unsigned>(written), esp_err_to_name(error));
    return sendOtaFailure(request);
  }

  ESP_LOGI(kTag, "Upload complete: %u bytes",
           static_cast<unsigned>(written));
  const esp_err_t responseError =
      sendText(request, "200 OK", "text/plain; charset=utf-8",
               "OTA成功，设备正在重启。");
  if (responseError != ESP_OK) {
    ESP_LOGW(kTag, "OTA committed but response send failed: %s",
             esp_err_to_name(responseError));
  }
  vTaskDelay(pdMS_TO_TICKS(500));
  esp_restart();
  return ESP_OK;
}

esp_err_t rawUpdateHandler(httpd_req_t *request) {
  const size_t passwordLength =
      httpd_req_get_hdr_value_len(request, "X-OTA-Password");
  std::array<char, 128> password = {};
  if (passwordLength == 0 || passwordLength >= password.size() ||
      httpd_req_get_hdr_value_str(request, "X-OTA-Password",
                                  password.data(), password.size()) != ESP_OK) {
    return sendText(request, "401 Unauthorized", "application/json",
                    "{\"error\":\"OTA password required\"}");
  }
  const bool authorized = serviceConfig.password &&
                          std::strcmp(password.data(),
                                      serviceConfig.password) == 0;
  std::fill(password.begin(), password.end(), '\0');
  if (!authorized) {
    return sendText(request, "401 Unauthorized", "application/json",
                    "{\"error\":\"Invalid OTA password\"}");
  }
  if (request->content_len == 0 ||
      request->content_len > RF_OTA_MAX_IMAGE_BYTES) {
    return sendText(request, "400 Bad Request", "application/json",
                    "{\"error\":\"Invalid image length\"}");
  }

  const esp_partition_t *partition = esp_ota_get_next_update_partition(nullptr);
  if (!partition || partition->type != ESP_PARTITION_TYPE_APP ||
      partition->size < request->content_len) {
    return sendText(request, "500 Internal Server Error", "application/json",
                    "{\"error\":\"OTA partition not found\"}");
  }
  if (otaStartCallback && !otaStartCallback()) {
    return sendText(request, "500 Internal Server Error", "application/json",
                    "{\"error\":\"Actuator stop failed\"}");
  }

  esp_ota_handle_t handle = 0;
  esp_err_t error = esp_ota_begin(partition, request->content_len, &handle);
  const bool otaBegun = error == ESP_OK;
  bool otaEnded = false;
  size_t received = 0;
  while (error == ESP_OK && received < request->content_len) {
    const size_t wanted = std::min(kReceiveBufferBytes,
                                   request->content_len - received);
    const int count = receiveBody(request, otaWorkspace.data.data(), wanted);
    if (count <= 0) {
      error = ESP_FAIL;
      break;
    }
    error = esp_ota_write(handle, otaWorkspace.data.data(),
                          static_cast<size_t>(count));
    if (error == ESP_OK) received += static_cast<size_t>(count);
  }
  if (error == ESP_OK) {
    error = esp_ota_end(handle);
    otaEnded = true;
  }
  if (error == ESP_OK) error = esp_ota_set_boot_partition(partition);
  if (error != ESP_OK) {
    if (otaBegun && !otaEnded) esp_ota_abort(handle);
    ESP_LOGE(kTag, "Raw OTA failed after %u bytes: %s",
             static_cast<unsigned>(received), esp_err_to_name(error));
    return sendText(request, "500 Internal Server Error", "application/json",
                    "{\"error\":\"OTA update failed\"}");
  }

  const esp_err_t response = sendText(
      request, "200 OK", "application/json",
      "{\"success\":true,\"message\":\"OTA update successful, rebooting...\"}");
  vTaskDelay(pdMS_TO_TICKS(500));
  esp_restart();
  return response;
}

void wifiEventHandler(void *, esp_event_base_t eventBase, int32_t eventId,
                      void *eventData) {
  if (eventBase == WIFI_EVENT && eventId == WIFI_EVENT_STA_START &&
      savedStaSsid[0] != '\0') {
    esp_wifi_connect();
  } else if (eventBase == WIFI_EVENT &&
             eventId == WIFI_EVENT_STA_DISCONNECTED) {
    staConnected.store(false, std::memory_order_relaxed);
  } else if (eventBase == WIFI_EVENT &&
             eventId == WIFI_EVENT_AP_STACONNECTED) {
    const uint8_t current = apClientCount.load(std::memory_order_relaxed);
    if (current < UINT8_MAX) {
      apClientCount.store(static_cast<uint8_t>(current + 1),
                          std::memory_order_relaxed);
    }
  } else if (eventBase == WIFI_EVENT &&
             eventId == WIFI_EVENT_AP_STADISCONNECTED) {
    const uint8_t current = apClientCount.load(std::memory_order_relaxed);
    if (current > 0) {
      apClientCount.store(static_cast<uint8_t>(current - 1),
                          std::memory_order_relaxed);
    }
  } else if (eventBase == IP_EVENT && eventId == IP_EVENT_STA_GOT_IP) {
    staConnected.store(true, std::memory_order_relaxed);
  }

  if (serviceConfig.onLinkState) {
    serviceConfig.onLinkState(
        static_cast<uint8_t>(apClientCount.load(std::memory_order_relaxed)),
        staConnected.load(std::memory_order_relaxed),
        serviceConfig.linkStateContext);
  }
  (void)eventData;
}

bool initializeWifi() {
  if (esp_netif_init() != ESP_OK) return false;
  esp_err_t error = esp_event_loop_create_default();
  if (error != ESP_OK && error != ESP_ERR_INVALID_STATE) return false;

  std::fill(savedStaSsid.begin(), savedStaSsid.end(), '\0');
  std::fill(savedStaPassword.begin(), savedStaPassword.end(), '\0');
  nvs_handle_t nvs = 0;
  if (nvs_open(kWifiNamespace, NVS_READONLY, &nvs) == ESP_OK) {
    size_t ssidBytes = savedStaSsid.size();
    size_t passwordBytes = savedStaPassword.size();
    if (nvs_get_str(nvs, kWifiSsidKey, savedStaSsid.data(), &ssidBytes) !=
            ESP_OK ||
        nvs_get_str(nvs, kWifiPasswordKey, savedStaPassword.data(),
                    &passwordBytes) != ESP_OK) {
      std::fill(savedStaSsid.begin(), savedStaSsid.end(), '\0');
      std::fill(savedStaPassword.begin(), savedStaPassword.end(), '\0');
    }
    nvs_close(nvs);
  }
  if (savedStaSsid[0] == '\0' && serviceConfig.staSsid &&
      serviceConfig.staSsid[0] != '\0') {
    std::strncpy(savedStaSsid.data(), serviceConfig.staSsid,
                 savedStaSsid.size() - 1);
    if (serviceConfig.staPassword) {
      std::strncpy(savedStaPassword.data(), serviceConfig.staPassword,
                   savedStaPassword.size() - 1);
    }
  }

  apNetif = esp_netif_create_default_wifi_ap();
  // The recovered product always owns both interfaces. With no saved
  // credentials STA simply remains disconnected, while the fish AP stays up.
  staNetif = esp_netif_create_default_wifi_sta();
  if (!apNetif || !staNetif) return false;

  error = esp_netif_dhcps_stop(apNetif);
  if (error != ESP_OK && error != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
    return false;
  }
  const esp_netif_ip_info_t apInfo = {
      .ip = {.addr = ESP_IP4TOADDR(192, 168, 4, 1)},
      .netmask = {.addr = ESP_IP4TOADDR(255, 255, 255, 0)},
      .gw = {.addr = ESP_IP4TOADDR(192, 168, 4, 1)},
  };
  if (esp_netif_set_ip_info(apNetif, &apInfo) != ESP_OK ||
      esp_netif_dhcps_start(apNetif) != ESP_OK) {
    return false;
  }

  wifi_init_config_t initialization = WIFI_INIT_CONFIG_DEFAULT();
  if (esp_wifi_init(&initialization) != ESP_OK) return false;
  esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifiEventHandler,
                             nullptr);
  esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifiEventHandler,
                             nullptr);

  wifi_config_t apConfiguration = {};
  std::strncpy(reinterpret_cast<char *>(apConfiguration.ap.ssid),
               serviceConfig.apSsid, sizeof(apConfiguration.ap.ssid));
  std::strncpy(reinterpret_cast<char *>(apConfiguration.ap.password),
               serviceConfig.apPassword, sizeof(apConfiguration.ap.password));
  apConfiguration.ap.ssid_len = std::strlen(serviceConfig.apSsid);
  apConfiguration.ap.channel = RF_AP_CHANNEL;
  apConfiguration.ap.authmode = std::strlen(serviceConfig.apPassword) == 0
                                    ? WIFI_AUTH_OPEN
                                    : WIFI_AUTH_WPA2_PSK;
  apConfiguration.ap.ssid_hidden = 0;
  apConfiguration.ap.max_connection = RF_AP_MAX_CONNECTIONS;
  apConfiguration.ap.pmf_cfg.required = false;

  if (esp_wifi_set_mode(WIFI_MODE_APSTA) != ESP_OK ||
      esp_wifi_set_config(WIFI_IF_AP, &apConfiguration) != ESP_OK) {
    return false;
  }

  if (savedStaSsid[0] != '\0') {
    wifi_config_t staConfiguration = {};
    std::strncpy(reinterpret_cast<char *>(staConfiguration.sta.ssid),
                 savedStaSsid.data(), sizeof(staConfiguration.sta.ssid));
    std::strncpy(reinterpret_cast<char *>(staConfiguration.sta.password),
                 savedStaPassword.data(),
                 sizeof(staConfiguration.sta.password));
    if (esp_wifi_set_config(WIFI_IF_STA, &staConfiguration) != ESP_OK) {
      return false;
    }
  }

  if (esp_wifi_start() != ESP_OK) return false;
  ESP_LOGI(kTag, "AP '%s' ready: http://192.168.4.1/update",
           serviceConfig.apSsid);
  return true;
}

bool initializeHttpServer() {
  httpd_config_t configuration = HTTPD_DEFAULT_CONFIG();
  configuration.task_priority = RF_OTA_TASK_PRIORITY;
  configuration.stack_size = RF_OTA_TASK_STACK_BYTES;
  configuration.core_id = RF_OTA_TASK_CORE;
  configuration.max_uri_handlers = 40;
  configuration.recv_wait_timeout = 10;
  configuration.send_wait_timeout = 10;

  if (httpd_start(&server, &configuration) != ESP_OK) return false;

  const httpd_uri_t routes[] = {
      {.uri = "/get_ip",
       .method = HTTP_GET,
       .handler = getIpHandler,
       .user_ctx = nullptr},
      {.uri = "/save_wifi",
       .method = HTTP_GET,
       .handler = saveWifiHandler,
       .user_ctx = nullptr},
      {.uri = "/update",
       .method = HTTP_GET,
       .handler = updatePageHandler,
       .user_ctx = nullptr},
      {.uri = "/update",
       .method = HTTP_POST,
       .handler = updateHandler,
       .user_ctx = nullptr},
      {.uri = "/ota_upload",
       .method = HTTP_POST,
       .handler = rawUpdateHandler,
       .user_ctx = nullptr},
  };
  for (const auto &route : routes) {
    if (httpd_register_uri_handler(server, &route) != ESP_OK) {
      httpd_stop(server);
      server = nullptr;
      return false;
    }
  }
  return true;
}

}  // namespace

bool otaServiceInitializeNvs() {
  if (nvsInitialized.load(std::memory_order_acquire)) return true;
  esp_err_t error = nvs_flash_init();
  if (error == ESP_ERR_NVS_NO_FREE_PAGES ||
      error == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    if (nvs_flash_erase() != ESP_OK) return false;
    error = nvs_flash_init();
  }
  if (error == ESP_OK) {
    nvsInitialized.store(true, std::memory_order_release);
    return true;
  }
  return false;
}

bool otaServiceBegin(const OtaServiceConfig &config,
                     OtaStartCallback onStart) {
  if (server) return true;
  serviceConfig = config;
  otaStartCallback = onStart;

  if (!otaServiceInitializeNvs()) {
    ESP_LOGE(kTag, "Failed to initialize NVS");
    return false;
  }
  if (!initializeWifi()) {
    ESP_LOGE(kTag, "Failed to initialize Wi-Fi");
    return false;
  }
  if (!initializeHttpServer()) {
    ESP_LOGE(kTag, "Failed to start HTTP server");
    return false;
  }
  return true;
}

httpd_handle_t otaServiceHttpServer() { return server; }
