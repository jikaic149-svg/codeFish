#include "OtaService.h"

#include <Update.h>
#include <WebServer.h>
#include <WiFi.h>

namespace {

WebServer server(80);
OtaServiceConfig serviceConfig = {};
OtaStartCallback otaStartCallback = nullptr;
TaskHandle_t otaTaskHandle = nullptr;
volatile bool uploadStarted = false;
volatile bool uploadSucceeded = false;

const char OTA_HOME_PAGE[] PROGMEM = R"HTML(
<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>RoboFish Driver SDK</title><style>body{max-width:700px;margin:40px auto;padding:0 18px;background:#07151d;color:#eaf7fa;font-family:system-ui}section{background:#102630;border:1px solid #2b5364;border-radius:14px;padding:20px}a{display:inline-block;padding:11px 16px;background:#39d2a0;color:#042019;text-decoration:none;border-radius:9px;font-weight:700}code{color:#8de5ff}</style></head>
<body><section><h1>RoboFish Partner Driver SDK</h1><p>基础驱动与远程 OTA 服务已运行。本固件不包含游动、转向、CPG、演示或自主控制算法。</p><p>开发者需要在 <code>loop()</code> 中自行编写关节时序和控制策略。</p><a href="/update">Web OTA</a></section></body></html>
)HTML";

const char OTA_UPLOAD_PAGE[] PROGMEM = R"HTML(
<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>RoboFish OTA</title>
<style>body{max-width:620px;margin:40px auto;padding:0 18px;background:#07151d;color:#eaf7fa;font-family:system-ui}form{background:#102630;border:1px solid #2b5364;border-radius:14px;padding:24px}input,button{width:100%;margin-top:14px;padding:12px}button{background:#39d2a0;border:0;border-radius:8px;font-weight:700}</style></head>
<body><form method="POST" action="/update" enctype="multipart/form-data"><h2>RoboFish Web OTA</h2><p>请选择 Arduino 导出的应用固件 <code>.bin</code>。上传开始时所有执行器会停止并锁定。</p><input type="file" name="firmware" accept=".bin" required><button type="submit">上传并重启</button></form></body></html>
)HTML";

bool authenticated() {
  if (server.authenticate(serviceConfig.username, serviceConfig.password)) return true;
  server.requestAuthentication(BASIC_AUTH, "RoboFish OTA");
  return false;
}

void sendStatus() {
  char response[256];
  IPAddress apIp = WiFi.softAPIP();
  IPAddress staIp = WiFi.localIP();
  snprintf(response, sizeof(response),
           "{\"sdk\":\"1.0.0\",\"ap_ip\":\"%u.%u.%u.%u\","
           "\"sta_connected\":%s,\"sta_ip\":\"%u.%u.%u.%u\"}",
           apIp[0], apIp[1], apIp[2], apIp[3],
           WiFi.status() == WL_CONNECTED ? "true" : "false",
           staIp[0], staIp[1], staIp[2], staIp[3]);
  server.send(200, "application/json", response);
}

void handleUploadData() {
  if (!authenticated()) return;
  HTTPUpload &upload = server.upload();

  if (upload.status == UPLOAD_FILE_START) {
    uploadStarted = true;
    uploadSucceeded = false;
    if (otaStartCallback) otaStartCallback();
    Serial.printf("[OTA] Upload start: %s\n", upload.filename.c_str());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) Update.printError(Serial);
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    uploadSucceeded = Update.end(true);
    if (uploadSucceeded) {
      Serial.printf("[OTA] Upload complete: %u bytes\n", upload.totalSize);
    } else {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    Update.abort();
    uploadSucceeded = false;
    Serial.println("[OTA] Upload aborted");
  }
}

void finishUploadRequest() {
  if (!authenticated()) return;
  bool success = uploadStarted && uploadSucceeded && !Update.hasError();
  server.send(success ? 200 : 500, "text/plain; charset=utf-8",
              success ? "OTA成功，设备正在重启。" : "OTA失败，请查看串口日志后重试。");
  uploadStarted = false;
  if (success) {
    delay(500);
    ESP.restart();
  }
}

void otaTask(void *parameter) {
  (void)parameter;
  for (;;) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

}  // namespace

bool otaServiceBegin(const OtaServiceConfig &config, OtaStartCallback onStart) {
  if (otaTaskHandle) return true;
  serviceConfig = config;
  otaStartCallback = onStart;

  WiFi.mode(WIFI_AP_STA);
  if (!WiFi.softAP(serviceConfig.apSsid, serviceConfig.apPassword)) {
    Serial.println("[OTA] Failed to start access point");
    return false;
  }
  Serial.printf("[OTA] AP '%s' ready: http://%s/update\n",
                serviceConfig.apSsid, WiFi.softAPIP().toString().c_str());

  if (serviceConfig.staSsid && serviceConfig.staSsid[0] != '\0') {
    WiFi.begin(serviceConfig.staSsid, serviceConfig.staPassword);
    Serial.printf("[OTA] Connecting to STA '%s' in background\n", serviceConfig.staSsid);
  }

  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html; charset=utf-8", OTA_HOME_PAGE);
  });
  server.on("/status", HTTP_GET, sendStatus);
  server.on("/update", HTTP_GET, []() {
    if (!authenticated()) return;
    server.send_P(200, "text/html; charset=utf-8", OTA_UPLOAD_PAGE);
  });
  server.on("/update", HTTP_POST, finishUploadRequest, handleUploadData);
  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
  server.begin();

  BaseType_t created = xTaskCreatePinnedToCore(
      otaTask, "WebOTA", 6144, nullptr, 2, &otaTaskHandle, 0);
  if (created != pdPASS) {
    otaTaskHandle = nullptr;
    server.stop();
    Serial.println("[OTA] Failed to create background task");
    return false;
  }
  return true;
}

