#pragma once

#include <array>

#include "camera/CameraService.h"
#include "control/MotionController.h"
#include "esp_err.h"
#include "esp_http_server.h"
#include "partner/PartnerScriptEngine.h"

class ProductWebService {
 public:
  ProductWebService(MotionController &motion,
                    robofish::partner::ScriptEngine &partner,
                    CameraService &camera);

  esp_err_t registerRoutes(httpd_handle_t server);

  // CameraService invokes this only after it has rejected the variable as a
  // camera parameter, so unknown variables do not disturb Partner execution.
  static esp_err_t controlFallback(httpd_req_t *request,
                                   const char *variable, int value,
                                   void *context);

 private:
  static esp_err_t rootHandler(httpd_req_t *request);
  static esp_err_t partnerScriptGetHandler(httpd_req_t *request);
  static esp_err_t partnerScriptPostHandler(httpd_req_t *request);
  static esp_err_t partnerRunHandler(httpd_req_t *request);
  static esp_err_t partnerStopHandler(httpd_req_t *request);
  static esp_err_t partnerStatusHandler(httpd_req_t *request);
  static esp_err_t visionControlHandler(httpd_req_t *request);
  static esp_err_t visionPidHandler(httpd_req_t *request);
  static esp_err_t captivePortalHandler(httpd_req_t *request);

  esp_err_t dispatchMotionControl(httpd_req_t *request,
                                  const char *variable, int value);
  esp_err_t sendPartnerScript(httpd_req_t *request);
  esp_err_t savePartnerScript(httpd_req_t *request);
  esp_err_t runPartner(httpd_req_t *request);
  esp_err_t stopPartner(httpd_req_t *request);
  esp_err_t sendPartnerStatus(httpd_req_t *request);
  esp_err_t applyVisionControl(httpd_req_t *request);
  esp_err_t applyVisionPid(httpd_req_t *request);
  void stopPartnerForManualControl();

  MotionController &motion_;
  robofish::partner::ScriptEngine &partner_;
  CameraService &camera_;
  std::array<char, robofish::partner::kMaxJsonBytes + 1> scriptBuffer_{};
};
