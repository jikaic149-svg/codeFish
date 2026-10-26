#include "network.h"
#include "bsp.h"
#include "Service/LoRa/LoraProtocol.h"
#include "Service/Wifi/ControlProtocol.h"
#include <algorithm>
#include <array>
#include <atomic>
#include <cstdio>
#include <cstring>
#include "config/AppConfig.h"
#include "config/BoardConfig.h"
#include "config/AttitudeConfig.h"
#include "cJSON.h"
#include "esp_app_desc.h"
#include "esp_app_format.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "mbedtls/base64.h"
#include "nvs_flash.h"
#include "freertos/task.h"

namespace {
QueueHandle_t snapshots, stopRequests, stopResults, commands, commandResults;
std::atomic_bool runtimeReady{false};
httpd_handle_t server = nullptr;
esp_netif_t *apNetif = nullptr;
// One HTTP server task owns both buffers. The control loop never uses them.
std::array<char, 6144> responseBuffer;
std::array<uint8_t, 4096> imageBuffer;

esp_err_t json(httpd_req_t *req, cJSON *body) {
  if (!body) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "JSON allocation failed");
  const bool encoded = cJSON_PrintPreallocated(body, responseBuffer.data(), responseBuffer.size(), false);
  cJSON_Delete(body);
  if (!encoded) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "JSON capacity exceeded");
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, responseBuffer.data(), HTTPD_RESP_USE_STRLEN);
}

esp_err_t identity(httpd_req_t *req) {
  cJSON *body = cJSON_CreateObject();
  const esp_app_desc_t *app = esp_app_get_description();
  const esp_partition_t *running = esp_ota_get_running_partition();
  esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
  const bool confirmed = running && esp_ota_get_state_partition(running, &state) == ESP_OK &&
                         state == ESP_OTA_IMG_VALID;
  esp_netif_ip_info_t ip{};
  if (apNetif) esp_netif_get_ip_info(apNetif, &ip);
  char address[16];
  std::snprintf(address, sizeof(address), IPSTR, IP2STR(&ip.ip));
  cJSON_AddStringToObject(body, "firmware", "robotfish");
  cJSON_AddStringToObject(body, "hardware", "robotfish-esp32s3");
  cJSON_AddStringToObject(body, "project_name", app->project_name);
  cJSON_AddStringToObject(body, "version", app->version);
  cJSON_AddStringToObject(body, "running_slot", running ? running->label : "");
  cJSON_AddBoolToObject(body, "runtime_ready", runtimeReady.load());
  cJSON_AddBoolToObject(body, "ota_confirmed", confirmed && runtimeReady.load());
  cJSON_AddBoolToObject(body, "sta_connected", false);
  cJSON_AddStringToObject(body, "sta_ip", "0.0.0.0");
  cJSON_AddStringToObject(body, "sta_ssid", "");
  cJSON_AddStringToObject(body, "ap_ip", address);
  cJSON_AddStringToObject(body, "ssid", appconfig::kSsid);
  cJSON_AddBoolToObject(body, "camera_available", false);
  cJSON_AddBoolToObject(body, "pump_available", false);
  cJSON_AddBoolToObject(body, "wifi_control_allowed", true);
  cJSON_AddNumberToObject(body, "control_protocol", 1);
  cJSON_AddStringToObject(body, "control_transport", "lora_wifi_fallback");
  cJSON_AddStringToObject(body, "telemetry_transport", "wifi");
  cJSON_AddNumberToObject(body, "lora_protocol", 2);
  cJSON_AddNumberToObject(body, "lora_timeout_ms", robofish::config::lora::kLinkTimeoutMs);
  cJSON_AddStringToObject(body, "imu_model", "JY61P");
  cJSON_AddNumberToObject(body, "imu_baud", board::config::kImuBaud);
  cJSON_AddBoolToObject(body, "heave_fin_only", true);
  cJSON_AddBoolToObject(body, "propulsion_scale_control", true);
  const auto attitudeSettings = robofish::config::attitude::makeSettings();
  cJSON_AddBoolToObject(body, "attitude_control_supported", true);
  cJSON_AddBoolToObject(body, "yaw_pid_supported", true);
  cJSON_AddBoolToObject(body, "heading_target_supported", true);
  cJSON_AddBoolToObject(body, "heading_tail_guidance_supported", true);
  cJSON_AddBoolToObject(body, "heading_bias_priority_supported", true);
  cJSON_AddStringToObject(body, "heading_allocation_profile", robofish::config::control::kHeadingAllocationProfile);
  cJSON_AddBoolToObject(body, "dynamics_calibration_supported", true);
  const auto calibration = robofish::attitude::evaluate(attitudeSettings, {}, 0, false, false, false);
  cJSON_AddBoolToObject(body, "attitude_control_calibrated", calibration.pitchCalibrated || calibration.rollCalibrated || calibration.yawCalibrated);
  return json(req, body);
}

cJSON *frameJson(const robofish::ActuatorFrame &frame) {
  cJSON *body = cJSON_CreateObject();
  cJSON_AddNumberToObject(body, "tail_front_deg", frame.tailFrontDegrees);
  cJSON_AddNumberToObject(body, "tail_rear_deg", frame.tailRearDegrees);
  cJSON_AddNumberToObject(body, "pectoral_left_deg", frame.pectoralLeftDegrees);
  cJSON_AddNumberToObject(body, "pectoral_right_deg", frame.pectoralRightDegrees);
  cJSON_AddNumberToObject(body, "left_motor_pct", frame.leftPropulsionPercent);
  cJSON_AddNumberToObject(body, "right_motor_pct", frame.rightPropulsionPercent);
  cJSON_AddBoolToObject(body, "flow_assisted", frame.flowAssisted);
  return body;
}

esp_err_t telemetry(httpd_req_t *req) {
  MotionSnapshot s;
  if (xQueuePeek(snapshots, &s, 0) != pdTRUE) {
    httpd_resp_set_status(req, "503 Service Unavailable");
    return httpd_resp_send(req, "No control snapshot", HTTPD_RESP_USE_STRLEN);
  }
  const auto &target = s.control.target;
  const bool controlActive = s.control.leaseActive ||
      (s.control.attitudeMotionAllowed() && s.attitude.yawActive);
  const bool failsafe = s.control.expired || s.control.outputRecovery || s.control.outputFault;
  cJSON *body = cJSON_CreateObject();
  cJSON_AddNumberToObject(body, "version", 1);
  cJSON_AddStringToObject(body, "source", failsafe ? "failsafe" : controlActive ? s.control.controlSource() : "none");
  cJSON_AddStringToObject(body, "last_source", s.control.controlSource());
  cJSON_AddStringToObject(body, "control_transport", "lora_wifi_fallback");
  cJSON_AddStringToObject(body, "telemetry_transport", "wifi");
  cJSON_AddNumberToObject(body, "lora_protocol", 2);
  cJSON_AddNumberToObject(body, "lora_timeout_ms", robofish::config::lora::kLinkTimeoutMs);
  cJSON_AddBoolToObject(body, "wifi_control_allowed", true);
  cJSON_AddNumberToObject(body, "control_protocol", 1);
  cJSON_AddNumberToObject(body, "mode", static_cast<int>(target.mode));
  cJSON_AddNumberToObject(body, "propulsion_scale_percent", target.propulsionScale * 100);
  const auto action = !s.control.leaseActive ? MotionAction::Stopped :
      target.surgePercent < 0 ? MotionAction::Backward :
      target.yawPercent < 0 ? MotionAction::Left : target.yawPercent > 0 ? MotionAction::Right :
      target.surgePercent > 0 ? MotionAction::Forward : MotionAction::Stopped;
  cJSON_AddNumberToObject(body, "action", static_cast<int>(action));
  cJSON_AddBoolToObject(body, "remote_active", controlActive);
  cJSON_AddBoolToObject(body, "failsafe_active", failsafe);
  cJSON_AddNumberToObject(body, "motion_timeout_ms", appconfig::kLeaseMs);
  cJSON_AddStringToObject(body, "lease_servo_action", appconfig::kLeaseMs ? "center" : "hold");
  cJSON_AddStringToObject(body, "safety_state", s.control.safetyState());
  cJSON_AddBoolToObject(body, "output_warning", s.control.outputFailureStreak > 0 && !s.control.outputRecovery && !s.control.outputFault);
  cJSON_AddBoolToObject(body, "output_recovery", s.control.outputRecovery);
  cJSON_AddBoolToObject(body, "output_fault", s.control.outputFault);
  cJSON_AddNumberToObject(body, "output_failure_streak", s.control.outputFailureStreak);
  cJSON_AddNumberToObject(body, "failed_mask", s.control.failedMask);
  cJSON_AddBoolToObject(body, "control_rearm_required", s.control.outputRecovery || s.control.outputFault);
  cJSON_AddBoolToObject(body, "ota_locked", s.control.maintenance);
  cJSON_AddBoolToObject(body, "soft_stopping", s.control.softStopping);
  cJSON_AddBoolToObject(body, "pump_enabled", false);
  cJSON_AddNumberToObject(body, "uptime_ms", s.nowMs);
  const auto age = uint32_t(s.nowMs - s.control.lastMotionMs);
  cJSON_AddNumberToObject(body, "lease_remaining_ms", s.control.leaseActive && age < appconfig::kLeaseMs ? appconfig::kLeaseMs - age : 0);
  cJSON *remote = cJSON_AddObjectToObject(body, "remote_target");
  cJSON_AddBoolToObject(remote, "enabled", target.enabled);
  cJSON_AddNumberToObject(remote, "surge_pct", target.surgePercent);
  cJSON_AddNumberToObject(remote, "yaw_pct", target.yawPercent);
  cJSON_AddNumberToObject(remote, "heave_pct", target.pitchPercent);
  cJSON_AddBoolToObject(body, "dynamics_calibration_supported", true);
  cJSON *calibration = cJSON_AddObjectToObject(body, "dynamics_calibration");
  cJSON_AddBoolToObject(body, "heading_tail_guidance_supported", true);
  cJSON_AddBoolToObject(body, "heading_bias_priority_supported", true);
  cJSON_AddStringToObject(body, "heading_allocation_profile", robofish::config::control::kHeadingAllocationProfile);
  cJSON_AddBoolToObject(calibration, "active", s.control.calibrationActive);
  cJSON_AddBoolToObject(calibration, "baseline", s.control.calibration.baseline);
  cJSON_AddNumberToObject(calibration, "mode", static_cast<int>(target.mode));
  cJSON_AddNumberToObject(calibration, "tail_drive_pct", s.control.calibration.tailDrivePercent);
  cJSON_AddNumberToObject(calibration, "tail_bias_pct", s.control.calibration.tailBiasPercent);
  cJSON_AddNumberToObject(calibration, "prop_surge_pct", s.control.calibration.propSurgePercent);
  cJSON_AddNumberToObject(calibration, "prop_diff_pct", s.control.calibration.propDiffPercent);
  cJSON_AddNumberToObject(calibration, "command_received_ms", s.control.lastMotionMs);
  cJSON_AddItemToObject(body, "last_frame", frameJson(s.attempted));
  cJSON_AddItemToObject(body, "last_written", frameJson(s.written));
  cJSON_AddNumberToObject(body, "written_mask", s.writtenMask);
  cJSON_AddBoolToObject(body, "api_submitted", s.writtenMask == 63);
  cJSON_AddNumberToObject(body, "attempts", s.attempts);
  cJSON_AddNumberToObject(body, "failures", s.failures);
  cJSON_AddNumberToObject(body, "period_ms", s.periodMs);
  cJSON *allocation = cJSON_AddObjectToObject(body, "allocation");
  cJSON_AddNumberToObject(allocation, "phase_rad", s.allocation.phaseRad);
  cJSON_AddNumberToObject(allocation, "common_pct", s.allocation.commonPropulsionPercent);
  cJSON_AddNumberToObject(allocation, "differential_pct", s.allocation.differentialPropulsionPercent);
  cJSON_AddNumberToObject(allocation, "tail_drive_pct", s.allocation.tailDrivePercent);
  cJSON_AddNumberToObject(allocation, "tail_bias_pct", s.allocation.tailBiasPercent);
  cJSON_AddNumberToObject(allocation, "tail_frequency_hz", s.allocation.tailFrequencyHz);
  cJSON_AddNumberToObject(allocation, "tail_amplitude_deg", s.allocation.tailAmplitudeDegrees);
  // Nominal joint commands after travel limiting and before output slew; no servo sensor is present.
  cJSON_AddNumberToObject(allocation, "effective_tail_bias_deg", s.allocation.effectiveTailBiasDegrees);
  cJSON_AddNumberToObject(allocation, "effective_tail_amplitude_deg", s.allocation.effectiveTailAmplitudeDegrees);
  cJSON_AddNumberToObject(allocation, "tail_load", s.allocation.tailLoad);
  cJSON_AddNumberToObject(allocation, "written_common_pct", (s.written.leftPropulsionPercent + s.written.rightPropulsionPercent) * .5f);
  cJSON_AddNumberToObject(allocation, "tail_amplitude_gain", s.control.tailAmplitudeGain());
  cJSON_AddNumberToObject(allocation, "written_differential_pct", (s.written.leftPropulsionPercent - s.written.rightPropulsionPercent) * .5f);
  cJSON_AddNumberToObject(allocation, "written_tail_front_offset_deg", s.written.tailFrontDegrees - robofish::config::lora::kTailFrontCenter);
  cJSON_AddNumberToObject(allocation, "written_tail_rear_offset_deg", s.written.tailRearDegrees - robofish::config::lora::kTailRearCenter);
  cJSON_AddNumberToObject(allocation, "limited_reasons", s.allocation.limitedReasons);
  cJSON *wifi = cJSON_AddObjectToObject(body, "wifi");
  cJSON_AddStringToObject(wifi, "session", s.wifi.session);
  cJSON_AddNumberToObject(wifi, "sequence", s.wifi.sequence);
  cJSON_AddNumberToObject(wifi, "accepted", s.wifi.accepted);
  cJSON_AddNumberToObject(wifi, "rejected", s.wifi.rejected);
  cJSON_AddNumberToObject(wifi, "result_drops", s.wifi.resultDrops);
  cJSON_AddStringToObject(body, "preferred_control", s.control.loraHealthy(s.nowMs) ? "lora" : "wifi");
  cJSON *lora = cJSON_AddObjectToObject(body, "lora");
  cJSON_AddBoolToObject(lora, "healthy", s.control.loraHealthy(s.nowMs));
  cJSON_AddBoolToObject(lora, "seen", s.control.loRaSeen);
  if (s.control.loRaSeen) cJSON_AddNumberToObject(lora, "age_ms", uint32_t(s.nowMs - s.control.lastLoRaMs));
  cJSON_AddBoolToObject(lora, "uart_ready", s.loraReady);
  cJSON_AddBoolToObject(lora, "tx_enabled", s.lora.txEnabled);
  cJSON_AddNumberToObject(lora, "accepted", s.lora.accepted);
  cJSON_AddNumberToObject(lora, "rejected", s.lora.rejected);
  cJSON_AddNumberToObject(lora, "duplicates", s.lora.duplicates);
  cJSON_AddNumberToObject(lora, "stale", s.lora.stale);
  cJSON_AddNumberToObject(lora, "receive_errors", s.lora.receiveErrors);
  cJSON_AddNumberToObject(lora, "last_source", s.lora.lastSource);
  cJSON_AddNumberToObject(lora, "last_sequence", s.lora.lastSequence);
  cJSON_AddNumberToObject(lora, "last_code", s.lora.lastCode);
  cJSON_AddNumberToObject(lora, "last_command_ms", s.lora.lastCommandMs);
  cJSON_AddNumberToObject(lora, "knob", s.control.knob);
  const auto &result = s.control.loraResult;
  cJSON *reply = cJSON_AddObjectToObject(body, "command_result");
  cJSON_AddBoolToObject(reply, "available", result.available);
  if (result.available) {
    cJSON_AddBoolToObject(reply, "accepted", result.code == robofish::lora::ControlCode::Accepted);
    cJSON_AddStringToObject(reply, "result", robofish::lora::controlCodeName(result.code));
    cJSON_AddNumberToObject(reply, "source", result.command.source);
    cJSON_AddNumberToObject(reply, "sequence", result.command.sequence);
    cJSON_AddNumberToObject(reply, "code", result.command.code);
    cJSON_AddNumberToObject(reply, "received_ms", result.receivedMs);
  }
  char ack[64]{};
  if (s.lora.accepted) {
    robofish::lora::Command accepted;
    accepted.source = s.lora.lastSource; accepted.sequence = s.lora.lastSequence; accepted.code = s.lora.lastCode;
    robofish::lora::formatAck(ack, sizeof(ack), accepted, s.lora.accepted);
  }
  cJSON_AddStringToObject(lora, "last_ack", ack); // 原 910 回执仅通过 HTTP 读取。
  const auto &imu = s.imu;
  cJSON *sensor = cJSON_AddObjectToObject(body, "imu");
  cJSON_AddStringToObject(sensor, "model", "JY61P");
  cJSON_AddBoolToObject(sensor, "uart_ready", imu.uartReady);
  cJSON_AddNumberToObject(sensor, "baud", imu.baud);
  cJSON_AddNumberToObject(sensor, "rx_bytes", imu.receivedBytes);
  cJSON_AddNumberToObject(sensor, "frames", imu.frames);
  cJSON_AddNumberToObject(sensor, "checksum_errors", imu.checksumErrors);
  cJSON_AddNumberToObject(sensor, "receive_errors", imu.receiveErrors);
  const auto sample = [&](const char *name, const robofish::imu::Vector3 &value, bool valid, uint32_t at) {
    cJSON *item = cJSON_AddObjectToObject(sensor, name);
    cJSON_AddBoolToObject(item, "valid", valid && uint32_t(s.nowMs - at) <= 500);
    if (valid) {
      cJSON_AddNumberToObject(item, "x", value.x);
      cJSON_AddNumberToObject(item, "y", value.y);
      cJSON_AddNumberToObject(item, "z", value.z);
      cJSON_AddNumberToObject(item, "age_ms", uint32_t(s.nowMs - at));
    }
  };
  sample("acceleration_g", imu.acceleration, imu.hasAcceleration, imu.accelerationMs);
  sample("angular_velocity_dps", imu.angularVelocity, imu.hasAngularVelocity, imu.angularVelocityMs);
  sample("angles_deg", imu.angles, imu.hasAngles, imu.anglesMs);
  sample("filtered_angles_deg",s.filteredAngles,s.filteredAnglesValid,s.filteredAnglesMs);
  cJSON_AddNumberToObject(sensor,"filter_stage_hz",robofish::config::attitude::kAngleFilterStageHz);
  cJSON_AddBoolToObject(body, "attitude_available", imu.hasAngles && uint32_t(s.nowMs - imu.anglesMs) <= 500);
  cJSON_AddBoolToObject(body, "attitude_control_supported", true);
  cJSON *attitude = cJSON_AddObjectToObject(body, "attitude_control");
  const auto &feedback = s.attitude;
  cJSON_AddBoolToObject(attitude, "enabled", s.control.attitudeEnabled);
  cJSON_AddBoolToObject(attitude, "yaw_pid_supported", true);
  cJSON_AddBoolToObject(attitude, "heading_target_supported", true);
  cJSON_AddBoolToObject(attitude, "heading_target_active", s.control.headingTargetActive);
  cJSON_AddStringToObject(attitude, "heading_allocation_profile", robofish::config::control::kHeadingAllocationProfile);
  cJSON_AddNumberToObject(attitude, "heading_tail_drive_ratio", robofish::config::control::kHeadingTailDriveRatio);
  cJSON_AddNumberToObject(attitude, "heading_tail_amplitude_gain", robofish::config::control::kHybridTailAmplitudeGain);
  cJSON_AddNumberToObject(attitude, "heading_prop_diff_gain", robofish::config::control::kHeadingPropDiffGain);
  cJSON_AddNumberToObject(attitude, "heading_prop_diff_limit_pct", robofish::config::control::kHeadingPropDiffLimitPercent);
  if (s.control.headingTargetActive) cJSON_AddNumberToObject(attitude,"requested_heading_cw_deg",s.control.headingTargetCwDegrees);
  cJSON_AddNumberToObject(attitude,"pid_period_ms",robofish::config::attitude::kYawPidPeriodMs);
  cJSON_AddNumberToObject(attitude,"yaw_max_correction_pct",robofish::config::attitude::kMaximumYawCorrectionPercent);
  cJSON_AddNumberToObject(attitude,"yaw_command_sign",robofish::config::attitude::kYawCommandSign);
  cJSON *pid=cJSON_AddObjectToObject(attitude,"pid");
  cJSON_AddNumberToObject(pid,"kp",s.control.yawPid.kp);
  cJSON_AddNumberToObject(pid,"ki",s.control.yawPid.ki);
  cJSON_AddNumberToObject(pid,"kd",s.control.yawPid.kd);
  cJSON_AddStringToObject(attitude, "reason", robofish::attitude::reasonName(feedback.reason));
  cJSON_AddBoolToObject(attitude, "pitch_calibrated", feedback.pitchCalibrated);
  cJSON_AddBoolToObject(attitude, "roll_calibrated", feedback.rollCalibrated);
  cJSON_AddBoolToObject(attitude, "yaw_calibrated", feedback.yawCalibrated);
  cJSON_AddBoolToObject(attitude, "body_valid", feedback.bodyValid);
  cJSON_AddBoolToObject(attitude, "pitch_active", feedback.pitchActive);
  cJSON_AddBoolToObject(attitude, "roll_active", feedback.rollActive);
  cJSON_AddBoolToObject(attitude, "yaw_active", feedback.yawActive);
  cJSON_AddBoolToObject(attitude, "heading_valid", feedback.headingValid);
  cJSON_AddBoolToObject(attitude, "heading_locked", feedback.headingLocked);
  cJSON_AddBoolToObject(attitude, "manual_yaw", feedback.manualYaw);
  cJSON_AddNumberToObject(attitude, "maximum_age_ms", robofish::config::attitude::kMaximumSampleAgeMs);
  cJSON_AddNumberToObject(attitude, "pitch_correction_pct", feedback.pitchCorrectionPercent);
  cJSON_AddNumberToObject(attitude, "roll_correction_pct", feedback.rollCorrectionPercent);
  cJSON_AddNumberToObject(attitude, "yaw_correction_pct", feedback.yawCorrectionPercent);
  cJSON_AddNumberToObject(attitude, "yaw_deadband_deg", robofish::config::attitude::kYawDeadbandDegrees);
  cJSON_AddNumberToObject(attitude, "yaw_actuator_gain", robofish::config::attitude::kYawActuatorGain);
  cJSON_AddNumberToObject(attitude, "yaw_turn_pct", feedback.yawActive ? std::clamp(
      feedback.yawCorrectionPercent * robofish::config::attitude::kYawActuatorGain, -100.0f, 100.0f) : 0);
  cJSON_AddNumberToObject(attitude,"yaw_error_deg",feedback.yawErrorDegrees);
  cJSON_AddNumberToObject(attitude,"yaw_p",feedback.yawP);
  cJSON_AddNumberToObject(attitude,"yaw_i",feedback.yawI);
  cJSON_AddNumberToObject(attitude,"yaw_d",feedback.yawD);
  cJSON *preview=cJSON_AddObjectToObject(attitude,"tilt_preview");
  cJSON_AddBoolToObject(preview,"valid",feedback.tiltPreviewValid);
  cJSON_AddBoolToObject(preview,"outputs_connected",robofish::config::attitude::kPitchOutputEnabled || robofish::config::attitude::kRollOutputEnabled);
  cJSON_AddNumberToObject(preview,"pitch_body_pct",feedback.pitchPreviewPercent);
  cJSON_AddNumberToObject(preview,"roll_body_pct",feedback.rollPreviewPercent);
  if (feedback.hasSample)
    cJSON_AddNumberToObject(attitude, "age_ms", uint32_t(s.nowMs - feedback.sampleMs));
  if (feedback.bodyValid) {
    cJSON_AddNumberToObject(attitude, "body_roll_deg", feedback.rollDegrees);
    cJSON_AddNumberToObject(attitude, "body_pitch_deg", feedback.pitchDegrees);
    if (feedback.headingValid) cJSON_AddNumberToObject(attitude, "body_yaw_deg", feedback.yawDegrees);
  }
  if (feedback.headingLocked) cJSON_AddNumberToObject(attitude, "heading_target_deg", feedback.headingTargetDegrees);
  return json(req, body);
}

bool authenticated(httpd_req_t *req) {
  char password[128]{};
  if (httpd_req_get_hdr_value_str(req, "X-OTA-Password", password, sizeof(password)) == ESP_OK) {
    const bool accepted = std::strcmp(password, appconfig::kOtaPassword) == 0;
    std::memset(password, 0, sizeof(password));
    return accepted;
  }
  char header[160]{};
  unsigned char decoded[112]{};
  size_t bytes = 0;
  if (httpd_req_get_hdr_value_str(req, "Authorization", header, sizeof(header)) != ESP_OK ||
      std::strncmp(header, "Basic ", 6) != 0 ||
      mbedtls_base64_decode(decoded, sizeof(decoded) - 1, &bytes,
          reinterpret_cast<unsigned char *>(header + 6), std::strlen(header + 6)) != 0)
    return false;
  const bool accepted = bytes == std::strlen(appconfig::kOtaUser) + 1 + std::strlen(appconfig::kOtaPassword) &&
      std::memcmp(decoded, appconfig::kOtaUser, std::strlen(appconfig::kOtaUser)) == 0 &&
      decoded[std::strlen(appconfig::kOtaUser)] == ':' &&
      std::memcmp(decoded + std::strlen(appconfig::kOtaUser) + 1,
                  appconfig::kOtaPassword, std::strlen(appconfig::kOtaPassword)) == 0;
  std::memset(decoded, 0, sizeof(decoded));
  return accepted;
}

esp_err_t unauthorized(httpd_req_t *req) {
  httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"RobotFish\"");
  httpd_resp_set_status(req, "401 Unauthorized");
  cJSON *body = cJSON_CreateObject();
  cJSON_AddStringToObject(body, "error", "invalid_password");
  return json(req, body);
}

esp_err_t controlError(httpd_req_t *req, const char *status, const char *message) {
  httpd_resp_set_status(req, status);
  cJSON *body = cJSON_CreateObject();
  cJSON_AddBoolToObject(body, "accepted", false);
  cJSON_AddStringToObject(body, "error", message);
  return json(req, body);
}

esp_err_t controlRequest(httpd_req_t *req) {
  if (!authenticated(req)) return unauthorized(req);
  char contentType[80]{};
  if (httpd_req_get_hdr_value_str(req, "Content-Type", contentType, sizeof(contentType)) != ESP_OK ||
      std::strncmp(contentType, "application/json", 16) != 0 ||
      (contentType[16] && contentType[16] != ';'))
    return controlError(req, "400 Bad Request", "application_json_required");
  if (!runtimeReady.load()) return controlError(req, "503 Service Unavailable", "runtime_not_ready");
  if (!req->content_len || req->content_len > robofish::wifi::kMaximumControlJsonBytes)
    return controlError(req, "400 Bad Request", "invalid_length");
  std::array<char, robofish::wifi::kMaximumControlJsonBytes + 1> data{};
  size_t received = 0;
  while (received < req->content_len) {
    const int count = httpd_req_recv(req, data.data() + received, req->content_len - received);
    if (count <= 0) return controlError(req, "400 Bad Request", "incomplete_request");
    received += count;
  }
  robofish::wifi::Command command{};
  if (!robofish::wifi::parseControlJson(data.data(), received, command))
    return controlError(req, "400 Bad Request", "invalid_control");
  static uint32_t requestId = 0;
  command.requestId = ++requestId;
  command.receivedMs = board::nowMs();
  if (xQueueSend(commands, &command, 0) != pdTRUE)
    return controlError(req, "503 Service Unavailable", "control_busy");
  robofish::wifi::Result result{};
  const TickType_t started = xTaskGetTickCount();
  const TickType_t budget = pdMS_TO_TICKS(appconfig::kCommandResultTimeoutMs);
  for (;;) {
    const TickType_t elapsed = xTaskGetTickCount() - started;
    if (elapsed >= budget || xQueueReceive(commandResults, &result, budget - elapsed) != pdTRUE)
      return controlError(req, "503 Service Unavailable", "control_timeout");
    if (result.requestId == command.requestId) break;
  }
  using Code = robofish::wifi::Code;
  if (result.code != Code::Accepted) {
    const char *reason = result.code == Code::Invalid ? "invalid_control" :
        result.code == Code::SessionConflict ? "control_session_changed" :
        result.code == Code::StaleSequence ? "stale_sequence" :
        result.code == Code::Expired ? "command_expired" :
        result.code == Code::OutputRecovery ? "output_recovery" :
        result.code == Code::OutputFault ? "output_fault" :
          result.code == Code::LoRaActive ? "lora_primary_active" :
          result.code == Code::NeutralPending ? "neutral_pending" :
          result.code == Code::HeadingUnavailable ? "heading_unavailable" : "ota_locked";
    return controlError(req, result.code == Code::Invalid ? "400 Bad Request" : "409 Conflict", reason);
  }
  cJSON *body = cJSON_CreateObject();
  cJSON_AddBoolToObject(body, "accepted", true);
  cJSON_AddStringToObject(body, "session", result.session);
  cJSON_AddNumberToObject(body, "sequence", result.sequence);
  cJSON_AddNumberToObject(body, "id", command.requestId);
  cJSON_AddNumberToObject(body, "mode", static_cast<int>(result.mode));
  cJSON_AddStringToObject(body, "owner", "wifi");
  cJSON_AddBoolToObject(body, "simulated", false);
  cJSON_AddBoolToObject(body, "lease_active", result.leaseActive);
  cJSON_AddBoolToObject(body, "stop_pending", result.stopPending);
  cJSON_AddBoolToObject(body, "attitude_enabled", result.attitudeEnabled);
  cJSON *axes = cJSON_AddObjectToObject(body, "axes");
  cJSON_AddNumberToObject(axes, "surge", result.axes.surge);
  cJSON_AddNumberToObject(axes, "yaw", result.axes.yaw);
  cJSON_AddNumberToObject(axes, "heave", result.axes.heave);
  return json(req, body);
}

extern const unsigned char pageStart[] asm("_binary_control_html_start");
extern const unsigned char pageEnd[] asm("_binary_control_html_end");

esp_err_t updatePage(httpd_req_t *req) {
  if (!authenticated(req)) return unauthorized(req);
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, reinterpret_cast<const char *>(pageStart), pageEnd - pageStart - 1);
}

esp_err_t update(httpd_req_t *req) {
  if (!authenticated(req)) return unauthorized(req);
  constexpr size_t prefixBytes = sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t);
  if (req->content_len < prefixBytes || req->content_len > appconfig::kMaximumImageBytes)
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid application size");
  size_t received = 0;
  while (received < prefixBytes) {
    const int count = httpd_req_recv(req, reinterpret_cast<char *>(imageBuffer.data()) + received, prefixBytes - received);
    if (count <= 0) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Incomplete image header");
    received += count;
  }
  esp_image_header_t header;
  esp_app_desc_t desc;
  std::memcpy(&header, imageBuffer.data(), sizeof(header));
  std::memcpy(&desc, imageBuffer.data() + sizeof(header) + sizeof(esp_image_segment_header_t), sizeof(desc));
  if (header.magic != ESP_IMAGE_HEADER_MAGIC || header.chip_id != ESP_CHIP_ID_ESP32S3 ||
      desc.magic_word != ESP_APP_DESC_MAGIC_WORD ||
      std::memcmp(desc.project_name, "robotfish\0", 10) != 0)
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Expected robotfish ESP32-S3 application image");
  const esp_partition_t *partition = esp_ota_get_next_update_partition(nullptr);
  if (!partition || req->content_len > partition->size)
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Application exceeds inactive partition");
  const bool stop = true;
  bool stopped = false;
  xQueueReset(stopResults);
  if (xQueueSend(stopRequests, &stop, 0) != pdTRUE ||
      xQueueReceive(stopResults, &stopped, pdMS_TO_TICKS(appconfig::kOtaStopTimeoutMs)) != pdTRUE || !stopped)
    return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Actuator stop not confirmed");
  esp_ota_handle_t handle = 0;
  esp_err_t error = esp_ota_begin(partition, req->content_len, &handle);
  const bool begun = error == ESP_OK;
  if (error == ESP_OK) error = esp_ota_write(handle, imageBuffer.data(), received);
  while (error == ESP_OK && received < req->content_len) {
    const size_t wanted = std::min(imageBuffer.size(), req->content_len - received);
    const int count = httpd_req_recv(req, reinterpret_cast<char *>(imageBuffer.data()), wanted);
    if (count <= 0) { error = ESP_FAIL; break; }
    error = esp_ota_write(handle, imageBuffer.data(), count);
    received += count;
  }
  if (error != ESP_OK) {
    if (begun) esp_ota_abort(handle);
    return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA transfer failed; motion remains locked");
  }
  error = esp_ota_end(handle);
  if (error == ESP_OK) error = esp_ota_set_boot_partition(partition);
  if (error != ESP_OK)
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Image validation failed; motion remains locked");
  cJSON *body = cJSON_CreateObject();
  cJSON_AddBoolToObject(body, "success", true);
  cJSON_AddStringToObject(body, "message", "Image committed; rebooting");
  const esp_err_t sent = json(req, body);
  vTaskDelay(pdMS_TO_TICKS(300));
  esp_restart();
  return sent;
}
}

bool networkBegin(QueueHandle_t values, QueueHandle_t controlCommands, QueueHandle_t controlResults,
                  QueueHandle_t requests, QueueHandle_t results) {
  commands = controlCommands; commandResults = controlResults;
  snapshots = values;
  stopRequests = requests; stopResults = results;
  if (nvs_flash_init() != ESP_OK || esp_netif_init() != ESP_OK ||
      esp_event_loop_create_default() != ESP_OK) return false;
  apNetif = esp_netif_create_default_wifi_ap();
  const wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
  if (!apNetif || esp_wifi_init(&init) != ESP_OK || esp_wifi_set_storage(WIFI_STORAGE_RAM) != ESP_OK)
    return false;
  wifi_config_t wifi{};
  std::memcpy(wifi.ap.ssid, appconfig::kSsid, sizeof(appconfig::kSsid));
  std::memcpy(wifi.ap.password, appconfig::kWifiPassword, sizeof(appconfig::kWifiPassword));
  wifi.ap.ssid_len = std::strlen(appconfig::kSsid);
  wifi.ap.channel = 1; wifi.ap.max_connection = 4; wifi.ap.authmode = WIFI_AUTH_WPA2_PSK;
  if (esp_wifi_set_mode(WIFI_MODE_AP) != ESP_OK || esp_wifi_set_config(WIFI_IF_AP, &wifi) != ESP_OK ||
      esp_wifi_start() != ESP_OK) return false;
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.stack_size = 8192;
  config.recv_wait_timeout = config.send_wait_timeout = 10;
  if (httpd_start(&server, &config) != ESP_OK) return false;
  const httpd_uri_t routes[] = {
      {.uri="/get_ip", .method=HTTP_GET, .handler=identity, .user_ctx=nullptr},
      {.uri="/motion_status", .method=HTTP_GET, .handler=telemetry, .user_ctx=nullptr},
      {.uri="/status", .method=HTTP_GET, .handler=telemetry, .user_ctx=nullptr},
      {.uri="/lora_status", .method=HTTP_GET, .handler=telemetry, .user_ctx=nullptr},
      {.uri="/api/v1/control", .method=HTTP_POST, .handler=controlRequest, .user_ctx=nullptr},
      {.uri="/", .method=HTTP_GET, .handler=updatePage, .user_ctx=nullptr},
      {.uri="/update", .method=HTTP_GET, .handler=updatePage, .user_ctx=nullptr},
      {.uri="/ota_upload", .method=HTTP_POST, .handler=update, .user_ctx=nullptr}};
  for (const auto &route : routes)
    if (httpd_register_uri_handler(server, &route) != ESP_OK) return false;
  ESP_LOGI("network", "AP %s; telemetry and OTA ready", appconfig::kSsid);
  return true;
}

void networkConfirmStartup(bool ready) {
  runtimeReady.store(ready);
  if (!ready) return;
  const esp_partition_t *partition = esp_ota_get_running_partition();
  esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
  if (partition && esp_ota_get_state_partition(partition, &state) == ESP_OK &&
      state == ESP_OTA_IMG_PENDING_VERIFY && esp_ota_mark_app_valid_cancel_rollback() != ESP_OK)
    ESP_LOGE("network", "Could not confirm startup image");
}
