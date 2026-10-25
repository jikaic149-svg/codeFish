#include "Service/Wifi/ControlProtocol.h"
#include <cmath>
#include <cstring>
#include "cJSON.h"

namespace robofish::wifi {
namespace {
bool integer(const cJSON *item, double minimum, double maximum) {
  return cJSON_IsNumber(item) && std::isfinite(item->valuedouble) &&
      item->valuedouble >= minimum && item->valuedouble <= maximum &&
      std::floor(item->valuedouble) == item->valuedouble;
}
bool axis(const cJSON *item) {
  return cJSON_IsNumber(item) && std::isfinite(item->valuedouble) &&
      item->valuedouble >= -100 && item->valuedouble <= 100;
}
bool uniqueFields(const cJSON *object, const char *const *allowed, size_t count) {
  for (const cJSON *item = object->child; item; item = item->next) {
    if (!item->string) return false;
    bool known = false;
    for (size_t i = 0; i < count; ++i) if (!std::strcmp(item->string, allowed[i])) known = true;
    if (!known) return false;
    for (const cJSON *next = item->next; next; next = next->next)
      if (next->string && !std::strcmp(item->string, next->string)) return false;
  }
  return true;
}
}

bool parseControlJson(const char *data, size_t length, Command &command) {
  if (!data || !length || length > kMaximumControlJsonBytes || std::memchr(data, '\0', length)) return false;
  // JSON NUL escapes truncate cJSON's C strings; no control field needs them.
  for (size_t i = 0; i + 6 <= length; ++i)
    if (!std::memcmp(data + i, "\\u0000", 6)) return false;
  const char *end = nullptr;
  cJSON *root = cJSON_ParseWithLengthOpts(data, length, &end, false);
  if (!root) return false;
  bool valid = cJSON_IsObject(root);
  const char *const fields[] = {"action", "session", "sequence", "mode", "axes", "propulsion_scale", "pid", "heading_cw_deg", "surge_pct",
      "tail_drive_pct", "tail_bias_pct", "prop_surge_pct", "prop_diff_pct", "baseline"};
  valid = valid && uniqueFields(root, fields, sizeof(fields) / sizeof(fields[0]));
  for (const char *p = end; valid && p < data + length; ++p)
    if (*p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') valid = false;
  Command parsed{};
  const cJSON *action = cJSON_GetObjectItemCaseSensitive(root, "action");
  const cJSON *session = cJSON_GetObjectItemCaseSensitive(root, "session");
  const cJSON *sequence = cJSON_GetObjectItemCaseSensitive(root, "sequence");
  const cJSON *mode = cJSON_GetObjectItemCaseSensitive(root, "mode");
  const cJSON *axes = cJSON_GetObjectItemCaseSensitive(root, "axes");
  const cJSON *propulsionScale = cJSON_GetObjectItemCaseSensitive(root, "propulsion_scale");
  const cJSON *pid = cJSON_GetObjectItemCaseSensitive(root, "pid");
  const cJSON *heading = cJSON_GetObjectItemCaseSensitive(root, "heading_cw_deg");
  const cJSON *surgeGoal = cJSON_GetObjectItemCaseSensitive(root, "surge_pct");
  const cJSON *tailDrive = cJSON_GetObjectItemCaseSensitive(root, "tail_drive_pct");
  const cJSON *tailBias = cJSON_GetObjectItemCaseSensitive(root, "tail_bias_pct");
  const cJSON *propSurge = cJSON_GetObjectItemCaseSensitive(root, "prop_surge_pct");
  const cJSON *propDiff = cJSON_GetObjectItemCaseSensitive(root, "prop_diff_pct");
  const cJSON *baseline = cJSON_GetObjectItemCaseSensitive(root, "baseline");
  valid = valid && cJSON_IsString(action) && cJSON_IsString(session);
  if (valid) {
    const size_t size = std::strlen(session->valuestring);
    valid = size > 0 && size < sizeof(parsed.session);
    for (size_t i = 0; valid && i < size; ++i)
      valid = static_cast<unsigned char>(session->valuestring[i]) >= 33 &&
              static_cast<unsigned char>(session->valuestring[i]) <= 126;
    if (valid) std::memcpy(parsed.session, session->valuestring, size + 1);
    if (!std::strcmp(action->valuestring, "acquire")) parsed.action = Action::Acquire;
    else if (!std::strcmp(action->valuestring, "target")) parsed.action = Action::Target;
    else if (!std::strcmp(action->valuestring, "mode")) parsed.action = Action::Mode;
    else if (!std::strcmp(action->valuestring, "stop")) parsed.action = Action::Stop;
    else if (!std::strcmp(action->valuestring, "attitude_on")) parsed.action = Action::AttitudeOn;
    else if (!std::strcmp(action->valuestring, "attitude_off")) parsed.action = Action::AttitudeOff;
    else if (!std::strcmp(action->valuestring, "yaw_pid")) parsed.action = Action::YawPid;
    else if (!std::strcmp(action->valuestring, "heading_target")) parsed.action = Action::HeadingTarget;
    else if (!std::strcmp(action->valuestring, "calibration_target")) parsed.action = Action::CalibrationTarget;
    else valid = false;
  }
  if (valid && parsed.action != Action::Acquire) {
    valid = integer(sequence, 1, UINT32_MAX);
    if (valid) parsed.sequence = static_cast<uint32_t>(sequence->valuedouble);
  }
  if (valid && parsed.action == Action::Acquire) valid = !sequence && !mode && !axes;
  if (valid && propulsionScale) {
    valid = (parsed.action == Action::Target || parsed.action == Action::HeadingTarget) && cJSON_IsNumber(propulsionScale) &&
        std::isfinite(propulsionScale->valuedouble) &&
        propulsionScale->valuedouble >= 0 && propulsionScale->valuedouble <= 100;
    if (valid) parsed.propulsionScalePercent = static_cast<float>(propulsionScale->valuedouble);
  }
  if (valid && parsed.action == Action::Target) {
    const char *const axisFields[] = {"surge", "yaw", "heave"};
    valid = !mode && cJSON_IsObject(axes) && uniqueFields(axes, axisFields, 3);
    const cJSON *surge = cJSON_GetObjectItemCaseSensitive(axes, "surge");
    const cJSON *yaw = cJSON_GetObjectItemCaseSensitive(axes, "yaw");
    const cJSON *heave = cJSON_GetObjectItemCaseSensitive(axes, "heave");
    valid = valid && axis(surge) && axis(yaw) && axis(heave);
    if (valid) parsed.axes = {static_cast<float>(surge->valuedouble),
        static_cast<float>(yaw->valuedouble), static_cast<float>(heave->valuedouble)};
  }
  if (valid && parsed.action == Action::Mode) {
    valid = !axes && integer(mode, 1, 3);
    if (valid) parsed.mode = static_cast<DriveMode>(static_cast<unsigned>(mode->valuedouble));
  }
  if (valid && (parsed.action == Action::Stop || parsed.action == Action::AttitudeOn ||
                parsed.action == Action::AttitudeOff)) valid = !axes && !mode;
  if (valid && parsed.action == Action::YawPid) {
    const char *const names[] = {"kp", "ki", "kd"};
    valid = !axes && !mode && cJSON_IsObject(pid) && uniqueFields(pid, names, 3);
    float *gains[] = {&parsed.yawPid.kp, &parsed.yawPid.ki, &parsed.yawPid.kd};
    for (unsigned i=0; valid && i<3; ++i) {
      const cJSON *gain=cJSON_GetObjectItemCaseSensitive(pid,names[i]);
      valid=cJSON_IsNumber(gain) && std::isfinite(gain->valuedouble) &&
          gain->valuedouble>=0 && gain->valuedouble<=100;
      if (valid) *gains[i]=static_cast<float>(gain->valuedouble);
    }
  } else if (pid) valid=false;
  if (valid && parsed.action==Action::HeadingTarget) {
    valid=!mode && !axes && cJSON_IsNumber(heading) && std::isfinite(heading->valuedouble) &&
        heading->valuedouble>=-180 && heading->valuedouble<=180 && axis(surgeGoal) && surgeGoal->valuedouble>=0;
    if (valid) {
      parsed.headingCwDegrees=static_cast<float>(heading->valuedouble);
      parsed.axes.surge=static_cast<float>(surgeGoal->valuedouble);
    }
  } else if (heading || surgeGoal) valid=false;
  if (valid && parsed.action == Action::CalibrationTarget) {
    valid = !axes && !propulsionScale && integer(mode, 1, 3) && axis(tailDrive) &&
        tailDrive->valuedouble >= 0 && axis(tailBias) && axis(propSurge) && axis(propDiff) &&
        (!baseline || cJSON_IsBool(baseline));
    if (valid) {
      parsed.mode = static_cast<DriveMode>(static_cast<unsigned>(mode->valuedouble));
      parsed.calibration = {static_cast<float>(tailDrive->valuedouble), static_cast<float>(tailBias->valuedouble),
          static_cast<float>(propSurge->valuedouble), static_cast<float>(propDiff->valuedouble), cJSON_IsTrue(baseline) != 0};
      if (parsed.calibration.baseline) valid = parsed.mode == DriveMode::Hybrid &&
          tailDrive->valuedouble == propSurge->valuedouble && tailBias->valuedouble == propDiff->valuedouble;
      // Unused actuator inputs must be zero, so recorded given values match the selected trial.
      if (parsed.mode == DriveMode::Bionic) valid = valid && propSurge->valuedouble == 0 && propDiff->valuedouble == 0;
      if (parsed.mode == DriveMode::Prop) valid = valid && tailDrive->valuedouble == 0 && tailBias->valuedouble == 0;
    }
  } else if (tailDrive || tailBias || propSurge || propDiff || baseline) valid = false;
  cJSON_Delete(root);
  if (valid) command = parsed;
  return valid;
}
}
