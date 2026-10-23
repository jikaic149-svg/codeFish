#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include "Service/Wifi/ControlProtocol.h"

using namespace robofish::wifi;
static unsigned checks = 0;
static void check(bool condition, const char *message) {
  ++checks;
  if (!condition) { std::fprintf(stderr, "FAIL: %s\n", message); std::exit(1); }
}
static bool parse(const std::string &json, Command &out) {
  return parseControlJson(json.data(), json.size(), out);
}
static std::string target(const std::string &axes, const std::string &sequence = "1") {
  return "{\"action\":\"target\",\"session\":\"browser-1\",\"sequence\":" + sequence +
      ",\"axes\":" + axes + "}";
}
static bool same(const Command &a, const Command &b) {
  return a.requestId == b.requestId && a.sequence == b.sequence && a.receivedMs == b.receivedMs &&
      !std::strcmp(a.session, b.session) && a.action == b.action && a.mode == b.mode &&
      a.axes.surge == b.axes.surge && a.axes.yaw == b.axes.yaw && a.axes.heave == b.axes.heave;
}
static void reject(const std::string &json) {
  Command out;
  out.requestId = 42; out.sequence = 99; out.receivedMs = 80;
  std::strcpy(out.session, "unchanged"); out.action = Action::Mode;
  out.mode = DriveMode::Hybrid; out.axes = {12, -20, 50};
  const auto before = out;
  if (parse(json, out)) { std::fprintf(stderr, "Unexpected acceptance: %s\n", json.c_str()); std::exit(1); }
  check(same(before, out), "invalid JSON leaves command untouched");
}

int main() {
  Command out;
  check(parse("{\"action\":\"acquire\",\"session\":\"browser-1\"}", out) &&
      out.action == Action::Acquire && out.sequence == 0 && !std::strcmp(out.session, "browser-1"),
      "acquire accepts without a sequence");
  check(parse(target("{\"surge\":-100,\"yaw\":100,\"heave\":0.5}", "4294967295"), out) &&
      out.action == Action::Target && out.sequence == UINT32_MAX && out.axes.surge == -100 &&
      out.axes.yaw == 100 && out.axes.heave == 0.5f, "target accepts signed limits, fractions and UINT32_MAX");
  for (int mode = 1; mode <= 3; ++mode)
    check(parse("{\"action\":\"mode\",\"session\":\"browser-1\",\"sequence\":1,\"mode\":" +
        std::to_string(mode) + "}", out) && static_cast<int>(out.mode) == mode,
        "mode accepts precisely the three supported values");
  check(parse("{\"action\":\"stop\",\"session\":\"browser-1\",\"sequence\":1}", out) &&
      out.action == Action::Stop, "stop accepts with no axes");
  for (const char *action : {"attitude_on", "attitude_off"}) {
    const std::string prefix=std::string("{\"action\":\"")+action+"\",\"session\":\"browser-1\"";
    check(parse(prefix+",\"sequence\":1}",out) &&
        out.action==(std::string(action)=="attitude_on"?Action::AttitudeOn:Action::AttitudeOff),
        "attitude configuration uses ordered authenticated control");
    reject(prefix+"}");
    reject(prefix+",\"sequence\":0}");
    reject(prefix+",\"sequence\":1,\"axes\":{}}");
    reject(prefix+",\"sequence\":1,\"mode\":2}");
    reject(prefix+",\"sequence\":1,\"propulsion_scale\":50}");
  }
  check(parse("{\"action\":\"acquire\",\"session\":\"" + std::string(64, 'x') + "\"}", out) &&
      std::strlen(out.session) == 64, "session accepts 64 printable bytes");
  const std::string valid = target("{\"surge\":0,\"yaw\":0,\"heave\":0}");
  const std::string maximum = valid + std::string(kMaximumControlJsonBytes - valid.size(), ' ');
  check(parse(maximum, out), "maximum 512-byte JSON with trailing whitespace accepts");
  check(parse("\r\n\t " + valid + " \r\n\t", out), "JSON surrounding whitespace accepts");
  reject(maximum + " ");
  reject(""); reject("null"); reject("[]"); reject("{}"); reject("[1,2]");
  reject(valid + "false"); reject(valid + "{}"); reject(valid.substr(0, valid.size() - 1));
  reject(std::string(valid.data(), valid.size()) + std::string(1, '\0'));
  check(!parseControlJson(nullptr, valid.size(), out), "null input rejects");
  for (const char *json : {
      "{\"action\":\"target\",\"session\":\"x\",\"sequence\":1}",
      "{\"session\":\"x\"}", "{\"action\":\"acquire\"}",
      "{\"action\":1,\"session\":\"x\"}", "{\"action\":\"acquire\",\"session\":true}",
      "{\"action\":\"Acquire\",\"session\":\"x\"}", "{\"action\":\"toggle\",\"session\":\"x\"}",
      "{\"action\":\"acquire\",\"session\":\"\"}", "{\"action\":\"acquire\",\"session\":\"has space\"}",
      "{\"action\":\"acquire\",\"session\":\"x\\t\"}", "{\"action\":\"acquire\",\"session\":\"x\\u007f\"}",
      "{\"action\":\"acquire\",\"session\":\"x\\u0080\"}", "{\"action\":\"acquire\",\"session\":\"x\\u0000y\"}",
      "{\"action\":\"acquire\\u0000other\",\"session\":\"x\"}",
      "{\"action\":\"acquire\",\"session\":\"x\",\"sequence\":0}",
      "{\"action\":\"acquire\",\"session\":\"x\",\"mode\":1}",
      "{\"action\":\"acquire\",\"session\":\"x\",\"axes\":{}}",
      "{\"action\":\"acquire\",\"session\":\"x\",\"source\":\"lora\"}",
      "{\"action\":\"acquire\",\"session\":\"x\",\"session\":\"y\"}",
      "{\"action\":\"acquire\",\"Action\":\"stop\",\"session\":\"x\"}",
      "{\"action\":\"acquire\",\"session\":\"x\",\"token\":\"unused\"}",
      "{\"action\":\"stop\",\"session\":\"x\",\"sequence\":1,\"axes\":null}",
      "{\"action\":\"stop\",\"session\":\"x\",\"sequence\":1,\"mode\":null}",
      "{\"action\":\"stop\",\"session\":\"x\"}",
      "{\"action\":\"mode\",\"session\":\"x\",\"sequence\":1}",
      "{\"action\":\"mode\",\"session\":\"x\",\"sequence\":1,\"mode\":true}",
      "{\"action\":\"mode\",\"session\":\"x\",\"sequence\":1,\"mode\":0}",
      "{\"action\":\"mode\",\"session\":\"x\",\"sequence\":1,\"mode\":4}",
      "{\"action\":\"mode\",\"session\":\"x\",\"sequence\":1,\"mode\":1.5}",
      "{\"action\":\"mode\",\"session\":\"x\",\"sequence\":1,\"mode\":2,\"axes\":{}}",
      "{\"action\":\"target\",\"session\":\"x\",\"sequence\":1,\"axes\":{\"surge\":0,\"yaw\":0,\"heave\":0},\"mode\":2}"
  }) reject(json);
  reject("{\"action\":\"acquire\",\"session\":\"" + std::string(65, 'x') + "\"}");
  for (const char *sequence : {"0", "-1", "4294967296", "1.5", "true", "null", "\"1\"", "1e400"})
    reject(target("{\"surge\":0,\"yaw\":0,\"heave\":0}", sequence));
  for (const char *axes : {"null", "[]", "{}", "{\"surge\":0,\"yaw\":0}",
      "{\"surge\":0,\"yaw\":0,\"heave\":0,\"roll\":0}",
      "{\"surge\":0,\"yaw\":0,\"heave\":0,\"surge\":10}",
      "{\"Surge\":0,\"yaw\":0,\"heave\":0}"}) reject(target(axes));
  for (const char *value : {"-100.001", "100.001", "1e400", "-1e400", "true", "false", "null", "\"50\"", "NaN", "Infinity"}) {
    reject(target(std::string("{\"surge\":") + value + ",\"yaw\":0,\"heave\":0}"));
    reject(target(std::string("{\"surge\":0,\"yaw\":") + value + ",\"heave\":0}"));
    reject(target(std::string("{\"surge\":0,\"yaw\":0,\"heave\":") + value + "}"));
  }
  std::printf("PASS: %u real cJSON Wi-Fi protocol checks (strict fields, limits, types, NUL, sessions, sequences)\n", checks);
}
