#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>
#include <limits>
#include <type_traits>
#include "ControlState.h"

using namespace robofish::wifi;
static unsigned checks = 0;
static void check(bool value, const char *label) {
  ++checks;
  if (!value) { std::fprintf(stderr, "FAIL: %s\n", label); std::exit(1); }
}
static Command command(Action action, uint32_t sequence = 0, uint32_t received = 0,
                       const char *session = "browser-1") {
  Command value;
  value.requestId = sequence + 10;
  value.action = action;
  value.sequence = sequence;
  value.receivedMs = received;
  std::snprintf(value.session, sizeof(value.session), "%s", session);
  return value;
}
static void acquire(ControlState &state) {
  const auto value = state.accept(command(Action::Acquire), 0);
  check(value.code == Code::Accepted && value.stopPending && value.sequence == 0 &&
      std::strcmp(value.session, "browser-1") == 0 && !value.leaseActive,
      "new session is an acknowledged full-neutral barrier");
  state.stopPending = false; // Simulate the sole owner's successful six-channel neutral frame.
}
static Result move(ControlState &state, uint32_t sequence, uint32_t now,
                   Axes axes = {100, 0, 0}) {
  auto value = command(Action::Target, sequence, now);
  value.axes = axes;
  return state.accept(value, now);
}
static void boundaries() {
  check(appconfig::kLeaseMs == 3000, "lease remains exactly three seconds");
  for (uint32_t start : {0u, 100u, 0xfffffff0u}) {
    ControlState state;
    acquire(state);
    check(move(state, 1, start).code == Code::Accepted && state.leaseActive &&
        state.target.enabled && !state.expired && state.lastMotionMs == start,
        "target starts a lease, including time zero and clock wrap");
    check(!state.expire(start + 2999u) && state.leaseActive, "lease valid at 2999 ms");
    check(state.expire(start + 3000u) && !state.leaseActive && state.expired &&
        !state.target.enabled, "lease expires at exactly 3000 ms");
    check(state.target.surgePercent == 0 && state.target.yawPercent == 0 &&
        state.target.pitchPercent == 0 && !state.target.allowFlowAssist,
        "expiry discards every retained motion field");
    check(!state.expire(start + 3001u), "expiry is a single event");
    check(move(state, 2, start + 3002u, {0, 0, -40}).code == Code::Accepted &&
        state.target.surgePercent == 0 && state.target.pitchPercent == -40 &&
        !state.expired, "fresh absolute target never revives expired surge");
  }
  ControlState queued;
  acquire(queued);
  auto value = command(Action::Target, 1, 100);
  value.axes = {20, -30, 40};
  check(queued.accept(value, 249).code == Code::Accepted && queued.lastMotionMs == 100,
      "queue delay cannot move the original receive deadline");
  check(queued.expire(3100), "delayed target expires at its original three-second deadline");
  value.sequence = 2;
  check(queued.accept(value, 250).code == Code::Expired && queued.wifi.sequence == 1,
      "target at exact HTTP timeout rejects without committing its sequence");
}
static void absoluteTargetsAndModes() {
  ControlState state;
  acquire(state);
  check(move(state, 1, 100, {75, -25, 30}).code == Code::Accepted &&
      state.target.surgePercent == 75 && state.target.yawPercent == -25 &&
      state.target.pitchPercent == 30 && state.target.outputScale == 1 &&
      state.target.allowFlowAssist, "full percentage target is applied once without a second throttle scale");
  check(move(state, 2, 200, {-40, 0, 0}).code == Code::Accepted &&
      state.target.surgePercent == -40 && state.target.yawPercent == 0 &&
      state.target.pitchPercent == 0 && !state.target.allowFlowAssist,
      "next absolute target clears omitted previous-axis demands");
  auto mode = command(Action::Mode, 3, 3199);
  mode.mode = DriveMode::Hybrid;
  check(state.accept(mode, 3199).code == Code::Accepted && state.target.mode == DriveMode::Hybrid &&
      state.lastMotionMs == 200 && state.target.surgePercent == -40,
      "mode selection preserves current target but does not renew its lease");
  check(state.expire(3200), "mode traffic cannot postpone expiry");
  mode.sequence = 4; mode.mode = DriveMode::Bionic;
  mode.receivedMs = 3300;
  check(state.accept(mode, 3300).code == Code::Accepted && state.expired &&
      !state.leaseActive && !state.target.enabled && state.target.mode == DriveMode::Bionic,
      "mode selection cannot restart expired motion");
  check(move(state, 5, 3400, {0, 0, 80}).code == Code::Accepted &&
      state.target.mode == DriveMode::Bionic && state.target.surgePercent == 0 &&
      state.target.pitchPercent == 80, "fresh heave preserves configured mode only");
  auto stopped = move(state, 6, 3500, {0, 0, 0});
  check(stopped.code == Code::Accepted && stopped.stopPending && !stopped.leaseActive &&
      !state.expired && !state.target.enabled && state.target.pitchPercent == 0 &&
      stopped.mode == DriveMode::Bionic, "zero axes explicitly stop and center every actuator");
}
static void orderingAndSessions() {
  ControlState state;
  acquire(state);
  check(move(state, 1, 100).code == Code::Accepted, "initial target accepted");
  check(move(state, 1, 3099).code == Code::StaleSequence && state.lastMotionMs == 100 &&
      state.wifi.sequence == 1, "duplicate target cannot renew the lease");
  check(state.expire(3100), "duplicate traffic cannot prevent expiry");
  check(move(state, 1, 3101).code == Code::StaleSequence && !state.leaseActive,
      "duplicate target cannot revive expired motion");
  auto stop = command(Action::Stop, 10, 3200);
  auto result = state.accept(stop, 3200);
  check(result.code == Code::Accepted && result.requestId == stop.requestId &&
      result.sequence == 10 && result.stopPending && !result.leaseActive,
      "STOP result identifies the accepted command and its neutral barrier");
  check(move(state, 9, 3201).code == Code::StaleSequence && !state.leaseActive,
      "older in-flight motion cannot cross the STOP sequence barrier");
  const auto again = state.accept(command(Action::Acquire, 0, 3202), 3202);
  check(again.code == Code::Accepted && again.sequence == 10 && state.stopPending &&
      !state.leaseActive, "same-session acquire preserves STOP sequence watermark");
  state.stopPending = false;
  check(move(state, 11, 3300).code == Code::Accepted, "newer target may resume after neutral output");
  check(state.accept(command(Action::Acquire, 0, 3400), 3400).code == Code::Accepted &&
      state.wifi.sequence == 11 && state.lastMotionMs == 3300 && state.leaseActive,
      "duplicate acquire does not reset sequence or renew the current target");
  const auto takeover = state.accept(command(Action::Acquire, 0, 3500, "new-page-uuid"), 3500);
  check(takeover.code == Code::Accepted && takeover.sequence == 0 && takeover.stopPending &&
      !takeover.leaseActive && state.target.surgePercent == 0,
      "new page session explicitly stops and resets its own sequence");
  check(move(state, 12, 3501).code == Code::SessionConflict && !state.leaseActive,
      "old page cannot send motion after another page acquired control");
  auto target = command(Action::Target, 1, 3600, "new-page-uuid"); target.axes.surge = 20;
  check(state.accept(target, 3600).code == Code::Accepted && state.wifi.sequence == 1,
      "new session starts at its own first sequence");
  target.sequence = UINT32_MAX;
  check(state.accept(target, 3601).code == Code::Accepted && state.wifi.sequence == UINT32_MAX,
      "maximum sequence is accepted without signed narrowing");
  target.sequence = 1;
  check(state.accept(target, 3602).code == Code::StaleSequence,
      "sequence cannot wrap and revive old packets within one session");
}
static void rejection() {
  ControlState state;
  auto target = command(Action::Target, 1, 0); target.axes.surge = 25;
  check(state.accept(target, 0).code == Code::SessionConflict,
      "motion requires explicit session acquisition");
  acquire(state);
  for (float invalid : {100.01f, -100.01f, std::numeric_limits<float>::infinity(),
      -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN()}) {
    for (unsigned axis = 0; axis < 3; ++axis) {
      target.axes = {10, 20, 30};
      if (axis == 0) target.axes.surge = invalid;
      else if (axis == 1) target.axes.yaw = invalid;
      else target.axes.heave = invalid;
      check(state.accept(target, 0).code == Code::Invalid && state.wifi.sequence == 0 &&
          !state.leaseActive, "nonfinite and out-of-range axes reject without committing state");
    }
  }
  target.axes = {-100, 100, -100};
  check(state.accept(target, 0).code == Code::Accepted && state.wifi.sequence == 1,
      "percentage endpoints are inclusive");
  for (DriveMode invalid : {DriveMode::Idle, static_cast<DriveMode>(4), static_cast<DriveMode>(255)}) {
    auto mode = command(Action::Mode, 2); mode.mode = invalid;
    check(state.accept(mode, 0).code == Code::Invalid && state.wifi.sequence == 1,
        "unknown or idle mode rejects without consuming sequence");
  }
  auto bad = command(static_cast<Action>(255), 2);
  check(state.accept(bad, 0).code == Code::Invalid, "unknown action rejects");
  bad = command(Action::Acquire); bad.session[0] = 0;
  check(state.accept(bad, 0).code == Code::Invalid, "empty session rejects");
  std::memset(bad.session, 'x', sizeof(bad.session));
  check(state.accept(bad, 0).code == Code::Invalid, "unterminated session rejects");
  bad = command(Action::Acquire); bad.session[1] = '\n';
  check(state.accept(bad, 0).code == Code::Invalid, "control characters in session reject");
  bad.session[1] = static_cast<char>(128);
  check(state.accept(bad, 0).code == Code::Invalid, "non-ASCII session rejects");
  std::memset(bad.session, 'x', sizeof(bad.session)); bad.session[64] = 0;
  check(state.accept(bad, 0).code == Code::Accepted, "64-character printable session accepted");
  check(move(state, 0, 0).code == Code::SessionConflict, "old session rejected before sequence handling");
  auto zero = command(Action::Stop, 0); std::memcpy(zero.session, bad.session, sizeof(zero.session));
  check(state.accept(zero, 0).code == Code::StaleSequence, "sequence zero is never accepted");

  ControlState ota;
  acquire(ota);
  move(ota, 1, 100);
  ota.stop(); ota.maintenance = true;
  for (Action action : {Action::Acquire, Action::Target, Action::Mode, Action::Stop}) {
    auto value = command(action, 2, 200); value.axes.surge = 100;
    check(ota.accept(value, 200).code == Code::Maintenance && ota.maintenance &&
        !ota.leaseActive && !ota.target.enabled && ota.stopPending && ota.wifi.sequence == 1,
        "OTA maintenance rejects every action and cannot be released by a client");
  }
}
static void requestDeadlines() {
  for (uint32_t received : {0u, 100u, 0xfffffff0u}) {
    for (Action action : {Action::Acquire, Action::Target, Action::Mode, Action::Stop}) {
      ControlState state;
      acquire(state);
      const auto prepare = move(state, 1, received);
      check(prepare.code == Code::Accepted, "prepare deadline case");
      auto value = command(action, 2, received);
      value.axes = {-25, 30, -40};
      value.mode = DriveMode::Hybrid;
      if (action == Action::Acquire) std::strcpy(value.session, "late-page-uuid");
      const auto timeout = state.accept(value, received + appconfig::kCommandResultTimeoutMs);
      check(timeout.code == Code::Expired && state.wifi.sequence == 1 &&
          std::strcmp(state.wifi.session, "browser-1") == 0 && state.target.mode == DriveMode::Prop &&
          state.target.surgePercent == 100 && state.target.yawPercent == 0 &&
          state.target.pitchPercent == 0 && state.lastMotionMs == received && state.leaseActive &&
          !state.stopPending, "timed-out HTTP command changes no session, target, mode or sequence");
      check(state.accept(value, received + appconfig::kCommandResultTimeoutMs - 1).code == Code::Accepted,
          "deadline permits last millisecond, including clock wraparound");
    }
  }
}
int main() {
  static_assert(std::is_trivially_copyable_v<Command> && std::is_trivially_copyable_v<Result>);
  boundaries(); absoluteTargetsAndModes(); orderingAndSessions(); rejection(); requestDeadlines();
  std::printf("PASS: %u Wi-Fi control checks (absolute targets, 3s lease/wrap, sessions, sequence/STOP replay barrier, validation, OTA lock)\n", checks);
}
