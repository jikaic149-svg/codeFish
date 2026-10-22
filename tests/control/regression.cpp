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
      !state.target.allowFlowAssist, "absolute fin state never enables automatic propulsion or tail drive");
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
  check(stopped.code == Code::Accepted && !stopped.stopPending && stopped.leaseActive && state.softStopping &&
      !state.expired && !state.target.enabled && state.target.pitchPercent == 0 &&
      stopped.mode == DriveMode::Bionic, "zero axes request an ordinary ramp to neutral without an emergency barrier");
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
static void outputDegradation() {
  for (unsigned failedChannel = 0; failedChannel < 6; ++failedChannel) {
    ControlState state;
    acquire(state);
    check(move(state, 1, 10).code == Code::Accepted, "start degradation fixture");
    const uint8_t incomplete = static_cast<uint8_t>(63u & ~(1u << failedChannel));
    state.observeOutput(incomplete);
    check(state.outputFailureStreak == 1 && state.failedMask == (1u << failedChannel) &&
        !state.outputRecovery && state.leaseActive &&
        std::strcmp(state.safetyState(), "OUTPUT_WARNING") == 0,
        "one failed channel is reported without discarding the active command");
    state.observeOutput(63);
    check(state.outputFailureStreak == 0 && state.failedMask == 0 && state.leaseActive &&
        std::strcmp(state.safetyState(), "RUNNING") == 0,
        "one fully successful frame clears a transient warning");
    for (unsigned count = 1; count <= appconfig::kOutputNeutralFailures; ++count) {
      state.observeOutput(incomplete);
      check(state.outputFailureStreak == count, "failures count actual consecutive output submissions");
      if (count < appconfig::kOutputNeutralFailures)
        check(!state.outputRecovery && state.leaseActive, "warning does not stop before threshold");
    }
    check(state.outputRecovery && !state.outputFault && state.stopPending && !state.leaseActive &&
        !state.target.enabled && state.wifi.session[0] == 0 && state.wifi.sequence == 0 &&
        std::strcmp(state.safetyState(), "OUTPUT_RECOVERY") == 0,
        "repeated output failures neutralize and invalidate the old control session");
    for (Action action : {Action::Acquire, Action::Target, Action::Mode}) {
      auto value = command(action, 2, 20); value.axes.surge = 100;
      check(state.accept(value, 20).code == Code::OutputRecovery && !state.leaseActive,
          "every motion or authorization request rejects while output recovery is active");
    }
    state.observeOutput(63);
    check(!state.outputRecovery && !state.outputFault && state.outputFailureStreak == 0 &&
        state.wifi.session[0] == 0 && !state.leaseActive,
        "successful neutral output clears recovery without resurrecting session or motion");
    state.stopPending = false;
    check(move(state, 2, 30).code == Code::SessionConflict && !state.leaseActive,
        "old session targets cannot resume after output recovery");
    check(state.accept(command(Action::Acquire, 0, 40), 40).code == Code::Accepted &&
        state.stopPending && !state.leaseActive, "recovery needs fresh explicit authorization");
    state.stopPending = false;
    check(move(state, 1, 50).code == Code::Accepted, "fresh authorized target may resume recovered hardware");
    for (unsigned count = 0; count < appconfig::kOutputFaultFailures; ++count)
      state.observeOutput(incomplete);
    check(state.outputFault && state.outputRecovery && !state.leaseActive &&
        std::strcmp(state.safetyState(), "OUTPUT_FAULT") == 0,
        "long consecutive failures latch an output fault");
    for (unsigned count = 0; count < 300; ++count) state.observeOutput(incomplete);
    check(state.outputFailureStreak == appconfig::kOutputFaultFailures,
        "failure streak saturates instead of wrapping back into normal operation");
    state.observeOutput(63);
    state.stopPending = false;
    check(state.outputFault && !state.outputRecovery && state.outputFailureStreak == 0 &&
        std::strcmp(state.safetyState(), "OUTPUT_FAULT") == 0,
        "a later successful output cannot clear the latched fault");
    for (Action action : {Action::Acquire, Action::Target, Action::Mode}) {
      auto value = command(action, 2, 60); value.axes.surge = 100;
      check(state.accept(value, 60).code == Code::OutputFault && !state.leaseActive,
          "latched fault rejects client attempts to restart or reauthorize");
    }
    state.stop();
    check(state.outputFault, "STOP cannot reset the latched hardware-output fault");
  }
}
int main() {
  static_assert(std::is_trivially_copyable_v<Command> && std::is_trivially_copyable_v<Result>);
  boundaries(); absoluteTargetsAndModes(); orderingAndSessions(); rejection(); requestDeadlines();
  outputDegradation();
  std::printf("PASS: %u Wi-Fi control checks (absolute targets, 3s lease/wrap, sessions, sequence/STOP replay barrier, validation, OTA lock, output degradation/recovery/fault)\n", checks);
}
