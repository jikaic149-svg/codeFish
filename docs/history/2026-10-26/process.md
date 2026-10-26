# 2026-10-26 开发记录

来源：归档前根目录 `process.md`。以下正文按原文保留；历史决策只适用于各自版本和试验条件。

<!-- original-process-entry: 066 sha256: 95db452084e050f2e320dba15f358a017e9d4b7b5e2b9a9c3a048665542287b5 -->
## 2026-10-26: continuous tracking and controlled hydrodynamic trials

14:52 trace review found a PC imu_unavailable STOP at 14:51:27.823 while fish IMU remained valid. The 107-degree restart spike came from path progress reset/local projection, not an IMU flip. Active-only PC zero-power count is 1/207; did not add minimum propulsion. WebBridge keeps original timestamps during transport gaps; a bounded 250ms zero-output observation pause retains authorization and path, while persistent invalidity/manual STOP/takeover/reboot cannot resume automatically. Camera receipt freshness is separate from the estimated 300ms exposure delay; sample timestamps still drive speed derivatives and logging. ACK >1s and pause history are explicit.

PathGuidance initializes from global path projection and initial heading, keeps valid progress, applies bounded curvature lookahead to the actual heading target, filters lateral error and limits heading-reference slew. Trial values: straight .12m/s, tight-curve base .05, minimum target .04; existing PID/firmware unchanged. Geometric target radius .60m is used only when it fits within .15m offset; constrained U turns keep the original path and report achieved curvature/unachievable, never silently use a tighter fillet. Actual velocity-vector projections/course/sideslip are logged; main speed feedback is unchanged.

The supplied yaw_map.py adds absolute prop differential to tail bias, so its 80-percent point and 67-percent extrapolation are not single-tail-bias measurements; pure-tail drive/bias are also confounded. Added an explicit static-tail test recipe: no oscillation/differential, 0/40/60/80/100 bias presets, fixed prop common. During manual hold X/Z apply +/-60/0 bias steps without changing common propulsion; release remains full STOP and coast recording. No automatic test duration, assumed tau, or fictitious hydrodynamic envelope.

Workstation revision33. 23 focused no-hardware tests plus Node tail-step invariants and syntax/diff checks passed. No camera/serial/fish commands or new firmware build. New water performance remains unverified; kinematic replay cannot establish a 10cm physical tracking error. Existing 2.7.4 image is retained.


<!-- original-process-entry: 067 sha256: cbf83032e3d75f5f62957194d7ceee38725fb06fb5f588b67460932bf97e7593 -->
## 2026-10-26: restore and strengthen tail gait

User explicitly corrected the static-tail interpretation: tracking must use oscillating tail propulsion as well as common prop thrust. Heading tail-drive ratio is now 1.0; yaw shifts its centre without increasing drive/frequency or restoring the 35-percent floor. Replaced pure-prop forward references with the measured 1:1 hybrid points (.05734369/.14829029/.23487840m/s at 10/15/20 percent). New nine angle-radius trials have zero tail drive, left common10/right20 and remain static references, not an oscillating-tail envelope.

After the 16:53 trace (about73s continuous to arrival, median tail drive11.44%) user requested stronger motion. Final 2.7.6 adds amplitude gain1.5 to heading and explicit non-baseline HYBRID calibration; unchanged nominal frequency, bias gains, periods and mechanical/rate limits. Old baseline and static comparison retained. Source forward references were measured at gain1 and are now marked initial estimates, not verified gain1.5 calibration.

Made two always-visible test entry buttons and direct #swim/#static links. Selecting them never sends motion; bias steps preserve each chosen gait/common output. PC revision35 requires the matching firmware. Final image artifacts/RobotFish-ESP32S3-2.7.6-strong-tail.bin: 961344 bytes, SHA256 94c4518c817d1c11e1e2bd5b4e0ee2179ed996dbd29fe27b004ca90880a3d110. ESP-IDF build and image validation,161 native checks,20 PC checks and JS step invariants passed. No physical commands, camera, serial or flashing; delivered first, then documented. New water response remains unverified.


<!-- original-process-entry: 068 sha256: 74babb7a09085be15e99b8fbfa6f83a597f31ee3003d26c391f293e4aea086d2 -->
## 2026-10-26: preserve steering bias with stronger gait

User confirmed the screenshot was captured after 2.7.6. Identified a deterministic joint-travel allocation conflict: front travel +/-32deg, requested full bias24deg plus gain1.5 gait at drive20% (16.77deg half-amplitude) previously scaled both, reducing bias to18.84deg. 2.7.7 gives heading and non-baseline HYBRID calibration bias priority, retaining24deg bias and8deg half-amplitude in that case. Straight gait gain1.5 remains; no new propulsion floor, PID, speed/path gains, mechanical or rate limits. Ordinary manual/pure-tail/pure-prop calibration and old baseline remain on their original allocation.

Motion request preserveTailBias and result effectiveTailBiasDegrees/effectiveTailAmplitudeDegrees make the allocation explicit. Identity/status advertise heading_bias_priority_supported. Allocation telemetry effective_tail_bias_deg/effective_tail_amplitude_deg represents nominal front-joint values after travel limiting but before final angle slew, not measured servo feedback. PC revision36 requires this capability and records these fields in terminal, host telemetry and observation CSV. Path progress/segment is also recorded to distinguish future self-crossing association problems; current screenshot alone does not establish a wrong branch.

ESP-IDF build and image validation passed. Image artifacts/RobotFish-ESP32S3-2.7.7-steering-priority.bin: 961600 bytes; SHA256 20f9f9b30a8958f76242afb09858262b198db965f78295bbe6da05498c612e59. Native checks:193 targeted calibration/allocation,2140 LoRa owner,2161 Wi-Fi owner,60561 existing allocation checks over51840 ticks.20 focused PC checks plus Python/JS syntax and diff whitespace checks passed. Did not start or stop services, open camera/serial, flash, or send fish commands. Existing water measurements remain reference only; new physical tracking response is unverified.

