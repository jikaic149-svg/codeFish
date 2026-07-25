/*
 * CameraTest.ino.bin — recovered high-confidence pseudocode
 *
 * This is documentation, not original or buildable source. Names and types
 * are assigned from behavior. Addresses in comments are binary evidence.
 */

enum class DriveMode : unsigned {
    BIONIC = 1,
    PROP = 2,
    HYBRID = 3,
};

struct MotionState {
    volatile bool forward;
    volatile bool backward;
    volatile bool turning;
    DriveMode mode;
    unsigned char bionicPowerPercent;
    unsigned char propPowerPercent;
    float tail1Target;
    float tail2Target;
    int escThrottleTarget;
    int escSteeringTarget;
};

static MotionState motion;
static bool failsafeActive;
static unsigned lastLinkCheckMs;
static unsigned disconnectedSinceMs;

// 0x420036dc
static void onControlCommand() {
    if (!failsafeActive) return;
    failsafeActive = false;
    Serial.println("[FAILSAFE] 检测到新控制指令 → 退出自动上浮，恢复正常操控");
    updateModeLed();
}

// 0x42003cd8, name assigned from side effects
static void emergencyStop() {
    motion.forward = false;
    motion.backward = false;
    motion.turning = false;
    motion.tail1Target = 86.0f;
    motion.tail2Target = 86.0f;
    motion.escThrottleTarget = 90;
    motion.escSteeringTarget = 90;
}

// 0x42003d14
static void triggerFailsafe(const char *reason) {
    if (failsafeActive) return;
    failsafeActive = true;
    Serial.printf("[FAILSAFE] 触发 (%s) → 紧急停止 + 头部上扬自由上浮\n", reason);
    emergencyStop();
    setPitch(45);
    setPump(false);                 // GPIO 9 is active-low
    setRgbAll(255, 120, 0);         // eight pixels
}

// 0x42003d60
static void failsafeWatchdog() {
    const unsigned now = millis();
    if (now - lastLinkCheckMs <= 499) return;
    lastLinkCheckMs = now;
    if (now - bootTimeMs <= 14999) return;

    const bool apPresent = softApStationCount() != 0;
    const bool staPresent = wifiStatus() == WL_CONNECTED;
    if (apPresent || staPresent) {
        disconnectedSinceMs = 0;
    } else {
        if (disconnectedSinceMs == 0) disconnectedSinceMs = now;
        if (!failsafeActive && now - disconnectedSinceMs > 3000)
            triggerFailsafe("WiFi双链路断开");
    }

    if (failsafeActive) {
        updateFailsafeLedPulse(now);
        if (currentPitch != 45) setPitch(45);
    }
}

// 0x42003708 — mathematical variable names are approximate.
static void controlTick() {
    if (!motion.forward && !motion.backward && !motion.turning) {
        motion.tail1Target = 86.0f;
        motion.tail2Target = 86.0f;
        slewTailOutputs();
        slewEscOutputs();
        return;
    }

    if (motion.forward) {
        if (motion.mode == DriveMode::BIONIC || motion.mode == DriveMode::HYBRID) {
            const float amplitude =
                (motion.mode == DriveMode::HYBRID ? 58.0f : 50.0f) *
                motion.bionicPowerPercent / 100.0f;
            const float phaseRate = motion.mode == DriveMode::HYBRID ? 2.45f : 1.7f;
            // The binary applies sin/fmod plus turning asymmetry here.
            motion.tail1Target = clamp180(86.0f + tailWave1(millis(), phaseRate, amplitude));
            motion.tail2Target = clamp180(86.0f + tailWave2(millis(), phaseRate, amplitude));
        }
        if (motion.mode == DriveMode::PROP || motion.mode == DriveMode::HYBRID) {
            const int gain = motion.mode == DriveMode::HYBRID ? 44 : 36;
            motion.escThrottleTarget = 88 + roundedPercent(gain, motion.propPowerPercent);
        }
    } else if (motion.backward) {
        if (motion.mode == DriveMode::BIONIC || motion.mode == DriveMode::HYBRID)
            computeReverseTailWave();
        if (motion.mode == DriveMode::PROP || motion.mode == DriveMode::HYBRID) {
            const int gain = motion.mode == DriveMode::HYBRID ? 42 : 34;
            motion.escThrottleTarget = 88 - roundedPercent(gain, motion.propPowerPercent);
        }
    }

    applyTurningMix();
    slewTailOutputs(motion.mode == DriveMode::HYBRID ? 10.0f : 6.0f);
    slewEscOutputs(motion.mode == DriveMode::HYBRID ? 4 : 3);
}

// 0x42003a8c
static void CPGTask(void *) {
    Serial.println("CPG Task Started!");
    for (;;) {
        controlTick();
        delay(20);
    }
}

enum DemoState : unsigned { DEMO_IDLE, DEMO_FORWARD, DEMO_DIVE, DEMO_SURFACE, DEMO_STOP };
static DemoState demoState;
static unsigned demoStateStartedMs;

// 0x42003f18
static const char *requestDemo() {
    if (demoState != DEMO_IDLE) {
        cancelDemo(true);
        return "DEMO_CANCELLED";
    }
    if (failsafeActive) return "DEMO_BLOCKED_FAILSAFE";
    if (motion.mode != DriveMode::BIONIC) setDriveMode(DriveMode::BIONIC);
    demoState = DEMO_FORWARD;
    demoStateStartedMs = millis();
    setPitch(90);
    setForward(true);
    return "DEMO_STARTED";
}

// 0x42003f9c
static void demoTick() {
    if (demoState == DEMO_IDLE) return;
    if (failsafeActive) { cancelDemo(false); return; }
    const unsigned now = millis();
    const unsigned elapsed = now - demoStateStartedMs;
    switch (demoState) {
    case DEMO_FORWARD:
        if (elapsed > 3999) { demoState = DEMO_DIVE; demoStateStartedMs = now; setPitch(135); }
        break;
    case DEMO_DIVE:
        if (elapsed > 3499) { demoState = DEMO_SURFACE; demoStateStartedMs = now; emergencyStop(); setPitch(45); }
        break;
    case DEMO_SURFACE:
        if (elapsed > 3999) { demoState = DEMO_STOP; demoStateStartedMs = now; emergencyStop(); setPitch(90); }
        break;
    case DEMO_STOP:
        if (elapsed > 1199) { demoState = DEMO_IDLE; Serial.println("[DEMO] 演示结束"); }
        break;
    default:
        demoState = DEMO_IDLE;
    }
}

struct PartnerStep {
    Action action;
    unsigned durationMs;
    int angle;
    int power;
    int strength;
};

// 0x420070c8 and callers — condensed parser contract.
static bool validatePartnerScript(const char *json, size_t length, Error &error) {
    if (length < 1 || length > 3500) return error("Script size must be 1..3500 bytes");
    parseStrictJsonObject(json);
    requireIntegerRange("repeat", 1, 10);
    requireArrayLength("steps", 1, 32);
    rejectUnknownFields();
    for (const PartnerStep &step : parsedSteps) {
        requireKnownAction(step.action);
        requireRange(step.durationMs, 0, 30000);
        if (isMotion(step.action)) requireMinimum(step.durationMs, 100);
        if (step.action == PUMP_ON) requireRange(step.durationMs, 100, 10000);
        if (step.action == PITCH) { requireRange(step.angle, 45, 135); requireMinimum(step.durationMs, 100); }
        if (hasPower(step)) requireRange(step.power, 20, 70);
        if (hasStrength(step)) requireRange(step.strength, 10, 70);
    }
    requireTotalDuration(1, 300000);
    return true;
}

// 0x42007438
static void partnerTick() {
    if (!partnerRunning) return;
    if (failsafeActive) { partnerStop(false); return; }
    const unsigned now = millis();
    if (now - stepStartedMs < currentStep().durationMs) return;
    safelyEndCurrentStep();
    if (++stepIndex == stepCount) {
        stepIndex = 0;
        if (++passIndex == repeatCount) {
            partnerRunning = false;
            safelyEndCurrentStep();
            return;
        }
    }
    stepStartedMs = now;
    startStep(currentStep());
}

// 0x420054b0 — error responses omitted for readability.
static esp_err_t otaUpload(httpd_req_t *request) {
    char suppliedPassword[80] = {};
    requireHeader(request, "X-OTA-Password", suppliedPassword, sizeof(suppliedPassword));
    unsigned char digest[32];
    sha256(suppliedPassword, digest);
    if (!constantTimeEqual(digest, embeddedOtaDigest, 32)) return http401();

    emergencyStop();
    const esp_partition_t *partition = esp_ota_get_next_update_partition(nullptr);
    esp_ota_handle_t handle;
    esp_ota_begin(partition, request->content_len_or_partition_size, &handle);
    while (bytesRemain()) {
        int received = httpd_req_recv(request, buffer, sizeof(buffer));
        esp_ota_write(handle, buffer, received);
    }
    esp_ota_end(handle);
    esp_ota_set_boot_partition(partition);
    sendSuccessJson(request);
    delay(1000);
    ESP.restart();
}

// 0x4200447c and 0x42004864
void setup() {
    Serial.begin(115200);
    delay(2000);
    camera_config_t camera = recoveredXiaoSenseCameraConfig(12000000, PIXFORMAT_JPEG);
    if (esp_camera_init(&camera) != ESP_OK) return;
    initCameraLed(21);
    startHttpServersAndRoutes();
    startApAndSavedSta();
    loadPersistentConfiguration();
    initActuatorsSafe();
    xTaskCreatePinnedToCore(CPGTask, "CPGTask", 4096, nullptr, 2, nullptr, 1);
    bootTimeMs = millis();
}

void loop() {
    failsafeWatchdog();
    demoTick();
    partnerTick();
    delay(10);
}
