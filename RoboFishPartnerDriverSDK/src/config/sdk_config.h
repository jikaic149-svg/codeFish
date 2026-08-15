#pragma once

// ============================================================================
// RoboFish Partner Driver SDK configuration
// This file is intentionally readable and editable by the partner.
// It contains hardware mapping and service credentials, but no swimming logic.
// ============================================================================

// ESP-IDF startup and control-loop cadence. Units: milliseconds.
// The boot delay gives the USB Serial/JTAG console time to enumerate. The main
// loop delay keeps driver timeout polling at the original Arduino cadence.
#define RF_BOOT_SERIAL_DELAY_MS   1000
#define RF_MAIN_LOOP_DELAY_MS       10

// XIAO ESP32S3 actuator pins (matching the existing RoboFish hardware).
#define RF_TAIL_SERVO_1_PIN   5
#define RF_TAIL_SERVO_2_PIN   6
#define RF_PITCH_SERVO_PIN    4
#define RF_PUMP_PIN           9
#define RF_ESC_THROTTLE_PIN  43
#define RF_ESC_STEERING_PIN  44

// Calibrated neutral positions and conservative software limits.
#define RF_TAIL_CENTER          86
#define RF_TAIL_MIN_ANGLE       20
#define RF_TAIL_MAX_ANGLE      152

// Rear-tail phase lag relative to the front joint, in waveform cycles.
// The existing generator evaluates sin(phase + offset), so offsets above 0.5
// encode a negative phase: 0.8333 == -60 degrees and 0.80 == -72 degrees.
#define RF_BIONIC_TAIL_PHASE_LAG_CYCLES 0.83333333f
#define RF_HYBRID_TAIL_PHASE_LAG_CYCLES 0.80f

// Tail oscillation frequency in hertz. Higher values produce faster, sharper
// motion and require more per-tick servo travel.
#define RF_BIONIC_TAIL_FREQUENCY_HZ 1.7f
#define RF_HYBRID_TAIL_FREQUENCY_HZ 2.0f

// Maximum servo-angle change per 20 ms control tick. These limits track the
// 75% web-command waveform without rate clipping at the configured frequency.
#define RF_BIONIC_TAIL_SLEW_DEG_PER_TICK  8.0f
#define RF_HYBRID_TAIL_SLEW_DEG_PER_TICK 11.0f

// Constant steering bias applied to both tail joints at 100% turn strength.
// Larger values tighten turns while the oscillation amplitude stays unchanged.
#define RF_BIONIC_TAIL_TURN_BIAS_DEG 18.0f
#define RF_HYBRID_TAIL_TURN_BIAS_DEG 18.0f

#define RF_PITCH_CENTER         86
#define RF_PITCH_UI_NEUTRAL     90
#define RF_PITCH_MIN_ANGLE      45
#define RF_PITCH_MAX_ANGLE     135
#define RF_ESC_THROTTLE_NEUTRAL 90
#define RF_ESC_THROTTLE_MIN     35
#define RF_ESC_THROTTLE_MAX    150
#define RF_ESC_STEERING_CENTER  90
#define RF_ESC_STEERING_MIN     40
#define RF_ESC_STEERING_MAX    136
#define RF_ESC_PWM_MIN_US      900
#define RF_ESC_PWM_MAX_US     2100

// Propeller steering offsets at 100% turn strength, in servo-angle degrees.
// The positive/right PWM side has a measured weak response, so it receives an
// affine dead-zone compensation: any non-zero right command starts at 2 deg
// and reaches a higher calibrated maximum. Left commands remain unchanged.
// All values must remain within the configured steering min/max limits.
#define RF_PROP_TURN_MAX_OFFSET_DEG        32
#define RF_HYBRID_PROP_TURN_MAX_OFFSET_DEG 26
#define RF_PROP_RIGHT_TURN_DEADZONE_OFFSET_DEG  2
#define RF_PROP_RIGHT_TURN_MAX_OFFSET_DEG      35
#define RF_HYBRID_PROP_RIGHT_TURN_MAX_OFFSET_DEG 25

// Web manual-control power defaults, in percent (valid range: 0..100).
// BIONIC and HYBRID use the stronger tail setting with matching slew limits;
// pure PROP has no CPG waveform and keeps the full throttle command.
#define RF_MANUAL_TAIL_POWER_PERCENT  75
#define RF_MANUAL_PROP_POWER_PERCENT 100
#define RF_MANUAL_PROP_TURN_POWER_PERCENT 80

// ESC output shaping per 20 ms control tick. HYBRID accelerates away from
// neutral more slowly to reduce startup reaction torque; deceleration, STOP,
// and direction changes return toward neutral at the normal throttle rate.
#define RF_ESC_THROTTLE_SLEW_DEG_PER_TICK         3
#define RF_HYBRID_ESC_ACCEL_SLEW_DEG_PER_TICK     1
#define RF_ESC_STEERING_SLEW_DEG_PER_TICK          8
#define RF_HYBRID_ESC_FORWARD_GAIN_DEG             52
#define RF_HYBRID_ESC_REVERSE_GAIN_DEG             46

// Current relay board is active-low.
#define RF_PUMP_ACTIVE_HIGH false

// Safety timeouts. The partner may study and adjust these values, but disabling
// them increases the risk of a runaway propeller or continuously running pump.
#define RF_PROPULSION_TIMEOUT_MS 2000
#define RF_PUMP_MAX_ON_MS       10000

// Computer-vision control lease. The PC must refresh /vision_control within
// this many milliseconds; otherwise propulsion stops and steering centers.
#define RF_VISION_COMMAND_TIMEOUT_MS 500

// MCU-side visual control tuning. Inputs are metres.  The LOS heading error
// is already filtered and latency-compensated on the PC, so X uses P only;
// adding I/D here amplifies fish-tail marker swing and delayed samples.
// 300 %/m maps the visual +/-0.18 m limit to about +/-54% steering.
#define RF_VISION_X_KP 300.0f
#define RF_VISION_X_KI   0.0f
#define RF_VISION_X_KD   0.0f
// 800 %/m reaches full straight power at about 0.125 m. This keeps normal
// tracking firm while retaining a short proportional slowdown near the goal.
#define RF_VISION_Y_KP 800.0f
#define RF_VISION_Y_KI   0.0f
#define RF_VISION_Y_KD   0.0f
#define RF_VISION_PID_INTEGRAL_LIMIT 0.50f
#define RF_VISION_PID_DEADBAND_M     0.006f

// Vision HYBRID open-loop curve slowdown.  Full bends reduce both propulsion
// sources; measured camera speed is deliberately not closed around throttle
// because its latency would create another oscillating loop.  Tail steering
// bias remains active at 60% power.  Terminal braking keeps the existing
// neutral/dwell/current-limited reverse sequence.
#define RF_VISION_TAIL_STRAIGHT_POWER_PERCENT       74
#define RF_VISION_TAIL_TURN_POWER_PERCENT           60
#define RF_VISION_PROP_CURVE_MIN_POWER_PERCENT      20
#define RF_VISION_BRAKE_REVERSE_POWER_PERCENT       26
#define RF_VISION_BRAKE_NEUTRAL_TOLERANCE_DEG        1
#define RF_VISION_BRAKE_NEUTRAL_DWELL_MS           160
#define RF_VISION_BRAKE_REVERSE_MIN_MS              120
#define RF_VISION_BRAKE_REVERSE_MAX_MS              480
#define RF_VISION_BRAKE_SETTLED_SPEED_MPS          0.03f

// ESP-IDF HTTP server task. ESP-IDF expresses its stack size in bytes.
// Priority and core preserve the previous background OTA execution context.
#define RF_OTA_TASK_STACK_BYTES 6144
#define RF_OTA_TASK_PRIORITY       2
#define RF_OTA_TASK_CORE           0
#define RF_OTA_MAX_IMAGE_BYTES 0x330000

// The SDK always starts a local access point so OTA remains reachable without
// a previously configured router. Web OTA: http://192.168.4.1/update
#define RF_AP_SSID     "RoboFish-ND1"
#define RF_AP_PASSWORD "12345678"
#define RF_AP_CHANNEL  1
#define RF_AP_MAX_CONNECTIONS 4

// Optional institution/router Wi-Fi. Leave empty to use AP mode only.
#define RF_STA_SSID     ""
#define RF_STA_PASSWORD ""

// These credentials protect against accidental uploads on the local network.
// Because this SDK source is delivered to the partner, they are not secret from
// the partner and are not an intellectual-property protection mechanism.
#define RF_OTA_USERNAME "admin"
#define RF_OTA_PASSWORD "RoboFishSDK2026"
