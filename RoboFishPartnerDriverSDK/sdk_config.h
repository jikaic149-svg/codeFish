#pragma once

// ============================================================================
// RoboFish Partner Driver SDK configuration
// This file is intentionally readable and editable by the partner.
// It contains hardware mapping and service credentials, but no swimming logic.
// ============================================================================

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
#define RF_PITCH_CENTER         90
#define RF_PITCH_MIN_ANGLE      45
#define RF_PITCH_MAX_ANGLE     135
#define RF_ESC_THROTTLE_NEUTRAL 90
#define RF_ESC_THROTTLE_MIN     35
#define RF_ESC_THROTTLE_MAX    150
#define RF_ESC_STEERING_CENTER  88
#define RF_ESC_STEERING_MIN     40
#define RF_ESC_STEERING_MAX    136
#define RF_ESC_PWM_MIN_US      900
#define RF_ESC_PWM_MAX_US     2100

// Current relay board is active-low.
#define RF_PUMP_ACTIVE_HIGH false

// Safety timeouts. The partner may study and adjust these values, but disabling
// them increases the risk of a runaway propeller or continuously running pump.
#define RF_PROPULSION_TIMEOUT_MS 2000
#define RF_PUMP_MAX_ON_MS       10000

// The SDK always starts a local access point so OTA remains reachable without
// a previously configured router. Web OTA: http://192.168.4.1/update
#define RF_AP_SSID     "RoboFish-Partner-SDK"
#define RF_AP_PASSWORD "12345678"

// Optional institution/router Wi-Fi. Leave empty to use AP mode only.
#define RF_STA_SSID     ""
#define RF_STA_PASSWORD ""

// These credentials protect against accidental uploads on the local network.
// Because this SDK source is delivered to the partner, they are not secret from
// the partner and are not an intellectual-property protection mechanism.
#define RF_OTA_USERNAME "admin"
#define RF_OTA_PASSWORD "RoboFishSDK2026"

