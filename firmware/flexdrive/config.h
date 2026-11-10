#pragma once
// =============================================================================
//  flexDrive firmware v2 — central configuration
//  All pins, physical constants and timing live here.
// =============================================================================
#include <Arduino.h>

// ---- Firmware identity ------------------------------------------------------
#define FW_NAME    "flexDrive"
#define FW_VERSION "2.0.0-dev"

// ---- Serial link to PC / web page --------------------------------------------
constexpr uint32_t SERIAL_BAUD = 921600;   // CH9102 USB-UART; needed for 1 kHz capture dumps

// ---- Encoders -----------------------------------------------------------------
// Counts per revolution are after x4 quadrature decoding.

// Encoder 1: 4096 counts per revolution
constexpr uint8_t ENC1_PIN_A = 32;
constexpr uint8_t ENC1_PIN_B = 33;
constexpr float   ENC1_CPR   = 4096.0f;

// Encoder 2: motor encoder, 48 counts per motor revolution x 20.4 gear ratio
// -> counts per revolution of the gearbox output shaft.
// NOTE: GPIO36..39 are input-only and have NO internal pull-ups.
//       If the encoder has open-collector outputs, external pull-ups are required.
constexpr uint8_t ENC2_PIN_A = 36;
constexpr uint8_t ENC2_PIN_B = 37;
constexpr float   ENC2_CPR   = 48.0f * 20.4f;

// ---- Motor driver (H-bridge: PWM + direction) ------------------------------------
constexpr uint8_t  MOTOR_PWM_PIN  = 26;
constexpr uint8_t  MOTOR_DIR_PIN  = 25;
constexpr uint32_t MOTOR_PWM_FREQ = 10000;   // [Hz]
constexpr uint8_t  MOTOR_PWM_BITS = 12;      // duty 0..4095
constexpr int      MOTOR_PWM_MAX  = 4000;    // command limit (signed: -MAX..MAX)

// ---- Sampling -------------------------------------------------------------------
constexpr uint32_t SAMPLE_PERIOD_MS    = 1;     // 1 kHz sampling + observer update
constexpr uint32_t CAPTURE_MAX_SAMPLES = 5000;  // 1 kHz capture buffer (5 s)
constexpr uint32_t TELEMETRY_PERIOD_MS = 20;    // 50 Hz telemetry stream

// ---- Luenberger velocity observer (see velocity_observer.h) --------------------
// Higher bandwidth -> faster response, more quantization noise in velocity.
// Defaults from live step tests (1 kHz capture, PWM steps incl. reversal), chosen as the
// lowest f whose total error is within 15 % of the minimum (same rule as the web tool):
//   enc 1 (4096 CPR): 70 Hz -> tracking 0.037, quantization noise 0.013 turn/s RMS
//   enc 2 (979 CPR):  40 Hz -> tracking 0.074, quantization noise 0.023 turn/s RMS
// The coarser encoder gets a lower bandwidth because its quantization noise is ~4x larger.
// Re-tune in the web page, tab "Nastavljanje observerja".
constexpr float OBS_BW_DEFAULT[2] = {70.0f, 40.0f};   // [Hz] per encoder
constexpr float OBS_BW_MIN       = 0.1f;
constexpr float OBS_BW_MAX       = 200.0f;
constexpr float OBS_ZETA_DEFAULT = 0.707f;  // ~25 % less lag than zeta=1 at equal noise
constexpr float OBS_ZETA_MIN     = 0.3f;
constexpr float OBS_ZETA_MAX     = 3.0f;

// ---- FreeRTOS tasks ---------------------------------------------------------------
// Real-time work runs on core 1 (Arduino core); core 0 is left for WiFi later.
constexpr BaseType_t RT_CORE            = 1;
constexpr UBaseType_t SAMPLER_PRIORITY  = 5;   // highest: timing-critical
constexpr UBaseType_t TELEMETRY_PRIORITY = 2;
