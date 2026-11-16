#pragma once
// =============================================================================
//  flexDrive firmware v2 — central configuration
//  All pins, physical constants and timing live here.
// =============================================================================
#include <Arduino.h>

// ---- Firmware identity ------------------------------------------------------
#define FW_NAME    "flexDrive"
#define FW_VERSION "2.2.0"

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

// Count direction per encoder (+1 / -1), applied in encoderCount().
// A positive motor command must give a positive velocity on BOTH encoders, otherwise
// the closed loop pushes the wrong way and runs away. Check with "pwm 1500":
// v1 > 0 and v2 > 0. Flip the sign of the encoder that reads negative.
constexpr int32_t ENC_SIGN[2] = {+1, +1};

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

// ---- Velocity controller (mode 1, see controller.h) ----------------------------
// Units are the same as in the v1 firmware, so last year's gains stay meaningful:
//   ref [turn/s], output u [% of MOTOR_PWM_MAX],
//   Kp [%/(turn/s)], Ki [%/turn] = [%/(turn/s * s)], Kd [% * s/turn]
constexpr float    CTRL_REF_MAX       = 10.0f;    // |ref| limit [turn/s]
constexpr float    CTRL_KP_MAX        = 200.0f;
constexpr float    CTRL_KI_MAX        = 2000.0f;
constexpr float    CTRL_KD_MAX        = 10.0f;
constexpr uint32_t CTRL_TS_DEFAULT_MS = 1;        // controller period, multiple of SAMPLE_PERIOD_MS
constexpr uint32_t CTRL_TS_MAX_MS     = 50;
constexpr float    CTRL_UMAX_DEFAULT  = 100.0f;   // output limit [%]
constexpr float    CTRL_UMAX_MIN      = 1.0f;
constexpr float    CTRL_TD_FILTER_MS  = 5.0f;     // first-order filter on the D term
constexpr uint8_t  CTRL_FB_DEFAULT    = 1;        // feedback encoder: 1 = output shaft (4096 CPR)

// ---- Angle controller (mode 2) ------------------------------------------------------
// Same PID, but on the angle of the feedback encoder relative to the last "zero":
//   ref [deg], Kp [%/deg], Ki [%/(deg*s)], Kd [%*s/deg]; D acts on the observer velocity.
constexpr float    CTRL_POS_REF_MAX   = 3000.0f;  // |ref| limit in mode 2 [deg] (capture stores 0.1 deg in int16)

// ---- Reference profile ("prof" / "run") ----------------------------------------------
// Piecewise-linear reference, time in ms from the start of "run". Used as ref in every
// mode (in mode 0 it is only recorded, e.g. as the target for manual tracking).
constexpr uint32_t PROF_MAX_POINTS    = 24;
constexpr uint32_t PROF_MAX_MS        = 120000;
constexpr uint32_t CAPTURE_DEC_MAX    = 50;       // capture decimation: one sample every n ms

// ---- Step test ("step" command) ---------------------------------------------------
constexpr uint32_t STEP_PRE_DEFAULT = 200;        // samples recorded before the step

// ---- Safety (closed loop only, except the watchdog) -------------------------------
// Measured 2026-11-16 on the lab FlexDrive: 6.2 turn/s at "pwm 4000" (both encoders),
// open-loop dead zone ~6 % (shaft starts at pwm 240). Limit = 1.2x the maximum speed.
constexpr float    SAFETY_V_LIMIT        = 7.5f;   // [turn/s] on the feedback encoder
constexpr uint32_t SAFETY_OVERSPEED_MS   = 50;     // must persist this long to trip
// Runaway (wrong feedback sign): output saturated, shaft turning against the command
// and speeding up. A normal reversal decelerates first, so it does not trip this.
constexpr float    SAFETY_RUNAWAY_V      = 0.5f;   // [turn/s] minimum speed / speed-up
constexpr uint32_t SAFETY_RUNAWAY_MS     = 100;
constexpr uint32_t SAFETY_ENC_ERR_MAX    = 10;     // missed edges allowed per window
constexpr uint32_t SAFETY_ENC_WINDOW_MS  = 100;
constexpr uint32_t WD_MIN_MS             = 200;    // "wd" range (0 = off)
constexpr uint32_t WD_MAX_MS             = 10000;

// ---- FreeRTOS tasks ---------------------------------------------------------------
// Real-time work runs on core 1 (Arduino core); core 0 is left for WiFi later.
constexpr BaseType_t RT_CORE            = 1;
constexpr UBaseType_t SAMPLER_PRIORITY  = 5;   // highest: timing-critical
constexpr UBaseType_t TELEMETRY_PRIORITY = 2;
