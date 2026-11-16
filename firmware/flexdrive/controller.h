#pragma once
// =============================================================================
//  Velocity / angle controller and motor command arbitration
//
//  Modes:
//    0  open loop    motor = open-loop PWM command ("pwm")
//    1  velocity     motor = PID on the observer velocity of the feedback encoder
//    2  angle        motor = PID on the angle of the feedback encoder (servo)
//
//  Control law (modes 1 and 2, run every ts samples, output held in between):
//    mode 1:  y = v_fb [turn/s],                  D input = d(v_fb)/dt
//    mode 2:  y = theta_fb [deg] (since "zero"),  D input = observer velocity [deg/s]
//    e  = ref - y
//    uP = Kp * e                              [%]
//    uI = uI + Ki * e * dt   (conditional integration, |uI| <= u_max)   [%]
//    uD = -Kd * D input      (on measurement, first-order filtered)     [%]
//    u  = sat(uP + uI + uD, +-u_max)          [%]  ->  PWM = u * MOTOR_PWM_MAX / 100
//
//  Entering mode 2 sets ref to the current angle, so the shaft does not jump.
//
//  Reference profile: a piecewise-linear ref(t) loaded with controllerProfileAdd() and
//  started together with a capture (controllerArmProfile). It drives ref in every mode;
//  in mode 0 it is only recorded (target for manual tracking). Setting ref, stop and a
//  mode change end it.
//
//  The integrator stores uI (already scaled by Ki), so changing Ki while running
//  does not make the output jump.
//
//  Threading: controllerUpdate() is called by the sampler task once per 1 kHz
//  sample and is the ONLY place that writes the motor. The command handler
//  (telemetry task) only changes settings through the setters. Both tasks run on
//  RT_CORE; shared state is guarded by a spinlock held for a few microseconds.
// =============================================================================
#include <Arduino.h>
#include "sampler.h"

enum : uint8_t { MODE_OPEN = 0, MODE_CLOSED = 1, MODE_POS = 2 };

enum ControlFault : uint8_t {
  FAULT_NONE = 0,
  FAULT_OVERSPEED,   // |v_fb| > SAFETY_V_LIMIT
  FAULT_RUNAWAY,     // saturated, turning against the command and speeding up
  FAULT_ENCODER,     // too many missed encoder edges on the feedback encoder
  FAULT_WATCHDOG,    // no command from the PC within the watchdog period
};

struct ControlSettings {
  uint8_t  mode;
  float    ref;      // mode 1: [turn/s], mode 2: [deg]
  int      pwm;      // open-loop command
  float    kp, ki, kd;
  uint32_t tsMs;     // controller period [ms]
  uint8_t  fb;       // feedback encoder 1 or 2
  float    uMax;     // [%]
  uint32_t wdMs;     // 0 = watchdog off
};

struct ControlOutput {
  uint8_t mode;
  int     pwm;       // motor command applied in this sample
  float   ref;       // reference in effect (mode 1: turn/s, mode 2: deg)
  float   uPct;      // controller output [%] (open loop: pwm in %)
  float   uI;        // integral part [%]
  bool    sat;       // closed-loop output limited by u_max
  int32_t profMs;    // time since the profile started [ms], -1 if no profile is running
};

// ---- Sampler side -------------------------------------------------------------------
// capIdx: index of the capture sample being recorded in this tick, or -1 if none.
ControlOutput controllerUpdate(const Measurement& m, int64_t nowUs, int32_t capIdx);

// ---- Command side ---------------------------------------------------------------------
ControlSettings controllerSettings();
ControlOutput   controllerLatest();

void controllerSetMode(uint8_t mode);    // on change: resets the controller, pwm = 0, ends profile
void controllerSetRef(float ref);        // stored in all modes, used in 1 and 2; ends profile
bool controllerSetOpenPwm(int pwm);      // false (ignored) in closed loop
void controllerSetKp(float v);
void controllerSetKi(float v);           // Ki = 0 also clears the integral part
void controllerSetKd(float v);
void controllerSetTs(uint32_t ms);
void controllerSetFeedback(uint8_t fb);
void controllerSetUMax(float pct);
void controllerSetWatchdog(uint32_t ms, uint32_t nowMs);
void controllerStop();                   // pwm = 0, ref = 0, mode 0, step disarmed

// Arm a step: when capture sample `pre` is recorded, apply `val` (PWM in mode 0,
// ref in modes 1 and 2) in that same sample. Disarmed by stop or a mode change.
void controllerArmStep(uint32_t pre, float val);
void controllerDisarmStep();

// Reference profile (times strictly increasing, first point usually at t = 0).
void     controllerProfileClear();
bool     controllerProfileAdd(uint32_t tMs, float val);   // false if full or t not increasing
uint32_t controllerProfileCount();
float    controllerProfileMaxAbs();                       // largest |val|, 0 if empty
void     controllerArmProfile();     // starts in capture sample 0 of the next capture
void     controllerDisarmProfile();

void         controllerFeedWatchdog(uint32_t nowMs);   // any received command line
ControlFault controllerTakeFault();                    // returns and clears a pending fault
const char*  controllerFaultName(ControlFault f);
