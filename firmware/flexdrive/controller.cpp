#include "controller.h"
#include "config.h"
#include "motor.h"

namespace {

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

ControlSettings cfg = {
  MODE_OPEN, 0.0f, 0,
  0.0f, 0.0f, 0.0f,
  CTRL_TS_DEFAULT_MS, CTRL_FB_DEFAULT, CTRL_UMAX_DEFAULT, 0,
};

// ---- Controller dynamics --------------------------------------------------------------
float    uI       = 0.0f;    // integral part [%]
float    dFilt    = 0.0f;    // filtered d(v_fb)/dt [turn/s^2]
float    prevV    = 0.0f;
bool     firstRun = true;    // next run initialises prevV and runs immediately
uint32_t tick     = 0;       // samples since the last controller run
int64_t  lastRunUs = 0;
float    uHold    = 0.0f;    // output held between runs (ZOH) [%]
bool     satHold  = false;

// ---- Step test -------------------------------------------------------------------------
bool     stepArmed = false;
uint32_t stepAt    = 0;
float    stepVal   = 0.0f;
uint8_t  stepMode  = MODE_OPEN;

// ---- Reference profile ---------------------------------------------------------------
struct ProfPoint { uint32_t t; float v; };
ProfPoint prof[PROF_MAX_POINTS];
uint32_t  profN      = 0;
bool      profArmed  = false;
bool      profActive = false;
uint32_t  profT      = 0;     // samples since the start [ms]

bool      posInit    = false; // mode 2 entered: take the current angle as ref

float profileValueLocked(uint32_t t) {
  if (t <= prof[0].t) return prof[0].v;
  for (uint32_t i = 1; i < profN; ++i) {
    if (t <= prof[i].t) {
      const float f = (float)(t - prof[i - 1].t) / (float)(prof[i].t - prof[i - 1].t);
      return prof[i - 1].v + f * (prof[i].v - prof[i - 1].v);
    }
  }
  return prof[profN - 1].v;
}

// ---- Safety ----------------------------------------------------------------------------
uint32_t overspeedSamples = 0;
uint32_t runawaySamples   = 0;
float    runawayStartV    = 0.0f;
bool     encWinValid      = false;
uint32_t encWinStartMs    = 0;
uint32_t encWinErrors     = 0;
uint32_t lastFeedMs       = 0;
bool     wdTripped        = false;
ControlFault pendingFault = FAULT_NONE;

ControlOutput latestOut = {};

void resetDynamicsLocked() {
  uI = 0.0f; dFilt = 0.0f; prevV = 0.0f;
  firstRun = true; tick = 0; lastRunUs = 0;
  uHold = 0.0f; satHold = false;
  overspeedSamples = 0; runawaySamples = 0;
  encWinValid = false;
}

void stopLocked() {
  cfg.mode  = MODE_OPEN;
  cfg.ref   = 0.0f;
  cfg.pwm   = 0;
  stepArmed = false;
  profArmed = false; profActive = false;
  posInit   = false;
  resetDynamicsLocked();
}

inline int pctToPwm(float u) { return (int)lroundf(u * (float)MOTOR_PWM_MAX / 100.0f); }

// Closed-loop safety checks; returns a fault or FAULT_NONE. Called with the lock held.
ControlFault checkSafetyLocked(const Measurement& m, uint32_t nowMs, float v, int fbIdx) {
  // Overspeed
  overspeedSamples = (fabsf(v) > SAFETY_V_LIMIT) ? overspeedSamples + 1 : 0;
  if (overspeedSamples * SAMPLE_PERIOD_MS > SAFETY_OVERSPEED_MS) return FAULT_OVERSPEED;

  // Runaway: saturated output, shaft turning against it and speeding up.
  const bool against = satHold && (uHold * v < 0.0f) && fabsf(v) > SAFETY_RUNAWAY_V;
  if (!against) {
    runawaySamples = 0;
  } else {
    if (runawaySamples == 0) runawayStartV = fabsf(v);
    ++runawaySamples;
    if (runawaySamples * SAMPLE_PERIOD_MS >= SAFETY_RUNAWAY_MS &&
        fabsf(v) > runawayStartV + SAFETY_RUNAWAY_V) return FAULT_RUNAWAY;
  }

  // Missed encoder edges within a window
  const uint32_t errs = m.errors[fbIdx];
  if (!encWinValid || nowMs - encWinStartMs >= SAFETY_ENC_WINDOW_MS) {
    encWinValid = true; encWinStartMs = nowMs; encWinErrors = errs;
  } else if (errs - encWinErrors > SAFETY_ENC_ERR_MAX) {
    return FAULT_ENCODER;
  }
  return FAULT_NONE;
}

}  // namespace

// =============================================================================
//  Sampler side
// =============================================================================
ControlOutput controllerUpdate(const Measurement& m, int64_t nowUs, int32_t capIdx) {
  const uint32_t nowMs = (uint32_t)(nowUs / 1000);
  ControlFault fault = FAULT_NONE;
  ControlOutput out;

  portENTER_CRITICAL(&mux);

  // ---- Step: applied exactly in capture sample `stepAt` -------------------------------
  bool force = false;
  if (stepArmed && capIdx >= 0 && (uint32_t)capIdx == stepAt) {
    stepArmed = false;
    if (stepMode == cfg.mode) {
      if (cfg.mode == MODE_OPEN) {
        cfg.pwm = constrain((int)lroundf(stepVal), -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
      } else {
        cfg.ref = stepVal;
        force = true;   // compute the new output in this same sample
      }
    }
  }

  // ---- Watchdog (all modes) --------------------------------------------------------------
  if (cfg.wdMs > 0 && !wdTripped && nowMs - lastFeedMs > cfg.wdMs) {
    wdTripped = true;
    stopLocked();
    fault = FAULT_WATCHDOG;
  }

  // ---- Reference profile: starts in capture sample 0, advances 1 ms per sample ---------
  if (profArmed && capIdx == 0) {
    profArmed = false; profActive = true; profT = 0;
    force = true;
  }
  int32_t profMs = -1;
  if (profActive) {
    cfg.ref = profileValueLocked(profT);
    profMs  = (int32_t)profT;
    if (profT >= prof[profN - 1].t) profActive = false;   // last value stays as ref
    else ++profT;
  }

  const int   fbIdx  = (cfg.fb == 2) ? 1 : 0;
  const float v      = m.velocity[fbIdx];
  const float cprFb  = (fbIdx == 0) ? ENC1_CPR : ENC2_CPR;
  const float angle  = (float)m.count[fbIdx] * 360.0f / cprFb;   // [deg] since "zero"
  const float vDeg   = v * 360.0f;                                // [deg/s]
  const bool  closed = (cfg.mode != MODE_OPEN);

  if (cfg.mode == MODE_POS && posInit) { cfg.ref = angle; posInit = false; }

  // ---- Closed-loop safety ------------------------------------------------------------------
  if (closed) {
    const ControlFault f = checkSafetyLocked(m, nowMs, v, fbIdx);
    if (f != FAULT_NONE) { stopLocked(); fault = f; }
  }

  // ---- Control law ------------------------------------------------------------------------
  int pwmCmd;
  if (cfg.mode != MODE_OPEN) {   // re-read: a safety stop above may have opened the loop
    const bool pos = (cfg.mode == MODE_POS);
    ++tick;
    if (force || firstRun || tick >= cfg.tsMs / SAMPLE_PERIOD_MS) {
      tick = 0;
      const float dt = (lastRunUs != 0) ? (float)(nowUs - lastRunUs) * 1e-6f
                                        : (float)cfg.tsMs * 1e-3f;
      lastRunUs = nowUs;
      if (firstRun) { prevV = v; dFilt = pos ? vDeg : 0.0f; firstRun = false; }

      // D on measurement, first-order filtered.
      // Mode 1: derivative of the velocity. Mode 2: the observer velocity already is the
      // derivative of the angle, so it is only filtered.
      const float dIn = pos ? vDeg : (v - prevV) / dt;
      prevV = v;
      dFilt += (dt / (CTRL_TD_FILTER_MS * 1e-3f + dt)) * (dIn - dFilt);

      const float e   = cfg.ref - (pos ? angle : v);
      const float uP  = cfg.kp * e;
      const float uD  = -cfg.kd * dFilt;
      const float uIc = uI + cfg.ki * e * dt;

      // Conditional integration: do not integrate further into saturation.
      const float uRaw   = uP + uIc + uD;
      const bool  windup = (uRaw > cfg.uMax && e > 0.0f) || (uRaw < -cfg.uMax && e < 0.0f);
      if (!windup) uI = uIc;
      uI = constrain(uI, -cfg.uMax, cfg.uMax);

      const float u = uP + uI + uD;
      satHold = fabsf(u) > cfg.uMax;
      uHold   = constrain(u, -cfg.uMax, cfg.uMax);
    }
    pwmCmd = pctToPwm(uHold);
    out.uPct = uHold;
    out.uI   = uI;
    out.sat  = satHold;
  } else {
    pwmCmd   = cfg.pwm;
    out.uPct = (float)pwmCmd * 100.0f / (float)MOTOR_PWM_MAX;
    out.uI   = 0.0f;
    out.sat  = false;
  }

  out.mode  = cfg.mode;
  out.pwm   = pwmCmd;
  out.ref    = cfg.ref;
  out.profMs = profMs;
  latestOut  = out;
  if (fault != FAULT_NONE) pendingFault = fault;

  portEXIT_CRITICAL(&mux);

  if (pwmCmd != motorPwm()) motorSetPwm(pwmCmd);
  return out;
}

// =============================================================================
//  Command side
// =============================================================================
ControlSettings controllerSettings() {
  portENTER_CRITICAL(&mux);
  const ControlSettings s = cfg;
  portEXIT_CRITICAL(&mux);
  return s;
}

ControlOutput controllerLatest() {
  portENTER_CRITICAL(&mux);
  const ControlOutput o = latestOut;
  portEXIT_CRITICAL(&mux);
  return o;
}

void controllerSetMode(uint8_t mode) {
  portENTER_CRITICAL(&mux);
  if (mode != cfg.mode) {
    cfg.mode  = mode;
    cfg.pwm   = 0;
    stepArmed = false;
    profArmed = false; profActive = false;
    posInit   = (mode == MODE_POS);
    resetDynamicsLocked();
  }
  portEXIT_CRITICAL(&mux);
}

void controllerSetRef(float ref) {
  portENTER_CRITICAL(&mux);
  cfg.ref = ref;
  posInit = false;
  profArmed = false; profActive = false;
  portEXIT_CRITICAL(&mux);
}

bool controllerSetOpenPwm(int pwm) {
  portENTER_CRITICAL(&mux);
  const bool ok = (cfg.mode == MODE_OPEN);
  if (ok) cfg.pwm = pwm;
  portEXIT_CRITICAL(&mux);
  return ok;
}

void controllerSetKp(float v) {
  portENTER_CRITICAL(&mux);
  cfg.kp = v;
  portEXIT_CRITICAL(&mux);
}

void controllerSetKi(float v) {
  portENTER_CRITICAL(&mux);
  cfg.ki = v;
  if (v == 0.0f) uI = 0.0f;   // pure P: no frozen offset left behind
  portEXIT_CRITICAL(&mux);
}

void controllerSetKd(float v) {
  portENTER_CRITICAL(&mux);
  cfg.kd = v;
  portEXIT_CRITICAL(&mux);
}

void controllerSetTs(uint32_t ms) {
  portENTER_CRITICAL(&mux);
  cfg.tsMs = ms;
  tick = 0;
  portEXIT_CRITICAL(&mux);
}

void controllerSetFeedback(uint8_t fb) {
  portENTER_CRITICAL(&mux);
  if (fb != cfg.fb) {
    cfg.fb = fb;
    firstRun = true;            // re-initialise the D-term history
    encWinValid = false;
    overspeedSamples = 0; runawaySamples = 0;
  }
  portEXIT_CRITICAL(&mux);
}

void controllerSetUMax(float pct) {
  portENTER_CRITICAL(&mux);
  cfg.uMax = pct;
  portEXIT_CRITICAL(&mux);
}

void controllerSetWatchdog(uint32_t ms, uint32_t nowMs) {
  portENTER_CRITICAL(&mux);
  cfg.wdMs   = ms;
  lastFeedMs = nowMs;
  wdTripped  = false;
  portEXIT_CRITICAL(&mux);
}

void controllerStop() {
  portENTER_CRITICAL(&mux);
  stopLocked();
  portEXIT_CRITICAL(&mux);
}

void controllerArmStep(uint32_t pre, float val) {
  portENTER_CRITICAL(&mux);
  stepArmed = true;
  stepAt    = pre;
  stepVal   = val;
  stepMode  = cfg.mode;
  portEXIT_CRITICAL(&mux);
}

void controllerDisarmStep() {
  portENTER_CRITICAL(&mux);
  stepArmed = false;
  portEXIT_CRITICAL(&mux);
}

void controllerFeedWatchdog(uint32_t nowMs) {
  portENTER_CRITICAL(&mux);
  lastFeedMs = nowMs;
  wdTripped  = false;
  portEXIT_CRITICAL(&mux);
}

void controllerProfileClear() {
  portENTER_CRITICAL(&mux);
  profN = 0; profArmed = false; profActive = false;
  portEXIT_CRITICAL(&mux);
}

bool controllerProfileAdd(uint32_t tMs, float val) {
  portENTER_CRITICAL(&mux);
  const bool ok = !profActive && !profArmed && profN < PROF_MAX_POINTS &&
                  (profN == 0 || tMs > prof[profN - 1].t);
  if (ok) prof[profN++] = { tMs, val };
  portEXIT_CRITICAL(&mux);
  return ok;
}

uint32_t controllerProfileCount() {
  portENTER_CRITICAL(&mux);
  const uint32_t n = profN;
  portEXIT_CRITICAL(&mux);
  return n;
}

float controllerProfileMaxAbs() {
  portENTER_CRITICAL(&mux);
  float mx = 0.0f;
  for (uint32_t i = 0; i < profN; ++i) mx = fmaxf(mx, fabsf(prof[i].v));
  portEXIT_CRITICAL(&mux);
  return mx;
}

void controllerArmProfile() {
  portENTER_CRITICAL(&mux);
  if (profN > 0) { profArmed = true; profActive = false; posInit = false; }
  portEXIT_CRITICAL(&mux);
}

void controllerDisarmProfile() {
  portENTER_CRITICAL(&mux);
  profArmed = false;
  portEXIT_CRITICAL(&mux);
}

ControlFault controllerTakeFault() {
  portENTER_CRITICAL(&mux);
  const ControlFault f = pendingFault;
  pendingFault = FAULT_NONE;
  portEXIT_CRITICAL(&mux);
  return f;
}

const char* controllerFaultName(ControlFault f) {
  switch (f) {
    case FAULT_OVERSPEED: return "overspeed";
    case FAULT_RUNAWAY:   return "runaway";
    case FAULT_ENCODER:   return "encoder";
    case FAULT_WATCHDOG:  return "watchdog";
    default:              return "none";
  }
}
