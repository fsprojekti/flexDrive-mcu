#include "telemetry.h"
#include "config.h"
#include "sampler.h"
#include "controller.h"

namespace {

char   cmdBuf[64];
size_t cmdLen = 0;

// Statically reserved: a 100 kB malloc at runtime fails due to heap fragmentation.
CaptureSample  capStorage[CAPTURE_MAX_SAMPLES];
CaptureSample* capBuffer = nullptr;   // non-null while a capture is running or being dumped
uint32_t       capCount  = 0;

// Settings at the start of the capture, written into cap_begin so a saved CSV is
// self-contained.
struct CaptureMeta {
  ControlSettings ctrl;
  bool     isStep;
  uint32_t stepPre;
  float    stepVal;
};
CaptureMeta capMeta = {};

bool telemetryOn = true;              // "tel 0" silences the stream for manual terminal use

void pollCommands();

// ---------------------------------------------------------------------------
//  Messages
// ---------------------------------------------------------------------------
// Controller settings as JSON key/value pairs (no braces), shared by info/state/cap_begin.
void printCtrlKeys(const ControlSettings& s) {
  Serial.printf("\"mode\":%u,\"ref\":%.6g,\"pwm\":%d,\"kp\":%.6g,\"ki\":%.6g,\"kd\":%.6g,"
                "\"ts\":%lu,\"fb\":%u,\"umax\":%.6g,\"wd\":%lu",
                (unsigned)s.mode, (double)s.ref, s.pwm,
                (double)s.kp, (double)s.ki, (double)s.kd,
                (unsigned long)s.tsMs, (unsigned)s.fb, (double)s.uMax, (unsigned long)s.wdMs);
}

void sendInfo() {
  Serial.printf("{\"type\":\"info\",\"fw\":\"%s\",\"ver\":\"%s\",\"sample_ms\":%lu,"
                "\"tel_ms\":%lu,\"cpr\":[%.1f,%.1f],\"obs_bw\":[%.6g,%.6g],\"obs_zeta\":%.6g,"
                "\"pwm_max\":%d,\"cap_max\":%lu,\"ref_max\":%.6g,\"v_limit\":%.6g,",
                FW_NAME, FW_VERSION,
                (unsigned long)SAMPLE_PERIOD_MS, (unsigned long)TELEMETRY_PERIOD_MS,
                ENC1_CPR, ENC2_CPR,
                (double)samplerObserverBandwidth(0), (double)samplerObserverBandwidth(1),
                (double)samplerObserverDamping(),
                MOTOR_PWM_MAX, (unsigned long)CAPTURE_MAX_SAMPLES,
                (double)CTRL_REF_MAX, (double)SAFETY_V_LIMIT);
  printCtrlKeys(controllerSettings());
  Serial.print("}\n");
}

void sendState() {
  Serial.print("{\"type\":\"state\",");
  printCtrlKeys(controllerSettings());
  Serial.print("}\n");
}

void sendError(const char* msg) {
  Serial.printf("{\"type\":\"err\",\"msg\":\"%s\"}\n", msg);
}

void sendTelemetry(const Measurement& m, const ControlOutput& c) {
  char buf[256];
  const int n = snprintf(buf, sizeof(buf),
      "{\"type\":\"tel\",\"t\":%lu,\"p1\":%ld,\"v1\":%.4f,\"p2\":%ld,\"v2\":%.4f,"
      "\"u\":%d,\"e1\":%lu,\"e2\":%lu,\"mode\":%u,\"ref\":%.4g,\"uc\":%.2f,\"ui\":%.2f,\"sat\":%d}\n",
      (unsigned long)m.timeMs,
      (long)m.count[0], m.velocity[0],
      (long)m.count[1], m.velocity[1],
      c.pwm,
      (unsigned long)m.errors[0], (unsigned long)m.errors[1],
      (unsigned)c.mode, (double)c.ref, (double)c.uPct, (double)c.uI, c.sat ? 1 : 0);
  if (n > 0 && n < (int)sizeof(buf)) Serial.write(reinterpret_cast<const uint8_t*>(buf), n);
}

// Dump a finished capture as CSV between cap_begin / cap_end markers.
// Columns are listed in "fields"; new columns are only ever appended.
void dumpCapture() {
  Serial.printf("{\"type\":\"cap_begin\",\"n\":%lu,\"fields\":\"t_us,c1,c2,v1,v2,pwm,ref,ui\","
                "\"obs_bw\":[%.6g,%.6g],\"obs_zeta\":%.6g,\"cpr\":[%.1f,%.1f],\"ctrl\":{",
                (unsigned long)capCount,
                (double)samplerObserverBandwidth(0), (double)samplerObserverBandwidth(1),
                (double)samplerObserverDamping(),
                ENC1_CPR, ENC2_CPR);
  printCtrlKeys(capMeta.ctrl);
  if (capMeta.isStep) {
    Serial.printf("},\"step\":{\"pre\":%lu,\"val\":%.6g,\"mode\":%u}}\n",
                  (unsigned long)capMeta.stepPre, (double)capMeta.stepVal, (unsigned)capMeta.ctrl.mode);
  } else {
    Serial.print("},\"step\":null}\n");
  }

  char line[112];
  uint32_t tUs = 0;
  for (uint32_t k = 0; k < capCount; ++k) {
    const CaptureSample& s = capBuffer[k];
    tUs += s.dtUs;
    const int n = snprintf(line, sizeof(line), "%lu,%ld,%ld,%.3f,%.3f,%d,%.3f,%.2f\n",
                           (unsigned long)tUs, (long)s.count[0], (long)s.count[1],
                           s.velMilli[0] * 1e-3f, s.velMilli[1] * 1e-3f, (int)s.pwm,
                           s.refMilli * 1e-3f, s.uiCenti * 1e-2f);
    if (n > 0 && n < (int)sizeof(line)) Serial.write(reinterpret_cast<const uint8_t*>(line), n);
    if ((k & 127) == 127) pollCommands();   // keep "stop" responsive during long dumps
  }
  Serial.printf("{\"type\":\"cap_end\",\"n\":%lu}\n", (unsigned long)capCount);
  capBuffer = nullptr;
  capCount  = 0;
}

// ---------------------------------------------------------------------------
//  Argument parsing
// ---------------------------------------------------------------------------
bool parseFloatArg(const char* arg, float lo, float hi, float& out) {
  char* end = nullptr;
  const float v = strtof(arg, &end);
  if (end == arg || !isfinite(v) || v < lo || v > hi) return false;
  out = v;
  return true;
}

bool parseIntArg(const char* arg, long lo, long hi, long& out) {
  char* end = nullptr;
  const long v = strtol(arg, &end, 10);
  if (end == arg || v < lo || v > hi) return false;
  out = v;
  return true;
}

// ---------------------------------------------------------------------------
//  Commands
// ---------------------------------------------------------------------------
// "obs_bw <Hz>"          -> same bandwidth for both encoders
// "obs_bw <Hz1> <Hz2>"   -> per-encoder bandwidth
void cmdObserverBandwidth(const char* arg) {
  char* end = nullptr;
  const float b1 = strtof(arg, &end);
  if (end == arg) { sendError("obs_bw: missing value"); return; }
  const char* rest = end;
  float b2 = strtof(rest, &end);
  if (end == rest) b2 = b1;   // single value -> both
  const bool ok = isfinite(b1) && isfinite(b2) &&
                  b1 >= OBS_BW_MIN && b1 <= OBS_BW_MAX && b2 >= OBS_BW_MIN && b2 <= OBS_BW_MAX;
  if (!ok) { sendError("obs_bw: value out of range"); return; }
  samplerSetObserverBandwidth(0, b1);
  samplerSetObserverBandwidth(1, b2);
  sendInfo();
}

// "obs_zeta <zeta>"  -> observer damping
void cmdObserverDamping(const char* arg) {
  float v;
  if (!parseFloatArg(arg, OBS_ZETA_MIN, OBS_ZETA_MAX, v)) { sendError("obs_zeta: value out of range"); return; }
  samplerSetObserverDamping(v);
  sendInfo();
}

// "pwm <value>"  -> open-loop motor command, signed (mode 0 only)
void cmdPwm(const char* arg) {
  long v;
  if (!parseIntArg(arg, -MOTOR_PWM_MAX, MOTOR_PWM_MAX, v)) { sendError("pwm: value out of range"); return; }
  if (!controllerSetOpenPwm((int)v)) sendError("pwm: closed loop active (use mode 0)");
}

// Reserve the capture buffer and start recording; false (error sent) if busy.
bool startCapture(const char* who, uint32_t n) {
  if (capBuffer != nullptr) {
    Serial.printf("{\"type\":\"err\",\"msg\":\"%s: capture busy\"}\n", who);
    return false;
  }
  capBuffer = capStorage;
  capCount  = n;
  if (!samplerStartCapture(capBuffer, capCount)) {
    capBuffer = nullptr; capCount = 0;
    Serial.printf("{\"type\":\"err\",\"msg\":\"%s: capture busy\"}\n", who);
    return false;
  }
  return true;
}

// "capture <samples>"  -> record samples at 1 kHz, then dump them
void cmdCapture(const char* arg) {
  long n;
  if (!parseIntArg(arg, 1, CAPTURE_MAX_SAMPLES, n)) { sendError("capture: sample count out of range"); return; }
  capMeta = { controllerSettings(), false, 0, 0.0f };
  if (!startCapture("capture", (uint32_t)n)) return;
  Serial.printf("{\"type\":\"cap_start\",\"n\":%lu}\n", (unsigned long)capCount);
}

// "step <val> <n> [pre]"  -> capture n samples; at sample `pre` apply val
//                            (mode 0: PWM, mode 1: ref [turn/s]) in the same sample.
void cmdStep(const char* arg) {
  char* end = nullptr;
  const float val = strtof(arg, &end);
  if (end == arg || !isfinite(val)) { sendError("step: missing value"); return; }
  const char* p = end;
  const long n = strtol(p, &end, 10);
  if (end == p) { sendError("step: missing sample count"); return; }
  p = end;
  long pre = strtol(p, &end, 10);
  if (end == p) pre = STEP_PRE_DEFAULT;

  if (n < 1 || n > (long)CAPTURE_MAX_SAMPLES) { sendError("step: sample count out of range"); return; }
  if (pre < 0 || pre >= n) { sendError("step: pre must be 0..n-1"); return; }

  const ControlSettings s = controllerSettings();
  if (s.mode == MODE_OPEN) {
    if (fabsf(val) > (float)MOTOR_PWM_MAX) { sendError("step: pwm out of range"); return; }
  } else {
    if (fabsf(val) > CTRL_REF_MAX) { sendError("step: ref out of range"); return; }
  }
  if (capBuffer != nullptr) { sendError("step: capture busy"); return; }

  capMeta = { s, true, (uint32_t)pre, val };
  controllerArmStep((uint32_t)pre, val);   // arm first: the capture starts on the next sample
  if (!startCapture("step", (uint32_t)n)) { controllerDisarmStep(); return; }
  Serial.printf("{\"type\":\"cap_start\",\"n\":%lu,\"step\":{\"pre\":%ld,\"val\":%.6g,\"mode\":%u}}\n",
                (unsigned long)capCount, pre, (double)val, (unsigned)s.mode);
}

void cmdMode(const char* arg) {
  long v;
  if (!parseIntArg(arg, 0, 1, v)) { sendError("mode: use 0 or 1"); return; }
  controllerSetMode((uint8_t)v);
  sendState();
}

void cmdRef(const char* arg) {
  float v;
  if (!parseFloatArg(arg, -CTRL_REF_MAX, CTRL_REF_MAX, v)) { sendError("ref: value out of range"); return; }
  controllerSetRef(v);
  sendState();
}

void cmdGain(const char* name, const char* arg, float hi, void (*set)(float)) {
  float v;
  if (!parseFloatArg(arg, 0.0f, hi, v)) {
    Serial.printf("{\"type\":\"err\",\"msg\":\"%s: value out of range\"}\n", name);
    return;
  }
  set(v);
  sendState();
}

void cmdTs(const char* arg) {
  long v;
  if (!parseIntArg(arg, (long)SAMPLE_PERIOD_MS, (long)CTRL_TS_MAX_MS, v)) { sendError("ts: value out of range"); return; }
  controllerSetTs((uint32_t)v);
  sendState();
}

void cmdFeedback(const char* arg) {
  long v;
  if (!parseIntArg(arg, 1, 2, v)) { sendError("fb: use 1 or 2"); return; }
  controllerSetFeedback((uint8_t)v);
  sendState();
}

void cmdUMax(const char* arg) {
  float v;
  if (!parseFloatArg(arg, CTRL_UMAX_MIN, 100.0f, v)) { sendError("umax: value out of range"); return; }
  controllerSetUMax(v);
  sendState();
}

void cmdWatchdog(const char* arg) {
  long v;
  if (!parseIntArg(arg, 0, (long)WD_MAX_MS, v) || (v != 0 && v < (long)WD_MIN_MS)) {
    sendError("wd: use 0 or 200..10000");
    return;
  }
  controllerSetWatchdog((uint32_t)v, millis());
  sendState();
}

// "tel <0|1>"
void cmdTelemetry(const char* arg) {
  long v;
  if (!parseIntArg(arg, 0, 1, v)) { sendError("tel: use 0 or 1"); return; }
  telemetryOn = (v == 1);
}

void sendHelp() {
  Serial.print(
    "# flexDrive commands (end each line with newline):\n"
    "#   help               this list\n"
    "#   info               firmware info and current settings (JSON)\n"
    "#   state              current controller settings (JSON)\n"
    "#   tel <0|1>          telemetry stream off/on\n"
    "#   mode <0|1>         0 = open loop (pwm), 1 = closed-loop velocity control\n"
    "#   pwm <n>            open-loop motor command, -4000..4000 (mode 0 only)\n"
    "#   ref <x>            velocity reference [turn/s], -10..10 (used in mode 1)\n"
    "#   kp <x>             proportional gain [%/(turn/s)], 0..200\n"
    "#   ki <x>             integral gain [%/turn], 0..2000\n"
    "#   kd <x>             derivative gain [%*s/turn], 0..10\n"
    "#   ts <ms>            controller period, 1..50 ms\n"
    "#   fb <1|2>           feedback encoder: 1 = output shaft, 2 = motor\n"
    "#   umax <x>           controller output limit [%], 1..100\n"
    "#   stop               motor off, ref 0, back to mode 0\n"
    "#   step <v> <n> [pre] capture n samples, apply v at sample pre (default 200)\n"
    "#                      mode 0: v = pwm, mode 1: v = ref [turn/s]\n"
    "#   capture <n>        record n samples at 1 kHz (max 5000), then dump CSV\n"
    "#   wd <ms>            stop the motor if no command arrives for ms (0 = off)\n"
    "#   ping               keep-alive for the watchdog (no reply)\n"
    "#   obs_bw <f>         observer bandwidth [Hz] for both encoders\n"
    "#   obs_bw <f1> <f2>   observer bandwidth [Hz] per encoder\n"
    "#   obs_zeta <z>       observer damping (0.3..3)\n");
}

bool isCmd(const char* cmd, const char* name, const char*& arg) {
  const size_t len = strlen(name);
  if (strncmp(cmd, name, len) != 0 || cmd[len] != ' ') return false;
  arg = cmd + len + 1;
  return true;
}

void handleCommand(const char* cmd) {
  const char* a = nullptr;
  if      (strcmp(cmd, "ping") == 0)  { /* keep-alive only */ }
  else if (strcmp(cmd, "stop") == 0)  { controllerStop(); sendState(); }
  else if (strcmp(cmd, "info") == 0)  sendInfo();
  else if (strcmp(cmd, "state") == 0) sendState();
  else if (strcmp(cmd, "help") == 0)  sendHelp();
  else if (isCmd(cmd, "pwm", a))      cmdPwm(a);
  else if (isCmd(cmd, "mode", a))     cmdMode(a);
  else if (isCmd(cmd, "ref", a))      cmdRef(a);
  else if (isCmd(cmd, "kp", a))       cmdGain("kp", a, CTRL_KP_MAX, controllerSetKp);
  else if (isCmd(cmd, "ki", a))       cmdGain("ki", a, CTRL_KI_MAX, controllerSetKi);
  else if (isCmd(cmd, "kd", a))       cmdGain("kd", a, CTRL_KD_MAX, controllerSetKd);
  else if (isCmd(cmd, "ts", a))       cmdTs(a);
  else if (isCmd(cmd, "fb", a))       cmdFeedback(a);
  else if (isCmd(cmd, "umax", a))     cmdUMax(a);
  else if (isCmd(cmd, "step", a))     cmdStep(a);
  else if (isCmd(cmd, "capture", a))  cmdCapture(a);
  else if (isCmd(cmd, "wd", a))       cmdWatchdog(a);
  else if (isCmd(cmd, "tel", a))      cmdTelemetry(a);
  else if (isCmd(cmd, "obs_bw", a))   cmdObserverBandwidth(a);
  else if (isCmd(cmd, "obs_zeta", a)) cmdObserverDamping(a);
  else sendError("unknown command");
}

// Non-blocking line reader for incoming commands. Every complete line feeds the
// watchdog, also during long capture dumps.
void pollCommands() {
  while (Serial.available() > 0) {
    const char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (cmdLen > 0) {
        cmdBuf[cmdLen] = '\0';
        controllerFeedWatchdog(millis());
        handleCommand(cmdBuf);
        cmdLen = 0;
      }
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = ch;
    }
  }
}

// A safety stop in the sampler is reported as an error followed by the new state.
void reportFaults() {
  const ControlFault f = controllerTakeFault();
  if (f == FAULT_NONE) return;
  sendError(controllerFaultName(f));
  sendState();
}

void telemetryTask(void*) {
  sendInfo();
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
    pollCommands();
    reportFaults();
    if (telemetryOn) sendTelemetry(samplerLatest(), controllerLatest());
    if (capBuffer != nullptr && samplerCaptureDone()) {
      dumpCapture();
      reportFaults();
      lastWake = xTaskGetTickCount();   // do not try to catch up missed periods
    }
  }
}

}  // namespace

void telemetryBegin() {
  xTaskCreatePinnedToCore(telemetryTask, "telemetry", 4096, nullptr,
                          TELEMETRY_PRIORITY, nullptr, RT_CORE);
}
