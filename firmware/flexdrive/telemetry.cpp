#include "telemetry.h"
#include "config.h"
#include "sampler.h"
#include "controller.h"
#include <Preferences.h>

namespace {

char   cmdBuf[CMD_LINE_MAX + 1];
char   calJson[CAL_MAX_LEN + 1] = "";   // calibration from NVS, "" = none
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
  uint32_t dec;        // one row every dec ms
  float    refScale;   // ref stored as int16 ref*refScale
  bool     isRun;      // capture started together with the reference profile
};
CaptureMeta capMeta = {};

// Capture resolution of ref: 0.001 turn/s (modes 0/1) or 0.1 deg (mode 2, +-3276 deg).
float refScaleFor(uint8_t mode) { return mode == MODE_POS ? 10.0f : 1000.0f; }
float refLimitFor(uint8_t mode) { return mode == MODE_POS ? CTRL_POS_REF_MAX : CTRL_REF_MAX; }

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
  // Board id: last three bytes of the factory MAC, so a page can tell the models apart.
  const uint64_t mac = ESP.getEfuseMac();
  Serial.printf("{\"type\":\"info\",\"id\":\"%02X%02X%02X\",\"cal\":%s,",
                (unsigned)((mac >> 24) & 0xFF), (unsigned)((mac >> 32) & 0xFF), (unsigned)((mac >> 40) & 0xFF),
                calJson[0] ? calJson : "null");
  Serial.printf("\"fw\":\"%s\",\"ver\":\"%s\",\"sample_ms\":%lu,"
                "\"tel_ms\":%lu,\"cpr\":[%.1f,%.1f],\"obs_bw\":[%.6g,%.6g],\"obs_zeta\":%.6g,"
                "\"pwm_max\":%d,\"cap_max\":%lu,\"ref_max\":%.6g,\"pos_ref_max\":%.6g,"
                "\"prof_max\":%lu,\"dec_max\":%lu,\"v_limit\":%.6g,",
                FW_NAME, FW_VERSION,
                (unsigned long)SAMPLE_PERIOD_MS, (unsigned long)TELEMETRY_PERIOD_MS,
                ENC1_CPR, ENC2_CPR,
                (double)samplerObserverBandwidth(0), (double)samplerObserverBandwidth(1),
                (double)samplerObserverDamping(),
                MOTOR_PWM_MAX, (unsigned long)CAPTURE_MAX_SAMPLES,
                (double)CTRL_REF_MAX, (double)CTRL_POS_REF_MAX,
                (unsigned long)PROF_MAX_POINTS, (unsigned long)CAPTURE_DEC_MAX,
                (double)SAFETY_V_LIMIT);
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
      "\"u\":%d,\"e1\":%lu,\"e2\":%lu,\"mode\":%u,\"ref\":%.6g,\"uc\":%.2f,\"ui\":%.2f,"
      "\"sat\":%d,\"pt\":%ld}\n",
      (unsigned long)m.timeMs,
      (long)m.count[0], m.velocity[0],
      (long)m.count[1], m.velocity[1],
      c.pwm,
      (unsigned long)m.errors[0], (unsigned long)m.errors[1],
      (unsigned)c.mode, (double)c.ref, (double)c.uPct, (double)c.uI, c.sat ? 1 : 0,
      (long)c.profMs);
  if (n > 0 && n < (int)sizeof(buf)) Serial.write(reinterpret_cast<const uint8_t*>(buf), n);
}

// Dump a finished capture as CSV between cap_begin / cap_end markers.
// Columns are listed in "fields"; new columns are only ever appended.
void dumpCapture() {
  Serial.printf("{\"type\":\"cap_begin\",\"n\":%lu,\"fields\":\"t_us,c1,c2,v1,v2,pwm,ref,ui\","
                "\"dec\":%lu,\"run\":%s,"
                "\"obs_bw\":[%.6g,%.6g],\"obs_zeta\":%.6g,\"cpr\":[%.1f,%.1f],\"ctrl\":{",
                (unsigned long)capCount, (unsigned long)capMeta.dec, capMeta.isRun ? "true" : "false",
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
                           s.refScaled / capMeta.refScale, s.uiCenti * 1e-2f);
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
bool startCapture(const char* who, uint32_t n, uint32_t dec, float refScale) {
  if (capBuffer != nullptr) {
    Serial.printf("{\"type\":\"err\",\"msg\":\"%s: capture busy\"}\n", who);
    return false;
  }
  capBuffer = capStorage;
  capCount  = n;
  if (!samplerStartCapture(capBuffer, capCount, dec, refScale)) {
    capBuffer = nullptr; capCount = 0;
    Serial.printf("{\"type\":\"err\",\"msg\":\"%s: capture busy\"}\n", who);
    return false;
  }
  return true;
}

// "<n> [dec]" -> sample count and decimation; false (error sent) if out of range
bool parseCountDec(const char* who, const char* arg, long& n, long& dec) {
  char* end = nullptr;
  n = strtol(arg, &end, 10);
  if (end == arg || n < 1 || n > (long)CAPTURE_MAX_SAMPLES) {
    Serial.printf("{\"type\":\"err\",\"msg\":\"%s: sample count out of range\"}\n", who);
    return false;
  }
  const char* p = end;
  dec = strtol(p, &end, 10);
  if (end == p) dec = 1;
  if (dec < 1 || dec > (long)CAPTURE_DEC_MAX) {
    Serial.printf("{\"type\":\"err\",\"msg\":\"%s: dec must be 1..%lu\"}\n", who,
                  (unsigned long)CAPTURE_DEC_MAX);
    return false;
  }
  return true;
}

// "capture <n> [dec]"  -> record n samples, one every dec ms (default 1), then dump them
void cmdCapture(const char* arg) {
  long n, dec;
  if (!parseCountDec("capture", arg, n, dec)) return;
  const ControlSettings s = controllerSettings();
  capMeta = { s, false, 0, 0.0f, (uint32_t)dec, refScaleFor(s.mode), false };
  if (!startCapture("capture", (uint32_t)n, capMeta.dec, capMeta.refScale)) return;
  Serial.printf("{\"type\":\"cap_start\",\"n\":%lu,\"dec\":%ld}\n", (unsigned long)capCount, dec);
}

// "run <n> [dec]"  -> start the reference profile and a capture in the same sample
void cmdRun(const char* arg) {
  long n, dec;
  if (!parseCountDec("run", arg, n, dec)) return;
  if (controllerProfileCount() == 0) { sendError("run: no profile (use prof add)"); return; }
  const ControlSettings s = controllerSettings();
  if (s.mode != MODE_OPEN && controllerProfileMaxAbs() > refLimitFor(s.mode)) {
    sendError("run: profile out of range for this mode");
    return;
  }
  if (capBuffer != nullptr) { sendError("run: capture busy"); return; }
  // A profile in degrees (mode 2, or mode 0 as a target for manual tracking) needs the
  // 0.1-deg resolution: 0.001 units would clip at +-32.767.
  const float scale = (s.mode == MODE_POS || controllerProfileMaxAbs() > 32.0f) ? 10.0f : 1000.0f;
  capMeta = { s, false, 0, 0.0f, (uint32_t)dec, scale, true };
  controllerArmProfile();   // arm first: the capture starts on the next sample
  if (!startCapture("run", (uint32_t)n, capMeta.dec, capMeta.refScale)) { controllerDisarmProfile(); return; }
  Serial.printf("{\"type\":\"cap_start\",\"n\":%lu,\"dec\":%ld,\"run\":true}\n",
                (unsigned long)capCount, dec);
}

// "prof clear" | "prof add <t_ms> <val>"  -> build the reference profile point by point
void cmdProfile(const char* arg) {
  if (strcmp(arg, "clear") == 0) {
    controllerProfileClear();
    Serial.print("{\"type\":\"prof\",\"n\":0}\n");
    return;
  }
  if (strncmp(arg, "add ", 4) != 0) { sendError("prof: use 'prof clear' or 'prof add <t_ms> <val>'"); return; }
  char* end = nullptr;
  const char* p = arg + 4;
  const long t = strtol(p, &end, 10);
  if (end == p || t < 0 || t > (long)PROF_MAX_MS) { sendError("prof: time out of range"); return; }
  p = end;
  const float v = strtof(p, &end);
  if (end == p || !isfinite(v) || fabsf(v) > CTRL_POS_REF_MAX) { sendError("prof: value out of range"); return; }
  if (!controllerProfileAdd((uint32_t)t, v)) {
    sendError("prof: full, time not increasing, or profile running");
    return;
  }
  Serial.printf("{\"type\":\"prof\",\"n\":%lu}\n", (unsigned long)controllerProfileCount());
}

// "zero"  -> angle 0 at the current position (mode 0 only, so the shaft cannot jump)
void cmdZero() {
  if (controllerSettings().mode != MODE_OPEN) { sendError("zero: only in mode 0"); return; }
  samplerZero();
  sendState();
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
    if (fabsf(val) > refLimitFor(s.mode)) { sendError("step: ref out of range"); return; }
  }
  if (capBuffer != nullptr) { sendError("step: capture busy"); return; }

  capMeta = { s, true, (uint32_t)pre, val, 1, refScaleFor(s.mode), false };
  controllerArmStep((uint32_t)pre, val);   // arm first: the capture starts on the next sample
  if (!startCapture("step", (uint32_t)n, 1, capMeta.refScale)) { controllerDisarmStep(); return; }
  Serial.printf("{\"type\":\"cap_start\",\"n\":%lu,\"step\":{\"pre\":%ld,\"val\":%.6g,\"mode\":%u}}\n",
                (unsigned long)capCount, pre, (double)val, (unsigned)s.mode);
}

void cmdMode(const char* arg) {
  long v;
  if (!parseIntArg(arg, 0, 2, v)) { sendError("mode: use 0, 1 or 2"); return; }
  controllerSetMode((uint8_t)v);
  sendState();
}

void cmdRef(const char* arg) {
  float v;
  const float lim = refLimitFor(controllerSettings().mode);
  if (!parseFloatArg(arg, -lim, lim, v)) { sendError("ref: value out of range"); return; }
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
    "#   mode <0|1|2>       0 = open loop (pwm), 1 = velocity control, 2 = angle control\n"
    "#   pwm <n>            open-loop motor command, -4000..4000 (mode 0 only)\n"
    "#   ref <x>            mode 1: velocity [turn/s] -10..10, mode 2: angle [deg] -3000..3000\n"
    "#   kp <x>             proportional gain, 0..200   [%/(turn/s)] or [%/deg]\n"
    "#   ki <x>             integral gain, 0..2000      [%/turn] or [%/(deg*s)]\n"
    "#   kd <x>             derivative gain, 0..10      [%*s/turn] or [%*s/deg]\n"
    "#   zero               angle 0 at the current position (mode 0 only)\n"
    "#   ts <ms>            controller period, 1..50 ms\n"
    "#   fb <1|2>           feedback encoder: 1 = output shaft, 2 = motor\n"
    "#   umax <x>           controller output limit [%], 1..100\n"
    "#   stop               motor off, ref 0, back to mode 0\n"
    "#   step <v> <n> [pre] capture n samples, apply v at sample pre (default 200)\n"
    "#                      mode 0: v = pwm, modes 1/2: v = ref\n"
    "#   capture <n> [dec]  record n samples (max 5000), one every dec ms, then dump CSV\n"
    "#   prof clear         clear the reference profile\n"
    "#   prof add <t> <v>   add a profile point: t [ms] from start, v = ref (max 24 points)\n"
    "#   run <n> [dec]      start the profile and a capture of n samples (one every dec ms)\n"
    "#   wd <ms>            stop the motor if no command arrives for ms (0 = off)\n"
    "#   ping               keep-alive for the watchdog (no reply)\n"
    "#   obs_bw <f>         observer bandwidth [Hz] for both encoders\n"
    "#   obs_bw <f1> <f2>   observer bandwidth [Hz] per encoder\n"
    "#   obs_zeta <z>       observer damping (0.3..3)\n"
    "#   cal {json}         store this model's calibration in flash (shown in info)\n"
    "#   cal clear          erase the stored calibration\n");
}

// "cal {json}" stores this model's calibration in NVS, "cal clear" erases it.
// The firmware does not parse the JSON; it only checks the braces and the length.
void cmdCal(const char* arg) {
  const size_t n = strlen(arg);
  Preferences prefs;
  if (strcmp(arg, "clear") == 0) {
    prefs.begin("flexdrive", false);
    prefs.remove("cal");
    prefs.end();
    calJson[0] = '\0';
  } else if (n >= 2 && n <= CAL_MAX_LEN && arg[0] == '{' && arg[n - 1] == '}') {
    prefs.begin("flexdrive", false);
    prefs.putString("cal", arg);
    prefs.end();
    strcpy(calJson, arg);
  } else {
    sendError("cal: use cal {json} (max 300 chars) or cal clear");
    return;
  }
  sendInfo();
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
  else if (isCmd(cmd, "run", a))      cmdRun(a);
  else if (isCmd(cmd, "prof", a))     cmdProfile(a);
  else if (strcmp(cmd, "zero") == 0)  cmdZero();
  else if (isCmd(cmd, "wd", a))       cmdWatchdog(a);
  else if (isCmd(cmd, "tel", a))      cmdTelemetry(a);
  else if (isCmd(cmd, "cal", a))      cmdCal(a);
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
  Preferences prefs;
  if (prefs.begin("flexdrive", true)) {   // read-only; fails harmlessly before the first "cal"
    prefs.getString("cal", calJson, sizeof(calJson));
    prefs.end();
  }
  xTaskCreatePinnedToCore(telemetryTask, "telemetry", 4096, nullptr,
                          TELEMETRY_PRIORITY, nullptr, RT_CORE);
}
