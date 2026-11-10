#include "telemetry.h"
#include "config.h"
#include "sampler.h"
#include "motor.h"

namespace {

char   cmdBuf[64];
size_t cmdLen = 0;

// Statically reserved: a 120 kB malloc at runtime fails due to heap fragmentation.
CaptureSample  capStorage[CAPTURE_MAX_SAMPLES];
CaptureSample* capBuffer = nullptr;   // non-null while a capture is running or being dumped
uint32_t       capCount  = 0;

bool telemetryOn = true;              // "tel 0" silences the stream for manual terminal use

void pollCommands();

// ---------------------------------------------------------------------------
//  Messages
// ---------------------------------------------------------------------------
void sendInfo() {
  Serial.printf("{\"type\":\"info\",\"fw\":\"%s\",\"ver\":\"%s\",\"sample_ms\":%lu,"
                "\"tel_ms\":%lu,\"cpr\":[%.1f,%.1f],\"obs_bw\":[%.6g,%.6g],\"obs_zeta\":%.6g,"
                "\"pwm_max\":%d,\"cap_max\":%lu}\n",
                FW_NAME, FW_VERSION,
                (unsigned long)SAMPLE_PERIOD_MS, (unsigned long)TELEMETRY_PERIOD_MS,
                ENC1_CPR, ENC2_CPR,
                (double)samplerObserverBandwidth(0), (double)samplerObserverBandwidth(1),
                (double)samplerObserverDamping(),
                MOTOR_PWM_MAX, (unsigned long)CAPTURE_MAX_SAMPLES);
}

void sendError(const char* msg) {
  Serial.printf("{\"type\":\"err\",\"msg\":\"%s\"}\n", msg);
}

void sendTelemetry(const Measurement& m) {
  char buf[192];
  const int n = snprintf(buf, sizeof(buf),
      "{\"type\":\"tel\",\"t\":%lu,\"p1\":%ld,\"v1\":%.4f,\"p2\":%ld,\"v2\":%.4f,"
      "\"u\":%d,\"e1\":%lu,\"e2\":%lu}\n",
      (unsigned long)m.timeMs,
      (long)m.count[0], m.velocity[0],
      (long)m.count[1], m.velocity[1],
      motorPwm(),
      (unsigned long)m.errors[0], (unsigned long)m.errors[1]);
  if (n > 0 && n < (int)sizeof(buf)) Serial.write(reinterpret_cast<const uint8_t*>(buf), n);
}

// Dump a finished capture as CSV between cap_begin / cap_end markers.
void dumpCapture() {
  Serial.printf("{\"type\":\"cap_begin\",\"n\":%lu,\"fields\":\"t_us,c1,c2,v1,v2,pwm\","
                "\"obs_bw\":[%.6g,%.6g],\"obs_zeta\":%.6g,\"cpr\":[%.1f,%.1f]}\n",
                (unsigned long)capCount,
                (double)samplerObserverBandwidth(0), (double)samplerObserverBandwidth(1),
                (double)samplerObserverDamping(),
                ENC1_CPR, ENC2_CPR);
  char line[96];
  uint32_t tUs = 0;
  for (uint32_t k = 0; k < capCount; ++k) {
    const CaptureSample& s = capBuffer[k];
    tUs += s.dtUs;
    const int n = snprintf(line, sizeof(line), "%lu,%ld,%ld,%.3f,%.3f,%d\n",
                           (unsigned long)tUs, (long)s.count[0], (long)s.count[1],
                           s.velMilli[0] * 1e-3f, s.velMilli[1] * 1e-3f, (int)s.pwm);
    if (n > 0 && n < (int)sizeof(line)) Serial.write(reinterpret_cast<const uint8_t*>(line), n);
    if ((k & 127) == 127) pollCommands();   // keep "stop" responsive during long dumps
  }
  Serial.printf("{\"type\":\"cap_end\",\"n\":%lu}\n", (unsigned long)capCount);
  capBuffer = nullptr;
  capCount  = 0;
}

// ---------------------------------------------------------------------------
//  Commands
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

// "pwm <value>"  -> open-loop motor command, signed
void cmdPwm(const char* arg) {
  long v;
  if (!parseIntArg(arg, -MOTOR_PWM_MAX, MOTOR_PWM_MAX, v)) { sendError("pwm: value out of range"); return; }
  motorSetPwm((int)v);
}

// "capture <samples>"  -> record samples at 1 kHz, then dump them
void cmdCapture(const char* arg) {
  long n;
  if (!parseIntArg(arg, 1, CAPTURE_MAX_SAMPLES, n)) { sendError("capture: sample count out of range"); return; }
  if (capBuffer != nullptr) { sendError("capture: already running"); return; }
  capBuffer = capStorage;
  capCount  = (uint32_t)n;
  if (!samplerStartCapture(capBuffer, capCount)) {
    capBuffer = nullptr; capCount = 0;
    sendError("capture: busy");
    return;
  }
  Serial.printf("{\"type\":\"cap_start\",\"n\":%lu}\n", (unsigned long)capCount);
}

void sendHelp() {
  Serial.print(
    "# flexDrive commands (end each line with newline):\n"
    "#   help               this list\n"
    "#   info               firmware info and current settings (JSON)\n"
    "#   tel <0|1>          telemetry stream off/on\n"
    "#   pwm <n>            motor command, -4000..4000\n"
    "#   stop               motor off\n"
    "#   obs_bw <f>         observer bandwidth [Hz] for both encoders\n"
    "#   obs_bw <f1> <f2>   observer bandwidth [Hz] per encoder\n"
    "#   obs_zeta <z>       observer damping (0.3..3)\n"
    "#   capture <n>        record n samples at 1 kHz (max 5000), then dump CSV\n");
}

// "tel <0|1>"
void cmdTelemetry(const char* arg) {
  long v;
  if (!parseIntArg(arg, 0, 1, v)) { sendError("tel: use 0 or 1"); return; }
  telemetryOn = (v == 1);
}

void handleCommand(const char* cmd) {
  if (strcmp(cmd, "info") == 0) {
    sendInfo();
  } else if (strcmp(cmd, "help") == 0) {
    sendHelp();
  } else if (strncmp(cmd, "tel ", 4) == 0) {
    cmdTelemetry(cmd + 4);
  } else if (strcmp(cmd, "stop") == 0) {
    motorSetPwm(0);
  } else if (strncmp(cmd, "pwm ", 4) == 0) {
    cmdPwm(cmd + 4);
  } else if (strncmp(cmd, "capture ", 8) == 0) {
    cmdCapture(cmd + 8);
  } else if (strncmp(cmd, "obs_bw ", 7) == 0) {
    cmdObserverBandwidth(cmd + 7);
  } else if (strncmp(cmd, "obs_zeta ", 9) == 0) {
    cmdObserverDamping(cmd + 9);
  } else {
    sendError("unknown command");
  }
}

// Non-blocking line reader for incoming commands.
void pollCommands() {
  while (Serial.available() > 0) {
    const char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (cmdLen > 0) {
        cmdBuf[cmdLen] = '\0';
        handleCommand(cmdBuf);
        cmdLen = 0;
      }
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = ch;
    }
  }
}

void telemetryTask(void*) {
  sendInfo();
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
    pollCommands();
    if (telemetryOn) sendTelemetry(samplerLatest());
    if (capBuffer != nullptr && samplerCaptureDone()) {
      dumpCapture();
      lastWake = xTaskGetTickCount();   // do not try to catch up missed periods
    }
  }
}

}  // namespace

void telemetryBegin() {
  xTaskCreatePinnedToCore(telemetryTask, "telemetry", 4096, nullptr,
                          TELEMETRY_PRIORITY, nullptr, RT_CORE);
}
