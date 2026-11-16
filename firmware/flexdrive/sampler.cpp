#include "sampler.h"
#include "config.h"
#include "velocity_observer.h"
#include "controller.h"
#include "esp_timer.h"

namespace {

const float kCpr[NUM_ENCODERS] = {ENC1_CPR, ENC2_CPR};

portMUX_TYPE latestMux = portMUX_INITIALIZER_UNLOCKED;
Measurement  latest    = {};

// Written by the command handler, read by the sampler (32-bit float writes are atomic).
volatile float obsBandwidthHz[NUM_ENCODERS] = {OBS_BW_DEFAULT[0], OBS_BW_DEFAULT[1]};
volatile float obsZeta        = OBS_ZETA_DEFAULT;

// Capture state: set up by telemetry task, filled by sampler task (both on RT_CORE).
CaptureSample* volatile capBuf = nullptr;
volatile uint32_t       capN   = 0;
volatile uint32_t       capIdx = 0;
volatile uint32_t       capDec = 1;
volatile float          capRefScale = 1000.0f;
uint32_t                capSub = 0;
int64_t                 capPrevUs = 0;

volatile bool zeroRequest = false;
int32_t       zeroOffset[NUM_ENCODERS] = {0, 0};

void samplerTask(void*) {
  EncoderObserver obs[NUM_ENCODERS];
  for (int i = 0; i < NUM_ENCODERS; ++i) obs[i].reset(encoderCount(i), kCpr[i]);

  int64_t    prevUs   = esp_timer_get_time();
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));

    // Use the measured interval, not the nominal one, so scheduling jitter
    // does not bias the estimate.
    const int64_t nowUs = esp_timer_get_time();
    const float   dt    = (float)(nowUs - prevUs) * 1e-6f;
    prevUs = nowUs;
    if (dt <= 0.0f) continue;

    const float zeta = obsZeta;

    Measurement m;
    m.timeMs = (uint32_t)(nowUs / 1000);
    int32_t raw[NUM_ENCODERS];
    for (int i = 0; i < NUM_ENCODERS; ++i) raw[i] = encoderCount(i);
    if (zeroRequest) {
      for (int i = 0; i < NUM_ENCODERS; ++i) zeroOffset[i] = raw[i];
      zeroRequest = false;
    }
    for (int i = 0; i < NUM_ENCODERS; ++i) {
      const float wn = 2.0f * PI * obsBandwidthHz[i];
      obs[i].update(raw[i], dt, wn, zeta);   // observer on raw counts: unaffected by zero
      m.count[i]    = (int32_t)((uint32_t)raw[i] - (uint32_t)zeroOffset[i]);
      m.velocity[i] = obs[i].velocity();
      m.errors[i]   = encoderErrors(i);
    }

    portENTER_CRITICAL(&latestMux);
    latest = m;
    portEXIT_CRITICAL(&latestMux);

    // Controller runs after the observer, before the sample is recorded, so a step
    // applied at capture sample k shows up in row k (pwm/ref) of the dump.
    CaptureSample* buf = capBuf;
    const bool capturing = (buf != nullptr && capIdx < capN);
    const ControlOutput out = controllerUpdate(m, nowUs, capturing ? (int32_t)capIdx : -1);

    if (capturing && capSub == 0) {
      CaptureSample& s = buf[capIdx];
      s.dtUs = (capIdx == 0) ? 0 : (uint16_t)min<int64_t>(nowUs - capPrevUs, 65535);
      capPrevUs = nowUs;
      for (int i = 0; i < NUM_ENCODERS; ++i) {
        s.count[i]    = m.count[i];
        s.velMilli[i] = (int16_t)constrain(lroundf(m.velocity[i] * 1000.0f), -32767L, 32767L);
      }
      s.pwm       = (int16_t)out.pwm;
      s.refScaled = (int16_t)constrain(lroundf(out.ref * capRefScale), -32767L, 32767L);
      s.uiCenti   = (int16_t)constrain(lroundf(out.uI * 100.0f), -32767L, 32767L);
      capIdx = capIdx + 1;
    }
    if (capturing) capSub = (capSub + 1 >= capDec) ? 0 : capSub + 1;
  }
}

}  // namespace

void samplerBegin() {
  xTaskCreatePinnedToCore(samplerTask, "sampler", 4096, nullptr,
                          SAMPLER_PRIORITY, nullptr, RT_CORE);
}

Measurement samplerLatest() {
  portENTER_CRITICAL(&latestMux);
  const Measurement m = latest;
  portEXIT_CRITICAL(&latestMux);
  return m;
}

bool samplerStartCapture(CaptureSample* buf, uint32_t n, uint32_t dec, float refScale) {
  if (capBuf != nullptr && capIdx < capN) return false;   // still running
  capBuf = nullptr;
  capIdx = 0;
  capN   = n;
  capDec = dec < 1 ? 1 : dec;
  capRefScale = refScale;
  capSub = 0;
  capBuf = buf;   // publish last: sampler starts filling on its next tick
  return true;
}

bool samplerCaptureDone() {
  return capBuf != nullptr && capIdx >= capN;
}

void  samplerZero() { zeroRequest = true; }
void  samplerSetObserverBandwidth(int idx, float hz) { obsBandwidthHz[idx] = hz; }
void  samplerSetObserverDamping(float zeta)          { obsZeta = zeta; }
float samplerObserverBandwidth(int idx)              { return obsBandwidthHz[idx]; }
float samplerObserverDamping()              { return obsZeta; }
