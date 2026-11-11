#pragma once
// Periodic sampling task: reads encoders at a fixed rate, estimates angle and
// velocity with a Luenberger observer and runs the controller (controller.h)
// synchronous with the measurement.
#include <Arduino.h>
#include "encoders.h"

struct Measurement {
  uint32_t timeMs;                    // time of sample since boot [ms]
  int32_t  count[NUM_ENCODERS];       // raw encoder counts
  float    velocity[NUM_ENCODERS];    // observer velocity estimate [turn/s]
  uint32_t errors[NUM_ENCODERS];      // encoder error counters
};

void        samplerBegin();
Measurement samplerLatest();          // consistent snapshot of the latest sample

// ---- High-rate capture -----------------------------------------------------------
// Records every sample (1 kHz) into a caller-provided buffer.
// Packed to 20 bytes so 5 s fit into static DRAM.
struct CaptureSample {
  int32_t  count[NUM_ENCODERS];
  int16_t  velMilli[NUM_ENCODERS];    // observer velocity [1e-3 turn/s], saturated at +-32.767
  uint16_t dtUs;                      // time since previous sample [us] (0 for the first)
  int16_t  pwm;
  int16_t  refMilli;                  // controller reference [1e-3 turn/s]
  int16_t  uiCenti;                   // integral part of the controller [0.01 %]
};
static_assert(sizeof(CaptureSample) == 20, "CaptureSample should stay compact");

bool samplerStartCapture(CaptureSample* buf, uint32_t n);   // false if one is running
bool samplerCaptureDone();

// Observer tuning, applied from the next sample on.
void  samplerSetObserverBandwidth(int idx, float hz);   // per encoder [Hz]
void  samplerSetObserverDamping(float zeta);           // shared
float samplerObserverBandwidth(int idx);
float samplerObserverDamping();
