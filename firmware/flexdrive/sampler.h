#pragma once
// Periodic sampling task: reads encoders at a fixed rate, estimates angle and
// velocity with a Luenberger observer and runs the controller (controller.h)
// synchronous with the measurement.
#include <Arduino.h>
#include "encoders.h"

struct Measurement {
  uint32_t timeMs;                    // time of sample since boot [ms]
  int32_t  count[NUM_ENCODERS];       // encoder counts since the last "zero"
  float    velocity[NUM_ENCODERS];    // observer velocity estimate [turn/s]
  uint32_t errors[NUM_ENCODERS];      // encoder error counters
};

void        samplerBegin();
Measurement samplerLatest();          // consistent snapshot of the latest sample

// ---- High-rate capture -----------------------------------------------------------
// Records every dec-th sample (1 kHz / dec) into a caller-provided buffer.
// Packed to 20 bytes so 5 s fit into static DRAM.
struct CaptureSample {
  int32_t  count[NUM_ENCODERS];
  int16_t  velMilli[NUM_ENCODERS];    // observer velocity [1e-3 turn/s], saturated at +-32.767
  uint16_t dtUs;                      // time since previous sample [us] (0 for the first)
  int16_t  pwm;
  int16_t  refScaled;                 // reference * refScale (see samplerStartCapture)
  int16_t  uiCenti;                   // integral part of the controller [0.01 %]
};
static_assert(sizeof(CaptureSample) == 20, "CaptureSample should stay compact");

// dec: record one sample every dec ms (1..CAPTURE_DEC_MAX).
// refScale: ref is stored as int16 ref*refScale (1000 for turn/s, 10 for deg).
bool samplerStartCapture(CaptureSample* buf, uint32_t n, uint32_t dec, float refScale);   // false if busy
bool samplerCaptureDone();

// Set the reported counts of both encoders to 0 in the next sample (the observers keep
// running on the raw counts, so the velocity does not jump).
void samplerZero();

// Observer tuning, applied from the next sample on.
void  samplerSetObserverBandwidth(int idx, float hz);   // per encoder [Hz]
void  samplerSetObserverDamping(float zeta);           // shared
float samplerObserverBandwidth(int idx);
float samplerObserverDamping();
