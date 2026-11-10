#pragma once
// Quadrature encoder counting (interrupt driven, x4 decoding).
#include <Arduino.h>

constexpr int NUM_ENCODERS = 2;

void     encodersBegin();
int32_t  encoderCount(int idx);   // raw count since boot
uint32_t encoderErrors(int idx);  // illegal transitions (both channels changed) -> missed edges
