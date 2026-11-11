// =============================================================================
//  flexDrive firmware v2
//  Board: ESP32 (TTGO LoRa32)
//
//  Modules:
//    config.h      pins, constants, timing
//    encoders      quadrature decoding in interrupts
//    sampler       1 kHz sampling + velocity estimation, calls the controller
//    controller    velocity PID (mode 1), open-loop PWM (mode 0), step test, safety stops
//    velocity_observer  Luenberger observer for encoder angle/velocity
//    motor         H-bridge PWM + direction output
//    telemetry     JSON-lines link to the PC / web page (web/index.html)
// =============================================================================
#include "config.h"
#include "encoders.h"
#include "sampler.h"
#include "telemetry.h"
#include "motor.h"

void setup() {
  motorBegin();                   // first: make sure the motor is off

  Serial.setTxBufferSize(1024);   // must be set before begin(); avoids blocking on bursts
  Serial.begin(SERIAL_BAUD);

  encodersBegin();
  samplerBegin();
  telemetryBegin();
}

void loop() {
  // All work is done in FreeRTOS tasks.
  vTaskDelete(nullptr);
}
