#pragma once
// =============================================================================
//  Luenberger observer for encoder angle and velocity
//
//  Model:        theta' = omega,  omega' = 0       (constant velocity)
//  Measurement:  z = encoder count
//
//  Discrete predictor/corrector (alpha-beta form):
//    predict:  theta^ += omega^ * dt
//    correct:  y = z - theta^
//              theta^ += alpha * y
//              omega^ += (beta / dt) * y
//
//  Error dynamics:  z^2 - (2 - alpha - beta) z + (1 - alpha) = 0
//  The gains place both poles exactly at z = exp(s*dt), where s are the poles
//  of the continuous 2nd-order system  s^2 + 2*zeta*wn*s + wn^2.
//
//  Tuning: bandwidth f [Hz] (wn = 2*pi*f) and damping zeta.
//          zeta = 1     -> critically damped, no overshoot
//          zeta = 0.707 -> faster, ~4 % overshoot (= steady-state Kalman shape)
//
//  Numerics: the angle state is kept relative to the last measured count, so
//  float precision does not degrade as the absolute position grows.
// =============================================================================
#include <Arduino.h>

class EncoderObserver {
 public:
  void reset(int32_t count, float cpr);
  void update(int32_t count, float dt, float wn, float zeta);   // dt [s], wn [rad/s]

  float velocity() const { return w_ / cpr_; }   // [turn/s]

  // Observer gains for given sampling time and pole placement.
  static void gains(float dt, float wn, float zeta, float& alpha, float& beta);

 private:
  float   cpr_ = 1.0f;
  int32_t ref_ = 0;      // last measured count
  float   e_   = 0.0f;   // theta^ - ref_ [count]
  float   w_   = 0.0f;   // omega^ [count/s]
};
