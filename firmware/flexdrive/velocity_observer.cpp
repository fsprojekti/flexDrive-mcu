#include "velocity_observer.h"

void EncoderObserver::reset(int32_t count, float cpr) {
  cpr_ = cpr;
  ref_ = count;
  e_   = 0.0f;
  w_   = 0.0f;
}

void EncoderObserver::gains(float dt, float wn, float zeta, float& alpha, float& beta) {
  // Desired discrete characteristic polynomial  z^2 + a1 z + a0
  const float x  = zeta * wn * dt;
  const float a0 = expf(-2.0f * x);
  const float a1 = (zeta < 1.0f)
      ? -2.0f * expf(-x) * cosf(wn * dt * sqrtf(1.0f - zeta * zeta))
      : -2.0f * expf(-x) * coshf(wn * dt * sqrtf(zeta * zeta - 1.0f));
  alpha = 1.0f - a0;
  beta  = 1.0f + a0 + a1;
}

void EncoderObserver::update(int32_t count, float dt, float wn, float zeta) {
  float alpha, beta;
  gains(dt, wn, zeta, alpha, beta);   // recomputed per step: exact even with jitter in dt

  // ---- Predict ----------------------------------------------------------------
  e_ += w_ * dt;

  // ---- Correct with measurement z = count - ref_ ------------------------------
  const float z = (float)(int32_t)((uint32_t)count - (uint32_t)ref_);   // wrap-safe
  const float y = z - e_;
  e_ += alpha * y;
  w_ += (beta / dt) * y;

  // ---- Re-base the angle state onto the new measurement ----------------------
  e_  -= z;
  ref_ = count;
}
