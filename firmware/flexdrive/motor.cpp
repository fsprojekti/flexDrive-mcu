#include "motor.h"
#include "config.h"

namespace {
volatile int currentPwm = 0;
}

void motorBegin() {
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  digitalWrite(MOTOR_DIR_PIN, LOW);
  ledcAttach(MOTOR_PWM_PIN, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
  ledcWrite(MOTOR_PWM_PIN, 0);
  currentPwm = 0;
}

void motorSetPwm(int pwm) {
  pwm = constrain(pwm, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
  // Same convention as v1 firmware: negative command -> DIR pin HIGH.
  digitalWrite(MOTOR_DIR_PIN, pwm < 0 ? HIGH : LOW);
  ledcWrite(MOTOR_PWM_PIN, (uint32_t)abs(pwm));
  currentPwm = pwm;
}

int motorPwm() {
  return currentPwm;
}
