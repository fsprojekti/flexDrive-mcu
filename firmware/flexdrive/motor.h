#pragma once
// DC motor through an H-bridge: PWM magnitude + direction pin.
// Command is signed: positive and negative values select the direction.
#include <Arduino.h>

void motorBegin();
void motorSetPwm(int pwm);   // clamped to [-MOTOR_PWM_MAX, MOTOR_PWM_MAX]
int  motorPwm();
