#pragma once
// Serial link to the PC / web page.
//
// Every line sent by the firmware is one JSON object with a "type" field:
//   {"type":"info", "fw":..., "ver":..., "sample_ms":..., "tel_ms":..., "cpr":[c1,c2],
//                   "obs_bw":[Hz1,Hz2], "obs_zeta":zeta, "pwm_max":n, "cap_max":n}
//   {"type":"tel",  "t":ms, "p1":count, "v1":turn/s, "p2":count, "v2":turn/s, "u":pwm,
//                   "e1":n, "e2":n}       v1/v2 = observer velocity estimate
//   {"type":"err",  "msg":...}
//   {"type":"cap_start","n":N}
//   {"type":"cap_begin","n":N,"fields":"t_us,c1,c2,v1,v2,pwm",...}
//   t_us,c1,c2,v1,v2,pwm          <- N plain CSV lines, one per 1 ms sample
//   {"type":"cap_end","n":N}
//
// Lines starting with '#' are human-readable text (e.g. the help list).
//
// Commands from the PC are plain text lines:
//   help             -> list of commands
//   info             -> resend the info message
//   tel <0|1>        -> telemetry stream off/on (off is handy in a serial terminal)
//   pwm <n>          -> open-loop motor command, -pwm_max..pwm_max
//   stop             -> motor off
//   capture <n>      -> record n samples at 1 kHz, then dump them
//   obs_bw <f1> [f2] -> observer bandwidth [Hz] (one value = both encoders), replies with info
//   obs_zeta <zeta>  -> observer damping, replies with info
//
// This task is the only writer to Serial, so lines never interleave.
#include <Arduino.h>

void telemetryBegin();
