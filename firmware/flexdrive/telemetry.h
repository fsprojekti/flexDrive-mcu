#pragma once
// Serial link to the PC / web page.
//
// Every line sent by the firmware is one JSON object with a "type" field:
//   {"type":"info", "fw":..., "ver":..., "sample_ms":..., "tel_ms":..., "cpr":[c1,c2],
//                   "obs_bw":[Hz1,Hz2], "obs_zeta":zeta, "pwm_max":n, "cap_max":n,
//                   "ref_max":x, "v_limit":x, <state keys>}
//   {"type":"state", <state keys>}
//       state keys: "mode","ref","pwm","kp","ki","kd","ts","fb","umax","wd"
//       sent in reply to every command that changes a setting, to "state", and
//       after every safety stop
//   {"type":"tel",  "t":ms, "p1":count, "v1":turn/s, "p2":count, "v2":turn/s, "u":pwm,
//                   "e1":n, "e2":n, "mode":m, "ref":turn/s, "uc":%, "ui":%, "sat":0|1}
//                   v1/v2 = observer velocity estimate, uc = controller output [%],
//                   ui = its integral part [%], sat = output limited by umax
//   {"type":"err",  "msg":...}     safety stops: "overspeed", "runaway", "encoder", "watchdog"
//   {"type":"cap_start","n":N [,"step":{"pre":k,"val":x,"mode":m}]}
//   {"type":"cap_begin","n":N,"fields":"t_us,c1,c2,v1,v2,pwm,ref,ui","obs_bw":..,
//                       "obs_zeta":..,"cpr":..,"ctrl":{<state keys>},"step":{..}|null}
//   t_us,c1,c2,v1,v2,pwm,ref,ui   <- N plain CSV lines, one per 1 ms sample
//   {"type":"cap_end","n":N}
//   Columns are only ever appended; read them by name from "fields". Replies to
//   commands sent during a dump (e.g. "stop") appear as JSON lines between CSV rows.
//
// Lines starting with '#' are human-readable text (e.g. the help list).
//
// Commands from the PC are plain text lines:
//   help / info / state
//   tel <0|1>          telemetry stream off/on (off is handy in a serial terminal)
//   mode <0|1>         0 = open loop, 1 = closed-loop velocity control (see controller.h)
//   pwm <n>            open-loop motor command, -pwm_max..pwm_max (error in mode 1)
//   ref <x>            velocity reference [turn/s]
//   kp / ki / kd <x>   controller gains
//   ts <ms>            controller period
//   fb <1|2>           feedback encoder
//   umax <x>           output limit [%]
//   stop               motor off, ref 0, mode 0 (always works, also during dumps)
//   step <v> <n> [pre] capture n samples; at sample pre apply v (pwm or ref) in that sample
//   capture <n>        record n samples at 1 kHz, then dump them
//   wd <ms>            connection watchdog, 0 = off; any command line feeds it
//   ping               watchdog keep-alive, no reply
//   obs_bw <f1> [f2]   observer bandwidth [Hz] (one value = both encoders), replies with info
//   obs_zeta <zeta>    observer damping, replies with info
//
// This task is the only writer to Serial, so lines never interleave.
#include <Arduino.h>

void telemetryBegin();
