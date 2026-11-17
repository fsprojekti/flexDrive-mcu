# Firmware v2.2 — measurements on the lab FlexDrive (2026-11-16)

Board on COM3, arduino-cli `esp32:esp32:ttgo-lora32`, core 3.3.12 (309 kB flash,
123 kB static RAM). Tests driven from a PC over the serial protocol and from the VL4
student page.

## Basics

| Check | Result |
|---|---|
| `info` | `"ver":"2.2.0"` |
| `pwm 1500` | v1 = 2.38, v2 = 2.34 turn/s (both > 0, `ENC_SIGN` = +1/+1 is correct) |
| `pwm -1500` | v1 = −2.36, v2 = −2.41 |
| Speed vs PWM | 1000: 1.52, 2000: 3.16, 3000: 4.80, 4000: 6.21 turn/s → `SAFETY_V_LIMIT` = 7.5 |
| Dead zone | shaft starts at pwm 240 (6 %); 0.27 turn/s there |

## Open-loop step (u: 0 → 30 %, angle of encoder 2)

Max slope k = 684 °/s, L = 0.13 s (tangent through the straight part: 676 °/s, 0.11 s).
R = k/30 = 22.8 °/(s·%), a = R·L ≈ 3.0 °/%.
ZN: P Kp = 0.33; PI Kp = 0.30, Ki = 0.75; PID Kp = 0.40, Ki = 1.50, Kd = 0.026.

## Closed loop, mode 2, step 0 → 90° and 30 s profile (J = mean |e|)

| Kp / Ki / Kd | fb | Overshoot | e_ss [°] | J [°] |
|---|---|---|---|---|
| 0.36 / 0 / 0 (ZN from enc. 1) | 1 | 24 % | limit cycle ±20° | – |
| 0.43 / 1.76 / 0.026 (ZN) | 1 | 53 % | limit cycle | – |
| 0.33 / 0 / 0 (ZN) | 2 | 3 % | 2 | 9.0 |
| 0.30 / 0.75 / 0 (ZN) | 2 | 38 % | | 6.4 |
| 0.40 / 1.50 / 0.026 (ZN) | 2 | 40 % | | 5.8 |
| 1.5 / 0 / 0 | 2 | 16 % | −1.4 | 3.4–3.6 |
| 1.0 / 1.0 / 0 | 2 | 23–25 % | −0.4 … −1.5 | 3.1–3.2 |
| 2.5 / 2 / 0.15 | 2 | 6–7 % | −1.0 … −1.4 | 3.0 |
| 2 / 1 / 0.15 | 2 | 5 % | −2.1 | |
| 3 / 5 / 0.25 | 2 | 11 % | −1.0 | 3.4 |

Manual tracking in mode 0 (profile recorded as target) works; a simple scripted
"student" (120 ms reaction) reached J = 14.4°.

## Fixed during testing

- `run` in mode 0 stored the profile with the turn/s resolution and clipped it at 32.8°;
  the capture now uses 0.1° units whenever the profile exceeds ±32.

## Velocity control of the output shaft (mode 1, fb 1) — used by MKT VL4

Open-loop step u 20 → 40 % (velocity of encoder 1, after 1.5 s at 20 %): ω0 = 1.21, ω∞ = 2.54 turn/s,
K = 0.066 (turn/s)/%, tangent at the inflection L ≈ 0.05–0.07 s, T ≈ 0.08–0.12 s — a clean S curve.
The motor velocity (encoder 2) is not usable for ZN: it jumps, dips on the coupling and only then settles.

Closed loop, step ref 2 → 4 turn/s, and a 30 s profile 1 → 3 → ramp 5 → 2 → ramp 1 turn/s
(ref streamed by the PC as `ref` commands, recorded with `capture 3000 10`; J = mean |e|):

| Kp / Ki / Kd | Overshoot | e_ss [turn/s] | J [turn/s] |
|---|---|---|---|
| 18.1 / 0 / 0 (ZN, L 0.070, T 0.083) | 0 % | 1.87 | 1.43 |
| 30 / 0 / 0 | 0 % | 1.33–1.57 | 1.08–1.09 |
| 40 / 0 / 0 | oscillates | | |
| 16.3 / 70.5 / 0 (ZN) | 4 % | 0.01 | 0.082 |
| 20 / 90 / 0 | 9–10 % | 0.00 | 0.079–0.084 |
| 21.8 / 156 / 0.76 (ZN) | 9 % | 0.00 | 0.058 |
| 20 / 150 / 1.2 | 8–10 % | 0.01 | 0.052–0.061 |
| 46.5 / 502 / 1.08 (ZN from a steeper hand-drawn tangent, L 0.046, T 0.119) | 37 % | | 0.69 |

### v2.2.1 (2026-11-17): profile in both directions, run on the MCU, Admin autotuning

Profile (30 s, `prof add` + `run 3000 10`): 1 → 3 → ramp 5 → −2 → ramp −5 → 2 → ramp −2 → 4 → 0 turn/s.
Open loop 20 → 40 %: K = 0.066 (turn/s)/%, L = 0.050–0.053 s, T = 0.106–0.115 s.

| Kp / Ki / Kd | Overshoot (step 2 → 4) | J [turn/s] |
|---|---|---|
| 32.1 / 0 / 0 (ZN) | 0 % | 1.18 (1.27 with Kp 32.9: oscillates at times) |
| 27.3 / 0 / 0 | | 1.14 (best P; 24.1 → 1.19, 30.5 → 1.15) |
| 28.9 / 174 / 0 (ZN) | 54 % | 1.44 (oscillates, u ±100 %) |
| 14.5 / 109 / 0 | 6 % | 0.25 (best PI) |
| 38.5 / 387 / 0.96 (ZN) | 24 % | 0.49 |
| 19.3 / 149 / 0.96 | 8 % | 0.14 (best PID with overshoot < 10 %) |
| 25 / 250 / 1.2 | 17 % | 0.13 |

The Admin tab of the VL4 page found these automatically in 9 minutes (ZN, 4 P profiles,
9 PI and 12 PID step tests, 3 profiles each) and stored limits = best J × 1.15 on the
model with `cal`. An aggressive PID step (Kp 25, Ki 193, Kd 1.25) tripped the `encoder`
safety stop once (more than 10 missed edges in 100 ms on encoder 1).
