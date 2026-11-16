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
