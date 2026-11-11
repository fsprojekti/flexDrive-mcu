# Firmware v2.1 — acceptance tests on hardware

Status: **not run yet.** The firmware compiles without warnings
(arduino-cli, `esp32:esp32:ttgo-lora32`, core 3.3.12: 306 kB flash, 123 kB static RAM incl. the
5000-sample capture buffer), but none of the tests below has been run on a flexDrive board.

Run them in a serial terminal at 921600 baud (`tel 0` silences the telemetry stream) or in
`web/index.html`. Fill in the *Result* column and save the CSV files next to this document
(`tests_v2.1/Tn.csv`).

## 0. Before the tests

| Step | Command | Result |
|---|---|---|
| Upload firmware, check version | `info` → `"ver":"2.1.0"` | |
| Encoder direction | `pwm 1500`, read `v1`, `v2` in `tel` | v1 = ……, v2 = …… (both must be > 0, otherwise flip `ENC_SIGN` in `config.h`) |
| Maximum speed | `pwm 4000`, wait 2 s | v1 = …… turn/s → set `SAFETY_V_LIMIT` ≈ 1.2× = …… |

## 1. Tests

| # | Test | Expected | Result |
|---|---|---|---|
| T1 | `pwm 1500`, read `v1`, `v2` | both > 0, roughly equal | |
| T2 | `mode 1`, `kp 20`, `ref 5`, then `capture 5000` | `dt_us` 1000 ± 50 µs, no gap > 2 ms | |
| T3 | `mode 1`, `kp 20`, `ki 0`, `ref 5` | steady speed < 5, constant error (P controller) | |
| T4 | as T3 + `ki 50` | steady-state error ≈ 0 (< 0.05 turn/s) | |
| T5 | `ref 2.5`, wait, `step 5 3000 200` | in the CSV `ref` and `pwm` both change in data row 201 (index 200) | |
| T6 | PI as in T4, hold the output shaft by hand for 3 s, release | `sat` = 1 while held, `ui` stays ≤ `umax`, overshoot after release < 50 % of ref | |
| T7 | `mode 0`, `step 2000 3000 200` | PWM step in data row 201 | |
| T8 | `stop` while a capture is being dumped | motor stops in < 100 ms, `state` line appears between the CSV rows | |
| T9 | `wd 1000`, then send nothing | after ~1 s `err watchdog`, `state` with mode 0, motor stopped | |
| T10 | set `ENC_SIGN` of encoder 1 to −1 (test build only), `mode 1`, `kp 20`, `ref 2` | `err runaway` (or `overspeed`), motor stopped | |
| T11 | `mode 1`, then `pwm 1000` | `err pwm: closed loop active (use mode 0)`, motor does not react | |
| T12 | `ref 5`, `kp 20`, running; `ki 0` → `ki 50` while running | no jump in the output (`uc` in `tel`, or a `capture` across the change) | |
| T13 | `web/index.html`: connect, *Nadzor*, *Nastavljanje observerja* test | works as before (capture parsing uses the first 6 columns) | |

## 2. Measurements for the VL4 web page and lab goals

| Measurement | Procedure | Result |
|---|---|---|
| Static characteristic | `mode 0`, `pwm` 0, 100, 200, …, 4000; wait for steady state; mean `v1` over 1 s | table / CSV: |
| Dead zone | lowest PWM at which the shaft keeps turning | |
| Open-loop step | `mode 0`, `pwm 0`, `step 2000 3000 200` | CSV; first-order fit: K = …… (turn/s)/%, τ = …… ms |

## 3. Notes / deviations
