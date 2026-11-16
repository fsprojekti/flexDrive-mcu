# flexDrive-mcu

Application for control of the flexDrive model. This application is intended to be used with the [flexDrive hardware](https://lampa-fs.gitbook.io/knowledge-base/laboratorijska-oprema/makete/flexdrive).

---

## FlexDrive MCU Firmware and MATLAB/Simulink Communication Block

This repository provides the software required for controlling the FlexDrive model in real time using an ESP32 microcontroller and MATLAB/Simulink. It supports both open-loop (direct PWM control) and closed-loop (PID speed control using encoder feedback) operation modes.

---

## Repository Structure


---

## Operation Modes

### Open Loop Control
Open loop control is used to control the motor without feedback. The motor is controlled by setting the desired speed (PWM value) and direction. The motor will run at the set speed until the speed is changed or the motor is stopped. Both encoders are continuously read, and their data is transmitted to the host.

### Closed Loop Control
Closed loop control is used to regulate the motor speed using feedback from the encoders. A PID controller is employed to minimize the error between the reference velocity and the actual velocity. This ensures precise control of motor speed under varying load conditions.

---

## Firmware Overview

### Arduino Code (ESP32)
- **Location:** `firmware/`
- **Description:**
    - Designed for the ESP32 microcontroller acting as the controller for the FlexDrive model.
    - **Key Features:**
        - Controls a DC motor and reads data from encoders.
        - Supports two operation modes:
            - **Open Loop:** Motor is driven directly with a fixed PWM signal.
            - **Closed Loop:** PID speed control using encoder feedback.
        - Provides a serial CLI (Command Line Interface) for computer-based control.
        - **Additional Commands:**
            - `ts <value>`: Set the sampling time (in ms) for the control loop.
            - `version`: Fetch and display the firmware version.

### MATLAB/Simulink Communication Block
- **Location:** `matlab/`
- **Description:**
    - Not a simulation model; it enables real-time communication with the ESP32.
    - **Key Features:**
        - Send commands from MATLAB to control the motor.
        - Receive encoder data directly in MATLAB.
        - Facilitate custom control algorithm implementations in Simulink.

---

## Getting Started

### 1. Uploading the Firmware to ESP32

1. Clone or download the repository.
2. Navigate to the `firmware/` directory.
3. Open the project in Arduino IDE or PlatformIO.
4. Connect your ESP32 to your computer.
5. Upload the program (firmware) to your ESP32.

### 2. Using the CLI for Control

After uploading the firmware, the ESP32 can be controlled via a serial connection. The default serial settings are:

- **Baudrate:** 115200
- **Data Bits:** 8
- **Parity:** None
- **Stop Bits:** 1

You can use any serial terminal (Arduino Serial Monitor, PuTTY, CoolTerm, or MATLAB) to send commands.

---

## CLI Command Reference

Commands are entered via the serial interface and follow the format `<command> [parameters]`. Below is a detailed reference of the available commands:

| **Command**     | **Parameters**            | **Description**                                                       | **Example**                 | **Expected Output**                                     |
|-----------------|---------------------------|------------------------------------------------------------------------|-----------------------------|--------------------------------------------------------|
| `init`          | None                      | Initializes the system.                                                | `init`                      | "System Initialized."                                  |
| `run`           | None                      | Starts the system (sets the state to `Run`).                           | `run`                       | "System Started."                                      |
| `stop`          | None                      | Stops the system (sets the state to `Stop`).                           | `stop`                      | "System Stopped."                                      |
| `list`          | None                      | Displays the current system status.                                    | `list`                      | Lists mode, state, PWM, encoder data, etc.             |
| `mode`          | `0` or `1`                | Sets the mode: `0` for open loop, `1` for closed loop.                 | `mode 1`                    | "Mode set to: Closed Loop."                            |
| `pwm`           | `0` to `PWM_MAX`          | Sets the PWM output value.                                             | `pwm 2000`                  | "PWM set to: 2000."                                    |
| `dir`           | `0` or `1`                | Sets the direction: `0` for forward, `1` for reverse.                  | `dir 0`                     | "Direction set to: Forward."                           |
| `ref`           | `-10.0` to `10.0`         | Sets the reference velocity (in turns per second).                     | `ref 0.5`                   | "Reference Velocity set to: 0.5."                      |
| `kp`            | `0.0` to `100.0`          | Sets the proportional gain for the PID controller.                     | `kp 0.1`                    | "Kp set to: 0.1."                                      |
| `ki`            | `0.0` to `100.0`          | Sets the integral gain for the PID controller.                         | `ki 0.05`                   | "Ki set to: 0.05."                                     |
| `kd`            | `0.0` to `100.0`          | Sets the derivative gain for the PID controller.                       | `kd 0.01`                   | "Kd set to: 0.01."                                     |
| `pwm_max`       | At least `100`            | Sets the maximum allowable PWM value.                                  | `pwm_max 4000`              | "PWM Max set to: 4000."                                |
| `plot`          | `0` or `1`                | Enables or disables plotting of encoder and control data.              | `plot 1`                    | "Plotting Enabled."                                    |
| `ts`            | (Integer, in ms)          | Sets the sampling time for the control loop.                           | `ts 20`                     | "Sampling time (ts) set to: 20."                       |
| `version`       | None                      | Displays the firmware version.                                         | `version`                   | "Firmware Version: 1.0.0"                              |

> **Note:** Commands are sent in the form:
> ```
> command <parameter1> <parameter2> ...
> ```
> Parameters are separated by spaces. If a command is entered incorrectly, the ESP32 will return an error message.

---

## Examples

### 1. Running the System in Open Loop Mode
To operate in open loop mode, set the mode to `0`, specify the PWM value, and start the system.

```bash
mode 0
#-> Mode set to: Open Loop.
pwm 1500
#-> PWM set to: 1500.
dir 0
#-> Direction set to: Forward.
run
#-> System Started.

---

## Firmware v2.1 (`firmware/flexdrive/`)

The sections above describe the v1 firmware (`arduino/`). Firmware v2.x lives in
`firmware/flexdrive/` and talks to the web page `web/index.html` (Web Serial, Chrome/Edge)
with JSON lines. The full protocol is documented in `firmware/flexdrive/telemetry.h`.

- Serial: **921600 baud**, 8N1, commands are plain text lines.
- Sampling, velocity observer and controller run at **1 kHz** in one task; telemetry at 50 Hz.
- Velocity is the Luenberger observer estimate in **turns per second of the output shaft**
  for both encoders (encoder 2 already includes the 20.4 gear ratio).

### Modes

| Mode | Command | Motor is driven by |
|---|---|---|
| 0 – open loop | `mode 0` | `pwm <n>` |
| 1 – closed loop | `mode 1` | velocity PID on the feedback encoder (`fb`) |

Controller (mode 1), same units as v1 so v1 gains stay meaningful:

```
e  = ref - v_fb                      [turn/s]
u  = Kp*e + uI - Kd*d(v_fb)/dt       [% of PWM max], limited to +-umax
uI = uI + Ki*e*dt                    conditional integration (anti-windup), |uI| <= umax
```

`Kp` [%/(turn/s)], `Ki` [%/turn], `Kd` [%·s/turn]. The integrator stores `uI` (already
multiplied by `Ki`), so changing `Ki` while running does not make the output jump;
`ki 0` clears it.

### Commands

| Command | Range | Default | Reply |
|---|---|---|---|
| `mode <0\|1>` | | 0 | `state` |
| `pwm <n>` | −4000 … 4000, mode 0 only | 0 | – (error in mode 1) |
| `ref <x>` | −10 … 10 turn/s | 0 | `state` |
| `kp <x>` | 0 … 200 | 0 | `state` |
| `ki <x>` | 0 … 2000 | 0 | `state` |
| `kd <x>` | 0 … 10 | 0 | `state` |
| `ts <ms>` | 1 … 50 (controller period, output held in between) | 1 | `state` |
| `fb <1\|2>` | 1 = output shaft (4096 CPR), 2 = motor encoder | 1 | `state` |
| `umax <x>` | 1 … 100 % | 100 | `state` |
| `stop` | motor off, `ref 0`, back to mode 0 | | `state` |
| `step <v> <n> [pre]` | capture `n` samples; at sample `pre` (default 200) apply `v` — PWM in mode 0, ref in mode 1 — in that same sample | | `cap_start`, then the dump |
| `capture <n>` | 1 … 5000 samples at 1 kHz | | `cap_start`, then the dump |
| `wd <ms>` | 0 (off) or 200 … 10000 | 0 | `state` |
| `ping` | watchdog keep-alive | | – |
| `state`, `info`, `help` | | | |
| `tel <0\|1>` | telemetry stream off/on | 1 | – |
| `obs_bw <f> [f2]`, `obs_zeta <z>` | observer tuning | 70/40 Hz, 0.707 | `info` |

Capture dump columns: `t_us,c1,c2,v1,v2,pwm,ref,ui` (read them by name from `fields`
in `cap_begin`; new columns are only appended).

### Safety stops

Each one sets `stop` and sends `{"type":"err","msg":...}` followed by `state`:

| `msg` | When |
|---|---|
| `overspeed` | mode 1, speed of the feedback encoder above `SAFETY_V_LIMIT` for 50 ms |
| `runaway` | mode 1, output saturated while the shaft turns against it and speeds up (wrong feedback sign) |
| `encoder` | mode 1, more than 10 missed edges on the feedback encoder within 100 ms |
| `watchdog` | `wd` enabled and no command line received within the period |

Before first use on a new board: run `pwm 1500` and check that `v1 > 0` and `v2 > 0`;
otherwise flip `ENC_SIGN` in `config.h`. Measure the speed at `pwm 4000` and set
`SAFETY_V_LIMIT` to about 1.2× that value (see `firmware/TESTS_v2.1.md`).

### Example: step response of a P controller

```
mode 1
kp 20
ref 2.5        # wait until the speed settles
step 5 3000    # 200 samples before the step, ref 2.5 -> 5 at sample 200
```

## Firmware v2.2: angle control (servo) and reference profiles

v2.2 adds everything from v2.1 plus:

| Command | Range | Reply | Notes |
|---|---|---|---|
| `mode 2` | | `state` | PID on the **angle** of the feedback encoder; on entry `ref` = current angle (no jump) |
| `ref <x>` in mode 2 | −3000 … 3000 ° | `state` | |
| `zero` | mode 0 only | `state` | angle 0 at the current position (counts `p1`/`p2`/`c1`/`c2` restart at 0) |
| `prof clear` / `prof add <t_ms> <v>` | up to 24 points, t strictly increasing | `{"type":"prof","n":k}` | piecewise-linear ref(t) |
| `run <n> [dec]` | | `cap_start`, then the dump | starts the profile and a capture in the same sample |
| `capture <n> [dec]` | dec 1 … 50 | | one row every `dec` ms (5000 rows × 10 ms = 50 s) |

Mode 2 control law (same PID, same anti-windup):

```
e  = ref - theta_fb                  [deg], theta since the last "zero"
u  = Kp*e + uI - Kd*omega_fb         omega_fb = observer velocity [deg/s], filtered 5 ms
```

`Kp` [%/°], `Ki` [%/(°·s)], `Kd` [%·s/°]. The profile drives `ref` in every mode; in mode 0
it is only recorded (target for manual tracking). Setting `ref`, `stop` or a mode change
ends it. Telemetry gains `"pt"` (ms since the profile started, −1 if none); `cap_begin`
gains `"dec"` and `"run"`.

**Feedback encoder for angle control: use `fb 2`.** The output shaft (encoder 1) sits
behind the elastic coupling. With `fb 1` the shaft keeps swinging on the coupling
(~2 Hz, ±20°) while the motor sticks in its dead zone, and the loop limit-cycles even with
ZN gains. With `fb 2` the loop is well behaved and encoder 1 shows the coupling swing.

Measured on the lab unit (2026-11-16) are in `firmware/TESTS_v2.2.md`. The student
page for the lab exercise (MKT VL4) is maintained in the course repository.
