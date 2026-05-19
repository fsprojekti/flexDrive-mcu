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
