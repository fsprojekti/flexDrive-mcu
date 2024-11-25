# flexDrive-mcu

Application for control of the flexDrive model. This application is intended to be used with the [flexDrive hardware](https://lampa-fs.gitbook.io/knowledge-base/laboratorijska-oprema/makete/flexdrive).

---

## **Operation Modes**

### **Open Loop Control**
Open loop control is used to control the motor without feedback. The motor is controlled by setting the desired speed (PWM value) and direction. The motor will run at the set speed until the speed is changed or the motor is stopped. Both encoders are continuously read, and their data is transmitted to the host.

### **Closed Loop Control**
Closed loop control is used to regulate the motor speed using feedback from the encoders. A PID controller is employed to minimize the error between the reference velocity and the actual velocity. This ensures precise control of motor speed under varying load conditions.

---

## **CLI Command Reference**

The CLI (Command Line Interface) provides commands for configuring and controlling the system. Commands are entered via the serial interface and follow the format `<command> [parameters]`. Below is a detailed reference of the available commands:

| **Command**     | **Parameters**            | **Description**                                                       | **Example**                 | **Expected Output**                                     |
|------------------|---------------------------|------------------------------------------------------------------------|-----------------------------|--------------------------------------------------------|
| `init`          | None                      | Initializes the system.                                                | `init`                      | "System Initialized."                                  |
| `run`           | None                      | Starts the system (sets the state to `Run`).                           | `run`                       | "System Started."                                      |
| `stop`          | None                      | Stops the system (sets the state to `Stop`).                           | `stop`                      | "System Stopped."                                      |
| `list`          | None                      | Displays the current system status.                                    | `list`                      | Lists mode, state, PWM, encoder data, etc.            |
| `mode`          | `0` or `1`                | Sets the mode: `0` for open loop, `1` for closed loop.                 | `mode 1`                    | "Mode set to: Closed Loop."                            |
| `pwm`           | `0` to `PWM_MAX`          | Sets the PWM output value.                                             | `pwm 2000`                  | "PWM set to: 2000."                                    |
| `dir`           | `0` or `1`                | Sets the direction: `0` for forward, `1` for reverse.                  | `dir 0`                     | "Direction set to: Forward."                           |
| `ref`           | `-10.0` to `10.0`         | Sets the reference velocity (in turns per second).                     | `ref 0.5`                   | "Reference Velocity set to: 0.5."                      |
| `kp`            | `0.0` to `100.0`          | Sets the proportional gain for the PID controller.                     | `kp 0.1`                    | "Kp set to: 0.1."                                      |
| `ki`            | `0.0` to `100.0`          | Sets the integral gain for the PID controller.                         | `ki 0.05`                   | "Ki set to: 0.05."                                     |
| `kd`            | `0.0` to `100.0`          | Sets the derivative gain for the PID controller.                       | `kd 0.01`                   | "Kd set to: 0.01."                                     |
| `pwm_max`       | At least `100`            | Sets the maximum allowable PWM value.                                  | `pwm_max 4000`              | "PWM Max set to: 4000."                                |
| `plot`          | `0` or `1`                | Enables or disables plotting of encoder and control data.              | `plot 1`                    | "Plotting Enabled."                                    |

---

## **Examples**

### **1. Running the System in Open Loop Mode**
To operate in open loop mode, set the mode to `0`, specify the PWM value, and start the system.

``` bash
mode 0
#->Mode set to: Open Loop.
pwm 1500
#->PWM set to: 1500.
dir 0
#->Direction set to: Forward.
run
#->System Started.
```
---

### **2. Running the System in Closed Loop Mode**
For closed loop control, configure the PID parameters and reference velocity, set the mode to `1`, and start the system.

``` bash
mode 1
#->Mode set to: Closed Loop.
kp 0.2
#->Kp set to: 0.2.
ki 0.05
#->Ki set to: 0.05.
kd 0.01
#->Kd set to: 0.01.
ref 0.8
#->Reference Velocity set to: 0.8.
run
#->System Started.
```



---

### **System Status Example**
After starting the system, use the `list` command to verify the configuration.

``` bash
list

#->Mode: Closed Loop
#->State: Run
#->PWM: 1500
#->Dir: Forward
#->Reference: 0.8
#->Kp: 0.2
#->Ki: 0.05
#->Kd: 0.01
#->PWM Max: 4000
#->Plot: Disabled
#->Encoder1 Position: 1200
#->Encoder1 Velocity: 0.75
#->Encoder2 Position: 1180
#->Encoder2 Velocity: 0.73
```