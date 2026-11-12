# flexDrive-MCU — Upgrade Specification v2.0
## Gimbal Motor Torque Control

---

## 1. Motivation

The v1 firmware drives a **brushed DC motor** via a single PWM + direction signal (H-bridge).
The upgrade replaces it with the **T-Motor GB36-2 brushless gimbal motor** and shifts the
control paradigm from velocity PWM to **direct torque control** via Field-Oriented Control (FOC).

Gimbal motors are designed for maximum torque smoothness and low cogging — ideal for
precise force/torque-controlled applications.

---

## 2. Motor Specification — T-Motor GB36-2 KV30

| Parameter | Value |
|---|---|
| Type | 3-phase BLDC, outrunner, gimbal type |
| KV | 30 rpm/V |
| Torque constant $K_t$ | $\frac{60}{2\pi \times 30} \approx \mathbf{0.318}$ **Nm/A** |
| Back-EMF constant $K_e$ | 0.318 V·s/rad |
| Configuration | **12N14P** → **7 pole pairs** |
| Internal resistance | **16.4 Ω** line-to-line → **8.2 Ω per phase** |
| Phase inductance $L$ | ~16 mH (line-to-line, estimated) → ~8 mH per phase |
| Rated torque | **0.36 Nm** |
| Rated current | $0.36 / 0.318 \approx 1.1$ A |
| Peak current | ~3 A (short duration) |
| Supply voltage | **3S–6S LiPo (11.1 V – 22.2 V)** |
| Recommended supply | **3S LiPo (11.1 V)** |
| Weight (incl. cable) | **128 g** |
| Dimensions | **ø42.5 × 30 mm** |
| Cable length | 35 cm |
| Hollow shaft | Yes |

### Key Electrical Model (per-phase)

$$V_{phase} = R \cdot i + L \cdot \frac{di}{dt} + K_e \cdot \omega_e$$

Where $\omega_e = p \cdot \omega_m$ (electrical angular velocity, $p = 7$ pole pairs).

---

## 3. Control Architecture

### 3.1 Control Hierarchy

```
┌──────────────────────────────────────────────────────────────────┐
│  ESP32 (TTGO LoRa32)                                             │
│                                                                  │
│  Level 3 (optional): Position control   [ ~200 Hz ]             │
│  reference: θ_ref [rad]  →  output: ω_ref [rad/s]              │
│                                                                  │
│  Level 2: Velocity control (PID)        [ ~1 kHz ]              │
│  reference: ω_ref [rad/s]  →  output: Iq_ref [A]               │
│                                          │                       │
│                              UART (VESC protocol)               │
└──────────────────────────────────────────┼───────────────────────┘
                                           ▼
┌──────────────────────────────────────────────────────────────────┐
│  VESC 6  (runs internally)                                       │
│                                                                  │
│  Level 1: Torque / Current control (FOC) [ ~20 kHz ]            │
│  reference: Iq_ref [A]  →  output: 3-phase PWM voltages         │
│  Built-in current sensing + rotor encoder                        │
└──────────────────────────────────────────────────────────────────┘
```

### 3.2 Torque Control (FOC Inner Loop)

Torque is controlled by regulating the **q-axis current** $I_q$:

$$T = K_t \cdot I_q = 0.318 \cdot I_q \quad [\text{Nm}]$$

The full FOC loop (Clarke/Park transforms, current PI controllers, SVPWM) runs **entirely inside the VESC** at ~20 kHz. The ESP32 only sends a current reference via UART:

```cpp
vesc.setCurrent(Iq_ref);  // e.g. 1.1 A → 0.35 Nm
```

VESC configures its own current loop gains via **VESC Tool** (desktop app over USB).

### 3.3 Operating Modes

| Mode | Reference | Inner loop | Description |
|------|-----------|-----------|-------------|
| 0 | — | — | Stop / Idle |
| 1 | $I_{q,ref}$ [A] | Current FOC | **Direct torque control** |
| 2 | $\omega_{ref}$ [rad/s] | Velocity → Current | Velocity control via torque |
| 3 | $\theta_{ref}$ [rad] | Position → Velocity → Current | Position control via torque |

---

## 4. Hardware Changes

### 4.1 Overview

| Subsystem | v1 (current) | v2 (new) |
|---|---|---|
| MCU | ESP32 (TTGO LoRa32) | ESP32 (TTGO LoRa32) — **unchanged** |
| Motor | Brushed DC | T-Motor GB36-2 BLDC KV30 |
| Motor driver | H-bridge (1× PWM + 1× DIR) | **ODrive S1** (smart FOC ESC) |
| Position sensor | Quadrature encoder (4096 CPR) | **MA702** — on-board ODrive S1 (magnet on shaft only) |
| Current sensing | None | **Built into ODrive S1** (inline, 2-phase) |
| Interface ESP32↔driver | PWM + DIR | **UART** (ODrive ASCII protocol, isolated) |
| Power supply | — | 3S LiPo 11.1 V |

### 4.2 Motor Driver — ODrive S1

**ODrive S1** (ODrive Robotics, open-source hardware, €149)

> ODrive S1 handles everything internally: current sensing, FOC, gate driving, and
> protection. The ESP32 sends only a **torque reference** over isolated UART using
> the ODrive ASCII protocol. Configuration is done via **Web GUI** or `odrivetool` over USB.

| Parameter | Value |
|---|---|
| Input voltage | 12 – 48 V (50.5 V max) |
| Continuous current | 40 A (with heat spreader) |
| Current sensing | **Built-in inline, 2-phase** |
| PWM frequency | 24 kHz |
| Control loop | 8 kHz (current loop 24 kHz, upcoming) |
| FOC | Yes — runs internally |
| On-board encoder | **MA702** (magnetic, absolute) |
| Offboard encoder | SPI, ABI quadrature, Hall, RS485 |
| Dual encoder support | **Yes** (load + motor) |
| Interface to ESP32 | **UART isolated** (115 200 baud) |
| Also supports | CAN 2.0B / CAN-FD, USB, PWM, Step/Dir, Analog |
| Configuration | Web GUI (browser) or `odrivetool` (USB) |
| On-board brake chopper | **Yes** (with included 2Ω 50W resistor) |
| Protection | Over-current, over-temperature, over/under voltage |
| Mass | 35 g (solder pads) / 55 g (screw terminals) |
| Dimensions | 66 × 50 mm |

#### ODrive S1 Motor Configuration (via Web GUI / odrivetool)

| Setting | Value |
|---|---|
| `motor.motor_type` | `MOTOR_TYPE_HIGH_CURRENT` |
| `motor.pole_pairs` | 7 |
| `motor.resistance_calib_max_voltage` | 2.0 V |
| `motor.requested_current_range` | 3.0 A |
| `motor.current_control_bandwidth` | 100 (rad/s) |
| `encoder.mode` | `ENCODER_MODE_SPI_ABS_MA732` (MA702, on-board) |
| `controller.control_mode` | `CONTROL_MODE_TORQUE_CONTROL` |
| `controller.input_torque` | *(set at runtime via UART)* |
| `axis.motor.config.current_lim` | 1.5 A |
| `axis.motor.config.current_lim_margin` | 1.0 A |

> Run `odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION` then
> `AXIS_STATE_ENCODER_OFFSET_CALIBRATION` on first boot. Save with `odrv0.save_configuration()`.

#### UART Pin Assignment (ESP32 → ODrive S1)

| Signal | ESP32 GPIO | ODrive S1 pin |
|---|---|---|
| TX | 17 | UART\_RX (isolated) |
| RX | 16 | UART\_TX (isolated) |
| GND | GND | GND |

### 4.3 Position Sensor — MA702 (on-board ODrive S1)

| Parameter | Value |
|---|---|
| Type | Absolute magnetic rotary encoder |
| Location | **Soldered directly on the ODrive S1 PCB** |
| Interface | SPI (internal, no external wiring needed) |
| Resolution | 14-bit absolute (16 384 steps/rev) |
| Angular accuracy | ±0.2° |
| Required hardware | **6 mm diametric neodymium magnet glued to motor shaft end** |
| External board | **None** — no separate encoder module |

```
Motor shaft end
      │
   [magnet]   ← 6mm diametric neodymium, centred on shaft, ~1–3mm air gap
      │
   [MA702]    ← on ODrive S1 PCB, reads angle at FOC rate
```

ODrive S1 reads the encoder internally for FOC commutation. Position and velocity
feedback is returned to the ESP32 via UART (`f 0` reply: `pos vel Iq`).

### 4.4 Current Sensing

Current sensing is **fully internal to the ODrive S1**. Inline shunts on phases A and B
with on-board sense amplifiers. No connections to ESP32 ADC needed.

The measured current is read back by the ESP32 via the ODrive ASCII telemetry stream:

```cpp
// ODrive ASCII: send 'f 0\n', parse reply: "pos vel Iq"
Serial2.println("f 0");
// reply: "0.123 45.6 1.05\n"  (pos [turns], vel [turns/s], Iq [A])
```

---

## 5. Firmware Architecture

### 5.1 Recommended Library

**ODrive ASCII protocol** over `Serial2` (no external library required)
- Built into ODrive firmware, works over isolated UART at 115 200 baud
- Simple newline-terminated text commands
- Provides position, velocity, and current feedback in one packet

```cpp
void setup() {
  Serial2.begin(115200, SERIAL_8N1, 16, 17); // RX=16, TX=17
}

// Set torque: T = Kt * Iq = 0.318 * amps
void setTorque(float torqueNm) {
  float iq = constrain(torqueNm / 0.318f, -1.5f, 1.5f);
  Serial2.printf("c 0 %.4f\n", iq);  // 'c <axis> <current>'
}

// Read feedback: pos [turns], vel [turns/s], Iq [A]
void requestFeedback() {
  Serial2.println("f 0");            // 'f <axis>'
  // parse reply: "<pos> <vel> <Iq>\n"
}
```

### 5.2 FreeRTOS Task Structure

| Task | Priority | Period | Responsibility |
|------|----------|--------|---------------|
| `controlTask` | 2 (highest) | 1 ms (1 kHz) | Outer velocity/position loop → `setTorque()` |
| `odriveRxTask` | 2 | 10 ms | Send `f 0`, parse pos/vel/Iq from ODrive S1 |
| `cliTask` | 1 | 10 ms | Serial command parsing (SimpleCLI) |
| `displayTask` | 1 | 100 ms | TFT status display |
| `telemetryTask` | 1 | `ts` ms | Serial plotter / data logging |

> The FOC inner loop runs entirely inside the ODrive S1 — the ESP32 is not involved.

### 5.3 Serial CLI Commands (updated)

Retained from v1 where applicable, new commands added:

| Command | Args | Description |
|---------|------|-------------|
| `init` | — | Initialise and align motor (rotor alignment) |
| `run` | — | Enable control output |
| `stop` | — | Disable output, motor coasts |
| `list` | — | Print all parameters |
| `version` | — | Firmware version |
| `mode` | `0–3` | Control mode (0=stop, 1=torque, 2=velocity, 3=position) |
| `ref` | `float` | Reference: [A] for mode 1, [rad/s] for mode 2, [rad] for mode 3 |
| `kp` | `float` | Outer loop proportional gain |
| `ki` | `float` | Outer loop integral gain |
| `kd` | `float` | Outer loop derivative gain |
| `i_max` | `float` | Maximum $I_q$ command [A] sent to ODrive S1 (default 1.5) |

> FOC current loop gains are configured inside ODrive S1 via **Web GUI** or `odrivetool`
> (USB) — not via the ESP32 CLI.
| `ts` | `int` | Outer loop sampling time [ms] |
| `alpha` | `float` | Velocity filter coefficient (0–1) |
| `plot` | `0/1` | Enable raw serial data stream |
| `plot_period` | `int` | Plot decimation (multiples of `ts`) |
| `plotOpen` | — | Serial Plotter: torque, velocity |
| `plotClosed` | — | Serial Plotter: ref, velocity, torque |

### 5.4 Telemetry Format (Serial Plotter)

**Mode 1 – Torque control:**
```
iq_ref:<float> iq:<float> torque:<float>
```

**Mode 2 – Velocity control:**
```
vel_ref:<float> vel:<float> iq:<float> torque:<float>
```

**Mode 3 – Position control:**
```
pos_ref:<float> pos:<float> vel:<float> iq:<float>
```

---

## 6. Control Parameter Initialisation

### 6.1 FOC Current Loop — configured in ODrive Web GUI / odrivetool

Connect ODrive S1 via USB and open **Web GUI** (browser) or run `odrivetool`. Use
**Motor Calibration** to auto-measure R and L, then enter:

| Parameter | Value | ODrive config key |
|---|---|---|
| Phase resistance | 8.2 Ω | auto-measured by calibration |
| Phase inductance | 8 mH | auto-measured by calibration |
| Flux linkage $\lambda$ | 0.318 V·s/rad | auto-measured by calibration |
| Pole pairs | 7 | `motor.pole_pairs` |
| Current limit | 1.5 A | `motor.config.current_lim` |
| Current bandwidth | 100 rad/s | `motor.config.current_control_bandwidth` |
| Encoder | MA702 on-board | `encoder.mode = ENCODER_MODE_SPI_ABS_MA732` |

> Run `AXIS_STATE_FULL_CALIBRATION_SEQUENCE` on first boot.

### 6.2 Velocity Loop (outer, 1 kHz, runs on ESP32)

| Gain | Initial value | Unit |
|------|--------------|------|
| $K_p$ | 0.05 | A·s/rad |
| $K_i$ | 0.5 | A/rad |
| $K_d$ | 0.0 | — |
| $I_{q,max}$ | 1.5 | A |

---

## 7. Safety & Protection

| Condition | Where handled | Action |
|-----------|--------------|--------|
| $\|I_q\| > I_{max}$ | ESP32 | Clamp before sending to VESC |
| Phase overcurrent | **ODrive S1 hardware** | ODrive cuts output, sets error flag |
| Over-temperature | **ODrive S1 hardware** | ODrive ramps down current |
| Over/under voltage | **ODrive S1 hardware** | ODrive cuts output |
| ODrive error detected | ESP32 `odriveRxTask` | Parse error from `f` reply, stop + print |
| UART timeout (no reply) | ESP32 | Send `c 0 0\n`, print warning |

---

## 8. Wiring Diagram Summary

```
    3S LiPo 11.1 V ──────────────────────────────┐
    GND ─────────────────────────────────────┐   │
                                             │   │
                               ┌─────────────┴───┴──────────────────┐
                               │  ODrive S1  (66×50mm, 35g)        │
    ESP32 GPIO17 (TX) ─────────┤ UART_RX (isolated, 115200)        │
    ESP32 GPIO16 (RX) ─────────┤ UART_TX (isolated)              ├──── Phase A ─┐
    GND ───────────────────────┤ GND                             ├──── Phase B ──┤ GB36-2
                               │                                 ├──── Phase C ─┘
                               │  [built-in current sensing]     │
                               │  [FOC @ 24kHz PWM]              │
                               │  [MA702 encoder, on-board]      │
                               │                                 │
                               │  2Ω/50W  ──── brake resistor   │
                               └─────────────────────────────────┘

    ESP32 ─── USB/Serial ──→ PC (Matlab, Serial Monitor)
    ODrive S1 ─ USB ────→ PC (Web GUI / odrivetool, first-time config only)
```

---

## 9. Development Milestones

| # | Task | Status |
|---|------|--------|
| 1 | Assemble hardware: ODrive S1 + GB36-2 + 6mm diametric magnet on shaft | ⬜ |
| 2 | Configure ODrive S1 via Web GUI: motor type, pole pairs, current limits | ⬜ |
| 3 | Run `AXIS_STATE_FULL_CALIBRATION_SEQUENCE` (auto-measures R, L, λ, encoder offset) | ⬜ |
| 4 | Verify torque step: send `c 0 1.0\n` via UART → measure output torque | ⬜ |
| 5 | Validate linearity: $T$ vs $I_q$ across range 0 – 1.5 A | ⬜ |
| 6 | Implement ESP32 UART interface (ODrive ASCII protocol) | ⬜ |
| 7 | Implement and tune velocity loop (mode 2) on ESP32 | ⬜ |
| 8 | Update Simulink model (`openLoopII.slx`) for torque plant | ⬜ |
| 9 | Serial CLI integration and telemetry testing | ⬜ |
| 10 | TFT display update for new state variables (Iq, pos, vel, error) | ⬜ |

---

## 10. References

- [T-Motor GB36-2 Product Page](https://store.tmotor.com/product/gb36-2-gimbal-type.html)
- [SimpleFOC Documentation](https://docs.simplefoc.com)
- [ODrive S1 Product Page](https://eu.odriverobotics.com/shop/odrive-s1)
- [ODrive S1 Documentation](https://docs.odriverobotics.com)
- [ODrive ASCII Protocol](https://docs.odriverobotics.com/v/latest/interfaces/ascii-protocol.html)
- [MA702 Datasheet (MPS)](https://www.monolithicpower.com/en/ma702.html)
