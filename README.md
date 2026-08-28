# NEMA-6AXIS Robotic Arm Controller

[![PlatformIO](https://img.shields.io/badge/PlatformIO-ESP32--S3-orange.svg)](https://platformio.org/)
[![Framework](https://img.shields.io/badge/Framework-Arduino%20%2B%20FreeRTOS-blue.svg)](https://docs.espressif.com/projects/arduino-esp32/)
[![Language](https://img.shields.io/badge/Language-C%2B%2B17-green.svg)](https://isocpp.org/)
[![Kinematics](https://img.shields.io/badge/Kinematics-Craig%20Modified%20DH-brightgreen.svg)](docs/ARM_GEOMETRY.md)
[![License](https://img.shields.io/badge/License-Proprietary-red.svg)](#license)

Production-grade firmware and companion 3D digital twin engineering suite for a 6-degree-of-freedom (6-DOF) articulated robotic arm with a coaxial drawing tool, powered by the **ESP32-S3 DevKitC-1** dual-core microcontroller.

---

## 📑 Table of Contents

- [Overview & Architecture](#-overview--architecture)
- [Key Features](#-key-features)
- [Hardware Specifications](#-hardware-specifications)
- [Kinematics & Coordinate Systems](#-kinematics--coordinate-systems)
- [Quick Start Guide](#-quick-start-guide)
- [Digital Twin Simulator](#-digital-twin-simulator)
- [Web Interface & REST API](#-web-interface--rest-api)
- [Repository Structure](#-repository-structure)
- [Hardware Commissioning Workflow](#-hardware-commissioning-workflow)
- [Documentation & Technical References](#-documentation--technical-references)

---

## 🔭 Overview & Architecture

This repository contains the embedded real-time firmware, embedded web studio, and Python 3D simulation twin for high-precision joint control, Cartesian trajectory execution, and 3D inclined plane drawing.

```
                  ┌─────────────────────────────────────────────────────────┐
                  │                 ESP32-S3 Dual-Core SoC                  │
                  ├────────────────────────────┬────────────────────────────┤
                  │     Core 0 (Sensors)       │      Core 1 (Motion)       │
                  │                            │                            │
                  │  ┌──────────────────────┐  │  ┌──────────────────────┐  │
                  │  │ SensorScanTask       │  │  │ ArmMotionTask        │  │
                  │  │ - 200 Hz Rate        │  │  │ - 100 Hz Rate        │  │
                  │  │ - PCA9548A I2C Mux   │  │  │ - FSM & Arbiter      │  │
                  │  │ - 6x AS5600 Encoders │  │  │ - Trajectory Planner │  │
                  │  │ - Real-time Tracking │  │  │ - Closed-Form IK     │  │
                  │  └──────────┬───────────┘  │  └──────────┬───────────┘  │
                  └─────────────┼────────────────────────────┼──────────────┘
                                │                            │ Lock-Free SPSC
                                │ Differential Angle         ▼ (Align 64B)
                                │ Pipeline       ┌──────────────────────┐
                                └───────────────►│ Step Timer ISR       │
                                                 │ - 50 kHz Hardware    │
                                                 │ - Q32.32 DDA Engine  │
                                                 │ - Fail-Fast E-Stop   │
                                                 └──────────┬───────────┘
                                                            ▼
                                                [ 6x Stepper Motors ]
```

### Core Execution Pipeline:
1. **Core 0 — Sensor Task (200 Hz)**: Polls 6× AS5600 12-bit absolute magnetic encoders via a PCA9548A I2C multiplexer. Continuously updates accumulated joint angles and runs the real-time drift protection pipeline.
2. **Core 1 — Motion Task (100 Hz) & Web Server**: Consumes command queue requests (`JOG`, `MOVE_CART`, `DRAW_LINE`, `DRAW_CIRCLE`), solves Craig Modified DH kinematics, executes Gram-Schmidt WorkPlane projections, and generates motion blocks.
3. **Hardware Timer ISR (50 kHz)**: Lock-free Single-Producer Single-Consumer (SPSC) ring buffer with Q32.32 fixed-point DDA pulse generation and sub-20 µs hardware emergency stop response.

---

## ✨ Key Features

- **Craig Modified DH Kinematics**:
  - Forward Kinematics (FK) 4×4 transformation matrix chain.
  - Analytical Closed-Form Inverse Kinematics (IK) for vertical tool posture ($\theta_4 = 0, \theta_6 = 0$).
  - Full workspace boundary, inner deadzone, and singularity protection.
- **3-Point WorkPlane Calibration**:
  - Gram-Schmidt orthonormal coordinate frame $(\vec{u}, \vec{v}, \vec{n})$ calibrated from 3 physical points.
  - Draw 2D trajectories (Lines, Circles, Custom shapes) on tilted tables, easel boards ($35^\circ$), or vertical walls without manual reprogramming.
- **Deterministic 50 kHz Stepper Engine**:
  - Lock-free cacheline-aligned (64B) ring buffer between FreeRTOS tasks and hardware timer ISR.
  - Q32.32 fixed-point numerical integration with zero floating-point overhead in ISR.
  - Target step snapping to eliminate cumulative discretization errors.
- **Dual-Pipeline Closed-Loop Safety**:
  - **Pipeline A (Online 200 Hz)**: Immediate E-Stop trigger if instantaneous joint error $|\Delta\theta| > 3.0^\circ$.
  - **Pipeline B (Post-Stop Settlement 150 ms)**: 4-sample average filter to distinguish real motor stall from sensor noise/gear backlash.
- **Embedded Web Management Studio**:
  - 100% self-contained single-page application served directly from ESP32-S3 PROGMEM (zero external CDN dependencies).
  - Real-time 3D interactive preview canvas computed locally via client-side JavaScript kinematics engine.
  - Full mobile viewport adaptation (`pointer: coarse`, safe area insets) and WCAG AA contrast compliance.
- **Non-Volatile Calibration (NVS)**:
  - Stores WiFi credentials, measured homing direction signs, steps-per-degree ratios, and absolute encoder zero references in the `arm-cfg` partition.

---

## 🔌 Hardware Specifications

### Actuators & Drive Electronics

| Axis | Designation | Actuator | Driver IC | Control Interface | Gear Ratio | Steps / Degree |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **J1** | Base Yaw | NEMA 17 | TMC2209 | UART1 (Addr `0b00`) | **6:1** | 53.33 |
| **J2** | Shoulder Pitch | NEMA 17 | TMC2209 | UART1 (Addr `0b01`) | **20:1** | 177.78 |
| **J3** | Elbow Pitch | NEMA 17 | TMC2209 | UART1 (Addr `0b10`) | **20:1** | 177.78 |
| **J4** | Wrist Pan | NEMA 17 | TMC2209 | UART1 (Addr `0b11`) | **4:1** | 35.56 |
| **J5** | Wrist Tilt | NEMA 14 | A4988 | STEP (GPIO 38) + DIR (GPIO 39) | **3:1** | 26.67 |
| **J6** | Tool Roll | NEMA 14 | A4988 | STEP (GPIO 40) + DIR (GPIO 47) | **3:1** | 26.67 |

> [!NOTE]
> J1–J4 share a single half-duplex UART bus on GPIO 15 (RX) and GPIO 16 (TX). Direction reversal is handled directly via TMC2209 `shaft()` register over UART without dedicated physical DIR pins.

### Sensors & Peripherals

| Component | Function | Interface | Pins / Addresses |
| :--- | :--- | :--- | :--- |
| **PCA9548A** | 8-Channel I2C Multiplexer | I2C (800 kHz Fast-Mode+) | SDA: GPIO 8, SCL: GPIO 9 (Addr `0x70`) |
| **AS5600 ×6** | 12-bit Absolute Magnetic Encoders | I2C via PCA9548A | Sub-buses: Ch 0–2 (J1–J3), Ch 3–5 (J4–J6) |
| **Endstops** | Microswitches (Active LOW) | Internal Pullup + ISR | J1: 5/6, J2: 7/10, J3: 11/12 |

> [!WARNING]
> **Prohibited Pins on ESP32-S3**: Do not reassign strapping pins (GPIO 0, 3, 45, 46), Octal SPI Flash/PSRAM lines (GPIO 26–32), or USB OTG pins (GPIO 19, 20).

---

## 📐 Kinematics & Coordinate Systems

Robot geometry is defined using the **Craig Modified Denavit-Hartenberg (MDH)** convention:
$$^{i-1}T_i = R_x(\alpha_{i-1}) \cdot T_x(a_{i-1}) \cdot R_z(\theta_i) \cdot T_z(d_i)$$

```
  Base (J1) ──[D1=139]──► Shoulder (J2) ──[A2=138]──► Elbow (J3)
                                                        │
                                                 [A3=88, D4=126]
                                                        ▼
  Pen TCP ◄──[D_tool=20]── Tool Roll (J6) ◄── Wrist Center (J4/J5)
```

### Craig Modified DH Parameter Table

| Frame $i$ | Joint | Link Length $a_{i-1}$ | Link Twist $\alpha_{i-1}$ | Link Offset $d_i$ | Theta Offset $\theta_{\text{offset}}$ | Physical Range |
| :---: | :--- | :---: | :---: | :---: | :---: | :---: |
| **1** | J1 (Base Yaw) | $0\,\text{mm}$ | $0^\circ$ | **$139\,\text{mm}$** | $0^\circ$ | $[-90^\circ, +90^\circ]$ |
| **2** | J2 (Shoulder) | $0\,\text{mm}$ | $-90^\circ$ | $0\,\text{mm}$ | **$-90^\circ$** | $[-90^\circ, +90^\circ]$ |
| **3** | J3 (Elbow) | **$138\,\text{mm}$** | $0^\circ$ | $0\,\text{mm}$ | $0^\circ$ | $[0^\circ, +90^\circ]$ |
| **4** | J4 (Wrist Pan) | **$88\,\text{mm}$** | $-90^\circ$ | **$126\,\text{mm}$** | $0^\circ$ | $[-180^\circ, +180^\circ]$ |
| **5** | J5 (Wrist Tilt) | $0\,\text{mm}$ | $+90^\circ$ | $0\,\text{mm}$ | $0^\circ$ | $[-120^\circ, +120^\circ]$ |
| **6** | J6 (Tool Roll) | $0\,\text{mm}$ | $-90^\circ$ | **$31\,\text{mm}$** | $0^\circ$ | $[-360^\circ, +360^\circ]$ |
| **Tool**| Drawing Pen | $0\,\text{mm}$ | $0^\circ$ | **$20\,\text{mm}$** | $0^\circ$ | Fixed Tool Axis |

- **Home Reference Pose $(0^\circ, 0^\circ, 0^\circ, 0^\circ, 0^\circ, 0^\circ)$**:
  - Wrist Center (J5): $(X = 126.0\,\text{mm}, Y = 0.0\,\text{mm}, Z = 365.0\,\text{mm})$
  - J6 Origin: $(X = 157.0\,\text{mm}, Y = 0.0\,\text{mm}, Z = 365.0\,\text{mm})$
  - Tool Pen Tip: $(X = 177.0\,\text{mm}, Y = 0.0\,\text{mm}, Z = 365.0\,\text{mm})$
- **Effective Tool Length (J5 $\to$ TCP)**: $D_{\text{tool\_eff}} = 31\,\text{mm} + 20\,\text{mm} = \mathbf{51\,\text{mm}}$
- **Max Planar Reach (to J5)**: $R_{\text{max}} = A_2 + \sqrt{A_3^2 + D_4^2} = 138.0 + 153.69 = 291.69\,\text{mm}$
- **Inner Deadzone**: $R_{\text{min}} = |A_2 - \sqrt{A_3^2 + D_4^2}| = |138.0 - 153.69| = 15.69\,\text{mm}$

---

## 🚀 Quick Start Guide

### Prerequisites
- [PlatformIO Core (CLI)](https://docs.platformio.org/page/core/index.html) or [PlatformIO IDE for VSCode](https://platformio.org/install/ide?install=vscode)
- Python 3.10+ (for host tests and Digital Twin simulation)
- Git

### 1. Clone & Build Firmware
```bash
# Clone the repository
git clone https://github.com/<your-username>/robotic_arm.git
cd robotic_arm

# Compile firmware
pio run
```

### 2. Flash to ESP32-S3
Connect your ESP32-S3 DevKitC-1 via USB OTG / UART port:
```bash
# Upload firmware image
pio run -t upload

# Open serial telemetry monitor (115200 baud)
pio device monitor
```

### 3. Run Host Kinematics Unit Test Suite
To verify the analytical kinematics engine against thousands of test coordinates:
```bash
# Execute unit test runner via host g++
bash tools/run_kin_tests.sh
```

---

## 💻 Digital Twin Simulator

The project includes a standalone desktop engineering simulator [`digital_clone.py`](file:///E:/00.Project/04.robot-arm/robotic_arm/digital_clone.py) implemented with Matplotlib and NumPy.

```bash
# Launch interactive 3D/2D engineering interface
py -3 digital_clone.py

# Run automated kinematics, WorkPlane, and trajectory sweep audit
py -3 digital_clone.py --test
```

### Simulator Features:
- **3-Viewport Engineering Layout**: 3D Perspective with rotation/zoom, 2D Side Elevation ($X-Z$), and 2D Top Plan ($X-Y$).
- **Click-to-Move Positioning**: Click directly on the 2D Top or Side viewports to command Cartesian moves.
- **WorkPlane Calibrator**: Visualizes 3D tilted surfaces and projects Line, Circle, Spiral, and Star paths onto the calibrated frame.
- **Real-Time WiFi Bridge**:
  - `[📡 Sync ESP32]`: Polls `/api/status` from the physical robot and mirrors its pose in real time.
  - `[⚡ Send to Robot]`: Dispatches target positions directly to `/api/move` on the controller.
- **Velocity & Stepper Rate Profiler**: Verifies joint angular velocities and step pulse frequencies against the 50 kHz hardware timer limit.

---

## 🌐 Web Interface & REST API

Upon powering up, the controller attempts to connect to the provisioned WiFi network (STA mode). If unavailable, it falls back to Access Point (AP) mode:
- **Fallback AP SSID**: `6AXIS-CONTROLLER`
- **Fallback AP Password**: `12345678`
- **mDNS Hostname**: `http://robot-arm.local` (or the IP logged on Serial)

### REST API Reference (Port 80)

| Method | Endpoint | Payload / Query | Description |
| :--- | :--- | :--- | :--- |
| `GET` | `/` | — | Embedded single-page web application |
| `GET` | `/api/status` | — | Full JSON telemetry snapshot (mode, joints, pose, homing, safety) |
| `POST`| `/api/jog` | `{"axis": 0..5, "deg": float}` | Jog single joint angle by relative delta |
| `POST`| `/api/move` | `{"x": float, "y": float, "z": float, "feed": float}` | Command Cartesian linear point move |
| `POST`| `/api/draw` | `{"shape": "line"\|"circle", ...}` | Execute coordinated line or circular drawing trajectory |
| `GET` | `/api/home/all`| — | Initiate automatic sequential homing (J1 $\to$ J4) |
| `GET` | `/api/home/axis`| `?axis=0..3` | Initiate homing on a single designated axis |
| `GET` | `/api/sethome` | `?axis=0..5` | Zero joint at current physical location & save to NVS |
| `GET` | `/api/clearcalib`|`?axis=0..5` | Erase stored joint zero references from NVS |
| `POST`| `/api/workplane/calib`|`{"p1": [...], "p2": [...], "p3": [...]}` | Calibrate 3-point inclined WorkPlane |
| `POST`| `/api/workplane/toggle`|`{"enable": true\|false}` | Enable or disable WorkPlane UCS coordinate transformation |
| `GET` | `/api/stop` | — | **Emergency Stop**: Immediately aborts all motor pulses |
| `POST`| `/api/wifi` | `{"ssid": "...", "pass": "..."}` | Save WiFi credentials and restart controller |

---

## 📁 Repository Structure

```
.
├── platformio.ini              # PlatformIO build configuration & ESP32-S3 environment
├── digital_clone.py            # High-fidelity desktop 3D/2D digital twin simulator
├── src/
│   ├── config.h                # Single source of truth for pins, DH geometry & limits
│   ├── main.cpp                # System initialization, FreeRTOS tasks & web loop
│   ├── arm.h / .cpp            # Motion arbiter, command queue & core 1 task loop
│   ├── motor.h / .cpp          # 50kHz DDA pulse generator, TMC2209 UART & SPSC queue
│   ├── kinematics.h / .cpp     # Craig Modified DH matrix FK & analytical closed-form IK
│   ├── work_plane.h / .cpp     # 3-Point Gram-Schmidt coordinate transformation engine
│   ├── planner.h / .cpp        # Multi-axis segment synchronizer (Line, Circle, Pen lift)
│   ├── sensor.h / .cpp         # AS5600 ×6 over PCA9548A I2C mux (200Hz Core 0 task)
│   ├── homing.h / .cpp         # Multi-phase sequential homing FSM (J1–J4)
│   ├── joint_model.h / .cpp    # Degree-to-step conversion, NVS persistence & soft limits
│   ├── endstop.h / .cpp        # Interrupt service routines (ISR) with hardware debouncing
│   ├── web_server.h / .cpp     # Embedded PROGMEM SPA web UI & REST API handlers
│   ├── spsc_queue.h            # Lock-free 64-byte cacheline aligned SPSC ring buffer
│   ├── motion_block.h          # Q32.32 fixed-point motion segment definition
│   ├── nvs_store.h / .cpp      # ESP-IDF Preferences wrapper for non-volatile storage
│   ├── wifi_manager.h / .cpp   # Automatic STA / AP fallback network supervisor
│   └── rtos_guard.h            # RAII FreeRTOS mutex lock guards
├── test/
│   └── kinematics/             # Analytical kinematics unit tests (host g++)
├── tools/
│   └── run_kin_tests.sh        # Host test runner script
└── docs/
    ├── ARM_GEOMETRY.md         # Mathematical derivation of DH matrices & kinematics
    ├── SYSTEM_OVERVIEW.html    # Interactive system map (hardware, tasks, FSM, safety)
    └── IMPLEMENTATION_LOG.md   # Append-only chronological engineering changelog
```

---

## 🛠 Hardware Commissioning Workflow

When bringing up the physical robotic arm for the first time:

1. **Driver Addressing**: Configure TMC2209 MS1/MS2 jumpers (J1=`0b00`, J2=`0b01`, J3=`0b10`, J4=`0b11`). Confirm `[TMC2209 OK] Ver 0x21` log on serial for each driver.
2. **Current Tuning**: Measure and adjust Vref trimpots on J5/J6 A4988 modules to match motor coil ratings.
3. **Open-Loop Jog Validation**: Jog each joint by small increments ($+1.0^\circ$). Verify direction matches physical convention; flip `AXIS_STEP_SIGN` in [`src/config.h`](file:///E:/00.Project/04.robot-arm/robotic_arm/src/config.h) if inverted.
4. **Encoder Zeroing**: Confirm AS5600 magnet alignment (air gap 1–2 mm, centered over IC).
5. **Homing Calibration**: Execute `/api/home/all`. Verify J1/J2 center calibration and J3 backoff.
6. **Inclined Plane Test**: Calibrate WorkPlane with 3 corner points of a tilted drawing pad, then execute `/api/draw` to confirm accurate surface adherence.

---

## 📚 Documentation & Technical References

- **[System Architecture & Interactive Map](docs/SYSTEM_OVERVIEW.html)**: Complete visual map of FreeRTOS tasks, hardware wiring, and safety interlocks.
- **[Mathematical Kinematics Specification](docs/ARM_GEOMETRY.md)**: Analytical derivation of forward kinematics, closed-form IK equations, and coordinate frames.
- **[Engineering Implementation Log](docs/IMPLEMENTATION_LOG.md)**: Append-only ledger of all design decisions, bug fixes, and hardware test results.
- **[Agent Maintenance Contract](AGENTS.md)**: Rules and invariants for automated maintenance and code modifications.

---

## 📄 License

This repository is maintained for internal research and development. All rights reserved.