# Hexapod Spider Robot — Project Roadmap

**Assumptions:** 6 legs ("spider" configuration), 3 servos/leg (coxa–femur–tibia, 3DOF) = 18 servos total, STM32 board handles low-level servo/PWM and sensor timing, Raspberry Pi handles high-level control, camera/vision, and Wi-Fi/remote control. Adjust leg count if you meant something else (e.g. 4 or 8 legs).

---

## 1. System Architecture

```
[Camera] --> [Raspberry Pi] <--UART/I2C/SPI--> [STM32] --PWM--> [18x Servos]
                  |                                 |
              Wi-Fi/App                     IMU / current sensors / power mgmt
```

- **Raspberry Pi**: vision processing, path planning, gait selection, remote control interface, logging.
- **STM32**: real-time servo PWM generation, IK solving per leg (optional), reads IMU/foot sensors, sends state back to Pi.
- **Division of labor rationale**: Pi is not a hard real-time OS (runs Linux), so precise servo timing and closed-loop leg control belong on the STM32. The Pi should only send high-level commands ("leg 3 target angle set" or "walk forward at speed X") over UART/SPI/I2C.

---

## 2. Bill of Materials (core)

| Component | Qty | Notes |
|---|---|---|
| Micro servos (e.g. MG996R / SG90 depending on load) | 18 | 3 per leg x 6 legs |
| PCA9685 16-channel PWM driver (if STM32 lacks enough PWM pins) | 2 | Offloads PWM generation |
| STM32 dev board (Blue Pill / Nucleo / STM32F4) | 1 | Low-level real-time control |
| Raspberry Pi (4B or 5 recommended) | 1 | Vision + high-level control |
| Camera module (Pi Camera v3 or USB webcam) | 1 | Vision/navigation |
| IMU (MPU6050 or BNO055) | 1 | Balance/orientation feedback |
| BEC / voltage regulator (5V/6V high-current) | 1–2 | Servos draw significant current — do NOT power from Pi's 5V rail |
| LiPo battery (2S/3S, capacity sized to servo draw) | 1 | Main power |
| Chassis/leg frame (3D printed or laser-cut) | — | Spider/hexapod leg geometry |
| Logic-level converter (if 3.3V STM32 to 5V servo signals needed) | as needed | |
| Wiring, connectors, fuse/switch | — | |

---

## 3. Phase-by-Phase Roadmap

### Phase 0 — Planning & Design (1–2 weeks)
- Decide final leg geometry (coxa/femur/tibia link lengths) and gait type (tripod gait is standard starting point for 6-leg walkers).
- Choose servo torque based on estimated per-leg load (weight ÷ 3 legs in stance × safety factor).
- Sketch electrical block diagram (power domains: logic vs servo power, common ground).
- Decide Pi ↔ STM32 communication protocol (UART is simplest to start).

### Phase 1 — Mechanical Build
- 3D print or fabricate leg segments and body chassis.
- Assemble one leg first as a prototype; verify range of motion and mechanical clearance before mass-producing 6 legs.
- Mount servos, check for binding at extreme angles.

### Phase 2 — Electrical & Power
- Wire servo power bus separately from logic power (common ground between STM32/Pi/servo driver is mandatory).
- Add fusing/switch and confirm battery can supply peak current for all 18 servos moving simultaneously.
- Bench-test each servo individually with a simple PWM signal before full integration.

### Phase 3 — STM32 Firmware (low-level control)
- Set up PWM generation for 18 channels (via onboard timers and/or PCA9685 over I2C).
- Write single-leg inverse kinematics (IK): given target foot (x, y, z), compute 3 joint angles.
- Implement a servo calibration table (each servo has slightly different zero-offset).
- Build a simple command interface over UART (e.g. "set leg N to position X,Y,Z" or "set joint angle directly").
- Add IMU reading loop and basic filtering (complementary or Kalman) for tilt data.

### Phase 4 — Gait Engine
- Implement tripod gait (legs 1,3,5 move together, then 2,4,6) as the first walking pattern.
- Parametrize step height, stride length, and cycle speed.
- Test static stability first (robot standing, shifting weight) before dynamic walking.
- Add turning (differential stride on left/right leg groups) and strafing if desired.

### Phase 5 — Raspberry Pi High-Level Control
- Set up UART/SPI/I2C link Pi ↔ STM32; define and document the command protocol.
- Build a control loop on the Pi that sends gait commands and reads back IMU/status.
- Add a manual control interface first (keyboard, gamepad, or simple web UI) to validate walking before adding autonomy.

### Phase 6 — Camera & Vision Integration
- Get the camera streaming (e.g. via `libcamera`/OpenCV) and verify frame rate is sufficient for your use case.
- Start with a simple vision task: obstacle detection via depth cues, color blob tracking, or an object detector (e.g. lightweight YOLO/TFLite model) depending on goals.
- Fuse vision output into navigation decisions (e.g. "obstacle ahead → turn").

### Phase 7 — Integration & Autonomy
- Combine gait engine + IMU balance feedback + vision-based navigation into one control loop.
- Add safety behaviors: low-battery stop, servo overcurrent/stall detection, fall detection via IMU.
- Tune gait parameters for your specific chassis weight and servo torque.

### Phase 8 — Testing & Refinement
- Test on varied terrain/surfaces.
- Log servo currents/temperatures to catch overheating under sustained walking.
- Iterate on stride timing and IK calibration based on real-world drift.

### Phase 9 — Polish (optional)
- Add remote monitoring (video stream + telemetry dashboard).
- Add voice or app-based control.
- Improve autonomy (SLAM-lite, path planning).

---

## 4. Suggested Tooling

- **STM32 firmware**: STM32CubeIDE or PlatformIO, HAL or libopencm3.
- **Raspberry Pi software**: Python (OpenCV, pyserial for UART, Flask/FastAPI for a control dashboard) or C++ if performance-critical.
- **IK/gait math**: prototype in Python/MATLAB first, then port core loop to STM32 in C.
- **Version control**: keep firmware and Pi-side code in separate repos or clearly separated folders since they build/flash differently.

---

## 5. Key Risks to Watch

- **Power sag**: 18 servos moving at once can brown out a Pi if grounds/power aren't properly isolated.
- **Servo jitter**: cheap PWM drivers or noisy signal lines cause twitching — use shielded wiring and a dedicated PWM driver (PCA9685) rather than bit-banging from the Pi.
- **Real-time mismatch**: don't try to do precise leg timing from Python on the Pi — that belongs on the STM32.
- **Mechanical slop**: 3D printed joints with play will make IK calibration frustrating; tighten tolerances before software debugging.

---

*Adjust leg count, servo count, and phase order to match your actual design once finalized — this roadmap assumes a standard 6-leg/3-servo-per-leg hexapod as the most common interpretation of "spider legs."*
