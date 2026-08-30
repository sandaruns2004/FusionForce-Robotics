# RUNNER-4 Autonomous Quadruped Robot — 2-Page Proposal Summary
### FusionForce Robotics | EN2533 Robotic Design and Competition | 5 September 2026

---

## Overall Strategy

RUNNER-4 is a 12-DOF autonomous quadruped robot designed to complete four sequential subtasks: grid navigation with ball pickup, curved wall-following with gap handling, obstacle pushing, and colour-based ball sorting. The design prioritises **reliability over speed**, using a slow crawl gait (one leg swinging at a time) that guarantees static stability with three feet always on the ground. A dual-processor architecture separates computationally intensive camera vision (Raspberry Pi 4B) from deterministic real-time motion control (STM32F401CCU6), connected via CRC8-verified UART. A single CSI camera replaces traditional IR sensors, colour sensors, and ball detectors, providing line following, ball detection, colour classification, and junction recognition through OpenCV HSV pipelines. Three VL53L0X ToF sensors handle wall-following and obstacle detection in corridors where camera view is limited.

## Mechanical Design

The robot uses a **4-leg configuration with 3 joints per leg** (coxa/hip yaw, femur/hip pitch, tibia/knee pitch), totalling 12-DOF for locomotion. Proposed link lengths are L_coxa = 25 mm, L_femur = 50 mm, L_tibia = 55 mm. The body uses a **3-tier deck** construction (~160 × 140 mm): battery at bottom (lowest centre of mass), STM32 + PCA9685 at mid-level, and Raspberry Pi at top. A 2-servo **arm + gripper** at the front reaches the 5 cm ball pedestal, grasps the 40 mm ball, and deposits it into an **internal belly compartment** with a **servo-controlled gate** for secure storage and gravity-assisted release. A **passive front bumper plate** provides the pushing surface for Subtask 3, using the quadruped's leg force with no additional actuators. Target standing footprint: ≤ 250 × 250 mm. Estimated total mass: ~550 g. Material: 3D-printed PETG.

## Sensors

| Sensor | Qty | Function |
|--------|-----|----------|
| **CSI Camera** (Pi Camera Module) | 1 | Line following, ball detection, colour classification, junction detection |
| **VL53L0X ToF** | 3 | Wall distance (left/right), obstacle detection (front), corridor centering |
| **MPU6050 IMU** | 1 | Body pitch/roll for stabilization, tilt detection, push monitoring |
| **Start Button** | 1 | Single onboard switch for autonomous start |

The camera is mounted **front-top with −40° downward tilt**, covering the ground 100–250 mm ahead. ToF sensors are at the front-left, front-centre, and front-right corners, oriented perpendicular to walls. The IMU is centre-mounted at the body's centre of mass for cleanest angular measurements.

## Actuators

**15 × MG90S metal-gear micro servos** (2.2 kg·cm @ 6V, 13.4 g each):
- 12 for locomotion (3 per leg × 4 legs)
- 1 for ball arm pitch
- 1 for gripper open/close
- 1 for compartment gate

All driven by a **single PCA9685** 16-channel PWM servo driver (15/16 channels used). Servos operate at 6V from a dedicated **5V–6V / 15A switching BEC**, isolated from the logic power rail. Torque analysis confirms MG90S is sufficient for crawl gait (1.4× safety margin at estimated mass) but insufficient for trot gait — crawl gait is used exclusively.

## Algorithms

| Algorithm | Implementation | Processor |
|-----------|---------------|-----------|
| **3-DOF Inverse Kinematics** | Trigonometric solver (atan2, law of cosines) for 4 legs per 20 ms cycle | STM32 (C) |
| **Crawl Gait Generator** | Bezier-curve foot trajectories; FL→BR→FR→BL diagonal sequence; 2.0s cycle | STM32 (C) |
| **IMU Stabilization** | Complementary filter (α=0.98) + proportional pitch/roll correction to foot heights | STM32 (C) |
| **Line Following** | HSV threshold → contour centroid → PD heading controller (Kp=45, Kd=12) | Pi (Python/OpenCV) |
| **Ball Detection** | HSV colour masks (R/G/B) → contour area + circularity filter → colour classification | Pi (Python/OpenCV) |
| **Wall Following** | Dual-ToF PD centering (error = d_left − d_right) with **gap rejection filter** | STM32 (C) + Pi |
| **Obstacle Pushing** | Front ToF detection → lower body → widen stance → slow crawl push → IMU tilt monitoring | STM32 (C) + Pi |
| **Mission State Machine** | Hierarchical FSM: 6 major states with sub-states, timeouts, and error recovery | Pi (Python) |

**Key algorithmic innovation**: The wall gap rejection filter distinguishes actual wall gaps from noise by requiring 3+ consecutive readings above threshold before switching to single-wall following mode, preventing the robot from steering into gaps.

**Vision approach**: Pure OpenCV (no deep learning). HSV colour space segmentation provides lighting-robust detection. Ball colour stored in memory variable `stored_ball_color` from Subtask 1, recalled in Subtask 4.

## Task Delegation

| Team | Responsibility | Deliverables |
|------|---------------|-------------|
| **Mechanical** | Body + leg CAD; 3D printing; ball mechanism; bumper; assembly | SolidWorks assemblies; printed/assembled robot |
| **Electronics** | Power distribution; BEC/regulator selection; wiring harness; battery monitoring | Tested power system; wiring diagrams |
| **STM32 Firmware** | IK solver; gait engine; IMU processing; ToF reading; UART protocol; servo calibration | C firmware; walking + sensor test results |
| **Pi Software** | Camera pipeline; line/ball/colour/junction detection; HFSM state machine | Python code; tested vision + full mission |
| **Integration** | End-to-end testing; competition prep; 2-min calibration routine | Test reports; competition checklist |

## System Architecture

```
Camera → [Raspberry Pi 4B] ←UART→ [STM32F401] → [PCA9685] → 15× MG90S
              (OpenCV+FSM)          (IK+Gait+IMU)
                                        ↑
                                   MPU6050 + 3×VL53L0X

Power: 2S LiPo → 5V/15A BEC (Servos) + 5V/3A Buck (Pi) + 3.3V LDO (Logic)
```

## Key Design Decisions

| Decision | Choice | Rationale |
|---------|--------|-----------|
| Gait | Crawl only | MG90S insufficient for trot; static stability guaranteed |
| Ball mechanism | 2-servo arm + gripper | Ball on 5cm pedestal requires active reach |
| Vision | Camera + OpenCV (no ML) | Single sensor replaces IR + colour sensors; HSV sufficient |
| Pushing | Passive bumper | Zero additional actuators; high reliability |
| Pi model | Pi 4B (2GB) | 25+ FPS vision; Pi Zero too slow (8–12 FPS) |

**Estimated runtime**: ~24 min (1300mAh) — well within 15-min limit. **Top risk**: MG90S torque margin. **Mitigation**: strict mass control (<550g); backup path to MG996R hybrid.
