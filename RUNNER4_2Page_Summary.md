# RUNNER-4 Autonomous Quadruped Robot — 2-Page Proposal Summary
### FusionForce Robotics | EN2533 Robotic Design and Competition | 5 September 2026

---

## Overall Strategy

RUNNER-4 is a 12-DOF autonomous quadruped robot designed to complete four sequential subtasks: grid navigation with ball pickup, curved wall-following with gap handling, obstacle pushing, and colour-based ball sorting. The design prioritises **reliability over speed**, using a slow crawl gait (one leg swinging at a time) that guarantees static stability with three feet always on the ground. A **single-MCU architecture** places all perception, mission decision-making, and real-time motor control on one STM32F411CEU6. An 8-channel TCRT5000 IR line array handles line following and intersection detection; a TCS34725 RGBC colour sensor (mounted on the gripper arm tip) handles ball colour identification and floor zone detection in dual-mode. Three VL53L0X ToF sensors handle wall-following and obstacle detection in corridors. The robot is powered by a **3S LiPo (11.1V, 2200mAh, 60C)** for high-current headroom and extended runtime.

## Mechanical Design

The robot uses a **4-leg configuration with 3 joints per leg** (coxa/hip yaw, femur/hip pitch, tibia/knee pitch), totalling 12-DOF for locomotion. Proposed link lengths are L_coxa = 25 mm, L_femur = 50 mm, L_tibia = 55 mm. The body uses a **3-tier deck** construction (~160 × 140 mm): battery at bottom (lowest centre of mass), STM32 + PCA9685 at mid-level, and Raspberry Pi at top. A 2-servo **arm + gripper** at the front reaches the 5 cm ball pedestal, grasps the 40 mm ball, and deposits it into an **internal belly compartment** with a **servo-controlled gate** for secure storage and gravity-assisted release. A **passive front bumper plate** provides the pushing surface for Subtask 3, using the quadruped's leg force with no additional actuators. Target standing footprint: ≤ 250 × 250 mm. Estimated total mass: ~550 g. Material: 3D-printed PETG.

## Sensors

| Sensor | Qty | Function |
|--------|-----|----------|
| **8-ch TCRT5000 IR array** (GPIO PA0–PA7) | 1 | Line following, intersection/junction detection via weighted centroid |
| **TCS34725 RGBC** (I2C1, arm tip) | 1 | MODE A (arm 0°): ball colour; MODE B (arm −70°): floor zone colour |
| **VL53L0X ToF** | 3 | Wall distance (left/right), obstacle detection (front), corridor centering |
| **MPU6050 IMU** | 1 | Body pitch/roll for stabilization, tilt detection, push monitoring |
| **Start Button** | 1 | Single onboard switch for autonomous start |

The **line array** is mounted front-underside at 5–8mm floor height, centred on the robot midline. The **TCS34725** is at the gripper arm tip with a 3D-printed light shroud. ToF sensors are at the front-left, front-centre, and front-right corners. The IMU is centre-mounted at the body's centre of mass.

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
| **Line Following (PD)** | 8-sensor weighted centroid error → PD controller → Wz steering command | STM32 (C) |
| **Colour Classification** | TCS34725 R/G/B normalised ratios; non-blocking 50ms integration | STM32 (C) |
| **Wall Following** | Dual-ToF PD centering (error = d_left − d_right) with **gap rejection filter** | STM32 (C) |
| **Obstacle Pushing** | Front ToF detection → lower body → widen stance → slow crawl push → IMU tilt monitoring | STM32 (C) |
| **Mission State Machine** | 18-state HFSM (all 4 subtasks); Flash-persistent ball colour; 50Hz loop | STM32 (C) |

**Key algorithmic innovation**: The wall gap rejection filter distinguishes actual wall gaps from noise by requiring 3+ consecutive readings above threshold before switching to single-wall following mode, preventing the robot from steering into gaps.

**Vision approach**: Pure OpenCV (no deep learning). HSV colour space segmentation provides lighting-robust detection. Ball colour stored in memory variable `stored_ball_color` from Subtask 1, recalled in Subtask 4.

## Task Delegation

| Team | Responsibility | Deliverables |
|------|---------------|-------------|
| **Mechanical** | Body + leg CAD; 3D printing; arm with TCS34725 tip mount; line array bracket; bumper | SolidWorks assemblies; printed/assembled robot |
| **Electronics** | Power distribution; BEC selection (3S input); wiring harness; battery monitoring ADC | Tested power system; wiring diagrams |
| **STM32 Firmware** | IK solver; gait engine; IMU; all sensor drivers (TCRT5000+TCS34725+VL53L0X); state machine | C firmware; walking + full sensor test results |
| **Integration** | End-to-end testing; sensor calibration; competition prep; 2-min calibration routine | Test reports; competition checklist |

## System Architecture

```
8×TCRT5000 (PA0–PA7) ──────────────────────────────────────┐
TCS34725 (I2C1, arm tip) ──┐                               │
MPU6050  (I2C1)  ────────┤→ [STM32F411CEU6] → [PCA9685] → 15×MG90S
3×VL53L0X (I2C2) ────────┘   (Perception+FSM+IK+Gait+50Hz)

Power: 3S LiPo (11.1V, 2200mAh, 60C) → 5V/15A BEC (Servos) + 5V/1A Buck (STM32) + 3.3V LDO (Logic)
```

## Key Design Decisions

| Decision | Choice | Rationale |
|---------|--------|-----------|
| Gait | Crawl only | MG90S insufficient for trot; static stability guaranteed |
| Ball mechanism | 2-servo arm + gripper | Ball on 5cm pedestal requires active reach |
| Perception | TCRT5000 array + TCS34725 | Sub-ms line reads; dual-mode colour sensor replaces camera+Pi |
| Pushing | Passive bumper | Zero additional actuators; high reliability |
| **Battery** | **3S LiPo 2200mAh 60C** | 11.1V; 132A burst capable; ~60 min runtime; no brownout risk |

**Estimated runtime**: ~60 min (3S 2200mAh @ 11.1V) — **4× the 15-min competition limit**. **Top risk**: MG90S torque margin. **Mitigation**: strict mass control; backup path to MG996R hybrid.
