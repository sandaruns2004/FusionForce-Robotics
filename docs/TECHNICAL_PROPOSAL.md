# TECHNICAL PROPOSAL

## Executive Summary
This proposal outlines the engineering design for the RUNNER-4 quadruped robot competing in the IN24 EN2533 BREACH PROTOCOL competition. The solution uses a **single embedded MCU architecture** — an STM32F411CEU6 handles all perception, mission decision-making, and real-time motor control. An 8-channel TCRT5000 IR line array provides line following and intersection detection; a single TCS34725 RGBC colour sensor (mounted on the gripper arm tip) provides ball colour identification and floor zone detection in dual-mode. No Raspberry Pi, no camera, no Linux OS.

## Problem Statement
The competition demands a fully autonomous, legged robot capable of navigating a line-following grid, locating and grasping a coloured ball, traversing curved corridors with missing walls, pushing a sliding block, and depositing the ball into a specific colour-coded zone. This requires robust embedded perception, complex inverse kinematics, and strict size constraints (250×250mm).

## Competition Requirements
1. **Locomotion**: Must use active legs (no wheels).
2. **Autonomy**: Zero external inputs allowed post-start.
3. **Tasks**: Grid search, object retrieval, narrow corridor wall-following, obstacle pushing, and sorting.
4. **Constraints**: Maximum run time 15 minutes, 24V DC max, 250×250mm size limit.

## Proposed Solution
A 12-DOF 3D-printed quadruped platform controlled by a single STM32F411CEU6, augmented with a 2-DOF frontal arm+gripper (with TCS34725 at arm tip) and an integrated chassis bumper for pushing.

## System Architecture
A two-domain hierarchy:
- **Domain 1 (Brain + Spine)**: STM32F411CEU6 — all perception, state machine, IK, gait, safety
- **Domain 2 (Mechanical)**: 12-DOF chassis + arm/gripper + bumper

## Hardware Architecture

| Component | Part | Purpose |
|-----------|------|---------|
| **MCU** | STM32F411CEU6 (Black Pill) | Sole compute node; 100MHz, 512KB Flash |
| **Line Array** | 8× TCRT5000 (digital GPIO) | Line following, intersection/junction detect |
| **Colour Sensor** | TCS34725 RGBC (I2C1) | Ball colour ID (MODE A) + floor zone ID (MODE B) |
| **IMU** | MPU6050 (I2C1) | Pitch/roll stabilization |
| **Distance** | 3× VL53L0X ToF (I2C2) | Wall following, obstacle detection |
| **Servo Driver** | PCA9685 (I2C1) | 15× MG90S PWM generation |
| **Actuators** | 15× MG90S | 12 leg + arm + gripper + gate |

## Embedded Perception Approach
Two dedicated sensors replace the camera+Pi vision pipeline:

**8-Channel TCRT5000 IR Array (GPIO PA0–PA7):**
- Digital HIGH = white/reflective; LOW = black
- Weighted centroid algorithm → steering error (−3.5 to +3.5)
- Intersection detection: ≥6 sensors active for ≥3 consecutive 20ms cycles
- Sub-0.1ms read latency — 200× faster than camera frame processing

**TCS34725 RGBC Colour Sensor (I2C1, arm tip):**
- MODE A (arm 0°): Reads ball colour at pedestal → stores `stored_ball_color` in Flash
- MODE B (arm −70°): Reads floor colour zone at 3-way junction for branch selection
- Colour classification: normalised R/G/B channel ratio algorithm
- Non-blocking 50ms integration; effective 10Hz colour updates

## STM32 Approach
The STM32F411 is the complete system controller. It accepts sensor inputs directly and produces servo commands. It runs an 18-state Mission Hierarchical FSM covering all four subtasks, a PD line follower, IK solver, gait generator, and IMU stabilization — all within a 50Hz (20ms) deterministic loop.

## Locomotion Approach
Crawl gait exclusively (one leg swinging at a time, 3 feet always in stance). Bezier foot trajectory with 2.0s cycle time. MG90S servos at 6V provide adequate torque for crawl + pushing within mass budget.

## Mechanical Design
Low-CoG chassis (battery at bottom). 2-DOF gripper arm carries the TCS34725 sensor and adjusts to two read angles. Passive PETG bumper for obstacle pushing.

## Communication
All I2C internal. No external communication. Debug UART (dev only, disconnected at competition).

## Power
2S LiPo (7.4V). Simplified two-rail distribution:
- 5V/15A BEC → Servos
- STM32 VBUS → 3.3V LDO → Logic and sensors

Estimated runtime: ~35 minutes with 1300mAh (up from 24 min — Pi removed saves 1.5A).

## Safety
- Line lost watchdog (all GPIO LOW for >3s → SAFE_STOP).
- IMU tilt watchdog (>30° → emergency servo stop).
- Low battery ADC monitor.
- I2C auto-reinit on bus hang.
- Flash EEPROM emulation for ball colour survival across resets.

## Testing
1. **Component**: TCS34725 colour accuracy (paper samples); line array centroid verification.
2. **Subsystem**: Single-leg loaded crawl; gait stability; wall following in corridor.
3. **Integration**: Full state machine run through all 4 subtasks.
4. **System**: Full competition circuit simulation ×3 runs.

## Risk Management

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|-----------|
| MG90S torque marginal | High | Critical | Mass <550g; crawl only; 6V; MG996R backup |
| TCS34725 ambient light error | Medium | High | Built-in LED; gain tuning; 2-min prep calibration |
| IR array dirt/contamination | Medium | Medium | Clean before run; digital threshold pots |
| Ball colour memory lost on restart | Low | Critical | STM32 Flash write after Task 1 |

## Development Methodology
Phase-based. Team B (Embedded) handles all firmware including state machine. Team C (Mechanical) handles chassis, arm geometry, and sensor bracket design. Integration phase validates full state machine run.

## Team Responsibilities
- **Team B (Embedded)**: STM32 firmware — IK, gait, state machine, all sensor drivers, Flash persistence
- **Team C (Mechanical)**: CAD, 3D printing, servo tuning, line array bracket, TCS34725 arm mount, bumper

## Timeline
- **Weeks 1–2**: Hardware bring-up, CAD, sensor brackets
- **Weeks 3–4**: Locomotion + line following + colour sensor validation
- **Week 5**: Full state machine + task integration
- **Week 6**: Full circuit testing + optimisation

## Expected Performance
Target completion under 10 minutes. 35+ minute battery runtime. >95% colour classification accuracy with pre-competition calibration.

## Success Criteria
Robot reliably executes all 4 BREACH PROTOCOL subtasks 3 times in a row without human intervention.
