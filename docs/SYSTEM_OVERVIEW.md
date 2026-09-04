# SYSTEM OVERVIEW

## 1. High-Level Concept
The FusionForce RUNNER-4 quadruped is designed around a fundamental principle in embedded robotics: **Simplicity through Integration**.

Instead of splitting workload across two processors (a Raspberry Pi for vision + an STM32 for control), the architecture consolidates all perception, decision-making, and motor control onto a **single STM32F411CEU6 microcontroller**. This eliminates inter-CPU communication overhead, Linux boot delays, UART failure points, and weight penalties — all critical advantages in a time-limited, weight-constrained competition.

## 2. The Two Engineering Domains

The system is divided into two domains. This division dictates hardware placement, software architecture, and team responsibilities.

### Domain 1: STM32 + Embedded Sensing (The "Brain + Spine")
This single layer handles all cognitive functions AND all real-time control. It knows where to go and exactly how to move the legs to get there.
- **Hardware**: STM32F411CEU6 (Black Pill), 8-Channel TCRT5000 IR Line Array, TCS34725 RGBC Colour Sensor, MPU6050 IMU, 3× VL53L0X ToF Sensors.
- **Software**: Bare-metal C using STM32 HAL. 50Hz deterministic control loop.
- **Responsibilities**:
  - Reading 8-channel IR line array for line following and intersection detection.
  - Reading TCS34725 colour sensor for ball colour identification (Task 01) and floor zone detection (Task 04).
  - Running the Mission Hierarchical Finite State Machine (HFSM) — all 4 subtasks.
  - Inverse Kinematics (IK) math to determine joint angles.
  - Generating crawl gaits via Bezier trajectories.
  - Sending PWM target angles to the PCA9685 via I2C at 50Hz.
  - Reading IMU and running postural stabilization PID loop.
  - Reading ToF sensors for wall-distance feedback.
  - Persisting `stored_ball_color` in Flash across power cycles/restarts.

### Domain 2: Mechanical & Locomotion (The "Body")
This layer translates the mathematical outputs of Domain 1 into physical motion.
- **Hardware**: 3D-printed PETG chassis, 12× MG90S leg servos, 1× MG90S arm pitch, 1× MG90S gripper, 1× MG90S storage gate, passive front bumper.
- **Responsibilities**:
  - Providing a stable physical platform (Centre of Gravity optimisation).
  - Executing the physical crawl gait without slipping or stalling.
  - Physically grasping the ball and pushing the block.

## 3. High-Level Architecture Diagram

```text
         ┌──────────────────────────────────────────────────────┐
         │             DOMAIN 1: BRAIN + SPINE                  │
         │                                                      │
         │  ┌─ PERCEPTION ──────────────────────────────────┐   │
         │  │ 8× TCRT5000 IR Line Array (GPIO PA0–PA7)      │   │
         │  │  → Line centroid, Intersection/Junction detect │   │
         │  │ TCS34725 Colour Sensor (I2C1, 0x29, arm tip)  │   │
         │  │  → Ball colour ID (arm 0°) + Floor zone (−70°)│   │
         │  │ MPU6050 IMU (I2C1, 0x68)                      │   │
         │  │  → Pitch/roll for body stabilisation          │   │
         │  │ 3× VL53L0X ToF (I2C2, XSHUT-addressed)       │   │
         │  │  → Wall distances, obstacle detection         │   │
         │  └───────────────────────────────────────────────┘   │
         │                       │                              │
         │  ┌─ CONTROL ──────────▼──────────────────────────┐   │
         │  │ STM32F411CEU6 (100MHz, 512KB Flash, 128KB RAM) │   │
         │  │  Mission State Machine (18 states, all tasks)  │   │
         │  │  Gait Generator (crawl, Bezier trajectories)   │   │
         │  │  Inverse Kinematics (3-DOF per leg, 4 legs)    │   │
         │  │  Postural PID (IMU feedback → foot corrections) │   │
         │  └────────────────────┬──────────────────────────┘   │
         └───────────────────────┼──────────────────────────────┘
                                 │
                         I2C1 (400kHz)
                                 │
         ┌───────────────────────▼──────────────────────────────┐
         │                 DOMAIN 2: BODY                       │
         │                                                      │
         │  PCA9685 (I2C1, 0x40) — 16-ch PWM servo driver      │
         │  12× MG90S Leg Servos  (CH0–CH11)                    │
         │  1×  MG90S Arm Pitch   (CH12)                        │
         │  1×  MG90S Gripper     (CH13)                        │
         │  1×  MG90S Storage Gate (CH14)                       │
         │  Passive Front Bumper — Task 03 obstacle pushing     │
         └──────────────────────────────────────────────────────┘
```

## 4. Why This Architecture?

* **Real-Time Determinism**: The STM32 bare-metal C loop runs exactly every 20ms (50Hz), with no OS preemption. Every sensor read and every servo update is predictable and bounded.
* **Instant Boot**: The STM32 initialises in <500ms. No 30-second Linux boot, no SD card dependency, no filesystem corruption risk.
* **No Inter-CPU Failure Point**: The original Pi→STM32 UART link was a single point of failure. A single corrupted packet or Pi crash could halt the robot. With one CPU, this failure mode is eliminated entirely.
* **Weight & Power**: Removing the Raspberry Pi 4B (46g, 1.5A) and its power regulator saves **~73g** and **~1.5A** average current draw — extending estimated runtime from 24 minutes to 35+ minutes.
* **Deterministic Colour Detection**: The TCS34725 is a dedicated RGBC sensor — its readings are unaffected by frame rate, motion blur, or image compression. It produces stable, low-latency colour values regardless of robot vibration.
* **Simplicity**: Fewer boards, fewer cables, fewer failure modes. One flash target, one debugging interface, one codebase in one language (C).
