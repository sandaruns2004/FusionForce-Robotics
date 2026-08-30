# SYSTEM OVERVIEW

## 1. High-Level Concept
The FusionForce quadruped is designed around a fundamental principle in robotics: **Separation of Concerns**. 

Instead of forcing a single microcontroller to handle both complex computer vision and millisecond-accurate servo PWM generation, the architecture splits the workload into two distinct layers connected by a robust communication bus.

## 2. The Three Engineering Domains

The system is strictly divided into three domains. This division dictates hardware placement, software architecture, and team responsibilities.

### Domain 1: Raspberry Pi + Computer Vision (The "Brain")
This layer handles all high-level cognitive functions. It does not know how a "leg" works; it only knows where the robot needs to go.
- **Hardware**: Raspberry Pi 4B, Pi Camera.
- **Software**: Linux (Ubuntu/Raspberry Pi OS), Python, OpenCV.
- **Responsibilities**:
  - Image acquisition and perspective transformation.
  - Color filtering and line tracking.
  - Task state machine (knowing if the robot is in Task 01, 02, 03, or 04).
  - High-level decision making (e.g., "Turn left at 0.5 rad/s", "Grasp ball").
  - Transmitting velocity and action commands over UART.

### Domain 2: STM32 Low-Level Control (The "Spine")
This layer handles all hard real-time tasks. It does not know what "Task 02" is; it only knows it has been commanded to move forward at a specific velocity, and it must move the legs to achieve that.
- **Hardware**: STM32F411 (Black Pill), MPU6050 IMU, VL53L0X ToF Sensors.
- **Software**: Bare-metal C, utilizing Hardware Abstraction Layer (HAL).
- **Responsibilities**:
  - Receiving and parsing UART commands from the Pi.
  - Inverse Kinematics (IK) math to determine joint angles.
  - Generating Trot/Crawl gaits.
  - Sending PWM target angles to the PCA9685 via I2C at 50Hz.
  - Reading IMU data and running a postural stabilization PID loop.
  - Reading ToF sensors for wall distance feedback.

### Domain 3: Mechanical & Locomotion (The "Body")
This layer translates the mathematical outputs of Domain 2 into physical motion.
- **Hardware**: 3D-printed chassis, 12x DS3218 servos, 2-DOF Front Gripper, Integrated Bumper.
- **Responsibilities**:
  - Providing a stable physical platform (Center of Gravity optimization).
  - Executing the physical gait without slipping or stalling.
  - Physically grasping the ball and pushing the block.

## 3. High-Level Architecture Diagram

```text
                 ┌─────────────────────────────┐
                 │       DOMAIN 1: BRAIN       │
                 │                             │
                 │ - Raspberry Pi 4B           │
                 │ - OpenCV Line/Color Tracking│
                 │ - Task State Machine        │
                 │ - Decision Generation       │
                 └──────────────┬──────────────┘
                                │
                      UART (115200 baud)
                       Tx ↔ Rx | Rx ↔ Tx
                                │
                 ┌──────────────▼──────────────┐
                 │       DOMAIN 2: SPINE       │
                 │                             │
                 │ - STM32F411 (Black Pill)    │
                 │ - Gait Generator & IK       │
                 │ - IMU Balancing (PID)       │
                 │ - ToF Sensor Reading        │
                 └──────────────┬──────────────┘
                                │
                         I2C (400kHz)
                                │
                 ┌──────────────▼──────────────┐
                 │       DOMAIN 3: BODY        │
                 │                             │
                 │ - PCA9685 Servo Driver      │
                 │ - 12x DS3218 Leg Servos     │
                 │ - 2x Micro-servos (Gripper) │
                 │ - Mechanical Chassis/Bumper │
                 └─────────────────────────────┘
```

## 4. Why This Architecture?
* **Real-Time Determinism**: Linux on the Raspberry Pi is not a real-time OS. If the Pi pauses for 50ms to process a heavy OpenCV frame, a robot directly controlling servos from the Pi would fall over. The STM32 guarantees the gait updates exactly every 20ms.
* **Modularity**: The mechanical team can test walking gaits purely on the STM32 without needing the Raspberry Pi connected. The vision team can test line tracking on the Pi using a monitor without the robot body.
* **Safety**: If the Python script crashes, the STM32 watchdog detects the loss of UART heartbeat and safely halts the robot.
