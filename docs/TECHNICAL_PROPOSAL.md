# TECHNICAL PROPOSAL

## Executive Summary
This proposal outlines the engineering design for the FusionForce quadruped robot, engineered to compete in the IN24 EN2533 BREACH PROTOCOL competition. The solution utilizes a distributed dual-compute architecture, separating high-level Computer Vision and decision-making (Raspberry Pi 4B) from hard real-time locomotion and control (STM32F411). 

## Problem Statement
The competition demands a fully autonomous, legged robot capable of navigating a line-following grid, locating and grasping a colored ball, traversing curved corridors with missing walls, pushing a sliding block, and depositing the ball into a specific color-coded zone. This requires robust perception, complex inverse kinematics, and strict size constraints (250x250mm).

## Competition Requirements
1. **Locomotion**: Must use active legs (no wheels).
2. **Autonomy**: Zero external inputs allowed post-start.
3. **Tasks**: Grid search, object retrieval, narrow corridor wall-following, obstacle pushing, and sorting.
4. **Constraints**: Maximum run time 15 minutes, 24V DC max, 250x250mm size limit.

## Proposed Solution
A 12-DOF 3D-printed quadruped platform augmented with a 2-DOF frontal micro-gripper for object manipulation and an integrated chassis bumper for pushing. 

## System Architecture
A three-domain hierarchy ensures stable separation of concerns:
- **High-Level**: Raspberry Pi + Pi Camera (Perception and Task Planning)
- **Low-Level**: STM32F411 (Motor Control, IK, Feedback)
- **Mechanical**: 12-DOF chassis + Gripper

## Hardware Architecture
- **MCU**: STM32F411CEU6 (Black Pill)
- **SBC**: Raspberry Pi 4B (4GB/8GB)
- **Sensors**: Camera Module 3, 3x VL53L0X ToF, MPU6050
- **Actuators**: 12x DS3218 (legs), 2x SG90 (gripper)
- **Drivers**: PCA9685 16-channel PWM

## Software Architecture
- **Pi**: Python 3, OpenCV, multi-threaded state machine.
- **STM32**: Bare-metal C, utilizing HAL, hardware timers, and interrupts for zero-latency control.

## Computer Vision Approach
Using OpenCV, frames will undergo perspective transformation. Line following will rely on contour extraction and centroid calculation. Ball detection will use HSV color filtering (Red, Blue, Green) masked with Hough Circle Transforms for shape validation. 

## STM32 Approach
The STM32 acts as a real-time coprocessor. It accepts `Target Velocity` and `Target Heading` commands over UART. It processes a continuous gait generator (e.g. Trotting gait), calculates the Inverse Kinematics for all 12 joints, and updates the PCA9685 via I2C at 50Hz.

## Locomotion Approach
A static crawl gait (moving one leg at a time) ensures maximum stability during the grid search and object retrieval phases. A dynamic trot gait (diagonal pairs) can be engaged for faster traversal during straight corridors if stability margins permit.

## Mechanical Design
The chassis will be structurally optimized for a low center of gravity. The 2-DOF gripper is front-mounted independently of the legs to ensure the gait kinematics are not disturbed during grasping.

## Communication
A custom UART protocol at 115200 baud will bridge the Pi and STM32, utilizing explicit framing, sequence numbers, and CRC8 for robust error checking.

## Power
A 3S LiPo battery provides 11.1V. A 10A UBEC steps this down to 5V exclusively for the servos. A separate, isolated 5V 5A switching regulator powers the Raspberry Pi to prevent servo-induced voltage dips from crashing the compute layer.

## Safety
- Software Heartbeat (500ms timeout stops all motors).
- Physical battery disconnect switch.
- Current-limiting fuses on the servo rail.

## Testing
1. **Component**: Verify each sensor individually.
2. **Subsystem**: Test the quadruped gait on a suspension rig.
3. **Integration**: Verify Pi-STM32 UART packet integrity.
4. **System**: Full competition circuit simulation.

## Risk Management
| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|------------|
| Servo Overload / Brownout | High | High | Use 10A UBEC, monitor voltage |
| Odometry Drift | Medium | Medium | Use ToF sensors for wall distance feedback |
| OpenCV false positives | Medium | High | Rigorous HSV calibration and shape constraints |

## Development Methodology
Agile, phase-based development. Teams will independently develop the Vision, Embedded, and Mechanical systems against rigid interface contracts, followed by a dedicated Integration phase.

## Team Responsibilities
- **Team A (Vision & Pi)**: OpenCV, state machine, planning.
- **Team B (Embedded)**: STM32, IK, sensors, PWM.
- **Team C (Mechanical)**: CAD, printing, assembly, servo tuning.

## Timeline
- **Weeks 1-2**: Hardware Bring-Up & CAD.
- **Weeks 3-4**: Locomotion & Vision Development.
- **Week 5**: Pi-STM32 Integration.
- **Week 6**: Full Task Circuit Testing & Optimization.

## Expected Performance
The robot is expected to complete the circuit within 6-8 minutes, well under the 15-minute limit, maintaining a 95%+ success rate in ball grasping due to the dedicated gripper design.

## Success Criteria
The robot reliably executes the BREACH PROTOCOL circuit 3 times in a row without human intervention, maintaining stability and accurately identifying color zones.
