# TECHNICAL BLUEPRINT

## 1. System Overview
The FusionForce quadruped robot is an autonomous, 12-DOF legged platform engineered specifically for the IN24 EN2533 Robotic Design Competition (BREACH PROTOCOL). It incorporates a dual-board compute architecture separating high-level perception (Raspberry Pi 4B) from real-time locomotion and control (STM32 Black Pill). It integrates a front-mounted 2-DOF micro-gripper for ball grasping and an integrated flat chassis bumper for block pushing.

## 2. Requirements
Based on `Task/tasks_circuit_v2.md`:
* **Task 01**: Navigate a 4x4 grid (25cm cells, 3cm lines), find a colored ball at an intersection, identify color, grasp the ball, store it, and exit.
* **Task 02**: Navigate a curved corridor (30cm width, 20cm height walls) with missing wall gaps, avoiding early exit or collision.
* **Task 03**: Push a 25x25x20cm obstacle blocking the path out of a straight corridor.
* **Task 04**: Deliver the grasped ball to a matching color-coded sorting segment at a 3-way junction.
* **Physical Constraints**: Max dimensions 250mm x 250mm, single switch start, fully autonomous, active legged movement, no active wheels, max 24V DC.

## 3. Architecture
The robot uses a strict separation of concerns across 3 primary domains:
1. **Domain 1 (High-Level)**: Raspberry Pi 4B (Vision, Planning, Decision Making)
2. **Domain 2 (Low-Level)**: STM32F411 (Motor Control, Kinematics, Sensor reading, PID, Safety)
3. **Domain 3 (Mechanical)**: 12-DOF Quadruped chassis, 2-DOF front gripper, front pushing bumper.

## 4. Hardware Architecture
* **Compute**: Raspberry Pi 4B + STM32 Black Pill
* **Sensors**: Raspberry Pi Camera Module 3, 3x VL53L0X ToF Sensors (front, left, right), MPU6050 IMU, Color Sensor (TCS34725 or Camera).
* **Actuation**: 12x DS3218 leg servos, 2x Micro-servos (gripper), driven by PCA9685 via I2C from STM32.
* **Display**: SSD1306 OLED for debugging.

## 5. Software Architecture
* **Raspberry Pi**: Python-based processes for OpenCV frame processing, high-level path planning, and HFSM state logic.
* **STM32**: Bare-metal C (or FreeRTOS) structured into Core, Drivers, Control, Communication, and Robot state modules. Generates Bezier trot/crawl gaits and calculates IK.

## 6. Communication Architecture
* **Pi ↔ STM32**: Full-duplex UART at 115200 baud.
* **Protocol**: Header `[0xAA, 0x55]`, Msg Type, Payload Length, Payload, CRC8.
* **I2C Bus**: STM32 communicates with PCA9685, ToF sensors, and IMU via hardware I2C, optionally using a TCA9548A multiplexer.

## 7. Control Architecture
* **Locomotion**: STM32 receives velocity/turn targets (`Vx, Vy, Wz`). 
* **State Estimation**: MPU6050 feeds complementary or Mahony filter to determine roll/pitch.
* **Stabilization**: Postural PID controller adjusts IK end-effector targets to compensate for body tilt during locomotion.

## 8. Vision Architecture
* **Camera**: Pi Camera capturing at 30FPS.
* **Pipeline**: Frame → Undistort → ROI extraction → HSV Thresholding → Contour Detection.
* **Tasks**: Line centroid extraction for path following, color contour tracking for ball/sorting, ArUco/Shape detection for gaps.

## 9. Mechanical Architecture
* **Chassis**: 3D-printed quadruped structure.
* **Legs**: 3-DOF per leg (Coxa, Femur, Tibia).
* **Manipulator**: Front-mounted 2-DOF claw for grasping balls without destabilizing the COG.
* **Bumper**: Flat robust frontal plate below gripper for pushing the Task 03 obstacle.

## 10. Power Architecture
* **Source**: 3S LiPo Battery (11.1V).
* **Distribution**:
  - `5V / 10A UBEC` → Servos (PCA9685 power terminal).
  - `5V / 5A Buck Converter` → Raspberry Pi 4B.
  - `3.3V` (from Pi or dedicated LDO) → STM32 + Sensors.
* **Isolation**: Common ground between Pi, STM32, and UBEC is strictly required to prevent ground loops.

## 11. Data Flow
`Camera` → `Pi OpenCV` → `Navigation Vector` → `Pi State Machine` → `UART MOVE CMD` → `STM32` → `Gait Generator` → `Inverse Kinematics` → `PCA9685 (I2C)` → `DS3218 Servos` → `Movement` → `MPU6050 Feedback` → `STM32 Postural PID`.

## 12. State Machine
1. `BOOT_INIT`
2. `SENSOR_CALIB`
3. `IDLE_WAIT_START`
4. `TASK_1_GRID_SEARCH`
5. `TASK_1_BALL_GRAB`
6. `TASK_2_WALL_FOLLOW`
7. `TASK_3_BLOCK_PUSH`
8. `TASK_4_COLOR_SORT`
9. `ERROR_STOP`

## 13. Interfaces
* **Hardware Interface**: Pi (GPIO 14/15) ↔ STM32 (PA9/PA10).
* **Software Interface**: Clean API decoupling gait generation from servo drivers (e.g., `setFootPos(leg, x, y, z)`).

## 14. Safety
* **Emergency Stop**: Software e-stop via Pi; hardware kill switch on the battery line.
* **Failsafes**: Watchdog timer on STM32 to cut PCA9685 PWM if UART connection is lost for >500ms.

## 15. Testing
* **Unit Tests**: Mocked sensor inputs for IK math verification.
* **Subsystem Tests**: Suspension/gait testing on a test stand.
* **Integration Tests**: Pi sending synthetic UART commands to walking robot.

## 16. Deployment
* **STM32**: Flashed via SWD (ST-Link v2).
* **Pi**: Code deployed via SSH. Systemd services configure auto-start of the python control scripts on boot.

## 17. Performance Requirements
* **Vision Latency**: <30ms frame processing time.
* **Control Loop**: IK and gait updated at minimum 50Hz (20ms).
* **UART Bandwidth**: Status packets sent at 10Hz to prevent buffer overflow.

## 18. Risks
* High servo current causing brownouts (Mitigation: high-quality 10A UBEC, large decoupling capacitors).
* Odometry drift in the maze (Mitigation: Rely primarily on visual line-following and ToF wall distance).
* Center of gravity shifts when grasping the ball (Mitigation: Lightweight gripper, dynamic CoG IK shifting).

## 19. Future Improvements
* Implementation of a generic SLAM algorithm if grid layout becomes unpredictable.
* Use of CAN bus for distributed smart servo control instead of PWM/I2C.
