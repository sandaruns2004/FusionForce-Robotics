# TECHNICAL BLUEPRINT

## 1. System Overview
The RUNNER-4 quadruped robot is a fully autonomous 12-DOF legged platform engineered for the IN24 EN2533 Robotic Design Competition (BREACH PROTOCOL). It uses a **single embedded controller architecture**: one STM32F411CEU6 handles all perception, mission logic, and real-time motor control. Peripheral sensors (8-channel TCRT5000 IR line array, TCS34725 RGBC colour sensor, 3× VL53L0X ToF, MPU6050 IMU) provide all environmental awareness. No Raspberry Pi, no camera, no Linux OS.

## 2. Requirements
Based on `Task/tasks_circuit_v2.md`:
* **Task 01**: Navigate a 4×4 grid (25cm cells, 3cm lines), find a coloured ball at an intersection, identify colour using TCS34725, grasp the ball, store it, and exit.
* **Task 02**: Navigate a curved corridor (30cm width, 20cm walls with gaps) using dual-ToF wall centering with gap rejection filter.
* **Task 03**: Push a 25×25×20cm obstacle out of a straight corridor using the passive front bumper, then execute a left turn.
* **Task 04**: Follow the line to a 3-way colour junction; use TCS34725 in floor-read mode to identify matching colour branch; deliver ball; exit.
* **Physical Constraints**: Max 250mm × 250mm footprint; single switch start; fully autonomous; active legged; max 24V DC.

## 3. Architecture
The robot uses a **two-domain design** with a single compute node:

1. **Domain 1 (Brain + Spine)**: STM32F411CEU6 — all perception, decision-making, IK, gait, safety
2. **Domain 2 (Body)**: 3D-printed 12-DOF quadruped chassis, 2-DOF front arm+gripper, front bumper

## 4. Hardware Architecture
* **Compute**: STM32F411CEU6 Black Pill (sole processor — 100MHz, 512KB Flash, 128KB RAM)
* **Line Sensing**: 8-Channel TCRT5000 digital IR array (GPIO PA0–PA7). Line following, intersection detection, junction detection.
* **Colour Sensing**: TCS34725 RGBC sensor (I2C1, 0x29), mounted on gripper arm tip. Dual mode: ball colour (arm 0°) + floor zone colour (arm −70°).
* **Distance**: 3× VL53L0X ToF sensors (I2C2, XSHUT-addressed to 0x30/0x31/0x32). Wall following, obstacle detection.
* **IMU**: MPU6050 (I2C1, 0x68). 6-axis stabilization, tilt detection.
* **Actuation**: 15× MG90S servos (12 leg + arm + gripper + gate) driven by PCA9685 (I2C1, 0x40).

## 5. Software Architecture
* **STM32F411 Firmware**: Bare-metal C using STM32 HAL. All logic in one codebase:
  - **Drivers**: PCA9685, VL53L0X, MPU6050, TCS34725, LineArray
  - **Control**: IK solver (atan2/law of cosines), Bezier crawl gait, IMU PID stabilization
  - **Navigation**: 18-state Mission HFSM, PD line follower, task planner (ball grab, push, colour sort)

## 6. Communication Architecture
* **No external communication** — fully internal I2C buses.
* **I2C1** (PB6/PB7, 400kHz): PCA9685 (0x40) + MPU6050 (0x68) + TCS34725 (0x29)
* **I2C2** (PB10/PB3, 400kHz): 3× VL53L0X (0x30, 0x31, 0x32)
* **GPIO** (PA0–PA7): 8× TCRT5000 direct digital reads
* **Debug UART** (PA9/PA10, 115200): Development only — physically disconnected at competition

## 7. Control Architecture
* **Locomotion**: STM32 generates `Vx`, `Vy`, `Wz` targets internally from state machine and line follower. No external velocity commands.
* **State Estimation**: MPU6050 feeds complementary filter (α=0.98) to determine roll/pitch.
* **Stabilization**: Postural PID adjusts IK foot Z targets to compensate body tilt.

## 8. Perception Architecture
```
Input                       Processing                    Output
─────                       ──────────                    ──────
8× TCRT5000 GPIO bitmask → Weighted centroid (–3.5/+3.5) → PD Wz command
                          → Active count ≥6, ≥3 cycles   → Intersection flag

TCS34725 RGBC (I2C1)     → Normalised R/G/B ratios       → ColorID_t enum
  ARM MODE A (0°)        → Ball colour at pedestal        → stored_ball_color
  ARM MODE B (–70°)      → Floor zone at junction         → branch selection

VL53L0X × 3 (I2C2)       → Median filtered distances     → wall PD error, obstacle flag

MPU6050 (I2C1)            → Complementary filter          → pitch_deg, roll_deg
```

## 9. Mechanical Architecture
* **Chassis**: 3D-printed PETG multi-deck body (~160×140mm base).
* **Legs**: 3-DOF per leg (Coxa ±45°, Femur ±60°, Tibia 0°–135°). Link lengths: L_coxa=25mm, L_femur=50mm, L_tibia=55mm.
* **Gripper Arm**: Front-mounted 2-servo mechanism (arm pitch + claw). TCS34725 mounted at arm tip.
* **Line Array Bracket**: Front-underside mount, 5–8mm above floor.
* **Bumper**: Passive PETG plate (ground to 50mm height) for Task 03 obstacle pushing.

## 10. Power Architecture
* **Source**: 2S LiPo (7.4V nominal, 8.4V max, 1300–2200mAh, 25C+).
* **Distribution**:
  - `5V / 15A BEC` → Servos (PCA9685 power terminal)
  - `STM32 VBUS → 3.3V LDO` → STM32 + MPU6050 + ToF + TCS34725 + Line Array
* **No separate Pi power rail** — Pi removed; power system simplified.

## 11. Data Flow
```
TCRT5000 (GPIO) → Line centroid error → PD Controller → Wz
                → Intersection flag  → State Machine  → State transitions
TCS34725 (I2C1) → RGBC → colour classify → stored_ball_color / branch select
VL53L0X (I2C2)  → distances → wall PD / obstacle detect → Vx targets
MPU6050 (I2C1)  → pitch/roll → postural PID → foot Z offsets
                                                ↓
State Machine → [Vx, Vy, Wz, arm_action] → Gait Generator → IK → PCA9685 → Servos
```

## 12. State Machine
1. `BOOT_INIT`
2. `SENSOR_CALIB`
3. `IDLE_WAIT_START`
4. `TASK1_LINE_FOLLOW`
5. `TASK1_BALL_APPROACH`
6. `TASK1_COLOR_ID` (TCS34725 MODE A)
7. `TASK1_BALL_GRAB`
8. `TASK1_STORE`
9. `TASK1_GRID_EXIT`
10. `TASK2_WALL_FOLLOW`
11. `TASK3_WALL_FOLLOW`
12. `TASK3_OBSTACLE_DETECT`
13. `TASK3_PUSH`
14. `TASK3_TURN`
15. `TASK4_LINE_FOLLOW`
16. `TASK4_JUNCTION_DETECT` (TCS34725 MODE B)
17. `TASK4_BRANCH_FOLLOW`
18. `TASK4_BALL_RELEASE`
19. `FINISH`
20. `SAFE_STOP` / `ERROR_RECOVERY`

## 13. Interfaces
* **Hardware Interface**: All internal. I2C1, I2C2, GPIO — no external connectors during competition.
* **Software Interface**: Clean API — `setFootPos(leg, x, y, z)`, `StateMachine_Update()`, `LineArray_GetCentroid()`, `TCS34725_ClassifyColor()`.

## 14. Safety
* **Tilt Stop**: IMU >30° → emergency servo stop.
* **Line Lost**: All 8 GPIO LOW for >3s → SAFE_STOP.
* **Low Battery**: ADC <6.4V → halt.
* **I2C Watchdog**: Auto-reinit on I2C bus hang.
* **Flash Persistence**: Ball colour written to Flash after Task 1 → survives competition restart.

## 15. Testing
* **Unit Tests**: `TCS34725_ClassifyColor()` with paper samples; `LineArray_GetCentroid()` with known bitmasks; IK math with known (x,y,z) → angle tables.
* **Subsystem Tests**: Single-leg loaded crawl; gait on stand; wall following in corridor.
* **Integration Tests**: Full state machine run through all 4 subtasks.

## 16. Deployment
* **STM32**: Flashed via SWD (ST-Link v2). No SD card, no SSH, no Linux.
* **Pre-competition**: 2-minute calibration window — adjust line sensor thresholds and TCS34725 gain for arena lighting.

## 17. Performance Requirements
* **Line Array Scan**: <0.1ms per cycle (GPIO read).
* **Colour Reading**: 50ms integration (TCS34725); non-blocking; effective 10Hz updates.
* **Control Loop**: IK and gait updated at 50Hz (20ms). Loop budget: <18ms with all sensors.
* **State Machine**: <1ms per update cycle.

## 18. Risks
* **MG90S torque margin** (High, Critical): Keep mass <550g; crawl gait only; 6V servo supply. Backup: MG996R hybrid.
* **TCS34725 ambient light interference** (Medium, High): Built-in LED + controlled integration time. Tune in 2-min prep.
* **STM32 loop timing overflow** (Low, High): Profile with DWT cycle counter; TCS34725 reads are non-blocking.
* **Ball colour memory on restart** (Low, Critical): Flash EEPROM emulation writes colour after Task 1.

## 19. Future Improvements
* Add second TCS34725 at underside for simultaneous ball and floor detection.
* Implement SLAM-lite using ToF + IMU dead reckoning for grid position tracking.
* Upgrade leg servos to DS3218 for higher torque margin.
