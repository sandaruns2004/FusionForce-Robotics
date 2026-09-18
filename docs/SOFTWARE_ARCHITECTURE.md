# SOFTWARE ARCHITECTURE

## 1. Single-Domain Design
The entire software system resides on a **single STM32F411CEU6** running bare-metal C (HAL). There is no Raspberry Pi, no Linux OS, no Python, and no inter-CPU communication. All perception, mission logic, and motor control operate within one deterministic 50Hz control loop.

## 2. Firmware (STM32F411) — Complete System

### Directory Structure
```text
firmware/
├── Core/
│   ├── Src/main.c               # Entry point; init all peripherals; main while(1) loop
│   ├── Src/stm32f4xx_it.c       # Interrupt handlers (TIM2 50Hz, I2C, EXTI)
│   └── Src/system_stm32f4xx.c   # Clock config (100MHz via PLL)
├── Drivers/
│   ├── PCA9685/
│   │   ├── pca9685.h
│   │   └── pca9685.c            # I2C PWM driver: init, set channel, angle→tick
│   ├── VL53L0X/
│   │   ├── vl53l0x.h
│   │   └── vl53l0x.c            # ToF driver: XSHUT remap, range read, median filter
│   ├── MPU6050/
│   │   ├── mpu6050.h
│   │   └── mpu6050.c            # IMU driver: raw read, complementary filter
│   ├── TCS34725/                 ← Colour sensor driver
│   │   ├── tcs34725.h
│   │   └── tcs34725.c           # I2C RGBC read, integration time, colour classify
│   └── LineArray/                ← 9-channel ADC+DMA IR line sensor driver
│       ├── line_array.h
│       └── line_array.c         # ADC1 DMA circular, EMA filter, centroid, junction
├── Motion/                       ← Replaces old Control/ folder
│   ├── kinematics.h / kinematics.c    # 3-DOF geometric IK (L1=30 L2=60 L3=80mm)
│   ├── gait_engine.h / gait_engine.c  # Trot gait: diagonal FL+BR/FR+BL, T=600ms
│   └── arm_controller.h / arm_controller.c  # CH12 arm + CH13 grip + CH14 gate slew
└── Navigation/                   ← NEW: Replaces entire Raspberry Pi Python layer
    ├── state_machine.h / state_machine.c  # Mission HFSM (18 states, all 4 subtasks)
    ├── line_follower.h / line_follower.c  # PD controller from 8-sensor centroid
    └── task_planner.h / task_planner.c    # Arm sequencing, ball grab, push, release
```

### 50Hz Process Flow (STM32 — Full System)
`TIM2` interrupt fires every 20ms, setting `update_flag = 1`. The main `while(1)` loop executes:

```c
if (update_flag) {
    update_flag = 0;

    /* ─── PERCEPTION ────────────────────────────────────────────────── */
    // 1. Read 9-channel line array (ADC1 DMA circular — always running)
    LA_Update(&la_result);      // EMA filter, centroid, junction detect
    float   line_error   = la_result.error;       // -4.0 to +4.0
    bool    intersection = la_result.is_junction; // >=6/9 sensors, >=3 cycles
    bool    line_lost    = la_result.is_lost;

    // 2. Read IMU (I2C1 — ~1ms)
    MPU6050_Read(&pitch, &roll);

    // 3. Read 3× VL53L0X ToF (I2C2 — ~3ms total)
    tof_front = VL53L0X_GetRange(TOF_FRONT);
    tof_left  = VL53L0X_GetRange(TOF_LEFT);
    tof_right = VL53L0X_GetRange(TOF_RIGHT);

    // 4. Read TCS34725 (I2C1 — non-blocking, 50ms integration
    //    result polled every 5 cycles = 10Hz effective)
    if (colour_cycle_counter++ >= 5) {
        colour_cycle_counter = 0;
        TCS34725_ReadRGBC(&r, &g, &b, &clear);
        last_color = TCS34725_ClassifyColor(r, g, b, clear);
    }

    /* ─── DECISION ───────────────────────────────────────────── */
    // 5. Run Mission State Machine (uses all sensor data above)
    StateMachine_Update(line_bits, line_error, intersection,
                        tof_front, tof_left, tof_right,
                        last_color, pitch, roll);

    /* ─── CONTROL ────────────────────────────────────────────── */
    // 6. Apply IMU postural corrections to foot targets
    PID_ApplyPostureCorrection(pitch, roll, foot_targets);

    // 7. Update trot gait phase → compute next foot positions
    GaitEngine_Update(sm_Vx, sm_Vy, sm_Wz, foot_targets);

    // 8. Solve Inverse Kinematics → 12 joint angles
    Kinematics_SolveAll(foot_targets, joint_angles);

    // 9. Write PWM to PCA9685 via I2C1 (~2ms)
    PCA9685_WriteAllChannels(joint_angles);

    /* ─── SAFETY ────────────────────────────────────────────────    */
    // 10. Safety: IMU tilt > 20° -> SAFE_STOP
    if (fabsf(pitch_deg) > 20.0f || fabsf(roll_deg) > 20.0f)
        StateMachine_Transition(STATE_SAFE_STOP);
}
```

### State Machine Overview
The Mission HFSM (`Navigation/state_machine.c`) contains **18 states** covering all four competition subtasks:

```
BOOT_INIT → SENSOR_CALIB → IDLE_WAIT_START
  → TASK1_LINE_FOLLOW → TASK1_BALL_APPROACH → TASK1_COLOR_ID
  → TASK1_BALL_GRAB → TASK1_STORE → TASK1_GRID_EXIT
  → TASK2_WALL_FOLLOW
  → TASK3_WALL_FOLLOW → TASK3_OBSTACLE_DETECT → TASK3_PUSH → TASK3_TURN
  → TASK4_LINE_FOLLOW → TASK4_JUNCTION_DETECT → TASK4_BRANCH_FOLLOW
  → TASK4_BALL_RELEASE → FINISH
  → SAFE_STOP (any state on fault) → ERROR_RECOVERY
```

Key persistent variable: `stored_ball_color` — written to STM32 Flash via EEPROM emulation after Task 1 colour identification. Survives STM32 reset (competition restarts).

## 3. End-to-End Data Flow

```
8× TCRT5000 Line Array (GPIO)
    → Weighted centroid error
    → PD line follower → Wz (angular velocity)
    → State Machine → gait Vx, Vy, Wz targets
    → Gait Generator → 4 foot target (x,y,z) positions
    → IK Solver → 12 joint angles (θ1,θ2,θ3 per leg)
    → PCA9685 (I2C) → 15× MG90S servo PWM
    → Physical movement

TCS34725 Colour Sensor (I2C1, arm tip)
    → R/G/B/Clear raw values → colour ratio classification
    → STATE_MACHINE: stored_ball_color (Task 1) / branch selection (Task 4)

3× VL53L0X ToF (I2C2)
    → Wall PD centering (Task 2/3) / obstacle detect (Task 3)

MPU6050 IMU (I2C1)
    → Complementary filter → pitch, roll
    → Postural PID → foot Z offsets for level walking
```

## 4. Debug Interface (Development Only)
- **USART1** (PA9 TX, PA10 RX, 115200 baud): Outputs state machine transitions, sensor values, and timing data via serial terminal during development.
- **MUST be disconnected before competition** (no external communication allowed by rules).
- Use ST-Link SWD for firmware flashing and GDB debugging.
