# STM32 LOW-LEVEL ARCHITECTURE

## 1. Role in the System
The STM32F411CEU6 (Black Pill) is the **complete brain and spine** of the RUNNER-4 robot. It is the sole compute node in the system — there is no Raspberry Pi, no Linux OS, and no external computer involved during operation.

The STM32 handles everything:
- **Perception**: Reading the 8-channel TCRT5000 line array and TCS34725 colour sensor
- **Decision**: Running the 18-state Mission Hierarchical FSM across all 4 competition subtasks
- **Control**: Inverse Kinematics, gait generation, and IMU-based postural stabilization
- **Actuation**: Commanding all 15 MG90S servos via PCA9685 at exactly 50Hz
- **Safety**: Watchdog timers, battery monitoring, tilt detection

## 2. Selected MCU: STM32F411CEU6

| Parameter | Value |
|-----------|-------|
| CPU | ARM Cortex-M4F (hardware FPU) |
| Clock | 100 MHz (via PLL from 25MHz HSE or 8MHz HSI) |
| Flash | **512 KB** (sufficient for IK tables + gait + all drivers + state machine) |
| RAM | **128 KB** (sufficient for sensor buffers, gait matrices, and state variables) |
| I2C | 3× (I2C1 and I2C2 both used) |
| USART | 3× (USART1 for debug only) |
| GPIO | 50 pins |
| ADC | 12-bit, multiple channels |
| Form Factor | Black Pill (identical pinout to F401, drop-in replacement) |

> [!NOTE]
> The STM32F411CEU6 is a drop-in replacement for the original STM32F401CCU6. Same physical Black Pill board, same pinout. Only the STM32CubeMX target chip needs to change. All HAL driver code is reused without modification.

## 3. Firmware Architecture
The codebase is structured using STM32CubeMX HAL in C.

```text
firmware/
├── Core/
│   ├── Src/main.c                 (Entry point, peripheral init, main loop)
│   ├── Src/stm32f4xx_it.c         (TIM2 ISR, I2C ISR, EXTI)
│   └── Src/system_stm32f4xx.c    (100MHz clock config)
├── Drivers/
│   ├── PCA9685/                   (I2C PWM — all 15 servos)
│   ├── VL53L0X/                   (I2C ToF — 3 sensors, XSHUT remap)
│   ├── MPU6050/                   (I2C IMU — complementary filter)
│   ├── TCS34725/                  ← NEW: I2C RGBC colour sensor
│   └── LineArray/                 ← NEW: 8× GPIO digital IR line array
├── Control/
│   ├── kinematics.c               (3-DOF IK math — atan2, law of cosines)
│   ├── gait_generator.c           (Bezier crawl gait — FL→BR→FR→BL)
│   └── pid_posture.c              (IMU pitch/roll → foot Z corrections)
└── Navigation/                    ← NEW: Complete mission layer
    ├── state_machine.c            (18-state HFSM — all 4 subtasks)
    ├── line_follower.c            (PD controller from centroid error)
    └── task_planner.c             (Arm sequencing, ball grab, push, release)
```

## 4. The 50Hz Main Control Loop

The servos expect a PWM signal at 50Hz (every 20ms). The entire control loop must complete within 20ms. TIM2 hardware timer fires every 20ms, sets `update_flag`, and the main loop executes:

```c
// In main while(1):
if (update_flag) {
    update_flag = 0;

    // ── SENSE ──────────────────────────────────────────────────
    uint8_t line_bits  = LineArray_Read();              // <0.1ms, GPIO
    float   line_err   = LineArray_GetCentroid(line_bits);
    bool    at_cross   = LineArray_IsIntersection();    // temporal filter

    MPU6050_Read(&pitch, &roll);                        // ~1ms, I2C1
    tof_F = VL53L0X_GetRange(FRONT);                   // ~3ms, I2C2
    tof_L = VL53L0X_GetRange(LEFT);
    tof_R = VL53L0X_GetRange(RIGHT);

    TCS34725_PollNonBlocking(&last_color);              // non-blocking, ~0.1ms poll

    // ── DECIDE ─────────────────────────────────────────────────
    StateMachine_Update(line_bits, line_err, at_cross,
                        tof_F, tof_L, tof_R,
                        last_color, pitch, roll);       // <1ms

    // ── CONTROL ────────────────────────────────────────────────
    PID_ApplyPostureCorrection(pitch, roll, foot_targets);
    GaitGenerator_Update(Vx, Vy, Wz, foot_targets);
    Kinematics_SolveAll(foot_targets, joint_angles);
    PCA9685_WriteAllChannels(joint_angles);             // ~2ms, I2C1

    // ── SAFETY ─────────────────────────────────────────────────
    batt_mV = ADC_ReadBattery();
    Safety_Check(pitch, roll, line_bits, batt_mV);
}
// Total estimated loop time: ~8–12ms → well within 20ms budget
```

## 5. Hardware Peripherals Used

| Peripheral | Config | Purpose |
|-----------|--------|---------|
| **I2C1** (PB6/PB7) | 400kHz Fast Mode | PCA9685 (0x40) + MPU6050 (0x68) + TCS34725 (0x29) |
| **I2C2** (PB10/PB3) | 400kHz Fast Mode | 3× VL53L0X (0x30, 0x31, 0x32 after XSHUT remap) |
| **GPIOA** (PA0–PA7) | Input, pull-down | 8-channel TCRT5000 line array reads |
| **GPIOB** (PB12–PB14) | Output, push-pull | VL53L0X XSHUT pins (address remap at boot) |
| **GPIOC** (PC0) | Output, push-pull | TCS34725 built-in LED illuminator control |
| **GPIOC** (PC1) | ADC1 IN11 | Battery voltage monitor (voltage divider) |
| **TIM2** | 50Hz (20ms period) | Main control loop pacing interrupt |
| **TIM3** | Optional | Debug LED blink / buzzer tone |
| **USART1** (PA9/PA10) | 115200 baud | Debug serial — **dev only, disconnect at competition** |

## 6. Failsafes and Safety

| Condition | Detection | Response |
|-----------|-----------|---------|
| **Line Lost** | All 8 GPIOA LOW for >3 seconds | `Vx=0, Wz=0` → SAFE_STOP → ±15° rotate search |
| **Excessive Tilt** | IMU pitch or roll >30° | Emergency stop all servos → SAFE_STOP |
| **Low Battery** | ADC reads <6.8V | Reduce gait speed; flash LED warning; stop at 6.4V |
| **I2C Bus Hang** | HAL I2C timeout flag set | Deinit + reinit I2C peripheral automatically |
| **TCS34725 NACK** | I2C NACK on read | Retry once; if fail, skip colour classification (partial run) |
| **Servo Stall** | IMU shows no motion despite commands | Reduce load; command safe posture; retry gait |

## 7. Flash EEPROM Emulation (Ball Colour Persistence)
The STM32F411 has no hardware EEPROM. To persist `stored_ball_color` across resets (competition restart recovery):

```c
// Use last 2KB of Flash (sector 7 on F411, address 0x0807F800)
// Write after Task 1 colour identification:
void Flash_StoreBallColor(uint8_t color) {
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef erase = { FLASH_TYPEERASE_SECTORS, 7, 1, VOLTAGE_RANGE_3 };
    uint32_t err;
    HAL_FLASHEx_Erase(&erase, &err);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, 0x0807F800, color);
    HAL_FLASH_Lock();
}

// Read on boot — if valid, skip Task 1 colour detection:
uint8_t Flash_ReadBallColor(void) {
    uint8_t val = *(uint8_t*)0x0807F800;
    return (val == 0xFF) ? COLOR_UNKNOWN : val;  // 0xFF = erased (no data)
}
```
