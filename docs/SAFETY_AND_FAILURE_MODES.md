# SAFETY AND FAILURE MODES

## 1. Overview
The RUNNER-4 relies on a single STM32F411CEU6 for all control. Robust failure detection and safe recovery must be built into the firmware to protect the robot, arena, and competition score.

## 2. Hardware Safety

| Mechanism | Implementation |
|-----------|----------------|
| **Kill Switch** | Physical latching switch on main battery positive wire (rated ≥15A). Immediately cuts all power including servos. |
| **Fuse** | 10A automotive blade fuse between battery and BEC. Protects against servo stall shorting. |
| **Battery Voltage Monitor** | STM32 ADC (PC1) reads battery voltage via resistor divider. Software alert at <6.8V; halt at <6.4V. |
| **Decoupling Capacitor** | 4700µF electrolytic cap across PCA9685 V+ and GND terminals to absorb servo current spikes. |

## 3. Software Failsafes (STM32 Firmware)

### Excessive Tilt (IMU)
```c
// In Safety_Check(), called every 20ms:
if (fabsf(pitch_deg) > 30.0f || fabsf(roll_deg) > 30.0f) {
    PCA9685_AllServosDisable();   // Zero torque on all channels
    StateMachine_Transition(STATE_SAFE_STOP);
}
```
- **Trigger**: Body pitch or roll exceeds 30° (robot is falling)
- **Response**: All servo PWM disabled immediately → robot falls safely with zero additional motion
- **Recovery**: Manual restart (up to 3 competition restarts allowed)

### Line Lost (IR Array)
```c
static uint32_t all_black_count = 0;

if (LineArray_IsAllBlack()) {
    if (++all_black_count >= 150) {   // 150 × 20ms = 3 seconds
        Vx = 0; Wz = 0;
        StateMachine_Transition(STATE_SAFE_STOP);
        // Initiate search: rotate ±15° alternating to re-acquire line
    }
} else {
    all_black_count = 0;
}
```
- **Trigger**: All 8 TCRT5000 sensors read LOW for more than 3 consecutive seconds
- **Response**: Stop forward motion → slow rotate search pattern → if line found, resume state; if not found after 10s, remain in SAFE_STOP
- **Prevention**: Temporal filter prevents transient bounces (small lift, debris) from triggering

### Low Battery
- **6.8V**: Flash status LED; reduce max gait speed by 30%
- **6.4V**: `Vx = 0, Wz = 0` → SAFE_STOP state → LED rapid blink

### I2C Bus Hang
```c
// If HAL_I2C_Master_Transmit returns HAL_TIMEOUT:
HAL_I2C_DeInit(&hi2c1);
HAL_Delay(10);
HAL_I2C_Init(&hi2c1);   // Re-initialise peripheral
```
- **Applies to**: I2C1 (PCA9685/MPU6050/TCS34725) and I2C2 (VL53L0X)
- **If I2C1 fails**: Servo commands cannot be sent → SAFE_STOP
- **If I2C2 fails**: ToF sensors unavailable → fall back to line-only navigation for Tasks 1 and 4; SAFE_STOP for Tasks 2 and 3 (require ToF)

### TCS34725 Colour Sensor Failure
- **Detection**: NACK on I2C1 read after 2 retries
- **Response**: Skip colour classification states (`TASK1_COLOR_ID`, `TASK4_JUNCTION_DETECT`)
- **Impact**: Partial run (cannot identify ball colour or select branch) — accept points for Task 1 navigation and Task 3 push

### Servo Stall / Overload
- **Detection**: IMU shows no positional change despite gait commands for >2 seconds
- **Response**: Reduce servo load → lower body height (shorter IK reach) → retry gait
- **Escalation**: If stall persists for >5s → SAFE_STOP

### Ball Pickup Timeout
- **Detection**: Gripper closes but robot has not transitioned from `TASK1_BALL_GRAB` within 10 seconds
- **Response**: Retry grab once (open gripper → reposition arm → close)
- **Escalation**: If second attempt fails → skip ball; continue circuit without ball; accept partial score

## 4. Flash Persistence — Ball Colour Recovery
A competition restart clears STM32 RAM but **not Flash**. The `stored_ball_color` variable is written to a dedicated Flash sector after Task 1 colour identification. On boot, the state machine reads this value:

```c
// On BOOT_INIT:
uint8_t recalled = Flash_ReadBallColor();
if (recalled != COLOR_UNKNOWN) {
    stored_ball_color = recalled;
    // If restarting at Task 4 checkpoint: skip directly to TASK4_LINE_FOLLOW
}
```

This allows the team to restart at the Task 4 checkpoint (if allowed by judges) without losing the ball colour memory.

## 5. Competition Restart Procedure
1. Press kill switch → all power off.
2. Team member may adjust sensor positions (line array height, TCS34725 angle) — no code changes allowed.
3. Press kill switch ON → STM32 boots (<500ms) → reads Flash colour.
4. Position robot at checkpoint → press start button → robot resumes from saved state.

## 6. Arena Safety
- Robot must not damage arena walls or floor lines.
- Crawl gait limits peak leg force.
- Passive bumper (no sharp edges) for obstacle pushing.
- All cables secured inside chassis to prevent trailing marks on floor.
