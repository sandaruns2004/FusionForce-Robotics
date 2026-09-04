# EMBEDDED STATE MACHINE

> [!NOTE]
> This document replaces the former Python-based Hierarchical Finite State Machine that ran on the Raspberry Pi. The Mission HFSM is now implemented entirely in C on the STM32F411CEU6.

## 1. Overview
The Mission State Machine (`firmware/Navigation/state_machine.c`) is the top-level decision layer on the STM32. It runs every 20ms inside the main control loop, reads all sensor data, manages state transitions, and issues locomotion and arm commands to complete all four competition subtasks.

**Key design properties:**
- Deterministic: transitions are based on hard sensor thresholds, not probabilistic models
- Non-blocking: no `HAL_Delay` calls inside the state machine
- Flash-persistent: `stored_ball_color` survives STM32 resets (competition restart recovery)
- Timeout-safe: every state has a maximum timeout to prevent infinite hang

---

## 2. Input Data Structure
```c
typedef struct {
    uint8_t  line_bits;       // 8-bit bitmask from TCRT5000 (bit0=S1...bit7=S8)
    float    line_centroid;   // Weighted centroid –3.5 to +3.5 (NaN if all zero)
    bool     intersection;    // True if ≥6 sensors active for ≥3 consecutive cycles
    uint16_t tof_front_mm;    // VL53L0X front distance (mm)
    uint16_t tof_left_mm;     // VL53L0X left distance (mm)
    uint16_t tof_right_mm;    // VL53L0X right distance (mm)
    ColorID_t last_color;     // TCS34725 last classification result
    float    pitch_deg;       // MPU6050 filtered pitch (degrees)
    float    roll_deg;        // MPU6050 filtered roll (degrees)
    uint16_t batt_mv;         // ADC battery voltage (mV)
} SensorData_t;
```

---

## 3. Output Commands
The state machine sets global targets each cycle:
```c
float  gait_Vx;        // Forward velocity (mm/s)
float  gait_Vy;        // Lateral velocity (mm/s, usually 0)
float  gait_Wz;        // Angular velocity (rad/s) — from PD line follower or override
uint8_t arm_position;  // ARM_HOME | ARM_MODE_A | ARM_MODE_B
uint8_t gripper_state; // GRIPPER_OPEN | GRIPPER_CLOSE
uint8_t gate_state;    // GATE_LOCKED | GATE_OPEN
```

---

## 4. Complete State Table

| State | Entry Condition | Sensor Actions | Exit Condition | Next State |
|-------|----------------|---------------|----------------|-----------|
| `BOOT_INIT` | Power-on | Init all peripherals | Init complete | `SENSOR_CALIB` |
| `SENSOR_CALIB` | Auto | XSHUT remap VL53L0X; TCS34725 init; read Flash `stored_ball_color` | Calib done (<500ms) | `IDLE_WAIT_START` |
| `IDLE_WAIT_START` | Auto | Hold position; wait for start button (PA0) | Start button pressed | `TASK1_LINE_FOLLOW` |
| `TASK1_LINE_FOLLOW` | Auto | Line array PD steering; Vx=60mm/s | Intersection detected AND tof_front <80mm | `TASK1_BALL_APPROACH` |
| | | | Intersection detected AND tof_front ≥80mm | Stay (no ball here) — re-acquire line |
| `TASK1_BALL_APPROACH` | Intersection + ToF<80mm | Stop Vx; arm → MODE_A | arm at MODE_A confirmed | `TASK1_COLOR_ID` |
| `TASK1_COLOR_ID` | Arm at MODE_A | LED ON; TCS34725 reads (2 cycles for stable result) | `last_color` ≠ UNKNOWN for 3 cycles | `TASK1_BALL_GRAB` |
| | | | Timeout >10s | `TASK1_LINE_FOLLOW` (no ball, false alarm) |
| `TASK1_BALL_GRAB` | Colour confirmed | Store `color` → Flash; arm maintain MODE_A; gripper → CLOSE | Gripper closed + timeout 1s | `TASK1_STORE` |
| `TASK1_STORE` | Gripper closed | Raise arm → HOME; gate → LOCKED | Arm at HOME confirmed | `TASK1_GRID_EXIT` |
| `TASK1_GRID_EXIT` | Ball stored | Line follow; Vx=60mm/s | Line lost (arena exit) OR specific exit marker | `TASK2_WALL_FOLLOW` |
| `TASK2_WALL_FOLLOW` | Auto | Wall PD centering (tof_L, tof_R); gap rejection (require 3 gap readings); Vx=80mm/s | tof_front <150mm (obstacle OR end of corridor) | `TASK3_WALL_FOLLOW` |
| `TASK3_WALL_FOLLOW` | Auto | Wall PD + Vx=80mm/s | tof_front <150mm | `TASK3_OBSTACLE_DETECT` |
| `TASK3_OBSTACLE_DETECT` | ToF front <150mm | Stop; verify persistent (3 cycles) | Confirmed obstacle | `TASK3_PUSH` |
| `TASK3_PUSH` | Obstacle confirmed | Lower body; widen stance; Vx=50mm/s (push) | tof_front >300mm OR timeout 8s | `TASK3_TURN` |
| `TASK3_TURN` | Obstacle cleared | Execute left 90° turn; Vx=0; Wz override | Turn complete (heading 90° from entry) | `TASK4_LINE_FOLLOW` |
| `TASK4_LINE_FOLLOW` | Auto | Line array PD; Vx=60mm/s | Intersection detected | `TASK4_JUNCTION_DETECT` |
| `TASK4_JUNCTION_DETECT` | Intersection | Stop; arm → MODE_B (floor read); LED ON | `last_color` stable for 3 cycles | `TASK4_BRANCH_FOLLOW` |
| `TASK4_BRANCH_FOLLOW` | Colour matched | Arm → HOME; turn onto matching colour branch; line follow colour branch; Vx=60mm/s | End of branch (line lost + tof_front>400mm) | `TASK4_BALL_RELEASE` |
| `TASK4_BALL_RELEASE` | Branch end | Stop; gate → OPEN (gravity release); wait 1s | Gate opened 1s | `FINISH` |
| `FINISH` | Ball released | Stop all motion; flash LED pattern | Never (terminal) | — |
| `SAFE_STOP` | Any safety fault | Vx=0, Wz=0; hold position | Start button press for restart | `SENSOR_CALIB` |
| `ERROR_RECOVERY` | `SAFE_STOP` + line search | Rotate ±15° alternating for line re-acquisition | Line found | Previous task line state |

---

## 5. Key Transition Logic (C Pseudocode)

### Intersection → Ball vs No-Ball Decision (Task 1)
```c
case STATE_TASK1_LINE_FOLLOW:
    gait_Wz = LineFollower_ComputeOmega(sensors.line_centroid, dt);
    gait_Vx = 60.0f;

    if (sensors.intersection) {
        if (sensors.tof_front_mm < 80) {
            // Ball on pedestal detected — approach and read
            StateMachine_Transition(STATE_TASK1_BALL_APPROACH);
        }
        // else: no ball at this intersection — continue line following
    }
    break;
```

### Junction Branch Selection (Task 4)
```c
case STATE_TASK4_JUNCTION_DETECT:
    // Arm in MODE_B (floor-pointing)
    if (color_stable_count >= 3) {
        if (sensors.last_color == stored_ball_color) {
            branch_direction = sensors.last_color;
            StateMachine_Transition(STATE_TASK4_BRANCH_FOLLOW);
        } else {
            // Reading a non-matching branch; rotate to check another arm of junction
        }
    }
    break;
```

---

## 6. Memory Variables (Persistent Across Resets)

| Variable | Type | Storage | Purpose |
|----------|------|---------|---------|
| `stored_ball_color` | `uint8_t` | STM32 Flash sector | Ball colour from Task 1; recalled at boot for Task 4 |

```c
// Flash sector: Sector 7 of STM32F411 (0x0807F800, 128KB)
// Values: 0x01=RED, 0x02=GREEN, 0x03=BLUE, 0xFF=empty

void StateMachine_SaveBallColor(ColorID_t color) {
    Flash_StoreBallColor((uint8_t)color);
}

// Called in BOOT_INIT:
void StateMachine_Init(void) {
    uint8_t recalled = Flash_ReadBallColor();
    stored_ball_color = (recalled != 0xFF) ? (ColorID_t)recalled : COLOR_UNKNOWN;
}
```

---

## 7. Timing Budget Per State

| State | Primary Sensors Active | Estimated CPU Load |
|-------|----------------------|--------------------|
| `TASK1_LINE_FOLLOW` | Line array (GPIO) + IMU | Low (<2ms of 20ms budget) |
| `TASK1_COLOR_ID` | TCS34725 (non-blocking) + IMU | Medium (~3ms when read arrives) |
| `TASK2_WALL_FOLLOW` | ToF ×3 + IMU | Medium (~4ms for 3 ToF reads) |
| `TASK3_PUSH` | ToF front + IMU (tilt monitor) | Medium |
| `TASK4_JUNCTION_DETECT` | TCS34725 (non-blocking) + Line array | Medium |
| All states | PCA9685 PWM write | Fixed ~2ms per cycle |
| **Total worst case** | All sensors active | **~12ms < 20ms budget** ✅ |
