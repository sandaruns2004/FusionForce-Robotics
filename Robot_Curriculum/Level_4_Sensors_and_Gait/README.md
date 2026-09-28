# Level 4 — Sensors & Full Integration
## ToF + IR Array + TCS34725 + IMU + Full Mission Run

## Why This Level Matters
This is where individual subsystems merge into a robot that can autonomously complete all 4 competition tasks. By this level, each sensor driver and the gait engine are working individually — now integrate them and handle all the edge cases.

## Topics

### 1. VL53L0X ToF Sensor System

```
3 sensors on I2C2 (SDA=PB3, SCL=PB10):
  FRONT @ 0x30 — detects pedestals (Task 1), obstacles (Task 3)
  LEFT  @ 0x31 — wall following (Task 2)
  RIGHT @ 0x32 — wall following (Task 2)

Address remapping sequence (all boot at 0x29):
  1. PB12=PB13=PB14=LOW (all in reset)
  2. HAL_Delay(10)
  3. PB12 HIGH -> FRONT boots at 0x29 -> write 0x8A=0x30 -> FRONT=0x30
  4. PB13 HIGH -> LEFT  boots at 0x29 -> write 0x8A=0x31 -> LEFT =0x31
  5. PB14 HIGH -> RIGHT boots at 0x29 -> write 0x8A=0x32 -> RIGHT=0x32

Key thresholds:
  TOF_WALL_TARGET_MM   = 150  (corridor setpoint)
  TOF_GAP_THRESHOLD_MM = 250  (reading > this = gap/opening)
  TOF_OBSTACLE_MM      = 150  (Task 3: obstacle detected)
  TOF_BALL_APPROACH_MM =  80  (Task 1: stop at pedestal)
  TOF_GAP_CONSEC_MIN   =   3  (3 consecutive reads to confirm)

EMA filter: dist_ema = 0.4 * raw + 0.6 * prev  (alpha=0.4)
```

### 2. IR Line Array — 9-Channel TCRT5000

```
Sensor positions:  S1..S9, left to right, 10mm pitch, 80mm total
ADC DMA buffer:    9 sensors × 16 oversamples = 144 words (circular)

Processing pipeline:
  Layer 1: raw[i] = sum(dma_buf[i×16 .. i×16+15]) / 16
  Layer 2: ema[i] = 0.7 × raw[i] + 0.3 × ema_prev[i]
  Layer 3: spike reject if |ema[i] - prev| > 500 counts
  Normalise: lv[i] = 1.0 - clamp((ema[i]-cal_white[i]) / span, 0, 1)

  centroid = sum(i × lv[i]) / sum(lv[i])     [0..8 sensor index]
  error    = centroid - 4.0                   [-4..+4]
  error_mm = error × 10.0                    [-40..+40 mm]

Junction detection: active_count ≥ 6 for ≥ 3 consecutive cycles
Line lost:          sum(lv[i]) ≈ 0 for ≥ 5 cycles -> is_lost=true

Calibration via UART (run before every competition):
  "la_cal white\r\n" -> over white surface
  "la_cal black\r\n" -> over black tape -> saves to Flash
```

### 3. TCS34725 Colour Sensor

```
I2C1 @ 0x29 | Integration time: 50ms | Gain: 4x | LED: PC0=HIGH

Mounted on arm tip (CH12):
  ARM_MODE_A (0°):   sensor at ball height -> ball colour ID (Task 1)
  ARM_MODE_B (160°): sensor at floor level -> zone colour ID (Task 4)

Classification (ratio-dominance, normalised to clear channel):
  r_n = raw_r / raw_c    g_n = raw_g / raw_c    b_n = raw_b / raw_c

  RED:   r_n > 0.40 AND r_n > g_n×1.4 AND r_n > b_n×1.4
  GREEN: g_n > 0.35 AND g_n > r_n×1.2 AND g_n > b_n×1.2
  BLUE:  b_n > 0.30 AND b_n > r_n×1.2 AND b_n > g_n×1.2
  Else:  UNKNOWN

Stability filter: require 3 consecutive same-colour reads before acting
Non-blocking: sensor polled every 3rd 20ms cycle (60ms = faster than 50ms integration)

CRITICAL: Recalibrate thresholds under actual competition lighting.
Arena LED temperature matters — same ball reads differently under warm vs. cool LEDs.
Calibration procedure:
  ARM_MODE_A, LED ON, red ball -> read r_n, g_n, b_n
  Adjust TCS_RED_R_MIN to 0.85 × observed r_n
  Repeat for green and blue balls
```

### 4. MPU6050 IMU — Stability Monitor

```
I2C1 @ 0x68 | Range: ±500 dps gyro, ±2g accel | Update rate: 100Hz (TIM3 ISR)

Complementary filter (CF_ALPHA = 0.98):
  pitch_deg = 0.98 × (pitch_deg + gy_dps × 0.01) + 0.02 × accel_pitch
  roll_deg  = 0.98 × (roll_deg  + gx_dps × 0.01) + 0.02 × accel_roll

Used for:
  - Task 3 obstacle push: if |pitch_deg| > 20° -> robot tipping -> SAFE_STOP
  - Post-push verification: IMU level confirms robot recovered
  - Competition run: monitor for arena floor irregularities

IMU not used for locomotion control (gait is open-loop).
IMU is a safety guard only.
```

### 5. Full 50Hz Integration Loop

```
Every 20ms (TIM2 interrupt):

  1. LA_Update()              -> la.centroid, la.error, la.is_junction, la.is_lost
  2. TCS34725_Poll()          -> color (COLOR_RED/GREEN/BLUE/UNKNOWN)
  3. TOF_UpdateAll()          -> tof[0..2].dist_mm, tof[x].is_gap
  4. (TIM3 ISR, 100Hz)        -> pitch_deg, roll_deg (MPU6050)

  5. StateMachine_Update(s)   -> sm_Vx, sm_Vy, sm_Wz, sm_arm, sm_grip, sm_gate
     Internal PD controllers:
       sm_Wz = LinePD_Update(la.error)  in line-following states
       sm_Wz = WallPD_Update(L,R)       in wall-following states

  6. ARM_Update(arm,grip,gate) -> smooth slew CH12/13/14 via PCA9685
  7. GaitEngine_Update(Vx,Vy,Wz, feet)
  8. IK_SolveAll(feet, joints)
  9. PCA9685_WriteLegs(joints)

  Total: ~7.6ms / 20ms budget  (62% margin)
```

### 6. Competition Mission Sequence

```
Task 1 — Ball Retrieval (30-60s):
  Line follow at Vx=60 -> junction detected AND front<80mm
  -> ARM_MODE_A (800ms) -> colour ID (3 stable reads)
  -> GRIPPER_CLOSE (1200ms) -> ARM_HOME + GATE_LOCKED
  -> Line follow exit grid

Task 2 — Corridor (20-40s):
  Wall-follow at Vx=80 (L+R ToF PD)
  Gap detection: ToF>250mm × 3 readings = corridor opening

Task 3 — Obstacle Push (20-30s):
  Obstacle: FRONT<150mm × 3 readings -> slow push Vx=50
  Exit: FRONT>300mm OR 8s timeout -> in-place turn 3200ms @ Wz=-0.5 rad/s

Task 4 — Ball Release (30-50s):
  Line follow at Vx=60 -> junction
  -> ARM_MODE_B floor scan -> colour match × 3
  -> GATE_OPEN (1500ms) -> ball exits into correct zone
  -> FINISH
```

## Practical Exercises

1. **Sensor integration test**: Run full 50Hz loop with all sensors. Print one-line status via UART every 100ms:
   `[SM:3] C=4.1 F=200 L=155 R=145 col=0 p=0.3`

2. **Task 1 dry run**: Place a red ball 80mm from front sensor, run T1 sequence. Arm should reach, colour read RED, gripper close. Time it.

3. **Task 2 corridor run**: Set up two walls 300mm apart. Run wall-follow state. Robot should stay centred ±20mm for 1m.

4. **Task 3 obstacle**: Place a block 150mm from front. Robot should detect, push, then turn 90°.

5. **Full mission run**: Complete all 4 tasks sequentially under 3 minutes.

## Competition Preparation Tips

| Activity | When | Why |
|----------|------|-----|
| la_cal white+black | Arrival at venue | Arena floor reflectance varies |
| Colour recalibrate | After arena lights on | LED temperature affects TCS34725 |
| Servo trim check | Before each run | Vibration shifts servo offsets |
| ToF scan verify | Before each run | Connector vibration can disconnect |
| Walk test 1m | 5min before run | Confirms gait stable in arena |

---
🔙 **[Back to Curriculum](../README.md)**
