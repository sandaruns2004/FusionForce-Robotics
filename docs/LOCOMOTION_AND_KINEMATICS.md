# LOCOMOTION, KINEMATICS & GAIT CONTROL
## RUNNER-4 | Trot Gait | 3-DOF IK | L1=30mm L2=60mm L3=80mm

## 1. Coordinate Systems

```
World Frame (W):  Fixed to arena floor.
Body Frame (B):   Centre of robot chassis.
                  +X = forward (direction of travel)
                  +Y = left
                  +Z = up

Leg coxa pivot offsets from body centre:
  FL: (+80, +60,   0)   FR: (+80, -60,   0)
  BL: (-80, +60,   0)   BR: (-80, -60,   0)

Neutral foot positions (body frame, mm):
  FL: (+80, +60, -50)   FR: (+80, -60, -50)
  BL: (-80, +60, -50)   BR: (-80, -60, -50)
```

## 2. Link Lengths (Confirmed Hardware)

| Segment | Symbol | Length | Function |
|---------|--------|--------|----------|
| Coxa | L1 | **30 mm** | Horizontal yaw pivot, hip to femur |
| Femur | L2 | **60 mm** | Upper leg, load-bearing vertical link |
| Tibia | L3 | **80 mm** | Lower leg, ground contact |
| Max reach | L2+L3 | **140 mm** | From coxa pivot |
| Min reach | \|L2-L3\| | **20 mm** | From coxa pivot |
| Body half-len | Bl | 80 mm | Centre to front/rear coxa |
| Body half-wid | Bw | 60 mm | Centre to left/right coxa |

## 3. Inverse Kinematics (IK) — Geometric 3-DOF

### Derivation

```
Given target foot position (fx, fy, fz) in body frame:

STEP 1 — Coxa yaw angle (horizontal):
  theta1 = atan2(fy, fx)

STEP 2 — Elevation plane projection:
  d_xy    = sqrt(fx² + fy²) - L1     [horizontal reach past coxa]
  d_total = sqrt(d_xy² + fz²)        [femur-pivot to foot distance]

  Reachability: 20mm ≤ d_total ≤ 140mm

STEP 3 — Femur angle (law of cosines):
  cos_beta = (L2² + d_total² - L3²) / (2·L2·d_total)
  alpha    = atan2(-fz, d_xy)         [elevation angle to foot]
  theta2   = alpha + acos(cos_beta)   [femur from horizontal]

STEP 4 — Tibia angle (law of cosines):
  cos_gamma = (L2² + L3² - d_total²) / (2·L2·L3)
  theta3    = π - acos(cos_gamma)     [tibia fold angle]

STEP 5 — Convert to servo degrees:
  coxa_servo  = theta1 × (180/π) + 90   [centred = 90°]
  femur_servo = theta2 × (180/π) + 90
  tibia_servo = theta3 × (180/π)
  Right-side legs (FR, BR): femur_servo = 180 - femur_servo
                             tibia_servo = 180 - tibia_servo
```

### Neutral Stance Output Angles

| Leg | Coxa | Femur | Tibia | CH |
|-----|------|-------|-------|----|
| FL | 90° | 45° | 135° | CH0/1/2 |
| FR | 90° | 135° | 45° | CH3/4/5 |
| BL | 90° | 45° | 135° | CH6/7/8 |
| BR | 90° | 135° | 45° | CH9/10/11 |

### Implementation
- C code: `firmware/Motion/kinematics.h` + `kinematics.c`
- Key defines: `IK_L1_MM 30.0f` | `IK_L2_MM 60.0f` | `IK_L3_MM 80.0f`
- Function: `IK_SolveAll(feet[4], joint_angles_deg[12])`

## 4. Gait — Diagonal Trot

> **Gait selection:** Trot used exclusively. Crawl gait was previously considered but trot provides adequate stability for MG90S with 35mm step height on smooth arena floors.

### Phase Diagram

```
  Period T = 600ms | Swing = 50% (300ms) | Stance = 50% (300ms)

  Time:  0ms        300ms        600ms
         |            |            |
  FL:    [##SWING###][ STANCE     ]   Phase offset 0.0
  BR:    [##SWING###][ STANCE     ]   Phase offset 0.0
  FR:    [ STANCE    ][##SWING###]    Phase offset 0.5
  BL:    [ STANCE    ][##SWING###]    Phase offset 0.5

  2 legs always grounded -> stable diagonal support
```

### Foot Trajectory

```
SWING phase (parabolic arc):
  x(t) = lerp(x_liftoff, x_land, t_norm)
  y(t) = lerp(y_liftoff, y_land, t_norm)
  z(t) = -body_h + 35mm × sin(π × t_norm)   [35mm step height]

STANCE phase (body pushes forward over fixed foot):
  x(t) = x_land - Vx × stance_time × t_norm
  y(t) = y_land - Vy × stance_time × t_norm
  z(t) = -body_h   [foot stays on floor]

Swing landing target (velocity feedforward):
  x_land = x_neutral + Vx × 0.15 - Wz × y_neutral × 0.15
  y_land = y_neutral + Vy × 0.15 + Wz × x_neutral × 0.15
```

### Gait Parameters

| Parameter | Value | Range | Effect |
|-----------|-------|-------|--------|
| Period T | 600ms | 400–800ms | Slower = more stable |
| Step height | 35mm | 25–55mm | Higher = clears obstacles |
| Vx_max | 100mm/s | — | Forward speed clamp |
| Vy_max | 60mm/s | — | Lateral speed clamp |
| Wz_max | 0.80 rad/s | — | Yaw rate clamp |
| body_h | 50mm | 40–70mm | Body height above floor |

### Implementation
- C code: `firmware/Motion/gait_engine.h` + `gait_engine.c`

## 5. Locomotion Control Hierarchy

```
9× TCRT5000 IR Array (ADC1 DMA) or 3× VL53L0X ToF (I2C2)
   → Sensor error signal (line centroid or wall difference)
   → PD Controller → Wz (rad/s steering command)
         │
Mission State Machine (50Hz, TIM2 interrupt)
   → Sets Vx, Vy, Wz per state
         │
GaitEngine_Update(Vx, Vy, Wz, feet[4])
   → 4 foot (X, Y, Z) targets in body frame
         │
IK_SolveAll(feet[4], joints[12])
   → 12 joint angles [Coxa, Femur, Tibia × 4 legs]
         │
PCA9685_WriteLegs(joints[12]) via I2C1
   → 15× MG90S servo PWM → Physical motion
         │
MPU6050 IMU (100Hz, TIM3 ISR)
   → Complementary filter → pitch, roll
   → Safety watchdog only (open-loop gait — no postural correction)
```

## 6. PD Controllers

### Line Following PD
```c
#define LINE_KP   0.012f
#define LINE_KD   0.003f
#define WZ_MAX    0.80f     /* rad/s */

/* error = centroid - 4.0, range [-4..+4] */
float LinePD_Update(float err, float *prev, float dt_s) {
    float Wz = LINE_KP * err + LINE_KD * (err - *prev) / dt_s;
    *prev = err;
    return fmaxf(-WZ_MAX, fminf(WZ_MAX, Wz));
}
```

### Wall Following PD
```c
#define WALL_KP      0.008f
#define WALL_KD      0.002f
#define WALL_WZ_MAX  0.60f   /* rad/s */

/* error = left_mm - right_mm (+ve = drift right = turn left) */
float WallPD_Update(float L, float R, float *prev, float dt_s) {
    float err = L - R;
    float Wz = WALL_KP * err + WALL_KD * (err - *prev) / dt_s;
    *prev = err;
    return fmaxf(-WALL_WZ_MAX, fminf(WALL_WZ_MAX, Wz));
}
```

---
🔙 **[Back to docs](./README.md)**
