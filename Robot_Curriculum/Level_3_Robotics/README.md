# Level 3 — Robotics Core
## Inverse Kinematics, Trot Gait, PD Control, HFSM

## Why This Level Matters
The maths here is the difference between a robot that walks and one that falls over. Every competition run depends on the IK solver being correct and the gait engine being stable.

## Topics

### 1. Coordinate Frames

```
Body frame (fixed to robot chassis):
  X = forward (direction of travel)
  Y = left
  Z = up

Foot position expressed in body frame:
  FL neutral = (+80, +60, -50) mm
  FR neutral = (+80, -60, -50) mm
  BL neutral = (-80, +60, -50) mm
  BR neutral = (-80, -60, -50) mm

Body half-length = 80mm  (centre to front/rear coxa pivot)
Body half-width  = 60mm  (centre to left/right coxa pivot)
Stance depth     = 50mm  (foot below body, z = -50)
```

### 2. Inverse Kinematics — 3-DOF Geometric Solver

```
Link lengths: L1 (Coxa) = 30mm | L2 (Femur) = 60mm | L3 (Tibia) = 80mm
Max reach: L2 + L3 = 140mm | Min reach: |L2 - L3| = 20mm

Given target foot (fx, fy, fz):

Step 1 — Coxa yaw (horizontal plane):
  theta1 = atan2(fy, fx)

Step 2 — Elevation plane projection:
  d_xy    = sqrt(fx² + fy²) - L1      [horizontal reach past coxa]
  d_total = sqrt(d_xy² + fz²)         [femur-to-foot distance]
  Check:  20mm ≤ d_total ≤ 140mm     [reachability]

Step 3 — Femur angle (law of cosines):
  cos_beta = (L2² + d_total² - L3²) / (2·L2·d_total)
  alpha    = atan2(-fz, d_xy)         [elevation angle to foot]
  theta2   = alpha + acos(cos_beta)   [femur from horizontal]

Step 4 — Tibia angle (law of cosines):
  cos_gamma = (L2² + L3² - d_total²) / (2·L2·L3)
  theta3    = π - acos(cos_gamma)     [tibia fold angle]

Step 5 — Convert to servo degrees:
  coxa_servo  = theta1 * (180/π) + 90   [centred at 90°]
  femur_servo = theta2 * (180/π) + 90
  tibia_servo = theta3 * (180/π)
  For right-side (FR, BR): femur = 180 - femur, tibia = 180 - tibia
```

### 3. Trot Gait Engine

```
Gait parameters:
  Period T   = 600ms   (full cycle)
  Swing duty = 50%     (300ms in air)
  Stance duty= 50%     (300ms on ground)
  Step height= 35mm    (parabolic Z arc in swing)
  Phase offsets: FL=0.0, FR=0.5, BL=0.5, BR=0.0

Diagonal pairs: FL + BR swing together → FR + BL stance
               FR + BL swing together → FL + BR stance
Always 2 legs grounded → stable support

Foot trajectory:
  SWING:  x(t) = lerp(x_liftoff, x_land, t)
          y(t) = lerp(y_liftoff, y_land, t)
          z(t) = -body_h + step_height × sin(π·t)  [parabolic arc]

  STANCE: x(t) = x_land - Vx × stance_time × t   [body pushes over foot]
          z(t) = -body_h                            [foot on ground]

Landing target (feedforward):
  x_land = x_neutral + Vx × half_swing - Wz × y_neutral × half_swing
  y_land = y_neutral + Vy × half_swing + Wz × x_neutral × half_swing
```

### 4. PD Controllers

#### Line Following PD
```c
/* Input:  line centroid error [-4..+4] from 9-sensor array   */
/* Output: Wz (yaw rate rad/s) fed into GaitEngine_Update()   */

#define LINE_KP  0.012f   /* Proportional: rad/s per unit error */
#define LINE_KD  0.003f   /* Derivative: damping */
#define WZ_MAX   0.80f

float LinePD_Update(float err, float *prev, float dt) {
    float Wz = LINE_KP * err + LINE_KD * (err - *prev) / dt;
    *prev = err;
    return fmaxf(-WZ_MAX, fminf(WZ_MAX, Wz));
}

/* Tuning procedure:
   1. Set Kd=0, increase Kp until slow oscillation -> Kp_osc
   2. Set Kp = 0.5 * Kp_osc (typically 0.008-0.015)
   3. Increase Kd until oscillation damps (typically 0.001-0.005)
   Target: 1m straight + 90° corner without line loss */
```

#### Wall Following PD
```c
/* Input:  tof[LEFT].dist_mm, tof[RIGHT].dist_mm              */
/* Output: Wz to keep robot centred in corridor               */

#define WALL_KP  0.008f
#define WALL_KD  0.002f
#define WALL_WZ_MAX 0.60f

float WallPD_Update(float L, float R, float *prev, float dt) {
    float err = L - R;   /* +ve = drift right, turn left */
    float Wz = WALL_KP * err + WALL_KD * (err - *prev) / dt;
    *prev = err;
    return fmaxf(-WALL_WZ_MAX, fminf(WALL_WZ_MAX, Wz));
}
```

### 5. Hierarchical Finite State Machine (HFSM)

```
21 states, transitions triggered by sensor data each 20ms:

IDLE -> T1_LINE_FOLLOW -> T1_BALL_APPROACH -> T1_COLOR_ID
     -> T1_BALL_GRAB   -> T1_STORE         -> T1_GRID_EXIT
     -> T2_WALL_FOLLOW -> T3_WALL_FOLLOW   -> T3_OBSTACLE_DETECT
     -> T3_PUSH        -> T3_TURN          -> T4_LINE_FOLLOW
     -> T4_JUNCTION    -> T4_BRANCH        -> T4_BALL_RELEASE
     -> FINISH

Each state outputs:
  sm_Vx, sm_Vy:  body velocity mm/s  (into GaitEngine_Update)
  sm_Wz:         yaw rate rad/s
  sm_arm_pos:    ARM_HOME / ARM_MODE_A / ARM_MODE_B
  sm_gripper:    GRIPPER_OPEN / GRIPPER_CLOSE
  sm_gate:       GATE_LOCKED / GATE_OPEN
```

### 6. Complementary Filter (IMU)

```c
/* Fuses gyroscope (accurate short-term) with accelerometer (no drift)   */
/* CF_ALPHA = 0.98 -> trust gyro 98%, correct with accel 2% each cycle  */

pitch_deg = 0.98f * (pitch_deg + gy_dps * dt) + 0.02f * accel_pitch;
roll_deg  = 0.98f * (roll_deg  + gx_dps * dt) + 0.02f * accel_roll;

/* Used in Task 3: if |pitch_deg| > 20°, robot is tipping -> SAFE_STOP */
```

## Practical Exercises

1. **IK verification**: Command foot target FL=(80, 60, -50). Calculate theta1/2/3 by hand. Verify with `kinematics.c` output. Should give Coxa≈90°, Femur≈45°, Tibia≈135°.
2. **Static stance**: Call `IK_GetNeutralStance()` + `IK_SolveAll()` + `PCA9685_WriteLegs()`. Robot should stand level on all 4 feet.
3. **Slow trot**: Set Vx=20mm/s, Wz=0. Verify FL+BR lift together, then FR+BL. No toe drag (GAIT_STEP_HEIGHT ≥ 25mm).
4. **Line PD loop**: Wire 9 sensors, run line array + PD controller. Robot should follow a black tape line for 1m without loss.
5. **State machine stub**: Implement states 0-3 (BOOT → CALIB → IDLE → T1_LINE_FOLLOW). Press PC13 to trigger transition.

## Prerequisite for Level 4
- Static stance: robot stands level, all 4 feet grounded
- Slow trot stable at Vx=20mm/s for 30 seconds
- Line following at Vx=40mm/s tracks 2m of black tape
- State machine transitions 0→1→2→3 on button press

---
🔙 **[Back to Curriculum](../README.md)**
