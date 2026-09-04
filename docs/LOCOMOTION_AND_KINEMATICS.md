# LOCOMOTION, KINEMATICS & GAIT CONTROL

## 1. Coordinate Systems
To calculate how a leg must move, we must define strict coordinate frames:
1. **World Frame (W)**: Fixed to the arena floor.
2. **Body Frame (B)**: Center of the robot chassis. +X is Forward, +Y is Left, +Z is Up.
3. **Leg Frame (L)**: The shoulder joint of each leg. Offset from the Body Frame.
4. **Foot Frame (F)**: The tip of the leg.

## 2. Inverse Kinematics (IK)
Inverse Kinematics answers the question: *"If I want the foot at (X, Y, Z), what angles do the 3 servos need to be at?"*

For a 3-DOF mammalian or insectoid quadruped leg, the math involves trigonometry (Law of Cosines).
- **Inputs**: Desired Foot Position `(X, Y, Z)` relative to the Leg Frame.
- **Link Lengths**: `L1` (Coxa), `L2` (Femur), `L3` (Tibia).
- **Outputs**: Angles `Theta1`, `Theta2`, `Theta3`.

*The exact C code implementation will reside in `kinematics.c`.*

## 3. Gait Generation
A gait is a sequence of lifting and placing feet to move the Body Frame forward without falling.

### Phase 1: Static Crawl Gait (High Stability, Low Speed)
Used during Grid Search (Task 01) and Ball Grasping.
- Only ONE leg is lifted at a time. Three legs always remain on the ground.
- The Center of Gravity (CoG) must physically shift over the triangle formed by the 3 grounded legs *before* the 4th leg lifts.
- **Duty Cycle**: 0.75 (Each leg spends 75% of the time on the ground).

> [!WARNING]
> **Trot gait is NOT used in this design.** MG90S torque at 2-leg stance (trot) = 0.96× safety margin < 1.0 → insufficient. Crawl gait is used exclusively across all tasks.

## 4. Foot Trajectory (Bezier Curves)
When a leg swings forward, it shouldn't just move in a straight line (it would scrape the floor). It must trace an arc.
We use a **Cubic Bezier Curve** to generate a smooth, mathematically continuous arc for the foot to follow.
- `P0`: Start position (on ground).
- `P1`: Lift off control point.
- `P2`: Step down control point.
- `P3`: End position (on ground).

## 5. Locomotion Control Hierarchy
```text
8× TCRT5000 Line Array
   → Weighted centroid error (–3.5 to +3.5)
   → PD Line Follower (Kp, Kd) → Wz (rad/s steering command)
         │
Mission State Machine (STM32)
   → Sets Vx (forward speed) based on current task state
   → Passes [Vx, Vy=0, Wz] to Gait Generator
         │
Gait Generator → 4 foot (X,Y,Z) targets per 20ms cycle
         │
IK Solver → 12 joint angles (θ1,θ2,θ3 per leg)
         │
PCA9685 (I2C1) → 15× MG90S servo PWM → Physical motion
         │
MPU6050 IMU → pitch/roll feedback → Postural PID → foot Z corrections (closed loop)
```

All velocity commands are generated internally within the STM32. There is no external processor providing movement targets.
