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

### Phase 2: Dynamic Trot Gait (Medium Stability, High Speed)
Used during Straight Corridors (Task 03).
- Diagonal legs move together (e.g., Front-Left and Back-Right lift, while Front-Right and Back-Left push backward).
- **Duty Cycle**: 0.50 (Each leg spends 50% of the time on the ground).

## 4. Foot Trajectory (Bezier Curves)
When a leg swings forward, it shouldn't just move in a straight line (it would scrape the floor). It must trace an arc.
We use a **Cubic Bezier Curve** to generate a smooth, mathematically continuous arc for the foot to follow.
- `P0`: Start position (on ground).
- `P1`: Lift off control point.
- `P2`: Step down control point.
- `P3`: End position (on ground).

## 5. Locomotion Control Hierarchy
```text
Raspberry Pi requests Vx = 5cm/s, Wz = 0.2 rad/s
         │
STM32 Gait Generator calculates where each foot should be in space to achieve that speed
         │
STM32 IK converts that spatial position into servo angles
         │
PCA9685 physically moves the servos
```
