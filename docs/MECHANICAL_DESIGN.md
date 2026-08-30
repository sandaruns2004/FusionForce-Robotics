# MECHANICAL DESIGN

## 1. Overview
The FusionForce quadruped mechanical chassis is optimized for a low center of gravity (CoG), symmetrical mass distribution, and specific task execution (grasping and pushing) decoupled from the primary locomotion gait.

## 2. Base Chassis
- **Material**: 3D-printed PETG or ABS for impact resistance. Avoid PLA for parts under high servo-horn stress.
- **Form Factor**: Symmetrical rectangle.
- **Component Placement**:
  - **Bottom Deck**: 3S LiPo battery (heaviest component) mounted perfectly centered.
  - **Middle Deck**: PCA9685, 10A UBEC, and power distribution board.
  - **Top Deck**: Raspberry Pi and STM32.
  - **Front Plate**: Pi Camera module, mounted at a 45-degree downward angle. ToF sensor mounted perfectly flat.

## 3. Leg Design
- **Configuration**: 12-DOF (3-DOF per leg).
- **Joints**: 
  1. **Coxa (Shoulder Yaw)**: Sweeps the leg forward and backward.
  2. **Femur (Shoulder Pitch)**: Lifts the leg.
  3. **Tibia (Knee Pitch)**: Extends the leg downward.
- **Symmetry**: Legs must be mounted in an X-configuration or parallel spider configuration. Ensure all servos are mechanically zeroed (90 degrees) before attaching horns.

## 4. SubTask #01: Ball Gripper
- **Design**: A 2-DOF frontal micro-gripper.
- **Mechanism**:
  - A base servo (pitch) rotates a lightweight claw bracket 90 degrees up and down.
  - A top servo (yaw/grip) opens and closes two interlocking 3D-printed fingers.
- **Why**: Grabbing a ball using the quadruped's front legs requires shifting the robot's entire CoG onto its back two legs. This is computationally complex and mechanically unstable. A dedicated, lightweight front gripper solves the task trivially.

## 5. SubTask #03: Obstacle Pushing Bumper
- **Design**: A flat, 10cm wide, 5cm tall rigid bumper plate attached to the lower front chassis.
- **Why**: The obstacle in Task 03 is 25x25x20cm. If the quadruped tries to push it with its front legs, the legs will tangle. By walking forward, the fixed bumper transfers the forward momentum directly from the chassis into the obstacle.
