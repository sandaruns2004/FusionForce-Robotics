# MECHANICAL DESIGN

## 1. Overview
The RUNNER-4 quadruped mechanical chassis is optimised for a low centre of gravity (CoG), symmetrical mass distribution, and specific task execution (grasping, sensing, and pushing) decoupled from the primary locomotion gait.

## 2. Base Chassis

- **Material**: 3D-printed PETG (heat-resistant, impact-tough). 25–30% infill for structural parts.
- **Form Factor**: Symmetrical rectangle, target body ~160×140mm (≤250×250mm footprint with legs folded).
- **Multi-Deck Component Placement**:
  - **Bottom Deck**: 2S LiPo battery (heaviest component) mounted perfectly centred (lowest CoG).
  - **Middle Deck**: STM32F411 + PCA9685 + 5V/15A BEC + power distribution.
  - **Top Deck**: Debug header connector, LED indicators, power switch access.
  - **Front-Underside**: 8-channel TCRT5000 IR line array bracket (see Section 5).
  - **Front Face**: 3× VL53L0X ToF sensors (front, left-front, right-front corners).
  - **Front Arm**: 2-DOF arm+gripper with TCS34725 at tip (see Section 4).
  - **Front-Bottom**: Passive PETG bumper plate.

> [!NOTE]
> The Raspberry Pi 4B, Pi Camera Module, CSI ribbon cable, and separate 5V/3A Pi power regulator have been removed from this design. The top deck is now free and lighter.

## 3. Leg Design
- **Configuration**: 12-DOF (3-DOF per leg × 4 legs).
- **Joints**:
  1. **Coxa (Shoulder Yaw ±45°)**: Sweeps the leg forward/backward and laterally.
  2. **Femur (Shoulder Pitch ±60°)**: Lifts the leg.
  3. **Tibia (Knee Pitch 0°–135°)**: Extends the leg downward.
- **Link Lengths**: L_coxa = 25mm, L_femur = 50mm, L_tibia = 55mm (proposed; validate on prototype).
- **Symmetry**: All servos mechanically zeroed (90°) before attaching horns.
- **Foot**: Small rounded tip with rubber O-ring for traction on matte arena surface.

## 4. SubTask 01: Ball Arm + Gripper (with TCS34725)
- **Design**: 2-DOF frontal arm (arm pitch + claw open/close).
- **Mechanism**:
  - **Arm Pitch Servo (CH12)**: Rotates the arm to three calibrated positions:
    - `HOME` (~90°): Resting/travel position — arm retracted.
    - `MODE A` (~0° horizontal): Ball colour reading and grasp position — arm lowers to pedestal height.
    - `MODE B` (~−70° downward): Floor zone colour reading — arm points at floor below front of robot.
  - **Gripper Servo (CH13)**: Opens and closes claw around 40mm ball.
- **TCS34725 Colour Sensor Mounting**:
  - Sensor mounted rigidly at the arm tip.
  - A small 3D-printed shroud around the sensor blocks ambient arena light, ensuring the built-in LED is the primary light source for consistent readings.
  - When arm is in MODE A, sensor is ~1–2cm from ball surface — within TCS34725 optimal range.
  - When arm is in MODE B, sensor is ~1–3cm from floor line — within TCS34725 optimal range.
- **Why arm-tip mounting**: The arm's range of motion allows the single sensor to serve both the ball-reading task (elevated pedestal) and the floor-reading task (ground level), eliminating the need for a second sensor.

## 5. Line Array Bracket (NEW)
- **Sensor**: 8-channel TCRT5000 IR array.
- **Mount Position**: Front-underside of main body chassis, centred on robot midline, perpendicular to forward axis.
- **Mount Height**: 5–8mm above floor surface (adjust via slot in bracket; verify after leg length final assembly).
- **Bracket Design**: Simple 3D-printed L-bracket or integrated into body baseplate. Protect sensors from side impacts.
- **Wiring**: 8 signal wires routed inside chassis to STM32 GPIO PA0–PA7. Keep wire run <15cm to avoid noise.

## 6. SubTask 03: Obstacle Pushing Bumper
- **Design**: A flat, ~80–100mm wide, 50mm tall rigid bumper plate attached to the lower front chassis.
- **Material**: PETG, 50%+ infill for rigidity.
- **Why**: The Task 03 obstacle is 25×25×20cm. The flat bumper maximises contact area and transfers leg push force into the obstacle without leg entanglement.
- **No moving parts**: Pure passive structure — zero failure modes.

## 7. Centre of Mass Analysis
- Battery (bottom-centre) ensures lowest possible CoG.
- Removing Pi 4B (46g, top-mounted) lowers CoG further compared to original design.
- TCS34725 at arm tip (~5g) has negligible CoM impact.
- When ball (~30g) is stored in front compartment: CoM shifts forward ~+2mm — within support polygon margin.
