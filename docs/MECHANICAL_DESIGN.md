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
- **Link Lengths**: **L1 (Coxa)=30mm, L2 (Femur)=60mm, L3 (Tibia)=80mm** (confirmed hardware dimensions).
- **Max reach from coxa pivot**: 60+80 = **140mm** | Min reach: |60-80| = **20mm**
- **Symmetry**: All servos mechanically zeroed (90°) before attaching horns (see calibration guide).
- **Foot**: Small rounded tip with rubber O-ring for traction on matte arena surface.

## 4. SubTask 01: Ball Arm + Gripper (with TCS34725)
- **Design**: 2-DOF frontal arm (arm pitch + claw open/close).
- **Mechanism**:
  - **Arm Pitch Servo (CH12)**: Rotates the arm to three calibrated positions:
    - `HOME` (90°): Resting/travel position — arm vertical, safe for walking.
    - `MODE_A` (0° horizontal): Ball colour reading and grasp position — arm forward at pedestal height.
    - `MODE_B` (160° downward): Floor zone colour reading — arm angles to floor below front of robot.
  - **Gripper Servo (CH13)**: Opens (60°) and closes (115°) claw around 40mm ball.
  - **Gate Servo (CH14)**: Locks ball in internal compartment (0°); opens for gravity release (90°).
- **TCS34725 Colour Sensor Mounting**:
  - Sensor mounted rigidly at the arm tip.
  - A small 3D-printed shroud around the sensor blocks ambient arena light, ensuring the built-in LED is the primary light source for consistent readings.
  - When arm is in MODE A, sensor is ~1–2cm from ball surface — within TCS34725 optimal range.
  - When arm is in MODE B, sensor is ~1–3cm from floor line — within TCS34725 optimal range.
- **Why arm-tip mounting**: The arm's range of motion allows the single sensor to serve both the ball-reading task (elevated pedestal) and the floor-reading task (ground level), eliminating the need for a second sensor.

## 5. Line Array Bracket
- **Sensor**: 9-channel TCRT5000 IR array (analogue, ADC1 DMA).
- **Mount Position**: Front-underside of main body chassis, centred on robot midline, perpendicular to forward axis.
- **Mount Height**: 5mm above floor surface (TCRT5000 optimal at 5mm).
- **Array Width**: 80mm total, 10mm pitch, S1-S9 left to right (PA0-PA7, PB0).
- **Wiring**: S1-S8 signal wires to STM32 ADC1 PA0-PA7; S9 to PB0. 10kΩ pull-up to 3.3V per channel.
- **DMA Buffer**: ADC1 + DMA1 circular, 9×16 = 144 words. 16× oversampling per sensor.

## 6. SubTask 03: Obstacle Pushing Bumper
- **Design**: A flat, ~80–100mm wide, 50mm tall rigid bumper plate attached to the lower front chassis.
- **Material**: PETG, 50%+ infill for rigidity.
- **Why**: The Task 03 obstacle is 25×25×20cm. The flat bumper maximises contact area and transfers leg push force into the obstacle without leg entanglement.
- **No moving parts**: Pure passive structure — zero failure modes.

## 7. Centre of Mass Analysis
- Battery (bottom-centre) ensures lowest possible CoG.
- 3-DOF legs with L1=30 L2=60 L3=80mm: foot reaches up to 140mm from coxa pivot.
- At neutral stance (all feet at z=-50mm), body is 50mm above floor.
- TCS34725 at arm tip (~5g) has negligible CoM impact.
- When ball (~30g) is stored in front belly compartment: CoM shifts forward ~+2mm — within trot support polygon.
- During trot (2-leg stance FL+BR or FR+BL), support triangle is ~160mm × 120mm — CoG stays inside at Vx up to ~100mm/s.
