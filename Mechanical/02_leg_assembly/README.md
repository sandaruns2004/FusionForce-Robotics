# 02 Leg Assembly

## Objective
Assemble the four 3-DOF (Degree of Freedom) mammalian legs.

## Kinematic Structure
Each leg consists of three joints:
1. **Coxa (Shoulder Yaw)**: Moves the leg forward/backward.
2. **Femur (Shoulder Pitch)**: Lifts the leg up and down.
3. **Tibia (Knee Pitch)**: Extends the lower leg.

## Hardware Required
- 12x DS3218 (or similar 20kg+ torque) metal-gear servos.
- M3 Machine Screws and Locknuts.
- Radial bearings for the opposite side of the servo horns to prevent cantilever stress on the servo shafts.

## Assembly Steps
1. **DO NOT ATTACH SERVO HORNS YET.** (This is critical. Read Module 06).
2. Assemble the structural brackets for the Coxa, inserting the bearings.
3. Attach the Femur linkage to the Coxa bracket.
4. Attach the Tibia linkage to the Femur.
5. Add a rubber foot or silicone cap to the tip of the Tibia. Hard plastic on a hard arena floor will slip, causing the IK math to fail.

## Debugging
- **Binding**: If a joint feels stiff when moved by hand (with servo powered off), the bracket is too tight or misaligned. The servo will burn out trying to overcome this friction. Loosen the locknuts slightly.
