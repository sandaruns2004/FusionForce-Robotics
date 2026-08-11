# REQUIREMENTS

## 1. Primary Source of Truth
This document outlines the strict technical requirements for the robot, derived exclusively from the authoritative source: `Task/tasks_circuit_v2.md`.

## 2. Competition Objectives
The BREACH PROTOCOL competition requires a fully autonomous robot to traverse a highly specific 200cm x 200cm arena. The overall mission is broken into four continuous subtasks:
1. **Grid Search (SubTask 01)**: Navigate a 4x4 line grid, locate a ball, identify its color, grasp it, memorize the color, and exit.
2. **Guarded Corridor (SubTask 02)**: Navigate a curved corridor formed by 20cm high walls (with gaps) without colliding or exiting prematurely.
3. **Blocked Tunnel (SubTask 03)**: Navigate a straight corridor and physically push a 25x25x20cm obstacle out of the path.
4. **Data Core Delivery (SubTask 04)**: Arrive at a 3-way line junction, match the memorized ball color to the correct colored path (Red/Blue/Green), navigate to the end of that path, and drop the ball.

## 3. Physical Constraints & Dimensions
* **Maximum Size**: 250 mm x 250 mm (no height restriction).
* **Line Widths**: 30 mm (white non-reflective matte on black background).
* **Grid Cells**: 25 cm x 25 cm.
* **Corridor Width**: 30 cm between parallel walls.
* **Wall Height**: 20 cm.
* **Ball Size**: 2 cm radius (4 cm diameter), elevated on a 5 cm base.
* **Obstacle Size**: 25 cm (L) x 25 cm (W) x 20 cm (H).

## 4. Robot Behavior & Locomotion Requirements
* **Locomotion Mode**: Strictly active legged (quadruped, hexapod, etc.).
* **Wheels**: No active wheels allowed. Passive wheels (e.g., caster wheels) are permitted but not recommended for this design.
* **Autonomy**: Must operate 100% autonomously without external input after the initial start.
* **Starting Procedure**: Triggered by a single onboard switch.
* **Stability**: Must demonstrate stability and stand independently at the starting zone; failure results in disqualification.
* **Integrity**: Expansion is allowed, but the robot cannot split into multiple entities or leave parts behind.

## 5. Hardware Constraints
* **Power Supply**: Strict maximum of 24V DC. All power must be onboard (e.g., 3S LiPo). No external power supplies.
* **Components Allowed**: Pre-made microcontroller boards (Raspberry Pi, STM32) and sensor kits are allowed.
* **Components Prohibited**: Wireless communication modules (leads to disqualification), ready-made Lego kits, and off-the-shelf robotics kits.

## 6. Sensor Requirements
* **Color Detection**: Required to identify the ball in Task 01 and the sorting lines in Task 04.
* **Line Tracking**: Required for navigating the grid, corridors, and final sorting junction.
* **Wall Detection**: Required to follow walls and ignore gaps in Task 02.

## 7. Actuator & Mechanical Requirements
* **Legs**: Minimum 12 servos for a quadruped (3-DOF per leg) to enable omnidirectional walking and steering.
* **Manipulation**: Must physically grasp a 4cm diameter ball elevated at 5cm.
* **Pushing**: Must physically exert enough force to slide a 25x25x20cm block.

## 8. Failure Conditions & Penalties
* Exceeding 15-minute time limit.
* Damaging the arena.
* Deviating from a line and failing to return within 10 seconds (requires restart).
* Losing balance and falling.
* Wireless communication detected.

## 9. Implicit Technical Requirements
* **State Memory**: The robot must persist state data (the ball color) between SubTask 01 and SubTask 04.
* **Dynamic Gait Adjustment**: The robot must traverse a tight curved corridor (Rinner, Router), necessitating a gait capable of smooth continuous turning, not just straight walking and pivot turns.
* **Sensor Fusion**: In Task 02, where walls have gaps, line tracking must take precedence over wall-following to prevent the robot from turning into the gap.
