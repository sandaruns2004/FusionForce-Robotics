# REQUIREMENTS

## 1. Primary Source of Truth
This document outlines the strict technical requirements for the robot, derived exclusively from the authoritative source: `Task/tasks_circuit_v2.md`.

## 2. Competition Objectives
The BREACH PROTOCOL competition requires a fully autonomous robot to traverse a 200cm × 200cm arena. The overall mission is broken into four continuous subtasks:
1. **Grid Search (SubTask 01)**: Navigate a 4×4 line grid, locate a ball, identify its colour, grasp it, memorise the colour, and exit.
2. **Guarded Corridor (SubTask 02)**: Navigate a curved corridor formed by 20cm high walls (with gaps) without colliding or exiting prematurely.
3. **Blocked Tunnel (SubTask 03)**: Navigate a straight corridor and physically push a 25×25×20cm obstacle out of the path.
4. **Data Core Delivery (SubTask 04)**: Arrive at a 3-way line junction, match the memorised ball colour to the correct coloured path (Red/Blue/Green), navigate to the end of that path, and drop the ball.

## 3. Physical Constraints & Dimensions
* **Maximum Size**: 250 mm × 250 mm (no height restriction).
* **Line Widths**: 30 mm (white non-reflective matte on black background).
* **Grid Cells**: 25 cm × 25 cm.
* **Corridor Width**: 30 cm between parallel walls.
* **Wall Height**: 20 cm.
* **Ball Size**: 2 cm radius (4 cm diameter), elevated on a 5 cm base.
* **Obstacle Size**: 25 cm (L) × 25 cm (W) × 20 cm (H).

## 4. Robot Behaviour & Locomotion Requirements
* **Locomotion Mode**: Strictly active legged (quadruped).
* **Wheels**: No active wheels allowed.
* **Autonomy**: Must operate 100% autonomously without external input after start.
* **Starting Procedure**: Single onboard switch.
* **Stability**: Must stand independently at start; failure = disqualification.

## 5. Hardware Constraints
* **Power Supply**: Max 24V DC, onboard only (2S LiPo = 8.4V max — compliant).
* **Components Allowed**: Pre-made MCU boards (STM32) and sensor kits are allowed.
* **Components Prohibited**: Wireless communication modules, Lego kits, off-the-shelf robot kits.

## 6. Sensor Requirements

| Requirement | Implementation |
|-------------|----------------|
| **Colour Detection** | TCS34725 RGBC I2C sensor on gripper arm tip. MODE A (arm 0°) reads ball colour at pedestal; MODE B (arm −70°) reads floor colour zones at junction. Replaces camera HSV detection. |
| **Line Tracking** | 8-channel TCRT5000 digital IR reflectance array (GPIO PA0–PA7). Weighted centroid algorithm for steering; ≥6 sensor activation for intersection detection. Replaces camera contour extraction. |
| **Wall Detection** | 3× VL53L0X Time-of-Flight sensors (I2C2, XSHUT-addressed). Front (obstacle), Left+Right (wall centering). Unchanged from original design. |
| **Body Stabilisation** | MPU6050 6-axis IMU (I2C1). Complementary filter for pitch/roll; postural PID for body levelling during crawl gait. Unchanged. |

## 7. Actuator & Mechanical Requirements
* **Legs**: 12× MG90S servos (3-DOF per leg × 4 legs). Operated at 6V, crawl gait only.
* **Manipulation**: 2× MG90S (arm pitch + gripper) for grasping a 4cm ball at 5cm pedestal height.
* **Storage**: 1× MG90S (gate) for ball compartment.
* **Pushing**: Passive front PETG bumper — no additional actuator.

## 8. Failure Conditions & Penalties
* Exceeding 15-minute time limit.
* Damaging the arena.
* Deviating from a line and failing to return within 10 seconds (restart required).
* Losing balance and falling.
* Wireless communication detected (disqualification).

## 9. Implicit Technical Requirements
* **State Memory**: `stored_ball_color` must persist between SubTask 01 and SubTask 04 — written to STM32 Flash (EEPROM emulation) to survive competition restarts.
* **Dynamic Gait Adjustment**: Continuous turning capability via gait step-length asymmetry; not just pivot turns.
* **Temporal Filtering**: Intersection detection requires ≥3 consecutive high-sensor-count readings to prevent false triggers on curves.
* **Non-Blocking Colour Reads**: TCS34725 50ms integration must not block the 20ms control loop — requires asynchronous polling strategy.
