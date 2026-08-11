# 03 Gripper & Bumper

## Objective
Build the specific manipulators required to solve SubTask 01 (Ball Retrieval) and SubTask 03 (Obstacle Pushing).

## The Front Bumper (Task 03)
The robot must push a 25x25x20cm slideable block. 
- **Design**: A flat, 10cm wide, 5cm tall rigid PETG plate.
- **Mounting**: Attach it firmly to the front edge of the bottom chassis deck.
- **Why**: You cannot push a heavy block using alternating leg movements; the legs will tangle. A flat bumper allows the robot to simply walk forward, transferring force directly from the chassis into the block.

## The 2-DOF Micro-Gripper (Task 01)
The robot must grasp a 4cm diameter ball elevated at 5cm.
- **Hardware**: 2x SG90 or MG90S micro-servos.
- **Pitch Servo**: Mounted to the chassis. Rotates the entire claw assembly 90 degrees up (storage) and down (deployment).
- **Grip Servo**: Mounted on the pitch bracket. Opens and closes two interlocking 3D-printed fingers.
- **Fingers**: Must be lined with rubber bands or foam tape to provide grip friction on the plastic ball.

## Assembly Steps
1. Mount the bumper to the lowest front point.
2. Mount the Pitch servo slightly above the bumper.
3. Attach the Grip servo and fingers.
4. Route the thin SG90 wires carefully so they do not snag when the legs swing.
