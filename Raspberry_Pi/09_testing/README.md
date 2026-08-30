# 09 Testing Methodologies

## Objective
Safely validate the complex logic without destroying the physical quadruped hardware.

## Dry-Run Testing (Pi Only)
Before connecting to the STM32, test the Pi script using a mocked Serial class that returns fake ToF distances. 
- Use **Software-in-the-Loop (SIL)** by feeding a pre-recorded video of the competition floor into your OpenCV script instead of the live camera.
- Verify that the FSM transitions states correctly based on the fake sensor inputs and the video feed.

## Tethered Field Testing
1. Connect the Pi and STM32. 
2. Place the robot on a "Suspension Rig" (a wooden/PVC stand) so its legs are suspended off the ground.
3. Manually slide a piece of paper with a black line under the camera.
4. Ensure that the line movement accurately alters the PWM servo angles on the legs, proving the entire `Pi -> UART -> STM32 -> IK -> Servo` chain works perfectly.

## Untethered Track Runs
Begin full track runs on the floor. 
- Start by testing isolated tasks (e.g., just the Task 02 Corridor).
- Validate the Wall-Following PD controller before attempting a full run from Start to Finish.
- If the robot walks into a wall, suspend the robot, wave your hand in front of the ToF sensor, and verify the legs change their gait to avoid it.
