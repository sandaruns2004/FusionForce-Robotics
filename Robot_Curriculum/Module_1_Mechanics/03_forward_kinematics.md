# Module 1: Forward Kinematics

Kinematics is the branch of physics that describes the motion of points, bodies, and systems without considering the forces that cause them to move. In robotics, it's all about geometry.

## What is Forward Kinematics (FK)?

**Forward Kinematics** asks the question:
> *"If I know the exact angle of every motor in my robot leg, what are the (X, Y, Z) coordinates of the foot in 3D space?"*

Imagine you power on the robot, and the STM32 reads the current angles from the servos:
*   Coxa Motor (Hip Roll) = 0 degrees
*   Femur Motor (Hip Pitch) = 45 degrees
*   Tibia Motor (Knee Pitch) = -45 degrees

Where is the foot touching the ground relative to the hip? Forward kinematics uses trigonometry to calculate that position.

## The Math (2D Example)

To keep it simple, let's look at a 2D leg (just the Femur and Tibia, ignoring the Coxa roll).

Let:
*   $L1$ = Length of the Femur
*   $L2$ = Length of the Tibia
*   $\theta_1$ (Theta 1) = Angle of the Femur motor
*   $\theta_2$ (Theta 2) = Angle of the Tibia motor (relative to the Femur)

Using basic trigonometry (SOH CAH TOA):

**X Coordinate (Forward/Backward):**
`X = L1 * cos(θ1) + L2 * cos(θ1 + θ2)`

**Y Coordinate (Up/Down):**
`Y = L1 * sin(θ1) + L2 * sin(θ1 + θ2)`

If you plug the lengths of your 3D printed parts and the current motor angles into these equations, you will get the exact X and Y position of the foot.

## Why Do We Need FK?

You might think: *"I just want the robot to walk, why do I care where the foot currently is?"*

1.  **State Estimation**: If the robot is standing on a slope, the IMU tells you the body is tilted. By using Forward Kinematics, the Pi can calculate exactly where all 4 feet are relative to the tilted body to figure out the robot's overall posture.
2.  **Collision Detection**: Knowing where the feet are prevents the robot from accidentally crossing its legs and tripping over itself.

## 📺 Recommended Viewing
*   Search YouTube for: `"Forward Kinematics 2D robot arm Python"` - The math for a robot arm is exactly the same as the math for a robot leg.
