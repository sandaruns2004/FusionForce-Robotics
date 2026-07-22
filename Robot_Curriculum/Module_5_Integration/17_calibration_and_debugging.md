# Module 5: Calibration and Debugging

The code is written, the 3D parts are printed, and the wires are connected. Now comes the most dangerous part: turning it on.

**NEVER place the robot on the ground the first time you power it on.** If an angle is inverted, the robot will violently flip itself over, potentially breaking its 3D printed legs or burning out a servo.

## 1. The Calibration Stand

Before doing anything, you must build or buy a small stand that holds the robot's body in the air, allowing the legs to dangle freely without touching the ground.

## 2. Servo "Zero" Calibration

When you assemble the 3D printed leg, you push the servo horn (the plastic gear) onto the motor shaft. It is physically impossible to align the horn *perfectly* at exactly 90 degrees by hand. It will always be off by a few degrees.

If you don't fix this in software, your Inverse Kinematics math will be wrong, and the robot will limp.

### The Calibration Process
1.  Write a simple Arduino sketch on the STM32 that sets every single motor to exactly `90` degrees (center).
2.  Power on the robot on the stand. The legs will snap to their center positions.
3.  Use a protractor or a digital angle gauge. You will notice the Femur might be at 87 degrees instead of 90.
4.  In your STM32 code, create a constant "Offset" array for all 12 motors.
    *   `int offsets[12] = {0, +3, -2, ...};`
5.  When applying angles in your motor loop, always add the offset:
    *   `femurMotor.write(targetAngle + offsets[1]);`

## 3. Testing Kinematics (Air Walking)

With the robot still suspended on the stand, run your Raspberry Pi gait generation code.

1.  Watch the legs move in the air.
2.  Do they move in smooth, elliptical curves? (If they jerk violently, your Bezier curve math is wrong, or your Serial UART connection is dropping packets).
3.  Hold a piece of cardboard under one foot. As the foot "steps", trace its path with a marker. Does it draw a straight line backward during the "Stance" phase? (If it draws an arc, your IK math is wrong).

## 4. Ground Testing (The Final Boss)

Once it walks perfectly in the air:
1.  Lower the robot to the ground.
2.  Have your hand hovering over the main LiPo battery disconnect switch.
3.  Start the Creep gait (static walking). 
4.  Observe the PID loop. If the robot jitters or vibrates while standing, you need to lower the `Kp` and `Kd` values in your balancing code.

### A Note on Thermal Overload
RC Servos get very hot when lifting a robot. If a servo gets too hot to comfortably touch with your finger, turn the robot off immediately and let it cool down, or the motor coils will melt.

## 📺 Recommended Viewing
*   Search YouTube for: `"Spot Micro calibration and first steps"`
*   Search YouTube for: `"How to tune quadruped robot legs"`
