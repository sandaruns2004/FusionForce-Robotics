# 06 Calibration & Zeroing

## Objective
Mechanically align all servos to precisely 90 degrees. This is the single most important step in the entire mechanical build.

## Why is this necessary?
The Inverse Kinematics (IK) math on the STM32 assumes that when it commands an angle of 90 degrees, the physical leg segment is perfectly perpendicular to the body. If the servo horn was attached slightly off-center, a 90-degree command might actually result in an 82-degree physical angle. The robot will walk crooked, limp, or crash.

## The Zeroing Process
1. **DO NOT attach any servo horns or legs yet.**
2. Wire all 14 servos to the PCA9685.
3. Power on the Pi and STM32.
4. Write a simple C script on the STM32 (or a Python script on the Pi communicating via I2C) that sets the PWM signal for all 16 channels to exactly **1.5ms (1500µs)**, which is the 90-degree center point for standard servos.
5. While the servos are powered and actively holding the 90-degree position, carefully push the splined servo horns onto the servos so that the legs form perfect right angles according to your CAD design.
6. Screw the horns in place.

## Software Trimming
Because servo splines have teeth, you rarely get it *perfectly* straight. The horn might be 2 degrees off.
You must create a `calibration.h` file in the STM32 firmware that holds a small offset array:
```c
int servo_offsets[12] = {2, -1, 0, 4, ...};
```
Add these offsets to your IK angles before sending them to the PCA9685 to achieve absolute perfection.
