# RISK REGISTER

## FMEA & Risk Analysis

| Risk | Domain | Probability | Impact | Detection | Mitigation | Recovery | Owner | Status |
|---|---|---|---|---|---|---|---|---|
| **Servo Brownout** | Power | High | Critical | STM32 / Pi random reboots when walking. | Use a dedicated 10A UBEC for servos. Isolate logic power (Pi/STM32) on a separate 5V regulator. Install 1000uF capacitors on servo rails. | Reboot robot, ensure battery is fully charged. | Team B | In Progress |
| **Ground Loop Issue** | Power | Med | High | Erroneous sensor readings, unstable UART communication. | Verify common ground between battery, UBEC, Pi, and STM32 using a star grounding topology. | Physically rewire grounding. | Team B | Open |
| **Odometry Drift in Grid** | Vision | High | High | Robot deviates from the line in Task 01 and misses intersection. | Do not rely on dead reckoning. Use OpenCV contour extraction for continuous closed-loop line following. | Reverse and re-acquire line. | Team A | Open |
| **False Positive Color Detection** | Vision | Med | High | Robot drops the ball in the wrong color segment in Task 04. | Use strict HSV ranges. Add morphological transformations (erode/dilate) and Hough Circle validation. | N/A (Competition penalty). | Team A | Open |
| **Gait Instability / Falling** | Mech | Med | High | Robot tips over while raising a leg. | Ensure static crawl gait keeps Center of Gravity within the support polygon. | Fall detected by IMU; execute self-righting routine (complex) or require human restart. | Team C | Open |
| **Gripper Malfunction** | Mech | Low | High | Ball slips out of gripper during transit. | Design claw with rubberized grip pads. Servo maintains holding torque. | Restart task 01. | Team C | Open |
| **UART Comm Timeout** | Comms | Low | Critical | Pi freezes, STM32 continues executing last command indefinitely (crash). | Implement a heartbeat protocol. If STM32 receives no UART packet for 500ms, immediately command all motors to stop. | Resume normal operation when UART packet received. | Team B | Open |
| **ToF Sensor Interference** | Sensors | Low | Med | Multiple VL53L0X sensors conflict on same I2C address. | Use a TCA9548A I2C Multiplexer or manually change I2C addresses on boot using their XSHUT pins. | Reboot sensor bus. | Team B | Open |
| **Block Pushing Failure** | Mech | Low | Med | Robot lacks torque to push the Task 03 obstacle. | Design integrated flat bumper to maximize contact area. Use high-torque DS3218 servos on the legs. | Reverse and retry pushing with momentum. | Team C | Open |
| **Missing Wall Navigation Error** | Vision | Med | High | Robot exits the curved corridor prematurely through the missing wall gap. | Combine visual tracking (look ahead) with ToF side distance. If ToF reads infinity, trust the line follower strictly. | Competition restart required. | Team A | Open |
