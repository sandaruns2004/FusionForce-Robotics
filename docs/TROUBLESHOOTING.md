# TROUBLESHOOTING GUIDE

## 1. System Will Not Boot
- **Symptoms**: Pi has no lights; STM32 has no lights.
- **Causes**: Battery is dead, Kill switch is OFF, or UBEC/Buck is wired backward (catastrophic failure).
- **Fix**: Measure battery voltage with a multimeter. Verify 5V output on the Buck converter BEFORE plugging it into the Pi.

## 2. Servos Twitch Erratically
- **Symptoms**: As soon as the code runs, servos jitter randomly or violently swing to extremes.
- **Causes**: Ground loop! The PCA9685 and the STM32 do not share a common ground. 
- **Fix**: Ensure a thick, solid wire connects the Ground of the Pi, STM32, UBEC, and PCA9685 logic.

## 3. Robot Walks Crooked
- **Symptoms**: Commanded `Vx = 0.2`, `Wz = 0`, but the robot veers right.
- **Causes**: 
  1. Servos were not mechanically zeroed at 90 degrees before attaching the legs.
  2. The Center of Gravity is heavily biased to one side.
- **Fix**: Run the `servo_calibration.c` script to command all servos to exactly 90 degrees. Unscrew and realign any legs that are visibly off-center.

## 4. Raspberry Pi UART "Permission Denied"
- **Symptoms**: Python throws a `PermissionError` when opening `/dev/ttyS0` or `/dev/ttyAMA0`.
- **Causes**: The Linux serial console is occupying the UART port.
- **Fix**: Run `sudo raspi-config`, navigate to `Interface Options` -> `Serial Port`. Select **NO** for "Would you like a login shell to be accessible over serial?" and **YES** for "Would you like the serial port hardware to be enabled?". Reboot.

## 5. False Color Detection in Task 04
- **Symptoms**: Robot consistently misidentifies the green zone as blue.
- **Causes**: Changes in ambient room lighting drastically shift HSV values.
- **Fix**: Never hardcode HSV values for a competition. Run the `color_calibration.py` script on the actual competition floor with current lighting and update the `camera_config.json` thresholds.
