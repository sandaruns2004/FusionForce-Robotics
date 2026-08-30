# 04 Electronics Mounting

## Objective
Secure the "Brain" and "Spine" to the chassis to withstand continuous walking vibrations.

## Component Stack
1. **Bottom Deck**: 3S LiPo Battery (Velcro strap).
2. **Middle Deck**: 
   - 10A UBEC (Zip-tied or VHB tape).
   - 5A Buck Converter.
   - PCA9685 PWM Driver Board (Screwed into standoffs).
3. **Top Deck**: 
   - Raspberry Pi 4B (Screwed into M2.5 standoffs).
   - STM32 Black Pill (Mounted on a custom PCB or perfboard, screwed into standoffs).

## Sensor Placement (CRITICAL)
- **MPU6050 (IMU)**: Must be mounted exactly in the center of the robot, perfectly flat. If it is tilted, the postural PID controller will constantly fight to "level" a robot that is already level.
- **VL53L0X (ToF Sensors)**: Mount one pointing dead ahead, one exactly 90 degrees Left, and one exactly 90 degrees Right. Ensure no chassis parts block their Field of View (FoV).
- **Pi Camera Module 3**: Mount at the front of the top deck, angled exactly 45 degrees downwards to see the floor immediately in front of the robot.

## Vibration Mitigation
Use nylon standoffs and rubber washers when mounting the Raspberry Pi and STM32. High-frequency vibrations from the servos can loosen screws or damage the IMU readings.
