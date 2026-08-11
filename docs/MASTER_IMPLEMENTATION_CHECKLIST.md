# MASTER IMPLEMENTATION CHECKLIST

## 1. Hardware & Electronics
- `[ ]` 3S LiPo battery tested and fully charged.
- `[ ]` 10A UBEC output voltage verified at exactly 5.0V - 5.2V using multimeter.
- `[ ]` 5A Buck Converter (for Raspberry Pi) voltage verified.
- `[ ]` STM32 Black Pill powers on via 3.3V pin or USB.
- `[ ]` Raspberry Pi 4B boots successfully from SD card.
- `[ ]` Common Ground (GND) wire physically connected between Battery, UBEC, Pi, and STM32.
- `[ ]` PCA9685 I2C driver board connected to STM32.
- `[ ]` All 12 DS3218 leg servos connected to PCA9685.
- `[ ]` 2 SG90 gripper servos connected to PCA9685.
- `[ ]` 3x VL53L0X ToF sensors connected to STM32 I2C (using TCA9548A if address conflicts exist).
- `[ ]` MPU6050 IMU connected to STM32 I2C.
- `[ ]` Raspberry Pi Camera Module 3 connected via ribbon cable.
- `[ ]` Hardware kill switch installed on battery main positive line.

## 2. STM32 Firmware (Low-Level)
- `[ ]` STM32CubeMX project created with correct clock tree (84MHz).
- `[ ]` GPIO pins for UART and I2C configured.
- `[ ]` PCA9685 I2C communication verified (servos sweep).
- `[ ]` MPU6050 I2C reading verified (Roll/Pitch data stable).
- `[ ]` VL53L0X ToF readings verified (Distance accurate).
- `[ ]` Inverse Kinematics (IK) math implemented and unit tested on MCU.
- `[ ]` Crawl Gait generator implemented.
- `[ ]` Trot Gait generator implemented.
- `[ ]` Hardware UART Receive Interrupt (RX_IT) working.
- `[ ]` UART Protocol Parser implemented (Header, Checksum).
- `[ ]` Postural PID controller implemented to balance body pitch/roll based on IMU.
- `[ ]` Safety timeout: If UART command from Pi stops for >500ms, halt all motors.

## 3. Raspberry Pi (High-Level)
- `[ ]` Ubuntu/Raspberry Pi OS installed.
- `[ ]` SSH enabled, static IP configured.
- `[ ]` Pi Camera verified working (`libcamera-hello`).
- `[ ]` Python virtual environment created.
- `[ ]` OpenCV (`cv2`), `pyserial`, and `numpy` installed.
- `[ ]` Camera calibration script run (distortion matrix generated).
- `[ ]` UART port (`/dev/ttyS0` or `/dev/ttyAMA0`) configured without Linux console interference.
- `[ ]` High-Level Finite State Machine (HFSM) skeleton implemented.
- `[ ]` HSV Threshold tuning script created for Red, Blue, Green balls/zones.
- `[ ]` Line-following contour extraction algorithm implemented.

## 4. Mechanical & Assembly
- `[ ]` Chassis baseplate 3D printed.
- `[ ]` 12x Servo brackets 3D printed.
- `[ ]` 4x Legs assembled.
- `[ ]` Actuator angle limits mechanically verified (no self-collision).
- `[ ]` 2-DOF Front Gripper 3D printed and assembled.
- `[ ]` Integrated Bumper Plate attached to front chassis.
- `[ ]` Components mounted securely (Pi, STM32, battery) to achieve symmetrical Center of Gravity.
- `[ ]` Cables routed neatly to prevent snagging during leg swing.

## 5. Integration & Testing
- `[ ]` **Pi ↔ STM32 Comms**: Pi successfully sends `MOVE_FWD` command; STM32 correctly initiates gait.
- `[ ]` **Sensor Feedback**: STM32 successfully transmits ToF distance back to Pi.
- `[ ]` **Task 01**: Pi detects line, navigates grid, detects ball color, stops, STM32 triggers gripper to grab ball.
- `[ ]` **Task 02**: Pi uses ToF sensors and vision to navigate curved corridor with missing walls.
- `[ ]` **Task 03**: Robot walks straight and physically pushes obstacle using front bumper.
- `[ ]` **Task 04**: Pi recalls memorized color, detects matching color zone, and drops the ball.
- `[ ]` **Full Circuit**: Robot completes entire Task 01 to 04 sequence autonomously under 15 minutes.
- `[ ]` **Failure Recovery**: Robot handles temporary loss of line tracking gracefully.
