# Quadruped Robot Hardware Plan (Raspberry Pi 4B + STM32)

## System Architecture

-   Raspberry Pi 4B: Vision, AI, navigation, high-level decisions
-   STM32 Black Pill (STM32F411CEU6): Gait generation, servo control,
    sensors
-   Communication: UART

## Controllers

-   Raspberry Pi 4B (4 GB/8 GB)
-   STM32 Black Pill
-   32--64 GB microSD

## Actuators

-   12 × DS3218/DS3225 servos (legs)
-   1 × MG996R/DS3218 (gripper)
-   1 × SG90 (optional wrist)

## Servo Driver

-   PCA9685 16-channel

## Vision

-   Raspberry Pi Camera Module 3

## Sensors

-   3 × VL53L0X ToF
-   MPU6050 IMU

## Display

-   0.96-inch SSD1306 OLED

## Power

-   3S LiPo battery
-   5V 10A UBEC (servos)
-   5V 5A regulator (Pi)
-   Common ground

## Optional

-   TCA9548A I2C multiplexer
-   Cooling fan
-   Buzzer
-   Status LEDs
-   Battery monitor

## Task Distribution

### Raspberry Pi

-   OpenCV
-   Line following
-   Ball detection
-   Color recognition
-   Path planning
-   Sends commands: WALK_FORWARD, TURN_LEFT, PICK_BALL, PUSH_BLOCK,
    DROP_BALL

### STM32

-   Inverse kinematics
-   Walking gait
-   Servo control
-   Read MPU6050
-   Read ToF sensors
-   Balance correction

## Bill of Materials

  Component                  Qty
  ------------------------ -----
  Raspberry Pi 4B              1
  STM32 Black Pill             1
  Camera Module 3              1
  PCA9685                      1
  Leg Servos                  12
  Gripper Servo                1
  Wrist Servo (Optional)       1
  VL53L0X                      3
  MPU6050                      1
  SSD1306 OLED                 1
  TCA9548A (Recommended)       1
  3S LiPo                      1
  5V 10A UBEC                  1
  5V 5A Regulator              1
