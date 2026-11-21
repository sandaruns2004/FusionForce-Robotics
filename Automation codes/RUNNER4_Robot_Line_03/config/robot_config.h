#pragma once
// =============================================================================
// config/robot_config.h — RUNNER4 Centralized Hardware & Tuning Configuration
//
// All pin assignments, I2C addresses, servo limits, and system constants are
// defined here.  Include this FIRST in RUNNER4_Robot_Line_03.ino.
// =============================================================================

// -- WiFi Access Point ----------------------------------------------------------
#define AP_SSID  "RUNNER4-Config"
#define AP_PASS  "runner4robot"

// -- I2C Bus 0: PCA9685 (Servos) + 3x VL53L0X (ToF) --------------------------
#define I2C_SDA             11
#define I2C_SCL             12
#define I2C_HZ              400000
// VL53L0X XSHUT pins — pull LOW to hold sensor in reset, HIGH to release.
// All 3 sensors boot at 0x29; tofBegin() enumerates them one-by-one.
#define TOF_FRONT_XSHUT_PIN 13      // Front ToF  -> runtime addr 0x30
#define TOF_RIGHT_XSHUT_PIN 14      // Right ToF  -> runtime addr 0x31
#define TOF_LEFT_XSHUT_PIN  21      // Left ToF   -> runtime addr 0x32

// -- I2C Bus 1: TCS34725 (Colour Sensor) ---------------------------------------
#define COLOR_I2C_SDA       41
#define COLOR_I2C_SCL       42
#define COLOR_LED_PIN       15      // White LED control on TCS34725 module (HIGH = on)

// -- Status RGB LED (common-cathode, HIGH = ON) --------------------------------
#define RGB_RED_PIN    16
#define RGB_GREEN_PIN  17
#define RGB_BLUE_PIN   18

// -- PCA9685 Servo PWM Driver --------------------------------------------------
#define SERVOMIN     150     // Pulse count for 0°   (50 Hz, 12-bit PCA9685)
#define SERVOMAX     600     // Pulse count for 180°
#define SERVO_FREQ    50     // Hz

// -- Leg Geometry (mm) ---------------------------------------------------------
#define L1_MM   30.0f    // Coxa  — hip pivot to femur pivot
#define L2_MM   60.0f    // Femur — femur pivot to tibia pivot
#define L3_MM   80.0f    // Tibia — tibia pivot to foot tip

// -- PCA9685 Channel Map (Leg order: 0=FL  1=FR  2=BL  3=BR) ------------------
#define FL_HIP     0
#define FL_FEMUR   1
#define FL_TIBIA   15     // Remapped from CH2 to CH15 (PCA9685 CH2 hardware output failure; move servo plug to CH 15)
#define FR_HIP     3
#define FR_FEMUR   4
#define FR_TIBIA   5
#define BL_HIP    10
#define BL_FEMUR  11
#define BL_TIBIA  14
#define BR_HIP     6
#define BR_FEMUR   8
#define BR_TIBIA   9

// -- Gait Defaults (runtime vars in .ino shadow these; kept here for reference) -
#define WALK_HIP_DELTA_DEFAULT    15      // Reduced from 30 for stable, tight stride
#define WALK_FEMUR_DELTA_DEFAULT  20      // Femur lift set to 20 deg
#define TURN_HIP_DELTA_DEFAULT    15
#define TURN_FEMUR_DELTA_DEFAULT  20      // Femur lift set to 20 deg

// -- Background Service Rate Limits --------------------------------------------
#define WEB_SERVE_INTERVAL_MS    20    // Serve HTTP at most every 20 ms
#define LED_UPDATE_INTERVAL_MS   50    // RGB LED update at most every 50 ms

// -- Competition Start Button --------------------------------------------------
#define COMP_START_PIN   0     // GPIO 0 = BOOT button (active LOW INPUT_PULLUP)
#define COMP_DEBOUNCE_MS 200

// -- Arm Servo Pins (direct LEDC PWM — GPIO, NOT on PCA9685) ------------------
//   Lift servo: GPIO 47  (LEDC ch 6 or pin attach) — raises / lowers the arm
//   Grip servo: GPIO 44  (LEDC ch 7 or pin attach) — opens / closes the gripper
//   (Note: GPIO 21 is reserved for VL53L0X Left XSHUT. GPIO 43 is alternate)
#define ARM_LIFT_PIN   47
#define ARM_GRIP_PIN   44