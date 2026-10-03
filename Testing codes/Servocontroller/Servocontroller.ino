// =============================================================================
// PCA9685 Servo Controller - Arduino Uno 
// =============================================================================
//
// ARDUINO UNO → PCA9685 WIRING:
//   Arduino Uno 5V       ->  PCA9685 VCC
//   Arduino Uno GND      ->  PCA9685 GND
//   Arduino Uno A4 (SDA) ->  PCA9685 SDA
//   Arduino Uno A5 (SCL) ->  PCA9685 SCL
//
// DO NOT power servos from the Uno's 5V pin! Connect an external 5V-6V 
// power supply to the PCA9685 V+ and GND terminals.
// =============================================================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // Pulse length for 0 degrees
#define SERVOMAX  600 // Pulse length for 180 degrees

#define SERVO_CH0 0   // Channel 0
#define SERVO_CH2 2   // Channel 2

// Variables to hold the pulse lengths for our specific angles
int pulse180, pulse150, pulse100;

void setup() {
  Serial.begin(9600);
  
  pwm.begin();
  pwm.setPWMFreq(50);  
  delay(10);

  // Automatically calculate the pulse lengths for 180, 150, and 100 degrees
  pulse180 = map(180, 0, 180, SERVOMIN, SERVOMAX);
  pulse150 = map(150, 0, 180, SERVOMIN, SERVOMAX);
  pulse100 = map(100, 0, 180, SERVOMIN, SERVOMAX);

  // --- STAGGERED STARTUP ---
  Serial.println("Moving Ch 0 to 180...");
  pwm.setPWM(SERVO_CH0, 0, pulse180);
  delay(500); 

  Serial.println("Moving Ch 2 to 100...");
  pwm.setPWM(SERVO_CH2, 0, pulse100);
  delay(1000); 
}

void loop() {
  // 1. Sweep Channel 0 from 180 DOWN to 150
  Serial.println("Ch 0: 180 -> 150");
  for (uint16_t p = pulse180; p >= pulse150; p--) {
    pwm.setPWM(SERVO_CH0, 0, p);
    delay(5);
  }
  delay(500); 

  // 2. Sweep Channel 2 from 100 UP to 150
  Serial.println("Ch 2: 100 -> 150");
  for (uint16_t p = pulse100; p <= pulse150; p++) {
    pwm.setPWM(SERVO_CH2, 0, p);
    delay(5);
  }
  delay(1000); 

  // --- RETURN SWEEP ---
  // We sweep them back to their starting positions slowly so they don't 
  // violently snap back when the loop repeats.
  
  Serial.println("Resetting positions...");
  
  // Sweep Channel 0 back up to 180
  for (uint16_t p = pulse150; p <= pulse180; p++) {
    pwm.setPWM(SERVO_CH0, 0, p);
    delay(5);
  }
  
  // Sweep Channel 2 back down to 100
  for (uint16_t p = pulse150; p >= pulse100; p--) {
    pwm.setPWM(SERVO_CH2, 0, p);
    delay(5);
  }
  
  delay(1000); // Wait 1 second before starting the whole sequence over
}