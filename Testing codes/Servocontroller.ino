#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Initialize the PCA9685 driver on the default I2C address (0x40)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Depending on your servo make, the minimum and maximum pulse length count may vary.
// These are typical values for standard 180-degree servos (out of 4096)
#define SERVOMIN  150 // This is the 'minimum' pulse length count (approx 0 degrees)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (approx 180 degrees)
#define SERVO_NUM 0   // The channel your servo is plugged into

void setup() {
  Serial.begin(9600);
  Serial.println("PCA9685 Servo Test");

  pwm.begin();
  // Standard servos operate at 50 Hz (updates 50 times per second)
  pwm.setPWMFreq(50);  
  delay(10);
}

void loop() {
  // Move from 0 degrees to 180 degrees
  Serial.println("Moving to 180 degrees");
  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
    pwm.setPWM(SERVO_NUM, 0, pulselen);
    delay(5); // Adjust delay to change speed
  }

  delay(1000);

  // Move from 180 degrees back to 0 degrees
  Serial.println("Moving to 0 degrees");
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
    pwm.setPWM(SERVO_NUM, 0, pulselen);
    delay(5);
  }

  delay(1000);
}