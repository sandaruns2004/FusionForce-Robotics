// =============================================================================
// PCA9685 Single Servo Test - Arduino Uno
// =============================================================================
//
// WIRING:
//   Arduino Uno 5V       ->  PCA9685 VCC
//   Arduino Uno GND      ->  PCA9685 GND
//   Arduino Uno A4 (SDA) ->  PCA9685 SDA
//   Arduino Uno A5 (SCL) ->  PCA9685 SCL
//
// SERVO POWER:
//   Connect an external 5V-6V power supply to the PCA9685 V+ and GND terminals.
//
// SERVO CONNECTION:
//   Plug your servo into PCA9685 CHANNEL 0.
// =============================================================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Initialize the PCA9685 driver with the default I2C address (0x40)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo pulse lengths (adjust if your servo doesn't reach 0 or 180 degrees)
#define SERVOMIN  150 // Pulse length for 0 degrees
#define SERVOMAX  600 // Pulse length for 180 degrees

#define TEST_CHANNEL 0 // The channel we will test

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Single Servo Test on Channel 0");

  pwm.begin();
  
  // Standard analog servos run at 50 Hz updates
  pwm.setPWMFreq(50);
  
  delay(10);
}

// Helper function to map degrees (0-180) to PCA9685 pulse counts
int angleToPulse(int angle) {
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

void loop() {
  Serial.println("Moving to 0 degrees...");
  pwm.setPWM(TEST_CHANNEL, 0, angleToPulse(0));
  delay(1500); 

  Serial.println("Moving to 90 degrees...");
  pwm.setPWM(TEST_CHANNEL, 0, angleToPulse(90));
  delay(1500); 

  Serial.println("Moving to 180 degrees...");
  pwm.setPWM(TEST_CHANNEL, 0, angleToPulse(180));
  delay(1500); 

  Serial.println("Moving back to 90 degrees...");
  pwm.setPWM(TEST_CHANNEL, 0, angleToPulse(90));
  delay(1500); 
}
