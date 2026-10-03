// =============================================================================
// PCA9685 Back-Left Leg Test (Channels 6, 7, 8)
// =============================================================================
//
// WIRING:
//   Arduino Uno 5V       ->  PCA9685 VCC
//   Arduino Uno GND      ->  PCA9685 GND
//   Arduino Uno A4 (SDA) ->  PCA9685 SDA
//   Arduino Uno A5 (SCL) ->  PCA9685 SCL
//
// SERVO CONNECTIONS:
//   Channel 6 -> Back-Left Hip
//   Channel 7 -> Back-Left Femur
//   Channel 8 -> Back-Left Tibia
// =============================================================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150
#define SERVOMAX  600

#define BL_HIP    6
#define BL_FEMUR  7
#define BL_TIBIA  8

// Home angles based on your 12DOF Controller
#define INIT_BL_HIP   135
#define INIT_BL_FEMUR  80
#define INIT_BL_TIBIA   0

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Back-Left Leg Test (Channels 6, 7, 8)");

  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);

  Serial.println("Moving to Home Positions...");
  setServo(BL_HIP, INIT_BL_HIP);
  setServo(BL_FEMUR, INIT_BL_FEMUR);
  setServo(BL_TIBIA, INIT_BL_TIBIA);
  delay(2000);
}

int angleToPulse(int angle) {
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

void setServo(uint8_t channel, int angle) {
  pwm.setPWM(channel, 0, angleToPulse(angle));
}

void loop() {
  Serial.println("\n--- Starting Sweep Test ---");

  // 1. Test Hip
  Serial.println("Testing Hip (CH 6): 135 -> 90 -> 135");
  setServo(BL_HIP, 90);
  delay(1000);
  setServo(BL_HIP, INIT_BL_HIP);
  delay(1000);

  // 2. Test Femur
  Serial.println("Testing Femur (CH 7): 80 -> 120 -> 80");
  setServo(BL_FEMUR, 120);
  delay(1000);
  setServo(BL_FEMUR, INIT_BL_FEMUR);
  delay(1000);

  // 3. Test Tibia
  Serial.println("Testing Tibia (CH 8): 0 -> 45 -> 0");
  setServo(BL_TIBIA, 45);
  delay(1000);
  setServo(BL_TIBIA, INIT_BL_TIBIA);
  delay(1000);

  Serial.println("Waiting 2 seconds before next cycle...");
  delay(2000);
}
