// =============================================================================
// FusionForce RUNNER4 - 12 DOF Quadruped Robot
// PCA9685 Servo Controller - Full 12 Motor Implementation
// =============================================================================
//
// JOINT NAMING CONVENTION:
//   001 = Hip   (rotates leg forward / backward — mounted on the body frame)
//   002 = Femur (upper leg joint — lifts the leg up / down)
//   003 = Tibia (lower leg / knee joint — extends or retracts the foot)
//
// ─────────────────────────────────────────────────────────────────────────────
// STEP 1 — ARDUINO → PCA9685 I2C WIRING:
//   Arduino Pin  │  PCA9685 Pin  │  Wire Colour (typical)
//   ─────────────┼───────────────┼──────────────────────
//   5V           │  VCC          │  Red
//   GND          │  GND          │  Black
//   A4 (SDA)     │  SDA          │  Blue / White
//   A5 (SCL)     │  SCL          │  Yellow
//   (Mega: use pin 20=SDA, 21=SCL)
//
// STEP 2 — EXTERNAL POWER → PCA9685 V+ RAIL:
//   Power Supply +  →  PCA9685 V+  terminal  (big screw block)
//   Power Supply –  →  PCA9685 GND terminal  (big screw block)
//   Recommended: 6V DC, min 3A SMPS/BEC  (DO NOT use Arduino 5V for servos!)
//
// STEP 3 — SERVO → PCA9685 CHANNEL (plug 3-pin servo connector per channel):
//   Servo Wire     │  PCA9685 Pin Column
//   ───────────────┼──────────────────────────────────────────────────
//   Signal (Orange/Yellow) │  PWM  — innermost pin column
//   VCC    (Red)           │  V+   — middle pin column
//   GND    (Brown/Black)   │  GND  — outer pin column
//
// ─────────────────────────────────────────────────────────────────────────────
// CHANNEL-TO-SERVO WIRING TABLE:
// ─────────────────────────────────────────────────────────────────────────────
//  PCA9685   Original     Leg    Joint Label   Code ID    Init
//  Channel   Servo Var           (001/002/003)             Angle
//  ──────────────────────────────────────────────────────────────
//   CH  0    myServo      FL     001 = HIP     FL_HIP     45 deg
//   CH  1    myServo3     FL     002 = FEMUR   FL_FEMUR  100 deg
//   CH  2    myServo2     FL     003 = TIBIA   FL_TIBIA  180 deg
//  ──────────────────────────────────────────────────────────────
//   CH  3    myServo4     FR     001 = HIP     FR_HIP    135 deg
//   CH  4    myServo5     FR     002 = FEMUR   FR_FEMUR   80 deg
//   CH  5    myServo6     FR     003 = TIBIA   FR_TIBIA    0 deg
//  ──────────────────────────────────────────────────────────────
//   CH  6    myServo7     BL     001 = HIP     BL_HIP    135 deg
//   CH  7    myServo8     BL     002 = FEMUR   BL_FEMUR   80 deg
//   CH  8    myServo9     BL     003 = TIBIA   BL_TIBIA    0 deg
//  ──────────────────────────────────────────────────────────────
//   CH  9    myServo10    BR     001 = HIP     BR_HIP      0 deg
//   CH 10    myServo11    BR     002 = FEMUR   BR_FEMUR    0 deg
//   CH 11    myServo12    BR     003 = TIBIA   BR_TIBIA    0 deg
//  ──────────────────────────────────────────────────────────────
//   CH 12-15  (SPARE - available for sensors, camera pan/tilt, etc.)
//
// REQUIRED LIBRARIES (Arduino Library Manager):
//   1. "Adafruit PWM Servo Driver Library" by Adafruit
//   2. Wire (built-in, no install needed)
// =============================================================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// PCA9685 at default I2C address 0x40
// If address jumpers are soldered, change: Adafruit_PWMServoDriver(0x41) etc.
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// ─────────────────────────────────────────────────────────────────────────────
// SERVO PULSE CALIBRATION
// Tune SERVOMIN / SERVOMAX if your servos don't reach 0 deg or 180 deg fully.
// MG90S / SG90  → SERVOMIN=150, SERVOMAX=600  (defaults below)
// MG996R        → SERVOMIN=120, SERVOMAX=580
// ─────────────────────────────────────────────────────────────────────────────
#define SERVOMIN   150   // Raw pulse at  0 degrees (~600 µs)
#define SERVOMAX   600   // Raw pulse at 180 degrees (~2400 µs)
#define SERVO_FREQ  50   // 50 Hz — standard for analog servos

// ─────────────────────────────────────────────────────────────────────────────
// CHANNEL DEFINITIONS
// Each leg has 3 joints:  001=HIP  002=FEMUR  003=TIBIA
// ─────────────────────────────────────────────────────────────────────────────

// Front-Left (FL)
#define FL_HIP      0   // FL 001 — myServo   (was pin 9)
#define FL_FEMUR    1   // FL 002 — myServo3  (was pin 11)
#define FL_TIBIA    2   // FL 003 — myServo2  (was pin 10)

// Front-Right (FR)
#define FR_HIP      3   // FR 001 — myServo4  (was pin 12)
#define FR_FEMUR    4   // FR 002 — myServo5  (was pin 8)
#define FR_TIBIA    5   // FR 003 — myServo6  (was pin 7)

// Back-Left (BL)
#define BL_HIP      6   // BL 001 — myServo7  (was pin 6)
#define BL_FEMUR    7   // BL 002 — myServo8  (was pin 5)
#define BL_TIBIA    8   // BL 003 — myServo9  (was pin 4)

// Back-Right (BR)
#define BR_HIP      9   // BR 001 — myServo10 (was pin 3)
#define BR_FEMUR   10   // BR 002 — myServo11 (was pin 2)
#define BR_TIBIA   11   // BR 003 — myServo12 (was pin 13)

// ─────────────────────────────────────────────────────────────────────────────
// HOME / STANDING POSITION ANGLES
// Ported exactly from Servo_motor_checking_code.ino
// ─────────────────────────────────────────────────────────────────────────────

// Front-Left
#define INIT_FL_HIP    45    // FL 001
#define INIT_FL_FEMUR 100    // FL 002
#define INIT_FL_TIBIA 180    // FL 003

// Front-Right
#define INIT_FR_HIP   135    // FR 001
#define INIT_FR_FEMUR  80    // FR 002
#define INIT_FR_TIBIA   0    // FR 003

// Back-Left
#define INIT_BL_HIP   135    // BL 001
#define INIT_BL_FEMUR  80    // BL 002
#define INIT_BL_TIBIA   0    // BL 003

// Back-Right
#define INIT_BR_HIP     0    // BR 001
#define INIT_BR_FEMUR   0    // BR 002
#define INIT_BR_TIBIA   0    // BR 003

// ─────────────────────────────────────────────────────────────────────────────
// HELPER: angleToPulse
// Converts degrees (0-180) to a PCA9685 12-bit pulse count.
// ─────────────────────────────────────────────────────────────────────────────
uint16_t angleToPulse(int angle) {
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

// ─────────────────────────────────────────────────────────────────────────────
// HELPER: setServo(channel, angle)
// Set any PCA9685 channel to a specific angle in degrees.
// Example: setServo(FL_HIP, 90);
// ─────────────────────────────────────────────────────────────────────────────
void setServo(uint8_t channel, int angle) {
  pwm.setPWM(channel, 0, angleToPulse(angle));
}

// ─────────────────────────────────────────────────────────────────────────────
// HELPER: setAllServosHome
// Sends all 12 servos to their initial standing positions simultaneously.
// ─────────────────────────────────────────────────────────────────────────────
void setAllServosHome() {
  // Front-Left: Hip(001) = 45, Femur(002) = 100, Tibia(003) = 180
  setServo(FL_HIP,   INIT_FL_HIP);
  setServo(FL_FEMUR, INIT_FL_FEMUR);
  setServo(FL_TIBIA, INIT_FL_TIBIA);

  // Front-Right: Hip(001) = 135, Femur(002) = 80, Tibia(003) = 0
  setServo(FR_HIP,   INIT_FR_HIP);
  setServo(FR_FEMUR, INIT_FR_FEMUR);
  setServo(FR_TIBIA, INIT_FR_TIBIA);

  // Back-Left: Hip(001) = 135, Femur(002) = 80, Tibia(003) = 0
  setServo(BL_HIP,   INIT_BL_HIP);
  setServo(BL_FEMUR, INIT_BL_FEMUR);
  setServo(BL_TIBIA, INIT_BL_TIBIA);

  // Back-Right: Hip(001) = 0, Femur(002) = 0, Tibia(003) = 0
  setServo(BR_HIP,   INIT_BR_HIP);
  setServo(BR_FEMUR, INIT_BR_FEMUR);
  setServo(BR_TIBIA, INIT_BR_TIBIA);
}

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  Serial.begin(9600);
  Serial.println("FusionForce RUNNER4 - PCA9685 12-DOF Controller");
  Serial.println("Joint naming: 001=HIP  002=FEMUR  003=TIBIA");
  Serial.println("Initialising PCA9685...");

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  Serial.println("Moving all 12 servos to HOME / standing position...");
  setAllServosHome();
  delay(2000);

  Serial.println("Ready.");
}

// =============================================================================
// LOOP — Test Sequence
// Sweeps BR 002 (Femur, CH 10) from 0 to 90 and back.
// Replace this with your gait logic once all servos are verified.
// =============================================================================
void loop() {
  // Sweep BR Femur (BR 002) — same servo that was tested in original code
  Serial.println("Sweeping BR FEMUR (002) CH10: 0 -> 90 deg");
  for (int a = 0; a <= 90; a++) {
    setServo(BR_FEMUR, a);
    delay(15);
  }
  delay(100);

  Serial.println("Sweeping BR FEMUR (002) CH10: 90 -> 0 deg");
  for (int a = 90; a >= 0; a--) {
    setServo(BR_FEMUR, a);
    delay(15);
  }
  delay(100);

  // Return all servos to home between test cycles
  Serial.println("All servos -> HOME");
  setAllServosHome();
  delay(1000);
}
