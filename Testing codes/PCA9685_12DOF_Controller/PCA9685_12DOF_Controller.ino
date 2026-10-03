// =============================================================================
// FusionForce RUNNER4 - 12 DOF Quadruped Robot
// PCA9685 Servo Controller - Full 12 Motor Implementation
// UPDATED FOR: ESP32-S3 (Tested on ESP32-S3-DevKitC-1 / XIAO ESP32-S3)
// =============================================================================
//
// JOINT NAMING CONVENTION:
//   001 = Hip   (rotates leg forward / backward — mounted on the body frame)
//   002 = Femur (upper leg joint — lifts the leg up / down)
//   003 = Tibia (lower leg / knee joint — extends or retracts the foot)
//
// ─────────────────────────────────────────────────────────────────────────────
// ESP32-S3 → PCA9685 I2C WIRING:
//
//   ESP32-S3 Pin      │  PCA9685 Pin  │  Wire Colour  │  Notes
//   ──────────────────┼───────────────┼───────────────┼──────────────────────
//   3.3V (or 5V)      │  VCC          │  Red          │  Logic power only
//   GND               │  GND          │  Black        │  Common ground
//   GPIO 8 (SDA)      │  SDA          │  Blue/White   │  Default I2C SDA pin
//   GPIO 9 (SCL)      │  SCL          │  Yellow       │  Default I2C SCL pin
//
//   WHY GPIO 8 & 9?
//   The ESP32-S3 does NOT have fixed hardware I2C pins — any GPIO can be used.
//   GPIO 8 and 9 are the most commonly used defaults in community examples and
//   are exposed on most ESP32-S3 DevKit boards. They are NOT strapping pins.
//   If using XIAO ESP32-S3: SDA=GPIO5 (D4), SCL=GPIO6 (D5) — change below.
//
//   PULL-UP RESISTORS:
//   The ESP32-S3 has internal pull-ups enabled by Wire.begin().
//   For reliable I2C over longer cables or multiple devices on the same bus,
//   add external 4.7kΩ resistors from SDA and SCL to 3.3V.
//
// ─────────────────────────────────────────────────────────────────────────────
// STEP 2 — EXTERNAL POWER → PCA9685 V+ RAIL:
//   Power Supply +  →  PCA9685 V+  terminal  (big green screw block)
//   Power Supply –  →  PCA9685 GND terminal  (big green screw block)
//   Recommended: 6V DC, min 3A SMPS/BEC  (NEVER use ESP32-S3 3.3V for servos!)
//
//   WHY SEPARATE POWER?
//   A single MG90S servo can draw 500mA stall current. 12 servos could draw
//   up to 6A simultaneously. The ESP32-S3 3.3V LDO is rated for only ~500mA.
//   Using the wrong power source will BROWN OUT the ESP32-S3 instantly.
//
// STEP 3 — SERVO → PCA9685 CHANNEL (plug 3-pin servo connector per channel):
//   Servo Wire             │  PCA9685 Pin Column
//   ───────────────────────┼──────────────────────────────────────────────────
//   Signal (Orange/Yellow) │  PWM  — innermost pin column
//   VCC    (Red)           │  V+   — middle pin column (from external 6V rail)
//   GND    (Brown/Black)   │  GND  — outer pin column
//
// ─────────────────────────────────────────────────────────────────────────────
// CHANNEL-TO-SERVO WIRING TABLE:
// ─────────────────────────────────────────────────────────────────────────────
//  PCA9685   Leg    Joint Label   Code ID    Init Angle
//  Channel          (001/002/003)
//  ──────────────────────────────────────────────────────────────
//   CH  0    FL     001 = HIP     FL_HIP     45 deg
//   CH  1    FL     002 = FEMUR   FL_FEMUR  100 deg
//   CH  2    FL     003 = TIBIA   FL_TIBIA  180 deg
//  ──────────────────────────────────────────────────────────────
//   CH  3    FR     001 = HIP     FR_HIP    135 deg
//   CH  4    FR     002 = FEMUR   FR_FEMUR   80 deg
//   CH  5    FR     003 = TIBIA   FR_TIBIA    0 deg
//  ──────────────────────────────────────────────────────────────
//   CH  6    BL     001 = HIP     BL_HIP    135 deg
//   CH  7    BL     002 = FEMUR   BL_FEMUR   80 deg
//   CH  8    BL     003 = TIBIA   BL_TIBIA    0 deg
//  ──────────────────────────────────────────────────────────────
//   CH  9    BR     001 = HIP     BR_HIP      0 deg
//   CH 10    BR     002 = FEMUR   BR_FEMUR    0 deg
//   CH 11    BR     003 = TIBIA   BR_TIBIA    0 deg
//  ──────────────────────────────────────────────────────────────
//   CH 12-15  (SPARE — sensors, camera pan/tilt, arm, gripper, etc.)
//
// REQUIRED LIBRARIES (Arduino Library Manager / PlatformIO):
//   1. "Adafruit PWM Servo Driver Library" by Adafruit
//   2. Wire (built-in with ESP32 Arduino core — no install needed)
//
// BOARD SETUP (Arduino IDE):
//   Tools → Board → "ESP32S3 Dev Module" (or your specific variant)
//   Tools → Upload Speed → 921600
//   Tools → USB CDC On Boot → Enabled   ← IMPORTANT for Serial.println() output!
//   Tools → Flash Mode → QIO 80MHz
//   Tools → PSRAM → Disabled (unless you have PSRAM)
//
// =============================================================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ─────────────────────────────────────────────────────────────────────────────
// ESP32-S3 I2C PIN CONFIGURATION
// Change these if you use a different board variant or pinout.
// ─────────────────────────────────────────────────────────────────────────────
#define I2C_SDA  8   // GPIO 8 — SDA (Standard ESP32-S3 DevKit)
#define I2C_SCL  9   // GPIO 9 — SCL (Standard ESP32-S3 DevKit)

// WHY explicit pin definition?
// Unlike Arduino Uno (fixed A4/A5), the ESP32-S3 I2C controller is "flexible" —
// it can be mapped to ANY GPIO. Passing the pins to Wire.begin() makes the
// wiring intent clear and lets you move to other pins without hunting in code.

// ─────────────────────────────────────────────────────────────────────────────
// PCA9685 I2C ADDRESS
// Default address = 0x40 (all address jumpers open).
// If you have soldered the A0 jumper → 0x41
// If you have soldered the A1 jumper → 0x42
// Check your board and match the address below.
// ─────────────────────────────────────────────────────────────────────────────
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// ─────────────────────────────────────────────────────────────────────────────
// SERVO PULSE CALIBRATION
// These raw 12-bit tick values map to 0° and 180° on your servo.
// The PCA9685 at 50Hz uses 4096 ticks for 20ms (one full PWM period).
// 1 tick = 20ms / 4096 ≈ 4.88µs
//
// SERVOMIN = 150 ticks → 150 × 4.88µs ≈ 732µs  (some servos need ~600µs)
// SERVOMAX = 600 ticks → 600 × 4.88µs ≈ 2930µs (some servos need ~2400µs)
//
// Servo Tuning:
//   MG90S / SG90  → SERVOMIN=150, SERVOMAX=600  (defaults — try these first)
//   MG996R        → SERVOMIN=120, SERVOMAX=580
//   DS3225 / DS3218 → SERVOMIN=102, SERVOMAX=512
//
// TUNING PROCEDURE:
//   1. Set a servo to 0° and measure the physical angle. Adjust SERVOMIN.
//   2. Set to 180° and measure. Adjust SERVOMAX.
//   3. Verify 90° lands exactly at the midpoint.
// ─────────────────────────────────────────────────────────────────────────────
#define SERVOMIN   150   // 12-bit raw tick at  0 degrees (~732 µs)
#define SERVOMAX   600   // 12-bit raw tick at 180 degrees (~2930 µs)
#define SERVO_FREQ  50   // 50 Hz — standard update rate for analog servos

// WHY 50 Hz?
// Standard RC/analog servos expect a PWM pulse every 20ms (50Hz).
// Digital servos can accept 100-333Hz, but 50Hz is universally safe.
// The PCA9685 prescaler = round(25MHz / (4096 x 50)) - 1 = 121 (auto-computed
// by the Adafruit library from SERVO_FREQ).

// ─────────────────────────────────────────────────────────────────────────────
// CHANNEL DEFINITIONS
// Each leg has 3 joints:  001=HIP  002=FEMUR  003=TIBIA
// ─────────────────────────────────────────────────────────────────────────────

// Front-Left (FL)
#define FL_HIP      0   // FL 001 — Hip servo
#define FL_FEMUR    1   // FL 002 — Femur servo
#define FL_TIBIA    2   // FL 003 — Tibia servo

// Front-Right (FR)
#define FR_HIP      3   // FR 001 — Hip servo
#define FR_FEMUR    4   // FR 002 — Femur servo
#define FR_TIBIA    5   // FR 003 — Tibia servo

// Back-Left (BL)
#define BL_HIP      6   // BL 001 — Hip servo
#define BL_FEMUR    7   // BL 002 — Femur servo
#define BL_TIBIA    8   // BL 003 — Tibia servo

// Back-Right (BR)
#define BR_HIP      9   // BR 001 — Hip servo
#define BR_FEMUR   10   // BR 002 — Femur servo
#define BR_TIBIA   11   // BR 003 — Tibia servo

// ─────────────────────────────────────────────────────────────────────────────
// HOME / STANDING POSITION ANGLES
// These are the angles each joint goes to when the robot "stands".
// Angles are asymmetric across left/right because the servos mirror each other
// physically — a 45° "forward" hip on the left is 135° on the right.
// ─────────────────────────────────────────────────────────────────────────────

// Front-Left
#define INIT_FL_HIP    45    // FL 001 — Hip neutral
#define INIT_FL_FEMUR 100    // FL 002 — Femur slightly lifted
#define INIT_FL_TIBIA 180    // FL 003 — Tibia extended

// Front-Right  (mirror of FL — hip angle reversed)
#define INIT_FR_HIP   135    // FR 001
#define INIT_FR_FEMUR  80    // FR 002
#define INIT_FR_TIBIA   0    // FR 003

// Back-Left
#define INIT_BL_HIP   135    // BL 001
#define INIT_BL_FEMUR  80    // BL 002
#define INIT_BL_TIBIA   0    // BL 003

// Back-Right
#define INIT_BR_HIP     0    // BR 001 — NOTE: all zeros; this leg may need tuning
#define INIT_BR_FEMUR   0    // BR 002
#define INIT_BR_TIBIA   0    // BR 003

// ─────────────────────────────────────────────────────────────────────────────
// DEBUG / DIAGNOSTIC FLAGS
// Set to 1 to enable verbose Serial output during testing.
// Set to 0 to disable for production / final firmware.
// ─────────────────────────────────────────────────────────────────────────────
#define DEBUG_VERBOSE    1   // Prints angle + tick value on every setServo() call
#define I2C_SCAN_ON_BOOT 1   // Scans I2C bus at startup to confirm device found

// ─────────────────────────────────────────────────────────────────────────────
// HELPER: i2cScan
// Scans all 127 I2C addresses and prints any found devices.
// Use this at startup to confirm PCA9685 is wired correctly and responding.
// Expected output:  "I2C device found at 0x40"
// If nothing prints, check wiring, pull-ups, and power to PCA9685 VCC.
// ─────────────────────────────────────────────────────────────────────────────
void i2cScan() {
  Serial.println("-------------------------------------------------");
  Serial.println("I2C Bus Scan:");
  int devicesFound = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("  [OK] I2C device found at 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      devicesFound++;
    }
  }
  if (devicesFound == 0) {
    Serial.println("  [!!] No I2C devices found!");
    Serial.println("       Check: SDA/SCL wires, pull-up resistors (4.7k),");
    Serial.println("       PCA9685 VCC power, and I2C_SDA/I2C_SCL pin defs.");
  }
  Serial.print("  Total devices found: ");
  Serial.println(devicesFound);
  Serial.println("-------------------------------------------------");
}

// ─────────────────────────────────────────────────────────────────────────────
// HELPER: angleToPulse
// Converts degrees (0-180) to a PCA9685 12-bit pulse count.
//
// HOW IT WORKS:
//   The map() function linearly interpolates:
//     0°   → SERVOMIN (150 ticks)
//     180° → SERVOMAX (600 ticks)
//     90°  → (150+600)/2 = 375 ticks
//
//   constrain() clamps the angle to [0, 180] so no servo is ever commanded
//   beyond its hardware limits, which would strip gears or stall the motor.
// ─────────────────────────────────────────────────────────────────────────────
uint16_t angleToPulse(int angle) {
  angle = constrain(angle, 0, 180);
  return (uint16_t)map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

// ─────────────────────────────────────────────────────────────────────────────
// HELPER: setServo(channel, angle)
// Sends a PWM command to a single PCA9685 channel.
//
// setPWM(channel, on, off):
//   'on'  = tick count when the pulse goes HIGH (always 0 here)
//   'off' = tick count when the pulse goes LOW  (= angleToPulse output)
//
// In other words: the HIGH pulse lasts from tick 0 to tick 'off',
// giving a pulse width of (off / 4096) x 20ms.
//
// Example: setServo(FL_HIP, 90)
//   → angleToPulse(90) = 375
//   → setPWM(0, 0, 375)
//   → pulse width = 375/4096 x 20ms ≈ 1.83ms (servo centers at ~1.5ms ideal)
// ─────────────────────────────────────────────────────────────────────────────
void setServo(uint8_t channel, int angle) {
  uint16_t pulse = angleToPulse(angle);

#if DEBUG_VERBOSE
  Serial.print("  CH");
  if (channel < 10) Serial.print("0");
  Serial.print(channel);
  Serial.print(" -> ");
  Serial.print(angle);
  Serial.print(" deg  (tick=");
  Serial.print(pulse);
  Serial.println(")");
#endif

  pwm.setPWM(channel, 0, pulse);
}

// ─────────────────────────────────────────────────────────────────────────────
// HELPER: setAllServosHome
// Sends all 12 servos to their HOME / standing positions simultaneously.
// This is the starting posture for the robot before any gait begins.
//
// WHY call them all quickly without delay in-between?
// The PCA9685 stores the new target tick value in hardware registers the moment
// setPWM() is called over I2C. The servo physically moves to the target over
// ~200ms (MG90S no-load speed). Sending all 12 commands takes ~12ms over
// I2C at 100kHz, so all servos start moving almost simultaneously.
// ─────────────────────────────────────────────────────────────────────────────
void setAllServosHome() {
  Serial.println("Setting all 12 servos -> HOME positions:");

  Serial.println("  [FL] Front-Left:");
  setServo(FL_HIP,   INIT_FL_HIP);
  setServo(FL_FEMUR, INIT_FL_FEMUR);
  setServo(FL_TIBIA, INIT_FL_TIBIA);

  Serial.println("  [FR] Front-Right:");
  setServo(FR_HIP,   INIT_FR_HIP);
  setServo(FR_FEMUR, INIT_FR_FEMUR);
  setServo(FR_TIBIA, INIT_FR_TIBIA);

  Serial.println("  [BL] Back-Left:");
  setServo(BL_HIP,   INIT_BL_HIP);
  setServo(BL_FEMUR, INIT_BL_FEMUR);
  setServo(BL_TIBIA, INIT_BL_TIBIA);

  Serial.println("  [BR] Back-Right:");
  setServo(BR_HIP,   INIT_BR_HIP);
  setServo(BR_FEMUR, INIT_BR_FEMUR);
  setServo(BR_TIBIA, INIT_BR_TIBIA);

  Serial.println("  HOME complete.");
}

// ─────────────────────────────────────────────────────────────────────────────
// HELPER: sweepServo(channel, fromAngle, toAngle, stepDelayMs)
// Sweeps a single servo from one angle to another, one degree at a time.
// stepDelayMs controls the speed — higher = slower, smoother.
//
// WHY step-by-step instead of jumping directly?
// Large angle jumps cause current spikes that can brown out the servo rail,
// and cause mechanical stress on plastic gears. Stepping at 15ms per degree
// gives ~2.7 seconds for a 180° sweep — safe for testing.
// ─────────────────────────────────────────────────────────────────────────────
void sweepServo(uint8_t channel, int fromAngle, int toAngle, uint16_t stepDelayMs) {
  int step = (toAngle >= fromAngle) ? 1 : -1;
  for (int a = fromAngle; a != toAngle + step; a += step) {
    setServo(channel, a);
    delay(stepDelayMs);
  }
}

// =============================================================================
// SETUP
// Runs once after power-on or reset.
// =============================================================================
void setup() {
  // ── Serial Monitor ─────────────────────────────────────────────────────────
  // On ESP32-S3 with USB CDC enabled, Serial goes over the native USB port.
  // The 3000ms timeout waits for the USB host (your PC) to enumerate before
  // printing, so you don't miss the first messages in the Serial Monitor.
  // If you use a UART-to-USB adapter on TX0/RX0, this timeout is not needed.
  Serial.begin(115200);
  uint32_t t = millis();
  while (!Serial && (millis() - t) < 3000) {
    delay(10);  // Wait for USB CDC to connect (max 3s)
  }

  Serial.println();
  Serial.println("==============================================");
  Serial.println("  FusionForce RUNNER4 — ESP32-S3 Boot");
  Serial.println("  PCA9685 12-DOF Controller Test");
  Serial.println("==============================================");
  Serial.print("  I2C SDA -> GPIO"); Serial.println(I2C_SDA);
  Serial.print("  I2C SCL -> GPIO"); Serial.println(I2C_SCL);
  Serial.println("  PCA9685 address: 0x40");
  Serial.println("  SERVOMIN=150  SERVOMAX=600  FREQ=50Hz");
  Serial.println();

  // ── I2C Bus Initialisation ─────────────────────────────────────────────────
  // Wire.begin(SDA, SCL) — ESP32 Arduino core signature with explicit pins.
  // This is DIFFERENT from Arduino Uno where Wire.begin() takes no arguments.
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println("Wire (I2C) initialised.");

  // ── I2C Device Scan ────────────────────────────────────────────────────────
#if I2C_SCAN_ON_BOOT
  i2cScan();
  // Expected: "[OK] I2C device found at 0x40"
  // If 0x40 is NOT listed → PCA9685 not responding. Check wiring!
#endif

  // ── PCA9685 Initialisation ─────────────────────────────────────────────────
  Serial.println("Initialising PCA9685...");
  pwm.begin();
  // WHAT pwm.begin() DOES:
  //   1. Sends RESET command to PCA9685 (register 0x00 = 0x80 → software reset)
  //   2. Waits 10ms for oscillator to stabilise
  //   3. Leaves all channels at 0 (servos at ~0° or holding last position)

  pwm.setPWMFreq(SERVO_FREQ);
  // WHAT setPWMFreq() DOES:
  //   Computes prescale = round(25 000 000 / (4096 x 50)) - 1 = 121
  //   Writes prescale to register 0xFE while PCA9685 is in SLEEP mode,
  //   then wakes it. The internal 25MHz oscillator is factory-trimmed
  //   but may drift ±1%. For servos, this is acceptable.
  delay(10);  // Extra stabilisation margin after frequency set

  // ── Move All Servos to Home ────────────────────────────────────────────────
  Serial.println();
  setAllServosHome();
  delay(2000);  // Give servos 2 seconds to physically reach HOME position

  Serial.println();
  Serial.println("==============================================");
  Serial.println("  Setup complete. Starting test loop.");
  Serial.println("  Monitor: 115200 baud | USB CDC");
  Serial.println("==============================================");
  Serial.println();
}

// =============================================================================
// LOOP — Continuous Test Sequence
//
// WHAT THIS TEST DOES:
//   1. Sweeps BR FEMUR (CH10) from 0° → 90°  (leg raises)
//   2. Sweeps BR FEMUR (CH10) from 90° → 0°  (leg lowers)
//   3. Returns ALL 12 servos to HOME between cycles
//   4. Repeats forever
//
// WHY BR FEMUR first?
//   It is the easiest to observe — attach one servo to CH10 and watch it move.
//   Once confirmed, expand to full gait logic by replacing this loop.
//
// HOW TO EXPAND:
//   Replace or add to this loop with calls like:
//     sweepServo(FL_HIP, 45, 90, 15);  // Sweep FL Hip from 45° to 90°
//     setServo(FL_HIP, 45);            // Jump FL Hip to 45° instantly
//     setAllServosHome();              // Reset all legs
// =============================================================================
void loop() {
  // ── Test 1: Sweep BR Femur 0° → 90° ────────────────────────────────────────
  Serial.println("-- Test: BR FEMUR (CH10) sweep  0 -> 90 deg --");
  sweepServo(BR_FEMUR, 0, 90, 15);
  // 90 steps x 15ms = 1350ms (1.35 seconds to sweep)
  delay(300);

  // ── Test 2: Sweep BR Femur 90° → 0° ────────────────────────────────────────
  Serial.println("-- Test: BR FEMUR (CH10) sweep 90 ->  0 deg --");
  sweepServo(BR_FEMUR, 90, 0, 15);
  delay(300);

  // ── Test 3: Verify All Legs Return Home ─────────────────────────────────────
  Serial.println("-- All servos -> HOME --");
  setAllServosHome();
  delay(1500);

  Serial.println();
}
