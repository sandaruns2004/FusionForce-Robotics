#pragma once
// =============================================================================
// arm_interface.h — Direct 2-Servo Arm Control via ESP32 GPIO PWM
//
// HARDWARE: 2 servo motors wired directly to the main ESP32-S3 (no separate
//           arm ESP32 needed). The ESP32 LEDC peripheral generates the PWM
//           signal — no PCA9685 channels are used for the arm.
//
// PIN ASSIGNMENT (chosen from free GPIOs — confirmed against pinout doc):
//   ARM_LIFT_PIN  = GPIO 47  — Lift servo  (raises / lowers the arm)
//   ARM_GRIP_PIN  = GPIO 21  — Grip servo  (opens / closes the gripper)
//
//   GPIO 47 : explicitly listed as [Free GPIO] on the DevKit right header.
//   GPIO 21 : Left ToF XSHUT — the left ToF sensor is NOT fitted, so this
//             pin is free for digital output.
//
// WIRING:
//   Servo signal wire → GPIO pin
//   Servo red (VCC)   → 5-6 V servo power rail  (DO NOT use 3.3 V)
//   Servo black/brown → Common GND
//
// SERVO PWM SPEC (standard hobby servo):
//   Frequency : 50 Hz (20 ms period)
//   Pulse range: 500 µs (0°) … 2500 µs (180°)
//   Neutral    : 1500 µs (90°)
//
// COMPATIBILITY:
//   Supports both ESP32 Arduino Core 2.x and Core 3.x+ (Core 3.x uses
//   ledcAttach/ledcWrite pin API; Core 2.x uses ledcSetup/ledcAttachPin).
//   ESP32-S3 has 8 LEDC channels (0..7) and 14-bit max timer resolution.
//
// MOTION SEQUENCE:
//   PICK  : lift UP → grip OPEN → lift DOWN → grip CLOSE → lift UP
//   PLACE : lift DOWN → grip OPEN → settle → lift UP → grip CLOSE
//   UP    : lift servo → SAFE angle
//   DOWN  : lift servo → LOW angle
//   OPEN  : grip servo → OPEN angle
//   CLOSE : grip servo → CLOSED angle
//
// All angles are configurable below. Run the web UI (/comp/arm?cmd=X) or
// individual /arm/* routes to test each command.
// =============================================================================

#include <Arduino.h>
#include <esp_arduino_version.h>

// =============================================================================
// PIN CONFIGURATION  — edit here if you rewire
// =============================================================================
#ifndef ARM_LIFT_PIN
#define ARM_LIFT_PIN   47    // Lift servo GPIO (free pin on DevKit right header)
#endif
#ifndef ARM_GRIP_PIN
#define ARM_GRIP_PIN   21    // Grip servo GPIO (Left XSHUT — left ToF not fitted)
#endif

// =============================================================================
// LEDC CONFIGURATION
// =============================================================================
#define ARM_LEDC_FREQ      50      // Hz — standard servo frequency
// ESP32-S3 maximum hardware LEDC timer bit width is 14 bits (SOC_LEDC_TIMER_BIT_WIDTH = 14)
#define ARM_LEDC_RES       14      // bits resolution (16383 counts)
#define ARM_LIFT_CHANNEL    6      // LEDC channel (0-7 valid on ESP32-S3)
#define ARM_GRIP_CHANNEL    7      // LEDC channel (0-7 valid on ESP32-S3)

// Duty conversion for 14-bit / 50 Hz (20,000 µs period = 16383 counts):
#define ARM_US_TO_DUTY(us)  ((uint32_t)(((float)(us) / 20000.0f) * ((1 << ARM_LEDC_RES) - 1)))

// =============================================================================
// SERVO ANGLE PRESETS  — tune these for your physical arm
// =============================================================================

// ---- Lift servo angles (degrees, converted to pulse width via map) -----------
// Angle 0° = 500 µs, Angle 180° = 2500 µs (standard)
#define ARM_LIFT_UP_DEG        30    // Safe travel position (arm raised)
#define ARM_LIFT_DOWN_DEG     150    // Lowered to ball pick/place level
#define ARM_LIFT_SAFE_DEG      30    // Alias — same as UP

// ---- Grip servo angles -------------------------------------------------------
#define ARM_GRIP_OPEN_DEG      30    // Gripper open (ready to grab or release)
#define ARM_GRIP_CLOSE_DEG    140    // Gripper closed (holding ball)

// ---- Timing (ms) -------------------------------------------------------------
#define ARM_MOVE_SETTLE_MS    400    // Wait after any single servo move
#define ARM_GRIP_SETTLE_MS    300    // Wait after grip open/close
#define ARM_PICK_SETTLE_MS    400    // Extra wait while gripping ball
#define ARM_PLACE_SETTLE_MS   400    // Extra wait while releasing ball

// =============================================================================
// FORWARD DECLARATIONS (from main .ino)
// =============================================================================
extern void serviceBackground();
extern void waitMs(uint32_t ms);
extern volatile bool compStop;

// =============================================================================
// INTERNAL HELPERS
// =============================================================================
namespace armns {

static bool sInitialised = false;

// Convert degrees (0–180) to a 14-bit LEDC duty value
// Standard servo: 500 µs = 0°, 2500 µs = 180°
static inline uint32_t degToDuty(int deg) {
    deg = constrain(deg, 0, 180);
    // Map 0°→500µs, 180°→2500µs (linear)
    uint32_t us = (uint32_t)map(deg, 0, 180, 500, 2500);
    return ARM_US_TO_DUTY(us);
}

// Write to lift servo pin directly (handles Core 2.x and Core 3.x+)
static inline void writeLift(int deg) {
    uint32_t duty = degToDuty(deg);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWrite(ARM_LIFT_PIN, duty);
#else
    ledcWrite(ARM_LIFT_CHANNEL, duty);
#endif
}

// Write to grip servo pin directly (handles Core 2.x and Core 3.x+)
static inline void writeGrip(int deg) {
    uint32_t duty = degToDuty(deg);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWrite(ARM_GRIP_PIN, duty);
#else
    ledcWrite(ARM_GRIP_CHANNEL, duty);
#endif
}

// Generic write helper supporting pin or legacy channel arguments
static inline void writeServo(uint8_t pinOrChannel, int deg) {
    if (pinOrChannel == ARM_LIFT_PIN || pinOrChannel == ARM_LIFT_CHANNEL || pinOrChannel == 8) {
        writeLift(deg);
    } else if (pinOrChannel == ARM_GRIP_PIN || pinOrChannel == ARM_GRIP_CHANNEL || pinOrChannel == 9) {
        writeGrip(deg);
    } else {
        uint32_t duty = degToDuty(deg);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
        ledcWrite(pinOrChannel, duty);
#else
        ledcWrite(pinOrChannel, duty);
#endif
    }
}

// Write servo and wait for movement to complete
static inline void writeServoWait(uint8_t pinOrChannel, int deg, uint32_t settleMs) {
    writeServo(pinOrChannel, deg);
    waitMs(settleMs);
}

} // namespace armns

// =============================================================================
// PUBLIC API — INIT
// =============================================================================

// Initialise LEDC PWM for both arm servos. Call once from setup().
inline void armInit() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    // ESP32 Arduino Core 3.x API
    ledcAttach(ARM_LIFT_PIN, ARM_LEDC_FREQ, ARM_LEDC_RES);
    ledcAttach(ARM_GRIP_PIN, ARM_LEDC_FREQ, ARM_LEDC_RES);
#else
    // ESP32 Arduino Core 2.x API
    ledcSetup(ARM_LIFT_CHANNEL, ARM_LEDC_FREQ, ARM_LEDC_RES);
    ledcSetup(ARM_GRIP_CHANNEL, ARM_LEDC_FREQ, ARM_LEDC_RES);
    ledcAttachPin(ARM_LIFT_PIN, ARM_LIFT_CHANNEL);
    ledcAttachPin(ARM_GRIP_PIN, ARM_GRIP_CHANNEL);
#endif

    // Move to safe starting position
    armns::writeLift(ARM_LIFT_UP_DEG);
    armns::writeGrip(ARM_GRIP_CLOSE_DEG);

    armns::sInitialised = true;

    Serial.printf("[ARM] Direct servo init: Lift=GPIO%d Grip=GPIO%d\n",
                  ARM_LIFT_PIN, ARM_GRIP_PIN);
    Serial.printf("[ARM] Lift angles: UP=%d° DOWN=%d°   Grip: OPEN=%d° CLOSE=%d°\n",
                  ARM_LIFT_UP_DEG, ARM_LIFT_DOWN_DEG,
                  ARM_GRIP_OPEN_DEG, ARM_GRIP_CLOSE_DEG);
}

// =============================================================================
// PUBLIC API — DIRECT ANGLE SET
// =============================================================================
inline void armSetLift(int deg) {
    armns::writeLift(deg);
}

inline void armSetGrip(int deg) {
    armns::writeGrip(deg);
}

// =============================================================================
// PUBLIC API — INDIVIDUAL MOVES
// =============================================================================

// Raise the lift servo to the safe travel position
inline bool armUp() {
    Serial.println("[ARM] Lift → UP");
    armns::writeServoWait(ARM_LIFT_PIN, ARM_LIFT_UP_DEG, ARM_MOVE_SETTLE_MS);
    return true;
}

// Lower the lift servo to ball level
inline bool armDown() {
    Serial.println("[ARM] Lift → DOWN");
    armns::writeServoWait(ARM_LIFT_PIN, ARM_LIFT_DOWN_DEG, ARM_MOVE_SETTLE_MS);
    return true;
}

// Open the gripper
inline bool armOpen() {
    Serial.println("[ARM] Grip → OPEN");
    armns::writeServoWait(ARM_GRIP_PIN, ARM_GRIP_OPEN_DEG, ARM_GRIP_SETTLE_MS);
    return true;
}

// Close the gripper
inline bool armClose() {
    Serial.println("[ARM] Grip → CLOSE");
    armns::writeServoWait(ARM_GRIP_PIN, ARM_GRIP_CLOSE_DEG, ARM_GRIP_SETTLE_MS);
    return true;
}

// Ensure arm is in safe travel position (up + closed)
inline void armSafePosition() {
    Serial.println("[ARM] → Safe travel position");
    armns::writeGrip(ARM_GRIP_CLOSE_DEG);
    armns::writeLift(ARM_LIFT_UP_DEG);
    waitMs(ARM_MOVE_SETTLE_MS);
}

// =============================================================================
// PUBLIC API — HIGH-LEVEL SEQUENCES
// =============================================================================

// Full pick sequence: UP → OPEN → DOWN → settle → CLOSE (grip) → UP
inline bool armPickBall() {
    if (!armns::sInitialised) {
        Serial.println("[ARM] ERROR: not initialised");
        return false;
    }
    Serial.println("[ARM] === PICK SEQUENCE ===");

    // 1. Ensure arm is raised before starting
    armns::writeLift(ARM_LIFT_UP_DEG);
    waitMs(ARM_MOVE_SETTLE_MS);
    if (compStop) return false;

    // 2. Open gripper (ready to receive ball)
    armns::writeGrip(ARM_GRIP_OPEN_DEG);
    waitMs(ARM_GRIP_SETTLE_MS);
    if (compStop) return false;

    // 3. Lower arm to ball
    armns::writeLift(ARM_LIFT_DOWN_DEG);
    waitMs(ARM_MOVE_SETTLE_MS);
    if (compStop) return false;

    // 4. Grip the ball — settle slightly longer to grip firmly
    armns::writeGrip(ARM_GRIP_CLOSE_DEG);
    waitMs(ARM_PICK_SETTLE_MS);
    if (compStop) return false;

    // 5. Lift arm up with ball secured
    armns::writeLift(ARM_LIFT_UP_DEG);
    waitMs(ARM_MOVE_SETTLE_MS);

    Serial.println("[ARM] Pick complete — ball secured");
    return true;
}

// Full place sequence: DOWN (while gripping) → OPEN (release ball) → UP → CLOSE
inline bool armPlaceBall() {
    if (!armns::sInitialised) {
        Serial.println("[ARM] ERROR: not initialised");
        return false;
    }
    Serial.println("[ARM] === PLACE SEQUENCE ===");

    // 1. Lower arm while holding ball
    armns::writeLift(ARM_LIFT_DOWN_DEG);
    waitMs(ARM_MOVE_SETTLE_MS);
    if (compStop) return false;

    // 2. Open gripper to release ball
    armns::writeGrip(ARM_GRIP_OPEN_DEG);
    waitMs(ARM_PLACE_SETTLE_MS);  // let ball settle
    if (compStop) return false;

    // 3. Raise arm (leave ball on floor)
    armns::writeLift(ARM_LIFT_UP_DEG);
    waitMs(ARM_MOVE_SETTLE_MS);
    if (compStop) return false;

    // 4. Close gripper (clean travel position)
    armns::writeGrip(ARM_GRIP_CLOSE_DEG);
    waitMs(ARM_GRIP_SETTLE_MS);

    Serial.println("[ARM] Place complete — ball released");
    return true;
}

// =============================================================================
// PUBLIC API — GENERIC COMMAND (used by /comp/arm?cmd=X web route)
// =============================================================================
inline bool armSendCmd(const char* cmd, uint32_t /*timeoutMs*/ = 2000) {
    String c = String(cmd);
    c.trim();
    c.toLowerCase();

    if (c == "pick")  return armPickBall();
    if (c == "place") return armPlaceBall();
    if (c == "up")    return armUp();
    if (c == "down")  return armDown();
    if (c == "open")  return armOpen();
    if (c == "close") return armClose();
    if (c == "safe")  { armSafePosition(); return true; }

    // Status — just return current pin info
    Serial.printf("[ARM] status: lift_pin=GPIO%d grip_pin=GPIO%d (direct PWM mode)\n",
                  ARM_LIFT_PIN, ARM_GRIP_PIN);
    return true;  // "ping" always succeeds in direct mode
}

// Legacy aliases
inline bool armPickUp()    { return armPickBall();  }
inline bool armPlaceDown() { return armPlaceBall(); }
inline bool armPing()      { return armSendCmd("status"); }
