#pragma once
// =============================================================================
// nav_primitives.h — High-Level Navigation Primitives for RUNNER-4 Competition
//
// Provides BLOCKING functions for the competition state machine:
//   - Line following with custom stop conditions (junction, ToF, line-lost)
//   - Calibrated 90°/180° turns
//   - ToF-based ball and obstacle detection
//   - Color reading with timeout
//   - Post-turn line alignment
//
// Uses the same trot gait engine (6-phase diagonal pair walk) as the existing
// walkForward()/lineFollowWalk(), but with custom stop conditions for
// competition decision-making.
//
// All functions are BLOCKING but cooperative: they call serviceBackground()
// during waits so the web server, sensors, and LEDs keep updating.
//
// DEPENDENCIES:
//   From .ino : moveToTarget(), moveToPose(), syncGaitBase(), walkForward(),
//               turnGait(), serviceBackground(), waitMs(), buildGaitPose()
//   Modules   : line_sensor, tof_sensor, color_sensor
// =============================================================================

#include <Arduino.h>

// ── Forward declarations of core robot functions from .ino ───────────────────
void moveToTarget(const int targetAngles[4][3], int speedMs);
void moveToPose(int id, int speedMs);
void syncGaitBase();
void serviceBackground();
void waitMs(uint32_t ms);
void buildGaitPose(int out[4][3], const int hipBase[4], const int hipSign[4],
                   int hipDelta, const int femPlant[4], const int femSign[4],
                   const int femDelta[4], uint8_t swingMask, uint8_t liftMask);
void walkForward(int steps, int speedMs, int variant);
void turnGait(int steps, int speedMs, bool rightTurn);
bool robotBusy();
void addCORS();

// ── Forward declarations of gait base arrays from .ino ───────────────────────
extern int WALK_HIP_BASE[4], WALK_FEM_PLANT[4], GAIT_TIBIA[4];
extern int WALK_HIP_DELTA, WALK_FEMUR_DELTA;
extern int TURN_HIP_DELTA, TURN_FEMUR_DELTA;
extern const int WALK_HIP_SIGN[4];
extern const int FEMUR_LIFT_SIGN[4];
extern const int WALK_FEMUR_TRIM[4];

// ── Forward declarations from sensor modules ─────────────────────────────────
// MPU6050 gyroscope (sensors/mpu6050.h)
extern float mpuGetYaw();
extern float mpuGetGz();
extern bool mpuIsReady();
extern void mpuResetYaw();
extern void mpuSetStationary(bool);
extern bool mpuTurnToAngle(float deg, bool rightTurn);

extern bool lineCalibrated();
extern bool lineFound();
extern float lineErrorMm();
extern bool lineJunction();
extern const char *lineJunctionType();
extern bool lineLost();
extern bool lineSafeStop();
extern void lineSetAlpha(float a);
extern uint8_t lineActiveCount();

extern float tofFront();
extern float tofRight();
extern float tofLeft();
extern bool tofOnline();
extern bool tofFrontOnline();
extern bool tofRightOnline();
extern bool tofLeftOnline();

extern uint8_t colorConfirmed();
extern uint8_t colorStoredBall();
extern const char *colorName(uint8_t id);

// ── Competition stop flag (defined in competition_fsm.h) ─────────────────────
extern volatile bool compStop;

// ── Heading constants ────────────────────────────────────────────────────────
#define HEADING_NORTH 0
#define HEADING_EAST 1
#define HEADING_SOUTH 2
#define HEADING_WEST 3

// ── Leg bit masks (for gait building) ────────────────────────────────────────
#define NAV_FL 1
#define NAV_FR 2
#define NAV_BL 4
#define NAV_BR 8

// =============================================================================
// CALIBRATION CONSTANTS — MUST BE TUNED ON THE ACTUAL ROBOT
// =============================================================================

// Turn calibration: number of turnGait() steps for 90° and 180°.
// Calibrated for TURN_HIP_DELTA=10 deg (tighter, smoother rotation)
#define NAV_TURN_STEPS_90 6   // turnGait steps for a 90° turn
#define NAV_TURN_STEPS_180 12 // turnGait steps for a 180° turn
#define NAV_TURN_SPEED_MS 400 // ms per phase during turns
#define NAV_TURN_TOL_DEG 3.0f // gyro turn tolerance in degrees

// Line-follow PD gains (simplified for nav use; same defaults as line_follow.h)
#define NAV_KP 0.30f          // proportional gain (deg/mm)
#define NAV_KD 0.10f          // derivative gain
#define NAV_MAX_CORRECTION 15 // max steering correction in degrees
#define NAV_WALK_SPEED_MS 200 // ms per gait phase during line following
#define NAV_BASE_HIP_DELTA 15 // base hip swing degrees (tight, stable stride)
#define NAV_FEMUR_DELTA 20    // femur lift degrees

// ToF detection thresholds (mm)
#define NAV_BALL_DETECT_MM 200.0f // ToF < this → ball probably 1 cell ahead
#define NAV_BALL_CLOSE_MM 120.0f  // ToF < this → ball very close (pickup range)
#define NAV_OBSTACLE_DETECT_MM 150.0f // ToF < this → wall/obstacle ahead
#define NAV_EMPTY_PATH_MM 400.0f      // ToF > this → clear path ahead

// Junction detection confirmation (avoid false positives from wide lines)
#define NAV_JUNCTION_CONFIRM 2 // require N consecutive junction readings

// Post-turn alignment
#define NAV_ALIGN_MAX_STEPS 3 // max walk steps to re-find line after turn

// Walking distance calibration
// How many line-follow cycles ≈ one 25cm grid cell?  Must be calibrated.
#define NAV_CYCLES_PER_CELL 6 // approximate; tune on real robot

// =============================================================================
// STOP CONDITION FLAGS (bitwise OR for navLineFollowUntil)
// =============================================================================
#define STOP_NONE 0x00
#define STOP_ON_JUNCTION 0x01  // Stop when line sensor detects junction
#define STOP_ON_LINE_LOST 0x02 // Stop when line is lost
#define STOP_ON_TOF_CLOSE 0x04 // Stop when ToF reading < tofThreshold
#define STOP_ON_ABORT 0x08     // Stop when compStop flag is set
#define STOP_ON_COUNT 0x10     // Stop after maxCycles reached
#define STOP_ON_ALL_DARK 0x20  // Stop when all line sensors dark (off grid)

// =============================================================================
// RESULT STRUCT returned by navLineFollowUntil
// =============================================================================
struct NavResult {
  uint8_t reason;    // which STOP_ON_xxx flag triggered the stop
  int cyclesDone;    // total walk cycles completed
  float lastToF;     // last ToF reading in mm
  float lastError;   // last line error in mm
  bool junctionSeen; // was a junction detected at stop?
  char juncType[16]; // junction type string ("T/CROSS", "LEFT", etc.)

  // Convenience checks
  bool stoppedOnJunction() const { return reason & STOP_ON_JUNCTION; }
  bool stoppedOnTof() const { return reason & STOP_ON_TOF_CLOSE; }
  bool stoppedOnLost() const { return reason & STOP_ON_LINE_LOST; }
  bool stoppedOnAbort() const { return reason & STOP_ON_ABORT; }
};

// =============================================================================
// INTERNAL HELPERS (namespace to avoid symbol clashes)
// =============================================================================
namespace navns {

// ── Asymmetric gait pose builder ─────────────────────────────────────────────
// Like buildGaitPose() but with separate L/R hip deltas for differential
// steering.  Leg sides: FL(0)=LEFT, FR(1)=RIGHT, BL(2)=LEFT, BR(3)=RIGHT.
static void buildSteerPose(int out[4][3], const int hipBase[4],
                           const int hipSign[4], int hipDeltaL, int hipDeltaR,
                           const int femPlant[4], const int femSign[4],
                           const int femDelta[4], uint8_t swingMask,
                           uint8_t liftMask) {
  const int hipDelta[4] = {hipDeltaL, hipDeltaR, hipDeltaL, hipDeltaR};
  for (int leg = 0; leg < 4; leg++) {
    out[leg][0] = hipBase[leg] +
                  (((swingMask >> leg) & 1) ? hipSign[leg] * hipDelta[leg] : 0);
    out[leg][1] = femPlant[leg] +
                  (((liftMask >> leg) & 1) ? femSign[leg] * femDelta[leg] : 0);
    out[leg][2] = GAIT_TIBIA[leg];
    out[leg][0] = constrain(out[leg][0], 0, 180);
    out[leg][1] = constrain(out[leg][1], 0, 180);
  }
}

// ── PD steering from line sensor ─────────────────────────────────────────────
static float sPrevErr = 0;

static void computeSteering(int &hipDeltaL, int &hipDeltaR) {
  float err = lineErrorMm();
  float dErr = err - sPrevErr;
  float correction = NAV_KP * err + NAV_KD * dErr;
  correction = constrain(correction, -(float)NAV_MAX_CORRECTION,
                         (float)NAV_MAX_CORRECTION);
  sPrevErr = err;

  // correction > 0 → left legs stride MORE  → curves left
  // correction < 0 → right legs stride MORE → curves right
  hipDeltaL = constrain(NAV_BASE_HIP_DELTA + (int)roundf(correction), 0, 45);
  hipDeltaR = constrain(NAV_BASE_HIP_DELTA - (int)roundf(correction), 0, 45);
}

} // namespace navns

// =============================================================================
// PUBLIC API
// =============================================================================

// ─────────────────────────────────────────────────────────────────────────────
// navLineFollowUntil — Walk forward with PD line-following, stop on condition
// ─────────────────────────────────────────────────────────────────────────────
//
// Parameters:
//   maxCycles     Maximum walk cycles before giving up (0 = unlimited, capped
//                 at 10 000 internally for safety).
//   stopFlags     Bitwise OR of STOP_ON_xxx flags.
//   tofThreshold  ToF distance threshold in mm (used with STOP_ON_TOF_CLOSE).
//
// Returns:
//   NavResult with the stop reason, cycle count, and sensor readings at stop.
//
// The robot is left in the STANDING pose after this function returns.
//
inline NavResult navLineFollowUntil(int maxCycles, uint8_t stopFlags,
                                    float tofThreshold = NAV_BALL_DETECT_MM) {
  NavResult result = {};
  syncGaitBase();
  navns::sPrevErr = 0;

  // Override line sensor EMA for smoother readings during walk
  lineSetAlpha(0.50f);

  int femDelta[4];
  for (int l = 0; l < 4; l++)
    femDelta[l] = NAV_FEMUR_DELTA + WALK_FEMUR_TRIM[l];

  // 6-phase trot sequence: {swingMask, liftMask}
  const uint8_t seq[6][2] = {
      {0, NAV_FL | NAV_BR},               // 1: FL&BR lift
      {NAV_FL | NAV_BR, NAV_FL | NAV_BR}, // 2: FL&BR swing
      {NAV_FL | NAV_BR, 0},               // 3: FL&BR plant
      {0, NAV_FR | NAV_BL},               // 4: FR&BL lift
      {NAV_FR | NAV_BL, NAV_FR | NAV_BL}, // 5: FR&BL swing
      {NAV_FR | NAV_BL, 0}                // 6: FR&BL plant
  };

  int limit = (maxCycles <= 0) ? 10000 : maxCycles;
  int juncConfirmCount = 0;
  bool stopped = false;

  Serial.printf("[NAV] LineFollow start: max=%d flags=0x%02X tof=%.0f\n",
                maxCycles, stopFlags, tofThreshold);

  for (int cycle = 0; cycle < limit && !stopped && !compStop; cycle++) {
    // Execute one full 6-phase trot cycle
    for (int p = 0; p < 6 && !compStop; p++) {
      // PD steering
      int hipL = NAV_BASE_HIP_DELTA;
      int hipR = NAV_BASE_HIP_DELTA;
      if (lineFound()) {
        navns::computeSteering(hipL, hipR);
      }

      // Build and execute one gait phase
      int t[4][3];
      navns::buildSteerPose(t, WALK_HIP_BASE, WALK_HIP_SIGN, hipL, hipR,
                            WALK_FEM_PLANT, FEMUR_LIFT_SIGN, femDelta,
                            seq[p][0], seq[p][1]);
      moveToTarget(t, NAV_WALK_SPEED_MS);
    }

    result.cyclesDone = cycle + 1;
    result.lastToF = tofFront();
    result.lastError = lineErrorMm();

    // ── Check stop conditions in priority order ──────────────────────

    // 1. Abort
    if ((stopFlags & STOP_ON_ABORT) && compStop) {
      result.reason = STOP_ON_ABORT;
      stopped = true;
      break;
    }

    // 2. Junction (with confirmation to avoid glitches)
    if (stopFlags & STOP_ON_JUNCTION) {
      if (lineJunction()) {
        juncConfirmCount++;
        if (juncConfirmCount >= NAV_JUNCTION_CONFIRM) {
          result.reason = STOP_ON_JUNCTION;
          result.junctionSeen = true;
          strncpy(result.juncType, lineJunctionType(),
                  sizeof(result.juncType) - 1);
          Serial.printf("[NAV] Junction: %s @ cycle %d\n", result.juncType,
                        cycle + 1);
          stopped = true;
          break;
        }
      } else {
        juncConfirmCount = 0;
      }
    }

    // 3. ToF obstacle/ball
    if ((stopFlags & STOP_ON_TOF_CLOSE) && tofOnline()) {
      float d = tofFront();
      if (d > 10.0f && d < tofThreshold) {
        result.reason = STOP_ON_TOF_CLOSE;
        result.lastToF = d;
        Serial.printf("[NAV] ToF: %.0fmm < %.0fmm @ cycle %d\n", d,
                      tofThreshold, cycle + 1);
        stopped = true;
        break;
      }
    }

    // 4. Line lost
    if ((stopFlags & STOP_ON_LINE_LOST) && lineLost()) {
      result.reason = STOP_ON_LINE_LOST;
      Serial.printf("[NAV] Line lost @ cycle %d\n", cycle + 1);
      stopped = true;
      break;
    }

    // 5. All sensors dark (off the grid entirely)
    if ((stopFlags & STOP_ON_ALL_DARK) && lineActiveCount() == 0) {
      result.reason = STOP_ON_ALL_DARK;
      Serial.printf("[NAV] All dark @ cycle %d\n", cycle + 1);
      stopped = true;
      break;
    }

    // 6. Cycle count reached
    if ((stopFlags & STOP_ON_COUNT) && cycle + 1 >= maxCycles) {
      result.reason = STOP_ON_COUNT;
      stopped = true;
      break;
    }
  }

  // Final abort check
  if (compStop && result.reason == 0) {
    result.reason = STOP_ON_ABORT;
  }

  // Restore sensor EMA
  lineSetAlpha(-1); // -1 = restore previous value

  // Return to STANDING pose so we're ready for the next action
  moveToPose(1, 400);

  Serial.printf(
      "[NAV] LineFollow done: reason=0x%02X cycles=%d toF=%.0f err=%.1f\n",
      result.reason, result.cyclesDone, result.lastToF, result.lastError);
  return result;
}

// ─────────────────────────────────────────────────────────────────────────────
// POST-TURN LINE ALIGNMENT
// ─────────────────────────────────────────────────────────────────────────────
// After a turn, the line sensor may not see the new line immediately.
// Walk forward a few steps to re-find the line.

inline bool navAlignToLine(int maxSteps = NAV_ALIGN_MAX_STEPS) {
  Serial.println("[NAV] Aligning to line...");
  // First check if we're already on the line
  serviceBackground();
  if (lineFound()) {
    Serial.println("[NAV] Already aligned");
    return true;
  }
  // Walk forward slowly until line found
  for (int i = 0; i < maxSteps && !compStop; i++) {
    walkForward(1, NAV_WALK_SPEED_MS, 0);
    serviceBackground();
    if (lineFound()) {
      Serial.println("[NAV] Line re-acquired after alignment walk");
      return true;
    }
  }
  Serial.println("[NAV] Alignment FAILED — line not found");
  return false;
}

// ─────────────────────────────────────────────────────────────────────────────
// LINE CENTERING — walk small steps to reach lineError ≈ 0
// Called after every turn to align the robot body squarely on the new line.
//
// Algorithm:
//   - Walk 1 step at a time (NAV_CENTER_STEP_MS ms, slow)
//   - After each step, read lineErrorMm() with PD steering
//   - Stop when |err| < NAV_CENTER_TOL_MM  OR  line lost  OR  maxSteps reached
//
// If the line is not found after the turn, it also tries navAlignToLine()
// first.
// ─────────────────────────────────────────────────────────────────────────────

#define NAV_CENTER_TOL_MM 8.0f // acceptable centering error (mm)
#define NAV_CENTER_STEP_MS 250 // ms per gait phase during centering (slow)
#define NAV_CENTER_MAX_STEPS 6 // max walk steps for centering

inline void navCenterOnLine() {
  // First make sure we can even see the line
  serviceBackground();
  if (!lineFound()) {
    Serial.println("[NAV] Center: no line — trying align first");
    navAlignToLine(NAV_ALIGN_MAX_STEPS);
    serviceBackground();
  }

  int femDelta[4];
  for (int l = 0; l < 4; l++)
    femDelta[l] = NAV_FEMUR_DELTA + WALK_FEMUR_TRIM[l];

  const uint8_t seq[6][2] = {{0, NAV_FL | NAV_BR},
                             {NAV_FL | NAV_BR, NAV_FL | NAV_BR},
                             {NAV_FL | NAV_BR, 0},
                             {0, NAV_FR | NAV_BL},
                             {NAV_FR | NAV_BL, NAV_FR | NAV_BL},
                             {NAV_FR | NAV_BL, 0}};

  float prevErr = 0;
  Serial.println("[NAV] CenterOnLine: start");

  for (int step = 0; step < NAV_CENTER_MAX_STEPS && !compStop; step++) {
    serviceBackground();
    float err = lineFound() ? lineErrorMm() : prevErr;
    Serial.printf("[NAV] Center step %d: err=%.1fmm\n", step, err);

    // Done if well-centered
    if (fabsf(err) < NAV_CENTER_TOL_MM && step > 0) {
      Serial.printf("[NAV] Centered! err=%.1fmm\n", err);
      break;
    }

    // PD steering correction for this step
    float dErr = err - prevErr;
    float correction = NAV_KP * err + NAV_KD * dErr;
    correction = constrain(correction, -(float)NAV_MAX_CORRECTION,
                           (float)NAV_MAX_CORRECTION);
    prevErr = err;

    int hipL = constrain(NAV_BASE_HIP_DELTA + (int)roundf(correction), 0, 45);
    int hipR = constrain(NAV_BASE_HIP_DELTA - (int)roundf(correction), 0, 45);

    // Execute one 6-phase gait cycle (slow for precision)
    for (int p = 0; p < 6 && !compStop; p++) {
      int t[4][3];
      navns::buildSteerPose(t, WALK_HIP_BASE, WALK_HIP_SIGN, hipL, hipR,
                            WALK_FEM_PLANT, FEMUR_LIFT_SIGN, femDelta,
                            seq[p][0], seq[p][1]);
      moveToTarget(t, NAV_CENTER_STEP_MS);
    }
  }

  // Return to standing, settled
  moveToPose(1, 300);
  waitMs(100);
  serviceBackground();
  Serial.printf("[NAV] CenterOnLine done: final err=%.1fmm\n",
                lineFound() ? lineErrorMm() : 999.0f);
}

// ─────────────────────────────────────────────────────────────────────────────
// CALIBRATED TURNS  (gyro-assisted + post-turn line centering)
//
// Full sequence for each 90° turn:
//   1. Mark gyro active (mpuSetStationary(false))
//   2. Record yaw BEFORE turn
//   3. Execute step-count turn (baseline dead-reckoning)
//   4. Wait 80ms for gyro filter to settle
//   5. Read actual turned angle; add correction steps if short by > tolerance
//   6. Mark gyro stationary (mpuSetStationary(true))
//   7. navCenterOnLine() — walk with PD until line error < 8mm
//
// If MPU is not ready, falls back to pure step-count + centering.
// ─────────────────────────────────────────────────────────────────────────────

inline bool navTurnLeft90() {
  if (compStop)
    return false;
  Serial.println("[NAV] Turn LEFT 90deg — gyro ON");

  // ── Step 1: Gyro ON ───────────────────────────────────────────────────
  mpuSetStationary(false);

  if (mpuIsReady()) {
    float yawBefore = mpuGetYaw();

    // ── Step 2: Primary step-count turn ──────────────────────────────
    turnGait(NAV_TURN_STEPS_90, NAV_TURN_SPEED_MS, false);
    serviceBackground();
    waitMs(80); // let EMA filter settle

    // ── Step 3: Measure actual rotation ──────────────────────────────
    float turned = mpuGetYaw() - yawBefore; // CCW is positive
    float remain = 90.0f - turned;
    Serial.printf("[NAV] Left90 gyro: turned=%.1f°  remain=%.1f°\n", turned,
                  remain);

    // ── Step 4: Fine correction if still short ────────────────────────
    if (remain > NAV_TURN_TOL_DEG && remain < 50.0f) {
      int xSteps = constrain((int)(remain / 12.0f) + 1, 1, 5);
      Serial.printf("[NAV] Left90 correction: +%d steps\n", xSteps);
      turnGait(xSteps, NAV_TURN_SPEED_MS, false);
      serviceBackground();
      waitMs(60);
    }
    float finalYaw = mpuGetYaw() - yawBefore;
    Serial.printf("[NAV] Left90 final: turned=%.1f° (target 90°)\n", finalYaw);

  } else {
    // Fallback: pure step-count
    Serial.println("[NAV] Left90 — MPU not ready, step-count only");
    turnGait(NAV_TURN_STEPS_90, NAV_TURN_SPEED_MS, false);
  }

  // ── Step 5: Gyro OFF → stationary drift correction ───────────────────
  mpuSetStationary(true);
  Serial.println("[NAV] Left90 — gyro OFF, centering on line");

  // ── Step 6: Center on line (walk until lineError ≈ 0) ────────────────
  navCenterOnLine();

  return !compStop;
}

inline bool navTurnRight90() {
  if (compStop)
    return false;
  Serial.println("[NAV] Turn RIGHT 90deg — gyro ON");

  // ── Step 1: Gyro ON ───────────────────────────────────────────────────
  mpuSetStationary(false);

  if (mpuIsReady()) {
    float yawBefore = mpuGetYaw();

    // ── Step 2: Primary step-count turn ──────────────────────────────
    turnGait(NAV_TURN_STEPS_90, NAV_TURN_SPEED_MS, true);
    serviceBackground();
    waitMs(80);

    // ── Step 3: Measure actual rotation ──────────────────────────────
    float turned = yawBefore - mpuGetYaw(); // CW = yaw decreases
    float remain = 90.0f - turned;
    Serial.printf("[NAV] Right90 gyro: turned=%.1f°  remain=%.1f°\n", turned,
                  remain);

    // ── Step 4: Fine correction if still short ────────────────────────
    if (remain > NAV_TURN_TOL_DEG && remain < 50.0f) {
      int xSteps = constrain((int)(remain / 12.0f) + 1, 1, 5);
      Serial.printf("[NAV] Right90 correction: +%d steps\n", xSteps);
      turnGait(xSteps, NAV_TURN_SPEED_MS, true);
      serviceBackground();
      waitMs(60);
    }
    float finalYaw = yawBefore - mpuGetYaw();
    Serial.printf("[NAV] Right90 final: turned=%.1f° (target 90°)\n", finalYaw);

  } else {
    Serial.println("[NAV] Right90 — MPU not ready, step-count only");
    turnGait(NAV_TURN_STEPS_90, NAV_TURN_SPEED_MS, true);
  }

  // ── Step 5: Gyro OFF ─────────────────────────────────────────────────
  mpuSetStationary(true);
  Serial.println("[NAV] Right90 — gyro OFF, centering on line");

  // ── Step 6: Center on line ────────────────────────────────────────────
  navCenterOnLine();

  return !compStop;
}

inline bool navTurn180() {
  if (compStop)
    return false;
  Serial.println("[NAV] Turn 180deg — gyro ON");

  // ── Step 1: Gyro ON ───────────────────────────────────────────────────
  mpuSetStationary(false);

  if (mpuIsReady()) {
    float yawBefore = mpuGetYaw();

    // ── Step 2: Primary step-count turn ──────────────────────────────
    turnGait(NAV_TURN_STEPS_180, NAV_TURN_SPEED_MS, true);
    serviceBackground();
    waitMs(80);

    // ── Step 3: Measure actual rotation ──────────────────────────────
    float turned = yawBefore - mpuGetYaw();
    float remain = 180.0f - turned;
    Serial.printf("[NAV] 180 gyro: turned=%.1f°  remain=%.1f°\n", turned,
                  remain);

    // ── Step 4: Fine correction ───────────────────────────────────────
    if (remain > NAV_TURN_TOL_DEG && remain < 90.0f) {
      int xSteps = constrain((int)(remain / 20.0f) + 1, 1, 6);
      Serial.printf("[NAV] 180 correction: +%d steps\n", xSteps);
      turnGait(xSteps, NAV_TURN_SPEED_MS, true);
      serviceBackground();
      waitMs(60);
    }
    float finalYaw = yawBefore - mpuGetYaw();
    Serial.printf("[NAV] 180 final: turned=%.1f° (target 180°)\n", finalYaw);

  } else {
    Serial.println("[NAV] 180 — MPU not ready, step-count only");
    turnGait(NAV_TURN_STEPS_180, NAV_TURN_SPEED_MS, true);
  }

  // ── Step 5: Gyro OFF ─────────────────────────────────────────────────
  mpuSetStationary(true);
  Serial.println("[NAV] 180 — gyro OFF, centering on line");

  // ── Step 6: Center on line ────────────────────────────────────────────
  navCenterOnLine();

  return !compStop;
}

// Helper: turn to face a target heading given current heading
// heading: 0=N, 1=E, 2=S, 3=W.   Returns the new heading.
inline int navTurnToHeading(int currentHeading, int targetHeading) {
  if (compStop)
    return currentHeading;
  int diff = (targetHeading - currentHeading + 4) % 4;
  switch (diff) {
  case 0:
    break; // already facing target
  case 1:
    navTurnRight90();
    break; // 90° right
  case 2:
    navTurn180();
    break; // 180°
  case 3:
    navTurnLeft90();
    break; // 90° left (=270° right)
  }
  Serial.printf("[NAV] Heading: %d -> %d\n", currentHeading, targetHeading);
  return targetHeading;
}

// ─────────────────────────────────────────────────────────────────────────────
// WALK WITHOUT LINE FOLLOWING (dead reckoning, straight walk)
// ─────────────────────────────────────────────────────────────────────────────

inline bool navWalkSteps(int steps, int speedMs = NAV_WALK_SPEED_MS) {
  if (compStop)
    return false;
  Serial.printf("[NAV] Walk %d steps (speed=%dms)\n", steps, speedMs);
  walkForward(steps, speedMs, 0);
  return !compStop;
}

// Walk forward N grid cells using line following (stop on junction N times)
inline bool navWalkCells(int cellCount) {
  for (int c = 0; c < cellCount && !compStop; c++) {
    NavResult r = navLineFollowUntil(NAV_CYCLES_PER_CELL * 3,
                                     STOP_ON_JUNCTION | STOP_ON_ABORT);
    if (r.stoppedOnAbort())
      return false;
    if (!r.stoppedOnJunction() && c < cellCount - 1) {
      // Didn't reach junction but not last cell — walk more
      navWalkSteps(NAV_CYCLES_PER_CELL);
    }
  }
  return !compStop;
}

// ─────────────────────────────────────────────────────────────────────────────
// TOF SCANNING
// ─────────────────────────────────────────────────────────────────────────────

// Take multiple ToF readings and return averaged distance (mm).
// Returns 0 if sensor offline.
inline float navScanToF(int samples = 5, int delayPerSample = 50) {
  if (!tofOnline())
    return 0;
  float sum = 0;
  int count = 0;
  for (int i = 0; i < samples; i++) {
    serviceBackground();
    waitMs(delayPerSample);
    float d = tofFront();
    if (d > 10.0f && d < 1200.0f) {
      sum += d;
      count++;
    }
  }
  float avg = (count > 0) ? (sum / count) : 0;
  Serial.printf("[NAV] ToF scan: %.0fmm (%d/%d valid)\n", avg, count, samples);
  return avg;
}

// Check thresholds
inline bool navIsBallDetected(float tofMm) {
  return tofMm > 10.0f && tofMm < NAV_BALL_DETECT_MM;
}
inline bool navIsBallClose(float tofMm) {
  return tofMm > 10.0f && tofMm < NAV_BALL_CLOSE_MM;
}
inline bool navIsObstacleDetected(float tofMm) {
  return tofMm > 10.0f && tofMm < NAV_OBSTACLE_DETECT_MM;
}

// ─────────────────────────────────────────────────────────────────────────────
// COLOR DETECTION
// ─────────────────────────────────────────────────────────────────────────────

// Wait for a stable color reading.  Polls until colorConfirmed() returns a
// non-zero value or timeoutMs elapses.  Returns COLOR_ID_xxx (0 = unknown).
inline uint8_t navReadColor(uint32_t timeoutMs = 3000) {
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    serviceBackground();
    uint8_t c = colorConfirmed();
    if (c != 0) {
      Serial.printf("[NAV] Color: %s (id=%d)\n", colorName(c), c);
      return c;
    }
    waitMs(50);
  }
  Serial.println("[NAV] Color timeout — defaulting to UNKNOWN");
  return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// POSE HELPERS
// ─────────────────────────────────────────────────────────────────────────────

inline void navStandUp() {
  Serial.println("[NAV] Standing up (Pose 1)");
  moveToPose(1, 1000); // STANDING pose with 1s transition
  waitMs(500);         // settle
}

inline void navSitDown() {
  Serial.println("[NAV] Sitting down (Pose 0)");
  moveToPose(0, 1000); // INITIAL/HOME pose
  waitMs(500);
}
