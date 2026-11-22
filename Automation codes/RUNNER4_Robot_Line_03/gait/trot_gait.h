#pragma once
// =============================================================================
// gait/trot_gait.h — Dynamic Diagonal Trot Gait (Walk, Turn, Sweep)
//
// 6-phase dynamic diagonal trot gait:
//   - Pair A: FL + BR
//   - Pair B: FR + BL
//
// Walking uses WALK_HIP_DELTA & WALK_FEMUR_DELTA.
// Turning uses TURN_HIP_DELTA & TURN_FEMUR_DELTA with directional signs.
// All base angles are synchronized to the STANDING pose (pose 1).
// =============================================================================

#include "../config/robot_config.h"
#include "servo_controller.h"
#include <Arduino.h>
#include <limits.h>

// -- Leg Bitmasks -------------------------------------------------------------
#ifndef M_FL
#define M_FL 1
#define M_FR 2
#define M_BL 4
#define M_BR 8
#endif

// -- Gait Parameters (editable dynamically via web UI) ------------------------
// Leg order: 0=FL, 1=FR, 2=BL, 3=BR
int WALK_HIP_DELTA =
    15; // hip swing, degrees (reduced from 30 for smooth, tight stride)
int WALK_FEMUR_DELTA = 20; // femur lift, degrees
const int WALK_FEMUR_TRIM[4] = {0, 0, 0, 0};

int TURN_HIP_DELTA =
    10; // hip swing, degrees (reduced from 15 for tight, controlled turn)
int TURN_FEMUR_DELTA =
    15; // femur lift, degrees (reduced from 20 for smoother rotation)

// Direction signs (FL, FR, BL, BR)
const int WALK_HIP_SIGN[4] = {-1, +1, -1, +1};
const int FEMUR_LIFT_SIGN[4] = {+1, -1, -1, +1};

// Offsets relative to STANDING pose (pose 1)
const int WALK_FEM_OFFSET[4] = {0, 0, 0, 0};
const int TURN_FEM_OFFSET[4] = {0, 0, 0, 0};
const int GAIT_TIB_OFFSET[4] = {0, 0, 0, 0};

// Working base angle buffers
int WALK_HIP_BASE[4], WALK_FEM_PLANT[4];
int TURN_HIP_BASE[4], TURN_FEM_PLANT[4];
int GAIT_TIBIA[4];

// -- External Runtime Motion State (defined in main .ino) ---------------------
extern volatile bool walkRunning, walkStop;
extern int walkStepsTarget, walkStepsDone;
extern volatile bool turnRunning, turnStop;
extern int turnStepsTarget, turnStepsDone;
extern char turnDirection[6];
extern char turnPhaseStr[48];
extern float turnTargetAngle;
extern float turnAngleDone;

// Gyro & system functions (forward declared; defined in mpu6050.h / main .ino)
float mpuGetYaw();
bool mpuIsReady();
void mpuSetStationary(bool stationary);
void serviceBackground();
void waitMs(uint32_t ms);

// =============================================================================
// GAIT BASE SYNCHRONIZATION
// =============================================================================
// Recomputes plant/base angles from STANDING pose (pose 1) each time a gait
// starts.
inline void syncGaitBase() {
  for (int l = 0; l < 4; l++) {
    WALK_HIP_BASE[l] = TURN_HIP_BASE[l] = poseAngles[1][l][0];
    WALK_FEM_PLANT[l] = poseAngles[1][l][1] + WALK_FEM_OFFSET[l];
    TURN_FEM_PLANT[l] = poseAngles[1][l][1] + TURN_FEM_OFFSET[l];
    GAIT_TIBIA[l] = poseAngles[1][l][2] + GAIT_TIB_OFFSET[l];
  }
}

// =============================================================================
// GAIT POSE BUILDER
// =============================================================================
// Build one 4x3 target pose.
//  swingMask : legs whose hip is moved by hipSign * hipDelta
//  liftMask  : legs whose femur is moved by femSign * femDelta
inline void buildGaitPose(int out[4][3], const int hipBase[4],
                          const int hipSign[4], int hipDelta,
                          const int femPlant[4], const int femSign[4],
                          const int femDelta[4], uint8_t swingMask,
                          uint8_t liftMask) {
  for (int leg = 0; leg < 4; leg++) {
    out[leg][0] =
        hipBase[leg] + (((swingMask >> leg) & 1) ? hipSign[leg] * hipDelta : 0);
    out[leg][1] = femPlant[leg] +
                  (((liftMask >> leg) & 1) ? femSign[leg] * femDelta[leg] : 0);
    out[leg][2] = GAIT_TIBIA[leg];
    out[leg][0] = constrain(out[leg][0], 0, 180);
    out[leg][1] = constrain(out[leg][1], 0, 180);
  }
}

// =============================================================================
// WALK GAIT
// =============================================================================
inline void walkForward(int steps, int speedMs, int variant) {
  syncGaitBase(); // gait follows the STANDING pose
  walkRunning = true;
  walkStop = false;
  walkStepsDone = 0;
  int totalCycles = (steps == 0) ? INT_MAX : steps;

  int femDelta[4];
  for (int l = 0; l < 4; l++)
    femDelta[l] = WALK_FEMUR_DELTA + WALK_FEMUR_TRIM[l];

  // {swingMask, liftMask} for the 6 phases
  const uint8_t seq[6][2] = {
      {0, M_FL | M_BR},           // 1: FL&BR femurs lift
      {M_FL | M_BR, M_FL | M_BR}, // 2: swing FL&BR hips
      {M_FL | M_BR, 0},           // 3: plant FL&BR femurs
      {0, M_FR | M_BL},           // 4: FL&BR hips back, FR&BL lift
      {M_FR | M_BL, M_FR | M_BL}, // 5: swing FR&BL hips
      {M_FR | M_BL, 0}            // 6: plant FR&BL femurs
  };

  for (int i = 0; i < totalCycles && !walkStop; i++) {
    for (int p = 0; p < 6 && !walkStop; p++) {
      int t[4][3];
      buildGaitPose(t, WALK_HIP_BASE, WALK_HIP_SIGN, WALK_HIP_DELTA,
                    WALK_FEM_PLANT, FEMUR_LIFT_SIGN, femDelta, seq[p][0],
                    seq[p][1]);
      moveToTarget(t, speedMs);
    }
    if (walkStop)
      break;
    walkStepsDone++;
    Serial.printf("[WALK] Step %d/%d done\n", walkStepsDone,
                  (steps == 0) ? -1 : steps);
  }
  moveToPose(1, 500); // Return to STANDING pose
  walkRunning = false;
  Serial.printf("[WALK] Complete. Steps=%d Variant=%s\n", walkStepsDone,
                variant == 0 ? "Basic" : "Enhanced");
}

// =============================================================================
// TURN GAIT (6-Phase Symmetrical Turn)
// =============================================================================
inline void turnGait(int steps, int speedMs, bool rightTurn) {
  syncGaitBase(); // gait follows the STANDING pose
  turnRunning = true;
  turnStop = false;
  turnStepsDone = 0;
  strncpy(turnDirection, rightTurn ? "right" : "left", sizeof(turnDirection));

  const int dir = rightTurn ? -1 : +1;
  const int turnHipSign[4] = {dir, dir, dir, dir};
  const int femDelta[4] = {TURN_FEMUR_DELTA, TURN_FEMUR_DELTA, TURN_FEMUR_DELTA,
                           TURN_FEMUR_DELTA};

  // {swingMask, liftMask} for the 6 phases
  const uint8_t seq[6][2] = {
      {0, M_FL | M_BR},           // 1: FL&BR lift, all hips at base
      {M_FL | M_BR, M_FL | M_BR}, // 2: swing FL&BR hips
      {M_FL | M_BR, 0},           // 3: put down FL&BR femurs
      {0, M_FR | M_BL},           // 4: FR&BL lift, FL&BR hips return to base
      {M_FR | M_BL, M_FR | M_BL}, // 5: swing FR&BL hips
      {M_FR | M_BL, 0}            // 6: put down FR&BL femurs
  };
  const char *names[6] = {
      "1: FL+BR lift / FR+BL back", "2: FL+BR swing", "3: FL+BR put down",
      "4: FR+BL lift / FL+BR back", "5: FR+BL swing", "6: FR+BL put down"};

  // First go to the STANDING pose
  strncpy(turnPhaseStr, "standing", sizeof(turnPhaseStr));
  moveToPose(1, 500);

  for (int i = 0; i < steps && !turnStop; i++) {
    for (int p = 0; p < 6 && !turnStop; p++) {
      strncpy(turnPhaseStr, names[p], sizeof(turnPhaseStr));
      int t[4][3];
      buildGaitPose(t, TURN_HIP_BASE, turnHipSign, TURN_HIP_DELTA,
                    TURN_FEM_PLANT, FEMUR_LIFT_SIGN, femDelta, seq[p][0],
                    seq[p][1]);
      moveToTarget(t, speedMs);
    }
    if (turnStop)
      break;
    turnStepsDone++;
    Serial.printf("[TURN] %s step %d/%d done\n", rightTurn ? "Right" : "Left",
                  turnStepsDone, steps);
  }
  strncpy(turnPhaseStr, "done", sizeof(turnPhaseStr));
  moveToPose(1, 500); // Return to STANDING pose
  turnRunning = false;
  Serial.printf("[TURN] %s complete. Steps=%d\n", rightTurn ? "Right" : "Left",
                turnStepsDone);
}

// =============================================================================
// TURN GYRO GAIT (Closed-Loop Gyro-Targeted Turn)
// =============================================================================
// Rotates the robot using MPU6050 yaw feedback until targetDeg is reached.
// Evaluates rotation at each half-cycle (phases 2 and 5 when all 4 feet are
// planted) for clean, high-precision stops.
inline void turnGyroGait(float targetDeg, int speedMs, bool rightTurn) {
  syncGaitBase(); // gait follows the STANDING pose
  turnRunning = true;
  turnStop = false;
  turnStepsDone = 0;
  turnTargetAngle = targetDeg;
  turnAngleDone = 0.0f;
  strncpy(turnDirection, rightTurn ? "right" : "left", sizeof(turnDirection));

  if (!mpuIsReady()) {
    Serial.println(
        "[TURN-GYRO] MPU not ready! Falling back to step-based turn.");
    int estSteps = constrain((int)roundf(targetDeg / 12.0f), 1, 30);
    turnGait(estSteps, speedMs, rightTurn);
    return;
  }

  const int dir = rightTurn ? -1 : +1;
  const int turnHipSign[4] = {dir, dir, dir, dir};
  const int femDelta[4] = {TURN_FEMUR_DELTA, TURN_FEMUR_DELTA, TURN_FEMUR_DELTA,
                           TURN_FEMUR_DELTA};

  // {swingMask, liftMask} for the 6 phases
  const uint8_t seq[6][2] = {
      {0, M_FL | M_BR},           // 1: FL&BR lift, all hips at base
      {M_FL | M_BR, M_FL | M_BR}, // 2: swing FL&BR hips
      {M_FL | M_BR, 0},           // 3: put down FL&BR femurs (all feet planted)
      {0, M_FR | M_BL},           // 4: FR&BL lift, FL&BR hips return to base
      {M_FR | M_BL, M_FR | M_BL}, // 5: swing FR&BL hips
      {M_FR | M_BL, 0}            // 6: put down FR&BL femurs (all feet planted)
  };
  const char *names[6] = {"1: FL+BR lift", "2: FL+BR swing", "3: FL+BR plant",
                          "4: FR+BL lift", "5: FR+BL swing", "6: FR+BL plant"};

  // First ensure robot is in STANDING pose
  strncpy(turnPhaseStr, "standing", sizeof(turnPhaseStr));
  moveToPose(1, 400);

  // Enable dynamic tracking on MPU (bypass stationary drift filter)
  mpuSetStationary(false);
  waitMs(80);

  float startYaw = mpuGetYaw();
  const float tolDeg = 3.0f;
  const int maxCycles = 30;
  uint32_t tStart = millis();

  Serial.printf("[TURN-GYRO] Target=%.1f deg (%s), startYaw=%.1f\n", targetDeg,
                rightTurn ? "Right/CW" : "Left/CCW", startYaw);

  bool reached = false;
  for (int i = 0; i < maxCycles && !turnStop && !reached; i++) {
    for (int p = 0; p < 6 && !turnStop; p++) {
      float currentYaw = mpuGetYaw();
      float turned =
          rightTurn ? (startYaw - currentYaw) : (currentYaw - startYaw);
      turnAngleDone = turned;
      snprintf(turnPhaseStr, sizeof(turnPhaseStr), "C%d %s (%.1f/%.0f)", i + 1,
               names[p], turned, targetDeg);

      int t[4][3];
      buildGaitPose(t, TURN_HIP_BASE, turnHipSign, TURN_HIP_DELTA,
                    TURN_FEM_PLANT, FEMUR_LIFT_SIGN, femDelta, seq[p][0],
                    seq[p][1]);
      moveToTarget(t, speedMs);

      // Check angle when all 4 feet are planted on the ground (phases 2 and 5)
      if (p == 2 || p == 5) {
        currentYaw = mpuGetYaw();
        turned = rightTurn ? (startYaw - currentYaw) : (currentYaw - startYaw);
        turnAngleDone = turned;
        if (turned >= targetDeg - tolDeg) {
          Serial.printf("[TURN-GYRO] Target reached at phase %d: turned=%.1f "
                        "deg (target=%.1f)\n",
                        p + 1, turned, targetDeg);
          reached = true;
          break;
        }
      }
    }
    if (turnStop)
      break;
    turnStepsDone++;
    if (millis() - tStart > 60000) {
      Serial.println("[TURN-GYRO] Safety timeout (60s) reached.");
      break;
    }
  }

  // Restore stationary drift filter
  mpuSetStationary(true);

  // Return cleanly to STANDING pose
  moveToPose(1, 400);
  turnRunning = false;

  float finalYaw = mpuGetYaw();
  float finalTurned = rightTurn ? (startYaw - finalYaw) : (finalYaw - startYaw);
  turnAngleDone = finalTurned;
  snprintf(turnPhaseStr, sizeof(turnPhaseStr), "Done: %.1f deg (%d cyc)",
           finalTurned, turnStepsDone);
  Serial.printf(
      "[TURN-GYRO] Finished: turned=%.1f deg target=%.1f deg cycles=%d\n",
      finalTurned, targetDeg, turnStepsDone);
}

// =============================================================================
// SWEEP TEST (Single Joint Movement Diagnostic)
// =============================================================================
inline void sweepTest() {
  Serial.println("[SWEEP] BR FEMUR (CH8): 0 -> 90 -> 0");
  for (int a = 0; a <= 90; a++) {
    setServo(BR_FEMUR, a);
    waitMs(15);
  }
  waitMs(200);
  for (int a = 90; a >= 0; a--) {
    setServo(BR_FEMUR, a);
    waitMs(15);
  }
  waitMs(200);
  setLegHome(3);
  Serial.println("[SWEEP] Done.");
}
