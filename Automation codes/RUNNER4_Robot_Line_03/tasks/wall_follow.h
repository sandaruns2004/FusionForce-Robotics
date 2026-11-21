#pragma once
// =============================================================================
// wall_follow.h — SubTask #2: Curved Corridor with Wall Gaps (ToF Wall Following)
//
// STRATEGY:
//   1. Enter from SubTask 1 (top-left exit after stepping North out of grid).
//   2. Take a 90° LEFT turn to face WEST into the curved corridor entrance.
//   3. Advance forward while actively steering using the Left and Right ToF sensors:
//      - Threshold: 15 cm (150 mm).
//      - If a sensor reads > 15 cm, it is in a WALL GAP (out of range) and ignored!
//      - Both sensors <= 15 cm: Center between walls (error = left - right).
//      - One sensor > 15 cm: Discard it and follow the single valid wall at
//        target distance (~65 mm).
//      - Both sensors > 15 cm: Double gap, walk straight forward.
//   4. Exit condition: Detects junction/guideline leading to SubTask 3 or max cycles.
//
// DEPENDENCIES:
//   nav_primitives.h, sensors/tof_sensor.h
// =============================================================================

#include <Arduino.h>
#include "../nav/nav_primitives.h"

// ── Calibration Parameters ───────────────────────────────────────────────────
#define WALL_MAX_CYCLES            120     // Upper bound cycles to traverse corridor
#define WALL_GAP_THRESHOLD_MM      150.0f  // 15 cm: readings > this are treated as Gaps / Out-of-Range
#define WALL_TARGET_DIST_MM        65.0f   // Target distance to single wall (30cm corridor - ~17cm bot / 2)
#define WALL_KP                    0.25f   // Proportional gain for differential hip steering
#define WALL_KD                    0.12f   // Derivative gain
#define WALL_MAX_CORRECTION        15      // Max degrees to adjust hip delta (±15°)
#define WALL_WALK_SPEED_MS         220     // Trot step speed (ms per phase)

// ── Result ───────────────────────────────────────────────────────────────────
struct WallFollowResult {
    bool success;       // true if corridor traversed
    int  cyclesDone;    // total walk cycles completed
    int  gapEvents;     // number of cycles where a wall gap was bypassed
};

// =============================================================================
// PUBLIC API — runWallFollow()
// =============================================================================
inline WallFollowResult runWallFollow() {
    WallFollowResult result = {false, 0, 0};

    Serial.println("============================================================");
    Serial.println("[WALL] SubTask 2: CURVED CORRIDOR (ToF Wall Follow) — START");
    Serial.println("============================================================");

    // Step 1: Turn 90° LEFT to face WEST into the corridor entrance
    Serial.println("[WALL] Turning 90° LEFT into corridor entrance...");
    navTurnLeft90();
    waitMs(300);
    if (compStop) return result;

    // Settle gait base
    syncGaitBase();

    int femDelta[4];
    for (int l = 0; l < 4; l++) {
        femDelta[l] = NAV_FEMUR_DELTA + WALK_FEMUR_TRIM[l];
    }

    // 6-phase diagonal trot sequence
    const uint8_t seq[6][2] = {
        {0,                       NAV_FL | NAV_BR},   // 1: FL&BR lift
        {NAV_FL | NAV_BR,         NAV_FL | NAV_BR},   // 2: FL&BR swing
        {NAV_FL | NAV_BR,         0},                 // 3: FL&BR plant
        {0,                       NAV_FR | NAV_BL},   // 4: FR&BL lift
        {NAV_FR | NAV_BL,         NAV_FR | NAV_BL},   // 5: FR&BL swing
        {NAV_FR | NAV_BL,         0}                  // 6: FR&BL plant
    };

    float prevErr = 0;
    int totalCycles = 0;
    bool corridorDone = false;

    while (totalCycles < WALL_MAX_CYCLES && !corridorDone && !compStop) {
        // Sample Left & Right ToF sensors
        serviceBackground();
        float dL = tofLeft();
        float dR = tofRight();
        float dF = tofFront();

        // Check if either sensor is out of range / in a gap (> 150 mm)
        bool leftValid  = (dL > 10.0f && dL <= WALL_GAP_THRESHOLD_MM);
        bool rightValid = (dR > 10.0f && dR <= WALL_GAP_THRESHOLD_MM);

        float err = 0;

        if (leftValid && rightValid) {
            // Case 1: Both walls detected -> Center robot between them
            // dL > dR means closer to right wall -> err > 0 -> steer left
            err = (dL - dR);
        } else if (leftValid && !rightValid) {
            // Case 2: Right wall in gap (>15cm) -> Follow Left wall only
            result.gapEvents++;
            // If dL > target (drifting away from left wall) -> err > 0 -> steer left towards it
            // If dL < target (too close to left wall)       -> err < 0 -> steer right away
            err = 2.0f * (dL - WALL_TARGET_DIST_MM);
        } else if (!leftValid && rightValid) {
            // Case 3: Left wall in gap (>15cm) -> Follow Right wall only
            result.gapEvents++;
            // If dR < target (too close to right wall)      -> err > 0 -> steer left away
            // If dR > target (drifting away from right wall) -> err < 0 -> steer right towards it
            err = 2.0f * (WALL_TARGET_DIST_MM - dR);
        } else {
            // Case 4: Both walls in gap (>15cm) -> Walk straight forward
            result.gapEvents++;
            err = 0;
        }

        // PD Steering calculation
        float dErr = err - prevErr;
        float corr = WALL_KP * err + WALL_KD * dErr;
        corr = constrain(corr, -(float)WALL_MAX_CORRECTION, (float)WALL_MAX_CORRECTION);
        prevErr = err;

        int hipL = constrain(NAV_BASE_HIP_DELTA + (int)roundf(corr), 0, 45);
        int hipR = constrain(NAV_BASE_HIP_DELTA - (int)roundf(corr), 0, 45);

        // Execute one full 6-phase trot cycle with differential steering
        for (int p = 0; p < 6 && !compStop; p++) {
            int t[4][3];
            navns::buildSteerPose(t, WALK_HIP_BASE, WALK_HIP_SIGN, hipL, hipR,
                                  WALK_FEM_PLANT, FEMUR_LIFT_SIGN, femDelta,
                                  seq[p][0], seq[p][1]);
            moveToTarget(t, WALL_WALK_SPEED_MS);
        }

        totalCycles++;

        // Log diagnostics periodically
        if (totalCycles % 5 == 0) {
            Serial.printf("[WALL] Cycle %d: L=%.0f%s R=%.0f%s -> err=%.1f corr=%.1f (L:%d R:%d)\n",
                          totalCycles, dL, leftValid?"":"(GAP)", dR, rightValid?"":"(GAP)",
                          err, corr, hipL, hipR);
        }

        // Check for Corridor Exit Conditions:
        // 1. Line sensor sees junction or guideline leading into SubTask 3
        if (totalCycles > 15 && lineJunction()) {
            Serial.printf("[WALL] Line junction seen (%s) — corridor exit reached!\n", lineJunctionType());
            corridorDone = true;
            result.success = true;
            break;
        }

        // 2. Front ToF detects obstacle / end wall
        if (totalCycles > 20 && dF > 10.0f && dF < 160.0f) {
            Serial.printf("[WALL] Front sensor detected obstacle/end (%.0f mm) — transition to SubTask 3!\n", dF);
            corridorDone = true;
            result.success = true;
            break;
        }
    }

    if (totalCycles >= WALL_MAX_CYCLES) {
        Serial.println("[WALL] Max cycles reached — assuming corridor completed");
        result.success = true;
    }

    result.cyclesDone = totalCycles;
    moveToPose(1, 400); // Stand pose

    Serial.println("============================================================");
    Serial.printf("[WALL] SubTask 2: %s (cycles=%d, gapEvents=%d)\n",
                  result.success ? "COMPLETE" : "FAILED",
                  totalCycles, result.gapEvents);
    Serial.println("============================================================");
    return result;
}
