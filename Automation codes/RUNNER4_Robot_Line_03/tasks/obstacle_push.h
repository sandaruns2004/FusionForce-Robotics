#pragma once
// =============================================================================
// obstacle_push.h — SubTask #3: Straight Corridor with Pushable Obstacle
//
// STRATEGY:
//   1. Line-follow through the straight corridor (30 cm wide, parallel walls)
//   2. Front ToF detects the obstacle (25×25×20 cm box) ahead
//   3. Continue walking forward to physically push the obstacle clear
//   4. After pushing, detect a left-turn junction
//   5. Turn left and line-follow to the exit point
//
// DEPENDENCIES:
//   nav_primitives.h — line following, turns, ToF detection
// =============================================================================

#include <Arduino.h>
#include "../nav/nav_primitives.h"

// ── Calibration ──────────────────────────────────────────────────────────────
// Max cycles to traverse the corridor before obstacle
#define PUSH_CORRIDOR_MAX_CYCLES   60
// ToF threshold for obstacle detection (mm)
#define PUSH_OBSTACLE_DETECT_MM    180.0f
// Walk steps to push the obstacle out of the way (needs calibration)
// The box is 25cm deep; each walk step is ~4-5cm; need 6-8 steps to push clear
#define PUSH_FORWARD_STEPS         8
// Speed for pushing (slower = more force, less slip)
#define PUSH_SPEED_MS              300
// Max cycles to find the left turn junction after pushing
#define PUSH_FIND_TURN_MAX_CYCLES  30
// Max cycles to follow exit line after turning
#define PUSH_EXIT_MAX_CYCLES       60

// ── Result ───────────────────────────────────────────────────────────────────
struct ObstaclePushResult {
    bool success;           // true if obstacle cleared and exit reached
    bool obstacleDetected;  // true if ToF detected the obstacle
    int  cyclesDone;        // total walk cycles
};

// =============================================================================
// PUBLIC API — runObstaclePush()
// =============================================================================
// Runs SubTask #3: straight corridor, push obstacle, turn left, exit.
//
// BLOCKING: does not return until the task is complete or aborted.
//
inline ObstaclePushResult runObstaclePush() {
    ObstaclePushResult result = {false, false, 0};

    Serial.println("============================================================");
    Serial.println("[PUSH] SubTask 3: OBSTACLE PUSH — START");
    Serial.println("============================================================");

    // =====================================================================
    // PHASE 1: Line-follow through corridor until obstacle detected
    // =====================================================================
    Serial.println("[PUSH] Phase 1: Following corridor, scanning for obstacle...");

    NavResult r = navLineFollowUntil(
        PUSH_CORRIDOR_MAX_CYCLES,
        STOP_ON_TOF_CLOSE | STOP_ON_ABORT | STOP_ON_LINE_LOST | STOP_ON_COUNT,
        PUSH_OBSTACLE_DETECT_MM);

    result.cyclesDone += r.cyclesDone;

    if (compStop) goto done;

    if (r.stoppedOnTof()) {
        result.obstacleDetected = true;
        Serial.printf("[PUSH] Obstacle detected at %.0fmm!\n", r.lastToF);
    } else {
        // No obstacle detected — continue anyway (might have been pushed already
        // in a previous attempt, or ToF didn't detect it)
        Serial.println("[PUSH] No obstacle detected — continuing through corridor");
    }

    // =====================================================================
    // PHASE 2: Push the obstacle forward
    // =====================================================================
    if (result.obstacleDetected) {
        Serial.println("[PUSH] Phase 2: Pushing obstacle...");

        // Walk forward firmly to push the box
        // Use straight walking (not line-following) for consistent push force
        navWalkSteps(PUSH_FORWARD_STEPS, PUSH_SPEED_MS);

        if (compStop) goto done;

        Serial.println("[PUSH] Push complete");
    }

    // =====================================================================
    // PHASE 3: Find and execute the left turn
    // =====================================================================
    Serial.println("[PUSH] Phase 3: Looking for left turn junction...");

    // Continue line-following until a junction is detected (the left turn point)
    r = navLineFollowUntil(
        PUSH_FIND_TURN_MAX_CYCLES,
        STOP_ON_JUNCTION | STOP_ON_ABORT | STOP_ON_LINE_LOST | STOP_ON_COUNT);

    result.cyclesDone += r.cyclesDone;

    if (compStop) goto done;

    if (r.stoppedOnJunction()) {
        Serial.printf("[PUSH] Turn junction found: %s\n", r.juncType);
    } else {
        // Junction not found — try walking more
        Serial.println("[PUSH] No junction found — walking further...");
        r = navLineFollowUntil(
            PUSH_FIND_TURN_MAX_CYCLES,
            STOP_ON_JUNCTION | STOP_ON_ABORT | STOP_ON_COUNT);
        result.cyclesDone += r.cyclesDone;
        if (compStop) goto done;
    }

    // Execute left turn
    Serial.println("[PUSH] Turning LEFT...");
    navTurnLeft90();
    navAlignToLine();

    if (compStop) goto done;

    // =====================================================================
    // PHASE 4: Follow exit line to the end of SubTask 3
    // =====================================================================
    Serial.println("[PUSH] Phase 4: Following exit line...");

    r = navLineFollowUntil(
        PUSH_EXIT_MAX_CYCLES,
        STOP_ON_LINE_LOST | STOP_ON_JUNCTION | STOP_ON_ABORT | STOP_ON_COUNT);

    result.cyclesDone += r.cyclesDone;

    if (r.stoppedOnJunction()) {
        // Junction at exit — might be the entry to SubTask 4
        Serial.printf("[PUSH] Exit junction: %s\n", r.juncType);
    }

    result.success = !compStop;

done:
    Serial.println("============================================================");
    Serial.printf("[PUSH] SubTask 3: %s (obstacle=%s, cycles=%d)\n",
                  result.success ? "COMPLETE" : "FAILED",
                  result.obstacleDetected ? "yes" : "no",
                  result.cyclesDone);
    Serial.println("============================================================");
    return result;
}
