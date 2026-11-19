#pragma once
// =============================================================================
// wall_follow.h — SubTask #2: Curved Corridor with Wall Gaps
//
// STRATEGY:
//   The corridor has a white guideline on the floor between two curved walls.
//   The robot follows the line using PD-controlled line-following (same as the
//   tuning lab).  Wall gaps don't affect the floor line, so the robot walks
//   straight through gap sections without any special logic.
//
//   If the line is temporarily lost (e.g., gap in the floor markings near a
//   wall gap), the robot walks forward slowly to re-acquire it.
//
//   The corridor ends when the robot exits to the SubTask 3 area.  This is
//   detected by a junction or by the line ending.
//
// DEPENDENCIES:
//   nav_primitives.h — line following, alignment
// =============================================================================

#include <Arduino.h>
#include "../nav/nav_primitives.h"

// ── Calibration ──────────────────────────────────────────────────────────────
// Max cycles to traverse the full corridor (generous upper bound)
#define WALL_MAX_CYCLES          200
// Max walk steps during line-recovery after a gap
#define WALL_RECOVER_MAX_STEPS   5
// Speed override for corridor (slightly slower for curved walls)
// Note: uses NAV_WALK_SPEED_MS by default; change if needed

// ── Result ───────────────────────────────────────────────────────────────────
struct WallFollowResult {
    bool success;       // true if corridor traversed
    int  cyclesDone;    // total walk cycles
    int  lostEvents;    // how many times the line was temporarily lost
};

// =============================================================================
// PUBLIC API — runWallFollow()
// =============================================================================
// Runs SubTask #2: traverse the curved corridor with wall gaps.
//
// BLOCKING: does not return until the corridor is traversed or aborted.
//
inline WallFollowResult runWallFollow() {
    WallFollowResult result = {false, 0, 0};

    Serial.println("============================================================");
    Serial.println("[WALL] SubTask 2: CURVED CORRIDOR — START");
    Serial.println("============================================================");

    // The robot enters the corridor already on the guideline, heading in the
    // direction of travel (should be set by the grid exit from SubTask 1).

    // Main corridor traverse loop:
    // Keep line-following until the line ends (exit point) or abort.
    // If the line is temporarily lost, try to recover by walking forward.
    int totalCycles = 0;
    bool corridorDone = false;

    while (!corridorDone && !compStop) {
        // Line follow with a generous cycle count.
        // Stop on: line lost (gap or exit), junction (might indicate exit), or abort.
        NavResult r = navLineFollowUntil(
            WALL_MAX_CYCLES,
            STOP_ON_LINE_LOST | STOP_ON_JUNCTION | STOP_ON_ABORT | STOP_ON_COUNT);

        totalCycles += r.cyclesDone;

        if (r.stoppedOnAbort()) {
            break;
        }

        if (r.stoppedOnJunction()) {
            // A junction in the corridor likely indicates the EXIT point
            // (transition to SubTask 3 area).
            Serial.printf("[WALL] Junction detected: %s — likely exit\n", r.juncType);
            corridorDone = true;
            result.success = true;
            break;
        }

        if (r.reason & STOP_ON_COUNT) {
            // Reached max cycles without any stop condition — corridor traversed
            Serial.println("[WALL] Max cycles reached — assuming corridor done");
            corridorDone = true;
            result.success = true;
            break;
        }

        if (r.stoppedOnLost()) {
            result.lostEvents++;
            Serial.printf("[WALL] Line lost (event #%d) — attempting recovery\n",
                          result.lostEvents);

            // Try to re-acquire the line by walking forward
            bool recovered = navAlignToLine(WALL_RECOVER_MAX_STEPS);
            if (recovered) {
                Serial.println("[WALL] Line recovered — continuing");
                continue;   // resume line-following
            }

            // If line not recovered after walking forward, the corridor might
            // have ended (we're at the exit).
            if (result.lostEvents >= 3) {
                Serial.println("[WALL] Multiple lost events — assuming corridor exit");
                corridorDone = true;
                result.success = true;
                break;
            }

            // Try walking a few more steps forward
            navWalkSteps(3, NAV_WALK_SPEED_MS);
            if (lineFound()) {
                Serial.println("[WALL] Line found after extra walk — continuing");
                continue;
            }

            // Still no line — assume exit
            Serial.println("[WALL] Line not found — corridor exit assumed");
            corridorDone = true;
            result.success = true;
            break;
        }
    }

    result.cyclesDone = totalCycles;

    Serial.println("============================================================");
    Serial.printf("[WALL] SubTask 2: %s (cycles=%d, lostEvents=%d)\n",
                  result.success ? "COMPLETE" : "FAILED",
                  totalCycles, result.lostEvents);
    Serial.println("============================================================");
    return result;
}
