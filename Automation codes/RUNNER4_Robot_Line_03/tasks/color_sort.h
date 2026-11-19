#pragma once
// =============================================================================
// color_sort.h — SubTask #4: Color Sorting Junction
//
// STRATEGY:
//   The robot follows the main white guideline to a 3-way color-sorting
//   junction.  At this junction, 3 colored lines (Red, Blue, Green) branch
//   off to the LEFT side of the main path.  Each leads to a separate sorting
//   segment separated by walls.
//
//   The robot must:
//   1. Follow the white line to the junction area
//   2. Identify which colored branch matches the ball color (stored from Task 1)
//   3. Turn LEFT onto that specific colored line
//   4. Follow the colored line to the end of the sorting segment
//   5. Place the ball using the arm
//   6. Return to the main white line
//   7. Continue to the FINISH point
//
// COLOR BRANCH ORDER (from the arena diagram):
//   As the robot approaches from the right, the branches encountered are:
//     1st junction = GREEN (rightmost branch)
//     2nd junction = BLUE  (middle branch)
//     3rd junction = RED   (leftmost branch)
//   *** THIS MUST BE VERIFIED ON THE ACTUAL ARENA ***
//   Change COLOR_SORT_ORDER[] below if the order is different.
//
// DEPENDENCIES:
//   nav_primitives.h — line following, turns, color detection
//   arm_interface.h  — place ball
// =============================================================================

#include <Arduino.h>
#include "../nav/nav_primitives.h"
#include "../arm/arm_interface.h"

// -- Color branch order configuration -----------------------------------------
// Order of colored branches as the robot encounters them while walking along
// the main white line.  Index 0 = first branch encountered.
// Values: COLOR_ID_GREEN=2, COLOR_ID_BLUE=3, COLOR_ID_RED=1
// (from color_sensor.h: enum ColorId)
#ifndef COLOR_SORT_BRANCH_1
#define COLOR_SORT_BRANCH_1  2   // COLOR_ID_GREEN — first branch (rightmost)
#endif
#ifndef COLOR_SORT_BRANCH_2
#define COLOR_SORT_BRANCH_2  3   // COLOR_ID_BLUE  — second branch (middle)
#endif
#ifndef COLOR_SORT_BRANCH_3
#define COLOR_SORT_BRANCH_3  1   // COLOR_ID_RED   — third branch (leftmost)
#endif

// -- Calibration --------------------------------------------------------------
// Max cycles to reach the junction area from corridor exit
#define SORT_APPROACH_MAX_CYCLES   80
// Max cycles between color branch junctions
#define SORT_INTER_BRANCH_CYCLES   20
// Max cycles to follow a colored line to its end
#define SORT_COLOR_LINE_CYCLES     20
// Max cycles to return from a branch to the main line
#define SORT_RETURN_MAX_CYCLES     20
// Max cycles to follow the main line to the FINISH point
#define SORT_FINISH_MAX_CYCLES     80

// -- Result -------------------------------------------------------------------
struct ColorSortResult {
    bool    success;        // true if ball placed and finish reached
    bool    ballPlaced;     // true if armPlaceBall succeeded
    uint8_t targetColor;    // the color we sorted to
    int     branchIndex;    // which branch (1, 2, or 3) was used
    int     cyclesDone;     // total walk cycles
};

// =============================================================================
// INTERNAL HELPERS
// =============================================================================
namespace sortns {

// Determine which branch number (1–3) matches the target ball color.
// Returns 0 if the color doesn't match any branch (error).
static int colorToBranch(uint8_t ballColor) {
    if (ballColor == COLOR_SORT_BRANCH_1) return 1;
    if (ballColor == COLOR_SORT_BRANCH_2) return 2;
    if (ballColor == COLOR_SORT_BRANCH_3) return 3;
    Serial.printf("[SORT] WARNING: Unknown ball color %d — defaulting to branch 2\n",
                  ballColor);
    return 2;  // default to middle branch
}

} // namespace sortns

// =============================================================================
// PUBLIC API — runColorSort()
// =============================================================================
// Runs SubTask #4: color sorting at the 3-way junction.
//
// Parameters:
//   ballColor — the ball color from SubTask 1 (COLOR_ID_RED/GREEN/BLUE).
//               If 0 (unknown), defaults to the middle branch.
//
// BLOCKING: does not return until the task is complete or aborted.
//
inline ColorSortResult runColorSort(uint8_t ballColor) {
    ColorSortResult result = {false, false, ballColor, 0, 0};

    Serial.println("============================================================");
    Serial.printf("[SORT] SubTask 4: COLOR SORT — START (target=%d %s)\n",
                  ballColor, ballColor ? colorName(ballColor) : "UNKNOWN");
    Serial.println("============================================================");

    // Determine which branch to take
    int targetBranch = sortns::colorToBranch(ballColor);
    result.branchIndex = targetBranch;
    Serial.printf("[SORT] Target branch: %d (1=first, 2=middle, 3=last)\n",
                  targetBranch);

    // =====================================================================
    // PHASE 1: Follow main white line toward the junction area
    // =====================================================================
    Serial.println("[SORT] Phase 1: Following main line to junction area...");

    NavResult r = navLineFollowUntil(
        SORT_APPROACH_MAX_CYCLES,
        STOP_ON_JUNCTION | STOP_ON_ABORT | STOP_ON_COUNT);

    result.cyclesDone += r.cyclesDone;
    if (compStop) goto done;

    if (!r.stoppedOnJunction()) {
        Serial.println("[SORT] WARNING: No junction found on approach — walking more");
        r = navLineFollowUntil(
            SORT_APPROACH_MAX_CYCLES,
            STOP_ON_JUNCTION | STOP_ON_ABORT | STOP_ON_COUNT);
        result.cyclesDone += r.cyclesDone;
        if (compStop) goto done;
    }

    Serial.printf("[SORT] First junction reached: %s\n", r.juncType);

    // =====================================================================
    // PHASE 2: Navigate to the correct branch junction
    // =====================================================================
    // The robot is at the first branch junction.  If the target is branch 1,
    // turn left here.  If branch 2 or 3, continue straight past this junction
    // to the next one(s).

    Serial.printf("[SORT] Phase 2: Navigating to branch %d...\n", targetBranch);

    for (int branch = 1; branch < targetBranch && !compStop; branch++) {
        // Skip this branch — continue straight along the main line
        Serial.printf("[SORT] Skipping branch %d — continuing to next junction\n",
                      branch);
        r = navLineFollowUntil(
            SORT_INTER_BRANCH_CYCLES,
            STOP_ON_JUNCTION | STOP_ON_ABORT | STOP_ON_COUNT);
        result.cyclesDone += r.cyclesDone;

        if (!r.stoppedOnJunction()) {
            Serial.printf("[SORT] WARNING: Expected junction for branch %d not found\n",
                          branch + 1);
        }
    }

    if (compStop) goto done;

    // =====================================================================
    // PHASE 3: Turn LEFT onto the matching colored line
    // =====================================================================
    Serial.printf("[SORT] Phase 3: Turning LEFT onto branch %d...\n", targetBranch);

    navTurnLeft90();
    navAlignToLine();

    if (compStop) goto done;

    // =====================================================================
    // PHASE 4: Follow the colored line to the end of the sorting segment
    // =====================================================================
    Serial.println("[SORT] Phase 4: Following colored line to segment end...");

    r = navLineFollowUntil(
        SORT_COLOR_LINE_CYCLES,
        STOP_ON_LINE_LOST | STOP_ON_ABORT | STOP_ON_COUNT);

    result.cyclesDone += r.cyclesDone;

    if (compStop) goto done;

    Serial.printf("[SORT] Reached end of color segment (reason=0x%02X)\n", r.reason);

    // =====================================================================
    // PHASE 5: Place the ball
    // =====================================================================
    Serial.println("[SORT] Phase 5: Placing ball...");

    result.ballPlaced = armPlaceBall();
    if (result.ballPlaced) {
        Serial.println("[SORT] Ball placed successfully!");
    } else {
        Serial.println("[SORT] WARNING: Ball placement may have failed");
    }

    if (compStop) goto done;

    // =====================================================================
    // PHASE 6: Return to the main white line
    // =====================================================================
    Serial.println("[SORT] Phase 6: Returning to main line...");

    // Turn around (180°) to face back toward the main line
    navTurn180();
    navAlignToLine();

    // Follow the colored line back to the junction
    r = navLineFollowUntil(
        SORT_RETURN_MAX_CYCLES,
        STOP_ON_JUNCTION | STOP_ON_ABORT | STOP_ON_COUNT);

    result.cyclesDone += r.cyclesDone;

    if (compStop) goto done;

    // At the junction, turn RIGHT to continue along the main line
    // (we came from the left, so turning right puts us back on the main line
    //  heading toward the FINISH)
    Serial.println("[SORT] Turning RIGHT back onto main line...");
    navTurnRight90();
    navAlignToLine();

    if (compStop) goto done;

    // =====================================================================
    // PHASE 7: Follow the main line to the FINISH point
    // =====================================================================
    Serial.println("[SORT] Phase 7: Following main line to FINISH...");

    r = navLineFollowUntil(
        SORT_FINISH_MAX_CYCLES,
        STOP_ON_LINE_LOST | STOP_ON_ABORT | STOP_ON_COUNT);

    result.cyclesDone += r.cyclesDone;

    // If line ends, we've reached the FINISH area
    result.success = !compStop;

done:
    Serial.println("============================================================");
    Serial.printf("[SORT] SubTask 4: %s (branch=%d, placed=%s, cycles=%d)\n",
                  result.success ? "COMPLETE" : "FAILED",
                  result.branchIndex,
                  result.ballPlaced ? "yes" : "no",
                  result.cyclesDone);
    Serial.println("============================================================");
    return result;
}
