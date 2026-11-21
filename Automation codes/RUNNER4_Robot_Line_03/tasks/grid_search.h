#pragma once
// =============================================================================
// grid_search.h — SubTask #1: Grid Search, Ball Detection, Pick-up & Exit
//
// STRATEGY: "Row-Sweep with Inline ToF Detection"
//   The robot enters the 4×4 grid from the bottom-right (START), heading north.
//   It traverses each row of internal crosses in a serpentine pattern:
//     Row 1: walk WEST (right→left)
//     Row 2: walk EAST (left→right)
//     Row 3: walk WEST (right→left)
//   While walking along each row, the front ToF sensor naturally scans ahead
//   in the direction of travel.  When the ball+pedestal is detected (ToF
//   reading drops below threshold), the robot stops, approaches, reads color,
//   picks up, and navigates to the grid exit.
//
// GRID LAYOUT (4×4 grid, 25cm cells, 3cm white lines):
//   5 vertical lines (cols 0–4), 5 horizontal lines (rows 0–4).
//   9 internal crosses at (col 1–3, row 1–3) where ball can be placed.
//   Entry: bottom-right corner → heading NORTH.
//   Exit:  top-left corner → continues to SubTask 2.
//
// DEPENDENCIES:
//   nav_primitives.h — line following, turns, ToF, color
//   arm_interface.h  — pick up ball
// =============================================================================

#include <Arduino.h>
#include "../nav/nav_primitives.h"
#include "../arm/arm_interface.h"

// ── Grid calibration ─────────────────────────────────────────────────────────
// How many line-follow cycles to traverse one 25cm cell (must be calibrated)
#define GRID_CYCLES_PER_CELL     NAV_CYCLES_PER_CELL
// Max cycles for one full row scan (~3 cells + margin)
#define GRID_ROW_MAX_CYCLES      (GRID_CYCLES_PER_CELL * 5)
// Max cycles to walk from entry to first junction (~1-2 cells)
#define GRID_ENTRY_MAX_CYCLES    (GRID_CYCLES_PER_CELL * 4)
// Cycles to walk one cell between rows
#define GRID_INTER_ROW_CYCLES    (GRID_CYCLES_PER_CELL * 3)
// Max cycles for exit navigation per leg
#define GRID_EXIT_MAX_CYCLES     (GRID_CYCLES_PER_CELL * 6)

// ToF threshold for ball detection while walking along a row (mm)
// The ball is at the next cross, ~250mm ahead.  Set threshold slightly above.
#define GRID_BALL_TOF_MM         250.0f

// Approach distance: how close to walk before picking (mm)
#define GRID_APPROACH_TOF_MM     100.0f
// Max walk steps during final approach
#define GRID_APPROACH_MAX_STEPS  4

// ── Result struct ────────────────────────────────────────────────────────────
struct GridResult {
    bool     success;       // true if ball found and picked
    uint8_t  ballColor;     // COLOR_ID_RED / GREEN / BLUE (0 if failed)
    int      ballRow;       // row where ball was found (1–3), -1 if not found
    int      searchCycles;  // total walk cycles spent searching
};

// =============================================================================
// INTERNAL HELPERS
// =============================================================================
namespace gridns {

// Current heading tracker
static int sHeading = HEADING_NORTH;

// Turn to a target heading, updating the tracked heading.
// navTurnLeft90/Right90/180 now automatically:
//   1. Turn on gyro → take turn → turn off gyro
//   2. Run navCenterOnLine() to reach lineError ≈ 0
// So we NO LONGER call navAlignToLine() here — it is handled inside the turn.
static void turnTo(int target) {
    sHeading = navTurnToHeading(sHeading, target);
    // Brief settle wait after centering before line-follow resumes
    waitMs(150);
    serviceBackground();
}

// Walk forward along the line until the next junction or max cycles.
// Returns the NavResult.
static NavResult walkToNextJunction(int maxCycles = GRID_INTER_ROW_CYCLES) {
    return navLineFollowUntil(maxCycles,
                              STOP_ON_JUNCTION | STOP_ON_ABORT | STOP_ON_COUNT);
}

// Scan one row by walking in the current heading direction.
// Stops if ToF detects ball ahead OR maxCycles reached.
// Returns NavResult (check stoppedOnTof() for ball detection).
static NavResult scanRow(int maxCycles = GRID_ROW_MAX_CYCLES) {
    Serial.printf("[GRID] Scanning row, heading=%d, max=%d cycles\n",
                  sHeading, maxCycles);
    return navLineFollowUntil(
        maxCycles,
        STOP_ON_TOF_CLOSE | STOP_ON_COUNT | STOP_ON_ABORT | STOP_ON_LINE_LOST,
        GRID_BALL_TOF_MM);
}

// After ToF detected ball ahead, approach it carefully:
// Walk forward in small increments, checking ToF, until close enough to pick.
// Returns true if ball is within pickup range.
static bool approachBall() {
    Serial.println("[GRID] Approaching ball...");
    for (int step = 0; step < GRID_APPROACH_MAX_STEPS && !compStop; step++) {
        float d = navScanToF();
        Serial.printf("[GRID] Approach step %d — ToF=%.0fmm\n", step, d);
        if (d > 10.0f && d < GRID_APPROACH_TOF_MM) {
            Serial.println("[GRID] Ball in pickup range!");
            return true;
        }
        // Walk one step closer
        navWalkSteps(1, NAV_WALK_SPEED_MS);
    }
    // Even if we didn't reach "close" threshold, we may be close enough
    Serial.println("[GRID] Approach done (max steps reached)");
    return true;
}

// Read ball color and pick it up.  Returns the color ID.
static uint8_t readColorAndPick() {
    Serial.println("[GRID] Reading ball color...");

    // Wait a moment for color sensor to settle
    waitMs(500);

    // Read color
    uint8_t color = navReadColor(5000);
    Serial.printf("[GRID] Ball color: %d (%s)\n",
                  color, color ? colorName(color) : "UNKNOWN");

    // Pick up the ball
    Serial.println("[GRID] Picking up ball...");
    bool picked = armPickBall();
    if (!picked) {
        Serial.println("[GRID] WARNING: armPickBall reported failure");
    }

    return color;
}

// Navigate from current position to the grid exit (top-left corner = END).
//
// CUPHOLDER CONSTRAINT:
//   The ball sits on a cupholder/pedestal at the cross.
//   After picking, the robot has already done a 180° turn (see PHASE 7/8),
//   so it is now facing BACK along the row it was scanning.
//   sHeading has been updated to reflect this reversed heading.
//
// EXIT STRATEGY (always works regardless of which row ball was in):
//   1. Walk back along the row toward the row's start boundary (junction)
//   2. Face NORTH  → walk to top horizontal boundary (junction or line-lost)
//   3. Face WEST   → walk to left vertical boundary (line-lost = exit col 0)
//   4. Face NORTH  → walk a few steps past the grid to signal Task 1 done
//
static void navigateToExit() {
    Serial.println("[GRID] ── navigateToExit ──");

    // ── Leg A: Walk back to the row boundary ─────────────────────────────
    // After the 180° turn we face back along the row (EAST if we were scanning
    // WEST, or WEST if we were scanning EAST). Walk until we hit the row-end
    // junction or run out of line.
    Serial.println("[GRID] ExitA: walking back to row boundary...");
    NavResult r = navLineFollowUntil(GRID_EXIT_MAX_CYCLES,
                                     STOP_ON_JUNCTION | STOP_ON_LINE_LOST |
                                     STOP_ON_ABORT | STOP_ON_COUNT);
    Serial.printf("[GRID] ExitA done: reason=0x%02X cycles=%d\n",
                  r.reason, r.cyclesDone);
    if (compStop) return;

    // ── Leg B: Turn NORTH → walk to top boundary ─────────────────────────
    Serial.println("[GRID] ExitB: turning NORTH, walking to top boundary...");
    turnTo(HEADING_NORTH);
    r = navLineFollowUntil(GRID_EXIT_MAX_CYCLES,
                           STOP_ON_JUNCTION | STOP_ON_LINE_LOST |
                           STOP_ON_ABORT | STOP_ON_COUNT);
    Serial.printf("[GRID] ExitB done: reason=0x%02X cycles=%d\n",
                  r.reason, r.cyclesDone);
    if (compStop) return;

    // ── Leg C: Turn WEST → walk to left boundary (col 0) ─────────────────
    Serial.println("[GRID] ExitC: turning WEST, walking to left boundary...");
    turnTo(HEADING_WEST);
    r = navLineFollowUntil(GRID_EXIT_MAX_CYCLES,
                           STOP_ON_LINE_LOST | STOP_ON_JUNCTION |
                           STOP_ON_ABORT | STOP_ON_COUNT);
    Serial.printf("[GRID] ExitC done: reason=0x%02X cycles=%d\n",
                  r.reason, r.cyclesDone);
    if (compStop) return;

    // ── Leg D: Turn NORTH → walk to top-left corner (END) ────────────────
    Serial.println("[GRID] ExitD: turning NORTH, walking to END corner...");
    turnTo(HEADING_NORTH);
    r = navLineFollowUntil(GRID_EXIT_MAX_CYCLES,
                           STOP_ON_LINE_LOST | STOP_ON_JUNCTION |
                           STOP_ON_ABORT | STOP_ON_COUNT);
    Serial.printf("[GRID] ExitD done: reason=0x%02X cycles=%d\n",
                  r.reason, r.cyclesDone);
    if (compStop) return;

    // ── Final: Step ~15 cm north past grid boundary ───────────────────────
    Serial.println("[GRID] Reached END corner! Stepping North to clear boundary...");
    navWalkSteps(3, NAV_WALK_SPEED_MS);
    Serial.println("[GRID] Grid cleared! SubTask 1 officially finished.");
}

} // namespace gridns

// =============================================================================
// PUBLIC API — runGridSearch()
// =============================================================================
// Runs the complete SubTask #1: grid search, ball pickup, and exit.
//
// BLOCKING: this function does not return until the task is complete, failed,
//           or aborted via compStop.
//
// Returns GridResult with ball color and success/failure info.
//
inline GridResult runGridSearch() {
    GridResult result = {false, 0, -1, 0};
    int totalCycles = 0;
    gridns::sHeading = HEADING_NORTH;

    Serial.println("============================================================");
    Serial.println("[GRID] SubTask 1: GRID SEARCH — START");
    Serial.println("============================================================");

    // Ensure arm is in safe travel position
    armSafePosition();

    // =====================================================================
    // PHASE 1: Enter the grid
    // =====================================================================
    // Robot starts at START (bottom-right), facing north on the entry line.
    // Line-follow north until first junction (row 1 boundary).
    Serial.println("[GRID] Phase 1: Entering grid...");
    NavResult r = navLineFollowUntil(GRID_ENTRY_MAX_CYCLES,
                                      STOP_ON_JUNCTION | STOP_ON_ABORT | STOP_ON_COUNT);
    totalCycles += r.cyclesDone;
    if (compStop) goto done;

    if (!r.stoppedOnJunction()) {
        Serial.println("[GRID] WARNING: No junction found at grid entry — walking more");
        r = navLineFollowUntil(GRID_ENTRY_MAX_CYCLES,
                                STOP_ON_JUNCTION | STOP_ON_ABORT | STOP_ON_COUNT);
        totalCycles += r.cyclesDone;
        if (compStop) goto done;
    }

    Serial.printf("[GRID] Entry junction reached (%s)\n", r.juncType);

    // =====================================================================
    // PHASE 2: Scan Row 1 (walk WEST)
    // =====================================================================
    Serial.println("[GRID] Phase 2: Scanning Row 1 (WEST)...");
    gridns::turnTo(HEADING_WEST);
    r = gridns::scanRow();
    totalCycles += r.cyclesDone;
    if (compStop) goto done;

    if (r.stoppedOnTof()) {
        Serial.println("[GRID] Ball detected in Row 1!");
        result.ballRow = 1;
        goto found;
    }
    Serial.println("[GRID] Row 1: No ball found");

    // =====================================================================
    // PHASE 3: Navigate to Row 2 (turn north, walk 1 cell, turn east)
    // =====================================================================
    Serial.println("[GRID] Phase 3: Moving to Row 2...");

    // At end of row 1, currently facing WEST at leftmost position
    // Turn RIGHT to face NORTH
    gridns::turnTo(HEADING_NORTH);

    // Walk north one cell to row 2
    r = gridns::walkToNextJunction();
    totalCycles += r.cyclesDone;
    if (compStop) goto done;

    // Turn RIGHT to face EAST for row 2 scan (serpentine: right-to-left was row 1,
    // now left-to-right is row 2)
    gridns::turnTo(HEADING_EAST);

    // =====================================================================
    // PHASE 4: Scan Row 2 (walk EAST)
    // =====================================================================
    Serial.println("[GRID] Phase 4: Scanning Row 2 (EAST)...");
    r = gridns::scanRow();
    totalCycles += r.cyclesDone;
    if (compStop) goto done;

    if (r.stoppedOnTof()) {
        Serial.println("[GRID] Ball detected in Row 2!");
        result.ballRow = 2;
        goto found;
    }
    Serial.println("[GRID] Row 2: No ball found");

    // =====================================================================
    // PHASE 5: Navigate to Row 3 (turn north, walk 1 cell, turn west)
    // =====================================================================
    Serial.println("[GRID] Phase 5: Moving to Row 3...");

    // At end of row 2, currently facing EAST at rightmost position
    // Turn LEFT to face NORTH
    gridns::turnTo(HEADING_NORTH);

    // Walk north one cell to row 3
    r = gridns::walkToNextJunction();
    totalCycles += r.cyclesDone;
    if (compStop) goto done;

    // Turn LEFT to face WEST for row 3 scan
    gridns::turnTo(HEADING_WEST);

    // =====================================================================
    // PHASE 6: Scan Row 3 (walk WEST) — ball MUST be here
    // =====================================================================
    Serial.println("[GRID] Phase 6: Scanning Row 3 (WEST) — ball must be here...");
    r = gridns::scanRow();
    totalCycles += r.cyclesDone;
    if (compStop) goto done;

    if (r.stoppedOnTof()) {
        Serial.println("[GRID] Ball detected in Row 3!");
        result.ballRow = 3;
        goto found;
    }

    // Ball not found in any row — this shouldn't happen
    Serial.println("[GRID] ERROR: Ball not found in any row!");
    result.success = false;
    goto done;

    // =====================================================================
    // PHASE 7: Ball found — approach, read color, pick up
    // =====================================================================
found:
    Serial.printf("[GRID] BALL FOUND in row %d! Approaching...\n", result.ballRow);

    // Approach the ball (walk closer until in pickup range)
    gridns::approachBall();
    if (compStop) goto done;

    // Read color and pick up
    result.ballColor = gridns::readColorAndPick();
    if (result.ballColor != 0) {
        result.success = true;
        Serial.printf("[GRID] SUCCESS: Ball color=%d (%s) picked up!\n",
                      result.ballColor, colorName(result.ballColor));
    } else {
        Serial.println("[GRID] WARNING: Color read failed — picked anyway");
        result.success = true;  // ball is picked even if color unknown
    }

    if (compStop) goto done;

    // =====================================================================
    // PHASE 8: Clear the cupholder — 180° turn then exit
    // =====================================================================
    // The ball sits on a cupholder/pedestal at the cross.
    // We CANNOT continue forward past this point (cupholder blocks the path).
    // Strategy:
    //   1. 180° turn (face back along the row we came from)
    //   2. Update heading tracker so navigateToExit() knows our direction
    //   3. Walk 2 steps forward (clears the cross, gets back on a clean line segment)
    //   4. navigateToExit() — walk back to boundary → north → west → north → done
    Serial.println("[GRID] Phase 8: 180° turn to clear cupholder...");

    navTurn180();

    // Flip the heading tracker: row scan was WEST → now EAST, or EAST → now WEST
    gridns::sHeading = (gridns::sHeading + 2) % 4;
    Serial.printf("[GRID] Heading after 180°: %d\n", gridns::sHeading);

    // Brief wait for settle + gyro off (navTurn180 already called mpuSetStationary(true))
    waitMs(200);
    serviceBackground();

    // Walk a couple steps back to clear the cross so we're on a plain line segment
    Serial.println("[GRID] Walking back 2 steps to clear cross...");
    navWalkSteps(2, NAV_WALK_SPEED_MS);
    if (compStop) goto done;

    Serial.println("[GRID] Phase 8: Exiting grid...");
    gridns::navigateToExit();

done:
    result.searchCycles = totalCycles;
    Serial.println("============================================================");
    Serial.printf("[GRID] SubTask 1: %s (color=%d, row=%d, cycles=%d)\n",
                  result.success ? "COMPLETE" : "FAILED",
                  result.ballColor, result.ballRow, totalCycles);
    Serial.println("============================================================");
    return result;
}
