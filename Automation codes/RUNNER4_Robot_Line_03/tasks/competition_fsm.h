#pragma once
// =============================================================================
// competition_fsm.h — Master Competition State Machine for RUNNER-4
//
// Sequences through all 4 SubTasks in order:
//   SubTask 1: Grid Search — find, identify, and pick up the colored ball
//   SubTask 2: Wall Follow — traverse curved corridor with gaps
//   SubTask 3: Obstacle Push — push box, turn left, exit
//   SubTask 4: Color Sort — place ball on matching colored line, finish
//
// CONTROL:
//   - Start via web: GET /comp/start    or /comp/start?from=N (checkpoint)
//   - Stop via web:  GET /comp/stop
//   - Status:        GET /comp/status
//   - Physical start button on GPIO COMP_START_PIN
//
// CHECKPOINTS:
//   The competition supports restarting from checkpoints (per competition rules,
//   up to 3 restarts within 10 minutes).  Use /comp/start?from=N where:
//     N=0 ? start from SubTask 1 (full run)
//     N=1 ? start from SubTask 1 (same as 0)
//     N=2 ? start from SubTask 2 (skip grid search)
//     N=3 ? start from SubTask 3 (skip grid + corridor)
//     N=4 ? start from SubTask 4 (skip to color sort)
//
// DEPENDENCIES:
//   nav_primitives.h, arm_interface.h
//   grid_search.h, wall_follow.h, obstacle_push.h, color_sort.h
// =============================================================================

#include <Arduino.h>
#include <WebServer.h>

// -- Competition abort flag — shared by all modules ---------------------------
volatile bool compStop = false;

// -- Physical start button (optional) -----------------------------------------
#ifndef COMP_START_PIN
#define COMP_START_PIN  0   // GPIO 0 = BOOT button on most ESP32 boards
#endif
// COMP_DEBOUNCE_MS defined in config/robot_config.h

// Include all SubTask modules
#include "grid_search.h"
#include "wall_follow.h"
#include "obstacle_push.h"
#include "color_sort.h"

// Competition dashboard web page
#include "../dashboard/comp_page.h"

// -- Competition state --------------------------------------------------------
enum CompState : uint8_t {
    COMP_IDLE       = 0,    // Waiting for start
    COMP_STARTING   = 1,    // Startup sequence (stand up)
    COMP_TASK1      = 2,    // Grid search
    COMP_TASK2      = 3,    // Wall follow
    COMP_TASK3      = 4,    // Obstacle push
    COMP_TASK4      = 5,    // Color sort
    COMP_FINISHED   = 6,    // All tasks complete!
    COMP_ABORTED    = 7,    // Stopped early
    COMP_ERROR      = 8     // Unrecoverable error
};

// -- External globals from main .ino ------------------------------------------
enum Job : uint8_t;
extern volatile Job pendingJob;
extern int jobSteps, jobSpeed;
extern volatile bool walkStop, turnStop, lfStop;

namespace compns {

// -- State tracking -----------------------------------------------------------
static volatile CompState  sState       = COMP_IDLE;
static uint8_t             sBallColor   = 0;     // from Task 1
static int                 sStartFrom   = 1;     // checkpoint (1–4)
static uint32_t            sStartTime   = 0;     // millis() at start
static uint32_t            sEndTime     = 0;     // millis() at end
static bool                sTask1Ok     = false;
static bool                sTask2Ok     = false;
static bool                sTask3Ok     = false;
static bool                sTask4Ok     = false;
static WebServer*          sSrv         = nullptr;

// -- State name for logging / web ---------------------------------------------
static const char* stateName(CompState s) {
    switch (s) {
        case COMP_IDLE:       return "IDLE";
        case COMP_STARTING:   return "STARTING";
        case COMP_TASK1:      return "TASK1_GRID";
        case COMP_TASK2:      return "TASK2_CORRIDOR";
        case COMP_TASK3:      return "TASK3_OBSTACLE";
        case COMP_TASK4:      return "TASK4_COLOR";
        case COMP_FINISHED:   return "FINISHED";
        case COMP_ABORTED:    return "ABORTED";
        case COMP_ERROR:      return "ERROR";
        default:              return "UNKNOWN";
    }
}

// =============================================================================
// THE MAIN COMPETITION RUN — called as a job from loop()
// =============================================================================
static void runCompetition() {
    sStartTime = millis();
    compStop = false;

    Serial.println("\n");
    Serial.println("############################################################");
    Serial.println("#                                                          #");
    Serial.println("#   RUNNER-4  COMPETITION RUN  —  BREACH PROTOCOL          #");
    Serial.println("#                                                          #");
    Serial.printf( "#   Starting from: SubTask %d                               #\n",
                   sStartFrom);
    Serial.println("#                                                          #");
    Serial.println("############################################################\n");

    // -- STARTUP ----------------------------------------------------------
    sState = COMP_STARTING;
    Serial.println("[COMP] Standing up...");
    navStandUp();
    waitMs(1000);   // settle

    if (compStop) { sState = COMP_ABORTED; goto finish; }

    // -- SUBTASK 1: Grid Search -------------------------------------------
    if (sStartFrom <= 1) {
        sState = COMP_TASK1;
        Serial.println("[COMP] --- SubTask 1: Grid Search ---");

        GridResult gr = runGridSearch();
        sTask1Ok  = gr.success;
        sBallColor = gr.ballColor;

        if (compStop) { sState = COMP_ABORTED; goto finish; }

        if (!gr.success) {
            Serial.println("[COMP] WARNING: Grid search failed — continuing anyway");
            // If ball not picked, Task 4 will have nothing to place.
            // Continue to earn points from Tasks 2 and 3.
        }

        Serial.printf("[COMP] Ball color stored: %d (%s)\n",
                      sBallColor, sBallColor ? colorName(sBallColor) : "NONE");
    }

    // -- SUBTASK 2: Wall Follow -------------------------------------------
    if (sStartFrom <= 2) {
        sState = COMP_TASK2;
        Serial.println("[COMP] --- SubTask 2: Curved Corridor ---");

        WallFollowResult wf = runWallFollow();
        sTask2Ok = wf.success;

        if (compStop) { sState = COMP_ABORTED; goto finish; }

        if (!wf.success) {
            Serial.println("[COMP] WARNING: Wall follow incomplete — continuing");
        }
    }

    // -- SUBTASK 3: Obstacle Push -----------------------------------------
    if (sStartFrom <= 3) {
        sState = COMP_TASK3;
        Serial.println("[COMP] --- SubTask 3: Obstacle Push ---");

        ObstaclePushResult op = runObstaclePush();
        sTask3Ok = op.success;

        if (compStop) { sState = COMP_ABORTED; goto finish; }

        if (!op.success) {
            Serial.println("[COMP] WARNING: Obstacle push incomplete — continuing");
        }
    }

    // -- SUBTASK 4: Color Sort --------------------------------------------
    if (sStartFrom <= 4) {
        sState = COMP_TASK4;
        Serial.println("[COMP] --- SubTask 4: Color Sort ---");

        // If we skipped Task 1 (checkpoint restart), try to use stored ball color
        if (sBallColor == 0) {
            // Try to read from flash (colorStoredBall from TCS34725 module)
            sBallColor = colorStoredBall();
            Serial.printf("[COMP] Ball color from flash: %d\n", sBallColor);
        }

        ColorSortResult cs = runColorSort(sBallColor);
        sTask4Ok = cs.success;

        if (compStop) { sState = COMP_ABORTED; goto finish; }
    }

    // -- FINISHED ---------------------------------------------------------
    sState = COMP_FINISHED;

finish:
    sEndTime = millis();
    uint32_t elapsed = sEndTime - sStartTime;

    Serial.println("\n");
    Serial.println("############################################################");
    Serial.println("#                                                          #");
    Serial.printf( "#   RUN %s                                      #\n",
                   sState == COMP_FINISHED ? "COMPLETE  " :
                   sState == COMP_ABORTED ? "ABORTED   " : "ERROR     ");
    Serial.println("#                                                          #");
    Serial.printf( "#   Time: %02d:%02d.%d                                        #\n",
                   (int)(elapsed / 60000),
                   (int)((elapsed / 1000) % 60),
                   (int)((elapsed / 100) % 10));
    Serial.printf( "#   Task1=%s  Task2=%s  Task3=%s  Task4=%s    #\n",
                   sTask1Ok ? "OK  " : "FAIL",
                   sTask2Ok ? "OK  " : "FAIL",
                   sTask3Ok ? "OK  " : "FAIL",
                   sTask4Ok ? "OK  " : "FAIL");
    Serial.printf( "#   Ball color: %d (%s)                              #\n",
                   sBallColor, sBallColor ? colorName(sBallColor) : "NONE");
    Serial.println("#                                                          #");
    Serial.println("############################################################\n");

    // Return to standing pose
    navStandUp();
}

// =============================================================================
// WEB HANDLERS
// =============================================================================

// GET /comp/start?from=N
static void handleStart() {
    addCORS();
    if (sState != COMP_IDLE && sState != COMP_FINISHED &&
        sState != COMP_ABORTED && sState != COMP_ERROR) {
        sSrv->send(200, "application/json",
                   "{\"ok\":false,\"msg\":\"Competition already running\"}");
        return;
    }

    sStartFrom = sSrv->hasArg("from")
                     ? constrain(sSrv->arg("from").toInt(), 1, 4)
                     : 1;

    // If restarting from a checkpoint, allow setting ball color manually
    if (sSrv->hasArg("color")) {
        sBallColor = constrain(sSrv->arg("color").toInt(), 0, 3);
    }

    // Reset task results
    sTask1Ok = sTask2Ok = sTask3Ok = sTask4Ok = false;
    compStop = false;
    sState = COMP_IDLE;  // will be set to STARTING by runCompetition()

    // Queue the competition job
    jobSteps = sStartFrom;  // pass checkpoint via jobSteps
    pendingJob = (Job)9;     // JOB_COMPETITION = 9

    char msg[128];
    snprintf(msg, sizeof(msg),
             "{\"ok\":true,\"msg\":\"Competition starting from task %d\"}",
             sStartFrom);
    sSrv->send(200, "application/json", msg);
    Serial.printf("[COMP] Queued: start from task %d\n", sStartFrom);
}

// GET /comp/stop
static void handleStop() {
    addCORS();
    compStop = true;
    // Also stop any running walks/turns
    walkStop = true;
    turnStop = true;
    lfStop   = true;
    sSrv->send(200, "application/json",
               "{\"ok\":true,\"msg\":\"Competition abort requested\"}");
    Serial.println("[COMP] ABORT requested!");
}

// GET /comp/status
static void handleStatus() {
    uint32_t elapsed = (sState >= COMP_STARTING && sState <= COMP_TASK4)
                           ? (millis() - sStartTime)
                           : (sEndTime > sStartTime ? sEndTime - sStartTime : 0);

    // Gather live sensor data for the dashboard
    float tof = tofOnline() ? tofFront() : -1;
    float lineErr = lineFound() ? lineErrorMm() : -999;
    bool junc = lineJunction();
    const char* jtype = lineJunctionType();
    bool lost = lineLost();

    char buf[600];
    snprintf(buf, sizeof(buf),
        "{\"state\":\"%s\",\"stateId\":%d,"
        "\"from\":%d,\"elapsed\":%lu,"
        "\"task1\":%s,\"task2\":%s,\"task3\":%s,\"task4\":%s,"
        "\"ballColor\":%d,\"colorName\":\"%s\","
        "\"compStop\":%s,"
        "\"tof\":%.1f,\"lineErr\":%.1f,"
        "\"junction\":%s,\"jtype\":\"%s\",\"lost\":%s}",
        stateName(sState), (int)sState,
        sStartFrom, (unsigned long)elapsed,
        sTask1Ok ? "true" : "false",
        sTask2Ok ? "true" : "false",
        sTask3Ok ? "true" : "false",
        sTask4Ok ? "true" : "false",
        sBallColor, sBallColor ? colorName(sBallColor) : "none",
        compStop ? "true" : "false",
        tof, lineErr,
        junc ? "true" : "false", jtype,
        lost ? "true" : "false");

    sSrv->sendHeader("Cache-Control", "no-store");
    sSrv->sendHeader("Access-Control-Allow-Origin", "*");
    sSrv->send(200, "application/json", buf);
}

// GET /comp  — serve the competition dashboard page
static void handlePage() {
    sSrv->send_P(200, "text/html", COMP_PAGE);
}

// GET /comp/arm?cmd=pick|place|up|down|open|close
static void handleArm() {
    addCORS();
    if (!sSrv->hasArg("cmd")) {
        sSrv->send(400, "text/plain", "Missing ?cmd= parameter");
        return;
    }
    String cmd = sSrv->arg("cmd");
    bool ok = armSendCmd(cmd.c_str(), 8000);
    char resp[128];
    snprintf(resp, sizeof(resp), "{\"ok\":%s,\"cmd\":\"%s\"}",
             ok ? "true" : "false", cmd.c_str());
    sSrv->send(200, "application/json", resp);
}

} // namespace compns

// =============================================================================
// PUBLIC API
// =============================================================================

// The actual competition run function — called from the job system in loop()
inline void runCompetition(int startFromTask) {
    compns::sStartFrom = constrain(startFromTask, 1, 4);
    compns::runCompetition();
}

// Register web routes and init.  Call once from setup().
inline void competitionBegin(WebServer &server) {
    compns::sSrv = &server;

    // Physical start button
    pinMode(COMP_START_PIN, INPUT_PULLUP);

    // Web routes
    server.on("/comp",        compns::handlePage);    // dashboard HTML
    server.on("/comp/start",  compns::handleStart);
    server.on("/comp/stop",   compns::handleStop);
    server.on("/comp/status", compns::handleStatus);
    server.on("/comp/arm",    compns::handleArm);     // arm proxy commands

    Serial.println("[COMP] Competition FSM initialised");
    Serial.println("  /comp              — competition dashboard");
    Serial.println("  /comp/start?from=N — start (N=1..4 checkpoint)");
    Serial.println("  /comp/stop         — abort");
    Serial.println("  /comp/status       — JSON status");
    Serial.println("  /comp/arm?cmd=X    — arm command proxy");
}

// Check physical start button — call from loop() when idle.
// Returns true if the button was pressed and competition should start.
inline bool competitionCheckButton() {
    if (compns::sState != COMP_IDLE &&
        compns::sState != COMP_FINISHED &&
        compns::sState != COMP_ABORTED) {
        return false;   // competition already running
    }

    static bool lastBtn = true;
    static uint32_t lastPress = 0;
    bool btn = digitalRead(COMP_START_PIN);

    if (lastBtn == true && btn == false) {
        // Button pressed (active LOW)
        if (millis() - lastPress > COMP_DEBOUNCE_MS) {
            lastPress = millis();
            lastBtn = btn;
            return true;
        }
    }
    lastBtn = btn;
    return false;
}
