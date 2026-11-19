#pragma once
// =============================================================================
// crawl_gait.h — Wave / Crawl Gait (One Leg Lift at a Time)
// FusionForce RUNNER4 Quadruped Robot
//
// GAIT PRINCIPLE:
//   In this gait, only ONE leg lifts and swings forward at any given moment,
//   while the other THREE legs remain firmly planted on the ground.
//   This provides maximum static stability because the robot's center of mass
//   (CoM) always stays securely within the support triangle of the 3 planted feet.
//
// SEQUENCE:
//   1. Front-Left  (FL, Leg 0) : Lift -> Swing Forward -> Plant -> Stance Push
//   2. Back-Right  (BR, Leg 3) : Lift -> Swing Forward -> Plant -> Stance Push
//   3. Front-Right (FR, Leg 1) : Lift -> Swing Forward -> Plant -> Stance Push
//   4. Back-Left   (BL, Leg 2) : Lift -> Swing Forward -> Plant -> Stance Push
//   Diagonal separation (FL then BR, FR then BL) balances ground reaction forces.
//
// ANGLES REFERENCE (Pose 1 - STANDING):
//   FL (Leg 0): Hip=45,  Femur=25,  Tibia=115
//   FR (Leg 1): Hip=94,  Femur=135, Tibia=45
//   BL (Leg 2): Hip=94,  Femur=111, Tibia=55
//   BR (Leg 3): Hip=45,  Femur=37,  Tibia=115
//
// MOTOR DIRECTIONS:
//   Lifting: FL & BR femurs increase (+20); FR & BL femurs decrease (-20).
//   Forward Swing: FL & BL hips decrease (-20); FR & BR hips increase (+20).
// =============================================================================

#include <Arduino.h>
#include <WebServer.h>
#include <limits.h>

// Forward declarations of core robot functions from RUNNER4_Robot_Line.ino
void moveToTarget(const int targetAngles[4][3], int speedMs);
void moveToPose(int id, int speedMs);
bool robotBusy();
void addCORS();

// Crawl gait state variables (defined in RUNNER4_Robot_Line.ino)
extern volatile bool crawlRunning;
extern volatile bool crawlStop;
extern int crawlStepsTarget;
extern int crawlStepsDone;
extern char crawlPhaseStr[32];

// Core job scheduling variables (from RUNNER4_Robot_Line.ino)
enum Job : uint8_t;
extern volatile Job pendingJob;
extern int jobSpeed;
extern int jobSteps;

// ── TUNING CONSTANTS ─────────────────────────────────────────────────────────
// Adjust these to calibrate step length and ground clearance
static const int CRAWL_SWING = 20;  // Deg of hip swing per step (stride length)
static const int CRAWL_LIFT  = 20;  // Deg of femur lift per step (ground clearance)

// Standing neutral angles
static const int CRAWL_FL_H = 45,  CRAWL_FL_F = 25,  CRAWL_FL_T = 115;
static const int CRAWL_FR_H = 94,  CRAWL_FR_F = 135, CRAWL_FR_T = 45;
static const int CRAWL_BL_H = 94,  CRAWL_BL_F = 111, CRAWL_BL_T = 55;
static const int CRAWL_BR_H = 45,  CRAWL_BR_F = 37,  CRAWL_BR_T = 115;

// Swung forward hip targets (stride forward in air)
static const int CRAWL_FL_H_SW = CRAWL_FL_H - CRAWL_SWING; // 25
static const int CRAWL_FR_H_SW = CRAWL_FR_H + CRAWL_SWING; // 114
static const int CRAWL_BL_H_SW = CRAWL_BL_H - CRAWL_SWING; // 74
static const int CRAWL_BR_H_SW = CRAWL_BR_H + CRAWL_SWING; // 65

// Lifted femur targets (raise foot clear of ground)
static const int CRAWL_FL_F_UP = CRAWL_FL_F + CRAWL_LIFT;  // 45
static const int CRAWL_BR_F_UP = CRAWL_BR_F + CRAWL_LIFT;  // 57
static const int CRAWL_FR_F_UP = CRAWL_FR_F - CRAWL_LIFT;  // 115
static const int CRAWL_BL_F_UP = CRAWL_BL_F - CRAWL_LIFT;  // 91

// =============================================================================
// crawlForward() — Executes N cycles of one-leg-at-a-time walking
// =============================================================================
inline void crawlForward(int steps, int speedMs) {
  crawlRunning   = true;
  crawlStop      = false;
  crawlStepsDone = 0;
  int totalCycles = (steps == 0) ? INT_MAX : steps;

  Serial.printf("[CRAWL] Starting 1-leg wave gait: %d cycles @ %dms/step\n",
                steps, speedMs);

  for (int cycle = 0; cycle < totalCycles && !crawlStop; cycle++) {

    // ─────────────────────────────────────────────────────────────────────────
    // STEP 1: FRONT-LEFT (FL, Leg 0)
    // ─────────────────────────────────────────────────────────────────────────
    // 1A. Lift FL foot off ground (FR, BL, BR firmly planted)
    strncpy(crawlPhaseStr, "1A: FL Lift", sizeof(crawlPhaseStr));
    {
      const int t[4][3] = {
        {CRAWL_FL_H,    CRAWL_FL_F_UP, CRAWL_FL_T}, // FL lifted
        {CRAWL_FR_H,    CRAWL_FR_F,    CRAWL_FR_T}, // FR planted
        {CRAWL_BL_H,    CRAWL_BL_F,    CRAWL_BL_T}, // BL planted
        {CRAWL_BR_H,    CRAWL_BR_F,    CRAWL_BR_T}  // BR planted
      };
      moveToTarget(t, speedMs);
    }
    if (crawlStop) break;

    // 1B. Swing FL hip forward in the air
    strncpy(crawlPhaseStr, "1B: FL Swing", sizeof(crawlPhaseStr));
    {
      const int t[4][3] = {
        {CRAWL_FL_H_SW, CRAWL_FL_F_UP, CRAWL_FL_T}, // FL swung forward
        {CRAWL_FR_H,    CRAWL_FR_F,    CRAWL_FR_T},
        {CRAWL_BL_H,    CRAWL_BL_F,    CRAWL_BL_T},
        {CRAWL_BR_H,    CRAWL_BR_F,    CRAWL_BR_T}
      };
      moveToTarget(t, speedMs);
    }
    if (crawlStop) break;

    // 1C. Plant FL foot onto ground at forward stance position
    strncpy(crawlPhaseStr, "1C: FL Plant", sizeof(crawlPhaseStr));
    {
      const int t[4][3] = {
        {CRAWL_FL_H_SW, CRAWL_FL_F,    CRAWL_FL_T}, // FL planted forward
        {CRAWL_FR_H,    CRAWL_FR_F,    CRAWL_FR_T},
        {CRAWL_BL_H,    CRAWL_BL_F,    CRAWL_BL_T},
        {CRAWL_BR_H,    CRAWL_BR_F,    CRAWL_BR_T}
      };
      moveToTarget(t, speedMs);
    }
    if (crawlStop) break;

    // 1D. Stance Push: all 4 feet on ground push body forward (FL hip -> neutral)
    strncpy(crawlPhaseStr, "1D: Stance Push", sizeof(crawlPhaseStr));
    {
      const int t[4][3] = {
        {CRAWL_FL_H,    CRAWL_FL_F,    CRAWL_FL_T}, // FL returns to neutral
        {CRAWL_FR_H,    CRAWL_FR_F,    CRAWL_FR_T},
        {CRAWL_BL_H,    CRAWL_BL_F,    CRAWL_BL_T},
        {CRAWL_BR_H,    CRAWL_BR_F,    CRAWL_BR_T}
      };
      moveToTarget(t, speedMs);
    }
    if (crawlStop) break;

    // ─────────────────────────────────────────────────────────────────────────
    // STEP 2: BACK-RIGHT (BR, Leg 3)
    // ─────────────────────────────────────────────────────────────────────────
    // 2A. Lift BR foot off ground
    strncpy(crawlPhaseStr, "2A: BR Lift", sizeof(crawlPhaseStr));
    {
      const int t[4][3] = {
        {CRAWL_FL_H,    CRAWL_FL_F,    CRAWL_FL_T},
        {CRAWL_FR_H,    CRAWL_FR_F,    CRAWL_FR_T},
        {CRAWL_BL_H,    CRAWL_BL_F,    CRAWL_BL_T},
        {CRAWL_BR_H,    CRAWL_BR_F_UP, CRAWL_BR_T}  // BR lifted
      };
      moveToTarget(t, speedMs);
    }
    if (crawlStop) break;

    // 2B. Swing BR hip forward in the air
    strncpy(crawlPhaseStr, "2B: BR Swing", sizeof(crawlPhaseStr));
    {
      const int t[4][3] = {
        {CRAWL_FL_H,    CRAWL_FL_F,    CRAWL_FL_T},
        {CRAWL_FR_H,    CRAWL_FR_F,    CRAWL_FR_T},
        {CRAWL_BL_H,    CRAWL_BL_F,    CRAWL_BL_T},
        {CRAWL_BR_H_SW, CRAWL_BR_F_UP, CRAWL_BR_T}  // BR swung forward
      };
      moveToTarget(t, speedMs);
    }
    if (crawlStop) break;

    // 2C. Plant BR foot onto ground
    strncpy(crawlPhaseStr, "2C: BR Plant", sizeof(crawlPhaseStr));
    {
      const int t[4][3] = {
        {CRAWL_FL_H,    CRAWL_FL_F,    CRAWL_FL_T},
        {CRAWL_FR_H,    CRAWL_FR_F,    CRAWL_FR_T},
        {CRAWL_BL_H,    CRAWL_BL_F,    CRAWL_BL_T},
        {CRAWL_BR_H_SW, CRAWL_BR_F,    CRAWL_BR_T}  // BR planted forward
      };
      moveToTarget(t, speedMs);
    }
    if (crawlStop) break;

    // 2D. Stance Push: body advances
    strncpy(crawlPhaseStr, "2D: Stance Push", sizeof(crawlPhaseStr));
    {
      const int t[4][3] = {
        {CRAWL_FL_H,    CRAWL_FL_F,    CRAWL_FL_T},
        {CRAWL_FR_H,    CRAWL_FR_F,    CRAWL_FR_T},
        {CRAWL_BL_H,    CRAWL_BL_F,    CRAWL_BL_T},
        {CRAWL_BR_H,    CRAWL_BR_F,    CRAWL_BR_T}  // BR returns to neutral
      };
      moveToTarget(t, speedMs);
    }
    if (crawlStop) break;

    // ─────────────────────────────────────────────────────────────────────────
    // STEP 3: FRONT-RIGHT (FR, Leg 1)
    // ─────────────────────────────────────────────────────────────────────────
    // 3A. Lift FR foot off ground (femur decreases to lift)
    strncpy(crawlPhaseStr, "3A: FR Lift", sizeof(crawlPhaseStr));
    {
      const int t[4][3] = {
        {CRAWL_FL_H,    CRAWL_FL_F,    CRAWL_FL_T},
        {CRAWL_FR_H,    CRAWL_FR_F_UP, CRAWL_FR_T}, // FR lifted
        {CRAWL_BL_H,    CRAWL_BL_F,    CRAWL_BL_T},
        {CRAWL_BR_H,    CRAWL_BR_F,    CRAWL_BR_T}
      };
      moveToTarget(t, speedMs);
    }
    if (crawlStop) break;

    // 3B. Swing FR hip forward in the air
    strncpy(crawlPhaseStr, "3B: FR Swing", sizeof(crawlPhaseStr));
    {
      const int t[4][3] = {
        {CRAWL_FL_H,    CRAWL_FL_F,    CRAWL_FL_T},
        {CRAWL_FR_H_SW, CRAWL_FR_F_UP, CRAWL_FR_T}, // FR swung forward
        {CRAWL_BL_H,    CRAWL_BL_F,    CRAWL_BL_T},
        {CRAWL_BR_H,    CRAWL_BR_F,    CRAWL_BR_T}
      };
      moveToTarget(t, speedMs);
    }
    if (crawlStop) break;

    // 3C. Plant FR foot onto ground
    strncpy(crawlPhaseStr, "3C: FR Plant", sizeof(crawlPhaseStr));
    {
      const int t[4][3] = {
        {CRAWL_FL_H,    CRAWL_FL_F,    CRAWL_FL_T},
        {CRAWL_FR_H_SW, CRAWL_FR_F,    CRAWL_FR_T}, // FR planted forward
        {CRAWL_BL_H,    CRAWL_BL_F,    CRAWL_BL_T},
        {CRAWL_BR_H,    CRAWL_BR_F,    CRAWL_BR_T}
      };
      moveToTarget(t, speedMs);
    }
    if (crawlStop) break;

    // 3D. Stance Push: body advances
    strncpy(crawlPhaseStr, "3D: Stance Push", sizeof(crawlPhaseStr));
    {
      const int t[4][3] = {
        {CRAWL_FL_H,    CRAWL_FL_F,    CRAWL_FL_T},
        {CRAWL_FR_H,    CRAWL_FR_F,    CRAWL_FR_T}, // FR returns to neutral
        {CRAWL_BL_H,    CRAWL_BL_F,    CRAWL_BL_T},
        {CRAWL_BR_H,    CRAWL_BR_F,    CRAWL_BR_T}
      };
      moveToTarget(t, speedMs);
    }
    if (crawlStop) break;

    // ─────────────────────────────────────────────────────────────────────────
    // STEP 4: BACK-LEFT (BL, Leg 2)
    // ─────────────────────────────────────────────────────────────────────────
    // 4A. Lift BL foot off ground (femur decreases to lift)
    strncpy(crawlPhaseStr, "4A: BL Lift", sizeof(crawlPhaseStr));
    {
      const int t[4][3] = {
        {CRAWL_FL_H,    CRAWL_FL_F,    CRAWL_FL_T},
        {CRAWL_FR_H,    CRAWL_FR_F,    CRAWL_FR_T},
        {CRAWL_BL_H,    CRAWL_BL_F_UP, CRAWL_BL_T}, // BL lifted
        {CRAWL_BR_H,    CRAWL_BR_F,    CRAWL_BR_T}
      };
      moveToTarget(t, speedMs);
    }
    if (crawlStop) break;

    // 4B. Swing BL hip forward in the air
    strncpy(crawlPhaseStr, "4B: BL Swing", sizeof(crawlPhaseStr));
    {
      const int t[4][3] = {
        {CRAWL_FL_H,    CRAWL_FL_F,    CRAWL_FL_T},
        {CRAWL_FR_H,    CRAWL_FR_F,    CRAWL_FR_T},
        {CRAWL_BL_H_SW, CRAWL_BL_F_UP, CRAWL_BL_T}, // BL swung forward
        {CRAWL_BR_H,    CRAWL_BR_F,    CRAWL_BR_T}
      };
      moveToTarget(t, speedMs);
    }
    if (crawlStop) break;

    // 4C. Plant BL foot onto ground
    strncpy(crawlPhaseStr, "4C: BL Plant", sizeof(crawlPhaseStr));
    {
      const int t[4][3] = {
        {CRAWL_FL_H,    CRAWL_FL_F,    CRAWL_FL_T},
        {CRAWL_FR_H,    CRAWL_FR_F,    CRAWL_FR_T},
        {CRAWL_BL_H_SW, CRAWL_BL_F,    CRAWL_BL_T}, // BL planted forward
        {CRAWL_BR_H,    CRAWL_BR_F,    CRAWL_BR_T}
      };
      moveToTarget(t, speedMs);
    }
    if (crawlStop) break;

    // 4D. Stance Push: body advances
    strncpy(crawlPhaseStr, "4D: Stance Push", sizeof(crawlPhaseStr));
    {
      const int t[4][3] = {
        {CRAWL_FL_H,    CRAWL_FL_F,    CRAWL_FL_T},
        {CRAWL_FR_H,    CRAWL_FR_F,    CRAWL_FR_T},
        {CRAWL_BL_H,    CRAWL_BL_F,    CRAWL_BL_T}, // BL returns to neutral
        {CRAWL_BR_H,    CRAWL_BR_F,    CRAWL_BR_T}
      };
      moveToTarget(t, speedMs);
    }
    if (crawlStop) break;

    crawlStepsDone++;
    Serial.printf("[CRAWL] Cycle %d/%d completed.\n", crawlStepsDone,
                  (steps == 0) ? -1 : steps);
  }

  strncpy(crawlPhaseStr, "Standing", sizeof(crawlPhaseStr));
  moveToPose(1, 400); // Return cleanly to STANDING pose
  crawlRunning = false;
  Serial.printf("[CRAWL] Gait finished. Total cycles = %d\n", crawlStepsDone);
}

// =============================================================================
// HTTP HANDLERS
// =============================================================================

// GET /crawl?steps=N&speed=X   |   GET /crawl?stop=1
inline void hCrawl(WebServer &server) {
  addCORS();
  if (server.hasArg("stop")) {
    crawlStop = true;
    server.send(200, "application/json", "{\"ok\":true,\"msg\":\"stopping crawl\"}");
    return;
  }
  if (robotBusy()) {
    server.send(200, "application/json", "{\"ok\":false,\"msg\":\"busy\"}");
    return;
  }
  int steps = server.hasArg("steps") ? server.arg("steps").toInt() : 4;
  jobSpeed = server.hasArg("speed")
                 ? constrain(server.arg("speed").toInt(), 20, 3000)
                 : 300;
  jobSteps = crawlStepsTarget = (steps < 0) ? 0 : steps;
  crawlStepsDone = 0;
  crawlStop = false;
  crawlRunning = true;
  strncpy(crawlPhaseStr, "Starting", sizeof(crawlPhaseStr));
  Serial.printf("[CRAWL] Queued: steps=%d speed=%dms\n", jobSteps, jobSpeed);
  server.send(200, "application/json",
              "{\"ok\":true,\"msg\":\"crawl started\"}");
  pendingJob = (Job)7; // JOB_CRAWL
}

// GET /crawlstatus
inline void hCrawlStatus(WebServer &server) {
  addCORS();
  server.send(200, "application/json",
              String("{\"running\":") + (crawlRunning ? "true" : "false") +
                  ",\"steps\":" + crawlStepsDone + ",\"target\":" +
                  crawlStepsTarget + ",\"phase\":\"" + crawlPhaseStr + "\"}");
}

// Helper to register HTTP endpoints onto the web server
inline void crawlRegisterRoutes(WebServer &server) {
  server.on("/crawl", [&server]() { hCrawl(server); });
  server.on("/crawlstatus", [&server]() { hCrawlStatus(server); });
}
