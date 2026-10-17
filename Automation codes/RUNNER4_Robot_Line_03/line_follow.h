#pragma once
// =============================================================================
// line_follow.h — Proportional Line-Following Gait for RUNNER4
//
// Closed-loop control: reads lineErrorMm() between gait phases and applies
// differential hip deltas so the robot steers back towards the line centre.
//
// HTTP routes (all registered on the shared WebServer):
//   /linefollow          → line-follow tuning & test page
//   /lf/start?steps=N&speed=X  → begin line-following walk
//   /lf/stop             → abort
//   /lf/status           → JSON snapshot (err, correction, hipL, hipR, …)
//   /lf/config           → JSON of all tuneable parameters
//   /lf/set?kp=X&kd=Y…  → live-tweak any parameter
//   /lf/cmd?n=save|reset → persist / reset config
//
// ARCHITECTURE:
//   Uses the same trot gait engine (6-phase diagonal pair walk) as walkForward()
//   but with asymmetric hip deltas computed from the line sensor error.
//   All motion helpers (moveToTarget, setServo, etc.) are shared with the main
//   sketch via forward declarations.
// =============================================================================

#include <Arduino.h>
#include <WebServer.h>
#include <Preferences.h>
#include <limits.h>
#include <math.h>

// ---- Forward declarations of core robot functions from .ino ----
void moveToTarget(const int targetAngles[4][3], int speedMs);
void moveToPose(int id, int speedMs);
void syncGaitBase();
bool robotBusy();
void addCORS();

// ---- Forward declarations of gait base arrays from .ino ----
extern int WALK_HIP_BASE[4], WALK_FEM_PLANT[4], GAIT_TIBIA[4];
extern const int WALK_HIP_SIGN[4];
extern const int FEMUR_LIFT_SIGN[4];
extern const int WALK_FEMUR_TRIM[4];

// ---- Forward declarations of line sensor API ----
extern bool    lineCalibrated();
extern bool    lineFound();
extern float   lineErrorMm();
extern float   lineWz();
extern uint8_t lineActiveCount();
extern bool    lineJunction();
extern const char *lineJunctionType();
extern bool    lineLost();
extern bool    lineSafeStop();
extern void    lineSetAlpha(float a);

// ---- Line-follow state variables (declared in .ino) ----
extern volatile bool  lfRunning;
extern volatile bool  lfStop;
extern int            lfStepsTarget;
extern int            lfStepsDone;
extern char           lfPhaseStr[48];

// ---- Job system linkage ----
enum Job : uint8_t;
extern volatile Job pendingJob;
extern int jobSpeed, jobSteps;

// ── Leg bit masks (same as main sketch) ──────────────────────────────────────
#define LF_M_FL 1
#define LF_M_FR 2
#define LF_M_BL 4
#define LF_M_BR 8

namespace lfns {   // anonymous-ish namespace to avoid symbol clashes

// =============================================================================
// CONFIGURATION (persisted to NVS)
// =============================================================================
struct LFConfig {
  uint32_t magic;
  int   baseHipDelta;     // base hip swing (straight line), degrees
  int   femurDelta;       // femur lift height, degrees
  int   speedMs;          // ms per phase
  float kp;               // proportional gain (deg correction per mm error)
  float kd;               // derivative gain
  int   maxCorrection;    // clamp on |correction| in degrees
  int   steerSign;        // +1 or -1 to flip steering direction
  int   sampleMode;       // 0 = per-cycle, 1 = per-phase
  int   stopOnLost;       // 1 = stop when line lost
  float slowFactor;       // speed multiplier (>1 = slower)
  float emaAlpha;         // EMA override for line sensor during follow
  int   slowOnError;      // 1 = slow down when error is large
  float slowErrorThresh;  // mm threshold for speed reduction
  float slowSpeedMult;    // multiplier when error > threshold
};

#define LF_CFG_MAGIC 0x4C464350UL  // "LFCP" (v2: steerSign inverted to -1)
static LFConfig cfg;
static Preferences lfPrefs;
static bool cfgFromFlash = false;
static WebServer *srv = nullptr;
static const char *apName = "";

// ── Runtime state ────────────────────────────────────────────────────────────
static float prevErr = 0;
static float curCorrection = 0;
static int   curHipDeltaL = 0, curHipDeltaR = 0;
static int   curPhase = 0;
static uint32_t totalCycles = 0;
static uint32_t lineLostEvents = 0;
static uint32_t junctionEvents = 0;
static float  maxErrSeen = 0;
static float  errSum = 0;
static uint32_t errSamples = 0;
static float savedAlpha = 0.7f;  // to restore sensor alpha after run

// =============================================================================
// DEFAULTS / SAVE / LOAD
// =============================================================================
static void setDefaults() {
  memset(&cfg, 0, sizeof(cfg));
  cfg.magic          = LF_CFG_MAGIC;
  cfg.baseHipDelta   = 20;
  cfg.femurDelta     = 20;
  cfg.speedMs        = 500;
  cfg.kp             = 0.30f;
  cfg.kd             = 0.10f;
  cfg.maxCorrection  = 15;
  cfg.steerSign      = -1;    // -1 = inverted (matches physical robot mechanics)
  cfg.sampleMode     = 1;     // per-phase by default
  cfg.stopOnLost     = 1;
  cfg.slowFactor     = 1.0f;
  cfg.emaAlpha       = 0.50f;
  cfg.slowOnError    = 1;
  cfg.slowErrorThresh = 20.0f;
  cfg.slowSpeedMult  = 1.5f;
}

static bool saveCfg() {
  lfPrefs.begin("lfollow", false);
  size_t w = lfPrefs.putBytes("cfg", &cfg, sizeof(cfg));
  lfPrefs.end();
  return w == sizeof(cfg);
}

static void loadCfg() {
  setDefaults();
  lfPrefs.begin("lfollow", true);
  bool needsRewrite = false;
  if (lfPrefs.getBytesLength("cfg") == sizeof(LFConfig)) {
    LFConfig tmp;
    lfPrefs.getBytes("cfg", &tmp, sizeof(tmp));
    if (tmp.magic == LF_CFG_MAGIC) {
      cfg = tmp;
      cfgFromFlash = true;
    } else {
      // Old config magic found (e.g. previous steerSign=+1) -> rewrite with updated defaults
      needsRewrite = true;
    }
  }
  lfPrefs.end();

  if (needsRewrite) {
    saveCfg();
    cfgFromFlash = true;
  }
}

// =============================================================================
// STEERING MATH
// =============================================================================
static void computeSteering() {
  float err = lineErrorMm();
  float dErr = err - prevErr;
  float correction = cfg.kp * err + cfg.kd * dErr;
  correction = constrain(correction, -(float)cfg.maxCorrection, (float)cfg.maxCorrection);
  correction *= cfg.steerSign;
  prevErr = err;
  curCorrection = correction;

  // Differential hip deltas: left side vs right side
  curHipDeltaL = constrain(cfg.baseHipDelta + (int)roundf(correction), 0, 45);
  curHipDeltaR = constrain(cfg.baseHipDelta - (int)roundf(correction), 0, 45);
}

// =============================================================================
// ASYMMETRIC GAIT POSE BUILDER
// =============================================================================
// Like buildGaitPose() but with separate L/R hip deltas for differential steering.
// Leg sides: FL(0)=LEFT, FR(1)=RIGHT, BL(2)=LEFT, BR(3)=RIGHT
static void buildSteerGaitPose(int out[4][3],
    const int hipBase[4], const int hipSign[4],
    int hipDeltaL, int hipDeltaR,
    const int femPlant[4], const int femSign[4],
    const int femDelta[4],
    uint8_t swingMask, uint8_t liftMask)
{
  // Per-leg hip delta based on which side the leg is on
  const int hipDelta[4] = {hipDeltaL, hipDeltaR, hipDeltaL, hipDeltaR};
  for (int leg = 0; leg < 4; leg++) {
    out[leg][0] = hipBase[leg] + (((swingMask >> leg) & 1) ? hipSign[leg] * hipDelta[leg] : 0);
    out[leg][1] = femPlant[leg] + (((liftMask >> leg) & 1) ? femSign[leg] * femDelta[leg] : 0);
    out[leg][2] = GAIT_TIBIA[leg];
    out[leg][0] = constrain(out[leg][0], 0, 180);
    out[leg][1] = constrain(out[leg][1], 0, 180);
  }
}

// =============================================================================
// RESET DIAGNOSTICS
// =============================================================================
static void resetDiag() {
  prevErr = 0; curCorrection = 0;
  curHipDeltaL = cfg.baseHipDelta; curHipDeltaR = cfg.baseHipDelta;
  curPhase = 0; totalCycles = 0;
  lineLostEvents = 0; junctionEvents = 0;
  maxErrSeen = 0; errSum = 0; errSamples = 0;
}

// =============================================================================
// LINE-FOLLOW WALK — Main gait loop with closed-loop steering
// =============================================================================
inline void lineFollowWalk(int steps, int speedMs) {
  syncGaitBase();   // pull base angles from STANDING pose

  lfRunning   = true;
  lfStop      = false;
  lfStepsDone = 0;
  resetDiag();

  // Override sensor EMA for smoother readings during walk
  // (lineSetAlpha stores/restores internally)
  lineSetAlpha(cfg.emaAlpha);

  int totalSteps = (steps == 0) ? INT_MAX : steps;

  // Femur delta array with per-leg trim
  int femDelta[4];
  for (int l = 0; l < 4; l++)
    femDelta[l] = cfg.femurDelta + WALK_FEMUR_TRIM[l];

  // 6-phase trot sequence: {swingMask, liftMask}
  const uint8_t seq[6][2] = {
    {0,                       LF_M_FL | LF_M_BR},              // 1: FL&BR lift
    {LF_M_FL | LF_M_BR,      LF_M_FL | LF_M_BR},              // 2: FL&BR swing
    {LF_M_FL | LF_M_BR,      0},                               // 3: FL&BR plant
    {0,                       LF_M_FR | LF_M_BL},              // 4: FR&BL lift
    {LF_M_FR | LF_M_BL,      LF_M_FR | LF_M_BL},              // 5: FR&BL swing
    {LF_M_FR | LF_M_BL,      0}                                // 6: FR&BL plant
  };

  Serial.printf("[LF] Started: steps=%d speed=%dms Kp=%.2f Kd=%.2f sign=%d\n",
                steps, speedMs, cfg.kp, cfg.kd, cfg.steerSign);

  for (int cycle = 0; cycle < totalSteps && !lfStop; cycle++) {
    for (int p = 0; p < 6 && !lfStop; p++) {
      curPhase = p + 1;
      snprintf(lfPhaseStr, sizeof(lfPhaseStr), "Cycle %d Phase %d/6", cycle + 1, p + 1);

      // ── Read sensor and compute steering ──
      // sampleMode 1 = every phase, sampleMode 0 = only at cycle start
      if (cfg.sampleMode == 1 || p == 0) {
        if (lineFound()) {
          computeSteering();

          // Track diagnostics
          float absErr = fabsf(lineErrorMm());
          if (absErr > maxErrSeen) maxErrSeen = absErr;
          errSum += absErr;
          errSamples++;

          if (lineJunction()) junctionEvents++;
        } else {
          // Line lost during walk
          lineLostEvents++;
          if (cfg.stopOnLost && lineSafeStop()) {
            snprintf(lfPhaseStr, sizeof(lfPhaseStr), "SAFE STOP (line lost)");
            Serial.println("[LF] Safe stop — line lost too long");
            lfStop = true;
            break;
          }
          // If not safe-stop yet, keep going with last known correction
          // (don't reset correction — hold the turn)
        }
      }

      // ── Compute phase speed with optional error-based slowdown ──
      int phaseSpeed = (int)(speedMs * cfg.slowFactor);
      if (cfg.slowOnError && fabsf(prevErr) > cfg.slowErrorThresh) {
        phaseSpeed = (int)(phaseSpeed * cfg.slowSpeedMult);
      }

      // ── Build asymmetric gait pose and execute ──
      int t[4][3];
      buildSteerGaitPose(t, WALK_HIP_BASE, WALK_HIP_SIGN,
                         curHipDeltaL, curHipDeltaR,
                         WALK_FEM_PLANT, FEMUR_LIFT_SIGN, femDelta,
                         seq[p][0], seq[p][1]);
      moveToTarget(t, phaseSpeed);
    }

    if (lfStop) break;

    lfStepsDone++;
    totalCycles++;
    Serial.printf("[LF] Cycle %d/%d err=%.1fmm corr=%.1f° hipL=%d hipR=%d\n",
                  lfStepsDone, (steps == 0) ? -1 : steps,
                  prevErr, curCorrection, curHipDeltaL, curHipDeltaR);
  }

  // Restore sensor EMA to default
  lineSetAlpha(-1);  // -1 = restore previous

  moveToPose(1, 500);  // Return to STANDING
  lfRunning = false;
  snprintf(lfPhaseStr, sizeof(lfPhaseStr), "Done (%d cycles)", lfStepsDone);
  Serial.printf("[LF] Finished. Cycles=%d avgErr=%.1fmm maxErr=%.1fmm lostEvents=%lu\n",
                lfStepsDone,
                errSamples > 0 ? errSum / errSamples : 0.0f,
                maxErrSeen, (unsigned long)lineLostEvents);
}

// =============================================================================
// HTTP HANDLERS
// =============================================================================
static void sendJSON(const char *b) {
  srv->sendHeader("Cache-Control", "no-store");
  srv->sendHeader("Access-Control-Allow-Origin", "*");
  srv->send(200, "application/json", b);
}

// GET /lf/start?steps=N&speed=X
static void handleStart() {
  addCORS();
  if (!lineCalibrated()) {
    srv->send(200, "application/json",
      "{\"ok\":false,\"msg\":\"Line sensor not calibrated. Use /line to calibrate first.\"}");
    return;
  }
  if (robotBusy()) {
    srv->send(200, "application/json", "{\"ok\":false,\"msg\":\"busy\"}");
    return;
  }
  int steps = srv->hasArg("steps") ? srv->arg("steps").toInt() : 0;
  jobSpeed = srv->hasArg("speed")
    ? constrain(srv->arg("speed").toInt(), 50, 3000)
    : cfg.speedMs;
  jobSteps = lfStepsTarget = (steps < 0) ? 0 : steps;
  lfStepsDone = 0;
  lfStop = false;
  lfRunning = true;
  strncpy(lfPhaseStr, "Starting", sizeof(lfPhaseStr));
  Serial.printf("[LF] Queued: steps=%d speed=%dms\n", jobSteps, jobSpeed);
  srv->send(200, "application/json",
    "{\"ok\":true,\"msg\":\"line follow started\"}");
  pendingJob = (Job)8;  // JOB_LINEFOLLOW
}

// GET /lf/stop
static void handleStop() {
  addCORS();
  lfStop = true;
  srv->send(200, "application/json", "{\"ok\":true,\"msg\":\"stopping\"}");
}

// GET /lf/status — live telemetry
static void handleStatus() {
  static char b[800];
  float avgErr = errSamples > 0 ? errSum / errSamples : 0.0f;
  snprintf(b, sizeof(b),
    "{\"running\":%s,\"cycle\":%d,\"target\":%d,\"phase\":%d,\"phaseStr\":\"%s\","
    "\"err\":%.1f,\"correction\":%.1f,\"hipL\":%d,\"hipR\":%d,"
    "\"found\":%d,\"lost\":%d,\"safe\":%d,\"junction\":%d,\"jtype\":\"%s\","
    "\"act\":%u,\"avgErr\":%.1f,\"maxErr\":%.1f,"
    "\"lostEvents\":%lu,\"junEvents\":%lu,\"errSamples\":%lu,"
    "\"cal\":%d,\"kp\":%.3f,\"kd\":%.2f,\"baseHip\":%d,\"maxCorr\":%d,"
    "\"steerSign\":%d,\"sampleMode\":%d,\"speed\":%d,\"slowFactor\":%.1f}",
    lfRunning ? "true" : "false", lfStepsDone, lfStepsTarget, curPhase, lfPhaseStr,
    lineErrorMm(), curCorrection, curHipDeltaL, curHipDeltaR,
    lineFound() ? 1 : 0, lineLost() ? 1 : 0, lineSafeStop() ? 1 : 0,
    lineJunction() ? 1 : 0, lineJunctionType(),
    lineActiveCount(), avgErr, maxErrSeen,
    (unsigned long)lineLostEvents, (unsigned long)junctionEvents, (unsigned long)errSamples,
    lineCalibrated() ? 1 : 0, cfg.kp, cfg.kd, cfg.baseHipDelta, cfg.maxCorrection,
    cfg.steerSign, cfg.sampleMode, cfg.speedMs, cfg.slowFactor);
  sendJSON(b);
}

// GET /lf/config — full config dump
static void handleConfig() {
  char b[500];
  snprintf(b, sizeof(b),
    "{\"hip\":%d,\"fem\":%d,\"speed\":%d,"
    "\"kp\":%.3f,\"kd\":%.2f,\"maxcorr\":%d,"
    "\"sign\":%d,\"sample\":%d,\"stopLost\":%d,"
    "\"slow\":%.2f,\"alpha\":%.2f,"
    "\"slowErr\":%d,\"slowThresh\":%.1f,\"slowMult\":%.2f,"
    "\"flash\":%d}",
    cfg.baseHipDelta, cfg.femurDelta, cfg.speedMs,
    cfg.kp, cfg.kd, cfg.maxCorrection,
    cfg.steerSign, cfg.sampleMode, cfg.stopOnLost,
    cfg.slowFactor, cfg.emaAlpha,
    cfg.slowOnError, cfg.slowErrorThresh, cfg.slowSpeedMult,
    cfgFromFlash ? 1 : 0);
  sendJSON(b);
}

// GET /lf/set?kp=X&kd=Y...
static void handleSet() {
  for (int i = 0; i < srv->args(); i++) {
    String k = srv->argName(i);
    float f = srv->arg(i).toFloat();
    int   n = srv->arg(i).toInt();
    if      (k == "hip")       cfg.baseHipDelta   = constrain(n, 5, 45);
    else if (k == "fem")       cfg.femurDelta      = constrain(n, 5, 45);
    else if (k == "speed")     cfg.speedMs         = constrain(n, 50, 3000);
    else if (k == "kp")        cfg.kp              = constrain(f, 0.0f, 2.0f);
    else if (k == "kd")        cfg.kd              = constrain(f, 0.0f, 2.0f);
    else if (k == "maxcorr")   cfg.maxCorrection   = constrain(n, 1, 40);
    else if (k == "sign")      cfg.steerSign       = (n < 0) ? -1 : 1;
    else if (k == "sample")    cfg.sampleMode      = constrain(n, 0, 1);
    else if (k == "stopLost")  cfg.stopOnLost      = n ? 1 : 0;
    else if (k == "slow")      cfg.slowFactor      = constrain(f, 0.3f, 3.0f);
    else if (k == "alpha")     cfg.emaAlpha        = constrain(f, 0.1f, 1.0f);
    else if (k == "slowErr")   cfg.slowOnError     = n ? 1 : 0;
    else if (k == "slowThresh") cfg.slowErrorThresh = constrain(f, 5.0f, 40.0f);
    else if (k == "slowMult")  cfg.slowSpeedMult   = constrain(f, 1.0f, 3.0f);
  }
  addCORS();
  srv->send(200, "text/plain", "OK");
}

// GET /lf/cmd?n=save|reset
static void handleCmd() {
  String n = srv->arg("n"), m = "OK";
  if (n == "save") {
    m = saveCfg() ? "Saved to flash" : "Save FAILED";
  } else if (n == "reset") {
    setDefaults();
    saveCfg();
    m = "Defaults restored";
  } else if (n == "resetdiag") {
    resetDiag();
    m = "Diagnostics reset";
  } else {
    m = "Unknown command";
  }
  addCORS();
  srv->send(200, "text/plain", m);
}

// GET /lf/sensordata — passthrough to line sensor data for the embedded array view
static void handleSensorData() {
  // This just redirects to /line/data — but we serve it here so the
  // linefollow page can fetch from a single origin path
  srv->sendHeader("Location", "/line/data");
  srv->send(302, "text/plain", "See /line/data");
}

}  // namespace lfns

// =============================================================================
// PUBLIC API
// =============================================================================

// Public wrapper — called from the main .ino job dispatch (JOB_LINEFOLLOW)
inline void lineFollowWalk(int steps, int speedMs) {
  lfns::lineFollowWalk(steps, speedMs);
}

// Include the line-follow web page
#include "line_follow_page.h"

// Call once from setup() after lineBegin()
inline void lineFollowBegin(WebServer &server, const char *ssid) {
  lfns::srv = &server;
  lfns::apName = ssid;
  lfns::loadCfg();
  Serial.printf("[LF] Config %s, Kp=%.2f Kd=%.2f steerSign=%d\n",
                lfns::cfgFromFlash ? "from flash" : "defaults",
                lfns::cfg.kp, lfns::cfg.kd, lfns::cfg.steerSign);

  server.on("/linefollow", []() {
    lfns::srv->send_P(200, "text/html", LF_PAGE);
  });
  server.on("/lf/start",      lfns::handleStart);
  server.on("/lf/stop",       lfns::handleStop);
  server.on("/lf/status",     lfns::handleStatus);
  server.on("/lf/config",     lfns::handleConfig);
  server.on("/lf/set",        lfns::handleSet);
  server.on("/lf/cmd",        lfns::handleCmd);
  server.on("/lf/sensordata", lfns::handleSensorData);
}
