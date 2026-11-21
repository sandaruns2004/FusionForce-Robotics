#pragma once
// =============================================================================
// arm_interface.h  -  2-DOF Robot Arm  (Lift + Grip servos, direct LEDC PWM)
//
// PIN ASSIGNMENT (free GPIOs, confirmed against RUNNER4 pinout doc):
//   ARM_LIFT_PIN = GPIO 47  (DevKit right header pin 17 - free GPIO)
//   ARM_GRIP_PIN = GPIO 44  (U0RXD - free when USB CDC is active)
//
// WIRING:
//   Servo signal wire -> GPIO pin
//   Servo red (VCC)   -> 5-6 V servo power rail  (NEVER 3.3 V)
//   Servo black/brown -> Common GND (shared with ESP32)
//
// SERVO PWM:
//   50 Hz, 500 us (0 deg) .. 2500 us (180 deg), 14-bit LEDC resolution
//
// FEATURES (ported from arm_gripper_esp32_v2.ino):
//   - Smooth servo movement (configurable ms-per-degree stepping)
//   - Preferences-backed config (survives reboot)
//   - Full web API: /arm  /arm/status  /arm/open  /arm/close
//                   /arm/up  /arm/down  /arm/pick  /arm/place
//                   /arm/cfg  /arm/raw  /arm/reset
//   - serviceBackground() called during blocking moves so web stays alive
//
// SEQUENCES:
//   PICK  : OPEN -> DOWN -> CLOSE (grip) -> UP
//   PLACE : DOWN (gripping) -> OPEN -> UP -> CLOSE
//   DOWN  : OPEN first, then lower arm
// =============================================================================

#include <Arduino.h>
#include <WebServer.h>
#include <Preferences.h>
#include <esp_arduino_version.h>

// =============================================================================
// PIN & LEDC CONFIG
// =============================================================================
#ifndef ARM_LIFT_PIN
#define ARM_LIFT_PIN   47
#endif
#ifndef ARM_GRIP_PIN
#define ARM_GRIP_PIN   44
#endif

#define ARM_LEDC_FREQ      50
#define ARM_LEDC_RES       14          // 14-bit = 16383 counts
#define ARM_LIFT_CHANNEL    6
#define ARM_GRIP_CHANNEL    7
#define ARM_LIFT_MIN       20          // hard safety limit
#define ARM_LIFT_MAX      160          // hard safety limit
#define ARM_US_TO_DUTY(us) ((uint32_t)(((float)(us)/20000.0f)*((1<<ARM_LEDC_RES)-1)))

// =============================================================================
// FORWARD DECLARATIONS (from main .ino)
// =============================================================================
extern void serviceBackground();
extern void waitMs(uint32_t ms);
extern volatile bool compStop;

// =============================================================================
// INTERNAL NAMESPACE
// =============================================================================
namespace armns {

// ---- Factory defaults (reference: arm_gripper_esp32_v2.ino) ----------------
static const int DEF_GRIP_OPEN  = 89;
static const int DEF_GRIP_CLOSE = 53;
static const int DEF_LIFT_UP    = 75;
static const int DEF_LIFT_DOWN  = 160;
static const int DEF_GRIP_DELAY = 15;   // ms per degree (speed)
static const int DEF_LIFT_DELAY = 15;
static const int DEF_PAUSE      = 300;  // ms pause between steps in sequences

// ---- Live config (loaded from NVS on armBegin) ------------------------------
int GRIP_OPEN  = DEF_GRIP_OPEN;
int GRIP_CLOSE = DEF_GRIP_CLOSE;
int LIFT_UP    = DEF_LIFT_UP;
int LIFT_DOWN  = DEF_LIFT_DOWN;
int gripDelay  = DEF_GRIP_DELAY;
int liftDelay  = DEF_LIFT_DELAY;
int stepPause  = DEF_PAUSE;

// ---- Runtime state ----------------------------------------------------------
int  gripPos   = DEF_GRIP_CLOSE;
int  liftPos   = DEF_LIFT_UP;
bool gripOpen  = false;
bool sInit     = false;
Preferences prefs;

// ---- Persistence ------------------------------------------------------------
void loadCfg() {
    prefs.begin("arm2", true);
    GRIP_OPEN  = constrain(prefs.getInt("gopen",  DEF_GRIP_OPEN),  0, 180);
    GRIP_CLOSE = constrain(prefs.getInt("gclose", DEF_GRIP_CLOSE), 0, 180);
    LIFT_UP    = constrain(prefs.getInt("lup",    DEF_LIFT_UP),    ARM_LIFT_MIN, ARM_LIFT_MAX);
    LIFT_DOWN  = constrain(prefs.getInt("ldown",  DEF_LIFT_DOWN),  ARM_LIFT_MIN, ARM_LIFT_MAX);
    gripDelay  = constrain(prefs.getInt("gs",     DEF_GRIP_DELAY), 1, 100);
    liftDelay  = constrain(prefs.getInt("ls",     DEF_LIFT_DELAY), 1, 100);
    stepPause  = constrain(prefs.getInt("pause",  DEF_PAUSE),      0, 3000);
    prefs.end();
}

void saveCfg(const char* key, int v) {
    prefs.begin("arm2", false);
    prefs.putInt(key, v);
    prefs.end();
}

void resetCfg() {
    GRIP_OPEN=DEF_GRIP_OPEN; GRIP_CLOSE=DEF_GRIP_CLOSE;
    LIFT_UP=DEF_LIFT_UP; LIFT_DOWN=DEF_LIFT_DOWN;
    gripDelay=DEF_GRIP_DELAY; liftDelay=DEF_LIFT_DELAY; stepPause=DEF_PAUSE;
    prefs.begin("arm2", false); prefs.clear(); prefs.end();
}

// ---- LEDC write helpers -----------------------------------------------------
uint32_t degToDuty(int deg) {
    deg = constrain(deg, 0, 180);
    uint32_t us = (uint32_t)map(deg, 0, 180, 500, 2500);
    return ARM_US_TO_DUTY(us);
}

void rawLift(int deg) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWrite(ARM_LIFT_PIN, degToDuty(deg));
#else
    ledcWrite(ARM_LIFT_CHANNEL, degToDuty(deg));
#endif
    liftPos = deg;
}

void rawGrip(int deg) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWrite(ARM_GRIP_PIN, degToDuty(deg));
#else
    ledcWrite(ARM_GRIP_CHANNEL, degToDuty(deg));
#endif
    gripPos = deg;
}

// ---- Smooth servo sweep (delays serviceBackground between each degree) ------
void sweepLift(int target, int delayMs) {
    target = constrain(target, ARM_LIFT_MIN, ARM_LIFT_MAX);
    int dir = (target > liftPos) ? 1 : -1;
    while (liftPos != target) {
        liftPos += dir;
        rawLift(liftPos);
        serviceBackground();
        delay(constrain(delayMs, 1, 100));
    }
}

void sweepGrip(int target, int delayMs) {
    target = constrain(target, 0, 180);
    int dir = (target > gripPos) ? 1 : -1;
    while (gripPos != target) {
        gripPos += dir;
        rawGrip(gripPos);
        serviceBackground();
        delay(constrain(delayMs, 1, 100));
    }
}

void pauseStep() { waitMs(stepPause); }

// ---- Status JSON ------------------------------------------------------------
String statusJson() {
    char b[300];
    snprintf(b, sizeof(b),
        "{\"g\":%d,\"l\":%d,\"open\":%s,"
        "\"gopen\":%d,\"gclose\":%d,\"lup\":%d,\"ldown\":%d,"
        "\"gs\":%d,\"ls\":%d,\"pause\":%d,"
        "\"lmin\":%d,\"lmax\":%d,"
        "\"grip_pin\":%d,\"lift_pin\":%d}",
        gripPos, liftPos, gripOpen?"true":"false",
        GRIP_OPEN, GRIP_CLOSE, LIFT_UP, LIFT_DOWN,
        gripDelay, liftDelay, stepPause,
        ARM_LIFT_MIN, ARM_LIFT_MAX,
        ARM_GRIP_PIN, ARM_LIFT_PIN);
    return String(b);
}

} // namespace armns

// =============================================================================
// PUBLIC API - INIT
// =============================================================================
inline void armInit() {
    armns::loadCfg();
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcAttach(ARM_LIFT_PIN, ARM_LEDC_FREQ, ARM_LEDC_RES);
    ledcAttach(ARM_GRIP_PIN, ARM_LEDC_FREQ, ARM_LEDC_RES);
#else
    ledcSetup(ARM_LIFT_CHANNEL, ARM_LEDC_FREQ, ARM_LEDC_RES);
    ledcSetup(ARM_GRIP_CHANNEL, ARM_LEDC_FREQ, ARM_LEDC_RES);
    ledcAttachPin(ARM_LIFT_PIN, ARM_LIFT_CHANNEL);
    ledcAttachPin(ARM_GRIP_PIN, ARM_GRIP_CHANNEL);
#endif
    // Startup pose: arm UP first, then gripper CLOSE
    armns::rawLift(armns::LIFT_UP);   delay(600);
    armns::rawGrip(armns::GRIP_CLOSE); delay(400);
    armns::gripOpen = false;
    armns::sInit = true;
    Serial.printf("[ARM] Init: Lift=GPIO%d (UP=%d deg) Grip=GPIO%d (CLOSE=%d deg)\n",
                  ARM_LIFT_PIN, armns::LIFT_UP, ARM_GRIP_PIN, armns::GRIP_CLOSE);
}

// =============================================================================
// PUBLIC API - DIRECT ANGLE SET (no smooth sweep, for competition FSM speed)
// =============================================================================
inline void armSetLift(int deg) { armns::rawLift(constrain(deg, ARM_LIFT_MIN, ARM_LIFT_MAX)); }
inline void armSetGrip(int deg) { armns::rawGrip(constrain(deg, 0, 180)); }

// =============================================================================
// PUBLIC API - INDIVIDUAL MOVES (smooth sweep)
// =============================================================================
inline bool armUp() {
    Serial.println("[ARM] Lift -> UP");
    armns::sweepLift(armns::LIFT_UP, armns::liftDelay);
    return true;
}
// armDown: open gripper first then lower (competition sequence)
inline bool armDown() {
    Serial.println("[ARM] Grip -> OPEN, then Lift -> DOWN");
    armns::sweepGrip(armns::GRIP_OPEN, armns::gripDelay);
    armns::gripOpen = true;
    armns::pauseStep();
    armns::sweepLift(armns::LIFT_DOWN, armns::liftDelay);
    return true;
}
// armLiftUp / armLiftDown: move ONLY the lift servo, no gripper change
// Used by the dashboard UP/DOWN buttons with inline angle sliders
inline bool armLiftUp(int angle = -1) {
    int target = (angle < 0) ? armns::LIFT_UP : constrain(angle, ARM_LIFT_MIN, ARM_LIFT_MAX);
    Serial.printf("[ARM] LiftOnly -> UP to %d deg\n", target);
    armns::sweepLift(target, armns::liftDelay);
    return true;
}
inline bool armLiftDown(int angle = -1) {
    int target = (angle < 0) ? armns::LIFT_DOWN : constrain(angle, ARM_LIFT_MIN, ARM_LIFT_MAX);
    Serial.printf("[ARM] LiftOnly -> DOWN to %d deg\n", target);
    armns::sweepLift(target, armns::liftDelay);
    return true;
}
inline bool armOpen() {
    Serial.println("[ARM] Grip -> OPEN");
    armns::sweepGrip(armns::GRIP_OPEN, armns::gripDelay);
    armns::gripOpen = true;
    return true;
}
inline bool armClose() {
    Serial.println("[ARM] Grip -> CLOSE");
    armns::sweepGrip(armns::GRIP_CLOSE, armns::gripDelay);
    armns::gripOpen = false;
    return true;
}
inline void armSafePosition() {
    armns::rawGrip(armns::GRIP_CLOSE);
    armns::rawLift(armns::LIFT_UP);
    armns::gripOpen = false;
    waitMs(400);
}

// =============================================================================
// PUBLIC API - HIGH-LEVEL SEQUENCES
// =============================================================================
inline bool armPickBall() {
    if (!armns::sInit) return false;
    Serial.println("[ARM] === PICK: open->down->grip->up ===");
    // 1. Open gripper
    armns::sweepGrip(armns::GRIP_OPEN, armns::gripDelay); armns::gripOpen=true;
    armns::pauseStep(); if (compStop) return false;
    // 2. Lower arm
    armns::sweepLift(armns::LIFT_DOWN, armns::liftDelay);
    armns::pauseStep(); if (compStop) return false;
    // 3. Close gripper (grab ball)
    armns::sweepGrip(armns::GRIP_CLOSE, armns::gripDelay); armns::gripOpen=false;
    armns::pauseStep(); if (compStop) return false;
    // 4. Raise arm
    armns::sweepLift(armns::LIFT_UP, armns::liftDelay);
    Serial.println("[ARM] Pick complete");
    return true;
}

inline bool armPlaceBall() {
    if (!armns::sInit) return false;
    Serial.println("[ARM] === PLACE: down->open->up->close ===");
    // 1. Lower arm while gripping
    armns::sweepLift(armns::LIFT_DOWN, armns::liftDelay);
    armns::pauseStep(); if (compStop) return false;
    // 2. Release ball
    armns::sweepGrip(armns::GRIP_OPEN, armns::gripDelay); armns::gripOpen=true;
    armns::pauseStep(); if (compStop) return false;
    // 3. Raise arm
    armns::sweepLift(armns::LIFT_UP, armns::liftDelay);
    armns::pauseStep(); if (compStop) return false;
    // 4. Close gripper (clean travel pos)
    armns::sweepGrip(armns::GRIP_CLOSE, armns::gripDelay); armns::gripOpen=false;
    Serial.println("[ARM] Place complete");
    return true;
}

// =============================================================================
// PUBLIC API - WEB ROUTES  (call armBegin() from setup() AFTER server routes)
// =============================================================================
inline void armBegin(WebServer &server) {
    using namespace armns;

    // Dashboard page
    server.on("/arm", [&server]() {
        extern const char ARM_PAGE[];
        server.send_P(200, "text/html", ARM_PAGE);
    });

    // Status JSON (live readout)
    server.on("/arm/status", [&server]() {
        server.sendHeader("Cache-Control","no-store");
        server.send(200, "application/json", armns::statusJson());
    });

    // Individual actions - each runs the motion then returns updated status
    auto sendSt = [&server]() {
        server.sendHeader("Cache-Control","no-store");
        server.send(200,"application/json", armns::statusJson());
    };

    server.on("/arm/open",  [sendSt](){ armOpen();  sendSt(); });
    server.on("/arm/close", [sendSt](){ armClose(); sendSt(); });
    server.on("/arm/up",    [sendSt](){ armUp();    sendSt(); });
    server.on("/arm/down",  [sendSt](){ armDown();  sendSt(); });
    server.on("/arm/pick",  [sendSt](){ armPickBall();  sendSt(); });
    server.on("/arm/place", [sendSt](){ armPlaceBall(); sendSt(); });

    // /arm/liftup?a=75   — move ONLY lift to angle (default=LIFT_UP), no gripper
    // /arm/liftdown?a=150 — move ONLY lift to angle (default=LIFT_DOWN), no gripper
    // The dashboard UP/DOWN buttons use /arm/raw?l=X which is instant (no sweep).
    // These routes use smooth sweep and are for the action buttons with sliders.
    server.on("/arm/liftup", [&server, sendSt]() {
        int ang = server.hasArg("a") ? constrain(server.arg("a").toInt(), ARM_LIFT_MIN, ARM_LIFT_MAX) : -1;
        armLiftUp(ang);
        sendSt();
    });
    server.on("/arm/liftdown", [&server, sendSt]() {
        int ang = server.hasArg("a") ? constrain(server.arg("a").toInt(), ARM_LIFT_MIN, ARM_LIFT_MAX) : -1;
        armLiftDown(ang);
        sendSt();
    });

    // /arm/raw?g=90&l=120  - set raw angle instantly (no sweep, for calibration)
    server.on("/arm/raw", [&server, sendSt]() {
        if (server.hasArg("g")) armSetGrip(server.arg("g").toInt());
        if (server.hasArg("l")) armSetLift(server.arg("l").toInt());
        sendSt();
    });

    // /arm/cfg?k=gopen&v=89&p=1  - update config key, optionally preview
    server.on("/arm/cfg", [&server, sendSt]() {
        String k = server.arg("k");
        int    v = server.arg("v").toInt();
        bool   p = (server.arg("p") == "1");
        bool   ok = true;
        if      (k=="gopen")  { GRIP_OPEN  = constrain(v,0,180);       saveCfg("gopen",  GRIP_OPEN);  if(p) armOpen(); }
        else if (k=="gclose") { GRIP_CLOSE = constrain(v,0,180);       saveCfg("gclose", GRIP_CLOSE); if(p) armClose(); }
        else if (k=="lup")    { LIFT_UP    = constrain(v,ARM_LIFT_MIN,ARM_LIFT_MAX); saveCfg("lup",LIFT_UP);   if(p) armUp(); }
        else if (k=="ldown")  { LIFT_DOWN  = constrain(v,ARM_LIFT_MIN,ARM_LIFT_MAX); saveCfg("ldown",LIFT_DOWN); if(p) sweepLift(LIFT_DOWN,liftDelay); }
        else if (k=="gs")     { gripDelay  = constrain(v,1,100);        saveCfg("gs",    gripDelay); }
        else if (k=="ls")     { liftDelay  = constrain(v,1,100);        saveCfg("ls",    liftDelay); }
        else if (k=="pause")  { stepPause  = constrain(v,0,3000);       saveCfg("pause", stepPause); }
        else ok = false;
        if (!ok) { server.send(400,"text/plain","Unknown key"); return; }
        sendSt();
    });

    // /arm/reset - restore factory defaults
    server.on("/arm/reset", [sendSt]() {
        armns::resetCfg();
        sendSt();
    });

    Serial.printf("[ARM] Web routes: /arm  /arm/status  /arm/open  /arm/close\n");
    Serial.printf("[ARM]             /arm/up  /arm/down  /arm/liftup  /arm/liftdown\n");
    Serial.printf("[ARM]             /arm/pick  /arm/place  /arm/raw  /arm/cfg  /arm/reset\n");
}

// =============================================================================
// PUBLIC API - GENERIC COMMAND (for competition FSM: armSendCmd("pick") etc.)
// =============================================================================
inline bool armSendCmd(const char* cmd, uint32_t /*unused*/ = 2000) {
    String c = String(cmd); c.trim(); c.toLowerCase();
    if (c=="pick")     return armPickBall();
    if (c=="place")    return armPlaceBall();
    if (c=="up")       return armUp();
    if (c=="down")     return armDown();
    if (c=="liftup")   return armLiftUp();
    if (c=="liftdown") return armLiftDown();
    if (c=="open")     return armOpen();
    if (c=="close")    return armClose();
    if (c=="safe")     { armSafePosition(); return true; }
    Serial.printf("[ARM] armSendCmd: unknown '%s'\n", cmd);
    return false;
}

// Legacy aliases
inline bool armPickUp()    { return armPickBall();  }
inline bool armPlaceDown() { return armPlaceBall(); }
inline bool armPing()      { return true; }