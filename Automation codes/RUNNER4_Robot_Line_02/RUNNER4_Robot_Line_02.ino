// =============================================================================
// FusionForce RUNNER4 — 12 DOF Quadruped Robot
// MAIN ROOT : PCA9685 servo control + pose presets + FK/IK + walk/turn gaits
// MODULES   : TCS34725 colour sensor (color_sensor.cpp/.h/color_page.h)
//             9x TCRT5000 line array (line_sensor.cpp/.h/line_page.h)
// BOARD     : ESP32-S3
// VERSION   : 5.2  (gait tuned by single variables, follows STANDING pose)
// =============================================================================
//
// WEB ROUTES (one WiFi AP, one web server)
//   http://192.168.4.1/        -> robot control page
//   http://192.168.4.1/color   -> colour sensor lab
//   http://192.168.4.1/line    -> 9-ch line array lab
//   /color/data /color/config /color/set /color/cmd /color/wifi /color/export
//   /line/data  /line/config  /line/set  /line/cmd  /line/wifi  /line/export
//
// SKETCH FOLDER (ALL files in ONE folder named RUNNER4_Robot_Line):
//   RUNNER4_Robot_Line.ino     <- this file (robot)
//   robot_page.h               <- robot web page
//   color_sensor.h / .cpp      <- colour sensor module (LED pin GPIO 15)
//   color_page.h               <- colour web page
//   line_sensor.h / .cpp       <- line array module
//   line_page.h                <- line array web page
//
// WIRING (v5.0 PIN MAP - ADC1 only, because WiFi AP blocks ADC2 on ESP32-S3)
//   I2C (PCA9685 0x40 + TCS34725 0x29, shared bus):
//     GPIO 11 (SDA) -> PCA9685 SDA  and  TCS34725 SDA
//     GPIO 12 (SCL) -> PCA9685 SCL  and  TCS34725 SCL
//     3.3V / GND    -> PCA9685 VCC/GND and TCS34725 VIN/GND
//   TCS34725 LED pin : GPIO 15  (change COLOR_LED_PIN in color_sensor.h)
//   Line array (TCRT5000 analog outputs, S1 far LEFT ... S9 far RIGHT, 3.3V
//   ONLY):
//     S1=GPIO1  S2=GPIO2  S3=GPIO4  S4=GPIO5  S5=GPIO6
//     S6=GPIO7  S7=GPIO8  S8=GPIO9  S9=GPIO10
//     (edit LINE_PIN_LIST in line_sensor.h to change; keep within GPIO 1..10)
//   Servo power: 6V BEC -> PCA9685 V+ terminal (NEVER from the ESP32 3.3V)
//
// REQUIRED LIBRARIES: "Adafruit PWM Servo Driver Library" (WiFi/WebServer/Wire/
//   Preferences are built into the ESP32 core). No TCS34725 library needed.
//
// BOARD SETTINGS: ESP32S3 Dev Module, USB CDC On Boot = Enabled, 921600 baud.
//
// HOW MOTION AND SENSING SHARE ONE CPU
//   HTTP handlers never run long motions. They validate, send the response and
//   queue a "job". loop() runs the job; every wait inside a motion calls
//   serviceBackground() = web server + colour sensor + line array, so Stop
//   buttons work and both sensors keep updating while the robot walks.
//
// CHANNEL MAP:
//   CH 0:FL_HIP  CH 1:FL_FEMUR  CH 2:FL_TIBIA
//   CH 3:FR_HIP  CH 4:FR_FEMUR  CH 5:FR_TIBIA
//   CH10:BL_HIP  CH11:BL_FEMUR  CH14:BL_TIBIA
//   CH 6:BR_HIP  CH 8:BR_FEMUR  CH 9:BR_TIBIA
// =============================================================================

#include <Adafruit_PWMServoDriver.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_system.h>
#include <limits.h>
#include <math.h>

#include "color_sensor.h" // colour module API
#include "line_sensor.h"  // line array module API
#include "robot_page.h"   // INDEX_HTML

struct Vec3f {
  float x, y, z;
};

// ── WiFi AP ──────────────────────────────────────────────────────────────────
const char *AP_SSID = "RUNNER4-Config";
const char *AP_PASS = "runner4robot";

// ── I2C (shared by PCA9685 and TCS34725) ─────────────────────────────────────
#define I2C_SDA 11
#define I2C_SCL 12
#define I2C_HZ 400000

// ── PCA9685 ──────────────────────────────────────────────────────────────────
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
#define SERVOMIN 150
#define SERVOMAX 600
#define SERVO_FREQ 50

// ── Leg geometry (mm) ────────────────────────────────────────────────────────
#define L1_MM 30.0f
#define L2_MM 60.0f
#define L3_MM 80.0f

// ── Channels ─────────────────────────────────────────────────────────────────
#define FL_HIP 0
#define FL_FEMUR 1
#define FL_TIBIA 2
#define FR_HIP 3
#define FR_FEMUR 4
#define FR_TIBIA 5
#define BL_HIP 10
#define BL_FEMUR 11
#define BL_TIBIA 14
#define BR_HIP 6
#define BR_FEMUR 8
#define BR_TIBIA 9

// legChannels[leg][0=hip,1=femur,2=tibia]; leg 0=FL 1=FR 2=BL 3=BR
const uint8_t legChannels[4][3] = {{FL_HIP, FL_FEMUR, FL_TIBIA},
                                   {FR_HIP, FR_FEMUR, FR_TIBIA},
                                   {BL_HIP, BL_FEMUR, BL_TIBIA},
                                   {BR_HIP, BR_FEMUR, BR_TIBIA}};
const int legMirror[4] = {1, -1, 1, -1};

// ── Pose presets [pose][leg][hip,femur,tibia] — editable from the web UI ─────
int poseAngles[13][4][3] = {
    // 0 INITIAL (HOME)
    {{45, 80, 160}, {94, 80, 0}, {94, 35, 10}, {45, 92, 160}},
    // 1 STANDING
    {{45, 15, 105}, {94, 140, 50}, {94, 94, 55}, {45, 37, 115}},
    // 2 LOW CROUCH
    {{45, 110, 140}, {94, 110, 40}, {94, 110, 140}, {45, 110, 140}},
    // 3 LOW STAND
    {{45, 29, 119}, {94, 112, 35}, {94, 86, 50}, {45, 48, 120}},
    // 4 FL LEG UP
    {{45, 38, 115}, {94, 125, 35}, {94, 101, 45}, {45, 80, 160}},
    // 5 FR LEG UP
    {{45, 80, 160}, {94, 55, 20}, {94, 56, 160}, {45, 92, 160}},
    // 6 TROT A
    {{45, 60, 120}, {94, 80, 0}, {94, 56, 160}, {45, 72, 140}},
    // 7 TROT B
    {{45, 80, 160}, {94, 55, 20}, {94, 36, 130}, {45, 92, 160}},
    // 8 STRETCH
    {{45, 90, 90}, {94, 90, 90}, {94, 90, 90}, {45, 90, 90}},
    // 9 WALK A basic
    {{15, 45, 115}, {94, 135, 45}, {94, 111, 55}, {75, 57, 115}},
    // 10 WALK B basic
    {{15, 20, 115}, {124, 115, 45}, {64, 91, 55}, {75, 32, 115}},
    // 11 WALK A enhanced
    {{15, 45, 130}, {94, 135, 45}, {94, 111, 55}, {75, 57, 130}},
    // 12 WALK B enhanced
    {{15, 20, 115}, {124, 115, 30}, {64, 91, 30}, {75, 32, 115}}};
#define POSE_COUNT 13

// =============================================================================
// GAIT SETTINGS: change these numbers to tune walking and turning
// =============================================================================
// Leg order everywhere: 0=FL  1=FR  2=BL  3=BR

// ---- WALK ----
int WALK_HIP_DELTA = 30;   // hip swing, degrees (45->15 = 30)
int WALK_FEMUR_DELTA = 25; // femur lift, degrees

// Optional per-leg femur correction. The original code lifted FR by 15, not 25,
// so FR is -10. Set all four to 0 for an identical lift on every leg.
const int WALK_FEMUR_TRIM[4] = {0, -10, 0, 0};

// ---- TURN ----
int TURN_HIP_DELTA = 20;   // hip swing, degrees (same on all 4 legs)
int TURN_FEMUR_DELTA = 20; // femur lift, degrees

// ---- CRAWL (1-leg-at-a-time wave gait) ----
int CRAWL_HIP_DELTA = 30;   // hip swing per step, degrees
int CRAWL_FEMUR_DELTA = 15; // femur lift per step, degrees

// ---- Signs and base values (change only if you remount or recalibrate servos)
// Hip swing direction while walking: FL -, FR +, BL -, BR +
const int WALK_HIP_SIGN[4] = {-1, +1, -1, +1};
// Hip direction while turning: all legs share one sign (right = -1, left = +1)
// Femur lift direction: FL +, FR -, BL -, BR +
const int FEMUR_LIFT_SIGN[4] = {+1, -1, +1, +1};

// ---- Gait follows the STANDING pose (pose 1) ----
// Change the STANDING angles (in the poseAngles table, or from the web page)
// and the walk/turn gaits follow automatically. The offsets below are the
// difference between STANDING and the original gait angles, so with the
// default STANDING angles the gait is exactly the same as before.
// Leg order: FL, FR, BL, BR
const int WALK_FEM_OFFSET[4] = {0, -5, +22, -5};
const int TURN_FEM_OFFSET[4] = {+5, 0, +17, 0};
const int GAIT_TIB_OFFSET[4] = {0, -1, 0, 0};

// Filled by syncGaitBase() from poseAngles[1] each time a gait starts
int WALK_HIP_BASE[4], WALK_FEM_PLANT[4];
int TURN_HIP_BASE[4], TURN_FEM_PLANT[4];
int GAIT_TIBIA[4];

void syncGaitBase() {
  for (int l = 0; l < 4; l++) {
    WALK_HIP_BASE[l] = TURN_HIP_BASE[l] = poseAngles[1][l][0];
    WALK_FEM_PLANT[l] = poseAngles[1][l][1] + WALK_FEM_OFFSET[l];
    TURN_FEM_PLANT[l] = poseAngles[1][l][1] + TURN_FEM_OFFSET[l];
    GAIT_TIBIA[l] = poseAngles[1][l][2] + GAIT_TIB_OFFSET[l];
  }
}

// Leg bit masks
#define M_FL 1
#define M_FR 2
#define M_BL 4
#define M_BR 8

int servoAngles[16] = {0};

WebServer server(80);
bool serverStarted = false;
volatile uint32_t apConnectCount = 0;
volatile uint32_t apDisconnectCount = 0;
volatile uint32_t apStopCount = 0;
esp_reset_reason_t bootResetReason = ESP_RST_UNKNOWN;

// Wi-Fi events run on a different task. Only update counters here.
void onWiFiEvent(WiFiEvent_t event) {
  switch (event) {
  case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
    ++apConnectCount;
    break;
  case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
    ++apDisconnectCount;
    break;
  case ARDUINO_EVENT_WIFI_AP_STOP:
    ++apStopCount;
    break;
  default:
    break;
  }
}

// =============================================================================
// JOB SYSTEM — handlers queue work, loop() executes it
// =============================================================================
enum Job : uint8_t {
  JOB_NONE,
  JOB_HOME,
  JOB_LEGHOME,
  JOB_POSE,
  JOB_WALK,
  JOB_TURN,
  JOB_SWEEP,
  JOB_CRAWL
};
volatile Job pendingJob = JOB_NONE;
bool jobRunning = false;
int jobArg = 0, jobSpeed = 0, jobSteps = 0, jobVariant = 0;
bool jobRight = false;

inline bool robotBusy() { return jobRunning || pendingJob != JOB_NONE; }

// Walk / turn state
volatile bool walkRunning = false, walkStop = false;
int walkStepsTarget = 0, walkStepsDone = 0;
volatile bool turnRunning = false, turnStop = false;
int turnStepsTarget = 0, turnStepsDone = 0;
char turnDirection[6] = "none";
char turnPhaseStr[32] = "";

// Crawl state
volatile bool crawlRunning = false, crawlStop = false;
int crawlStepsTarget = 0, crawlStepsDone = 0;
char crawlPhaseStr[32] = "";

// =============================================================================
// BACKGROUND SERVICE — called from every wait inside motion code
// =============================================================================
void serviceBackground() {
  if (serverStarted)
    server.handleClient();
  colorPoll();
  linePoll();
}

// Wait ms while keeping WiFi + colour sensor + line array alive
void waitMs(uint32_t ms) {
  uint32_t t0 = millis();
  do {
    serviceBackground();
    if (millis() - t0 >= ms)
      break;
    delay(1);
  } while (true);
}

// =============================================================================
// SERVO HELPERS
// =============================================================================
uint16_t angleToPulse(int angle) {
  angle = constrain(angle, 0, 180);
  return (uint16_t)map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

void setServo(uint8_t ch, int angle) {
  angle = constrain(angle, 0, 180);
  servoAngles[ch] = angle;
  pwm.setPWM(ch, 0, angleToPulse(angle));
}

void setAllServosHome() {
  Serial.println("[HOME] All 12 servos -> INITIAL pose");
  for (int leg = 0; leg < 4; leg++)
    for (int joint = 0; joint < 3; joint++) {
      setServo(legChannels[leg][joint], poseAngles[0][leg][joint]);
      waitMs(10); // stagger to reduce current spike
    }
}

void setLegHome(int leg) {
  for (int joint = 0; joint < 3; joint++)
    setServo(legChannels[leg][joint], poseAngles[0][leg][joint]);
}

// Linear interpolation of all 12 servos over speedMs (0 = instant)
void moveToTarget(const int targetAngles[4][3], int speedMs) {
  if (speedMs <= 0) {
    for (int leg = 0; leg < 4; leg++) {
      for (int joint = 0; joint < 3; joint++) {
        setServo(legChannels[leg][joint], targetAngles[leg][joint]);
        waitMs(8);
      }
    }
    return;
  }
  int maxDiff = 0;
  for (int leg = 0; leg < 4; leg++)
    for (int joint = 0; joint < 3; joint++) {
      int diff =
          abs(targetAngles[leg][joint] - servoAngles[legChannels[leg][joint]]);
      if (diff > maxDiff)
        maxDiff = diff;
    }
  if (maxDiff == 0)
    return;

  int startAngles[16];
  memcpy(startAngles, servoAngles, sizeof(servoAngles));
  unsigned long moveStart = millis();
  for (int step = 1; step <= maxDiff; step++) {
    float t = (float)step / (float)maxDiff;
    for (int leg = 0; leg < 4; leg++)
      for (int joint = 0; joint < 3; joint++) {
        uint8_t ch = legChannels[leg][joint];
        setServo(ch, startAngles[ch] +
                         (int)roundf(
                             t * (targetAngles[leg][joint] - startAngles[ch])));
      }
    unsigned long stepTarget =
        moveStart + (unsigned long)((long)speedMs * step / maxDiff);
    unsigned long now = millis();
    if (stepTarget > now)
      waitMs(stepTarget - now);
    else
      serviceBackground();
  }
  Serial.printf("[POSE] Transition done. Steps=%d, duration=%lums\n", maxDiff,
                millis() - moveStart);
}

void moveToPose(int id, int speedMs) {
  if (id < 0 || id >= POSE_COUNT)
    return;
  moveToTarget(poseAngles[id], speedMs);
}

// =============================================================================
// KINEMATICS
// =============================================================================
Vec3f computeFK(int hipDeg, int femurDeg, int tibiaDeg, int mirror) {
  const float a1 = (hipDeg - 90) * DEG_TO_RAD;
  const float a2 = (femurDeg - 90) * DEG_TO_RAD;
  const float a3 = (tibiaDeg - 90) * DEG_TO_RAD;
  const float reach = L1_MM + L2_MM * cosf(a2) + L3_MM * cosf(a2 + a3);
  Vec3f p;
  p.x = reach * cosf(a1);
  p.y = reach * sinf(a1) * (float)mirror;
  p.z = L2_MM * sinf(a2) + L3_MM * sinf(a2 + a3);
  return p;
}

bool computeIK(float px, float py, float pz, int mirror, int *hipOut,
               int *femurOut, int *tibiaOut) {
  py *= (float)mirror;
  const float a1 = atan2f(py, px);
  const float r = sqrtf(px * px + py * py) - L1_MM;
  const float d = sqrtf(r * r + pz * pz);
  if (d > L2_MM + L3_MM - 0.5f || d < fabsf(L2_MM - L3_MM) + 0.5f)
    return false;
  float cosA3 = constrain((d * d - L2_MM * L2_MM - L3_MM * L3_MM) /
                              (2.0f * L2_MM * L3_MM),
                          -1.0f, 1.0f);
  const float a3 = acosf(cosA3);
  const float a2 =
      atan2f(pz, r) - atan2f(L3_MM * sinf(a3), L2_MM + L3_MM * cosf(a3));
  int hip = (int)roundf(a1 * RAD_TO_DEG) + 90;
  int femur = (int)roundf(a2 * RAD_TO_DEG) + 90;
  int tibia = (int)roundf(a3 * RAD_TO_DEG) + 90;
  if (hip < 0 || hip > 180 || femur < 0 || femur > 180 || tibia < 0 ||
      tibia > 180)
    return false;
  *hipOut = hip;
  *femurOut = femur;
  *tibiaOut = tibia;
  return true;
}

void i2cScan() {
  Serial.println("-------------------------------------------------");
  Serial.println("I2C Bus Scan (expect 0x29 colour sensor, 0x40 PCA9685):");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  [OK] 0x%02X\n", addr);
      found++;
    }
  }
  if (found == 0)
    Serial.println("  [!!] No I2C devices found!");
  Serial.println("-------------------------------------------------");
}

// =============================================================================
// JSON HELPERS
// =============================================================================
String stateJSON() {
  String j = "{\"a\":[";
  for (int i = 0; i < 16; i++) {
    j += servoAngles[i];
    if (i < 15)
      j += ',';
  }
  return j + "]}";
}

// Angles the robot WILL have after pose id (so the browser updates immediately)
String poseJSON(int id) {
  int t[16];
  memcpy(t, servoAngles, sizeof(t));
  for (int leg = 0; leg < 4; leg++)
    for (int joint = 0; joint < 3; joint++)
      t[legChannels[leg][joint]] = poseAngles[id][leg][joint];
  String j = "{\"a\":[";
  for (int i = 0; i < 16; i++) {
    j += t[i];
    if (i < 15)
      j += ',';
  }
  return j + "]}";
}

void addCORS() { server.sendHeader("Access-Control-Allow-Origin", "*"); }

bool rejectIfBusy() {
  if (!robotBusy())
    return false;
  addCORS();
  server.send(409, "application/json", "{\"ok\":false,\"msg\":\"busy\"}");
  return true;
}

// =============================================================================
// GAIT POSE BUILDER
// =============================================================================
// Build one 4x3 target pose.
//  swingMask : legs whose hip is moved by hipSign*hipDelta
//  liftMask  : legs whose femur is moved by femSign*femDelta
void buildGaitPose(int out[4][3], const int hipBase[4], const int hipSign[4],
                   int hipDelta, const int femPlant[4], const int femSign[4],
                   const int femDelta[4], uint8_t swingMask, uint8_t liftMask) {
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
// WALK GAIT (uses WALK_HIP_DELTA and WALK_FEMUR_DELTA)
// =============================================================================
// void walkForward(int steps, int speedMs, int variant) {
//   syncGaitBase(); // gait follows the STANDING pose
//   walkRunning = true;
//   walkStop = false;
//   walkStepsDone = 0;
//   int totalCycles = (steps == 0) ? INT_MAX : steps;

//   int femDelta[4];
//   for (int l = 0; l < 4; l++)
//     femDelta[l] = WALK_FEMUR_DELTA + WALK_FEMUR_TRIM[l];

//   // {swingMask, liftMask} for the 6 phases
//   const uint8_t seq[6][2] = {{0, M_FL | M_BR},               // 1: FL&BR
//   femurs
//                              {M_FL | M_BR, M_FL | M_BR}, // 2: swing
//                              {M_FL | M_BR, 0}, // 3: plant FL&BR
//                              {0, M_FR | M_BL},     // 4: FL&BR hips
//                              {M_FR | M_BL, M_FR | M_BL}, //
//                              5: swing FR & BL hips{M_FR | M_BL, 0}};  // 6:
//                              plant FR&BL
//   femurs

//       for (int i = 0; i < totalCycles && !walkStop; i++) {
//     for (int p = 0; p < 6 && !walkStop; p++) {
//       int t[4][3];
//       buildGaitPose(t, WALK_HIP_BASE, WALK_HIP_SIGN, WALK_HIP_DELTA,
//                     WALK_FEM_PLANT, FEMUR_LIFT_SIGN, femDelta, seq[p][0],
//                     seq[p][1]);
//       moveToTarget(t, speedMs);
//     }
//     if (walkStop)
//       break;
//     walkStepsDone++;
//     Serial.printf("[WALK] Step %d/%d done\n", walkStepsDone,
//                   (steps == 0) ? -1 : steps);
//   }
//   moveToPose(1, 500); // Return to STANDING pose
//   walkRunning = false;
//   Serial.printf("[WALK] Complete. Steps=%d Variant=%s\n", walkStepsDone,
//                 variant == 0 ? "Basic" : "Enhanced");
// }

void walkForward(int steps, int speedMs, int variant) {
  syncGaitBase(); // gait follows the STANDING pose
  walkRunning = true;
  walkStop = false;
  walkStepsDone = 0;
  int totalCycles = (steps == 0) ? INT_MAX : steps;

  int femDelta[4];
  for (int l = 0; l < 4; l++)
    femDelta[l] = WALK_FEMUR_DELTA + WALK_FEMUR_TRIM[l];

  // {swingMask, liftMask} for the 6 phases
  const uint8_t seq[6][2] = {{0, M_FL | M_BR},           // 1: FL&BR femurs lift
                             {M_FL | M_BR, M_FL | M_BR}, // 2: swing FL&BR hips
                             {M_FL | M_BR, 0}, // 3: plant FL&BR femurs
                             {0, M_FR | M_BL}, // 4: FL&BR hips back, FR&BL lift
                             {M_FR | M_BL, M_FR | M_BL}, // 5: swing FR&BL hips
                             {M_FR | M_BL, 0}}; // 6: plant FR&BL femurs

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
// TURN GAIT (uses TURN_HIP_DELTA and TURN_FEMUR_DELTA)
// =============================================================================
void turnGait(int steps, int speedMs, bool rightTurn) {
  syncGaitBase(); // gait follows the STANDING pose
  turnRunning = true;
  turnStop = false;
  turnStepsDone = 0;
  strncpy(turnDirection, rightTurn ? "right" : "left", sizeof(turnDirection));

  const int dir = rightTurn ? -1 : +1;
  const int turnHipSign[4] = {dir, dir, dir, dir};
  const int femDelta[4] = {TURN_FEMUR_DELTA, TURN_FEMUR_DELTA, TURN_FEMUR_DELTA,
                           TURN_FEMUR_DELTA};

  const uint8_t seq[5][2] = {{0, M_FL | M_BR},                         // A
                             {M_FL | M_BR, M_FL | M_BR},               // B
                             {M_FL | M_BR, M_FR | M_BL},               // C
                             {M_FL | M_FR | M_BL | M_BR, M_FR | M_BL}, // D
                             {M_FR | M_BL, M_FL | M_BR}};              // E
  const char *names[5] = {"A: FL+BR lift", "B: FL+BR swing",
                          "C: FL+BR plant / FR+BL lift", "D: FR+BL swing",
                          "E: FR+BL plant / FL+BR lift"};

  for (int i = 0; i < steps && !turnStop; i++) {
    for (int p = 0; p < 5 && !turnStop; p++) {
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

void sweepTest() {
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

// =============================================================================
// CRAWL GAIT — 1-leg-at-a-time wave gait
// Uses CRAWL_HIP_DELTA and CRAWL_FEMUR_DELTA (tunable via web or here).
// All moves are offsets from the STANDING pose (poseAngles[1]).
// Sequence per cycle (8 sub-steps):
//   1. FL femur+DELTA  + ALL hips shift (FL-,FR-,BL+,BR-)
//   2. FL femur-DELTA  (plant FL)
//   3. BR femur+DELTA  + BR hip+DELTA
//   4. BR femur-DELTA  (plant BR)
//   5. FR femur-DELTA  + FR hip+DELTA
//   6. FR femur+DELTA  (plant FR)
//   7. BL femur-DELTA  + BL hip-DELTA
//   8. BL femur+DELTA  (plant BL) -> back to STANDING
// =============================================================================
void crawlForward(int steps, int speedMs) {
  syncGaitBase(); // base angles follow the live STANDING pose
  crawlRunning = true;
  crawlStop = false;
  crawlStepsDone = 0;
  int totalCycles = (steps == 0) ? INT_MAX : steps;

  // Shorthand base (hip & femur from STANDING, tibia unchanged)
  const int *hb = WALK_HIP_BASE;  // [FL,FR,BL,BR]
  const int *fb = WALK_FEM_PLANT; // plant femur (standing femur + walk offset)
  const int *tb = GAIT_TIBIA;     // tibia (standing tibia + gait offset)

  const int H = CRAWL_HIP_DELTA;
  const int F = CRAWL_FEMUR_DELTA;

  // Femur lift signs: FL=+1, FR=-1, BL=-1, BR=+1
  // Hip swing direction used in crawl:
  //   Step 1 mass-shift: FL-, FR-, BL+, BR-
  //   Step 3 BR swing  : BR+
  //   Step 5 FR swing  : FR+
  //   Step 7 BL swing  : BL-

  Serial.printf("[CRAWL] Starting 1-leg wave gait: %d cycles @ %dms/step\n",
                steps, speedMs);

  for (int cycle = 0; cycle < totalCycles && !crawlStop; cycle++) {

    // -- Step 1: FL lift + all hips shift (FL-, FR-, BL+, BR-) simultaneously
    // --
    strncpy(crawlPhaseStr, "1: FL Lift + Hip Shift", sizeof(crawlPhaseStr));
    {
      int t[4][3] = {
          {constrain(hb[0] - H, 0, 180), constrain(fb[0] + F, 0, 180),
           tb[0]},                                      // FL lifted, hip fwd
          {constrain(hb[1] - H, 0, 180), fb[1], tb[1]}, // FR hip shifted
          {constrain(hb[2] + H, 0, 180), fb[2], tb[2]}, // BL hip shifted
          {constrain(hb[3] - H, 0, 180), fb[3], tb[3]}  // BR hip shifted
      };
      moveToTarget(t, speedMs);
    }
    if (crawlStop)
      break;

    // -- Step 2: FL plant (femur back down, hips stay) --
    strncpy(crawlPhaseStr, "2: FL Plant", sizeof(crawlPhaseStr));
    {
      int t[4][3] = {{constrain(hb[0] - H, 0, 180), fb[0], tb[0]}, // FL planted
                     {constrain(hb[1] - H, 0, 180), fb[1], tb[1]},
                     {constrain(hb[2] + H, 0, 180), fb[2], tb[2]},
                     {constrain(hb[3] - H, 0, 180), fb[3], tb[3]}};
      moveToTarget(t, speedMs);
    }
    if (crawlStop)
      break;

    // -- Step 3: BR lift + BR hip shift back toward neutral (+H from step-2
    // pos) --
    strncpy(crawlPhaseStr, "3: BR Lift + Hip Shift", sizeof(crawlPhaseStr));
    {
      int t[4][3] = {
          {constrain(hb[0] - H, 0, 180), fb[0], tb[0]},
          {constrain(hb[1] - H, 0, 180), fb[1], tb[1]},
          {constrain(hb[2] + H, 0, 180), fb[2], tb[2]},
          {constrain(hb[3] - H + H, 0, 180), constrain(fb[3] + F, 0, 180),
           tb[3]} // BR lifted, hip forward
      };
      moveToTarget(t, speedMs);
    }
    if (crawlStop)
      break;

    // -- Step 4: BR plant --
    strncpy(crawlPhaseStr, "4: BR Plant", sizeof(crawlPhaseStr));
    {
      int t[4][3] = {
          {constrain(hb[0] - H, 0, 180), fb[0], tb[0]},
          {constrain(hb[1] - H, 0, 180), fb[1], tb[1]},
          {constrain(hb[2] + H, 0, 180), fb[2], tb[2]},
          {hb[3], fb[3], tb[3]} // BR back to base
      };
      moveToTarget(t, speedMs);
    }
    if (crawlStop)
      break;

    // -- Step 5: FR lift + FR hip shift --
    strncpy(crawlPhaseStr, "5: FR Lift + Hip Shift", sizeof(crawlPhaseStr));
    {
      int t[4][3] = {{constrain(hb[0] - H, 0, 180), fb[0], tb[0]},
                     {constrain(hb[1] - H + H, 0, 180),
                      constrain(fb[1] - F, 0, 180),
                      tb[1]}, // FR lifted, hip forward
                     {constrain(hb[2] + H, 0, 180), fb[2], tb[2]},
                     {hb[3], fb[3], tb[3]}};
      moveToTarget(t, speedMs);
    }
    if (crawlStop)
      break;

    // -- Step 6: FR plant --
    strncpy(crawlPhaseStr, "6: FR Plant", sizeof(crawlPhaseStr));
    {
      int t[4][3] = {{constrain(hb[0] - H, 0, 180), fb[0], tb[0]},
                     {hb[1], fb[1], tb[1]}, // FR back to base
                     {constrain(hb[2] + H, 0, 180), fb[2], tb[2]},
                     {hb[3], fb[3], tb[3]}};
      moveToTarget(t, speedMs);
    }
    if (crawlStop)
      break;

    // -- Step 7: BL lift + BL hip shift --
    strncpy(crawlPhaseStr, "7: BL Lift + Hip Shift", sizeof(crawlPhaseStr));
    {
      int t[4][3] = {{constrain(hb[0] - H, 0, 180), fb[0], tb[0]},
                     {hb[1], fb[1], tb[1]},
                     {constrain(hb[2] + H - H, 0, 180),
                      constrain(fb[2] - F, 0, 180),
                      tb[2]}, // BL lifted, hip forward
                     {hb[3], fb[3], tb[3]}};
      moveToTarget(t, speedMs);
    }
    if (crawlStop)
      break;

    // -- Step 8: BL plant -> cycle end, all hips back to base (STANDING) --
    strncpy(crawlPhaseStr, "8: BL Plant", sizeof(crawlPhaseStr));
    {
      int t[4][3] = {{hb[0], fb[0], tb[0]}, // FL back to base
                     {hb[1], fb[1], tb[1]},
                     {hb[2], fb[2], tb[2]}, // BL back to base
                     {hb[3], fb[3], tb[3]}};
      moveToTarget(t, speedMs);
    }
    if (crawlStop)
      break;

    crawlStepsDone++;
    Serial.printf("[CRAWL] Cycle %d/%d completed.\n", crawlStepsDone,
                  (steps == 0) ? -1 : steps);
  }

  strncpy(crawlPhaseStr, "Standing", sizeof(crawlPhaseStr));
  moveToPose(1, 400); // Return cleanly to STANDING pose
  crawlRunning = false;
  Serial.printf("[CRAWL] Gait finished. Total cycles = %d\n", crawlStepsDone);
}

// Runs one queued job (called from loop only)
void runPendingJob() {
  if (pendingJob == JOB_NONE)
    return;
  Job j = pendingJob;
  pendingJob = JOB_NONE;
  jobRunning = true;
  switch (j) {
  case JOB_HOME:
    setAllServosHome();
    break;
  case JOB_LEGHOME:
    setLegHome(jobArg);
    Serial.printf("[WEB] Leg %d -> HOME done.\n", jobArg);
    break;
  case JOB_POSE:
    moveToPose(jobArg, jobSpeed);
    Serial.printf("[POSE] %d done.\n", jobArg);
    break;
  case JOB_WALK:
    walkForward(jobSteps, jobSpeed, jobVariant);
    break;
  case JOB_TURN:
    turnGait(jobSteps, jobSpeed, jobRight);
    break;
  case JOB_SWEEP:
    sweepTest();
    break;
  case JOB_CRAWL:
    crawlForward(jobSteps, jobSpeed);
    break;
  default:
    break;
  }
  jobRunning = false;
}

// =============================================================================
// HTTP HANDLERS (robot)
// =============================================================================
void hRoot() { server.send_P(200, "text/html", INDEX_HTML); }

void hSet() {
  if (!server.hasArg("ch") || !server.hasArg("angle")) {
    server.send(400, "application/json", "{\"error\":\"missing ch or angle\"}");
    return;
  }
  if (rejectIfBusy())
    return;
  int ch = server.arg("ch").toInt(), angle = server.arg("angle").toInt();
  if (ch < 0 || ch > 15 || angle < 0 || angle > 180) {
    server.send(400, "application/json", "{\"error\":\"out of range\"}");
    return;
  }
  setServo((uint8_t)ch, angle);
  addCORS();
  server.send(200, "application/json",
              "{\"ok\":true,\"ch\":" + String(ch) +
                  ",\"angle\":" + String(angle) + "}");
}

void hState() {
  addCORS();
  server.send(200, "application/json", stateJSON());
}

// Compare these values before and after a dropout. An HTTP timeout alone is
// not evidence that the phone disconnected from the access point.
void hHealth() {
  String json = "{\"uptime_ms\":" + String(millis());
  json += ",\"reset_reason\":" + String((int)bootResetReason);
  json += ",\"brownout_reset\":";
  json += (bootResetReason == ESP_RST_BROWNOUT ? "true" : "false");
  json += ",\"free_heap_bytes\":" + String(ESP.getFreeHeap());
  json += ",\"ap_clients\":" + String(WiFi.softAPgetStationNum());
  json += ",\"ap_connects\":" + String(apConnectCount);
  json += ",\"ap_disconnects\":" + String(apDisconnectCount);
  json += ",\"ap_stops\":" + String(apStopCount);
  json += "}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void hHome() {
  if (rejectIfBusy())
    return;
  addCORS();
  server.send(200, "application/json", poseJSON(0));
  pendingJob = JOB_HOME;
}

void hLegHome() {
  if (!server.hasArg("leg")) {
    server.send(400, "application/json", "{\"error\":\"missing leg\"}");
    return;
  }
  int leg = server.arg("leg").toInt();
  if (leg < 0 || leg > 3) {
    server.send(400, "application/json", "{\"error\":\"invalid leg (0-3)\"}");
    return;
  }
  if (rejectIfBusy())
    return;
  addCORS();
  server.send(200, "application/json",
              "{\"a\":[" + String(poseAngles[0][leg][0]) + "," +
                  String(poseAngles[0][leg][1]) + "," +
                  String(poseAngles[0][leg][2]) + "]}");
  jobArg = leg;
  pendingJob = JOB_LEGHOME;
}

void hIK() {
  if (!server.hasArg("leg") || !server.hasArg("x") || !server.hasArg("y") ||
      !server.hasArg("z")) {
    server.send(400, "application/json", "{\"error\":\"missing args\"}");
    return;
  }
  int leg = server.arg("leg").toInt();
  if (leg < 0 || leg > 3) {
    server.send(400, "application/json", "{\"error\":\"invalid leg\"}");
    return;
  }
  if (rejectIfBusy())
    return;
  int hip, femur, tibia;
  addCORS();
  if (!computeIK(server.arg("x").toFloat(), server.arg("y").toFloat(),
                 server.arg("z").toFloat(), legMirror[leg], &hip, &femur,
                 &tibia)) {
    server.send(200, "application/json",
                "{\"ok\":false,\"error\":\"unreachable\"}");
    return;
  }
  setServo(legChannels[leg][0], hip);
  setServo(legChannels[leg][1], femur);
  setServo(legChannels[leg][2], tibia);
  server.send(200, "application/json",
              "{\"ok\":true,\"hip\":" + String(hip) + ",\"femur\":" +
                  String(femur) + ",\"tibia\":" + String(tibia) + "}");
  Serial.printf("[IK] Leg %d -> H:%d F:%d T:%d\n", leg, hip, femur, tibia);
}

// GET /pose?id=N&speed=ms — response first, move in loop()
void hPose() {
  if (!server.hasArg("id")) {
    server.send(400, "application/json", "{\"error\":\"missing id\"}");
    return;
  }
  int id = server.arg("id").toInt();
  if (id < 0 || id >= POSE_COUNT) {
    server.send(400, "application/json",
                "{\"error\":\"invalid pose id (0-12)\"}");
    return;
  }
  if (rejectIfBusy())
    return;
  jobSpeed = server.hasArg("speed")
                 ? constrain(server.arg("speed").toInt(), 0, 5000)
                 : 0;
  jobArg = id;
  addCORS();
  server.send(200, "application/json", poseJSON(id));
  pendingJob = JOB_POSE;
}

// GET /savepose?id=N&a00=..  — store angles into a preset (RAM)
void hSavePose() {
  if (!server.hasArg("id")) {
    server.send(400, "application/json", "{\"error\":\"missing id\"}");
    return;
  }
  int id = server.arg("id").toInt();
  if (id < 0 || id >= POSE_COUNT) {
    server.send(400, "application/json", "{\"error\":\"invalid pose id\"}");
    return;
  }
  for (int leg = 0; leg < 4; leg++)
    for (int joint = 0; joint < 3; joint++) {
      char argName[4];
      snprintf(argName, sizeof(argName), "a%d%d", leg, joint);
      if (server.hasArg(argName))
        poseAngles[id][leg][joint] =
            constrain(server.arg(argName).toInt(), 0, 180);
    }
  Serial.printf("[POSE] Updated Pose %d in RAM.\n", id);
  addCORS();
  server.send(200, "application/json", "{\"ok\":true}");
}

// GET /walk?steps=N&speed=X&variant=0|1   |   /walk?stop=1
void hWalk() {
  addCORS();
  if (server.hasArg("stop")) {
    walkStop = true;
    server.send(200, "application/json", "{\"ok\":true,\"msg\":\"stopping\"}");
    return;
  }
  if (robotBusy()) {
    server.send(200, "application/json", "{\"ok\":false,\"msg\":\"busy\"}");
    return;
  }
  int steps = server.hasArg("steps") ? server.arg("steps").toInt() : 4;
  jobSpeed = server.hasArg("speed")
                 ? constrain(server.arg("speed").toInt(), 20, 3000)
                 : 600;
  jobVariant = server.hasArg("variant")
                   ? constrain(server.arg("variant").toInt(), 0, 1)
                   : 0;
  jobSteps = walkStepsTarget = (steps < 0) ? 0 : steps;
  walkStepsDone = 0;
  walkStop = false;
  walkRunning = true;
  Serial.printf("[WALK] steps=%d speed=%dms variant=%d\n", jobSteps, jobSpeed,
                jobVariant);
  server.send(200, "application/json",
              "{\"ok\":true,\"msg\":\"walk started\"}");
  pendingJob = JOB_WALK;
}

void hWalkStatus() {
  addCORS();
  server.send(200, "application/json",
              String("{\"running\":") + (walkRunning ? "true" : "false") +
                  ",\"steps\":" + walkStepsDone +
                  ",\"target\":" + walkStepsTarget + "}");
}

void startTurn(bool right) {
  addCORS();
  if (server.hasArg("stop")) {
    turnStop = true;
    server.send(200, "application/json", "{\"ok\":true,\"msg\":\"stopping\"}");
    return;
  }
  if (robotBusy()) {
    server.send(200, "application/json", "{\"ok\":false,\"msg\":\"busy\"}");
    return;
  }
  int steps = server.hasArg("steps") ? server.arg("steps").toInt() : 4;
  jobSpeed = server.hasArg("speed")
                 ? constrain(server.arg("speed").toInt(), 20, 3000)
                 : 500;
  jobSteps = turnStepsTarget = (steps < 1) ? 1 : steps;
  jobRight = right;
  turnStepsDone = 0;
  turnStop = false;
  turnRunning = true;
  strncpy(turnPhaseStr, "starting", sizeof(turnPhaseStr));
  Serial.printf("[TURN-%c] steps=%d speed=%dms\n", right ? 'R' : 'L', jobSteps,
                jobSpeed);
  server.send(200, "application/json",
              right ? "{\"ok\":true,\"msg\":\"turn right started\"}"
                    : "{\"ok\":true,\"msg\":\"turn left started\"}");
  pendingJob = JOB_TURN;
}
void hTurnLeft() { startTurn(false); }
void hTurnRight() { startTurn(true); }

void hTurnStatus() {
  addCORS();
  server.send(200, "application/json",
              String("{\"running\":") + (turnRunning ? "true" : "false") +
                  ",\"steps\":" + turnStepsDone + ",\"target\":" +
                  turnStepsTarget + ",\"phase\":\"" + turnPhaseStr + "\"}");
}

void hSweep() {
  addCORS();
  if (robotBusy()) {
    server.send(200, "application/json", "{\"ok\":false,\"msg\":\"busy\"}");
    return;
  }
  server.send(200, "application/json",
              "{\"ok\":true,\"msg\":\"sweep started\"}");
  pendingJob = JOB_SWEEP;
}

// GET /crawl?steps=N&speed=X   |   GET /crawl?stop=1
void hCrawl() {
  addCORS();
  if (server.hasArg("stop")) {
    crawlStop = true;
    server.send(200, "application/json",
                "{\"ok\":true,\"msg\":\"stopping crawl\"}");
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
  pendingJob = JOB_CRAWL;
}

// GET /crawlstatus
void hCrawlStatus() {
  addCORS();
  server.send(200, "application/json",
              String("{\"running\":") + (crawlRunning ? "true" : "false") +
                  ",\"steps\":" + crawlStepsDone + ",\"target\":" +
                  crawlStepsTarget + ",\"phase\":\"" + crawlPhaseStr + "\"}");
}

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 3000)
    delay(10);

  bootResetReason = esp_reset_reason();
  Serial.printf("[BOOT] Reset reason: %d%s\n", (int)bootResetReason,
                bootResetReason == ESP_RST_BROWNOUT ? " (BROWNOUT)" : "");

  Serial.println("\n=============================================");
  Serial.println("  FusionForce RUNNER4 v5.2 — ESP32-S3");
  Serial.println("  Robot (/)  +  Colour (/color)  +  Line (/line)");
  Serial.println("=============================================");

  // One shared I2C bus (GPIO 11/12 — GPIO 8/9 are line-array ADC inputs)
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_HZ);
  Wire.setTimeOut(50);
  i2cScan();

  Serial.println("Initialising PCA9685...");
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  Wire.setClock(I2C_HZ);
  delay(10);

  Serial.println("Moving all servos -> INITIAL pose...");
  setAllServosHome();
  delay(1500);

  // WiFi AP
  WiFi.mode(WIFI_AP);
  WiFi.onEvent(onWiFiEvent);
  bool apStarted = WiFi.softAP(AP_SSID, AP_PASS);
  WiFi.setSleep(false);
  Serial.printf("[WIFI] AP %s: '%s' -> http://%s\n",
                apStarted ? "started" : "FAILED", AP_SSID,
                WiFi.softAPIP().toString().c_str());

  // Robot routes (root)
  server.on("/", hRoot);
  server.on("/set", hSet);
  server.on("/state", hState);
  server.on("/health", hHealth);
  server.on("/home", hHome);
  server.on("/leghome", hLegHome);
  server.on("/ik", hIK);
  server.on("/pose", hPose);
  server.on("/savepose", hSavePose);
  server.on("/walk", hWalk);
  server.on("/walkstatus", hWalkStatus);
  server.on("/turnleft", hTurnLeft);
  server.on("/turnright", hTurnRight);
  server.on("/turnstatus", hTurnStatus);
  server.on("/sweep", hSweep);
  server.on("/crawl", hCrawl);
  server.on("/crawlstatus", hCrawlStatus);

  // Colour sensor module (separate root: /color, /color/...)
  colorBegin(server, AP_SSID, I2C_SDA, I2C_SCL);

  // Line array module (separate root: /line, /line/...)
  lineBegin(server, AP_SSID);

  server.begin();
  serverStarted = true;
  Serial.println(
      "HTTP server running.  Robot: /   Colour: /color   Line: /line\n");
}

// =============================================================================
// LOOP
// =============================================================================
void loop() {
  server.handleClient();
  colorPoll();
  linePoll();
  runPendingJob();
}
