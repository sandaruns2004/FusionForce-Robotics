// =============================================================================
// FusionForce RUNNER4 — 12 DOF Quadruped Robot
// MAIN ROOT : PCA9685 servo control + pose presets + FK/IK + walk/turn gaits
// MODULE    : TCS34725 colour sensor (color_sensor.cpp/.h/color_page.h)
// BOARD     : ESP32-S3
// VERSION   : 4.0
// =============================================================================
//
// WEB ROUTES (one WiFi AP, one web server)
//   http://192.168.4.1/        -> robot control page (main root, unchanged UI)
//   http://192.168.4.1/color   -> colour sensor lab (separate root / module)
//   /color/data /color/config /color/set /color/cmd /color/wifi /color/export
//
// SKETCH FOLDER (all 4 files in ONE folder named RUNNER4_Robot_Colour):
//   RUNNER4_Robot_Colour.ino   <- this file (robot)
//   robot_page.h               <- robot web page
//   color_sensor.h / .cpp      <- colour sensor module
//   color_page.h               <- colour web page
//
// WIRING (one shared I2C bus, different addresses: PCA9685=0x40, TCS34725=0x29)
//   ESP32-S3 3.3V -> PCA9685 VCC  and  TCS34725 VIN
//   ESP32-S3 GND  -> PCA9685 GND  and  TCS34725 GND
//   GPIO 8 (SDA)  -> PCA9685 SDA  and  TCS34725 SDA
//   GPIO 9 (SCL)  -> PCA9685 SCL  and  TCS34725 SCL
//   GPIO 4        -> TCS34725 LED pin   (change COLOR_LED_PIN in color_sensor.h)
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
//   serviceBackground() = web server + colour sensor, so Stop buttons work and
//   the colour reading keeps updating while the robot walks.
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
#include <math.h>
#include <limits.h>

#include "robot_page.h"    // INDEX_HTML
#include "color_sensor.h"  // colour module API

struct Vec3f { float x, y, z; };

// ── WiFi AP ──────────────────────────────────────────────────────────────────
const char *AP_SSID = "RUNNER4-Config";
const char *AP_PASS = "runner4robot";

// ── I2C (shared by PCA9685 and TCS34725) ─────────────────────────────────────
#define I2C_SDA 8
#define I2C_SCL 9
#define I2C_HZ  400000

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
    {{45, 80, 160}, {94, 80, 0}, {94, 56, 10}, {45, 92, 160}},
    // 1 STANDING
    {{45, 25, 115}, {94, 135, 45}, {94, 111, 55}, {45, 37, 115}},
    // 2 LOW CROUCH
    {{45, 110, 140}, {94, 110, 40}, {94, 110, 140}, {45, 110, 140}},
    // 3 LOW STAND
    {{45, 50, 130}, {94, 110, 30}, {94, 86, 40}, {45, 62, 130}},
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

int servoAngles[16] = {0};

WebServer server(80);
bool serverStarted = false;

// =============================================================================
// JOB SYSTEM — handlers queue work, loop() executes it
// =============================================================================
enum Job : uint8_t { JOB_NONE, JOB_HOME, JOB_LEGHOME, JOB_POSE, JOB_WALK, JOB_TURN, JOB_SWEEP };
volatile Job pendingJob = JOB_NONE;
bool jobRunning = false;
int  jobArg = 0, jobSpeed = 0, jobSteps = 0, jobVariant = 0;
bool jobRight = false;

inline bool robotBusy() { return jobRunning || pendingJob != JOB_NONE; }

// Walk / turn state
volatile bool walkRunning = false, walkStop = false;
int walkStepsTarget = 0, walkStepsDone = 0;
volatile bool turnRunning = false, turnStop = false;
int turnStepsTarget = 0, turnStepsDone = 0;
char turnDirection[6] = "none";
char turnPhaseStr[32] = "";

// =============================================================================
// BACKGROUND SERVICE — called from every wait inside motion code
// =============================================================================
void serviceBackground() {
  if (serverStarted) server.handleClient();
  colorPoll();
}

// Wait ms while keeping WiFi + colour sensor alive
void waitMs(uint32_t ms) {
  uint32_t t0 = millis();
  do {
    serviceBackground();
    if (millis() - t0 >= ms) break;
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
      waitMs(10);  // stagger to reduce current spike
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
      int diff = abs(targetAngles[leg][joint] - servoAngles[legChannels[leg][joint]]);
      if (diff > maxDiff) maxDiff = diff;
    }
  if (maxDiff == 0) return;

  int startAngles[16];
  memcpy(startAngles, servoAngles, sizeof(servoAngles));
  unsigned long moveStart = millis();
  for (int step = 1; step <= maxDiff; step++) {
    float t = (float)step / (float)maxDiff;
    for (int leg = 0; leg < 4; leg++)
      for (int joint = 0; joint < 3; joint++) {
        uint8_t ch = legChannels[leg][joint];
        setServo(ch, startAngles[ch] + (int)roundf(t * (targetAngles[leg][joint] - startAngles[ch])));
      }
    unsigned long stepTarget = moveStart + (unsigned long)((long)speedMs * step / maxDiff);
    unsigned long now = millis();
    if (stepTarget > now) waitMs(stepTarget - now); else serviceBackground();
  }
  Serial.printf("[POSE] Transition done. Steps=%d, duration=%lums\n", maxDiff, millis() - moveStart);
}

void moveToPose(int id, int speedMs) {
  if (id < 0 || id >= POSE_COUNT) return;
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

bool computeIK(float px, float py, float pz, int mirror, int *hipOut, int *femurOut, int *tibiaOut) {
  py *= (float)mirror;
  const float a1 = atan2f(py, px);
  const float r = sqrtf(px * px + py * py) - L1_MM;
  const float d = sqrtf(r * r + pz * pz);
  if (d > L2_MM + L3_MM - 0.5f || d < fabsf(L2_MM - L3_MM) + 0.5f) return false;
  float cosA3 = constrain((d * d - L2_MM * L2_MM - L3_MM * L3_MM) / (2.0f * L2_MM * L3_MM), -1.0f, 1.0f);
  const float a3 = acosf(cosA3);
  const float a2 = atan2f(pz, r) - atan2f(L3_MM * sinf(a3), L2_MM + L3_MM * cosf(a3));
  int hip = (int)roundf(a1 * RAD_TO_DEG) + 90;
  int femur = (int)roundf(a2 * RAD_TO_DEG) + 90;
  int tibia = (int)roundf(a3 * RAD_TO_DEG) + 90;
  if (hip < 0 || hip > 180 || femur < 0 || femur > 180 || tibia < 0 || tibia > 180) return false;
  *hipOut = hip; *femurOut = femur; *tibiaOut = tibia;
  return true;
}

void i2cScan() {
  Serial.println("-------------------------------------------------");
  Serial.println("I2C Bus Scan (expect 0x29 colour sensor, 0x40 PCA9685):");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) { Serial.printf("  [OK] 0x%02X\n", addr); found++; }
  }
  if (found == 0) Serial.println("  [!!] No I2C devices found!");
  Serial.println("-------------------------------------------------");
}

// =============================================================================
// JSON HELPERS
// =============================================================================
String stateJSON() {
  String j = "{\"a\":[";
  for (int i = 0; i < 16; i++) { j += servoAngles[i]; if (i < 15) j += ','; }
  return j + "]}";
}

// Angles the robot WILL have after pose id (so the browser updates immediately)
String poseJSON(int id) {
  int t[16];
  memcpy(t, servoAngles, sizeof(t));
  for (int leg = 0; leg < 4; leg++)
    for (int joint = 0; joint < 3; joint++) t[legChannels[leg][joint]] = poseAngles[id][leg][joint];
  String j = "{\"a\":[";
  for (int i = 0; i < 16; i++) { j += t[i]; if (i < 15) j += ','; }
  return j + "]}";
}

void addCORS() { server.sendHeader("Access-Control-Allow-Origin", "*"); }

bool rejectIfBusy() {
  if (!robotBusy()) return false;
  addCORS();
  server.send(409, "application/json", "{\"ok\":false,\"msg\":\"busy\"}");
  return true;
}

// =============================================================================
// WALK / TURN GAITS (your sequences, unchanged)
// =============================================================================
void walkForward(int steps, int speedMs, int variant) {
  walkRunning = true;
  walkStop = false;
  walkStepsDone = 0;
  int totalCycles = (steps == 0) ? INT_MAX : steps;

  for (int i = 0; i < totalCycles && !walkStop; i++) {
    // 1: FR&BL hips to 94 while FL&BR femurs lift
    const int t1[4][3] = {{45, 45, 115}, {94, 130, 45}, {94, 116, 55}, {45, 57, 115}};
    moveToTarget(t1, speedMs); if (walkStop) break;
    // 2: swing FL&BR hips
    const int t2[4][3] = {{15, 45, 115}, {94, 130, 45}, {94, 116, 55}, {75, 57, 115}};
    moveToTarget(t2, speedMs); if (walkStop) break;
    // 3: plant FL&BR femurs
    const int t3[4][3] = {{15, 20, 115}, {94, 130, 45}, {94, 116, 55}, {75, 32, 115}};
    moveToTarget(t3, speedMs); if (walkStop) break;
    // 4: push FL&BR hips to 45 while FR&BL femurs lift
    const int t4[4][3] = {{45, 20, 115}, {94, 115, 45}, {94, 91, 55}, {45, 32, 115}};
    moveToTarget(t4, speedMs); if (walkStop) break;
    // 5: swing FR&BL hips
    const int t5[4][3] = {{45, 20, 115}, {124, 115, 45}, {64, 91, 55}, {45, 32, 115}};
    moveToTarget(t5, speedMs); if (walkStop) break;
    // 6: plant FR&BL femurs
    const int t6[4][3] = {{45, 20, 115}, {124, 130, 45}, {64, 116, 55}, {45, 32, 115}};
    moveToTarget(t6, speedMs); if (walkStop) break;

    walkStepsDone++;
    Serial.printf("[WALK] Step %d/%d done\n", walkStepsDone, (steps == 0) ? -1 : steps);
  }
  moveToPose(0, 500);
  walkRunning = false;
  Serial.printf("[WALK] Complete. Steps=%d Variant=%s\n", walkStepsDone, variant == 0 ? "Basic" : "Enhanced");
}

void turnGait(int steps, int speedMs, bool rightTurn) {
  turnRunning = true;
  turnStop = false;
  turnStepsDone = 0;
  strncpy(turnDirection, rightTurn ? "right" : "left", sizeof(turnDirection));

  const int hipOffset = rightTurn ? -20 : 20;
  const int FL_HIP_S = 45, FL_FEM_S = 25, FL_TIB = 115;
  const int FR_HIP_S = 94, FR_FEM_S = 135, FR_TIB = 45;
  const int BL_HIP_S = 94, BL_FEM_S = 111, BL_TIB = 55;
  const int BR_HIP_S = 45, BR_FEM_S = 37, BR_TIB = 115;
  const int FL_FEM_UP = 45, BR_FEM_UP = 57, FR_FEM_UP = 115, BL_FEM_UP = 91;
  const int FL_HIP_SW = FL_HIP_S + hipOffset, BR_HIP_SW = BR_HIP_S + hipOffset;
  const int FR_HIP_SW = FR_HIP_S + hipOffset, BL_HIP_SW = BL_HIP_S + hipOffset;

  for (int i = 0; i < steps && !turnStop; i++) {
    strncpy(turnPhaseStr, "A: FL+BR lift", sizeof(turnPhaseStr));
    { const int t[4][3] = {{FL_HIP_S, FL_FEM_UP, FL_TIB}, {FR_HIP_S, FR_FEM_S, FR_TIB}, {BL_HIP_S, BL_FEM_S, BL_TIB}, {BR_HIP_S, BR_FEM_UP, BR_TIB}};
      moveToTarget(t, speedMs); }
    if (turnStop) break;

    strncpy(turnPhaseStr, "B: FL+BR swing", sizeof(turnPhaseStr));
    { const int t[4][3] = {{FL_HIP_SW, FL_FEM_UP, FL_TIB}, {FR_HIP_S, FR_FEM_S, FR_TIB}, {BL_HIP_S, BL_FEM_S, BL_TIB}, {BR_HIP_SW, BR_FEM_UP, BR_TIB}};
      moveToTarget(t, speedMs); }
    if (turnStop) break;

    strncpy(turnPhaseStr, "C: FL+BR plant / FR+BL lift", sizeof(turnPhaseStr));
    { const int t[4][3] = {{FL_HIP_SW, FL_FEM_S, FL_TIB}, {FR_HIP_S, FR_FEM_UP, FR_TIB}, {BL_HIP_S, BL_FEM_UP, BL_TIB}, {BR_HIP_SW, BR_FEM_S, BR_TIB}};
      moveToTarget(t, speedMs); }
    if (turnStop) break;

    strncpy(turnPhaseStr, "D: FR+BL swing", sizeof(turnPhaseStr));
    { const int t[4][3] = {{FL_HIP_SW, FL_FEM_S, FL_TIB}, {FR_HIP_SW, FR_FEM_UP, FR_TIB}, {BL_HIP_SW, BL_FEM_UP, BL_TIB}, {BR_HIP_SW, BR_FEM_S, BR_TIB}};
      moveToTarget(t, speedMs); }
    if (turnStop) break;

    strncpy(turnPhaseStr, "E: FR+BL plant / FL+BR lift", sizeof(turnPhaseStr));
    { const int t[4][3] = {{FL_HIP_S, FL_FEM_UP, FL_TIB}, {FR_HIP_SW, FR_FEM_S, FR_TIB}, {BL_HIP_SW, BL_FEM_S, BL_TIB}, {BR_HIP_S, BR_FEM_UP, BR_TIB}};
      moveToTarget(t, speedMs); }
    if (turnStop) break;

    turnStepsDone++;
    Serial.printf("[TURN] %s step %d/%d done\n", rightTurn ? "Right" : "Left", turnStepsDone, steps);
  }
  strncpy(turnPhaseStr, "done", sizeof(turnPhaseStr));
  moveToPose(0, 500);
  turnRunning = false;
  Serial.printf("[TURN] %s complete. Steps=%d\n", rightTurn ? "Right" : "Left", turnStepsDone);
}

void sweepTest() {
  Serial.println("[SWEEP] BR FEMUR (CH8): 0 -> 90 -> 0");
  for (int a = 0; a <= 90; a++) { setServo(BR_FEMUR, a); waitMs(15); }
  waitMs(200);
  for (int a = 90; a >= 0; a--) { setServo(BR_FEMUR, a); waitMs(15); }
  waitMs(200);
  setLegHome(3);
  Serial.println("[SWEEP] Done.");
}

// Runs one queued job (called from loop only)
void runPendingJob() {
  if (pendingJob == JOB_NONE) return;
  Job j = pendingJob;
  pendingJob = JOB_NONE;
  jobRunning = true;
  switch (j) {
    case JOB_HOME:    setAllServosHome(); break;
    case JOB_LEGHOME: setLegHome(jobArg); Serial.printf("[WEB] Leg %d -> HOME done.\n", jobArg); break;
    case JOB_POSE:    moveToPose(jobArg, jobSpeed); Serial.printf("[POSE] %d done.\n", jobArg); break;
    case JOB_WALK:    walkForward(jobSteps, jobSpeed, jobVariant); break;
    case JOB_TURN:    turnGait(jobSteps, jobSpeed, jobRight); break;
    case JOB_SWEEP:   sweepTest(); break;
    default: break;
  }
  jobRunning = false;
}

// =============================================================================
// HTTP HANDLERS (robot)
// =============================================================================
void hRoot() { server.send_P(200, "text/html", INDEX_HTML); }

void hSet() {
  if (!server.hasArg("ch") || !server.hasArg("angle")) {
    server.send(400, "application/json", "{\"error\":\"missing ch or angle\"}"); return;
  }
  if (rejectIfBusy()) return;
  int ch = server.arg("ch").toInt(), angle = server.arg("angle").toInt();
  if (ch < 0 || ch > 15 || angle < 0 || angle > 180) {
    server.send(400, "application/json", "{\"error\":\"out of range\"}"); return;
  }
  setServo((uint8_t)ch, angle);
  addCORS();
  server.send(200, "application/json", "{\"ok\":true,\"ch\":" + String(ch) + ",\"angle\":" + String(angle) + "}");
}

void hState() { addCORS(); server.send(200, "application/json", stateJSON()); }

void hHome() {
  if (rejectIfBusy()) return;
  addCORS();
  server.send(200, "application/json", poseJSON(0));
  pendingJob = JOB_HOME;
}

void hLegHome() {
  if (!server.hasArg("leg")) { server.send(400, "application/json", "{\"error\":\"missing leg\"}"); return; }
  int leg = server.arg("leg").toInt();
  if (leg < 0 || leg > 3) { server.send(400, "application/json", "{\"error\":\"invalid leg (0-3)\"}"); return; }
  if (rejectIfBusy()) return;
  addCORS();
  server.send(200, "application/json", "{\"a\":[" + String(poseAngles[0][leg][0]) + "," + String(poseAngles[0][leg][1]) + "," + String(poseAngles[0][leg][2]) + "]}");
  jobArg = leg;
  pendingJob = JOB_LEGHOME;
}

void hIK() {
  if (!server.hasArg("leg") || !server.hasArg("x") || !server.hasArg("y") || !server.hasArg("z")) {
    server.send(400, "application/json", "{\"error\":\"missing args\"}"); return;
  }
  int leg = server.arg("leg").toInt();
  if (leg < 0 || leg > 3) { server.send(400, "application/json", "{\"error\":\"invalid leg\"}"); return; }
  if (rejectIfBusy()) return;
  int hip, femur, tibia;
  addCORS();
  if (!computeIK(server.arg("x").toFloat(), server.arg("y").toFloat(), server.arg("z").toFloat(), legMirror[leg], &hip, &femur, &tibia)) {
    server.send(200, "application/json", "{\"ok\":false,\"error\":\"unreachable\"}"); return;
  }
  setServo(legChannels[leg][0], hip);
  setServo(legChannels[leg][1], femur);
  setServo(legChannels[leg][2], tibia);
  server.send(200, "application/json", "{\"ok\":true,\"hip\":" + String(hip) + ",\"femur\":" + String(femur) + ",\"tibia\":" + String(tibia) + "}");
  Serial.printf("[IK] Leg %d -> H:%d F:%d T:%d\n", leg, hip, femur, tibia);
}

// GET /pose?id=N&speed=ms — response first, move in loop()
void hPose() {
  if (!server.hasArg("id")) { server.send(400, "application/json", "{\"error\":\"missing id\"}"); return; }
  int id = server.arg("id").toInt();
  if (id < 0 || id >= POSE_COUNT) { server.send(400, "application/json", "{\"error\":\"invalid pose id (0-12)\"}"); return; }
  if (rejectIfBusy()) return;
  jobSpeed = server.hasArg("speed") ? constrain(server.arg("speed").toInt(), 0, 5000) : 0;
  jobArg = id;
  addCORS();
  server.send(200, "application/json", poseJSON(id));
  pendingJob = JOB_POSE;
}

// GET /savepose?id=N&a00=..  — store angles into a preset (RAM)
void hSavePose() {
  if (!server.hasArg("id")) { server.send(400, "application/json", "{\"error\":\"missing id\"}"); return; }
  int id = server.arg("id").toInt();
  if (id < 0 || id >= POSE_COUNT) { server.send(400, "application/json", "{\"error\":\"invalid pose id\"}"); return; }
  for (int leg = 0; leg < 4; leg++)
    for (int joint = 0; joint < 3; joint++) {
      char argName[4];
      snprintf(argName, sizeof(argName), "a%d%d", leg, joint);
      if (server.hasArg(argName)) poseAngles[id][leg][joint] = constrain(server.arg(argName).toInt(), 0, 180);
    }
  Serial.printf("[POSE] Updated Pose %d in RAM.\n", id);
  addCORS();
  server.send(200, "application/json", "{\"ok\":true}");
}

// GET /walk?steps=N&speed=X&variant=0|1   |   /walk?stop=1
void hWalk() {
  addCORS();
  if (server.hasArg("stop")) { walkStop = true; server.send(200, "application/json", "{\"ok\":true,\"msg\":\"stopping\"}"); return; }
  if (robotBusy()) { server.send(200, "application/json", "{\"ok\":false,\"msg\":\"busy\"}"); return; }
  int steps = server.hasArg("steps") ? server.arg("steps").toInt() : 4;
  jobSpeed = server.hasArg("speed") ? constrain(server.arg("speed").toInt(), 20, 3000) : 600;
  jobVariant = server.hasArg("variant") ? constrain(server.arg("variant").toInt(), 0, 1) : 0;
  jobSteps = walkStepsTarget = (steps < 0) ? 0 : steps;
  walkStepsDone = 0; walkStop = false; walkRunning = true;
  Serial.printf("[WALK] steps=%d speed=%dms variant=%d\n", jobSteps, jobSpeed, jobVariant);
  server.send(200, "application/json", "{\"ok\":true,\"msg\":\"walk started\"}");
  pendingJob = JOB_WALK;
}

void hWalkStatus() {
  addCORS();
  server.send(200, "application/json", String("{\"running\":") + (walkRunning ? "true" : "false") +
              ",\"steps\":" + walkStepsDone + ",\"target\":" + walkStepsTarget + "}");
}

void startTurn(bool right) {
  addCORS();
  if (server.hasArg("stop")) { turnStop = true; server.send(200, "application/json", "{\"ok\":true,\"msg\":\"stopping\"}"); return; }
  if (robotBusy()) { server.send(200, "application/json", "{\"ok\":false,\"msg\":\"busy\"}"); return; }
  int steps = server.hasArg("steps") ? server.arg("steps").toInt() : 4;
  jobSpeed = server.hasArg("speed") ? constrain(server.arg("speed").toInt(), 20, 3000) : 500;
  jobSteps = turnStepsTarget = (steps < 1) ? 1 : steps;
  jobRight = right;
  turnStepsDone = 0; turnStop = false; turnRunning = true;
  strncpy(turnPhaseStr, "starting", sizeof(turnPhaseStr));
  Serial.printf("[TURN-%c] steps=%d speed=%dms\n", right ? 'R' : 'L', jobSteps, jobSpeed);
  server.send(200, "application/json", right ? "{\"ok\":true,\"msg\":\"turn right started\"}" : "{\"ok\":true,\"msg\":\"turn left started\"}");
  pendingJob = JOB_TURN;
}
void hTurnLeft()  { startTurn(false); }
void hTurnRight() { startTurn(true); }

void hTurnStatus() {
  addCORS();
  server.send(200, "application/json", String("{\"running\":") + (turnRunning ? "true" : "false") +
              ",\"steps\":" + turnStepsDone + ",\"target\":" + turnStepsTarget + ",\"phase\":\"" + turnPhaseStr + "\"}");
}

void hSweep() {
  addCORS();
  if (robotBusy()) { server.send(200, "application/json", "{\"ok\":false,\"msg\":\"busy\"}"); return; }
  server.send(200, "application/json", "{\"ok\":true,\"msg\":\"sweep started\"}");
  pendingJob = JOB_SWEEP;
}

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 3000) delay(10);

  Serial.println("\n=============================================");
  Serial.println("  FusionForce RUNNER4 v4.0 — ESP32-S3");
  Serial.println("  Robot (root /)  +  Colour sensor (/color)");
  Serial.println("=============================================");

  // One shared I2C bus
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
  WiFi.softAP(AP_SSID, AP_PASS);
  WiFi.setSleep(false);
  Serial.printf("WiFi AP '%s' / '%s'  ->  http://%s\n", AP_SSID, AP_PASS, WiFi.softAPIP().toString().c_str());

  // Robot routes (root)
  server.on("/", hRoot);
  server.on("/set", hSet);
  server.on("/state", hState);
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

  // Colour sensor module (separate root: /color, /color/...)
  colorBegin(server, AP_SSID, I2C_SDA, I2C_SCL);

  server.begin();
  serverStarted = true;
  Serial.println("HTTP server running.  Robot: /   Colour: /color\n");
}

// =============================================================================
// LOOP
// =============================================================================
void loop() {
  server.handleClient();
  colorPoll();
  runPendingJob();
}
