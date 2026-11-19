// =============================================================================
// FusionForce RUNNER4 � 12 DOF Quadruped Robot
// MAIN ROOT : PCA9685 servo control + pose presets + FK/IK + walk/turn gaits
// MODULES   : TCS34725 colour sensor (color_sensor.cpp/.h/color_page.h)
//             9x TCRT5000 line array (line_sensor.cpp/.h/line_page.h)
//             Line follow (line_follow.h/line_follow_page.h)
//             3x VL53L0X ToF distance sensors (tof_sensor.h/tof_page.h)
//             RGB LED status indicator (GPIO 16=Red, 17=Green, 18=Blue)
// BOARD     : ESP32-S3
// VERSION   : 7.0  (Competition FSM + arm interface + nav primitives)
// =============================================================================
//
// WEB ROUTES (one WiFi AP, one web server)
//   http://192.168.4.1/            -> robot control page
//   http://192.168.4.1/color       -> colour sensor lab
//   http://192.168.4.1/line        -> 9-ch line array lab
//   http://192.168.4.1/linefollow  -> line-follow tuning & test lab
//   http://192.168.4.1/tof         -> ToF distance sensor lab  (NEW)
//   /color/data /color/config /color/set /color/cmd /color/wifi /color/export
//   /line/data  /line/config  /line/set  /line/cmd  /line/wifi  /line/export
//   /lf/start   /lf/stop      /lf/status /lf/config /lf/set     /lf/cmd
//   /tof/data
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
//   Main I2C0 (PCA9685 0x40 + VL53L0X ToF):
//     GPIO 11 (SDA) -> PCA9685 SDA and VL53L0X SDA
//     GPIO 12 (SCL) -> PCA9685 SCL and VL53L0X SCL
//     GPIO 13       -> VL53L0X Front XSHUT (active-LOW reset)
//     3.3V / GND    -> PCA9685 VCC / VL53L0X VIN and common GND
//   Color I2C1 (TCS34725 0x29 - separate hardware I2C controller):
//     GPIO 41 (SDA) -> TCS34725 SDA
//     GPIO 42 (SCL) -> TCS34725 SCL
//     GPIO 15       -> TCS34725 LED control (HIGH = white LED on)
//     3.3V / GND    -> TCS34725 VIN and common GND//   Line array (TCRT5000 analog outputs, S1 far LEFT ... S9 far RIGHT, 3.3V
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

#include "config/robot_config.h" // Centralized pins, constants, WiFi
#include <Adafruit_PWMServoDriver.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_system.h>
#include <limits.h>
#include <math.h>


#include "gait/kinematics.h"        // 3-DOF FK and IK solvers
#include "gait/servo_controller.h"   // PCA9685 driver, channel map, pose presets & interpolation
#include "gait/trot_gait.h"          // 6-phase diagonal trot gait (walk, turn, sweep)
#include "sensors/color_sensor.h"    // colour module API
#include "sensors/line_sensor.h"     // line array module API
#include "gait/line_follow.h"        // line-following gait + /linefollow web page
#include "dashboard/robot_page.h"    // INDEX_HTML


// ToF sensor module (header-only implementation)
#define TOF_IMPLEMENTATION
#include "sensors/tof_sensor.h" // 3x VL53L0X + /tof web page

// Competition state machine (includes nav_primitives, arm_interface,
// grid_search, wall_follow, obstacle_push, color_sort)
#include "tasks/competition_fsm.h"

// Vec3f, FK and IK moved to gait/kinematics.h

// I2C pins defined in config/robot_config.h

// -- RGB LED (common-cathode: HIGH = ON)
// --------------------------------------- Wiring: GPIO -> 330ohm resistor ->
// LED leg -> GND (common cathode)
// RGB LED pins defined in config/robot_config.h

void setRgb(bool red, bool green, bool blue) {
  digitalWrite(RGB_RED_PIN, red ? HIGH : LOW);
  digitalWrite(RGB_GREEN_PIN, green ? HIGH : LOW);
  digitalWrite(RGB_BLUE_PIN, blue ? HIGH : LOW);
}

// Update RGB LED to match confirmed colour from TCS34725
void updateColorLed() {
  uint8_t col = colorConfirmed();
  switch (col) {
  case COLOR_ID_RED:
    setRgb(true, false, false);
    break; // RED   on
  case COLOR_ID_GREEN:
    setRgb(false, true, false);
    break; // GREEN on
  case COLOR_ID_BLUE:
    setRgb(false, false, true);
    break; // BLUE  on
  default:
    setRgb(false, false, false);
    break; // off (unknown)
  }
}

// 12-DOF servo driver, pose presets, channel map moved to gait/servo_controller.h
// Walk & turn gait parameters moved to gait/trot_gait.h

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
// JOB SYSTEM � handlers queue work, loop() executes it
// =============================================================================
enum Job : uint8_t {
  JOB_NONE,
  JOB_HOME,
  JOB_LEGHOME,
  JOB_POSE,
  JOB_WALK,
  JOB_TURN,
  JOB_SWEEP,
  JOB_CRAWL,      // = 7  (used by crawl_gait.h)
  JOB_LINEFOLLOW, // = 8  (used by line_follow.h)
  JOB_COMPETITION // = 9  (used by competition_fsm.h)
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

// Line-follow state
volatile bool lfRunning = false, lfStop = false;
int lfStepsTarget = 0, lfStepsDone = 0;
char lfPhaseStr[48] = "";

// =============================================================================
// BACKGROUND SERVICE � called from every wait inside motion code
// =============================================================================
// Rate-limit server.handleClient() to at most once per WEB_SERVE_INTERVAL_MS.
// Previously it was called on every servo step (up to 180 times per
// moveToTarget call), which caused 20-50 ms HTTP processing to stall motion.
// WEB_SERVE_INTERVAL_MS=20 / LED_UPDATE_INTERVAL_MS=50 defined in
// config/robot_config.h  // update RGB LED at most every 50 ms

static uint32_t _lastServeMs = 0;
static uint32_t _lastLedMs = 0;

void serviceBackground() {
  uint32_t now = millis();
  // Throttled HTTP serving � prevents motion stutter from HTTP processing
  if (serverStarted && (now - _lastServeMs >= WEB_SERVE_INTERVAL_MS)) {
    server.handleClient();
    _lastServeMs = millis(); // re-read after handleClient (it may take time)
  }
  colorPoll();
  // Rate-limit LED GPIO writes (no need to toggle LEDs at kHz speeds)
  if (now - _lastLedMs >= LED_UPDATE_INTERVAL_MS) {
    updateColorLed();
    _lastLedMs = now;
  }
  linePoll();
  tofPoll(); // non-blocking ToF (continuous mode)
}

// Wait ms while keeping WiFi + colour sensor + line array alive
void waitMs(uint32_t ms) {
  uint32_t t0 = millis();
  do {
    serviceBackground();
    uint32_t elapsed = millis() - t0;
    if (elapsed >= ms)
      break;
    // Small sleep only if we have more than 1ms left, avoids busy-spin
    uint32_t remaining = ms - elapsed;
    if (remaining > 1)
      delay(1);
  } while (true);
}

// Servo helpers (setServo, moveToTarget, moveToPose) -> gait/servo_controller.h
// Kinematics (computeFK, computeIK) -> gait/kinematics.h

void i2cScan() {
  Serial.println("-------------------------------------------------");
  Serial.println("Main I2C0 Bus Scan (SDA=GPIO 11, SCL=GPIO 12):");
  Serial.println("  Expected devices: 0x40 (PCA9685), 0x29 (VL53L0X ToF)");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  [OK] Device ACK at 0x%02X%s%s\n", addr,
                    addr == 0x40 ? " (PCA9685)" : "",
                    (addr == 0x29 || addr == 0x30) ? " (VL53L0X ToF)" : "");
      found++;
    }
  }
  if (found == 0)
    Serial.println("  [!!] No I2C devices found on Main Bus (check SDA=11, SCL=12)!");
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

// Gait builders and walk/turn/sweep gaits moved to gait/trot_gait.h

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
  case JOB_LINEFOLLOW:
    lineFollowWalk(jobSteps, jobSpeed);
    break;
  case JOB_COMPETITION:
    runCompetition(jobSteps); // jobSteps = checkpoint (1-4)
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

// GET /pose?id=N&speed=ms � response first, move in loop()
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

// GET /savepose?id=N&a00=..  � store angles into a preset (RAM)
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
  Serial.println("  FusionForce RUNNER4 v6.1 � ESP32-S3");
  Serial.println("  Robot + Colour + Line + LineFollow + ToF");
  Serial.println("=============================================");

  // -- RGB LED init (GPIO 16=R, 17=G, 18=B, common-cathode) -----------------
  pinMode(RGB_RED_PIN, OUTPUT);
  digitalWrite(RGB_RED_PIN, LOW);
  pinMode(RGB_GREEN_PIN, OUTPUT);
  digitalWrite(RGB_GREEN_PIN, LOW);
  pinMode(RGB_BLUE_PIN, OUTPUT);
  digitalWrite(RGB_BLUE_PIN, LOW);
  Serial.println("[RGB] LED pins initialised (GPIO 16/17/18).");

  // One shared I2C bus (GPIO 11/12 � GPIO 8/9 are line-array ADC inputs)
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

  // ToF stays on the main bus (GPIO 11/12). The color sensor uses a second
  // hardware I2C controller on GPIO 41/42, so both may use address 0x29.
  tofBegin(server, AP_SSID);

  // Colour sensor module (separate root: /color, /color/...)
  colorBegin(server, AP_SSID, COLOR_I2C_SDA, COLOR_I2C_SCL);

  // Line array module (separate root: /line, /line/...)
  lineBegin(server, AP_SSID);

  // Line-follow module (separate root: /linefollow, /lf/...)
  lineFollowBegin(server, AP_SSID);

  // Competition FSM (separate root: /comp/...)
  competitionBegin(server);

  // Arm: 2 servos wired directly to GPIO 47 (Lift) and GPIO 21 (Grip) via LEDC
  // PWM
  armInit();

  // /arm/* routes � test/calibrate arm servos directly from browser
  server.on("/arm/pick", []() {
    addCORS();
    armPickBall();
    server.send(200, "application/json", "{\"ok\":true,\"cmd\":\"pick\"}");
  });
  server.on("/arm/place", []() {
    addCORS();
    armPlaceBall();
    server.send(200, "application/json", "{\"ok\":true,\"cmd\":\"place\"}");
  });
  server.on("/arm/up", []() {
    addCORS();
    armUp();
    server.send(200, "application/json", "{\"ok\":true,\"cmd\":\"up\"}");
  });
  server.on("/arm/down", []() {
    addCORS();
    armDown();
    server.send(200, "application/json", "{\"ok\":true,\"cmd\":\"down\"}");
  });
  server.on("/arm/open", []() {
    addCORS();
    armOpen();
    server.send(200, "application/json", "{\"ok\":true,\"cmd\":\"open\"}");
  });
  server.on("/arm/close", []() {
    addCORS();
    armClose();
    server.send(200, "application/json", "{\"ok\":true,\"cmd\":\"close\"}");
  });
  server.on("/arm/set", []() {
    addCORS();
    if (server.hasArg("lift")) {
      int deg = constrain(server.arg("lift").toInt(), 0, 180);
      armSetLift(deg);
      Serial.printf("[ARM] Manual: lift=%d\xC2\xB0\n", deg);
    }
    if (server.hasArg("grip")) {
      int deg = constrain(server.arg("grip").toInt(), 0, 180);
      armSetGrip(deg);
      Serial.printf("[ARM] Manual: grip=%d\xC2\xB0\n", deg);
    }
    server.send(200, "application/json", "{\"ok\":true}");
  });

  server.begin();
  serverStarted = true;
  Serial.println("HTTP server running.");
  Serial.println("  Robot:      http://192.168.4.1/");
  Serial.println("  Colour:     http://192.168.4.1/color");
  Serial.println("  Line:       http://192.168.4.1/line");
  Serial.println("  LineFollow: http://192.168.4.1/linefollow");
  Serial.println("  ToF:        http://192.168.4.1/tof");
  Serial.println("  Competition: /comp/start /comp/stop /comp/status");
  Serial.println("  Arm:         /arm/pick /arm/place /arm/up /arm/down "
                 "/arm/open /arm/close");
  Serial.println("  Arm tune:    /arm/set?lift=90&grip=45");
  Serial.println(
      "  Dashboard:  http://192.168.4.1/comp  ? START HERE FOR COMPETITION");
}

// =============================================================================
// LOOP
// =============================================================================
void loop() {
  // Use serviceBackground() to benefit from the same rate-limiting as motion
  // (avoids duplicate handleClient / LED update calls).
  serviceBackground();
  runPendingJob();

  // Check physical start button for competition (only when idle)
  if (!robotBusy() && competitionCheckButton()) {
    Serial.println("[COMP] Start button pressed!");
    jobSteps = 1; // start from task 1
    pendingJob = JOB_COMPETITION;
  }
}
