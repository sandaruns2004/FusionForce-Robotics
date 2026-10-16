// =============================================================================
// FusionForce RUNNER4 — Wi-Fi-only standing comparison
// Board : ESP32-S3   |   Wi-Fi AP, no web UI, no sensors
//
// Sequence on power-up:
//   1) wait 1000 ms
//   2) go to INITIAL pose
//   3) wait 1000 ms
//   4) start Wi-Fi AP; connect your phone and send 's' in Serial Monitor
//   5) go to STANDING pose at the same speed as RUNNER4_Startup
//
// Library: "Adafruit PWM Servo Driver Library"
// Wiring : GPIO 11 (SDA), GPIO 12 (SCL) -> PCA9685
//          Servo power: 6V BEC -> PCA9685 V+ (NEVER from ESP32 3.3V)
// =============================================================================

#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_system.h>

const char *AP_SSID = "RUNNER4-Stand-Test";
const char *AP_PASS = "runner4robot";

// ── I2C ──────────────────────────────────────────────────────────────────────
#define I2C_SDA 11
#define I2C_SCL 12
#define I2C_HZ 400000

// ── PCA9685 ──────────────────────────────────────────────────────────────────
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
#define SERVOMIN 150
#define SERVOMAX 600
#define SERVO_FREQ 50

// ── Timing ───────────────────────────────────────────────────────────────────
#define START_DELAY_MS 1000  // wait after power-on
#define INITIAL_HOLD_MS 1000 // wait in the initial pose
#define STAND_SPEED_MS 1000  // time to move initial -> standing (average speed)

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

// ── Poses [leg][hip, femur, tibia] ───────────────────────────────────────────
const int POSE_INITIAL[4][3] = {
    {45, 80, 160}, {94, 80, 0}, {94, 35, 10}, {45, 92, 160}};
// The full web sketch differs only at BL femur in its initial pose (56 vs 35).
const int POSE_INITIAL_WEB[4][3] = {
    {45, 80, 160}, {94, 80, 0}, {94, 56, 10}, {45, 92, 160}};
const int POSE_STANDING[4][3] = {
    {45, 20, 115}, {94, 135, 46}, {94, 94, 55}, {45, 37, 115}};

int servoAngles[16] = {0};

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

// Jump straight to a pose, staggered 10 ms per servo to reduce current spike.
// Used for the first move, when the real servo positions are unknown.
void setPoseDirect(const int pose[4][3]) {
  for (int leg = 0; leg < 4; leg++)
    for (int joint = 0; joint < 3; joint++) {
      setServo(legChannels[leg][joint], pose[leg][joint]);
      delay(10);
    }
}

// Smooth linear interpolation of all 12 servos over speedMs
void moveToTarget(const int target[4][3], int speedMs) {
  int maxDiff = 0;
  for (int leg = 0; leg < 4; leg++)
    for (int joint = 0; joint < 3; joint++) {
      int diff = abs(target[leg][joint] - servoAngles[legChannels[leg][joint]]);
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
        int nextAngle = startAngles[ch] +
                        (int)roundf(t * (target[leg][joint] - startAngles[ch]));
        if (nextAngle != servoAngles[ch])
          setServo(ch, nextAngle);
      }
    unsigned long stepTarget =
        moveStart + (unsigned long)((long)speedMs * step / maxDiff);
    unsigned long now = millis();
    if (stepTarget > now)
      delay(stepTarget - now);
  }
}

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  unsigned long serialStart = millis();
  while (!Serial && millis() - serialStart < 3000) delay(10);
  Serial.printf("[BOOT] Reset reason: %d%s\n", (int)esp_reset_reason(),
                esp_reset_reason() == ESP_RST_BROWNOUT ? " (BROWNOUT)" : "");

  // I2C + PCA9685 init
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_HZ);
  Wire.setTimeOut(50);

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  Wire.setClock(I2C_HZ);
  delay(10);

  // 1) wait after power-on
  Serial.println("[START] Waiting 1000 ms...");
  delay(START_DELAY_MS);

  // 2) go to INITIAL pose
  Serial.println("[POSE] -> INITIAL");
  setPoseDirect(POSE_INITIAL);

  // 3) hold in initial pose
  delay(INITIAL_HOLD_MS);

  // Start the AP before the stand command. The original startup sketch has
  // identical servo code but never starts Wi-Fi.
  WiFi.mode(WIFI_AP);
  bool apStarted = WiFi.softAP(AP_SSID, AP_PASS);
  WiFi.setSleep(false);
  Serial.printf("[WIFI] AP %s: %s at %s\n", apStarted ? "started" : "FAILED",
                AP_SSID, WiFi.softAPIP().toString().c_str());
  Serial.println("[TEST] Connect phone to AP, support robot, send 's' to stand.");
  Serial.println("[TEST] Send 'f' for the web sketch's initial pose, then 's' again.");
}

// =============================================================================
// LOOP — manual command prevents a repeated stand after a brownout/reboot
// =============================================================================
void loop() {
  if (Serial.available()) {
    char command = Serial.read();
    if (command == 's' || command == 'S') {
      Serial.printf("[TEST] Standing with %d AP client(s) at uptime %lu ms\n",
                    WiFi.softAPgetStationNum(), (unsigned long)millis());
      moveToTarget(POSE_STANDING, STAND_SPEED_MS);
      Serial.println("[DONE] Standing command finished. Check whether legs hold.");
    } else if (command == 'i' || command == 'I') {
      Serial.println("[TEST] Returning to automation initial pose");
      moveToTarget(POSE_INITIAL, STAND_SPEED_MS);
    } else if (command == 'f' || command == 'F') {
      Serial.println("[TEST] Moving to full web sketch initial pose");
      moveToTarget(POSE_INITIAL_WEB, STAND_SPEED_MS);
    }
  }
  static unsigned long lastStatus = 0;
  if (millis() - lastStatus >= 5000) {
    lastStatus = millis();
    Serial.printf("[STATUS] uptime=%lu ms clients=%d heap=%u bytes\n",
                  (unsigned long)millis(), WiFi.softAPgetStationNum(),
                  (unsigned)ESP.getFreeHeap());
  }
  delay(10);
}
