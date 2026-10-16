// =============================================================================
// FusionForce RUNNER4 — small WiFi control: 2 buttons (INITIAL / STANDING)
// Board : ESP32-S3
// Wi-Fi : connect to "RUNNER4-Config" (pass: runner4robot)
//         then open http://192.168.4.1/
//
// Power-up: wait 1000 ms -> INITIAL pose -> then use the buttons.
// Library : "Adafruit PWM Servo Driver Library"
// Wiring  : GPIO 11 (SDA), GPIO 12 (SCL) -> PCA9685. Servo power: 6V BEC -> V+
// =============================================================================

#include <Adafruit_PWMServoDriver.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>

// ── WiFi AP ──────────────────────────────────────────────────────────────────
const char *AP_SSID = "RUNNER4-Config";
const char *AP_PASS = "runner4robot";

// ── I2C / PCA9685 ────────────────────────────────────────────────────────────
#define I2C_SDA 11
#define I2C_SCL 12
#define I2C_HZ 400000
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
#define SERVOMIN 150
#define SERVOMAX 600
#define SERVO_FREQ 50

// ── Timing ───────────────────────────────────────────────────────────────────
#define START_DELAY_MS 1000
#define MOVE_SPEED_MS 1000 // time of each pose transition (average speed)

// ── Channels: legChannels[leg][hip,femur,tibia]; leg 0=FL 1=FR 2=BL 3=BR ─────
const uint8_t legChannels[4][3] = {
    {0, 1, 2}, {3, 4, 5}, {10, 11, 14}, {6, 8, 9}};

// ── Poses [leg][hip, femur, tibia] ───────────────────────────────────────────
const int POSE_INITIAL[4][3] = {
    {45, 80, 160}, {94, 80, 0}, {94, 35, 10}, {45, 92, 160}};
const int POSE_STANDING[4][3] = {
    {45, 20, 115}, {94, 135, 46}, {94, 94, 55}, {45, 37, 115}};

int servoAngles[16] = {0};
WebServer server(80);

// 0 = nothing queued, 1 = INITIAL, 2 = STANDING
volatile int pendingPose = 0;
volatile bool moving = false;

// ── Web page ─────────────────────────────────────────────────────────────────
const char PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>RUNNER4</title>
<style>
body{font-family:sans-serif;text-align:center;background:#111;color:#eee;margin:0;padding:20px}
button{display:block;width:90%;max-width:360px;margin:20px auto;padding:30px 0;
font-size:26px;border:0;border-radius:14px;color:#fff;background:#1e88e5}
button.b2{background:#43a047}
#s{margin-top:20px;font-size:18px;color:#aaa}
</style></head><body>
<h2>RUNNER4</h2>
<button onclick="go(1)">INITIAL</button>
<button class="b2" onclick="go(2)">STANDING</button>
<div id="s">ready</div>
<script>
function go(id){
  fetch('/pose?id='+id).then(r=>r.json()).then(j=>{
    document.getElementById('s').textContent=j.ok?'moving to '+(id==1?'INITIAL':'STANDING'):'busy - wait';
  }).catch(()=>{document.getElementById('s').textContent='no connection';});
}
</script></body></html>
)HTML";

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

// Jump to a pose, staggered 10 ms per servo (used once at boot)
void setPoseDirect(const int pose[4][3]) {
  for (int leg = 0; leg < 4; leg++)
    for (int joint = 0; joint < 3; joint++) {
      setServo(legChannels[leg][joint], pose[leg][joint]);
      delay(10);
    }
}

// Wait while keeping the web server responsive
void waitMs(uint32_t ms) {
  uint32_t t0 = millis();
  do {
    server.handleClient();
    if (millis() - t0 >= ms)
      break;
    delay(1);
  } while (true);
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
      waitMs(stepTarget - now);
    else
      server.handleClient();
  }
}

// =============================================================================
// HTTP HANDLERS
// =============================================================================
void hRoot() { server.send_P(200, "text/html", PAGE); }

// GET /pose?id=1 (INITIAL) or id=2 (STANDING)
void hPose() {
  int id = server.hasArg("id") ? server.arg("id").toInt() : 0;
  if (id != 1 && id != 2) {
    server.send(400, "application/json", "{\"ok\":false,\"msg\":\"bad id\"}");
    return;
  }
  if (moving || pendingPose != 0) {
    server.send(200, "application/json", "{\"ok\":false,\"msg\":\"busy\"}");
    return;
  }
  server.send(200, "application/json", "{\"ok\":true}");
  pendingPose = id; // executed in loop()
}

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_HZ);
  Wire.setTimeOut(50);

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  Wire.setClock(I2C_HZ);
  delay(10);

  // wait 1000 ms, then go to INITIAL pose
  delay(START_DELAY_MS);
  setPoseDirect(POSE_INITIAL);

  // WiFi AP + web server
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  WiFi.setSleep(false);
  Serial.printf("[WIFI] '%s' -> http://%s\n", AP_SSID,
                WiFi.softAPIP().toString().c_str());

  server.on("/", hRoot);
  server.on("/pose", hPose);
  server.begin();
}

// =============================================================================
// LOOP
// =============================================================================
void loop() {
  server.handleClient();

  if (pendingPose != 0) {
    int id = pendingPose;
    pendingPose = 0;
    moving = true;
    moveToTarget(id == 1 ? POSE_INITIAL : POSE_STANDING, MOVE_SPEED_MS);
    moving = false;
  }
}
