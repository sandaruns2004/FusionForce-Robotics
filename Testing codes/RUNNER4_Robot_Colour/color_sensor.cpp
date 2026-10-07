// =============================================================================
// color_sensor.cpp - TCS34725 colour module for RUNNER4
// Pipeline: I2C RGBC -> settle/AVALID gate -> EMA -> normalise -> white-balance
//           -> HSV -> ratio/HSV classifier -> majority vote -> confirmed
// All internals are in an anonymous namespace so they cannot clash with the
// robot code. Public API is in color_sensor.h
// =============================================================================
#include "color_sensor.h"
#include "color_page.h"
#include <Wire.h>
#include <WiFi.h>
#include <Preferences.h>
#include <math.h>

#define TCS_ADDR 0x29

namespace {

// ---------------- TCS34725 REGISTERS ----------------
#define REG_CMD    0x80
#define REG_AUTO   0x20
#define REG_ENABLE 0x00
#define REG_ATIME  0x01
#define REG_CTRL   0x0F
#define REG_ID     0x12
#define REG_STATUS 0x13
#define REG_CDATAL 0x14
#define EN_PON 0x01
#define EN_AEN 0x02

// index: 0=2.4ms 1=24ms 2=50ms 3=103ms 4=154ms
const uint8_t ATIME_V[5] = {0xFF, 0xF6, 0xEB, 0xD5, 0xC0};
const uint8_t GAIN_V[4]  = {0, 1, 2, 3};            // 1x 4x 16x 60x
const uint8_t GAIN_X[4]  = {1, 4, 16, 60};

enum Color : uint8_t { C_UNK = 0, C_RED, C_GREEN, C_BLUE };
const char* CN[4] = {"UNKNOWN", "RED", "GREEN", "BLUE"};
enum Cap : uint8_t { CAP_NONE = 0, CAP_WHITE, CAP_BLACK, CAP_RED, CAP_GREEN, CAP_BLUE };
#define CAP_SAMPLES 20
#define VMAX 15

// ---------------- PERSISTENT CONFIG ----------------
struct Config {
  uint32_t magic;
  float thr[3];      // min normalised value R,G,B
  float dom[3];      // dominance ratio R,G,B
  float hueC[3];     // hue centre R,G,B (deg)
  float hueTol, sMin;
  float wb[3];       // white reference (r/c,g/c,b/c)
  float alpha;       // EMA
  float blackC, colC[3];
  uint16_t minClear; // dark reject
  uint8_t voteN, confirmN, atimeIdx, gainIdx, mode; // mode 0 ratio,1 HSV,2 both agree
  uint8_t autoGain, led, wbOn, wbValid;
  int8_t  ball;      // stored ball colour (0 = none)
};
#define CFG_MAGIC 0x52344302UL
Config cfg;
Preferences prefs;
bool cfgFromFlash = false;

void setDefaults() {
  memset(&cfg, 0, sizeof(cfg));
  cfg.magic = CFG_MAGIC;
  float t[3] = {0.40f, 0.35f, 0.30f}, d[3] = {1.4f, 1.2f, 1.2f}, h[3] = {0, 120, 230};
  for (int i = 0; i < 3; i++) { cfg.thr[i] = t[i]; cfg.dom[i] = d[i]; cfg.hueC[i] = h[i]; cfg.wb[i] = 0.333f; }
  cfg.hueTol = 35; cfg.sMin = 0.25f; cfg.alpha = 0.5f; cfg.minClear = 200;
  cfg.voteN = 7; cfg.confirmN = 3; cfg.atimeIdx = 2; cfg.gainIdx = 1;
  cfg.mode = 0; cfg.autoGain = 0; cfg.led = 1; cfg.wbOn = 1; cfg.wbValid = 0; cfg.ball = 0;
}
bool saveCfg() {
  prefs.begin("tcs", false);
  size_t w = prefs.putBytes("cfg", &cfg, sizeof(cfg));
  prefs.end();
  return w == sizeof(cfg);
}
void loadCfg() {
  setDefaults();
  prefs.begin("tcs", true);
  if (prefs.getBytesLength("cfg") == sizeof(Config)) {
    Config tmp;
    prefs.getBytes("cfg", &tmp, sizeof(tmp));
    if (tmp.magic == CFG_MAGIC) { cfg = tmp; cfgFromFlash = true; }
  }
  prefs.end();
}

// ---------------- RUNTIME STATE ----------------
WebServer* srv = nullptr;
const char* apSsid = "";
int sdaPin = 0, sclPin = 0;
bool sensorOK = false;
uint8_t sensorId = 0;
uint16_t rawR, rawG, rawB, rawC;
float ema[4]; bool emaInit = false;
float nRaw[3], nWB[3], H, S, V, lux;
const char* statusTxt = "OFFLINE";
Color inst = C_UNK, voted = C_UNK, confirmed = C_UNK, lastVoted = C_UNK;
uint8_t vbuf[VMAX], vcount = 0, stableCnt = 0;
uint32_t settleUntil = 0, lastPoll = 0, lastSampleMs = 0, lastGainChg = 0, lastInitTry = 0, lastRateMs = 0, lastPrint = 0;
uint32_t sampleCount = 0, i2cErr = 0, rateCnt = 0;
float rateHz = 0;
Cap capTarget = CAP_NONE; uint8_t capN = 0; double acc[4];
char capMsg[96] = "Ready";

// ---------------- I2C ----------------
bool wr8(uint8_t reg, uint8_t v) {
  Wire.beginTransmission(TCS_ADDR); Wire.write(REG_CMD | reg); Wire.write(v);
  return Wire.endTransmission() == 0;
}
bool rdN(uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(TCS_ADDR);
  Wire.write(REG_CMD | (n > 1 ? REG_AUTO : 0) | reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)TCS_ADDR, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

// ---------------- HELPERS ----------------
float integMs()  { return (256 - ATIME_V[cfg.atimeIdx]) * 2.4f; }
float maxCount() { float m = (256 - ATIME_V[cfg.atimeIdx]) * 1024.0f; return m > 65535 ? 65535 : m; }

void applyHW() {
  wr8(REG_ATIME, ATIME_V[cfg.atimeIdx]);
  wr8(REG_CTRL, GAIN_V[cfg.gainIdx]);
  digitalWrite(COLOR_LED_PIN, cfg.led ? HIGH : LOW);
}
void flushFilters() {
  emaInit = false; vcount = 0; stableCnt = 0; lastVoted = C_UNK;
  voted = confirmed = inst = C_UNK;
  settleUntil = millis() + (uint32_t)(2.5f * integMs()) + 5;   // discard stale integrations
}
bool sensorInit() {
  if (!rdN(REG_ID, &sensorId, 1)) return false;
  if (sensorId != 0x44 && sensorId != 0x4D && sensorId != 0x10) return false;
  wr8(REG_ENABLE, EN_PON); delay(3);
  wr8(REG_ENABLE, EN_PON | EN_AEN);
  applyHW(); flushFilters();
  return true;
}
void rgb2hsv(const float* c, float* h, float* s, float* v) {
  float mx = fmaxf(fmaxf(c[0], c[1]), c[2]), mn = fminf(fminf(c[0], c[1]), c[2]), d = mx - mn;
  *v = mx; *s = mx > 1e-5f ? d / mx : 0;
  if (d < 1e-5f) { *h = 0; return; }
  if (mx == c[0])      *h = 60.0f * fmodf((c[1] - c[2]) / d, 6.0f);
  else if (mx == c[1]) *h = 60.0f * ((c[2] - c[0]) / d + 2.0f);
  else                 *h = 60.0f * ((c[0] - c[1]) / d + 4.0f);
  if (*h < 0) *h += 360;
}
void applyWB(const float* in, float* out) {
  bool on = cfg.wbOn && cfg.wbValid;
  for (int i = 0; i < 3; i++) out[i] = on ? constrain(in[i] / cfg.wb[i] * 0.333f, 0.0f, 1.0f) : in[i];
}
void recomputeMinClear() {
  float mn = 1e9; bool any = false;
  for (int i = 0; i < 3; i++) if (cfg.colC[i] > 0) { mn = fminf(mn, cfg.colC[i]); any = true; }
  float m;
  if (cfg.blackC > 0 && any) m = (cfg.blackC + mn) * 0.5f;
  else if (cfg.blackC > 0)   m = cfg.blackC * 2.0f;
  else if (any)              m = mn * 0.5f;
  else return;
  cfg.minClear = (uint16_t)constrain(m, 20.0f, 60000.0f);
}

// ---------------- CLASSIFIERS ----------------
Color classRatio() {                     // report: ratio-dominance
  for (int i = 0; i < 3; i++) {
    bool ok = nWB[i] > cfg.thr[i];
    for (int j = 0; j < 3 && ok; j++) if (j != i && !(nWB[i] > nWB[j] * cfg.dom[i])) ok = false;
    if (ok) return (Color)(i + 1);
  }
  return C_UNK;
}
Color classHSV() {                       // nearest hue centre within tolerance
  if (S < cfg.sMin) return C_UNK;
  int best = -1; float bd = 1e9;
  for (int i = 0; i < 3; i++) {
    float d = fabsf(H - cfg.hueC[i]); if (d > 180) d = 360 - d;
    if (d < bd) { bd = d; best = i; }
  }
  return bd <= cfg.hueTol ? (Color)(best + 1) : C_UNK;
}

void pushVote(Color c) {
  for (int i = 0; i < VMAX - 1; i++) vbuf[i] = vbuf[i + 1];
  vbuf[VMAX - 1] = c; if (vcount < VMAX) vcount++;
  uint8_t n = min(vcount, cfg.voteN), cnt[4] = {0, 0, 0, 0};
  for (int i = 0; i < n; i++) cnt[vbuf[VMAX - 1 - i]]++;
  int best = 0; for (int i = 1; i < 4; i++) if (cnt[i] > cnt[best]) best = i;
  voted = (cnt[best] * 2 > n) ? (Color)best : C_UNK;       // strict majority
  if (voted != C_UNK && voted == lastVoted) stableCnt = min(stableCnt + 1, 250);
  else stableCnt = (voted != C_UNK) ? 1 : 0;
  lastVoted = voted;
  confirmed = (voted != C_UNK && stableCnt >= cfg.confirmN) ? voted : C_UNK;
}

// ---------------- CAPTURE / CALIBRATION ----------------
void startCap(Cap t) {
  if (!sensorOK) { strcpy(capMsg, "Sensor offline"); return; }
  capTarget = t; capN = 0; memset(acc, 0, sizeof(acc));
  if (cfg.autoGain) { cfg.autoGain = 0; strcpy(capMsg, "Auto-gain turned OFF. Capturing..."); }
  else strcpy(capMsg, "Capturing...");
  flushFilters();
}
void finishCap() {
  float a[3] = {(float)(acc[0] / capN), (float)(acc[1] / capN), (float)(acc[2] / capN)};
  float c = (float)(acc[3] / capN);
  if (capTarget == CAP_WHITE) {
    for (int i = 0; i < 3; i++) cfg.wb[i] = fmaxf(a[i], 0.01f);
    cfg.wbValid = 1; cfg.wbOn = 1;
    snprintf(capMsg, sizeof(capMsg), "WHITE OK: r/c=%.3f g/c=%.3f b/c=%.3f clear=%.0f", a[0], a[1], a[2], c);
  } else if (capTarget == CAP_BLACK) {
    cfg.blackC = c; recomputeMinClear();
    snprintf(capMsg, sizeof(capMsg), "BLACK OK: clear=%.0f -> min clear=%u", c, cfg.minClear);
  } else {
    int k = capTarget - CAP_RED;
    float mo = 0; for (int j = 0; j < 3; j++) if (j != k) mo = fmaxf(mo, a[j]);
    if (!(a[k] > mo * 1.05f)) {
      snprintf(capMsg, sizeof(capMsg), "FAILED: %s not dominant (%.2f/%.2f/%.2f)", CN[k + 1], a[0], a[1], a[2]);
    } else {
      cfg.thr[k] = constrain(a[k] * 0.85f, 0.2f, 0.9f);
      cfg.dom[k] = constrain(a[k] / mo * 0.85f, 1.05f, 3.0f);
      float h, s, v; rgb2hsv(a, &h, &s, &v); cfg.hueC[k] = h; cfg.colC[k] = c;
      recomputeMinClear();
      snprintf(capMsg, sizeof(capMsg), "%s OK: min=%.2f dom=%.2f hue=%.0f", CN[k + 1], cfg.thr[k], cfg.dom[k], h);
    }
  }
  capTarget = CAP_NONE; flushFilters();
}

// ---------------- SAMPLE PROCESSING ----------------
void adaptGain(uint16_t c) {
  if (millis() - lastGainChg < 1500 || capTarget != CAP_NONE) return;
  float f = c / maxCount();
  if (f > 0.90f && cfg.gainIdx > 0) cfg.gainIdx--;
  else if (f < 0.05f && cfg.gainIdx < 3) cfg.gainIdx++;
  else return;
  lastGainChg = millis(); applyHW(); flushFilters();
}

void processSample(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  rawR = r; rawG = g; rawB = b; rawC = c;
  float in[4] = {(float)r, (float)g, (float)b, (float)c};
  if (!emaInit) { memcpy(ema, in, sizeof(ema)); emaInit = true; }
  else for (int i = 0; i < 4; i++) ema[i] = cfg.alpha * in[i] + (1 - cfg.alpha) * ema[i];

  bool sat  = c >= maxCount() * 0.95f;
  bool dark = ema[3] < cfg.minClear;
  for (int i = 0; i < 3; i++) nRaw[i] = ema[3] > 1 ? constrain(ema[i] / ema[3], 0.0f, 1.0f) : 0;
  applyWB(nRaw, nWB);
  rgb2hsv(nWB, &H, &S, &V);
  lux = fmaxf(-0.32466f * ema[0] + 1.57837f * ema[1] - 0.73191f * ema[2], 0.0f);
  statusTxt = sat ? "SATURATED" : dark ? "DARK" : "OK";

  Color ci = C_UNK;
  if (!sat && !dark) {
    Color a = classRatio(), h = classHSV();
    ci = cfg.mode == 0 ? a : cfg.mode == 1 ? h : (a == h ? a : C_UNK);
  }
  inst = ci;
  pushVote(ci);

  if (capTarget != CAP_NONE) {                 // use instantaneous (uncorrelated) samples
    if (capTarget == CAP_BLACK) { acc[3] += c; capN++; }
    else if (c > 1 && !sat) {
      float ir[3] = {(float)r / c, (float)g / c, (float)b / c}, o[3];
      if (capTarget == CAP_WHITE) memcpy(o, ir, sizeof(o)); else applyWB(ir, o);
      for (int i = 0; i < 3; i++) acc[i] += o[i];
      acc[3] += c; capN++;
    }
    if (capN >= CAP_SAMPLES) finishCap();
  }
  if (cfg.autoGain) adaptGain(c);
}

void pollSensor() {
  uint32_t now = millis();
  if (now - lastPoll < 5) return;
  lastPoll = now;
  if (!sensorOK) {
    if (now - lastInitTry > 2000) { lastInitTry = now; sensorOK = sensorInit(); }
    statusTxt = "OFFLINE"; return;
  }
  if (now - lastSampleMs < integMs() * 0.9f) return;       // one sample per integration
  uint8_t st;
  if (!rdN(REG_STATUS, &st, 1)) { i2cErr++; sensorOK = false; return; }
  if (!(st & 0x01)) return;                                // AVALID
  uint8_t d[8];
  if (!rdN(REG_CDATAL, d, 8)) { i2cErr++; sensorOK = false; return; }
  lastSampleMs = now;
  if (now < settleUntil) { statusTxt = "SETTLING"; return; }
  uint16_t c = d[0] | d[1] << 8, r = d[2] | d[3] << 8, g = d[4] | d[5] << 8, b = d[6] | d[7] << 8;
  sampleCount++; rateCnt++;
  processSample(r, g, b, c);
  if (now - lastRateMs >= 1000) { rateHz = rateCnt * 1000.0f / (now - lastRateMs); rateCnt = 0; lastRateMs = now; }
}

// ---------------- WEB HANDLERS ----------------
void sendJSON(const char* b) { srv->sendHeader("Cache-Control", "no-store"); srv->send(200, "application/json", b); }

void handleData() {
  char b[1100];
  int match = cfg.ball == 0 ? -2 : (confirmed == C_UNK ? -1 : (confirmed == cfg.ball ? 1 : 0));
  int p = snprintf(b, sizeof(b),
    "{\"ok\":%d,\"r\":%u,\"g\":%u,\"b\":%u,\"c\":%u,\"nr\":%.4f,\"ng\":%.4f,\"nb\":%.4f,"
    "\"h\":%.1f,\"s\":%.3f,\"v\":%.3f,\"lux\":%.1f,\"inst\":\"%s\",\"voted\":\"%s\",\"conf\":\"%s\","
    "\"stable\":%u,\"status\":\"%s\",\"ball\":\"%s\",\"match\":%d,\"cap\":%d,\"capn\":%u,\"capmsg\":\"%s\","
    "\"rate\":%.1f,\"gain\":%u,\"it\":%.1f,\"mx\":%u,\"votes\":[",
    sensorOK, rawR, rawG, rawB, rawC, nWB[0], nWB[1], nWB[2], H, S, V, lux, CN[inst], CN[voted], CN[confirmed],
    stableCnt, statusTxt, cfg.ball ? CN[cfg.ball] : "none", match, capTarget != CAP_NONE, capN, capMsg,
    rateHz, GAIN_X[cfg.gainIdx], integMs(), (unsigned)maxCount());
  uint8_t n = min(vcount, cfg.voteN);
  for (int i = n - 1; i >= 0 && p < (int)sizeof(b) - 8; i--) p += snprintf(b + p, sizeof(b) - p, "%u%s", vbuf[VMAX - 1 - i], i ? "," : "");
  snprintf(b + p, sizeof(b) - p, "]}");
  sendJSON(b);
}

void handleConfig() {
  char b[600];
  snprintf(b, sizeof(b),
    "{\"red\":%.2f,\"green\":%.2f,\"blue\":%.2f,\"domr\":%.2f,\"domg\":%.2f,\"domb\":%.2f,\"smin\":%.2f,\"htol\":%.0f,"
    "\"alpha\":%.2f,\"minc\":%u,\"vote\":%u,\"conf\":%u,\"mode\":%u,\"atime\":%u,\"gain\":%u,\"led\":%u,\"auto\":%u,\"wb\":%u}",
    cfg.thr[0], cfg.thr[1], cfg.thr[2], cfg.dom[0], cfg.dom[1], cfg.dom[2], cfg.sMin, cfg.hueTol,
    cfg.alpha, cfg.minClear, cfg.voteN, cfg.confirmN, cfg.mode, cfg.atimeIdx, cfg.gainIdx, cfg.led, cfg.autoGain, cfg.wbOn);
  sendJSON(b);
}

void handleSet() {
  bool hw = false;
  for (int i = 0; i < srv->args(); i++) {
    String k = srv->argName(i); float f = srv->arg(i).toFloat(); int n = srv->arg(i).toInt();
    if (k == "red") cfg.thr[0] = f; else if (k == "green") cfg.thr[1] = f; else if (k == "blue") cfg.thr[2] = f;
    else if (k == "domr") cfg.dom[0] = f; else if (k == "domg") cfg.dom[1] = f; else if (k == "domb") cfg.dom[2] = f;
    else if (k == "smin") cfg.sMin = f; else if (k == "htol") cfg.hueTol = f;
    else if (k == "alpha") cfg.alpha = constrain(f, 0.05f, 1.0f);
    else if (k == "minc") cfg.minClear = constrain(n, 0, 60000);
    else if (k == "vote") cfg.voteN = constrain(n, 3, VMAX); else if (k == "conf") cfg.confirmN = constrain(n, 1, 20);
    else if (k == "mode") cfg.mode = constrain(n, 0, 2);
    else if (k == "atime") { cfg.atimeIdx = constrain(n, 0, 4); hw = true; }
    else if (k == "gain")  { cfg.gainIdx = constrain(n, 0, 3); hw = true; }
    else if (k == "led")   { cfg.led = n ? 1 : 0; hw = true; }
    else if (k == "auto")  cfg.autoGain = n ? 1 : 0;
    else if (k == "wb")    cfg.wbOn = n ? 1 : 0;
  }
  if (hw) { applyHW(); flushFilters(); }
  srv->send(200, "text/plain", "OK");
}

void handleCmd() {
  String n = srv->arg("n"), m = "OK";
  if (n == "cap_white") startCap(CAP_WHITE); else if (n == "cap_black") startCap(CAP_BLACK);
  else if (n == "cap_red") startCap(CAP_RED); else if (n == "cap_green") startCap(CAP_GREEN); else if (n == "cap_blue") startCap(CAP_BLUE);
  else if (n == "ball_store") {
    if (confirmed != C_UNK) { cfg.ball = confirmed; saveCfg(); m = String("Ball colour stored: ") + CN[confirmed]; }
    else m = "No confirmed colour yet - wait for a stable reading";
  }
  else if (n == "ball_clear") { cfg.ball = 0; saveCfg(); m = "Ball cleared"; }
  else if (n == "save") m = saveCfg() ? "Saved to flash (survives reboot)" : "Save FAILED";
  else if (n == "reset") { setDefaults(); saveCfg(); applyHW(); flushFilters(); m = "Defaults restored"; }
  else if (n == "flush") flushFilters();
  else if (n == "reinit") { sensorOK = sensorInit(); m = sensorOK ? "Sensor OK" : "Sensor not found"; }
  else m = "Unknown command";
  srv->send(200, "text/plain", m);
}

void handleWifi() {
  char b[700];
  snprintf(b, sizeof(b),
    "{\"WiFi AP\":\"%s\",\"IP\":\"%s\",\"MAC\":\"%s\",\"Channel\":%d,\"Clients\":%d,\"Uptime s\":%lu,\"Free heap KB\":%u,"
    "\"Sensor ID\":\"0x%02X\",\"I2C pins SDA/SCL\":\"%d/%d @ %d kHz\",\"I2C errors\":%lu,\"Samples\":%lu,"
    "\"Sample age ms\":%lu,\"Config source\":\"%s\",\"Min clear\":%u,\"Stored ball\":\"%s\"}",
    apSsid, WiFi.softAPIP().toString().c_str(), WiFi.softAPmacAddress().c_str(), WiFi.channel(), WiFi.softAPgetStationNum(),
    (unsigned long)(millis() / 1000), (unsigned)(ESP.getFreeHeap() / 1024), sensorId, sdaPin, sclPin, 400,
    (unsigned long)i2cErr, (unsigned long)sampleCount, (unsigned long)(millis() - lastSampleMs),
    cfgFromFlash ? "flash (NVS)" : "defaults", cfg.minClear, cfg.ball ? CN[cfg.ball] : "none");
  sendJSON(b);
}

void handleExport() {   // paste into STM32 tcs34725.c
  char b[1400];
  int p = snprintf(b, sizeof(b),
    "/* Generated by TCS34725 Colour Lab - gain %ux, ATIME 0x%02X (%.1f ms) */\n"
    "#define TCS_MIN_CLEAR %u\n", GAIN_X[cfg.gainIdx], ATIME_V[cfg.atimeIdx], integMs(), cfg.minClear);
  if (cfg.wbOn && cfg.wbValid)
    p += snprintf(b + p, sizeof(b) - p, "static const float WB_R=%.4ff, WB_G=%.4ff, WB_B=%.4ff;\n"
      "/* after r_n=r/c etc:  r_n = r_n/WB_R*0.333f;  g_n = g_n/WB_G*0.333f;  b_n = b_n/WB_B*0.333f; */\n", cfg.wb[0], cfg.wb[1], cfg.wb[2]);
  const char* nm[3] = {"COLOR_RED", "COLOR_GREEN", "COLOR_BLUE"}; const char* v[3] = {"r_n", "g_n", "b_n"};
  p += snprintf(b + p, sizeof(b) - p, "if (raw->c < TCS_MIN_CLEAR) return COLOR_UNKNOWN;\n");
  for (int i = 0; i < 3; i++) {
    int j = (i + 1) % 3, k = (i + 2) % 3;
    p += snprintf(b + p, sizeof(b) - p, "if (%s > %.2ff && %s > %s * %.2ff && %s > %s * %.2ff) return %s;\n",
      v[i], cfg.thr[i], v[i], v[j], cfg.dom[i], v[i], v[k], cfg.dom[i], nm[i]);
  }
  snprintf(b + p, sizeof(b) - p, "return COLOR_UNKNOWN;\n");
  srv->send(200, "text/plain", b);
}


}  // namespace

// ---------------- PUBLIC API ----------------
bool colorBegin(WebServer &server, const char *ssid, int sda, int scl) {
  srv = &server; apSsid = ssid; sdaPin = sda; sclPin = scl;
  pinMode(COLOR_LED_PIN, OUTPUT); digitalWrite(COLOR_LED_PIN, LOW);
  loadCfg();
  sensorOK = sensorInit();
  Serial.printf("[COLOR] TCS34725 %s, config %s\n", sensorOK ? "detected" : "NOT found (will retry)",
                cfgFromFlash ? "loaded from flash" : "defaults");
  srv->on("/color",        []() { srv->send_P(200, "text/html", COLOR_PAGE); });
  srv->on("/color/data",   handleData);
  srv->on("/color/config", handleConfig);
  srv->on("/color/set",    handleSet);
  srv->on("/color/cmd",    handleCmd);
  srv->on("/color/wifi",   handleWifi);
  srv->on("/color/export", handleExport);
  return sensorOK;
}
void colorPoll() { if (srv) pollSensor(); }
bool colorSensorOnline() { return sensorOK; }
uint8_t colorConfirmed() { return (uint8_t)confirmed; }
uint8_t colorStoredBall() { return (uint8_t)(cfg.ball < 0 ? 0 : cfg.ball); }
const char *colorName(uint8_t id) { return CN[id > 3 ? 0 : id]; }
