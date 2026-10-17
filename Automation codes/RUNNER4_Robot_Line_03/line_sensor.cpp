// =============================================================================
// line_sensor.cpp - 9x TCRT5000 line array module for RUNNER4
// Pipeline: ADC oversample -> EMA -> per-sensor min/max normalise -> polarity
//           -> threshold -> weighted centroid -> error(mm) -> PD -> Wz
//           -> junction confirm -> lost / safe-stop state machine
// All internals live in an anonymous namespace. Public API: line_sensor.h
// =============================================================================
#include "line_sensor.h"
#include "line_page.h"
#include <WiFi.h>
#include <Preferences.h>
#include <math.h>

namespace {

const uint8_t PINS[LINE_NUM_SENSORS] = LINE_PIN_LIST;
#define N LINE_NUM_SENSORS

// ---------------- PERSISTENT CONFIG ----------------
struct Config {
  uint32_t magic;
  uint16_t calMin[N], calMax[N];
  float thr;          // normalised 0..1, above = on line
  float kp, kd;       // PD gains (per mm)
  float alpha;        // EMA on raw ADC (1.0 = off)
  float wzMax;        // output clamp
  float spacing;      // mm between sensors
  uint16_t minSpan;   // ADC span below which a sensor is "dead"
  uint16_t cycleMs;   // control period
  uint32_t lostMs;    // failsafe
  uint8_t samples;    // ADC oversample per sensor
  uint8_t jMin, jCycles;
  uint8_t whiteHigh, calValid, logSerial;
};
#define CFG_MAGIC 0x4C494E02UL
Config cfg;
Preferences prefs;
bool cfgFromFlash = false;

void setDefaults() {
  memset(&cfg, 0, sizeof(cfg));
  cfg.magic = CFG_MAGIC;
  for (int i = 0; i < N; i++) { cfg.calMin[i] = 4095; cfg.calMax[i] = 0; }
  cfg.thr = 0.5f; cfg.kp = 0.05f; cfg.kd = 0.20f; cfg.alpha = 0.7f;
  cfg.wzMax = 2.0f; cfg.spacing = 10.0f;
  cfg.minSpan = 100; cfg.cycleMs = 20; cfg.lostMs = 3000;
  cfg.samples = 4; cfg.jMin = 6; cfg.jCycles = 3;
  cfg.whiteHigh = 1; cfg.calValid = 0; cfg.logSerial = 0;
}
bool saveCfg() {
  prefs.begin("line", false);
  size_t w = prefs.putBytes("cfg", &cfg, sizeof(cfg));
  prefs.end();
  return w == sizeof(cfg);
}
void loadCfg() {
  setDefaults();
  prefs.begin("line", true);
  if (prefs.getBytesLength("cfg") == sizeof(Config)) {
    Config tmp;
    prefs.getBytes("cfg", &tmp, sizeof(tmp));
    if (tmp.magic == CFG_MAGIC) { cfg = tmp; cfgFromFlash = true; }
  }
  prefs.end();
}

// ---------------- RUNTIME STATE ----------------
WebServer *srv = nullptr;
const char *apSsid = "";
uint16_t raw[N];
float ema[N], norm[N];
bool sOK[N];
bool emaInit = false;

bool found = false, everFound = false, lost = false, safeStop = false, junction = false;
float errMm = 0, lastErr = 0, centroid = 4, wz = 0;
uint8_t activeCnt = 0, jConsec = 0;
uint16_t activeMask = 0;
const char *jType = "NONE";
const char *statusTxt = "NO CAL";
uint32_t lostSince = 0, lastCycle = 0, lastRateMs = 0, lastLogMs = 0, scanUs = 0, rateCnt = 0;
float rateHz = 0;

bool calActive = false;
uint32_t calUntil = 0;
uint16_t cMin[N], cMax[N];
char calMsg[96] = "Not calibrated";
#define CAL_MS 5000

// ---------------- CALIBRATION ----------------
void flushFilters() {
  emaInit = false; jConsec = 0; junction = false; lostSince = 0;
  lost = false; safeStop = false; lastErr = 0; wz = 0;
}
void startCal() {
  for (int i = 0; i < N; i++) { cMin[i] = 4095; cMax[i] = 0; }
  calActive = true; calUntil = millis() + CAL_MS;
  strcpy(calMsg, "Sweeping... slide array over BLACK floor and WHITE line");
  flushFilters();
}
void finishCal() {
  calActive = false;
  int good = 0; char dead[40] = "";
  for (int i = 0; i < N; i++) {
    cfg.calMin[i] = cMin[i]; cfg.calMax[i] = cMax[i];
    if (cMax[i] > cMin[i] && (cMax[i] - cMin[i]) >= cfg.minSpan) good++;
    else { char t[8]; snprintf(t, sizeof(t), " S%d", i + 1); strncat(dead, t, sizeof(dead) - strlen(dead) - 1); }
  }
  // This is a 9-sensor line array. Treating one changing input as a valid
  // calibration made the dashboard say "calibrated" even when the other
  // eight channels were disconnected, on the wrong GPIOs, or flat.
  cfg.calValid = (good == N);
  if (cfg.calValid) {
    snprintf(calMsg, sizeof(calMsg), "CAL OK: all %d sensors good (press Save to keep)", N);
  } else {
    snprintf(calMsg, sizeof(calMsg), "CAL INCOMPLETE: %d/%d good; check dead/flat:%s", good, N, dead);
  }
  flushFilters();
}

// ---------------- SCAN / CONTROL ----------------
const char *classifyJunction() {
  bool l = (activeMask & 0x007) == 0x007;
  bool r = ((activeMask >> (N - 3)) & 0x7) == 0x7;
  if (l && r) return "T/CROSS";
  if (l) return "LEFT";
  if (r) return "RIGHT";
  return "WIDE";
}

void cycle(uint32_t now) {
  uint32_t t0 = micros();
  for (int i = 0; i < N; i++) {
    uint32_t sum = 0;
    for (int k = 0; k < cfg.samples; k++) sum += analogRead(PINS[i]);
    raw[i] = (uint16_t)(sum / cfg.samples);
  }
  scanUs = micros() - t0;

  if (!emaInit) { for (int i = 0; i < N; i++) ema[i] = raw[i]; emaInit = true; }
  else for (int i = 0; i < N; i++) ema[i] = cfg.alpha * raw[i] + (1.0f - cfg.alpha) * ema[i];

  if (calActive) {                       // track min/max on averaged raw
    for (int i = 0; i < N; i++) { if (raw[i] < cMin[i]) cMin[i] = raw[i]; if (raw[i] > cMax[i]) cMax[i] = raw[i]; }
    statusTxt = "CALIBRATING"; found = false; wz = 0;
    if (now >= calUntil) finishCal();
    return;
  }

  // normalise
  for (int i = 0; i < N; i++) {
    int span = (int)cfg.calMax[i] - (int)cfg.calMin[i];
    sOK[i] = cfg.calValid && span >= (int)cfg.minSpan;
    if (!sOK[i]) { norm[i] = 0; continue; }
    float n = constrain((ema[i] - cfg.calMin[i]) / (float)span, 0.0f, 1.0f);
    norm[i] = cfg.whiteHigh ? n : 1.0f - n;
  }
  if (!cfg.calValid) { statusTxt = "CAL INCOMPLETE"; found = false; wz = 0; return; }

  // weighted centroid
  float sumW = 0, sumV = 0; activeCnt = 0; activeMask = 0;
  for (int i = 0; i < N; i++) {
    float v = norm[i];
    if (v < cfg.thr) v = 0; else { activeCnt++; activeMask |= (1 << i); }
    sumW += v; sumV += i * v;
  }

  if (sumW >= 0.01f) {
    centroid = sumV / sumW;
    errMm = (centroid - (N - 1) / 2.0f) * cfg.spacing;
    wz = constrain(cfg.kp * errMm + cfg.kd * (errMm - lastErr), -cfg.wzMax, cfg.wzMax);
    lastErr = errMm;
    found = true; everFound = true; lost = false; safeStop = false; lostSince = 0;
    if (activeCnt >= cfg.jMin) { if (jConsec < 250) jConsec++; } else jConsec = 0;
    junction = jConsec >= cfg.jCycles;
    jType = junction ? classifyJunction() : "NONE";
    statusTxt = junction ? "JUNCTION" : "OK";
    if (cfg.logSerial) {
      Serial.printf("err(mm): %.1f active: %u | Wz: %.3f%s\n", errMm, activeCnt, wz, junction ? "  <<< JUNCTION" : "");
    }
  } else {
    found = false; wz = 0; jConsec = 0; junction = false; jType = "NONE"; activeCnt = 0; activeMask = 0;
    if (lostSince == 0) lostSince = now;
    lost = true;
    bool wasSafe = safeStop;
    safeStop = (now - lostSince) > cfg.lostMs;
    statusTxt = safeStop ? "SAFE STOP" : "LINE LOST";
    if (cfg.logSerial && now - lastLogMs > 200) {
      lastLogMs = now;
      Serial.printf("LINE LOST, last side: %s\n", lastErr < 0 ? "left" : "right");
    }
    if (cfg.logSerial && safeStop && !wasSafe) Serial.println("SAFE STOP");
  }
}

// ---------------- WEB HANDLERS ----------------
void sendJSON(const char *b) { srv->sendHeader("Cache-Control", "no-store"); srv->send(200, "application/json", b); }

void handleData() {
  static char b[1200];
  uint32_t now = millis();
  uint16_t okMask = 0;
  for (int i = 0; i < N; i++) if (sOK[i]) okMask |= (1 << i);
  int p = snprintf(b, sizeof(b),
    "{\"cal\":%d,\"calact\":%d,\"calleft\":%lu,\"calmsg\":\"%s\",\"status\":\"%s\",\"found\":%d,"
    "\"err\":%.1f,\"cen\":%.2f,\"wz\":%.3f,\"act\":%u,\"mask\":%u,\"okmask\":%u,\"jun\":%d,\"jt\":\"%s\","
    "\"lost\":%d,\"lostms\":%lu,\"safe\":%d,\"side\":\"%s\",\"rate\":%.1f,\"scanus\":%lu,\"thr\":%.2f,\"raw\":[",
    cfg.calValid, calActive, calActive ? (unsigned long)(calUntil > now ? calUntil - now : 0) : 0UL, calMsg, statusTxt, found,
    errMm, centroid, wz, activeCnt, activeMask, okMask, junction, jType,
    lost, lost && lostSince ? (unsigned long)(now - lostSince) : 0UL, safeStop,
    everFound ? (lastErr < 0 ? "left" : "right") : "none", rateHz, (unsigned long)scanUs, cfg.thr);
  for (int i = 0; i < N; i++) p += snprintf(b + p, sizeof(b) - p, "%u%s", raw[i], i < N - 1 ? "," : "");
  p += snprintf(b + p, sizeof(b) - p, "],\"norm\":[");
  for (int i = 0; i < N; i++) p += snprintf(b + p, sizeof(b) - p, "%d%s", (int)(norm[i] * 1000), i < N - 1 ? "," : "");
  snprintf(b + p, sizeof(b) - p, "]}");
  sendJSON(b);
}

void handleConfig() {
  char b[420];
  snprintf(b, sizeof(b),
    "{\"thr\":%.2f,\"kp\":%.3f,\"kd\":%.2f,\"alpha\":%.2f,\"wzmax\":%.1f,\"spacing\":%.1f,\"minspan\":%u,"
    "\"cyc\":%u,\"lost\":%lu,\"samples\":%u,\"jmin\":%u,\"jcyc\":%u,\"white\":%u,\"log\":%u}",
    cfg.thr, cfg.kp, cfg.kd, cfg.alpha, cfg.wzMax, cfg.spacing, cfg.minSpan, cfg.cycleMs,
    (unsigned long)cfg.lostMs, cfg.samples, cfg.jMin, cfg.jCycles, cfg.whiteHigh, cfg.logSerial);
  sendJSON(b);
}

void handleSet() {
  for (int i = 0; i < srv->args(); i++) {
    String k = srv->argName(i); float f = srv->arg(i).toFloat(); int n = srv->arg(i).toInt();
    if (k == "thr") cfg.thr = constrain(f, 0.05f, 0.95f);
    else if (k == "kp") cfg.kp = constrain(f, 0.0f, 2.0f);
    else if (k == "kd") cfg.kd = constrain(f, 0.0f, 5.0f);
    else if (k == "alpha") cfg.alpha = constrain(f, 0.05f, 1.0f);
    else if (k == "wzmax") cfg.wzMax = constrain(f, 0.1f, 10.0f);
    else if (k == "spacing") cfg.spacing = constrain(f, 1.0f, 50.0f);
    else if (k == "minspan") cfg.minSpan = constrain(n, 10, 4000);
    else if (k == "cyc") cfg.cycleMs = constrain(n, 5, 200);
    else if (k == "lost") cfg.lostMs = constrain(n, 100, 60000);
    else if (k == "samples") cfg.samples = constrain(n, 1, 16);
    else if (k == "jmin") cfg.jMin = constrain(n, 2, N);
    else if (k == "jcyc") cfg.jCycles = constrain(n, 1, 20);
    else if (k == "white") { cfg.whiteHigh = n ? 1 : 0; flushFilters(); }
    else if (k == "log") cfg.logSerial = n ? 1 : 0;
  }
  srv->send(200, "text/plain", "OK");
}

void handleCmd() {
  String n = srv->arg("n"), m = "OK";
  if (n == "cal_start") { startCal(); m = "Calibration started (5 s)"; }
  else if (n == "cal_stop") { if (calActive) { finishCal(); m = "Calibration finished"; } else m = "Not calibrating"; }
  else if (n == "cal_clear") {
    for (int i = 0; i < N; i++) { cfg.calMin[i] = 4095; cfg.calMax[i] = 0; }
    cfg.calValid = 0; strcpy(calMsg, "Calibration cleared"); flushFilters(); m = "Calibration cleared";
  }
  else if (n == "save") m = saveCfg() ? "Saved to flash (survives reboot)" : "Save FAILED";
  else if (n == "reset") {                       // tuning back to defaults, keep calibration
    uint16_t mn[N], mx[N]; uint8_t cv = cfg.calValid;
    memcpy(mn, cfg.calMin, sizeof(mn)); memcpy(mx, cfg.calMax, sizeof(mx));
    setDefaults();
    memcpy(cfg.calMin, mn, sizeof(mn)); memcpy(cfg.calMax, mx, sizeof(mx)); cfg.calValid = cv;
    saveCfg(); flushFilters(); m = "Tuning defaults restored (calibration kept)";
  }
  else if (n == "flush") flushFilters();
  else m = "Unknown command";
  srv->send(200, "text/plain", m);
}

void handleWifi() {
  char b[900];
  String ps; for (int i = 0; i < N; i++) { ps += PINS[i]; if (i < N - 1) ps += ","; }
  snprintf(b, sizeof(b),
    "{\"WiFi AP\":\"%s\",\"IP\":\"%s\",\"Channel\":%d,\"Clients\":%d,\"Uptime s\":%lu,\"Free heap KB\":%u,"
    "\"Sensor pins S1..S9 (ADC1)\":\"%s\",\"Control rate\":\"%.1f Hz\",\"Scan time\":\"%lu us\","
    "\"Config source\":\"%s\",\"Calibration\":\"%s\",\"Polarity\":\"%s\"}",
    apSsid, WiFi.softAPIP().toString().c_str(), WiFi.channel(), WiFi.softAPgetStationNum(),
    (unsigned long)(millis() / 1000), (unsigned)(ESP.getFreeHeap() / 1024), ps.c_str(), rateHz, (unsigned long)scanUs,
    cfgFromFlash ? "flash (NVS)" : "defaults", cfg.calValid ? "valid" : "missing", cfg.whiteHigh ? "white = high" : "white = low");
  sendJSON(b);
}

void handleExport() {   // paste into the standalone line-follow sketch
  char b[900];
  int p = snprintf(b, sizeof(b), "/* Generated by Line Array Lab */\nconst int calMin[%d] = {", N);
  for (int i = 0; i < N; i++) p += snprintf(b + p, sizeof(b) - p, "%u%s", cfg.calMin[i], i < N - 1 ? ", " : "};\n");
  p += snprintf(b + p, sizeof(b) - p, "const int calMax[%d] = {", N);
  for (int i = 0; i < N; i++) p += snprintf(b + p, sizeof(b) - p, "%u%s", cfg.calMax[i], i < N - 1 ? ", " : "};\n");
  snprintf(b + p, sizeof(b) - p,
    "#define WHITE_IS_HIGH %s\nconst float LINE_THRESH = %.2f;\nfloat Kp = %.3f;\nfloat Kd = %.2f;\n"
    "const float SENSOR_SPACING_MM = %.1f;\nconst uint32_t LOST_LIMIT_MS = %lu;\n",
    cfg.whiteHigh ? "true" : "false", cfg.thr, cfg.kp, cfg.kd, cfg.spacing, (unsigned long)cfg.lostMs);
  srv->send(200, "text/plain", b);
}

}  // namespace

// ---------------- PUBLIC API ----------------
bool lineBegin(WebServer &server, const char *ssid) {
  srv = &server; apSsid = ssid;
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);          // full 0..3.3 V
  for (int i = 0; i < N; i++) { pinMode(PINS[i], INPUT); raw[i] = 0; norm[i] = 0; sOK[i] = false; }
  loadCfg();
  statusTxt = cfg.calValid ? "OK" : "NO CAL";
  Serial.printf("[LINE] 9-ch array, calibration %s, config %s\n", cfg.calValid ? "valid" : "MISSING (use /line)",
                cfgFromFlash ? "loaded from flash" : "defaults");
  srv->on("/line",        []() { srv->send_P(200, "text/html", LINE_PAGE); });
  srv->on("/line/data",   handleData);
  srv->on("/line/config", handleConfig);
  srv->on("/line/set",    handleSet);
  srv->on("/line/cmd",    handleCmd);
  srv->on("/line/wifi",   handleWifi);
  srv->on("/line/export", handleExport);
  return cfg.calValid;
}

void linePoll() {
  if (!srv) return;
  uint32_t now = millis();
  if (now - lastCycle < cfg.cycleMs) return;
  lastCycle = now;
  cycle(now);
  rateCnt++;
  if (now - lastRateMs >= 1000) { rateHz = rateCnt * 1000.0f / (now - lastRateMs); rateCnt = 0; lastRateMs = now; }
}

bool        lineCalibrated()   { return cfg.calValid; }
bool        lineFound()        { return found; }
float       lineErrorMm()      { return errMm; }
float       lineWz()           { return wz; }
uint8_t     lineActiveCount()  { return activeCnt; }
bool        lineJunction()     { return junction; }
const char *lineJunctionType() { return jType; }
bool        lineLost()         { return lost; }
bool        lineSafeStop()     { return safeStop; }

// Override EMA alpha for line-follow mode.
// Pass -1 to restore the previous value.
static float savedAlphaLF = -1;
void lineSetAlpha(float newAlpha) {
  if (newAlpha < 0) {
    // Restore previous
    if (savedAlphaLF >= 0) { cfg.alpha = savedAlphaLF; savedAlphaLF = -1; }
  } else {
    if (savedAlphaLF < 0) savedAlphaLF = cfg.alpha;  // save current
    cfg.alpha = constrain(newAlpha, 0.05f, 1.0f);
  }
}

