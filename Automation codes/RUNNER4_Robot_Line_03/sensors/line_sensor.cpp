#pragma once
// =============================================================================
// line_sensor.cpp - 9x TCRT5000 line array module for RUNNER4
// Pipeline: ADC oversample -> EMA -> per-sensor min/max normalise -> polarity
//           -> threshold -> weighted centroid -> error(mm) -> PD -> Wz
//           -> junction confirm -> lost / safe-stop state machine
// All internals live in namespace LineSensorInternal to prevent collisions.
// Public API: line_sensor.h
// =============================================================================
#include "line_sensor.h"
#include "../dashboard/line_page.h"
#include <WiFi.h>
#include <Preferences.h>
#include <math.h>

namespace LineSensorInternal {

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
uint16_t activeMask = 0, okMask = 0;
const char *jType = "NONE";
const char *statusTxt = "NO CAL";
const char *lastSide = "none";   // last side the line was seen on (for lost reporting)
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
  uint8_t okCount = 0;
  okMask = 0;
  for (int i = 0; i < N; i++) {
    uint16_t span = (cMax[i] > cMin[i]) ? (cMax[i] - cMin[i]) : 0;
    if (span >= cfg.minSpan) {
      cfg.calMin[i] = cMin[i]; cfg.calMax[i] = cMax[i];
      sOK[i] = true; okMask |= (1 << i); okCount++;
    } else {
      sOK[i] = false;
    }
  }
  if (okCount >= N - 2) {
    cfg.calValid = 1; statusTxt = "OK";
    snprintf(calMsg, sizeof(calMsg), "Calibration OK (%u/%u sensors active)", okCount, N);
  } else {
    cfg.calValid = 0; statusTxt = "CAL FAILED";
    snprintf(calMsg, sizeof(calMsg), "Calibration FAILED - only %u/%u sensors had enough span (min %u)",
             okCount, N, cfg.minSpan);
  }
  flushFilters();
}

// ---------------- ADC SCAN ----------------
void readSensors() {
  uint32_t t0 = micros();
  uint8_t smp = max((uint8_t)1, cfg.samples);
  for (int i = 0; i < N; i++) {
    uint32_t acc = 0;
    for (uint8_t s = 0; s < smp; s++) acc += analogRead(PINS[i]);
    raw[i] = acc / smp;
  }
  scanUs = micros() - t0;
}

// ---------------- ALGORITHM CYCLE ----------------
void cycle(uint32_t now) {
  readSensors();

  if (calActive) {
    for (int i = 0; i < N; i++) {
      if (raw[i] < cMin[i]) cMin[i] = raw[i];
      if (raw[i] > cMax[i]) cMax[i] = raw[i];
    }
    if (now >= calUntil) finishCal();
    return;
  }

  for (int i = 0; i < N; i++) {
    if (!emaInit) ema[i] = raw[i];
    else ema[i] = cfg.alpha * raw[i] + (1.0f - cfg.alpha) * ema[i];
  }
  emaInit = true;

  if (!cfg.calValid) {
    for (int i = 0; i < N; i++) norm[i] = ema[i] / 4095.0f;
    found = lost = safeStop = junction = false;
    activeCnt = 0; activeMask = 0; errMm = 0; wz = 0;
    statusTxt = "NO CAL";
    return;
  }

  activeCnt = 0; activeMask = 0; okMask = 0;
  for (int i = 0; i < N; i++) {
    uint16_t span = (cfg.calMax[i] > cfg.calMin[i]) ? (cfg.calMax[i] - cfg.calMin[i]) : 1;
    if (sOK[i]) okMask |= (1 << i);
    float n = (ema[i] - cfg.calMin[i]) / (float)span;
    n = constrain(n, 0.0f, 1.0f);
    if (!cfg.whiteHigh) n = 1.0f - n;
    norm[i] = n;
    if (norm[i] >= cfg.thr) { activeCnt++; activeMask |= (1 << i); }
  }

  float sumW = 0, sumWN = 0;
  for (int i = 0; i < N; i++) {
    if (norm[i] >= cfg.thr) {
      float w = norm[i] - cfg.thr;
      sumW += w;
      sumWN += w * i;
    }
  }

  if (sumW > 0.001f) {
    centroid = sumWN / sumW;
    errMm = (centroid - (N - 1) / 2.0f) * cfg.spacing;
    lastSide = (errMm < 0) ? "left" : (errMm > 0) ? "right" : "center";
    found = true; everFound = true;
    lost = false; safeStop = false; lostSince = 0;
  } else {
    found = false;
    if (everFound) {
      if (!lost) { lost = true; lostSince = now; }
      else if (now - lostSince > cfg.lostMs) safeStop = true;
    }
  }

  if (activeCnt >= cfg.jMin) {
    if (jConsec < 255) jConsec++;
    if (jConsec >= cfg.jCycles) junction = true;
  } else {
    jConsec = 0; junction = false;
  }

  if (junction) {
    bool leftSide  = (activeMask & 0x007) != 0;
    bool rightSide = (activeMask & 0x1C0) != 0;
    bool centre    = (activeMask & 0x038) != 0;
    if (leftSide && rightSide && centre) jType = "T/CROSS";
    else if (leftSide && !rightSide)     jType = "LEFT";
    else if (rightSide && !leftSide)     jType = "RIGHT";
    else                                 jType = "WIDE";
  } else {
    jType = "NONE";
  }

  if (found) {
    float dt = cfg.cycleMs / 1000.0f;
    float dErr = (dt > 0.001f) ? (errMm - lastErr) / dt : 0;
    lastErr = errMm;
    wz = constrain(cfg.kp * errMm + cfg.kd * dErr, -cfg.wzMax, cfg.wzMax);
  } else {
    wz = 0;
  }

  statusTxt = safeStop ? "SAFE STOP" : lost ? "LINE LOST" : junction ? jType : found ? "LOCKED" : "SEARCHING";

  if (cfg.logSerial && (now - lastLogMs >= 200)) {
    lastLogMs = now;
    Serial.printf("[LINE] err=%+5.1fmm wz=%+4.2f act=%u st=%s raw4=%u n4=%.2f\n",
                  errMm, wz, activeCnt, statusTxt, raw[4], norm[4]);
  }
}

// ---------------- WEB HANDLERS ----------------
void sendJSON(const char *b) { srv->sendHeader("Cache-Control", "no-store"); srv->send(200, "application/json", b); }

void handleData() {
  char b[1400];
  uint32_t lostMs = (lost && lostSince) ? (millis() - lostSince) : 0;
  uint32_t calLeft = calActive ? (calUntil > millis() ? calUntil - millis() : 0) : 0;
  int p = snprintf(b, sizeof(b),
    "{\"cal\":%u,\"found\":%d,\"err\":%.2f,\"wz\":%.2f,\"cen\":%.2f,\"act\":%u,\"mask\":%u,"
    "\"jun\":%d,\"jt\":\"%s\",\"lost\":%d,\"safe\":%d,\"status\":\"%s\","
    "\"rate\":%.1f,\"scanus\":%lu,\"calact\":%d,\"calmsg\":\"%s\","
    "\"thr\":%.3f,\"lostms\":%lu,\"side\":\"%s\",\"calleft\":%lu,"
    "\"okmask\":%u,"
    "\"raw\":[",
    cfg.calValid, found, errMm, wz, centroid, activeCnt, activeMask,
    junction, jType, lost, safeStop, statusTxt,
    rateHz, (unsigned long)scanUs, calActive, calMsg,
    cfg.thr, (unsigned long)lostMs, lastSide, (unsigned long)calLeft,
    okMask);
  for (int i = 0; i < N && p < (int)sizeof(b) - 10; i++) p += snprintf(b + p, sizeof(b) - p, "%u%s", raw[i], (i < N - 1) ? "," : "");
  p += snprintf(b + p, sizeof(b) - p, "],\"norm\":[");
  // JS divides norm by 1000, so send as millis (integer × 1000) for precision
  for (int i = 0; i < N && p < (int)sizeof(b) - 10; i++) p += snprintf(b + p, sizeof(b) - p, "%u%s", (unsigned)(norm[i] * 1000.0f + 0.5f), (i < N - 1) ? "," : "");
  snprintf(b + p, sizeof(b) - p, "]}");
  sendJSON(b);
}

void handleConfig() {
  char b[900];
  int p = snprintf(b, sizeof(b),
    "{\"thr\":%.2f,\"kp\":%.3f,\"kd\":%.3f,\"alpha\":%.2f,\"wzmax\":%.2f,\"spacing\":%.1f,"
    "\"minspan\":%u,\"cyc\":%u,\"lost\":%lu,\"samples\":%u,\"jmin\":%u,\"jcyc\":%u,"
    "\"white\":%u,\"valid\":%u,\"log\":%u,\"cmin\":[",
    cfg.thr, cfg.kp, cfg.kd, cfg.alpha, cfg.wzMax, cfg.spacing,
    cfg.minSpan, cfg.cycleMs, (unsigned long)cfg.lostMs, cfg.samples, cfg.jMin, cfg.jCycles,
    cfg.whiteHigh, cfg.calValid, cfg.logSerial);
  for (int i = 0; i < N && p < (int)sizeof(b) - 8; i++) p += snprintf(b + p, sizeof(b) - p, "%u%s", cfg.calMin[i], (i < N - 1) ? "," : "");
  p += snprintf(b + p, sizeof(b) - p, "],\"cmax\":[");
  for (int i = 0; i < N && p < (int)sizeof(b) - 8; i++) p += snprintf(b + p, sizeof(b) - p, "%u%s", cfg.calMax[i], (i < N - 1) ? "," : "");
  snprintf(b + p, sizeof(b) - p, "]}");
  sendJSON(b);
}

void handleSet() {
  for (int i = 0; i < srv->args(); i++) {
    String k = srv->argName(i); float f = srv->arg(i).toFloat(); int n = srv->arg(i).toInt();
    if (k == "thr")                        cfg.thr = constrain(f, 0.05f, 0.95f);
    else if (k == "kp")                    cfg.kp = constrain(f, 0.0f, 1.0f);
    else if (k == "kd")                    cfg.kd = constrain(f, 0.0f, 2.0f);
    else if (k == "alpha")                 cfg.alpha = constrain(f, 0.05f, 1.0f);
    else if (k == "wzmax")                 cfg.wzMax = constrain(f, 0.1f, 5.0f);
    else if (k == "spacing")               cfg.spacing = constrain(f, 1.0f, 30.0f);
    else if (k == "minspan")               cfg.minSpan = constrain(n, 20, 2000);
    else if (k == "cyc"  || k == "cycle")  cfg.cycleMs = constrain(n, 5, 200);
    else if (k == "lost" || k == "lostms") cfg.lostMs = constrain(n, 100, 30000);
    else if (k == "samples" || k == "smp") cfg.samples = constrain(n, 1, 16);
    else if (k == "jmin")                  cfg.jMin = constrain(n, 3, N);
    else if (k == "jcyc")                  cfg.jCycles = constrain(n, 1, 20);
    else if (k == "white")                 { cfg.whiteHigh = n ? 1 : 0; flushFilters(); }
    else if (k == "log")                   cfg.logSerial = n ? 1 : 0;
  }
  srv->send(200, "text/plain", "OK");
}

void handleCmd() {
  String n = srv->arg("n"), m = "OK";
  if (n == "cal_start") { startCal(); m = "Calibration started (5 s)"; }
  else if (n == "cal_stop") { if (calActive) { finishCal(); m = "Calibration finished"; } else m = "Not calibrating"; }
  else if (n == "cal_clear") {
    for (int i = 0; i < N; i++) { cfg.calMin[i] = 4095; cfg.calMax[i] = 0; }
    cfg.calValid = 0; okMask = 0; strcpy(calMsg, "Calibration cleared"); flushFilters(); m = "Calibration cleared";
  }
  else if (n == "cal_cancel") { calActive = false; strcpy(calMsg, "Calibration cancelled"); flushFilters(); }
  else if (n == "save")  m = saveCfg() ? "Saved to flash (survives reboot)" : "Save FAILED";
  else if (n == "reset") {                     // tuning back to defaults, keep calibration
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
  char b[600];
  snprintf(b, sizeof(b),
    "{\"WiFi AP\":\"%s\",\"IP\":\"%s\",\"MAC\":\"%s\",\"Channel\":%d,\"Clients\":%d,"
    "\"Uptime s\":%lu,\"Free heap KB\":%u,\"Sensors\":%u,\"Calibration\":\"%s\","
    "\"Config source\":\"%s\",\"Control Hz\":%.1f}",
    apSsid, WiFi.softAPIP().toString().c_str(), WiFi.softAPmacAddress().c_str(), WiFi.channel(), WiFi.softAPgetStationNum(),
    (unsigned long)(millis() / 1000), (unsigned)(ESP.getFreeHeap() / 1024), N,
    cfg.calValid ? "VALID" : "NOT CALIBRATED",
    cfgFromFlash ? "flash (NVS)" : "defaults", rateHz);
  sendJSON(b);
}

void handleExport() {
  char b[1200];
  int p = snprintf(b, sizeof(b),
    "/* Generated by RUNNER4 Line Array Lab */\n"
    "#define LINE_NUM_SENSORS %u\n"
    "static const uint16_t LINE_CAL_MIN[%u] = {", N, N);
  for (int i = 0; i < N && p < (int)sizeof(b) - 8; i++) p += snprintf(b + p, sizeof(b) - p, "%u%s", cfg.calMin[i], (i < N - 1) ? ", " : "};\n");
  p += snprintf(b + p, sizeof(b) - p, "static const uint16_t LINE_CAL_MAX[%u] = {", N);
  for (int i = 0; i < N && p < (int)sizeof(b) - 8; i++) p += snprintf(b + p, sizeof(b) - p, "%u%s", cfg.calMax[i], (i < N - 1) ? ", " : "};\n");
  p += snprintf(b + p, sizeof(b) - p,
    "static const float LINE_KP = %.4ff;\n"
    "static const float LINE_KD = %.4ff;\n"
    "static const float LINE_THR = %.2ff;\n"
    "static const float LINE_SPACING_MM = %.1ff;\n",
    cfg.kp, cfg.kd, cfg.thr, cfg.spacing);
  srv->send(200, "text/plain", b);
}

static float savedAlphaLF = -1;

}  // namespace LineSensorInternal

// ---------------- PUBLIC API ----------------
inline bool lineBegin(WebServer &server, const char *ssid) {
  using namespace LineSensorInternal;
  srv = &server; apSsid = ssid;
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);          // full 0..3.3 V
  for (int i = 0; i < N; i++) { pinMode(PINS[i], INPUT); raw[i] = 0; norm[i] = 0; sOK[i] = false; }
  loadCfg();
  // Rebuild okMask from calibration validity
  okMask = 0;
  if (cfg.calValid) {
    for (int i = 0; i < N; i++) {
      uint16_t span = (cfg.calMax[i] > cfg.calMin[i]) ? (cfg.calMax[i] - cfg.calMin[i]) : 0;
      if (span >= cfg.minSpan) { sOK[i] = true; okMask |= (1 << i); }
    }
  }
  statusTxt = cfg.calValid ? "OK" : "NO CAL";
  Serial.printf("[LINE] 9-ch array, calibration %s, config %s\n", cfg.calValid ? "valid" : "MISSING (use /line)",
                cfgFromFlash ? "loaded from flash" : "defaults");
  srv->on("/line",        []() { LineSensorInternal::srv->send_P(200, "text/html", LINE_PAGE); });
  srv->on("/line/data",   LineSensorInternal::handleData);
  srv->on("/line/config", LineSensorInternal::handleConfig);
  srv->on("/line/set",    LineSensorInternal::handleSet);
  srv->on("/line/cmd",    LineSensorInternal::handleCmd);
  srv->on("/line/wifi",   LineSensorInternal::handleWifi);
  srv->on("/line/export", LineSensorInternal::handleExport);
  return cfg.calValid;
}

inline void linePoll() {
  using namespace LineSensorInternal;
  if (!srv) return;
  uint32_t now = millis();
  if (now - lastCycle < cfg.cycleMs) return;
  lastCycle = now;
  cycle(now);
  rateCnt++;
  if (now - lastRateMs >= 1000) { rateHz = rateCnt * 1000.0f / (now - lastRateMs); rateCnt = 0; lastRateMs = now; }
}

inline bool        lineCalibrated()   { return LineSensorInternal::cfg.calValid; }
inline bool        lineFound()        { return LineSensorInternal::found; }
inline float       lineErrorMm()      { return LineSensorInternal::errMm; }
inline float       lineWz()           { return LineSensorInternal::wz; }
inline uint8_t     lineActiveCount()  { return LineSensorInternal::activeCnt; }
inline bool        lineJunction()     { return LineSensorInternal::junction; }
inline const char *lineJunctionType() { return LineSensorInternal::jType; }
inline bool        lineLost()         { return LineSensorInternal::lost; }
inline bool        lineSafeStop()     { return LineSensorInternal::safeStop; }

inline void lineSetAlpha(float newAlpha) {
  using namespace LineSensorInternal;
  if (newAlpha < 0) {
    if (savedAlphaLF >= 0) { cfg.alpha = savedAlphaLF; savedAlphaLF = -1; }
  } else {
    if (savedAlphaLF < 0) savedAlphaLF = cfg.alpha;
    cfg.alpha = constrain(newAlpha, 0.05f, 1.0f);
  }
}
