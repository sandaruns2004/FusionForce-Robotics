#pragma once
// =============================================================================
// tof_sensor.h  -  3x VL53L0X Time-of-Flight sensors  (NON-BLOCKING)
//
//   Three VL53L0X sensors share the main I2C bus (SDA=GPIO11, SCL=GPIO12).
//   Each sensor has its own XSHUT pin so we can enumerate them one at a time
//   and assign unique I2C addresses at boot.
//
//   XSHUT pins (active-LOW):
//     Front  -> GPIO 13   runtime address -> 0x30
//     Right  -> GPIO 14   runtime address -> 0x31
//     Left   -> GPIO 21   runtime address -> 0x32
//
//   Wire.begin(11, 12, 400000) must be called in setup() BEFORE tofBegin().
//
//   NON-BLOCKING design:
//     All sensors run in CONTINUOUS mode.  tofPoll() reads the latest
//     completed measurement from each sensor instantly (no waiting).
//     A per-sensor rolling-median filter smooths the output.
//
//   Web routes registered by tofBegin():
//     GET /tof          - live 3-sensor dashboard page
//     GET /tof/data     - JSON {front, right, left, front_raw, right_raw,
//                               left_raw, front_ok, right_ok, left_ok,
//                               front_addr, right_addr, left_addr}
//     GET /tof/diag     - JSON {i2c_scan[], sensors[]} for debugging
//
//   Call tofPoll() from loop() / serviceBackground() - non-blocking.
// =============================================================================
#include <Arduino.h>
#include <WebServer.h>
#include <Wire.h>
#include <VL53L0X.h>   // pololu/VL53L0X library

// ---- XSHUT pin assignments ---------------------------------------------------
#ifndef TOF_FRONT_XSHUT_PIN
#define TOF_FRONT_XSHUT_PIN   13
#endif
#ifndef TOF_RIGHT_XSHUT_PIN
#define TOF_RIGHT_XSHUT_PIN   14
#endif
#ifndef TOF_LEFT_XSHUT_PIN
#define TOF_LEFT_XSHUT_PIN    21
#endif

// ---- Runtime I2C addresses (assigned during tofBegin enumeration) -----------
#define TOF_FRONT_ADDR  0x30
#define TOF_RIGHT_ADDR  0x31
#define TOF_LEFT_ADDR   0x32

// ---- Filter / timing settings -----------------------------------------------
#define TOF_MAX_VALID_MM      1200
#define TOF_TIMING_BUDGET     33000   // us (33 ms ~= 30 Hz, fast mode)
#define TOF_MEDIAN_WIN        5       // rolling median window (odd)
#define TOF_POLL_INTERVAL_MS  40      // min ms between reads per sensor

// =============================================================================
// Public API
// =============================================================================
void  tofBegin(WebServer &server, const char *apSsid);
void  tofPoll();          // non-blocking; call from loop()

// Distance getters (calibrated median, mm)
float tofFront();
float tofRight();
float tofLeft();

// Raw last-reading getters
float tofFrontRaw();
float tofRightRaw();
float tofLeftRaw();

// Online flags
bool  tofFrontOnline();
bool  tofRightOnline();
bool  tofLeftOnline();

// Legacy single-sensor compat
bool  tofOnline();   // true if at least front is online

// =============================================================================
// IMPLEMENTATION (define TOF_IMPLEMENTATION in exactly one translation unit)
// =============================================================================
#ifdef TOF_IMPLEMENTATION

#include "../dashboard/tof_page.h"   // TOF_HTML PROGMEM string

// ---- Calibration table (piecewise-linear, common to all sensors) -----------
static const uint8_t  _TOF_N_CAL  = 18;
static const float _TOF_CAL_RAW[18]  = {52,63,74,85,94,103,115,123,129,137,145,152,159,169,175,181,186,191};
static const float _TOF_CAL_TRUE[18] = {30,40,50,60,70, 80, 90,100,110,120,130,140,150,160,170,180,190,200};

static float _tofPiecewise(float raw) {
    uint8_t i = 0;
    if      (raw <= _TOF_CAL_RAW[0])            i = 0;
    else if (raw >= _TOF_CAL_RAW[_TOF_N_CAL-1]) i = _TOF_N_CAL - 2;
    else { for (i = 0; i < _TOF_N_CAL-2; i++) if (raw < _TOF_CAL_RAW[i+1]) break; }
    float slope = (_TOF_CAL_TRUE[i+1] - _TOF_CAL_TRUE[i]) / (_TOF_CAL_RAW[i+1] - _TOF_CAL_RAW[i]);
    float v = _TOF_CAL_TRUE[i] + slope * (raw - _TOF_CAL_RAW[i]);
    return v < 0 ? 0 : v;
}

// ---- Per-sensor state -------------------------------------------------------
struct TofSensor {
    VL53L0X   dev;
    bool      ok        = false;
    uint8_t   xshutPin  = 0;
    uint8_t   addr      = 0;
    const char *name    = "";
    float     dist      = 0;   // calibrated median mm
    float     raw       = 0;   // latest raw mm
    uint32_t  lastMs    = 0;
    uint16_t  medBuf[TOF_MEDIAN_WIN] = {};
    uint8_t   medIdx    = 0;
    uint8_t   medFill   = 0;

    void medPush(uint16_t v) {
        medBuf[medIdx] = v;
        medIdx = (medIdx + 1) % TOF_MEDIAN_WIN;
        if (medFill < TOF_MEDIAN_WIN) medFill++;
    }
    uint16_t medResult() const {
        uint8_t n = (medFill < TOF_MEDIAN_WIN) ? medFill : TOF_MEDIAN_WIN;
        if (n == 0) return 0;
        uint16_t tmp[TOF_MEDIAN_WIN];
        for (uint8_t i = 0; i < n; i++) tmp[i] = medBuf[i];
        for (uint8_t i = 1; i < n; i++) {
            uint16_t key = tmp[i]; int8_t j = (int8_t)i - 1;
            while (j >= 0 && tmp[j] > key) { tmp[j+1] = tmp[j]; j--; }
            tmp[j+1] = key;
        }
        return tmp[n / 2];
    }
};

static TofSensor _tof[3];   // 0=Front  1=Right  2=Left

// ---- Sensor init helper -----------------------------------------------------
static void _tofInitAll() {
    // Pull all sensors into reset via XSHUT (LOW = reset)
    for (int i = 0; i < 3; i++) {
        pinMode(_tof[i].xshutPin, OUTPUT);
        digitalWrite(_tof[i].xshutPin, LOW);
    }
    delay(10);

    // Bring sensors up one at a time and assign unique addresses
    for (int i = 0; i < 3; i++) {
        digitalWrite(_tof[i].xshutPin, HIGH);   // release this sensor from reset
        delay(10);                               // wait for boot (datasheet: 1.2 ms min)

        _tof[i].dev.setTimeout(300);
        if (_tof[i].dev.init()) {
            _tof[i].dev.setAddress(_tof[i].addr);
            _tof[i].dev.setMeasurementTimingBudget(TOF_TIMING_BUDGET);
            _tof[i].dev.startContinuous();
            _tof[i].ok = true;
            Serial.printf("[TOF] %-6s VL53L0X found at 0x29, relocated -> 0x%02X\n",
                          _tof[i].name, _tof[i].addr);
        } else {
            _tof[i].ok = false;
            Serial.printf("[TOF] %-6s VL53L0X NOT FOUND (xshut=GPIO%u, target=0x%02X)\n",
                          _tof[i].name, _tof[i].xshutPin, _tof[i].addr);
        }
    }
}

// ---- Public getters ---------------------------------------------------------
float tofFront()       { return _tof[0].dist; }
float tofRight()       { return _tof[1].dist; }
float tofLeft()        { return _tof[2].dist; }
float tofFrontRaw()    { return _tof[0].raw;  }
float tofRightRaw()    { return _tof[1].raw;  }
float tofLeftRaw()     { return _tof[2].raw;  }
bool  tofFrontOnline() { return _tof[0].ok;   }
bool  tofRightOnline() { return _tof[1].ok;   }
bool  tofLeftOnline()  { return _tof[2].ok;   }
bool  tofOnline()      { return _tof[0].ok;   }   // legacy compat

// ---- tofBegin ---------------------------------------------------------------
void tofBegin(WebServer &server, const char *apSsid) {
    // Populate sensor descriptors (must be before _tofInitAll)
    _tof[0].xshutPin = TOF_FRONT_XSHUT_PIN; _tof[0].addr = TOF_FRONT_ADDR; _tof[0].name = "Front";
    _tof[1].xshutPin = TOF_RIGHT_XSHUT_PIN; _tof[1].addr = TOF_RIGHT_ADDR; _tof[1].name = "Right";
    _tof[2].xshutPin = TOF_LEFT_XSHUT_PIN;  _tof[2].addr = TOF_LEFT_ADDR;  _tof[2].name = "Left";

    _tofInitAll();

    // --- /tof  dashboard page ---
    server.on("/tof", [&server]() {
        server.send_P(200, "text/html", TOF_HTML);
    });

    // --- /tof/data  JSON live data ---
    server.on("/tof/data", [&server]() {
        server.sendHeader("Cache-Control", "no-store");
        server.sendHeader("Access-Control-Allow-Origin", "*");
        char b[400];
        snprintf(b, sizeof(b),
            "{\"front_ok\":%s,\"right_ok\":%s,\"left_ok\":%s,"
            "\"front\":%.1f,\"right\":%.1f,\"left\":%.1f,"
            "\"front_raw\":%.1f,\"right_raw\":%.1f,\"left_raw\":%.1f,"
            "\"front_addr\":\"0x%02X\",\"right_addr\":\"0x%02X\",\"left_addr\":\"0x%02X\"}",
            _tof[0].ok?"true":"false", _tof[1].ok?"true":"false", _tof[2].ok?"true":"false",
            _tof[0].dist, _tof[1].dist, _tof[2].dist,
            _tof[0].raw,  _tof[1].raw,  _tof[2].raw,
            (unsigned)_tof[0].addr, (unsigned)_tof[1].addr, (unsigned)_tof[2].addr);
        server.send(200, "application/json", b);
    });

    // --- /tof/diag  I2C bus scan + per-sensor detail for debugging ---
    server.on("/tof/diag", [&server]() {
        server.sendHeader("Cache-Control", "no-store");
        // Live I2C bus scan
        String j = "{\"i2c_scan\":[";
        bool first = true;
        for (uint8_t addr = 1; addr < 127; addr++) {
            Wire.beginTransmission(addr);
            if (Wire.endTransmission() == 0) {
                if (!first) j += ",";
                char hex[8]; snprintf(hex, sizeof(hex), "\"0x%02X\"", (unsigned)addr);
                j += hex;
                first = false;
            }
        }
        j += "],\"sensors\":[";
        const char* snames[] = {"front","right","left"};
        for (int i = 0; i < 3; i++) {
            if (i) j += ",";
            char entry[256];
            snprintf(entry, sizeof(entry),
                "{\"name\":\"%s\",\"ok\":%s,\"xshut_gpio\":%u,\"target_addr\":\"0x%02X\","
                "\"dist\":%.1f,\"raw\":%.1f}",
                snames[i], _tof[i].ok?"true":"false",
                (unsigned)_tof[i].xshutPin, (unsigned)_tof[i].addr,
                _tof[i].dist, _tof[i].raw);
            j += entry;
        }
        j += "]}";
        server.send(200, "application/json", j);
    });

    int found = 0;
    for (int i = 0; i < 3; i++) if (_tof[i].ok) found++;
    Serial.printf("[TOF] Init complete: %d/3 sensors online -> /tof  /tof/data  /tof/diag\n", found);
}

// ---- tofPoll ---------------------------------------------------------------
void tofPoll() {
    uint32_t now = millis();
    for (int i = 0; i < 3; i++) {
        if (!_tof[i].ok) continue;
        if (now - _tof[i].lastMs < TOF_POLL_INTERVAL_MS) continue;
        _tof[i].lastMs = now;

        uint16_t r = _tof[i].dev.readRangeContinuousMillimeters();
        if (_tof[i].dev.timeoutOccurred() || r >= TOF_MAX_VALID_MM) continue;

        _tof[i].medPush(r);
        _tof[i].raw  = (float)r;
        _tof[i].dist = _tofPiecewise((float)_tof[i].medResult());
    }
}

#endif // TOF_IMPLEMENTATION