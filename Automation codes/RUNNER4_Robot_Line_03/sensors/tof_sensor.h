#pragma once
// =============================================================================
// tof_sensor.h  -  Single VL53L0X Time-of-Flight sensor  (NON-BLOCKING)
//   One VL53L0X on the main I2C bus; color sensor uses a separate bus.
//
//   I2C bus: SDA=GPIO11, SCL=GPIO12 (shared with PCA9685 only).
//   Wire.begin(11, 12, 400000) must be called in setup() BEFORE tofBegin().
//
//   IMPORTANT — non-blocking design:
//     Sensor runs in CONTINUOUS mode.  tofPoll() calls
//     readRangeContinuousMillimeters() which returns the last completed
//     result instantly — it never blocks.  The rolling median is built
//     across multiple poll() calls rather than inside one blocking loop.
//
//   Web routes registered by tofBegin():
//     GET /tof       - live dashboard page
//     GET /tof/data  - JSON {"front":mm, "front_raw":mm, "ok":bool}
//
//   Call tofPoll() from loop() / serviceBackground() - non-blocking.
// =============================================================================
#include <Arduino.h>
#include <WebServer.h>
#include <Wire.h>
#include <VL53L0X.h>   // pololu/VL53L0X library

// ---- Filter / timing settings -----------------------------------------------
#define TOF_MAX_VALID_MM      1200
// Timing budget only affects sensor accuracy; in continuous mode the MCU
// never waits for it — the sensor measures autonomously.
#define TOF_TIMING_BUDGET     33000   // us  (33 ms ~= 30 Hz, fast mode)
#define TOF_MEDIAN_WIN        5       // rolling median window size (odd)
#define TOF_POLL_INTERVAL_MS  40      // min ms between poll() reads (guard)

// VL53L0X is moved from boot address 0x29 to this runtime address.
// MUST match what is used in tofBegin() setAddress() call.
#define TOF_I2C_ADDR          0x30
#ifndef TOF_FRONT_XSHUT_PIN
#define TOF_FRONT_XSHUT_PIN   13      // Front VL53L0X XSHUT hardware reset pin
#endif


// =============================================================================
// Public API
// =============================================================================
void  tofBegin(WebServer &server, const char *apSsid);
void  tofPoll();       // non-blocking; call from loop() and serviceBackground()
float tofFront();      // calibrated distance in mm
float tofFrontRaw();   // raw sensor reading in mm (latest single reading)
bool  tofOnline();

// =============================================================================
// IMPLEMENTATION (define TOF_IMPLEMENTATION in exactly one .ino TU)
// =============================================================================
#ifdef TOF_IMPLEMENTATION

#include "../dashboard/tof_page.h"   // TOF_HTML PROGMEM string

static VL53L0X _tofSensor;
static bool    _tofOk = false;

static float _front_mm  = 0;
static float _front_raw = 0;

// ---- Calibration table (piecewise-linear) -----------------------------------
// From vl53l0x_calibrated2_esp32_3cm_to_20cm.ino (front sensor, 30-200 mm)
static const uint8_t N_CAL = 18;
static const float CAL_RAW[18]  = {52,63,74,85,94,103,115,123,129,137,145,152,159,169,175,181,186,191};
static const float CAL_TRUE[18] = {30,40,50,60,70, 80, 90,100,110,120,130,140,150,160,170,180,190,200};

static float _piecewise(float raw) {
    uint8_t i;
    if      (raw <= CAL_RAW[0])        i = 0;
    else if (raw >= CAL_RAW[N_CAL-1])  i = N_CAL - 2;
    else { for (i = 0; i < N_CAL-2; i++) if (raw < CAL_RAW[i+1]) break; }
    float slope = (CAL_TRUE[i+1] - CAL_TRUE[i]) / (CAL_RAW[i+1] - CAL_RAW[i]);
    float v = CAL_TRUE[i] + slope * (raw - CAL_RAW[i]);
    return v < 0 ? 0 : v;
}

// ---- Rolling median filter (non-blocking — accumulates across poll calls) ---
static uint16_t _medBuf[TOF_MEDIAN_WIN];
static uint8_t  _medIdx  = 0;
static uint8_t  _medFill = 0;  // how many slots have been filled

static void _medPush(uint16_t v) {
    _medBuf[_medIdx] = v;
    _medIdx = (_medIdx + 1) % TOF_MEDIAN_WIN;
    if (_medFill < TOF_MEDIAN_WIN) _medFill++;
}

static uint16_t _medianResult() {
    uint8_t n = (_medFill < TOF_MEDIAN_WIN) ? _medFill : TOF_MEDIAN_WIN;
    if (n == 0) return 0;
    uint16_t tmp[TOF_MEDIAN_WIN];
    for (uint8_t i = 0; i < n; i++) tmp[i] = _medBuf[i];
    // insertion sort
    for (uint8_t i = 1; i < n; i++) {
        uint16_t key = tmp[i]; int8_t j = (int8_t)i - 1;
        while (j >= 0 && tmp[j] > key) { tmp[j+1] = tmp[j]; j--; }
        tmp[j+1] = key;
    }
    return tmp[n / 2];
}

// ---- Polling state ----------------------------------------------------------
static uint32_t _tofLastMs = 0;

// ---- Public functions -------------------------------------------------------
float tofFront()    { return _front_mm;  }
float tofFrontRaw() { return _front_raw; }
bool  tofOnline()   { return _tofOk;     }

void tofBegin(WebServer &server, const char *apSsid) {
    // Wire bus (GPIO 11 SDA, GPIO 12 SCL) already started in setup() - do NOT
    // call Wire.begin() here again.
    //
    // Color sensor is on a physically separate I2C controller, so the VL53L0X
    // can initialize at 0x29 without bus contention.
    _tofSensor.setTimeout(500);
    if (_tofSensor.init()) {
        // Move VL53L0X to its runtime address on the main bus.
        _tofSensor.setAddress(TOF_I2C_ADDR);
        // Fast timing budget — sensor measures autonomously in continuous mode.
        _tofSensor.setMeasurementTimingBudget(TOF_TIMING_BUDGET);
        // Continuous mode: sensor produces readings on its own.
        // tofPoll() just reads the latest completed result — no blocking.
        _tofSensor.startContinuous();
        _tofOk = true;
        Serial.printf("[TOF] VL53L0X detected at 0x29, relocated to 0x%02X - continuous mode ON\n", TOF_I2C_ADDR);
    } else {
        _tofOk = false;
        Serial.println("[TOF] VL53L0X NOT FOUND at 0x29");
        Serial.println("[TOF] Check: SDA=GPIO11, SCL=GPIO12, VIN=3.3V, 4.7k pullups");
    }


    server.on("/tof", [&server]() {
        server.send_P(200, "text/html", TOF_HTML);
    });

    server.on("/tof/data", [&server]() {
        server.sendHeader("Cache-Control", "no-store");
        server.sendHeader("Access-Control-Allow-Origin", "*");
        String j = "{";
        j += "\"ok\":"        + String(_tofOk ? "true" : "false") + ",";
        j += "\"front\":"     + String(_front_mm,  1) + ",";
        j += "\"front_raw\":" + String(_front_raw, 1);
        j += "}";
        server.send(200, "application/json", j);
    });

    Serial.printf("[TOF] %s -> /tof  /tof/data\n", _tofOk ? "READY" : "OFFLINE");
}

// ---------------------------------------------------------------------------
// tofPoll() — truly non-blocking.
//
// VL53L0X is in continuous mode, so the sensor keeps measuring on its own.
// readRangeContinuousMillimeters() returns the LAST completed result instantly
// (no wait).  We guard with TOF_POLL_INTERVAL_MS so we don't spam I2C.
// Each call adds one sample to the rolling median ring buffer.
// ---------------------------------------------------------------------------
void tofPoll() {
    if (!_tofOk) return;
    uint32_t now = millis();
    if (now - _tofLastMs < TOF_POLL_INTERVAL_MS) return;
    _tofLastMs = now;

    // readRangeContinuousMillimeters() does NOT block in continuous mode.
    uint16_t raw = _tofSensor.readRangeContinuousMillimeters();

    if (_tofSensor.timeoutOccurred() || raw >= TOF_MAX_VALID_MM) {
        // Bad reading — keep previous filtered output, don't update buffer
        return;
    }

    _medPush(raw);
    _front_raw = (float)raw;                              // latest raw reading
    _front_mm  = _piecewise((float)_medianResult());      // calibrated median
}

#endif // TOF_IMPLEMENTATION
