#pragma once
// =============================================================================
// line_sensor.h - 9x TCRT5000 analog line array module for RUNNER4 (ESP32-S3)
//   Page : /line          Data : /line/data      Config : /line/config
//   Set  : /line/set      Cmd  : /line/cmd       Health : /line/wifi
//   Export constants : /line/export
// Same architecture as color_sensor.*: own routes, own flash config (NVS),
// non-blocking poll(), anonymous-namespace internals.
//
// PIN RULES (ESP32-S3):
//   * WiFi AP is ON  -> ADC2 (GPIO 11-20) is unusable. Use ADC1 only = GPIO 1..10.
//   * GPIO 3 is a strapping pin -> skipped.
//   * Sensor modules MUST be powered from 3.3 V so the analog output never
//     exceeds 3.3 V (ESP32-S3 pins are not 5 V tolerant).
// Order below: S1 (far LEFT) ... S9 (far RIGHT).
// =============================================================================
#include <Arduino.h>
#include <WebServer.h>

#define LINE_NUM_SENSORS 9
#define LINE_PIN_LIST {1, 2, 4, 5, 6, 7, 8, 9, 10}   // S1..S9, all ADC1

// Register routes on the shared web server, load saved calibration, init ADC.
// Returns true if a valid calibration was loaded from flash.
bool lineBegin(WebServer &server, const char *apSsid);

// Call as often as possible (loop() and inside long motion waits). Non-blocking,
// internally rate-limited to the configured control cycle (default 20 ms).
void linePoll();

// ---- Use these from robot logic (gait / steering decisions) ----
bool    lineCalibrated();      // valid calibration present
bool    lineFound();           // line currently seen
float   lineErrorMm();         // -40 (left) .. +40 (right) mm for 10 mm spacing
float   lineWz();              // PD output: <0 turn left, >0 turn right
uint8_t lineActiveCount();     // sensors above threshold
bool    lineJunction();        // >= jMin sensors for >= jCycles cycles
const char *lineJunctionType();// "NONE","LEFT","RIGHT","T/CROSS","WIDE"
bool    lineLost();            // line not seen right now
bool    lineSafeStop();        // lost longer than the failsafe limit

// Override EMA alpha for line-follow mode (pass -1 to restore previous value)
void    lineSetAlpha(float newAlpha);
