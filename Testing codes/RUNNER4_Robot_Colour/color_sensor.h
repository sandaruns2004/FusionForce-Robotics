#pragma once
// =============================================================================
// color_sensor.h - TCS34725 colour module (separate module, own web routes)
//   Page : /color          Data : /color/data      Config : /color/config
//   Set  : /color/set      Cmd  : /color/cmd       Health : /color/wifi
//   Export STM32 code : /color/export
// The I2C bus (Wire) must already be started by the main sketch.
// =============================================================================
#include <Arduino.h>
#include <WebServer.h>

#ifndef COLOR_LED_PIN
#define COLOR_LED_PIN 4          // GPIO driving the TCS34725 LED pin (HIGH = on)
#endif

enum ColorId : uint8_t { COLOR_ID_UNKNOWN = 0, COLOR_ID_RED, COLOR_ID_GREEN, COLOR_ID_BLUE };

// Register routes on the shared web server, load saved calibration, init sensor.
// Returns true if the TCS34725 answered on the I2C bus.
bool colorBegin(WebServer &server, const char *apSsid, int sdaPin, int sclPin);

// Call as often as possible (loop() and inside long motion waits). Non-blocking.
void colorPoll();

// Use these from robot logic (e.g. gait / junction decisions)
bool        colorSensorOnline();
uint8_t     colorConfirmed();     // stable voted colour, 0 if none yet
uint8_t     colorStoredBall();    // ball colour stored in flash, 0 if none
const char *colorName(uint8_t id);
