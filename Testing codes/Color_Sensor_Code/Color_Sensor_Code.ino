#include <Wire.h>
#include "Adafruit_TCS34725.h"

// ==========================================
// PIN DEFINITIONS
// ==========================================
#define SDA_PIN 8
#define SCL_PIN 10
#define LED_PIN 4

// 50ms integration time (20Hz max rate), 4X gain
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

enum ColorID { COLOR_UNKNOWN, COLOR_RED, COLOR_GREEN, COLOR_BLUE };

void setup() {
  Serial.begin(115200);
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // Keep LED off initially

  Wire.begin(SDA_PIN, SCL_PIN);
  if (tcs.begin()) {
    Serial.println("Found TCS34725 sensor!");
  } else {
    Serial.println("No TCS34725 found... check your wiring.");
    while (1); 
  }
}

ColorID classifyColor(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  if (c == 0) return COLOR_UNKNOWN;

  float r_n = (float)r / c;
  float g_n = (float)g / c;
  float b_n = (float)b / c;

  if (r_n > 0.40 && r_n > g_n * 1.4 && r_n > b_n * 1.4) return COLOR_RED;
  if (g_n > 0.35 && g_n > r_n * 1.2 && g_n > b_n * 1.2) return COLOR_GREEN;
  if (b_n > 0.30 && b_n > r_n * 1.2 && b_n > g_n * 1.2) return COLOR_BLUE;

  return COLOR_UNKNOWN;
}

String getColorName(ColorID id) {
  if(id == COLOR_RED) return "RED";
  if(id == COLOR_GREEN) return "GREEN";
  if(id == COLOR_BLUE) return "BLUE";
  return "UNKNOWN";
}

void loop() {
  uint16_t r, g, b, c;

  digitalWrite(LED_PIN, HIGH);
  delay(60); // 50ms integration + 10ms safety margin
  tcs.getRawData(&r, &g, &b, &c);
  digitalWrite(LED_PIN, LOW); // Save power and prevent optical cross-talk
  
  ColorID color = classifyColor(r, g, b, c);
  Serial.print("Detected Color: ["); Serial.print(getColorName(color)); Serial.println("]");

  delay(500); // Main loop delay
}