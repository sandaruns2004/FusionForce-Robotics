/*
thick line tipex (mark in sensor)
*/

#include <Wire.h>
#include <VL53L0X.h>

#define SDA_PIN 21
#define SCL_PIN 22

VL53L0X sensor;

// ---------- Filter settings ----------
const uint8_t  MEDIAN_WINDOW    = 9;
const uint32_t TIMING_BUDGET_US = 200000;
const uint16_t MAX_VALID_MM     = 1200;

// ---------- Calibration table ----------
// RAW_MM must be strictly increasing
const uint8_t N_CAL = 18;
const float RAW_MM[N_CAL]  = {52,63,74,85, 94, 103, 115, 123, 129, 137, 145, 152, 159, 169, 175, 181, 186, 191};
const float TRUE_MM[N_CAL] = {30,40,50,60, 70,  80,  90, 100, 110, 120, 130, 140, 150, 160, 170, 180, 190, 200};

// Calibration method: piecewise-linear interpolation between the points above.
// (A polynomial fit left up to ~7 mm error on this data, so the table is used.)
float calibrate(float raw) {
  // Pick the segment; extrapolate using the first/last segment outside range
  uint8_t i;
  if (raw <= RAW_MM[0])              i = 0;
  else if (raw >= RAW_MM[N_CAL - 1]) i = N_CAL - 2;
  else {
    for (i = 0; i < N_CAL - 2; i++) {
      if (raw < RAW_MM[i + 1]) break;
    }
  }
  float slope = (TRUE_MM[i + 1] - TRUE_MM[i]) / (RAW_MM[i + 1] - RAW_MM[i]);
  float v = TRUE_MM[i] + slope * (raw - RAW_MM[i]);
  return v < 0 ? 0 : v;
}

// ---------- Median filter ----------
uint16_t medianOf(uint16_t *arr, uint8_t n) {
  uint16_t tmp[MEDIAN_WINDOW];
  for (uint8_t i = 0; i < n; i++) tmp[i] = arr[i];
  for (uint8_t i = 1; i < n; i++) {
    uint16_t key = tmp[i];
    int8_t j = i - 1;
    while (j >= 0 && tmp[j] > key) { tmp[j + 1] = tmp[j]; j--; }
    tmp[j + 1] = key;
  }
  return tmp[n / 2];
}

uint16_t readMedianMM() {
  uint16_t buf[MEDIAN_WINDOW];
  uint8_t count = 0;
  for (uint8_t i = 0; i < MEDIAN_WINDOW; i++) {
    uint16_t d = sensor.readRangeSingleMillimeters();
    if (!sensor.timeoutOccurred() && d < MAX_VALID_MM) buf[count++] = d;
  }
  if (count < (MEDIAN_WINDOW / 2 + 1)) return 0;
  return medianOf(buf, count);
}

void captureCalibrationPoint(uint8_t N = 50) {
  float sum = 0, sumSq = 0;
  uint16_t mn = 65535, mx = 0;
  uint8_t valid = 0;
  Serial.println(F("Capturing... keep sensor & target still"));
  for (uint8_t i = 0; i < N; i++) {
    uint16_t m = readMedianMM();
    if (m == 0) continue;
    sum += m; sumSq += (float)m * m;
    if (m < mn) mn = m;
    if (m > mx) mx = m;
    valid++;
  }
  if (!valid) { Serial.println(F("No valid readings!")); return; }
  float mean = sum / valid;
  float sd = sqrt(max(0.0f, sumSq / valid - mean * mean));
  Serial.printf("CAL -> raw mean: %.2f  min: %u  max: %u  std: %.2f  n: %u\n",
                mean, mn, mx, sd, valid);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  delay(50);

  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println(F("Failed to detect VL53L0X! Check wiring."));
    while (1) delay(10);
  }
  sensor.setMeasurementTimingBudget(TIMING_BUDGET_US);
  Serial.println(F("Ready. Output: raw(mm), calibrated(mm). 'c' = capture cal point."));
}

void loop() {
  if (Serial.available() && Serial.read() == 'c') captureCalibrationPoint();

  uint16_t raw = readMedianMM();
  if (raw == 0) {
    Serial.println(F("Out of range / invalid"));
    return;
  }
  float actual = calibrate((float)raw);
  Serial.printf("Raw: %u mm   Calibrated: %.1f mm\n", raw, actual);
}
