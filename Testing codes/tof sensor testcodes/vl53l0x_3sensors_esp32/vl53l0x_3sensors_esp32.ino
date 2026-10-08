/*
  3x VL53L0X + ESP32 (Pololu "VL53L0X" library)
  Median filter + per-sensor calibration, all readings on one line.

  Wiring:
    VIN (all 3) -> 3V3        GND (all 3) -> GND
    SDA (all 3) -> GPIO21     SCL (all 3) -> GPIO22
    XSHUT FRONT (thick tipex line)  -> GPIO25
    XSHUT SIDE1 (narrow tipex line) -> GPIO26
    XSHUT SIDE2 (no marks)          -> GPIO27

  At startup each sensor is woken one at a time through XSHUT and given
  its own I2C address (0x30, 0x31, 0x32).

  Output example:   FRONT: 120.3  SIDE1: 98.7  SIDE2: 143.1 mm

  Serial commands (115200): send '1' (FRONT), '2' (SIDE1) or '3' (SIDE2)
  to capture 50 filtered raw readings (for collecting calibration points).

  
*/

#include <Wire.h>
#include <VL53L0X.h>

#define SDA_PIN 21
#define SCL_PIN 22

const uint8_t NS = 3;

// ---------- Sensor identity ----------
// FRONT = thick tipex line on the sensor
// SIDE1 = narrow tipex line on the sensor
// SIDE2 = no marks on the sensor
const char*   NAMES[NS]  = {"FRONT", "SIDE1", "SIDE2"};
const uint8_t XSHUT[NS]  = {25, 26, 27};
const uint8_t ADDR[NS]   = {0x30, 0x31, 0x32};

VL53L0X dev[NS];
bool    sensorOk[NS] = {false, false, false};

// ---------- Filter settings ----------
const uint8_t  MEDIAN_WINDOW    = 9;
const uint32_t TIMING_BUDGET_US = 200000;   // 200 ms (same as calibration)
const uint16_t MAX_VALID_MM     = 1200;

// false = single-shot readings (same mode used for calibration, safest)
// true  = continuous mode: sensors measure in parallel, ~3x faster cycle
const bool USE_CONTINUOUS = false;

const bool SHOW_RAW = false;   // true -> also print raw values in brackets

// ================= CALIBRATION TABLES (EDIT HERE) =================
// RAW_x must be strictly increasing; TRUE_x has the same number of values.

// ----- FRONT (thick tipex line) -----
const float RAW_F[]  = {52, 63, 74, 85, 94, 103, 115, 123, 129, 137, 145, 152, 159, 169, 175, 181, 186, 191};
const float TRUE_F[] = {30, 40, 50, 60, 70,  80,  90, 100, 110, 120, 130, 140, 150, 160, 170, 180, 190, 200};

// ----- SIDE1 (narrow tipex line) -----
const float RAW_S1[]  = {66, 72, 83, 92, 100, 110, 119, 129, 138, 148, 156, 166, 173, 181, 189, 198, 202, 205};
const float TRUE_S1[] = {30, 40, 50, 60,  70,  80,  90, 100, 110, 120, 130, 140, 150, 160, 170, 180, 190, 200};

// ----- SIDE2 (no marks) -----
const float RAW_S2[]  = {56, 67, 72, 82, 90, 100, 108, 114, 120, 127, 131, 134, 139, 140};
const float TRUE_S2[] = {30, 40, 50, 60, 70,  80,  90, 100, 110, 120, 130, 140, 150, 160};
// ==================================================================

const float*  RAWS[NS]  = {RAW_F, RAW_S1, RAW_S2};
const float*  TRUES[NS] = {TRUE_F, TRUE_S1, TRUE_S2};
const uint8_t NCAL[NS]  = {
  (uint8_t)(sizeof(RAW_F)  / sizeof(RAW_F[0])),
  (uint8_t)(sizeof(RAW_S1) / sizeof(RAW_S1[0])),
  (uint8_t)(sizeof(RAW_S2) / sizeof(RAW_S2[0]))
};
static_assert(sizeof(RAW_F)  == sizeof(TRUE_F),  "FRONT: RAW and TRUE counts differ");
static_assert(sizeof(RAW_S1) == sizeof(TRUE_S1), "SIDE1: RAW and TRUE counts differ");
static_assert(sizeof(RAW_S2) == sizeof(TRUE_S2), "SIDE2: RAW and TRUE counts differ");

// Piecewise-linear calibration for sensor s
float calibrate(uint8_t s, float raw) {
  const float* R = RAWS[s];
  const float* T = TRUES[s];
  uint8_t n = NCAL[s];

  uint8_t i;
  if (raw <= R[0])           i = 0;
  else if (raw >= R[n - 1])  i = n - 2;
  else {
    for (i = 0; i < n - 2; i++) {
      if (raw < R[i + 1]) break;
    }
  }
  float slope = (T[i + 1] - T[i]) / (R[i + 1] - R[i]);
  float v = T[i] + slope * (raw - R[i]);
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

uint16_t readMedianMM(uint8_t s) {
  uint16_t buf[MEDIAN_WINDOW];
  uint8_t count = 0;
  for (uint8_t i = 0; i < MEDIAN_WINDOW; i++) {
    uint16_t d = USE_CONTINUOUS ? dev[s].readRangeContinuousMillimeters()
                                : dev[s].readRangeSingleMillimeters();
    if (!dev[s].timeoutOccurred() && d < MAX_VALID_MM) buf[count++] = d;
  }
  if (count < (MEDIAN_WINDOW / 2 + 1)) return 0;
  return medianOf(buf, count);
}

void captureCalibrationPoint(uint8_t s, uint8_t N = 50) {
  if (!sensorOk[s]) { Serial.printf("Sensor %s not available\n", NAMES[s]); return; }
  float sum = 0, sumSq = 0;
  uint16_t mn = 65535, mx = 0;
  uint8_t valid = 0;
  Serial.printf("Capturing sensor %s... keep sensor & target still\n", NAMES[s]);
  for (uint8_t i = 0; i < N; i++) {
    uint16_t m = readMedianMM(s);
    if (m == 0) continue;
    sum += m; sumSq += (float)m * m;
    if (m < mn) mn = m;
    if (m > mx) mx = m;
    valid++;
  }
  if (!valid) { Serial.println(F("No valid readings!")); return; }
  float mean = sum / valid;
  float sd = sqrt(max(0.0f, sumSq / valid - mean * mean));
  Serial.printf("CAL %s -> raw mean: %.2f  min: %u  max: %u  std: %.2f  n: %u\n",
                NAMES[s], mean, mn, mx, sd, valid);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // 1) Hold all sensors in reset
  for (uint8_t i = 0; i < NS; i++) {
    pinMode(XSHUT[i], OUTPUT);
    digitalWrite(XSHUT[i], LOW);
  }
  delay(20);

  // 2) Wake one at a time and assign a unique address
  for (uint8_t i = 0; i < NS; i++) {
    pinMode(XSHUT[i], INPUT);      // release XSHUT (board pull-up wakes it)
    delay(20);

    dev[i].setTimeout(500);
    if (!dev[i].init()) {
      Serial.printf("Sensor %s: init FAILED (check wiring / XSHUT pin %u)\n", NAMES[i], XSHUT[i]);
      continue;
    }
    dev[i].setAddress(ADDR[i]);
    dev[i].setMeasurementTimingBudget(TIMING_BUDGET_US);
    if (USE_CONTINUOUS) dev[i].startContinuous();
    sensorOk[i] = true;
    Serial.printf("Sensor %s: OK at 0x%02X\n", NAMES[i], ADDR[i]);
  }
  Serial.println(F("Ready. Send 1/2/3 to capture a calibration point for FRONT/SIDE1/SIDE2."));
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c >= '1' && c <= '3') captureCalibrationPoint(c - '1');
  }

  for (uint8_t s = 0; s < NS; s++) {
    Serial.printf("%s: ", NAMES[s]);
    if (!sensorOk[s]) {
      Serial.print("N/A");
    } else {
      uint16_t raw = readMedianMM(s);
      if (raw == 0) Serial.print("---");
      else {
        Serial.printf("%.1f", calibrate(s, (float)raw));
        if (SHOW_RAW) Serial.printf(" (raw %u)", raw);
      }
    }
    Serial.print(s < NS - 1 ? "  " : " mm\n");
  }
}
