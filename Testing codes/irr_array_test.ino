// ESP32 9x TCRT5000 analog line array (white line on black floor)
// S1 (index 0) = far left ... S9 (index 8) = far right, 10 mm spacing
#define NUM_SENSORS   9
#define RAW_TEST      false    // true: only print raw ADC values (step 1). false: full line following
#define WHITE_IS_HIGH true   // if white is high keep true , else false

const int sensorPins[9] = {35, 34, 33, 32, 27, 26, 25, 14, 13};
const float SENSOR_SPACING_MM = 10.0;
const float LINE_THRESH       = 0.5;    // normalised 0..1, above = "on white line"
const int   CYCLE_MS          = 20;     // 50 Hz control loop
const uint32_t LOST_LIMIT_MS  = 3000;   // your doc's failsafe

// PD gains for the angular velocity command Wz (tune on the robot)
float Kp = 0.05;   // per mm of error
float Kd = 0.20;

int calMin[NUM_SENSORS], calMax[NUM_SENSORS];

int readRaw(int i) {                     // average 8 reads to reduce ESP32 ADC noise
  long sum = 0;
  for (int k = 0; k < 8; k++) sum += analogRead(sensorPins[i]);
  return sum / 8;
}

// Sweep the array over black AND white for 5 s at power-up
void calibrate() {
  for (int i = 0; i < NUM_SENSORS; i++) { calMin[i] = 4095; calMax[i] = 0; }
  Serial.println("CALIBRATING 5 s: slide array across black floor and white line...");
  uint32_t start = millis();
  while (millis() - start < 5000) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      int r = readRaw(i);
      if (r < calMin[i]) calMin[i] = r;
      if (r > calMax[i]) calMax[i] = r;
    }
    delay(5);
  }
  Serial.println("Calibration done.");
}

// Returns 0..1 where 1 = white line under sensor
float readNorm(int i) {
  int span = calMax[i] - calMin[i];
  if (span < 100) return 0;              // sensor never changed: dead or calibration bad
  float n = (float)(readRaw(i) - calMin[i]) / span;
  n = constrain(n, 0.0f, 1.0f);
  return WHITE_IS_HIGH ? n : 1.0f - n;
}

// Weighted centroid. Returns false if line lost.
bool readLine(float &errorMm, int &activeCount) {
  float sumW = 0, sumV = 0;
  activeCount = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    float v = readNorm(i);
    if (v < LINE_THRESH) v = 0; else activeCount++;
    sumW += v;
    sumV += i * v;
  }
  if (sumW < 0.01f) return false;
  float centroid = sumV / sumW;                        // 0.0 .. 8.0
  errorMm = (centroid - 4.0f) * SENSOR_SPACING_MM;     // -40 mm (left) .. +40 mm (right)
  return true;
}

// Junction: >= 6 sensors active for >= 3 consecutive cycles
bool junctionConfirmed(int activeCount) {
  static uint8_t consecutive = 0;
  if (activeCount >= 6) consecutive++; else consecutive = 0;
  return consecutive >= 3;
}

// Replace with your gait controller: Wz < 0 turn left, > 0 turn right
void steer(float wz) {
  Serial.print("Wz: ");
  Serial.print(wz);
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);       // full 0..3.3 V range
  if (!RAW_TEST) calibrate();
}

void loop() {
  static uint32_t lostSince = 0;
  static float lastError = 0;

  if (RAW_TEST) {                        // step 1: look at raw values on black vs white
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(readRaw(i));
      Serial.print("\t");
    }
    Serial.println();
    delay(200);
    return;
  }

  float errorMm; int active;
  bool found = readLine(errorMm, active);

  if (found) {
    lostSince = 0;
    float wz = Kp * errorMm + Kd * (errorMm - lastError);
    lastError = errorMm;
    Serial.print("err(mm): "); Serial.print(errorMm);
    Serial.print(" active: "); Serial.print(active);
    Serial.print(" | ");
    steer(wz);
    if (junctionConfirmed(active)) Serial.print("  <<< JUNCTION");
    Serial.println();
  } else {
    if (lostSince == 0) lostSince = millis();
    Serial.print("LINE LOST, last side: ");
    Serial.println(lastError < 0 ? "left" : "right");
    if (millis() - lostSince > LOST_LIMIT_MS) {
      Serial.println("SAFE STOP");        // replace with your safe-stop / search routine
    }
  }
  delay(CYCLE_MS);
}
