// =============================================================================
// RUNNER4_MinTest v2.5 — All-in-One Minimal Diagnostic Dashboard
//
// Sections in one page:
//   1. MOTION   — Walk / Closed-Loop Gyro Turns (90°, 180°, custom deg) / Step Turns
//   2. ARM      — Lift + Gripper (angle sliders, Open/Close/Up/Down/Pick/Place)
//   3. LINE     — 9x TCRT5000 Pro Indication (Track position, Centroid, Error mm,
//                 normalized bars with threshold, active yellow highlight)
//                 + Full Calibration (5s Auto Sweep, Floor/Line Snapshots,
//                   Live Threshold slider, Polarity toggle, NVS flash save)
//                 + LINE FOLLOWING WALK (Closed-loop PD steering with inverted sign
//                   & optional sensor order swap)
//   4. TOF      — 3x VL53L0X live distance (Front/Right/Left)
//   5. GYRO     — MPU6050 live Yaw heading, Gz rate, Zero Yaw, I2C address
//                 + Deadband drift suppression (eliminates rapid stationary drift)
//                 + On-demand gyro bias calibration
//
// WiFi AP: RUNNER4-Test / runner4robot  →  http://192.168.4.1
//
// CHANNEL MAP:
//   FL: HIP=0  FEM=1  TIB=15   FR: HIP=3  FEM=4  TIB=5
//   BL: HIP=10 FEM=11 TIB=14   BR: HIP=6  FEM=8  TIB=9
//   ARM_LIFT = GPIO 47 (LEDC)   ARM_GRIP = GPIO 44 (LEDC)
//   LINE: S1-S9 = GPIO 1,2,4,5,6,7,8,9,10
//   TOF XSHUT: Front=GPIO13  Right=GPIO14  Left=GPIO21
//   MPU6050: SDA=GPIO11  SCL=GPIO12  Addr=0x68 / 0x69
//
// Libraries needed: Adafruit PWM Servo Driver, pololu/VL53L0X
// =============================================================================

#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <math.h>
#include <string.h>

// ─── Hardware constants ───────────────────────────────────────────────────────
#define I2C_SDA     11
#define I2C_SCL     12
#define I2C_HZ      400000
#define SERVOMIN    150
#define SERVOMAX    600
#define SERVO_FREQ  50
#define AP_SSID     "RUNNER4-Test"
#define AP_PASS     "runner4robot"

// ARM (direct LEDC, 50 Hz, 14-bit)
#define ARM_LIFT_PIN   47
#define ARM_GRIP_PIN   44
#define ARM_LEDC_FREQ  50
#define ARM_LEDC_RES   14
#define ARM_US_TO_DUTY(us) ((uint32_t)(((float)(us)/20000.0f)*((1<<ARM_LEDC_RES)-1)))

// Line array — ADC1 only (WiFi blocks ADC2 on ESP32-S3)
#define LINE_N 9
const uint8_t LINE_PINS[LINE_N] = {1, 2, 4, 5, 6, 7, 8, 9, 10};

// ToF XSHUT pins + runtime I2C addresses
#define TOF_FRONT_XSHUT 13
#define TOF_RIGHT_XSHUT 14
#define TOF_LEFT_XSHUT  21
#define TOF_FRONT_ADDR  0x30
#define TOF_RIGHT_ADDR  0x31
#define TOF_LEFT_ADDR   0x32

// MPU6050 constants
#define MPU_ADDR_DEF    0x68
#define MPU_GYRO_SENS   65.5f   // LSB per °/s at ±500°/s
#define MPU_TOL_DEG     3.0f
#define MPU_DEADBAND    0.25f   // °/s deadband threshold to eliminate stationary drift

// ─── Servo channel map ────────────────────────────────────────────────────────
// legChannels[leg][0=hip, 1=femur, 2=tibia]; leg: 0=FL 1=FR 2=BL 3=BR
const uint8_t CH[4][3] = {
    {0, 1, 15},   // FL
    {3, 4, 5},    // FR
    {10, 11, 14}, // BL
    {6, 8, 9}     // BR
};

// ─── Poses ───────────────────────────────────────────────────────────────────
const int HOME_POSE[4][3] = {
    {45, 60, 139}, {94, 80, 25}, {94, 70, 5}, {45, 80, 140}
};
const int STAND_POSE[4][3] = {
    {34, 2, 83}, {94, 135, 65}, {84, 125, 50}, {34, 15, 95}
};

// ─── Gait tuning (changeable live from browser) ───────────────────────────────
int LIFT_ANGLE       = 20;   // femur lift per phase for walking (degrees)
int SWING_ANGLE      = 15;   // hip swing per phase for walking (degrees)
int SETTLE_MS        = 140;  // ms to wait after each snapTo
int TURN_LIFT_ANGLE  = 15;   // reduced femur lift for turning (stability)
int TURN_SWING_ANGLE = 10;   // reduced hip swing for turning (stability)

int STEPS_90         = 7;    // fallback steps per 90° turn
int STEPS_180        = 14;   // fallback steps per 180° turn

// Gait signs
const int PAIR[4]        = {0, 1, 1, 0};        // FL+BR=0, FR+BL=1
const int LIFT_SIGN[4]   = {+1, -1, -1, +1};
const int WALK_SIGN[4]   = {-1, +1, -1, +1};
const int TURN_R_SIGN[4] = {-1, -1, -1, -1};
const int TURN_L_SIGN[4] = {+1, +1, +1, +1};

// ─── Arm defaults ─────────────────────────────────────────────────────────────
int armGripOpen  = 89;
int armGripClose = 53;
int armLiftUp    = 75;
int armLiftDown  = 160;
int armGripPos   = 53;   // current grip angle
int armLiftPos   = 75;   // current lift angle
int armSpeedMs   = 12;   // ms per degree during smooth arm moves

// ─── Global state ─────────────────────────────────────────────────────────────
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
WebServer server(80);
Preferences linePrefs;
int curAngle[4][3];
volatile bool busy = false;
char statusMsg[80] = "Idle — ready";

// ToF sensor objects
VL53L0X tofDev[3];
bool     tofOk[3]   = {false, false, false};
float    tofDist[3] = {0, 0, 0};  // mm
const uint8_t TOF_XSHUT[3] = {TOF_FRONT_XSHUT, TOF_RIGHT_XSHUT, TOF_LEFT_XSHUT};
const uint8_t TOF_ADDR[3]  = {TOF_FRONT_ADDR,  TOF_RIGHT_ADDR,  TOF_LEFT_ADDR};
const char*   TOF_NAME[3]  = {"Front", "Right", "Left"};
uint32_t      tofLastMs    = 0;

// Line array state, calibration & indication
uint16_t lineRaw[LINE_N]     = {0};
uint16_t lineMin[LINE_N]     = {500, 500, 500, 500, 500, 500, 500, 500, 500};
uint16_t lineMax[LINE_N]     = {3200, 3200, 3200, 3200, 3200, 3200, 3200, 3200, 3200};
float    lineNorm[LINE_N]    = {0};
float    lineThr             = 0.50f;
float    lineErrMm           = 0.0f;
float    lineCentroid        = 4.0f;
uint8_t  lineActCnt          = 0;
uint16_t lineMask            = 0;
bool     lineFound           = false;
bool     lineCalibrated      = false;
bool     lineCalActive       = false;
uint32_t lineCalUntil        = 0;
uint16_t calSweepMin[LINE_N];
uint16_t calSweepMax[LINE_N];
bool     lineWhiteHigh       = true;  // true: white line reads high; false: inverted
char     lineCalMsg[96]      = "Not calibrated";
uint32_t lineLastMs          = 0;

// Line follow walk state
volatile bool lfRunning    = false;
volatile bool lfStop       = false;
int           lfStepsDone  = 0;
int           lfTargetSteps= 0;
float         lfKp         = 0.35f;
float         lfKd         = 0.12f;
int           lfMaxCorr    = 12;
bool          lfStopLost   = true;
int           lfSteerSign  = -1;      // -1: inverted steering (corrects left/right inversion)
bool          lfSwapSens   = false;   // true: swap S1<->S9 sensor orientation
char          lfStatusStr[64] = "idle";

// MPU6050 Gyro state
bool     mpuReady       = false;
bool     mpuCalibrated  = false;
bool     mpuStationary  = true;
uint8_t  mpuAddr        = MPU_ADDR_DEF;
float    mpuGBiasZ      = 0.0f;
float    mpuYaw         = 0.0f;   // relative integrated heading
float    mpuGz          = 0.0f;   // filtered turn rate (°/s)
uint32_t mpuLastMs      = 0;

// Gyro turn state
volatile bool  gyroTurnRunning = false;
volatile bool  gyroTurnStop    = false;
float gyroTurnTarget  = 0.0f;
float gyroTurnDone    = 0.0f;
char  gyroTurnPhase[64] = "idle";

// ─── LEDC arm helpers ─────────────────────────────────────────────────────────
uint32_t armDegToDuty(int deg) {
    deg = constrain(deg, 0, 180);
    uint32_t us = (uint32_t)map(deg, 0, 180, 500, 2500);
    return ARM_US_TO_DUTY(us);
}

void armInitLedc() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcAttach(ARM_LIFT_PIN, ARM_LEDC_FREQ, ARM_LEDC_RES);
    ledcAttach(ARM_GRIP_PIN, ARM_LEDC_FREQ, ARM_LEDC_RES);
#else
    ledcSetup(6, ARM_LEDC_FREQ, ARM_LEDC_RES);
    ledcAttachPin(ARM_LIFT_PIN, 6);
    ledcSetup(7, ARM_LEDC_FREQ, ARM_LEDC_RES);
    ledcAttachPin(ARM_GRIP_PIN, 7);
#endif
}

void rawLift(int deg) {
    deg = constrain(deg, 20, 160);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWrite(ARM_LIFT_PIN, armDegToDuty(deg));
#else
    ledcWrite(6, armDegToDuty(deg));
#endif
    armLiftPos = deg;
}

void rawGrip(int deg) {
    deg = constrain(deg, 0, 180);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWrite(ARM_GRIP_PIN, armDegToDuty(deg));
#else
    ledcWrite(7, armDegToDuty(deg));
#endif
    armGripPos = deg;
}

void sweepLift(int target) {
    target = constrain(target, 20, 160);
    int dir = (target > armLiftPos) ? 1 : -1;
    while (armLiftPos != target) {
        armLiftPos += dir;
        rawLift(armLiftPos);
        delay(armSpeedMs);
    }
}

void sweepGrip(int target) {
    target = constrain(target, 0, 180);
    int dir = (target > armGripPos) ? 1 : -1;
    while (armGripPos != target) {
        armGripPos += dir;
        rawGrip(armGripPos);
        delay(armSpeedMs);
    }
}

// ─── ToF init & poll ──────────────────────────────────────────────────────────
void tofInit() {
    for (int i = 0; i < 3; i++) {
        pinMode(TOF_XSHUT[i], OUTPUT);
        digitalWrite(TOF_XSHUT[i], LOW);
    }
    delay(20);

    for (int i = 0; i < 3; i++) {
        digitalWrite(TOF_XSHUT[i], HIGH);
        delay(15);
        tofDev[i].setTimeout(300);
        if (tofDev[i].init()) {
            tofDev[i].setAddress(TOF_ADDR[i]);
            tofDev[i].setMeasurementTimingBudget(33000);
            tofDev[i].startContinuous();
            tofOk[i] = true;
            Serial.printf("[TOF] %s OK -> 0x%02X\n", TOF_NAME[i], TOF_ADDR[i]);
        } else {
            tofOk[i] = false;
            Serial.printf("[TOF] %s NOT FOUND\n", TOF_NAME[i]);
        }
    }
}

void tofPoll() {
    if (millis() - tofLastMs < 50) return;
    tofLastMs = millis();
    for (int i = 0; i < 3; i++) {
        if (!tofOk[i]) continue;
        uint16_t mm = tofDev[i].readRangeContinuousMillimeters();
        if (!tofDev[i].timeoutOccurred() && mm < 2000)
            tofDist[i] = (float)mm;
    }
}

// ─── Line array calibration persistent storage ────────────────────────────────
void lineSaveConfig() {
    linePrefs.begin("run4line", false);
    linePrefs.putBytes("min", lineMin, sizeof(lineMin));
    linePrefs.putBytes("max", lineMax, sizeof(lineMax));
    linePrefs.putFloat("thr", lineThr);
    linePrefs.putBool("wh", lineWhiteHigh);
    linePrefs.putBool("cal", lineCalibrated);
    linePrefs.putInt("sinv", lfSteerSign);
    linePrefs.putBool("swap", lfSwapSens);
    linePrefs.end();
    Serial.println("[LINE-CAL] Saved to NVS flash");
}

void lineLoadConfig() {
    linePrefs.begin("run4line", true);
    if (linePrefs.isKey("cal")) {
        linePrefs.getBytes("min", lineMin, sizeof(lineMin));
        linePrefs.getBytes("max", lineMax, sizeof(lineMax));
        lineThr = linePrefs.getFloat("thr", 0.50f);
        lineWhiteHigh = linePrefs.getBool("wh", true);
        lineCalibrated = linePrefs.getBool("cal", false);
        lfSteerSign = linePrefs.getInt("sinv", -1);
        lfSwapSens  = linePrefs.getBool("swap", false);
        snprintf(lineCalMsg, sizeof(lineCalMsg), lineCalibrated ? "Calibrated from Flash" : "Loaded uncalibrated");
        Serial.printf("[LINE-CAL] Loaded from flash: thr=%.2f wh=%d cal=%d sinv=%d swap=%d\n",
                      lineThr, lineWhiteHigh, lineCalibrated, lfSteerSign, lfSwapSens);
    } else {
        Serial.println("[LINE-CAL] No saved config found, using defaults");
    }
    linePrefs.end();
}

void lineStartCal(uint32_t durationMs = 5000) {
    for (int i = 0; i < LINE_N; i++) {
        calSweepMin[i] = 4095;
        calSweepMax[i] = 0;
    }
    lineCalActive = true;
    lineCalUntil = millis() + durationMs;
    snprintf(lineCalMsg, sizeof(lineCalMsg), "Sweeping... slide sensor over line & floor");
    Serial.println("[LINE-CAL] 5-second calibration sweep started");
}

void lineFinishCal() {
    lineCalActive = false;
    uint8_t okCount = 0;
    for (int i = 0; i < LINE_N; i++) {
        if (calSweepMax[i] > calSweepMin[i] + 60) {
            lineMin[i] = calSweepMin[i];
            lineMax[i] = calSweepMax[i];
            okCount++;
        }
    }
    if (okCount >= LINE_N - 2) {
        lineCalibrated = true;
        snprintf(lineCalMsg, sizeof(lineCalMsg), "Calibrated OK (%d/9 sensors)", okCount);
        Serial.printf("[LINE-CAL] %s\n", lineCalMsg);
        lineSaveConfig();
    } else {
        snprintf(lineCalMsg, sizeof(lineCalMsg), "Cal FAILED: only %d/9 sensors saw contrast (min span 60)", okCount);
        Serial.printf("[LINE-CAL] %s\n", lineCalMsg);
    }
}

// ─── Line array poll & centroid calculation ───────────────────────────────────
void linePoll() {
    if (millis() - lineLastMs < 20) return;
    lineLastMs = millis();
    uint32_t now = millis();

    // 1. If calibration sweep active: capture min and max
    if (lineCalActive) {
        for (int i = 0; i < LINE_N; i++) {
            uint16_t v = (uint16_t)analogRead(LINE_PINS[i]);
            lineRaw[i] = v;
            if (v < calSweepMin[i]) calSweepMin[i] = v;
            if (v > calSweepMax[i]) calSweepMax[i] = v;
        }
        if (now >= lineCalUntil) {
            lineFinishCal();
        }
        return;
    }

    // 2. Normal sensing
    lineActCnt = 0;
    lineMask = 0;
    float sumW = 0.0f;
    float sumWN = 0.0f;

    for (int i = 0; i < LINE_N; i++) {
        uint16_t v = (uint16_t)analogRead(LINE_PINS[i]);
        lineRaw[i] = v;

        uint16_t minV = lineMin[i];
        uint16_t maxV = lineMax[i];
        uint16_t span = (maxV > minV) ? (maxV - minV) : 1;
        if (span < 50) span = 50;

        float n = (float)(v - minV) / (float)span;
        n = constrain(n, 0.0f, 1.0f);
        if (!lineWhiteHigh) n = 1.0f - n; // invert if white reads low ADC
        lineNorm[i] = n;

        if (n >= lineThr) {
            lineActCnt++;
            lineMask |= (1 << i);
            float w = n - lineThr;
            sumW += w;
            sumWN += w * (float)i;
        }
    }

    if (sumW > 0.001f) {
        lineCentroid = sumWN / sumW;
        lineErrMm = (lineCentroid - 4.0f) * 10.0f; // 10 mm spacing, 4.0 is center (S5)
        lineFound = true;
    } else {
        lineFound = false;
    }
}

// ─── MPU6050 Driver with Deadband & Anti-Drift ────────────────────────────────
bool mpuWriteReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(mpuAddr);
    Wire.write(reg);
    Wire.write(val);
    return (Wire.endTransmission() == 0);
}

bool mpuReadGz(int16_t &rawGz) {
    Wire.beginTransmission(mpuAddr);
    Wire.write(0x47); // GYRO_ZOUT_H
    if (Wire.endTransmission(false) != 0) {
        Wire.beginTransmission(mpuAddr);
        Wire.write(0x47);
        if (Wire.endTransmission(true) != 0) return false;
    }
    if (Wire.requestFrom((uint8_t)mpuAddr, (uint8_t)2) < 2) return false;
    rawGz = (int16_t)((Wire.read() << 8) | Wire.read());
    return true;
}

void mpuCalibrate(uint16_t samples = 400) {
    Serial.println("[MPU] Calibrating gyro bias... keep robot still!");
    int16_t dummy;
    // Discard initial warmup readings so DLPF settles
    for (int i = 0; i < 40; i++) {
        mpuReadGz(dummy);
        delay(4);
    }

    int64_t sumZ = 0;
    int valid = 0;
    for (uint16_t i = 0; i < samples; i++) {
        int16_t gz;
        if (mpuReadGz(gz)) {
            sumZ += gz;
            valid++;
        }
        delay(4);
    }

    if (valid > 80) {
        mpuGBiasZ = (float)sumZ / (float)valid;
        mpuCalibrated = true;
        mpuReady = true;
        mpuGz = 0.0f;
        mpuYaw = 0.0f;
        Serial.printf("[MPU] Calibrated: gBiasZ=%.2f (%d valid samples)\n", mpuGBiasZ, valid);
    } else {
        Serial.println("[MPU] Calibration FAILED — check wiring");
    }
}

void mpuInit() {
    uint8_t addrs[2] = {0x68, 0x69};
    bool found = false;
    for (uint8_t a : addrs) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) {
            mpuAddr = a;
            found = true;
            break;
        }
    }
    if (!found) {
        Serial.println("[MPU] NOT FOUND at 0x68 or 0x69");
        mpuReady = false;
        return;
    }
    Serial.printf("[MPU] Found at 0x%02X — initializing...\n", mpuAddr);

    // Reset device
    mpuWriteReg(0x6B, 0x80);
    delay(50);
    // Wake & select PLL with X gyro clock
    mpuWriteReg(0x6B, 0x01);
    delay(50);
    // Enable all axes
    mpuWriteReg(0x6C, 0x00);
    delay(10);
    // Sample rate divider 1000 / (1 + 9) = 100 Hz
    mpuWriteReg(0x19, 0x09);
    // DLPF ~44 Hz bandwidth
    mpuWriteReg(0x1A, 0x03);
    // ±500 °/s
    mpuWriteReg(0x1B, 0x08);
    // ±2g
    mpuWriteReg(0x1C, 0x00);
    delay(50);

    // Calibrate gyro bias while resting still
    mpuCalibrate(350);
}

void mpuPoll() {
    if (!mpuReady) return;
    uint32_t now = millis();
    if (now - mpuLastMs < 10) return; // 100 Hz
    float dt = (now - mpuLastMs) / 1000.0f;
    if (dt > 0.1f) dt = 0.1f;
    mpuLastMs = now;

    int16_t raw;
    if (!mpuReadGz(raw)) return;

    float curGz = ((float)raw - mpuGBiasZ) / MPU_GYRO_SENS;

    // Stationary slow drift correction (only when robot is declared stationary)
    if (mpuStationary && fabsf(curGz) < 2.0f) {
        float alpha = dt / 2.0f;
        if (alpha > 0.01f) alpha = 0.01f;
        mpuGBiasZ = (1.0f - alpha) * mpuGBiasZ + alpha * (float)raw;
    }

    // DEADBAND: If rate is below threshold, force to exactly 0 to eliminate drift
    if (fabsf(curGz) < MPU_DEADBAND) {
        curGz = 0.0f;
    }

    // EMA filter
    mpuGz = 0.80f * mpuGz + 0.20f * curGz;
    if (fabsf(mpuGz) < 0.15f) {
        mpuGz = 0.0f;
    }

    // Integrate yaw: CW turn is negative, CCW turn is positive
    mpuYaw += mpuGz * dt;
}

// ─── Servo helpers ────────────────────────────────────────────────────────────
void setServo(uint8_t ch, int angle) {
    angle = constrain(angle, 0, 180);
    pwm.setPWM(ch, 0, map(angle, 0, 180, SERVOMIN, SERVOMAX));
}

void applyPose(const int pose[4][3]) {
    for (int l = 0; l < 4; l++)
        for (int j = 0; j < 3; j++) {
            setServo(CH[l][j], pose[l][j]);
            curAngle[l][j] = pose[l][j];
            delay(8);
        }
}

void snapTo(const int tgt[4][3], int settleMs) {
    for (int l = 0; l < 4; l++)
        for (int j = 0; j < 3; j++)
            if (tgt[l][j] != curAngle[l][j]) {
                setServo(CH[l][j], tgt[l][j]);
                curAngle[l][j] = tgt[l][j];
            }
    // Poll server, ToF, Line, and Gyro during settle so readings stay live
    uint32_t t0 = millis();
    while (millis() - t0 < (uint32_t)settleMs) {
        server.handleClient();
        tofPoll();
        linePoll();
        mpuPoll();
        delay(2);
    }
}

void moveSmooth(const int tgt[4][3], int steps, int stepMs) {
    int start[4][3];
    memcpy(start, curAngle, sizeof(start));
    for (int s = 1; s <= steps; s++) {
        float t = (float)s / steps;
        t = t * t * (3.0f - 2.0f * t);
        for (int l = 0; l < 4; l++)
            for (int j = 0; j < 3; j++) {
                int a = start[l][j] + (int)roundf(t * (tgt[l][j] - start[l][j]));
                setServo(CH[l][j], a);
                curAngle[l][j] = a;
            }
        delay(stepMs);
        mpuPoll();
    }
}

// ─── Core gait ────────────────────────────────────────────────────────────────
void gaitWith(const int signs[4], int numSteps) {
    int tgt[4][3];
    for (int s = 0; s < numSteps; s++) {
        int pair = s % 2;
        memcpy(tgt, curAngle, sizeof(tgt));
        for (int l = 0; l < 4; l++) {
            if (PAIR[l] == pair) {
                tgt[l][1] = STAND_POSE[l][1] + LIFT_SIGN[l] * LIFT_ANGLE;
                tgt[l][0] = STAND_POSE[l][0] + signs[l] * SWING_ANGLE;
            } else {
                tgt[l][0] = STAND_POSE[l][0];
            }
        }
        snapTo(tgt, SETTLE_MS);
        for (int l = 0; l < 4; l++)
            if (PAIR[l] == pair) tgt[l][1] = STAND_POSE[l][1];
        snapTo(tgt, SETTLE_MS);
    }
    // Return all hips to standing
    memcpy(tgt, curAngle, sizeof(tgt));
    for (int l = 0; l < 4; l++) tgt[l][0] = STAND_POSE[l][0];
    snapTo(tgt, SETTLE_MS);
}

// ─── Closed-Loop Gyro Turn ───────────────────────────────────────────────────
void gyroTurnWith(const int signs[4], float targetDeg) {
    // 1. Ensure Gyro is initialized / active
    if (!mpuReady) {
        Serial.println("[GYRO-TURN] MPU not ready! Initializing...");
        mpuInit();
    }
    if (!mpuReady) {
        Serial.println("[GYRO-TURN] MPU offline! Falling back to step count.");
        int estSteps = constrain((int)roundf(targetDeg / 12.0f), 1, 30);
        gaitWith(signs, estSteps);
        return;
    }

    gyroTurnRunning = true;
    gyroTurnStop    = false;
    gyroTurnTarget  = targetDeg;
    gyroTurnDone    = 0.0f;
    mpuStationary   = false;

    // 2. Settle and explicitly ZERO gyro yaw heading at start
    uint32_t tSettle = millis();
    while (millis() - tSettle < 60) {
        server.handleClient();
        mpuPoll();
        delay(2);
    }
    mpuYaw = 0.0f; // clean zero start

    bool rightTurn = (signs[0] < 0);
    const float tol = MPU_TOL_DEG;
    int tgt[4][3];
    int stepIdx = 0;
    uint32_t tStart = millis();

    Serial.printf("[GYRO-TURN] Heading zeroed (yaw=0.0°). Target=%.1f° (%s)\n",
                  targetDeg, rightTurn ? "Right" : "Left");

    while (!gyroTurnStop) {
        int pair = stepIdx % 2;

        // Phase A: lift swingPair and swing hips
        memcpy(tgt, curAngle, sizeof(tgt));
        for (int l = 0; l < 4; l++) {
            if (PAIR[l] == pair) {
                tgt[l][1] = STAND_POSE[l][1] + LIFT_SIGN[l] * TURN_LIFT_ANGLE;
                tgt[l][0] = STAND_POSE[l][0] + signs[l] * TURN_SWING_ANGLE;
            } else {
                tgt[l][0] = STAND_POSE[l][0];
            }
        }
        snprintf(gyroTurnPhase, sizeof(gyroTurnPhase), "S%d Ph-A (%.1f/%.0f°)", stepIdx + 1, gyroTurnDone, targetDeg);
        snapTo(tgt, SETTLE_MS);
        if (gyroTurnStop) break;

        // Phase B: lower swingPair back to standing (all 4 feet planted)
        for (int l = 0; l < 4; l++) {
            if (PAIR[l] == pair) tgt[l][1] = STAND_POSE[l][1];
        }
        snprintf(gyroTurnPhase, sizeof(gyroTurnPhase), "S%d Ph-B (%.1f/%.0f°)", stepIdx + 1, gyroTurnDone, targetDeg);
        snapTo(tgt, SETTLE_MS);

        // Measure turned angle (mpuYaw was zeroed at start)
        // Right turn: mpuYaw decreases into negative -> turned is -mpuYaw
        // Left turn:  mpuYaw increases into positive -> turned is +mpuYaw
        float currentYaw = mpuYaw;
        float turned = rightTurn ? -currentYaw : currentYaw;
        if (turned < 0.0f) turned = 0.0f;
        gyroTurnDone = turned;
        Serial.printf("[GYRO-TURN] Step %d done: turned=%.1f° / %.1f° (yaw=%.1f°)\n",
                      stepIdx + 1, turned, targetDeg, currentYaw);

        if (turned >= targetDeg - tol) {
            Serial.printf("[GYRO-TURN] Target reached: turned=%.1f° (target=%.1f°)\n", turned, targetDeg);
            break;
        }

        if (millis() - tStart > 60000) {
            Serial.println("[GYRO-TURN] 60s timeout reached");
            break;
        }

        stepIdx++;
    }

    // Return all hips to standing
    memcpy(tgt, curAngle, sizeof(tgt));
    for (int l = 0; l < 4; l++) tgt[l][0] = STAND_POSE[l][0];
    snapTo(tgt, SETTLE_MS);

    mpuStationary = true;
    gyroTurnRunning = false;
    snprintf(gyroTurnPhase, sizeof(gyroTurnPhase), "Done: %.1f° in %d steps", gyroTurnDone, stepIdx + 1);
    Serial.printf("[GYRO-TURN] Finished: turned=%.1f° target=%.1f° (yaw=%.1f°)\n",
                  gyroTurnDone, targetDeg, mpuYaw);
}

// ─── Line Following Walk (Closed-Loop Steering) ──────────────────────────────
void lineFollowWalk(int steps) {
    lfRunning = true;
    lfStop = false;
    lfStepsDone = 0;
    lfTargetSteps = steps;

    int totalSteps = (steps <= 0) ? 999999 : steps;
    float prevErr = lineErrMm;
    int tgt[4][3];

    Serial.printf("[LF] Starting line follow: steps=%d (0=inf), Kp=%.2f Kd=%.2f steerSign=%d swap=%d\n",
                  steps, lfKp, lfKd, lfSteerSign, lfSwapSens);
    snprintf(lfStatusStr, sizeof(lfStatusStr), "Line follow active (0/%d)", steps);

    for (int s = 0; s < totalSteps && !lfStop; s++) {
        // Read line error
        linePoll();

        if (!lineFound && lfStopLost) {
            Serial.println("[LF] Line lost — stopping line follow.");
            snprintf(lfStatusStr, sizeof(lfStatusStr), "Stopped: line lost at step %d", s);
            break;
        }

        // Calculate PD steering
        float err = lineFound ? lineErrMm : prevErr;
        if (lfSwapSens) err = -err; // optional swap if sensor board mounted reversed
        float dErr = err - prevErr;
        prevErr = err;

        // lfSteerSign applies user-selectable sign (default -1 inverted)
        float corr = (float)lfSteerSign * (lfKp * err + lfKd * dErr);
        corr = constrain(corr, (float)-lfMaxCorr, (float)lfMaxCorr);

        int swingL = constrain(SWING_ANGLE + (int)roundf(corr), 5, 35);
        int swingR = constrain(SWING_ANGLE - (int)roundf(corr), 5, 35);

        int hipSwing[4] = {swingL, swingR, swingL, swingR}; // 0=FL, 1=FR, 2=BL, 3=BR

        int pair = s % 2;

        // Phase A: lift swingPair & swing hips with differential stride
        memcpy(tgt, curAngle, sizeof(tgt));
        for (int l = 0; l < 4; l++) {
            if (PAIR[l] == pair) {
                tgt[l][1] = STAND_POSE[l][1] + LIFT_SIGN[l] * LIFT_ANGLE;
                tgt[l][0] = STAND_POSE[l][0] + WALK_SIGN[l] * hipSwing[l];
            } else {
                tgt[l][0] = STAND_POSE[l][0];
            }
        }
        snprintf(lfStatusStr, sizeof(lfStatusStr), "Step %d/%d (err=%.1fmm L=%d° R=%d°)",
                 s + 1, steps, err, swingL, swingR);
        snapTo(tgt, SETTLE_MS);
        if (lfStop) break;

        // Phase B: lower swingPair back to ground
        for (int l = 0; l < 4; l++) {
            if (PAIR[l] == pair) tgt[l][1] = STAND_POSE[l][1];
        }
        snapTo(tgt, SETTLE_MS);

        lfStepsDone++;
    }

    // Return all hips to standing pose
    memcpy(tgt, curAngle, sizeof(tgt));
    for (int l = 0; l < 4; l++) tgt[l][0] = STAND_POSE[l][0];
    snapTo(tgt, SETTLE_MS);

    lfRunning = false;
    snprintf(lfStatusStr, sizeof(lfStatusStr), "Done: %d steps completed", lfStepsDone);
    Serial.printf("[LF] Finished: %d steps done\n", lfStepsDone);
}

void walkForward(int steps) { gaitWith(WALK_SIGN, steps); }

void turnRight(int steps) {
    int prevLift = LIFT_ANGLE, prevSwing = SWING_ANGLE;
    LIFT_ANGLE = TURN_LIFT_ANGLE; SWING_ANGLE = TURN_SWING_ANGLE;
    gaitWith(TURN_R_SIGN, steps);
    LIFT_ANGLE = prevLift; SWING_ANGLE = prevSwing;
}

void turnLeft(int steps) {
    int prevLift = LIFT_ANGLE, prevSwing = SWING_ANGLE;
    LIFT_ANGLE = TURN_LIFT_ANGLE; SWING_ANGLE = TURN_SWING_ANGLE;
    gaitWith(TURN_L_SIGN, steps);
    LIFT_ANGLE = prevLift; SWING_ANGLE = prevSwing;
}

void standUp() {
    snprintf(statusMsg, sizeof(statusMsg), "Standing up...");
    moveSmooth(STAND_POSE, 60, 30);
    snprintf(statusMsg, sizeof(statusMsg), "Standing — idle");
}

// ─── DASHBOARD HTML ───────────────────────────────────────────────────────────
const char PAGE[] PROGMEM = R"html(
<!DOCTYPE html><html><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>RUNNER4 MinTest</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:system-ui,sans-serif;background:#0d1117;color:#e6edf3;padding:16px 14px}
h1{font-size:1.25rem;font-weight:800;letter-spacing:3px;margin-bottom:3px;
   background:linear-gradient(135deg,#58a6ff,#a5d6ff);-webkit-background-clip:text;-webkit-text-fill-color:transparent}
.subtitle{font-size:.68rem;color:#6e7681;margin-bottom:18px}
h2{font-size:.72rem;font-weight:700;text-transform:uppercase;letter-spacing:1.5px;
   color:#8b949e;margin-bottom:10px;padding-bottom:5px;border-bottom:1px solid #21262d}
.section{background:#161b22;border:1px solid #30363d;border-radius:12px;padding:14px;margin-bottom:14px}
.row{display:flex;gap:8px;flex-wrap:wrap;margin-bottom:10px}
button{padding:9px 14px;border:none;border-radius:8px;font-size:.78rem;font-weight:700;cursor:pointer;transition:all .15s;white-space:nowrap}
.go{background:linear-gradient(135deg,#1a7f37,#3fb950);color:#fff}
.bl{background:linear-gradient(135deg,#1f6feb,#58a6ff);color:#fff}
.or{background:linear-gradient(135deg,#b45309,#f59e0b);color:#fff}
.re{background:linear-gradient(135deg,#b91c1c,#f85149);color:#fff}
.pu{background:linear-gradient(135deg,#7c3aed,#a78bfa);color:#fff}
button:hover{transform:translateY(-1px);opacity:.88}
label{font-size:.66rem;color:#8b949e;display:block;margin-bottom:3px;font-weight:600;letter-spacing:.4px}
input[type=number],input[type=range]{width:100%;background:#0d1117;border:1px solid #30363d;
  border-radius:6px;color:#e6edf3;padding:5px 8px;font-size:.78rem;outline:none;margin-bottom:9px;display:block}
input[type=range]{padding:3px 0;border:none;background:transparent;accent-color:#58a6ff}
.param-grid{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px 12px;margin-bottom:10px}
.param-grid label{margin-bottom:1px}
.sl-row{display:flex;align-items:center;gap:8px;margin-bottom:7px}
.sl-row label{flex:0 0 90px;margin:0;font-size:.65rem}
.sl-row input[type=range]{flex:1;margin:0}
.sl-val{font-size:.75rem;font-family:monospace;color:#e6edf3;min-width:34px;text-align:right}

/* Line Array (Matches RUNNER4 Lab) */
.track{position:relative;height:18px;background:#0d1424;border-radius:9px;margin:8px 0 12px;border:1px solid #243049}
.track:after{content:'';position:absolute;left:50%;top:0;bottom:0;width:2px;background:#475569}
#mk{position:absolute;top:-3px;width:24px;height:24px;margin-left:-12px;border-radius:50%;background:#38bdf8;box-shadow:0 0 14px #38bdf8;left:50%;transition:left .08s;opacity:.3}
.sens{display:grid;grid-template-columns:repeat(9,1fr);gap:5px}
.sc{background:#0d1117;border-radius:8px;padding:6px 2px;text-align:center;border:2px solid #21262d;transition:border-color .15s}
.sc.on{border-color:#facc15;background:rgba(250,204,21,.07);box-shadow:0 0 8px rgba(250,204,21,.3)}
.bw{position:relative;height:75px;background:#020617;border-radius:4px;margin:4px auto;width:65%;overflow:hidden}
.bw i{position:absolute;left:0;right:0;bottom:0;background:linear-gradient(#58a6ff,#1f6feb);transition:height .15s}
.bw u{position:absolute;left:0;right:0;border-top:2px dashed #f87171}
.rv{font-size:.58rem;color:#6e7681;font-family:monospace}
.nv{font-size:.65rem;font-weight:700;font-family:monospace;color:#e6edf3}
.sn{font-size:.62rem;color:#58a6ff;font-weight:700;margin-top:2px}
.pill{display:inline-block;padding:3px 8px;border-radius:99px;font-size:.65rem;font-weight:700}
.pill.ok{background:#14532d;color:#bbf7d0}
.pill.bad{background:#7f1d1d;color:#fecaca}
.pill.warn{background:#713f12;color:#fde68a}

/* Sensors / ToF / Gyro Cards */
.tof-grid{display:grid;grid-template-columns:1fr 1fr 1fr;gap:10px;margin-top:4px}
.tof-card{background:#0d1117;border:1px solid #21262d;border-radius:9px;padding:10px;text-align:center}
.tof-name{font-size:.6rem;color:#6e7681;font-weight:700;letter-spacing:1px;text-transform:uppercase;margin-bottom:4px}
.tof-val{font-size:1.4rem;font-weight:800;font-family:monospace;color:#58a6ff}
.tof-unit{font-size:.6rem;color:#6e7681}
.tof-dot{width:7px;height:7px;border-radius:50%;margin:4px auto 0;background:#484f58}
.tof-dot.on{background:#3fb950;animation:blink 2s infinite}
@keyframes blink{0%,100%{opacity:1}50%{opacity:.35}}

/* Arm */
.arm-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.arm-card{background:#0d1117;border:1px solid #21262d;border-radius:9px;padding:10px}
.arm-title{font-size:.6rem;color:#6e7681;font-weight:700;letter-spacing:1px;text-transform:uppercase;margin-bottom:6px}

/* Status */
#st{font-size:.77rem;color:#3fb950;font-family:monospace;padding:7px 10px;
    background:#0d1117;border:1px solid #21262d;border-radius:7px;min-height:2rem;
    display:flex;align-items:center;gap:6px}
.dot{width:8px;height:8px;border-radius:50%;background:#3fb950;flex-shrink:0;animation:blink 2s infinite}
.dot.busy{background:#f59e0b}
.sub-btn-row{display:flex;gap:6px;flex-wrap:wrap;margin-bottom:8px}
</style>
</head><body>
<h1>RUNNER4 MinTest</h1>
<p class="subtitle">All sensors + motion · one page · http://192.168.4.1</p>

<!-- STATUS BAR -->
<div id="st"><span class="dot" id="sdot"></span><span id="smsg">Idle — ready</span></div>
<div style="height:12px"></div>

<!-- ═══ SECTION 1: MOTION ═══════════════════════════════════════════════════ -->
<div class="section">
<h2>&#128694; Motion</h2>

<div class="param-grid">
  <div><label>Walk Steps</label><input type="number" id="ws" value="10" min="1" max="200"></div>
  <div><label>Turn Steps (manual)</label><input type="number" id="ts" value="8" min="1" max="200"></div>
  <div><label>Lift Angle °</label><input type="number" id="la" value="20" min="5" max="60"></div>
  <div><label>Swing Angle °</label><input type="number" id="sa" value="15" min="5" max="60"></div>
  <div><label>Settle ms/phase</label><input type="number" id="sm" value="140" min="40" max="800"></div>
  <div><label>Turn Hip Swing °</label><input type="number" id="tsa" value="10" min="5" max="40"></div>
</div>

<div class="row">
  <button class="go" onclick="cmd('/stand')">&#127939; Stand Up</button>
  <button class="go" onclick="doWalk()">&#128694; Walk</button>
  <button class="re" onclick="cmd('/home')">&#8962; Home</button>
</div>

<!-- GYRO-TARGETED TURNS -->
<div style="margin-top:6px;padding-top:8px;border-top:1px solid #21262d">
  <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:6px">
    <span style="font-size:.65rem;color:#f59e0b;font-weight:700;text-transform:uppercase;letter-spacing:1px">&#9881; Closed-Loop Gyro Turns (MPU6050)</span>
    <span style="font-size:.6rem;color:#8b949e">Auto-zeroes Yaw · Target ±3°</span>
  </div>
  <div class="sub-btn-row">
    <button class="or" onclick="doGyroTurn('right',90)">&#8635; Right 90° (Gyro)</button>
    <button class="or" onclick="doGyroTurn('left',90)">&#8634; Left 90° (Gyro)</button>
    <button class="or" onclick="doGyroTurn('right',180)">&#8635; Right 180° (Gyro)</button>
    <button class="or" onclick="doGyroTurn('left',180)">&#8634; Left 180° (Gyro)</button>
  </div>
  <div class="row" style="align-items:center;margin-bottom:6px">
    <div style="flex:1;max-width:130px">
      <label style="margin-bottom:2px">Custom Target °</label>
      <input type="number" id="custAngle" value="90" min="5" max="360" style="margin-bottom:0">
    </div>
    <div style="display:flex;gap:6px;align-items:flex-end;flex:1">
      <button class="bl" style="margin-top:14px" onclick="doGyroTurn('right', document.getElementById('custAngle').value)">&#8635; Turn Right (Angle)</button>
      <button class="bl" style="margin-top:14px" onclick="doGyroTurn('left', document.getElementById('custAngle').value)">&#8634; Turn Left (Angle)</button>
    </div>
  </div>

  <button class="re" id="gyroStopBtn" onclick="stopGyroCmd()" style="display:none;width:100%;margin-bottom:8px">&#9209; STOP Gyro Turn</button>

  <div id="gyroProgBox" style="display:none;margin-top:6px;background:#0d1117;border:1px solid #30363d;border-radius:6px;padding:6px 8px">
    <div style="height:6px;background:#21262d;border-radius:3px;overflow:hidden;margin-bottom:4px">
      <div id="gyroProgBar" style="height:100%;width:0%;background:#f59e0b;transition:width .2s"></div>
    </div>
    <div id="gyroProgText" style="font-size:.62rem;color:#a5d6ff;font-family:monospace"></div>
  </div>
</div>

<!-- MANUAL STEP TURNS -->
<div style="margin-top:8px;padding-top:6px;border-top:1px solid #21262d">
  <span style="font-size:.62rem;color:#6e7681;display:block;margin-bottom:4px">MANUAL STEP-COUNT TURNS (No Gyro Target)</span>
  <div class="sub-btn-row">
    <button class="bl" onclick="doTurn('right')">&#8635; Turn Right (steps)</button>
    <button class="bl" onclick="doTurn('left')">&#8634; Turn Left (steps)</button>
  </div>
</div>
</div>

<!-- ═══ SECTION 2: ARM ═══════════════════════════════════════════════════════ -->
<div class="section">
<h2>&#129482; Robot Arm</h2>

<div class="arm-grid">
  <div class="arm-card">
    <div class="arm-title">Lift (GPIO 47)</div>
    <div class="sl-row"><label>Angle °</label>
      <input type="range" id="liftSl" min="20" max="160" value="75" oninput="liftSlChg()">
      <span class="sl-val" id="liftVal">75°</span>
    </div>
    <div class="row" style="margin-bottom:0">
      <button class="go" onclick="armCmd('/arm/up')">&#8679; Up</button>
      <button class="or" onclick="armCmd('/arm/down')">&#8681; Down</button>
      <button class="bl" onclick="armSetLift()">Set °</button>
    </div>
    <div style="margin-top:7px">
      <label>Up preset °</label><input type="number" id="lupIn" value="75" min="20" max="160">
      <label>Down preset °</label><input type="number" id="ldnIn" value="160" min="20" max="160">
      <button class="pu" style="width:100%" onclick="armSaveCfg()">Save Presets</button>
    </div>
  </div>

  <div class="arm-card">
    <div class="arm-title">Gripper (GPIO 44)</div>
    <div class="sl-row"><label>Angle °</label>
      <input type="range" id="gripSl" min="0" max="180" value="53" oninput="gripSlChg()">
      <span class="sl-val" id="gripVal">53°</span>
    </div>
    <div class="row" style="margin-bottom:0">
      <button class="go" onclick="armCmd('/arm/open')">&#128280; Open</button>
      <button class="re" onclick="armCmd('/arm/close')">&#128278; Close</button>
      <button class="bl" onclick="armSetGrip()">Set °</button>
    </div>
    <div style="margin-top:7px">
      <label>Open preset °</label><input type="number" id="gopIn" value="89" min="0" max="180">
      <label>Close preset °</label><input type="number" id="gclIn" value="53" min="0" max="180">
      <button class="pu" style="width:100%" onclick="armSaveCfg()">Save Presets</button>
    </div>
  </div>
</div>

<div class="row" style="margin-top:10px">
  <button class="pu" onclick="armCmd('/arm/pick')">&#129305; Pick</button>
  <button class="pu" onclick="armCmd('/arm/place')">&#128230; Place</button>
  <div><label style="margin-bottom:2px">Speed ms/deg</label>
    <input type="number" id="aspd" value="12" min="1" max="100" style="width:70px;margin:0">
  </div>
</div>
</div>

<!-- ═══ SECTION 3: LINE ARRAY & LINE FOLLOW ═══════════════════════════════════ -->
<div class="section">
<div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:8px;padding-bottom:5px;border-bottom:1px solid #21262d">
  <h2 style="border-bottom:none;margin-bottom:0;padding-bottom:0">&#9135; Line Array (9 &times; TCRT5000)</h2>
  <div style="display:flex;gap:6px;align-items:center">
    <span id="lineCalPill" class="pill bad">NOT CALIBRATED</span>
    <span id="linePill" class="pill ok">LINE OK</span>
  </div>
</div>

<div style="display:grid;grid-template-columns:1fr 1fr 1fr;gap:10px;margin-bottom:10px;text-align:center">
  <div class="tof-card" style="padding:8px">
    <div class="tof-name">Line Error</div>
    <div class="tof-val" id="lineErrVal" style="font-size:1.3rem;color:#38bdf8">0.0 mm</div>
    <div class="tof-unit">&minus; left &nbsp;&middot;&nbsp; + right</div>
  </div>
  <div class="tof-card" style="padding:8px">
    <div class="tof-name">Active Sensors</div>
    <div class="tof-val" id="lineActVal" style="font-size:1.3rem;color:#22c55e">0 / 9</div>
    <div class="tof-unit">centroid: <span id="lineCenVal" style="font-family:monospace;font-weight:700">-</span></div>
  </div>
  <div class="tof-card" style="padding:8px">
    <div class="tof-name">Status</div>
    <div class="tof-val" id="lineStatusVal" style="font-size:1.1rem;color:#facc15;padding-top:3px">Searching</div>
    <div class="tof-unit">GPIO 1..10 ADC1</div>
  </div>
</div>

<!-- Center track with sliding cyan position marker -->
<div class="track"><div id="mk"></div></div>

<!-- 9-sensor card display with raw, bar, threshold, norm, label -->
<div class="sens" id="lineSensors"></div>
<div style="font-size:.6rem;color:#6e7681;margin-top:6px;text-align:center">
  Yellow frame = active on line &nbsp;&middot;&nbsp; Dashed red = threshold &nbsp;&middot;&nbsp; Bar = normalised 0&ndash;1
</div>

<!-- ─── SENSOR CALIBRATION TOOLBAR ─── -->
<div style="margin-top:12px;padding:10px;background:#0d1117;border:1px solid #21262d;border-radius:8px">
  <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:6px">
    <span style="font-size:.68rem;color:#f59e0b;font-weight:700;text-transform:uppercase;letter-spacing:1px">&#129514; Line Calibration &amp; Threshold</span>
    <span id="calTimeLeft" style="font-size:.62rem;color:#58a6ff;font-family:monospace"></span>
  </div>

  <!-- Sweep progress bar (shown during 5s sweep) -->
  <div id="calProgWrap" style="height:6px;background:#21262d;border-radius:3px;overflow:hidden;margin-bottom:8px;display:none">
    <div id="calProgFill" style="height:100%;width:0%;background:#f59e0b;transition:width .2s"></div>
  </div>

  <div class="sub-btn-row" style="margin-bottom:8px">
    <button class="bl" id="calStartBtn" onclick="startLineCal()">&#9654; 5s Auto Sweep Cal</button>
    <button class="re" id="calStopBtn" style="display:none" onclick="stopLineCal()">&#9632; Finish Sweep Now</button>
    <button class="pu" onclick="setCalFloor()">&#9889; Set Floor (Dark)</button>
    <button class="pu" onclick="setCalLine()">&#9889; Set Line (White)</button>
    <button class="go" onclick="saveLineCal()">&#128190; Save Flash</button>
  </div>

  <!-- Live Threshold Slider & Polarity Switch -->
  <div class="sl-row" style="margin-bottom:4px">
    <label style="flex:0 0 100px">Threshold: <b id="thrNum" style="color:#58a6ff">0.50</b></label>
    <input type="range" id="thrSlider" min="0.10" max="0.90" step="0.02" value="0.50" oninput="onThrInput(this.value)" onchange="onThrChange(this.value)">
    <span class="sl-val" id="thrPct">50%</span>
  </div>

  <div style="display:flex;justify-content:space-between;align-items:center;margin-top:6px;flex-wrap:wrap;gap:6px">
    <label style="display:flex;align-items:center;gap:5px;font-size:.65rem;color:#c9d1d9;margin:0;cursor:pointer">
      <input type="checkbox" id="whCheck" checked onchange="onWhChange(this.checked)" style="width:auto;margin:0">
      White line reads HIGH (uncheck if white is low ADC)
    </label>
    <div id="calMsgText" style="font-size:.62rem;color:#8b949e;font-family:monospace">Not calibrated</div>
  </div>
</div>

<!-- LINE FOLLOWING WALK CONTROLS -->
<div style="margin-top:12px;padding-top:10px;border-top:1px solid #21262d">
  <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:6px">
    <span style="font-size:.68rem;color:#38bdf8;font-weight:700;text-transform:uppercase;letter-spacing:1px">&#129302; Line Following Walk</span>
    <span id="lfStatusPill" class="pill ok" style="display:none">ACTIVE</span>
  </div>

  <div class="row" style="align-items:center;margin-bottom:6px">
    <div style="flex:1;min-width:70px">
      <label style="margin-bottom:2px">Steps (0=&infin;)</label>
      <input type="number" id="lfSteps" value="30" min="0" max="1000" style="margin-bottom:0">
    </div>
    <div style="flex:1;min-width:70px">
      <label style="margin-bottom:2px">Kp Steering</label>
      <input type="number" id="lfKp" value="0.35" step="0.05" min="0.05" max="1.5" style="margin-bottom:0">
    </div>
    <div style="flex:1.5;min-width:140px;display:flex;flex-direction:column;gap:3px;padding-top:6px">
      <label style="display:flex;align-items:center;gap:5px;font-size:.65rem;color:#c9d1d9;margin:0;cursor:pointer">
        <input type="checkbox" id="lfStopLost" checked style="width:auto;margin:0"> Stop if line lost
      </label>
      <label style="display:flex;align-items:center;gap:5px;font-size:.65rem;color:#f59e0b;margin:0;cursor:pointer">
        <input type="checkbox" id="lfInvertSteer" checked onchange="onSteerInvChange(this.checked)" style="width:auto;margin:0">
        Invert Steering (Flip L/R)
      </label>
      <label style="display:flex;align-items:center;gap:5px;font-size:.65rem;color:#a5d6ff;margin:0;cursor:pointer">
        <input type="checkbox" id="lfSwapSens" onchange="onSwapSensChange(this.checked)" style="width:auto;margin:0">
        Swap Sensor Order (S1&harr;S9)
      </label>
    </div>
  </div>

  <div class="row" style="margin-bottom:4px">
    <button class="go" id="lfStartBtn" style="flex:1" onclick="toggleLineFollow()">&#9654; Start Line Follow</button>
    <button class="re" id="lfStopBtn" style="display:none;flex:1" onclick="stopLineFollowCmd()">&#9209; STOP Line Follow</button>
  </div>

  <div id="lfInfoRow" style="font-size:.65rem;color:#58a6ff;font-family:monospace;min-height:16px;margin-top:4px;display:none"></div>
</div>
</div>

<!-- ═══ SECTION 4: TOF ═══════════════════════════════════════════════════════ -->
<div class="section">
<h2>&#128225; ToF Sensors (3 × VL53L0X)</h2>
<div class="tof-grid">
  <div class="tof-card">
    <div class="tof-name">Front</div>
    <div class="tof-val" id="tf0">—</div>
    <div class="tof-unit">mm</div>
    <div class="tof-dot" id="td0"></div>
  </div>
  <div class="tof-card">
    <div class="tof-name">Right</div>
    <div class="tof-val" id="tf1">—</div>
    <div class="tof-unit">mm</div>
    <div class="tof-dot" id="td1"></div>
  </div>
  <div class="tof-card">
    <div class="tof-name">Left</div>
    <div class="tof-val" id="tf2">—</div>
    <div class="tof-unit">mm</div>
    <div class="tof-dot" id="td2"></div>
  </div>
</div>
</div>

<!-- ═══ SECTION 5: GYRO / IMU (MPU6050) ═════════════════════════════════════ -->
<div class="section">
<div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:6px;padding-bottom:5px;border-bottom:1px solid #21262d">
  <h2 style="border-bottom:none;margin-bottom:0;padding-bottom:0">&#129517; Gyro / IMU (MPU6050)</h2>
  <div style="display:flex;gap:6px">
    <button class="pu" style="padding:4px 9px;font-size:.65rem" onclick="calGyroCmd()">&#127919; Calibrate Bias (Still)</button>
    <button class="pu" style="padding:4px 9px;font-size:.65rem" onclick="zeroYaw()">&#8634; Zero Yaw</button>
  </div>
</div>
<div class="tof-grid">
  <div class="tof-card">
    <div class="tof-name">Heading (Yaw)</div>
    <div class="tof-val" id="mpuYawVal">0.0°</div>
    <div class="tof-unit" id="mpuTurnSub">relative heading</div>
    <div class="tof-dot" id="mpuDot"></div>
  </div>
  <div class="tof-card">
    <div class="tof-name">Turn Rate (Gz)</div>
    <div class="tof-val" id="mpuGzVal">0.00</div>
    <div class="tof-unit">deg / sec (deadband 0.25)</div>
  </div>
  <div class="tof-card">
    <div class="tof-name">Bias &amp; Address</div>
    <div class="tof-val" id="mpuBiasVal" style="font-size:1.0rem;padding-top:4px">0.00</div>
    <div class="tof-unit" id="mpuAddrVal">0x68 &middot; SDA=11 SCL=12</div>
  </div>
</div>
<div id="mpuStatusMsg" style="font-size:.62rem;color:#8b949e;margin-top:6px;font-family:monospace;text-align:right">Drift suppression active</div>
</div>

<script>
// ---- Motion -----------------------------------------------------------------
function motionParams(extraSteps){
  let p = '?la='+document.getElementById('la').value
         +'&sa='+document.getElementById('sa').value
         +'&sm='+document.getElementById('sm').value
         +'&tsa='+document.getElementById('tsa').value;
  if(extraSteps !== undefined) p += '&steps='+extraSteps;
  return p;
}

async function cmd(url){
  setBusy(true);
  try{
    const r = await fetch(url, {signal: AbortSignal.timeout(90000)});
    const d = await r.json();
    setMsg(d.msg || JSON.stringify(d));
  } catch(e){ setMsg('Error: '+e.message); }
  setBusy(false);
}

function doWalk(){
  const steps = document.getElementById('ws').value;
  cmd('/walk'+motionParams(steps));
}

function doTurn(dir){
  const steps = parseInt(document.getElementById('ts').value);
  cmd('/turn?dir='+dir+motionParams(steps));
}

// ---- Closed-loop Gyro Turning -----------------------------------------------
let gyroRunning = false;
let gyroPollTimer = null;

async function doGyroTurn(dir, deg){
  if(gyroRunning){ stopGyroCmd(); return; }
  const la = document.getElementById('la').value;
  const sm = document.getElementById('sm').value;
  const tsa = document.getElementById('tsa').value;

  gyroRunning = true;
  showGyroStop(true);
  updateGyroProgress(0, deg, 0, 'Zeroing gyro heading & starting '+dir+' '+deg+'° turn...');
  setMsg('Zeroing gyro heading & starting '+dir+' '+deg+'° turn...');

  try{
    const url = '/turn?dir='+dir+'&deg='+deg+'&la='+la+'&sa='+tsa+'&sm='+sm;
    const r = await fetch(url, {signal: AbortSignal.timeout(10000)});
    const d = await r.json();
    if(!d.ok){ throw new Error(d.msg || 'Busy or rejected'); }

    if(gyroPollTimer) clearInterval(gyroPollTimer);
    gyroPollTimer = setInterval(pollGyroStatus, 250);
  } catch(e){
    stopGyroUI('Turn error: '+e.message);
  }
}

async function stopGyroCmd(){
  try{
    await fetch('/turn?stop=1', {signal: AbortSignal.timeout(3000)});
  } catch(e){}
  setMsg('Stopping gyro turn...');
}

async function pollGyroStatus(){
  try{
    const r = await fetch('/gyrostatus', {signal: AbortSignal.timeout(1200)});
    const d = await r.json();
    const pct = d.target > 0 ? Math.min(100, Math.max(0, Math.round((d.done / d.target) * 100))) : 0;
    updateGyroProgress(pct, d.target, d.done, d.phase, d.yaw);

    // Synchronize GYRO card heading immediately
    const yEl = document.getElementById('mpuYawVal');
    if(yEl && d.yaw !== undefined) yEl.textContent = (d.yaw >= 0 ? '+' : '') + d.yaw.toFixed(1) + '°';
    const subEl = document.getElementById('mpuTurnSub');
    if(subEl) subEl.textContent = d.running ? ('Turn: '+(d.done?d.done.toFixed(1):0)+'° / '+(d.target||0)+'°') : 'relative heading';

    if(!d.running){
      stopGyroUI('Gyro turn complete: turned '+(d.done ? d.done.toFixed(1) : '0')+'° (yaw: '+(d.yaw ? d.yaw.toFixed(1) : '0')+'°)');
    }
  } catch(e){}
}

function updateGyroProgress(pct, target, done, phase, yaw){
  const pBox = document.getElementById('gyroProgBox');
  if(pBox) pBox.style.display = 'block';
  const pBar = document.getElementById('gyroProgBar');
  if(pBar) pBar.style.width = pct + '%';
  const pText = document.getElementById('gyroProgText');
  if(pText){
    let t = 'Target: '+(target||0)+'° · Done: '+(done ? done.toFixed(1) : 0)+'° ('+pct+'%)';
    if(yaw !== undefined) t += ' · Live Yaw: '+(yaw>=0?'+':'')+yaw.toFixed(1)+'°';
    if(phase) t += ' · '+phase;
    pText.textContent = t;
  }
}

function stopGyroUI(msg){
  gyroRunning = false;
  if(gyroPollTimer){ clearInterval(gyroPollTimer); gyroPollTimer = null; }
  showGyroStop(false);
  const subEl = document.getElementById('mpuTurnSub');
  if(subEl) subEl.textContent = 'relative heading';
  if(msg) setMsg(msg);
}

function showGyroStop(show){
  const stopBtn = document.getElementById('gyroStopBtn');
  if(stopBtn) stopBtn.style.display = show ? 'block' : 'none';
}

async function zeroYaw(){
  try{
    const r = await fetch('/mpu/zero');
    const d = await r.json();
    setMsg(d.msg || 'Yaw zeroed');
    document.getElementById('mpuYawVal').textContent = '0.0°';
  } catch(e){ setMsg('Zero failed: '+e.message); }
}

async function calGyroCmd(){
  try{
    setMsg('Calibrating gyro bias — keep robot still for 2s...');
    document.getElementById('mpuStatusMsg').textContent = 'Calibrating bias...';
    const r = await fetch('/mpu/cal');
    const d = await r.json();
    setMsg(d.msg || 'Gyro calibration done');
    document.getElementById('mpuStatusMsg').textContent = 'Bias: ' + (d.bias ? d.bias.toFixed(1) : '0.0');
    if(d.bias !== undefined){
      document.getElementById('mpuBiasVal').textContent = d.bias.toFixed(1);
    }
  } catch(e){ setMsg('Gyro cal failed: '+e.message); }
}

// ---- Line Follow ------------------------------------------------------------
let lfRunning = false;
let lfPollTimer = null;

async function toggleLineFollow(){
  if(lfRunning){
    stopLineFollowCmd();
    return;
  }
  const steps = parseInt(document.getElementById('lfSteps').value) || 0;
  const kp = parseFloat(document.getElementById('lfKp').value) || 0.35;
  const stopLost = document.getElementById('lfStopLost').checked ? 1 : 0;
  const invSteer = document.getElementById('lfInvertSteer').checked ? 1 : 0;
  const swapSens = document.getElementById('lfSwapSens').checked ? 1 : 0;
  const sm = parseInt(document.getElementById('sm').value) || 140;
  const la = parseInt(document.getElementById('la').value) || 20;
  const sa = parseInt(document.getElementById('sa').value) || 15;

  lfRunning = true;
  showLfRunningUI(true);
  setMsg('Starting line follow (' + (steps === 0 ? 'infinite' : steps + ' steps') + ')...');

  try{
    const url = '/lf/start?steps=' + steps + '&kp=' + kp + '&lost=' + stopLost + '&inv=' + invSteer + '&swap=' + swapSens + '&sm=' + sm + '&la=' + la + '&sa=' + sa;
    const r = await fetch(url, {signal: AbortSignal.timeout(10000)});
    const d = await r.json();
    if(!d.ok){ throw new Error(d.msg || 'Busy'); }

    if(lfPollTimer) clearInterval(lfPollTimer);
    lfPollTimer = setInterval(pollLfStatus, 250);
  } catch(e){
    stopLfUI('Line follow error: ' + e.message);
  }
}

async function stopLineFollowCmd(){
  try{
    await fetch('/lf/stop', {signal: AbortSignal.timeout(3000)});
  } catch(e){}
  setMsg('Stopping line follow...');
}

async function pollLfStatus(){
  try{
    const r = await fetch('/lf/status', {signal: AbortSignal.timeout(1200)});
    const d = await r.json();
    const info = document.getElementById('lfInfoRow');
    if(info){
      info.textContent = d.status + (d.err !== undefined ? ' · Err: ' + d.err.toFixed(1) + ' mm' : '');
    }
    if(!d.running){
      stopLfUI(d.status || 'Line follow stopped');
    }
  } catch(e){}
}

function showLfRunningUI(running){
  document.getElementById('lfStartBtn').style.display = running ? 'none' : 'block';
  document.getElementById('lfStopBtn').style.display  = running ? 'block' : 'none';
  document.getElementById('lfStatusPill').style.display = running ? 'inline-block' : 'none';
  document.getElementById('lfInfoRow').style.display   = running ? 'block' : 'none';
}

function stopLfUI(msg){
  lfRunning = false;
  if(lfPollTimer){ clearInterval(lfPollTimer); lfPollTimer = null; }
  showLfRunningUI(false);
  if(msg) setMsg(msg);
}

async function onSteerInvChange(checked){
  try{
    await fetch('/lf/cfg?inv=' + (checked ? 1 : 0));
    setMsg('Steering inversion set to: ' + (checked ? 'Inverted' : 'Normal'));
  } catch(e){}
}

async function onSwapSensChange(checked){
  try{
    await fetch('/lf/cfg?swap=' + (checked ? 1 : 0));
    setMsg('Sensor order swap set to: ' + (checked ? 'Swapped (S9-S1)' : 'Standard (S1-S9)'));
  } catch(e){}
}

// ---- Line Array Calibration -------------------------------------------------
let isCalibrating = false;
let thrDebounceTimer = null;

async function startLineCal(){
  try{
    const r = await fetch('/line/cal/start');
    const d = await r.json();
    setMsg(d.msg || 'Sweep started — slide sensor over floor and line!');
    isCalibrating = true;
    showCalProgress(true);
  } catch(e){ setMsg('Cal start failed: '+e.message); }
}

async function stopLineCal(){
  try{
    const r = await fetch('/line/cal/stop');
    const d = await r.json();
    setMsg(d.msg || 'Calibration sweep finished');
    isCalibrating = false;
    showCalProgress(false);
  } catch(e){ setMsg('Cal stop failed: '+e.message); }
}

async function setCalFloor(){
  try{
    const r = await fetch('/line/cal/setfloor');
    const d = await r.json();
    setMsg(d.msg || 'Floor baseline captured');
  } catch(e){ setMsg('Set floor failed: '+e.message); }
}

async function setCalLine(){
  try{
    const r = await fetch('/line/cal/setline');
    const d = await r.json();
    setMsg(d.msg || 'Line peak captured');
  } catch(e){ setMsg('Set line failed: '+e.message); }
}

async function saveLineCal(){
  try{
    const r = await fetch('/line/save');
    const d = await r.json();
    setMsg(d.msg || 'Calibration saved to flash');
  } catch(e){ setMsg('Save failed: '+e.message); }
}

function onThrInput(v){
  const f = parseFloat(v);
  document.getElementById('thrNum').textContent = f.toFixed(2);
  document.getElementById('thrPct').textContent = Math.round(f * 100) + '%';
  // Move red dashed line <u> on all 9 cards immediately in real-time
  for(let i=0; i<9; i++){
    const th = document.getElementById('th' + i);
    if(th) th.style.bottom = Math.round(f * 100) + '%';
  }
}

function onThrChange(v){
  clearTimeout(thrDebounceTimer);
  thrDebounceTimer = setTimeout(async ()=>{
    try{
      await fetch('/line/setthr?thr='+v);
      setMsg('Threshold saved: '+v);
    } catch(e){}
    thrDebounceTimer = null;
  }, 100);
}

async function onWhChange(checked){
  try{
    await fetch('/line/setwh?wh='+(checked ? 1 : 0));
    setMsg('Polarity updated: white is ' + (checked ? 'HIGH' : 'LOW'));
  } catch(e){}
}

function showCalProgress(active){
  document.getElementById('calProgWrap').style.display = active ? 'block' : 'none';
  document.getElementById('calStartBtn').style.display = active ? 'none' : 'block';
  document.getElementById('calStopBtn').style.display  = active ? 'block' : 'none';
}

// ---- Arm --------------------------------------------------------------------
async function armCmd(url){
  const spd = document.getElementById('aspd').value;
  setBusy(true);
  try{
    const r = await fetch(url+'?spd='+spd, {signal: AbortSignal.timeout(30000)});
    const d = await r.json();
    setMsg(d.msg || JSON.stringify(d));
    if(d.lift !== undefined){ document.getElementById('liftSl').value=d.lift; document.getElementById('liftVal').textContent=d.lift+'°'; }
    if(d.grip !== undefined){ document.getElementById('gripSl').value=d.grip; document.getElementById('gripVal').textContent=d.grip+'°'; }
  } catch(e){ setMsg('Arm error: '+e.message); }
  setBusy(false);
}

function liftSlChg(){ document.getElementById('liftVal').textContent = document.getElementById('liftSl').value+'°'; }
function gripSlChg(){ document.getElementById('gripVal').textContent = document.getElementById('gripSl').value+'°'; }

function armSetLift(){
  const a = document.getElementById('liftSl').value;
  armCmd('/arm/setlift?a='+a);
}
function armSetGrip(){
  const a = document.getElementById('gripSl').value;
  armCmd('/arm/setgrip?a='+a);
}
function armSaveCfg(){
  const p = '?lup='+document.getElementById('lupIn').value
           +'&ldn='+document.getElementById('ldnIn').value
           +'&gop='+document.getElementById('gopIn').value
           +'&gcl='+document.getElementById('gclIn').value;
  armCmd('/arm/cfg'+p);
}

// ---- Status helpers ---------------------------------------------------------
function setBusy(b){
  document.getElementById('sdot').className = b ? 'dot busy' : 'dot';
}
function setMsg(m){ document.getElementById('smsg').textContent = m; }

// ---- Line array live indication (RUNNER4 Lab layout) -------------------------
function initLineSensors(){
  const c = document.getElementById('lineSensors');
  if(!c) return;
  c.innerHTML = '';
  for(let i=0; i<9; i++){
    const sc = document.createElement('div');
    sc.className = 'sc';
    sc.id = 'sc' + i;
    sc.innerHTML = '<div class="rv" id="rv'+i+'">0</div>'
                 + '<div class="bw"><i id="bar'+i+'" style="height:0%"></i><u id="th'+i+'" style="bottom:50%"></u></div>'
                 + '<div class="nv" id="nv'+i+'">0.00</div>'
                 + '<div class="sn">S'+(i+1)+'</div>';
    c.appendChild(sc);
  }
}

function updateLine(d){
  if(!d) return;

  // 1. Error & centroid
  const errEl = document.getElementById('lineErrVal');
  if(errEl){
    errEl.textContent = d.found ? (d.err > 0 ? '+' : '') + d.err.toFixed(1) + ' mm' : '—';
    errEl.style.color = d.found ? (Math.abs(d.err) > 20 ? '#ef4444' : '#38bdf8') : '#6e7681';
  }

  const actEl = document.getElementById('lineActVal');
  if(actEl) actEl.textContent = d.act + ' / 9';

  const cenEl = document.getElementById('lineCenVal');
  if(cenEl) cenEl.textContent = d.found ? d.cen.toFixed(2) : '—';

  const stEl = document.getElementById('lineStatusVal');
  const pillEl = document.getElementById('linePill');
  if(d.found){
    if(stEl){ stEl.textContent = 'LOCKED'; stEl.style.color = '#3fb950'; }
    if(pillEl){ pillEl.textContent = 'LINE OK'; pillEl.className = 'pill ok'; }
  } else {
    if(stEl){ stEl.textContent = 'LOST'; stEl.style.color = '#f85149'; }
    if(pillEl){ pillEl.textContent = 'LINE LOST'; pillEl.className = 'pill bad'; }
  }

  // 2. Sliding track marker (0.0=S1 left ... 8.0=S9 right)
  const mk = document.getElementById('mk');
  if(mk){
    const pct = d.found ? (d.cen / 8.0 * 100) : 50;
    mk.style.left = Math.max(2, Math.min(98, pct)) + '%';
    mk.style.opacity = d.found ? 1 : 0.25;
  }

  // 3. 9-Sensor cards
  for(let i=0; i<9; i++){
    const rv = document.getElementById('rv' + i);
    const nv = document.getElementById('nv' + i);
    const bar = document.getElementById('bar' + i);
    const th = document.getElementById('th' + i);
    const sc = document.getElementById('sc' + i);

    if(d.raw && rv) rv.textContent = d.raw[i];
    if(d.norm && nv){
      const normVal = d.norm[i] / 1000.0;
      nv.textContent = normVal.toFixed(2);
      if(bar) bar.style.height = Math.round(normVal * 100) + '%';
    }
    if(th && d.thr !== undefined) th.style.bottom = Math.round(d.thr * 100) + '%';
    if(sc && d.mask !== undefined){
      sc.className = 'sc' + ((d.mask >> i) & 1 ? ' on' : '');
    }
  }

  // 4. Calibration status & feedback
  const calPill = document.getElementById('lineCalPill');
  if(calPill){
    calPill.textContent = d.cal ? 'CALIBRATED' : 'NOT CALIBRATED';
    calPill.className = 'pill ' + (d.cal ? 'ok' : 'bad');
  }
  const msgEl = document.getElementById('calMsgText');
  if(msgEl && d.calmsg) msgEl.textContent = d.calmsg;

  // 5. Calibration progress bar
  if(d.calact){
    showCalProgress(true);
    const leftMs = d.calleft || 0;
    const pct = Math.max(0, Math.min(100, Math.round((1 - leftMs / 5000.0) * 100)));
    document.getElementById('calProgFill').style.width = pct + '%';
    document.getElementById('calTimeLeft').textContent = (leftMs / 1000.0).toFixed(1) + 's left';
  } else if(isCalibrating){
    showCalProgress(false);
    isCalibrating = false;
    document.getElementById('calTimeLeft').textContent = '';
  }

  // 6. Sync threshold slider if not user dragging
  if(d.thr !== undefined && !thrDebounceTimer){
    const sl = document.getElementById('thrSlider');
    if(sl && Math.abs(parseFloat(sl.value) - d.thr) > 0.04){
      sl.value = d.thr;
      document.getElementById('thrNum').textContent = d.thr.toFixed(2);
      document.getElementById('thrPct').textContent = Math.round(d.thr * 100) + '%';
    }
  }
  if(d.wh !== undefined){
    const whCb = document.getElementById('whCheck');
    if(whCb) whCb.checked = d.wh;
  }
}

// ---- ToF live update --------------------------------------------------------
function updateTof(d){
  const names=['front','right','left'];
  for(let i=0;i<3;i++){
    const ok = d[names[i]+'_ok'];
    document.getElementById('td'+i).className = 'tof-dot'+(ok?' on':'');
    document.getElementById('tf'+i).textContent = ok ? Math.round(d[names[i]]) : '—';
  }
}

// ---- Gyro live update -------------------------------------------------------
function updateMpu(d){
  const yEl = document.getElementById('mpuYawVal');
  if(yEl && d.yaw !== undefined) yEl.textContent = (d.yaw >= 0 ? '+' : '') + d.yaw.toFixed(1) + '°';
  const gzEl = document.getElementById('mpuGzVal');
  if(gzEl && d.gz !== undefined) gzEl.textContent = (d.gz >= 0 ? '+' : '') + d.gz.toFixed(2);
  const dot = document.getElementById('mpuDot');
  if(dot) dot.className = 'tof-dot' + (d.ready ? ' on' : '');
  const biasEl = document.getElementById('mpuBiasVal');
  if(biasEl && d.bias !== undefined) biasEl.textContent = (d.bias >= 0 ? '+' : '') + d.bias.toFixed(1);
  const addrEl = document.getElementById('mpuAddrVal');
  if(addrEl && d.addr) addrEl.textContent = (d.ready ? d.addr : 'OFFLINE') + ' · SDA=11 SCL=12';
}

// ---- Sensor Polling ---------------------------------------------------------
async function pollSensors(){
  try{
    const [lr, tr, mr] = await Promise.all([
      fetch('/linedata',{signal:AbortSignal.timeout(1500)}),
      fetch('/tofdata', {signal:AbortSignal.timeout(1500)}),
      fetch('/mpudata', {signal:AbortSignal.timeout(1500)})
    ]);
    if(lr.ok){ const d=await lr.json(); updateLine(d); }
    if(tr.ok){ const d=await tr.json(); updateTof(d); }
    if(mr.ok){ const d=await mr.json(); updateMpu(d); }
  }catch(e){}
}

initLineSensors();
setInterval(pollSensors, 350);

// Initial sync
(async()=>{
  try{
    const [ar, lfc] = await Promise.all([
      fetch('/arm/status'),
      fetch('/lf/status')
    ]);
    if(ar.ok){
      const d = await ar.json();
      if(d.lift!==undefined){ document.getElementById('liftSl').value=d.lift; document.getElementById('liftVal').textContent=d.lift+'°'; }
      if(d.grip!==undefined){ document.getElementById('gripSl').value=d.grip; document.getElementById('gripVal').textContent=d.grip+'°'; }
      if(d.lup!==undefined)  document.getElementById('lupIn').value=d.lup;
      if(d.ldn!==undefined)  document.getElementById('ldnIn').value=d.ldn;
      if(d.gop!==undefined)  document.getElementById('gopIn').value=d.gop;
      if(d.gcl!==undefined)  document.getElementById('gclIn').value=d.gcl;
    }
  }catch(e){}
})();
</script>
</body></html>
)html";

// ─── HTTP helpers ─────────────────────────────────────────────────────────────
void cors() { server.sendHeader("Access-Control-Allow-Origin","*"); server.sendHeader("Cache-Control","no-store"); }

void sendBusy() {
    cors(); server.send(200,"application/json","{\"ok\":false,\"msg\":\"busy\"}");
}

void applyGaitFromArgs() {
    if (server.hasArg("la"))  LIFT_ANGLE = constrain(server.arg("la").toInt(), 5, 60);
    if (server.hasArg("sa"))  SWING_ANGLE = constrain(server.arg("sa").toInt(), 5, 60);
    if (server.hasArg("sm"))  SETTLE_MS   = constrain(server.arg("sm").toInt(), 40, 800);
    if (server.hasArg("tsa")) TURN_SWING_ANGLE = constrain(server.arg("tsa").toInt(), 5, 40);
}

// ─── Motion handlers ──────────────────────────────────────────────────────────
void hRoot()  { server.send_P(200,"text/html",PAGE); }

void hStand() {
    if(busy){sendBusy();return;}
    cors(); server.send(200,"application/json","{\"ok\":true,\"msg\":\"Standing up...\"}");
    busy=true; standUp(); busy=false;
}

void hHome() {
    if(busy){sendBusy();return;}
    cors(); server.send(200,"application/json","{\"ok\":true,\"msg\":\"Going HOME\"}");
    busy=true;
    snprintf(statusMsg,sizeof(statusMsg),"Going HOME...");
    applyPose(HOME_POSE);
    snprintf(statusMsg,sizeof(statusMsg),"HOME — idle");
    busy=false;
}

void hWalk() {
    if(busy){sendBusy();return;}
    int steps = server.hasArg("steps") ? constrain(server.arg("steps").toInt(),1,200) : 10;
    applyGaitFromArgs();
    cors();
    char msg[80];
    snprintf(msg,sizeof(msg),"Walk %d steps (lift=%d° swing=%d° settle=%dms)",steps,LIFT_ANGLE,SWING_ANGLE,SETTLE_MS);
    server.send(200,"application/json",String("{\"ok\":true,\"msg\":\"")+msg+"\"}");
    busy=true;
    snprintf(statusMsg,sizeof(statusMsg),"%s",msg);
    walkForward(steps);
    snprintf(statusMsg,sizeof(statusMsg),"Walk done — idle");
    busy=false;
}

void hTurn() {
    // 1. Immediate stop command
    if (server.hasArg("stop")) {
        gyroTurnStop = true;
        cors();
        server.send(200, "application/json", "{\"ok\":true,\"msg\":\"Stopping turn...\"}");
        return;
    }

    if (busy) { sendBusy(); return; }
    applyGaitFromArgs();

    String dir = server.hasArg("dir") ? server.arg("dir") : "right";
    bool right = (dir == "right" || dir == "cw");

    // 2. Closed-loop gyro turn requested
    if (server.hasArg("deg") || server.hasArg("preset")) {
        float deg = 90.0f;
        if (server.hasArg("deg")) deg = server.arg("deg").toFloat();
        else if (server.hasArg("preset")) deg = server.arg("preset").toFloat();
        deg = constrain(deg, 5.0f, 360.0f);

        cors();
        char msg[80];
        snprintf(msg, sizeof(msg), "Gyro turn %s %.1f° (lift=%d° swing=%d°)",
                 right ? "right" : "left", deg, TURN_LIFT_ANGLE, TURN_SWING_ANGLE);
        server.send(200, "application/json", String("{\"ok\":true,\"msg\":\"") + msg + "\"}");

        busy = true;
        snprintf(statusMsg, sizeof(statusMsg), "%s", msg);
        gyroTurnWith(right ? TURN_R_SIGN : TURN_L_SIGN, deg);
        snprintf(statusMsg, sizeof(statusMsg), "Gyro turn done (%.1f°) — idle", gyroTurnDone);
        busy = false;
        return;
    }

    // 3. Fallback: manual step-based turn
    int steps = server.hasArg("steps") ? constrain(server.arg("steps").toInt(), 1, 200) : STEPS_90;
    cors();
    char msg[80];
    snprintf(msg, sizeof(msg), "Turn %s %d steps (lift=%d° swing=%d°)",
             right ? "right" : "left", steps, TURN_LIFT_ANGLE, TURN_SWING_ANGLE);
    server.send(200, "application/json", String("{\"ok\":true,\"msg\":\"") + msg + "\"}");

    busy = true;
    snprintf(statusMsg, sizeof(statusMsg), "%s", msg);
    if (right) turnRight(steps); else turnLeft(steps);
    snprintf(statusMsg, sizeof(statusMsg), "Turn done — idle");
    busy = false;
}

// ─── Line Follow Handlers ─────────────────────────────────────────────────────
void hLfStart() {
    if (busy) { sendBusy(); return; }
    int steps = server.hasArg("steps") ? server.arg("steps").toInt() : 30;
    if (server.hasArg("kp"))   lfKp = constrain(server.arg("kp").toFloat(), 0.05f, 1.5f);
    if (server.hasArg("lost")) lfStopLost = (server.arg("lost").toInt() != 0);
    if (server.hasArg("inv"))  lfSteerSign = (server.arg("inv").toInt() != 0) ? -1 : 1;
    if (server.hasArg("swap")) lfSwapSens  = (server.arg("swap").toInt() != 0);
    applyGaitFromArgs();

    cors();
    char msg[80];
    snprintf(msg, sizeof(msg), "Line follow: %d steps (Kp=%.2f, steerSign=%d)", steps, lfKp, lfSteerSign);
    server.send(200, "application/json", String("{\"ok\":true,\"msg\":\"") + msg + "\"}");

    busy = true;
    snprintf(statusMsg, sizeof(statusMsg), "%s", msg);
    lineFollowWalk(steps);
    snprintf(statusMsg, sizeof(statusMsg), "Line follow ended — idle");
    busy = false;
}

void hLfStop() {
    lfStop = true;
    cors();
    server.send(200, "application/json", "{\"ok\":true,\"msg\":\"Stopping line follow...\"}");
}

void hLfCfg() {
    cors();
    if (server.hasArg("inv"))  lfSteerSign = (server.arg("inv").toInt() != 0) ? -1 : 1;
    if (server.hasArg("swap")) lfSwapSens  = (server.arg("swap").toInt() != 0);
    if (server.hasArg("kp"))   lfKp        = constrain(server.arg("kp").toFloat(), 0.05f, 1.5f);
    lineSaveConfig();
    server.send(200, "application/json", "{\"ok\":true,\"msg\":\"Line follow config updated\"}");
}

void hLfStatus() {
    cors();
    char j[250];
    snprintf(j, sizeof(j),
        "{\"running\":%s,\"steps\":%d,\"target\":%d,\"err\":%.1f,\"found\":%s,\"status\":\"%s\"}",
        lfRunning ? "true" : "false",
        lfStepsDone, lfTargetSteps,
        lineErrMm, lineFound ? "true" : "false",
        lfStatusStr);
    server.send(200, "application/json", j);
}

// ─── Line Array Calibration Handlers ──────────────────────────────────────────
void hLineCalStart() {
    cors();
    lineStartCal(5000);
    server.send(200, "application/json", "{\"ok\":true,\"msg\":\"Calibration sweep started (5s)\"}");
}

void hLineCalStop() {
    cors();
    lineFinishCal();
    server.send(200, "application/json", "{\"ok\":true,\"msg\":\"Calibration finished\"}");
}

void hLineCalSetFloor() {
    cors();
    for (int i = 0; i < LINE_N; i++) {
        uint16_t v = (uint16_t)analogRead(LINE_PINS[i]);
        lineMin[i] = v;
    }
    lineCalibrated = true;
    snprintf(lineCalMsg, sizeof(lineCalMsg), "Floor baseline set from current readings");
    lineSaveConfig();
    server.send(200, "application/json", "{\"ok\":true,\"msg\":\"Floor baseline set\"}");
}

void hLineCalSetLine() {
    cors();
    for (int i = 0; i < LINE_N; i++) {
        uint16_t v = (uint16_t)analogRead(LINE_PINS[i]);
        lineMax[i] = v;
    }
    lineCalibrated = true;
    snprintf(lineCalMsg, sizeof(lineCalMsg), "Line peak set from current readings");
    lineSaveConfig();
    server.send(200, "application/json", "{\"ok\":true,\"msg\":\"Line peak set\"}");
}

void hLineSetThr() {
    cors();
    if (server.hasArg("thr")) {
        lineThr = constrain(server.arg("thr").toFloat(), 0.05f, 0.95f);
        lineSaveConfig();
        server.send(200, "application/json", "{\"ok\":true,\"msg\":\"Threshold updated\"}");
    } else {
        server.send(400, "application/json", "{\"ok\":false,\"msg\":\"missing thr\"}");
    }
}

void hLineSetWh() {
    cors();
    if (server.hasArg("wh")) {
        lineWhiteHigh = (server.arg("wh").toInt() != 0);
        lineSaveConfig();
        server.send(200, "application/json", "{\"ok\":true,\"msg\":\"Polarity updated\"}");
    } else {
        server.send(400, "application/json", "{\"ok\":false,\"msg\":\"missing wh\"}");
    }
}

void hLineSave() {
    cors();
    lineSaveConfig();
    server.send(200, "application/json", "{\"ok\":true,\"msg\":\"Calibration saved to flash\"}");
}

// ─── ARM handlers ─────────────────────────────────────────────────────────────
void sendArmState(const char* msg="") {
    cors();
    char j[200];
    snprintf(j,sizeof(j),
        "{\"ok\":true,\"msg\":\"%s\",\"lift\":%d,\"grip\":%d,\"lup\":%d,\"ldn\":%d,\"gop\":%d,\"gcl\":%d}",
        msg, armLiftPos, armGripPos, armLiftUp, armLiftDown, armGripOpen, armGripClose);
    server.send(200,"application/json",j);
}

void hArmStatus() { sendArmState("arm status"); }

void hArmUp() {
    if(busy){sendBusy();return;}
    if(server.hasArg("spd")) armSpeedMs = constrain(server.arg("spd").toInt(),1,100);
    busy=true; sweepLift(armLiftUp); busy=false;
    sendArmState("Arm UP");
}
void hArmDown() {
    if(busy){sendBusy();return;}
    if(server.hasArg("spd")) armSpeedMs = constrain(server.arg("spd").toInt(),1,100);
    busy=true; sweepGrip(armGripOpen); delay(200); sweepLift(armLiftDown); busy=false;
    sendArmState("Arm DOWN (opened first)");
}
void hArmOpen() {
    if(busy){sendBusy();return;}
    if(server.hasArg("spd")) armSpeedMs = constrain(server.arg("spd").toInt(),1,100);
    busy=true; sweepGrip(armGripOpen); busy=false;
    sendArmState("Gripper OPEN");
}
void hArmClose() {
    if(busy){sendBusy();return;}
    if(server.hasArg("spd")) armSpeedMs = constrain(server.arg("spd").toInt(),1,100);
    busy=true; sweepGrip(armGripClose); busy=false;
    sendArmState("Gripper CLOSE");
}
void hArmSetLift() {
    if(!server.hasArg("a")){cors();server.send(400,"application/json","{\"error\":\"missing a\"}");return;}
    int a = constrain(server.arg("a").toInt(),20,160);
    if(server.hasArg("spd")) armSpeedMs = constrain(server.arg("spd").toInt(),1,100);
    if(busy){sendBusy();return;}
    busy=true; sweepLift(a); busy=false;
    sendArmState("Lift set");
}
void hArmSetGrip() {
    if(!server.hasArg("a")){cors();server.send(400,"application/json","{\"error\":\"missing a\"}");return;}
    int a = constrain(server.arg("a").toInt(),0,180);
    if(server.hasArg("spd")) armSpeedMs = constrain(server.arg("spd").toInt(),1,100);
    if(busy){sendBusy();return;}
    busy=true; sweepGrip(a); busy=false;
    sendArmState("Grip set");
}
void hArmPick() {
    if(busy){sendBusy();return;}
    if(server.hasArg("spd")) armSpeedMs = constrain(server.arg("spd").toInt(),1,100);
    cors(); server.send(200,"application/json","{\"ok\":true,\"msg\":\"Pick sequence...\"}");
    busy=true;
    sweepGrip(armGripOpen); delay(200);
    sweepLift(armLiftDown);  delay(200);
    sweepGrip(armGripClose); delay(200);
    sweepLift(armLiftUp);
    busy=false;
}
void hArmPlace() {
    if(busy){sendBusy();return;}
    if(server.hasArg("spd")) armSpeedMs = constrain(server.arg("spd").toInt(),1,100);
    cors(); server.send(200,"application/json","{\"ok\":true,\"msg\":\"Place sequence...\"}");
    busy=true;
    sweepLift(armLiftDown);  delay(200);
    sweepGrip(armGripOpen);  delay(200);
    sweepLift(armLiftUp);    delay(200);
    sweepGrip(armGripClose);
    busy=false;
}
void hArmCfg() {
    if(server.hasArg("lup")) armLiftUp   = constrain(server.arg("lup").toInt(),20,160);
    if(server.hasArg("ldn")) armLiftDown = constrain(server.arg("ldn").toInt(),20,160);
    if(server.hasArg("gop")) armGripOpen = constrain(server.arg("gop").toInt(),0,180);
    if(server.hasArg("gcl")) armGripClose= constrain(server.arg("gcl").toInt(),0,180);
    sendArmState("Config saved (RAM only)");
}

// ─── Sensor data handlers ─────────────────────────────────────────────────────
void hLineData() {
    cors();
    char buf[300];
    String j = "{\"raw\":[";
    for(int i=0;i<LINE_N;i++){ j += lineRaw[i]; if(i<LINE_N-1) j += ","; }
    j += "],\"norm\":[";
    for(int i=0;i<LINE_N;i++){ j += (int)(lineNorm[i]*1000.0f); if(i<LINE_N-1) j += ","; }

    uint32_t now = millis();
    uint32_t calLeft = (lineCalActive && lineCalUntil > now) ? (lineCalUntil - now) : 0;

    snprintf(buf, sizeof(buf),
        "],\"thr\":%.2f,\"mask\":%u,\"act\":%u,\"found\":%s,\"cen\":%.2f,\"err\":%.1f,"
        "\"cal\":%s,\"calact\":%s,\"calleft\":%u,\"calmsg\":\"%s\",\"wh\":%s}",
        lineThr, lineMask, lineActCnt, lineFound ? "true" : "false",
        lineCentroid, lineErrMm,
        lineCalibrated ? "true" : "false",
        lineCalActive ? "true" : "false",
        calLeft, lineCalMsg,
        lineWhiteHigh ? "true" : "false");
    j += buf;
    server.send(200,"application/json",j);
}

void hTofData() {
    cors();
    char b[200];
    snprintf(b,sizeof(b),
        "{\"front_ok\":%s,\"right_ok\":%s,\"left_ok\":%s,"
        "\"front\":%.1f,\"right\":%.1f,\"left\":%.1f}",
        tofOk[0]?"true":"false",tofOk[1]?"true":"false",tofOk[2]?"true":"false",
        tofDist[0],tofDist[1],tofDist[2]);
    server.send(200,"application/json",b);
}

void hMpuData() {
    cors();
    char j[140];
    snprintf(j, sizeof(j),
        "{\"ready\":%s,\"yaw\":%.2f,\"gz\":%.3f,\"cal\":%s,\"addr\":\"0x%02X\",\"bias\":%.2f}",
        mpuReady ? "true" : "false", mpuYaw, mpuGz,
        mpuCalibrated ? "true" : "false", mpuAddr, mpuGBiasZ);
    server.send(200, "application/json", j);
}

void hMpuCal() {
    cors();
    if (busy) { sendBusy(); return; }
    busy = true;
    snprintf(statusMsg, sizeof(statusMsg), "Calibrating MPU6050 gyro bias...");
    mpuCalibrate(400);
    snprintf(statusMsg, sizeof(statusMsg), "Gyro calibrated (bias=%.2f)", mpuGBiasZ);
    busy = false;

    char j[140];
    snprintf(j, sizeof(j),
        "{\"ok\":true,\"msg\":\"Gyro calibrated successfully\",\"bias\":%.2f,\"ready\":%s}",
        mpuGBiasZ, mpuReady ? "true" : "false");
    server.send(200, "application/json", j);
}

void hGyroStatus() {
    cors();
    char j[250];
    snprintf(j, sizeof(j),
        "{\"running\":%s,\"yaw\":%.2f,\"target\":%.1f,\"done\":%.1f,\"ready\":%s,\"phase\":\"%s\"}",
        gyroTurnRunning ? "true" : "false",
        mpuYaw, gyroTurnTarget, gyroTurnDone,
        mpuReady ? "true" : "false", gyroTurnPhase);
    server.send(200, "application/json", j);
}

void hMpuZero() {
    mpuYaw = 0.0f;
    cors();
    server.send(200, "application/json", "{\"ok\":true,\"msg\":\"Yaw zeroed to 0.0°\"}");
}

// ─── setup ───────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(I2C_HZ);
    Wire.setTimeOut(50);

    // PCA9685
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);
    delay(10);

    Serial.println("[MinTest] INITIAL pose");
    applyPose(HOME_POSE);
    delay(200); // allow robot body to completely settle before gyro calibration

    // Arm LEDC
    armInitLedc();
    rawLift(armLiftPos);
    rawGrip(armGripPos);
    Serial.printf("[ARM] Lift=%d Grip=%d\n", armLiftPos, armGripPos);

    // ToF sensors
    tofInit();

    // MPU6050 Gyro (calibrated while resting still)
    mpuInit();

    // Line array calibration load from NVS
    lineLoadConfig();

    // Line array ADC pins
    for(int i=0;i<LINE_N;i++) pinMode(LINE_PINS[i], INPUT);
    Serial.println("[LINE] ADC pins ready");

    // WiFi Access Point
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    WiFi.setSleep(false);
    Serial.printf("[WIFI] AP: %s  IP: %s\n", AP_SSID,
                  WiFi.softAPIP().toString().c_str());

    // HTTP Routes
    server.on("/",           hRoot);
    server.on("/stand",      hStand);
    server.on("/home",       hHome);
    server.on("/walk",       hWalk);
    server.on("/turn",       hTurn);
    // Line follow
    server.on("/lf/start",   hLfStart);
    server.on("/lf/stop",    hLfStop);
    server.on("/lf/cfg",     hLfCfg);
    server.on("/lf/status",  hLfStatus);
    // Line calibration
    server.on("/line/cal/start",    hLineCalStart);
    server.on("/line/cal/stop",     hLineCalStop);
    server.on("/line/cal/setfloor", hLineCalSetFloor);
    server.on("/line/cal/setline",  hLineCalSetLine);
    server.on("/line/setthr",       hLineSetThr);
    server.on("/line/setwh",        hLineSetWh);
    server.on("/line/save",         hLineSave);
    // Arm
    server.on("/arm/status", hArmStatus);
    server.on("/arm/up",     hArmUp);
    server.on("/arm/down",   hArmDown);
    server.on("/arm/open",   hArmOpen);
    server.on("/arm/close",  hArmClose);
    server.on("/arm/setlift",hArmSetLift);
    server.on("/arm/setgrip",hArmSetGrip);
    server.on("/arm/pick",   hArmPick);
    server.on("/arm/place",  hArmPlace);
    server.on("/arm/cfg",    hArmCfg);
    // Sensors
    server.on("/linedata",   hLineData);
    server.on("/tofdata",    hTofData);
    server.on("/mpudata",    hMpuData);
    server.on("/mpu/cal",    hMpuCal);
    server.on("/gyrostatus", hGyroStatus);
    server.on("/mpu/zero",   hMpuZero);

    server.begin();
    Serial.println("[MinTest] Ready at http://192.168.4.1");
}

// ─── loop ────────────────────────────────────────────────────────────────────
void loop() {
    server.handleClient();
    tofPoll();
    linePoll();
    mpuPoll();
}
