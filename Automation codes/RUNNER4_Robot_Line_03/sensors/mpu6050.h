#pragma once
// =============================================================================
// sensors/mpu6050.h  — MPU6050 / MPU6500 / MPU9250 Driver for RUNNER4 (header-only)
//
// Features:
//   - Auto-detects I2C address (0x68 or 0x69, auto-probes both)
//   - Repeated-start I2C reads with fallback
//   - Broad WHO_AM_I support (MPU-6050, MPU-6500, MPU-9250, clones)
//   - Power reset + PLL X-axis gyro clock selection (low drift)
//   - Background auto-reconnect if sensor is connected after boot
//   - Live I2C bus scanning endpoint (/mpu/rescan)
//   - EMA filtering + yaw integration (100 Hz)
// =============================================================================

#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>
#include <Wire.h>
#include <WebServer.h>
#include <math.h>

#ifndef MPU6050_I2C_ADDR
#define MPU6050_I2C_ADDR  0x68
#endif

#define MPU_ACC_ALPHA      0.90f   // EMA for accelerometer (90% old)
#define MPU_GYRO_ALPHA     0.90f   // EMA for gyroscope     (90% old)
#define MPU_DRIFT_THRESH   3.5f    // Gz below this -> allow drift correction (deg/s)
#define MPU_POLL_INTERVAL  10      // ms between mpuPoll() reads (100 Hz)
#define MPU_RETRY_INTERVAL 2500    // ms between auto-detect retries when offline
#define MPU_TURN_TOL_DEG   3.0f    // Gyro turn tolerance
#define MPU_TURN_MAX_OVERSHOOT 15.0f

typedef struct {
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x,  gyro_y,  gyro_z;
    int16_t temp;
} MPURawData;

typedef struct {
    float accel_x_g, accel_y_g, accel_z_g;
    float gyro_x_dps, gyro_y_dps, gyro_z_dps;
} MPUScaledData;

#ifdef MPU_IMPLEMENTATION

static bool     _mpuReady          = false;
static bool     _mpuCalibrated     = false;
static bool     _mpuStationary     = false;
static bool     _mpuFilterInit     = false;
static uint8_t  _mpuAddr           = 0x68;   // Auto-detected 0x68 or 0x69
static uint8_t  _mpuWhoAmI         = 0x00;
static float    _mpuYaw            = 0.0f;   // integrated heading degrees

// Calibration biases (raw LSB)
static float _mpuABiasX = 0, _mpuABiasY = 0, _mpuABiasZ = 0;
static float _mpuGBiasX = 0, _mpuGBiasY = 0, _mpuGBiasZ = 0;

// EMA filter state
static float _fAX = 0, _fAY = 0, _fAZ = 0;
static float _fGX = 0, _fGY = 0, _fGZ = 0;

// Timing
static uint32_t _mpuLastPollMs  = 0;
static uint32_t _mpuLastRetryMs = 0;

// ── Low-level register write ──────────────────────────────────────────────
static bool _mpuWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(_mpuAddr);
    Wire.write(reg);
    Wire.write(val);
    return (Wire.endTransmission() == 0);
}

// ── Raw burst read (14 bytes from 0x3B) ──────────────────────────────────
static bool _mpuReadRaw(MPURawData &d) {
    if (!_mpuReady) return false;
    Wire.beginTransmission(_mpuAddr);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0) {
        Wire.beginTransmission(_mpuAddr);
        Wire.write(0x3B);
        if (Wire.endTransmission() != 0) return false;
    }
    uint8_t count = Wire.requestFrom((uint8_t)_mpuAddr, (uint8_t)14);
    if (count < 14 || Wire.available() < 14) return false;

    uint8_t buf[14];
    for (int i = 0; i < 14; i++) buf[i] = Wire.read();
    d.accel_x = (int16_t)((buf[0]  << 8) | buf[1]);
    d.accel_y = (int16_t)((buf[2]  << 8) | buf[3]);
    d.accel_z = (int16_t)((buf[4]  << 8) | buf[5]);
    d.temp    = (int16_t)((buf[6]  << 8) | buf[7]);
    d.gyro_x  = (int16_t)((buf[8]  << 8) | buf[9]);
    d.gyro_y  = (int16_t)((buf[10] << 8) | buf[11]);
    d.gyro_z  = (int16_t)((buf[12] << 8) | buf[13]);
    return true;
}

// ── Calibration ───────────────────────────────────────────────────────────
static void _mpuCalibrate(uint16_t samples = 600) {
    MPURawData raw;
    Serial.printf("[MPU] Calibrating on 0x%02X (%u samples)...\n", _mpuAddr, samples);
    for (int i = 0; i < 30; i++) { _mpuReadRaw(raw); delay(3); } // discard warmup
    int64_t axS=0,ayS=0,azS=0,gxS=0,gyS=0,gzS=0; uint16_t n=0;
    for (uint16_t i = 0; i < samples; i++) {
        if (!_mpuReadRaw(raw)) { delay(3); continue; }
        axS+=raw.accel_x; ayS+=raw.accel_y; azS+=raw.accel_z;
        gxS+=raw.gyro_x;  gyS+=raw.gyro_y;  gzS+=raw.gyro_z;
        n++; delay(3);
    }
    if (n < 50) { Serial.println("[MPU] Cal FAILED — too few valid readings"); return; }
    _mpuABiasX = (float)axS/n; _mpuABiasY = (float)ayS/n;
    _mpuABiasZ = (float)azS/n - 16384.0f;   // correct for 1g
    _mpuGBiasX = (float)gxS/n; _mpuGBiasY = (float)gyS/n; _mpuGBiasZ = (float)gzS/n;
    _mpuCalibrated  = true;
    _mpuFilterInit  = false;   // re-seed EMA
    _mpuYaw         = 0.0f;
    Serial.printf("[MPU] Cal complete: gBiasZ=%.2f, aBiasZ=%.1f\n", _mpuGBiasZ, _mpuABiasZ);
}

// ── Hardware init with address auto-probing ───────────────────────────────
static bool _mpuHwInit() {
    uint8_t candidateAddresses[] = {0x68, 0x69};
    bool foundAck = false;

    for (uint8_t addr : candidateAddresses) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            _mpuAddr = addr;
            foundAck = true;
            Serial.printf("[MPU] Device ACK at address 0x%02X\n", _mpuAddr);
            break;
        }
    }

    if (!foundAck) {
        Serial.println("[MPU] Check wiring: No I2C ACK at 0x68 or 0x69 on SDA=11, SCL=12");
        return false;
    }

    // Read WHO_AM_I (Register 0x75)
    Wire.beginTransmission(_mpuAddr);
    Wire.write(0x75);
    if (Wire.endTransmission(false) != 0) {
        Wire.beginTransmission(_mpuAddr);
        Wire.write(0x75);
        Wire.endTransmission();
    }
    uint8_t n = Wire.requestFrom((uint8_t)_mpuAddr, (uint8_t)1);
    if (n > 0 && Wire.available()) {
        _mpuWhoAmI = Wire.read();
        Serial.printf("[MPU] WHO_AM_I register = 0x%02X\n", _mpuWhoAmI);
    } else {
        Serial.println("[MPU] Warning: WHO_AM_I read returned 0 bytes, proceeding with init...");
    }

    // 1. Device Reset (Bit 7 of PWR_MGMT_1)
    _mpuWrite(0x6B, 0x80);
    delay(50);

    // 2. Wake up & select PLL with X-axis gyroscope clock (0x01 is much more stable than 0x00)
    _mpuWrite(0x6B, 0x01);
    delay(50);

    // 3. Enable all 3 accel & 3 gyro axes in PWR_MGMT_2 (0x00)
    _mpuWrite(0x6C, 0x00);
    delay(10);

    // 4. Sample Rate Divider: 1000 / (1 + 9) = 100 Hz
    _mpuWrite(0x19, 0x09);

    // 5. DLPF (Digital Low Pass Filter) ~44 Hz bandwidth
    _mpuWrite(0x1A, 0x03);

    // 6. Gyroscope Range: ±500 °/s (sensitivity 65.5 LSB/(°/s))
    _mpuWrite(0x1B, 0x08);

    // 7. Accelerometer Range: ±2 g (sensitivity 16384 LSB/g)
    _mpuWrite(0x1C, 0x00);

    delay(50);
    return true;
}

// ── EMA poll + yaw integration ────────────────────────────────────────────
static void _mpuUpdateFilter(float dt) {
    MPURawData raw;
    if (!_mpuReadRaw(raw)) return;

    float ax = (raw.accel_x - _mpuABiasX) / 16384.0f;
    float ay = (raw.accel_y - _mpuABiasY) / 16384.0f;
    float az = (raw.accel_z - _mpuABiasZ) / 16384.0f;
    float gx = (raw.gyro_x  - _mpuGBiasX) / 65.5f;
    float gy = (raw.gyro_y  - _mpuGBiasY) / 65.5f;
    float gz = (raw.gyro_z  - _mpuGBiasZ) / 65.5f;

    if (!_mpuFilterInit) {
        _fAX=ax; _fAY=ay; _fAZ=az;
        _fGX=gx; _fGY=gy; _fGZ=gz;
        _mpuFilterInit = true;
        return;
    }
    _fAX = MPU_ACC_ALPHA*_fAX  + (1.0f-MPU_ACC_ALPHA)*ax;
    _fAY = MPU_ACC_ALPHA*_fAY  + (1.0f-MPU_ACC_ALPHA)*ay;
    _fAZ = MPU_ACC_ALPHA*_fAZ  + (1.0f-MPU_ACC_ALPHA)*az;
    _fGX = MPU_GYRO_ALPHA*_fGX + (1.0f-MPU_GYRO_ALPHA)*gx;
    _fGY = MPU_GYRO_ALPHA*_fGY + (1.0f-MPU_GYRO_ALPHA)*gy;
    _fGZ = MPU_GYRO_ALPHA*_fGZ + (1.0f-MPU_GYRO_ALPHA)*gz;

    // Stationary drift correction
    if (_mpuStationary && fabsf(_fGZ) < MPU_DRIFT_THRESH) {
        float alpha = dt / 2.0f;
        if (alpha > 1.0f) alpha = 1.0f;
        _mpuGBiasZ = (1.0f-alpha)*_mpuGBiasZ + alpha*(float)raw.gyro_z;
    }

    // Integrate yaw (Z axis positive = CCW when sensor Z points up)
    _mpuYaw += _fGZ * dt;
}

// ── JSON data builder ─────────────────────────────────────────────────────
static String _mpuJson() {
    char buf[300];
    snprintf(buf, sizeof(buf),
        "{\"ok\":%s,\"ready\":%s,\"addr\":\"0x%02X\",\"who\":\"0x%02X\","
        "\"yaw\":%.2f,\"gz\":%.3f,\"gx\":%.3f,\"gy\":%.3f,"
        "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"cal\":%s}",
        _mpuReady ? "true" : "false",
        _mpuReady ? "true" : "false",
        _mpuAddr, _mpuWhoAmI,
        _mpuYaw, _fGZ, _fGX, _fGY,
        _fAX, _fAY, _fAZ,
        _mpuCalibrated ? "true" : "false");
    return String(buf);
}

// ── Live I2C scan on Wire (GPIO 11/12) ────────────────────────────────────
static String _mpuI2cScanJson() {
    String j = "[";
    bool first = true;
    for (uint8_t a = 1; a < 127; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) {
            if (!first) j += ",";
            char hex[8]; snprintf(hex, sizeof(hex), "\"0x%02X\"", a);
            j += hex;
            first = false;
        }
    }
    j += "]";
    return j;
}

#endif  // MPU_IMPLEMENTATION

// ---------------------------------------------------------------------------
// Public API declarations (always visible)
// ---------------------------------------------------------------------------
bool  mpuBegin(WebServer &server, const char *ssid);
void  mpuPoll();
float mpuGetYaw();
float mpuGetGz();
bool  mpuIsReady();
bool  mpuIsCalibrated();
void  mpuResetYaw();
void  mpuSetStationary(bool s);
bool  mpuTurnToAngle(float targetDeg, bool rightTurn);

// ---------------------------------------------------------------------------
// Implementation
// ---------------------------------------------------------------------------
#ifdef MPU_IMPLEMENTATION

void turnGait(int steps, int speedMs, bool rightTurn);
void serviceBackground();
extern volatile bool compStop;

float mpuGetYaw()         { return _mpuYaw; }
float mpuGetGz()          { return _fGZ; }
bool  mpuIsReady()        { return _mpuReady; }
bool  mpuIsCalibrated()   { return _mpuCalibrated; }
void  mpuResetYaw()       { _mpuYaw = 0.0f; }
void  mpuSetStationary(bool s) { _mpuStationary = s; }

void mpuPoll() {
    uint32_t now = millis();

    // Auto-detect retry if offline
    if (!_mpuReady) {
        if (now - _mpuLastRetryMs >= MPU_RETRY_INTERVAL) {
            _mpuLastRetryMs = now;
            if (_mpuHwInit()) {
                _mpuReady = true;
                _mpuCalibrate(400);
                Serial.println("[MPU] Auto-detected and connected successfully!");
            }
        }
        return;
    }

    if (now - _mpuLastPollMs < MPU_POLL_INTERVAL) return;
    float dt = (now - _mpuLastPollMs) / 1000.0f;
    if (dt > 0.1f) dt = 0.1f;
    _mpuLastPollMs = now;
    _mpuUpdateFilter(dt);
}

bool mpuTurnToAngle(float targetDeg, bool rightTurn) {
    if (!_mpuReady) {
        Serial.println("[MPU] Not ready — skipping gyro turn");
        return true;
    }
    float startYaw = _mpuYaw;
    float target   = rightTurn ? startYaw - targetDeg : startYaw + targetDeg;
    float maxOver  = targetDeg + MPU_TURN_MAX_OVERSHOOT;
    Serial.printf("[MPU] TurnTo %.1f deg (%s), yaw=%.1f target=%.1f\n",
                  targetDeg, rightTurn?"CW":"CCW", startYaw, target);

    uint32_t tStart = millis();
    while (!compStop) {
        mpuPoll();
        float turned = rightTurn ? (startYaw - _mpuYaw) : (_mpuYaw - startYaw);
        if (turned >= targetDeg - MPU_TURN_TOL_DEG) {
            Serial.printf("[MPU] Turn done: turned=%.1f deg in %lu ms\n",
                          turned, millis() - tStart);
            return true;
        }
        if (turned > maxOver) {
            Serial.printf("[MPU] Turn OVERSHOOT: %.1f deg — stopping\n", turned);
            return false;
        }
        serviceBackground();
        delay(2);
    }
    return false;
}

static WebServer *_mpuSrv = nullptr;

static void _mpuHandleData() {
    _mpuSrv->sendHeader("Cache-Control","no-store");
    _mpuSrv->sendHeader("Access-Control-Allow-Origin", "*");
    _mpuSrv->send(200,"application/json", _mpuJson());
}
static void _mpuHandleZero() {
    mpuResetYaw();
    _mpuSrv->sendHeader("Cache-Control","no-store");
    _mpuSrv->send(200,"application/json","{\"ok\":true,\"msg\":\"yaw zeroed\"}");
}
static void _mpuHandleCal() {
    _mpuSrv->sendHeader("Cache-Control","no-store");
    if (!_mpuReady) {
        _mpuHwInit();
    }
    if (_mpuHwInit()) {
        _mpuReady = true;
        _mpuCalibrate(800);
        _mpuSrv->send(200,"application/json","{\"ok\":true,\"msg\":\"calibrated\"}");
    } else {
        _mpuSrv->send(200,"application/json","{\"ok\":false,\"msg\":\"sensor not found\"}");
    }
}
static void _mpuHandleRescan() {
    _mpuSrv->sendHeader("Cache-Control","no-store");
    _mpuSrv->sendHeader("Access-Control-Allow-Origin", "*");
    bool ok = _mpuHwInit();
    if (ok) {
        _mpuReady = true;
        _mpuCalibrate(300);
    }
    String json = "{\"ok\":" + String(_mpuReady ? "true" : "false") +
                  ",\"addr\":\"0x" + String(_mpuAddr, HEX) + "\"" +
                  ",\"who\":\"0x" + String(_mpuWhoAmI, HEX) + "\"" +
                  ",\"i2c_scan\":" + _mpuI2cScanJson() + "}";
    _mpuSrv->send(200, "application/json", json);
}

bool mpuBegin(WebServer &server, const char *ssid) {
    _mpuSrv = &server;
    Serial.println("[MPU] Probing MPU6050 on Wire (GPIO 11 SDA, GPIO 12 SCL)...");
    _mpuReady = _mpuHwInit();
    if (_mpuReady) {
        Serial.printf("[MPU] Sensor initialized at 0x%02X (WHO_AM_I=0x%02X)\n", _mpuAddr, _mpuWhoAmI);
        _mpuCalibrate(400);
    } else {
        Serial.println("[MPU] Offline at boot (will auto-retry in background every 2.5s)");
    }

    server.on("/mpu/data",   HTTP_GET, _mpuHandleData);
    server.on("/mpu/status", HTTP_GET, _mpuHandleData);
    server.on("/mpu/zero",   HTTP_GET, _mpuHandleZero);
    server.on("/mpu/cal",    HTTP_GET, _mpuHandleCal);
    server.on("/mpu/rescan", HTTP_GET, _mpuHandleRescan);

    _mpuLastPollMs = millis();
    _mpuLastRetryMs = millis();
    return _mpuReady;
}

#endif  // MPU_IMPLEMENTATION
#endif  // MPU6050_H