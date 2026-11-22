#pragma once
// =============================================================================
// gait/servo_controller.h — 12-DOF Quadruped Servo Driver & Interpolation
// Engine
//
// PCA9685 I2C servo controller, 12-servo channel mappings, preset poses,
// and linear multi-servo interpolation engine.
// =============================================================================

#include "../config/robot_config.h"
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <Wire.h>

// Forward declarations of background service (defined in main .ino)
extern void serviceBackground();
extern void waitMs(uint32_t ms);

// -- PCA9685 Driver Instance (I2C address 0x40 on main Wire bus) --------------
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// -- Channel Map & Mirroring --------------------------------------------------
// legChannels[leg][0=hip, 1=femur, 2=tibia]; leg: 0=FL, 1=FR, 2=BL, 3=BR
const uint8_t legChannels[4][3] = {{FL_HIP, FL_FEMUR, FL_TIBIA},
                                   {FR_HIP, FR_FEMUR, FR_TIBIA},
                                   {BL_HIP, BL_FEMUR, BL_TIBIA},
                                   {BR_HIP, BR_FEMUR, BR_TIBIA}};
const int legMirror[4] = {1, -1, 1, -1};

// -- Current Live Servo Angles (16 channels) ----------------------------------
int servoAngles[16] = {0};

// -- Pose presets [pose][leg][hip,femur,tibia] — editable from web UI ---------
#define POSE_COUNT 13

int poseAngles[13][4][3] = {
    // 0 INITIAL (HOME)
    {{45, 70, 139}, {94, 80, 25}, {94, 70, 5}, {45, 92, 160}},
    // 1 STANDING
    {{45, 15, 100}, {94, 140, 70}, {74, 133, 60}, {34, 35, 120}},
    // 2 LOW CROUCH
    {{45, 110, 140}, {94, 110, 40}, {94, 110, 140}, {45, 110, 140}},
    // 3 LOW STAND
    {{45, 29, 119}, {94, 112, 30}, {94, 86, 40}, {45, 62, 130}},
    // 4 FL LEG UP
    {{45, 38, 115}, {94, 125, 35}, {94, 101, 45}, {45, 80, 160}},
    // 5 FR LEG UP
    {{45, 80, 160}, {94, 55, 20}, {94, 56, 160}, {45, 92, 160}},
    // 6 TROT A
    {{45, 60, 120}, {94, 80, 0}, {94, 56, 160}, {45, 72, 140}},
    // 7 TROT B
    {{45, 80, 160}, {94, 55, 20}, {94, 36, 130}, {45, 92, 160}},
    // 8 STRETCH
    {{45, 90, 90}, {94, 90, 90}, {94, 90, 90}, {45, 90, 90}},
    // 9 WALK A basic
    {{15, 45, 115}, {94, 135, 45}, {94, 111, 55}, {75, 57, 115}},
    // 10 WALK B basic
    {{15, 20, 115}, {124, 115, 45}, {64, 91, 55}, {75, 32, 115}},
    // 11 WALK A enhanced
    {{15, 45, 130}, {94, 135, 45}, {94, 111, 55}, {75, 57, 130}},
    // 12 WALK B enhanced
    {{15, 20, 115}, {124, 115, 30}, {64, 91, 30}, {75, 32, 115}}};

// =============================================================================
// SERVO HARDWARE HELPERS
// =============================================================================

inline uint16_t angleToPulse(int angle) {
  angle = constrain(angle, 0, 180);
  return (uint16_t)map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

inline void setServo(uint8_t ch, int angle) {
  angle = constrain(angle, 0, 180);
  servoAngles[ch] = angle;
  pwm.setPWM(ch, 0, angleToPulse(angle));
}

inline void setAllServosHome() {
  Serial.println("[HOME] All 12 servos -> INITIAL pose");
  for (int leg = 0; leg < 4; leg++)
    for (int joint = 0; joint < 3; joint++) {
      setServo(legChannels[leg][joint], poseAngles[0][leg][joint]);
      waitMs(10); // stagger to reduce current spike
    }
}

inline void setLegHome(int leg) {
  for (int joint = 0; joint < 3; joint++)
    setServo(legChannels[leg][joint], poseAngles[0][leg][joint]);
}

// Linear interpolation of all 12 servos over speedMs (0 = instant)
//
// PERFORMANCE FIX:
//  - Instant mode (speedMs == 0): writes all servos immediately, single
//  background tick.
//  - Timed mode: uses fixed number of steps (max 30) capped at one step per
//  ~speedMs/30 ms,
//    preventing HTTP requests from stalling motion.
inline void moveToTarget(const int targetAngles[4][3], int speedMs) {
  if (speedMs <= 0) {
    // Instant: write all 12 servos immediately, no per-joint delay
    for (int leg = 0; leg < 4; leg++)
      for (int joint = 0; joint < 3; joint++)
        setServo(legChannels[leg][joint], targetAngles[leg][joint]);
    serviceBackground(); // single background tick after all writes
    return;
  }

  // Compute max angular travel to decide number of steps
  int maxDiff = 0;
  for (int leg = 0; leg < 4; leg++)
    for (int joint = 0; joint < 3; joint++) {
      int diff =
          abs(targetAngles[leg][joint] - servoAngles[legChannels[leg][joint]]);
      if (diff > maxDiff)
        maxDiff = diff;
    }
  if (maxDiff == 0)
    return;

  // Cap interpolation steps: no more than 30 (was up to 180).
  const int MAX_STEPS = 30;
  int nSteps = (maxDiff < MAX_STEPS) ? maxDiff : MAX_STEPS;

  int startAngles[16];
  memcpy(startAngles, servoAngles, sizeof(servoAngles));
  unsigned long moveStart = millis();

  for (int step = 1; step <= nSteps; step++) {
    float t = (float)step / (float)nSteps;
    for (int leg = 0; leg < 4; leg++)
      for (int joint = 0; joint < 3; joint++) {
        uint8_t ch = legChannels[leg][joint];
        setServo(ch, startAngles[ch] +
                         (int)roundf(
                             t * (targetAngles[leg][joint] - startAngles[ch])));
      }
    // Wait until this step's scheduled time, running background tasks
    unsigned long stepTarget =
        moveStart + (unsigned long)((long)speedMs * step / nSteps);
    unsigned long now = millis();
    if (stepTarget > now)
      waitMs(stepTarget - now);
    else
      serviceBackground();
  }
}

inline void moveToPose(int id, int speedMs) {
  if (id < 0 || id >= POSE_COUNT)
    return;
  moveToTarget(poseAngles[id], speedMs);
}
