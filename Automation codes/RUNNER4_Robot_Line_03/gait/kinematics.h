#pragma once
// =============================================================================
// gait/kinematics.h — 3-DOF Quadruped Forward and Inverse Kinematics
//
// Solves Cartesian foot coordinates (x, y, z) <-> Joint angles (hip, femur, tibia)
// for RUNNER4 leg geometry.
//
// GEOMETRY (from config/robot_config.h):
//   L1_MM: Coxa length (30.0 mm) — hip pivot to femur pivot
//   L2_MM: Femur length (60.0 mm) — femur pivot to tibia pivot
//   L3_MM: Tibia length (80.0 mm) — tibia pivot to foot tip
// =============================================================================

#include <Arduino.h>
#include <math.h>
#include "../config/robot_config.h"

struct Vec3f {
  float x, y, z;
};

// Forward Kinematics: Joint angles (degrees) -> Cartesian foot position (mm)
// mirror: +1 for Left legs (FL, BL), -1 for Right legs (FR, BR)
inline Vec3f computeFK(int hipDeg, int femurDeg, int tibiaDeg, int mirror) {
  const float a1 = (hipDeg - 90) * DEG_TO_RAD;
  const float a2 = (femurDeg - 90) * DEG_TO_RAD;
  const float a3 = (tibiaDeg - 90) * DEG_TO_RAD;
  const float reach = L1_MM + L2_MM * cosf(a2) + L3_MM * cosf(a2 + a3);
  Vec3f p;
  p.x = reach * cosf(a1);
  p.y = reach * sinf(a1) * (float)mirror;
  p.z = L2_MM * sinf(a2) + L3_MM * sinf(a2 + a3);
  return p;
}

// Inverse Kinematics: Cartesian foot position (mm) -> Joint angles (degrees)
// Returns true if reachable, false if outside workspace.
inline bool computeIK(float px, float py, float pz, int mirror, int *hipOut,
                      int *femurOut, int *tibiaOut) {
  py *= (float)mirror;
  const float a1 = atan2f(py, px);
  const float r = sqrtf(px * px + py * py) - L1_MM;
  const float d = sqrtf(r * r + pz * pz);
  if (d > L2_MM + L3_MM - 0.5f || d < fabsf(L2_MM - L3_MM) + 0.5f)
    return false;
  float cosA3 = constrain((d * d - L2_MM * L2_MM - L3_MM * L3_MM) /
                              (2.0f * L2_MM * L3_MM),
                          -1.0f, 1.0f);
  const float a3 = acosf(cosA3);
  const float a2 =
      atan2f(pz, r) - atan2f(L3_MM * sinf(a3), L2_MM + L3_MM * cosf(a3));
  int hip = (int)roundf(a1 * RAD_TO_DEG) + 90;
  int femur = (int)roundf(a2 * RAD_TO_DEG) + 90;
  int tibia = (int)roundf(a3 * RAD_TO_DEG) + 90;
  if (hip < 0 || hip > 180 || femur < 0 || femur > 180 || tibia < 0 ||
      tibia > 180)
    return false;
  *hipOut = hip;
  *femurOut = femur;
  *tibiaOut = tibia;
  return true;
}
