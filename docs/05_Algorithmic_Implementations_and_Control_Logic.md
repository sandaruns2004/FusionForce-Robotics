# 05. Algorithmic Implementations & Control Logic

## 1. Executive Summary & Algorithmic Architecture

This document provides the mathematical formulations, control loops, and reference code implementations for the core autonomous capabilities of the **FusionForce Quadruped**:
1. **OpenCV Vision Pipeline:** High-speed line following, grid cross detection, and HSV color segmentation.
2. **PD Wall-Following & Gap Rejection Algorithm:** Narrow corridor centering with discontinuity filtering.
3. **3-DOF Leg Inverse Kinematics (IK) & Gait Generator:** Deterministic foot trajectory planning for Trot and Crawl gaits.

---

## 2. OpenCV Vision Pipeline (Python / Raspberry Pi 4B)

### 2.1 Mathematical Centroid & Heading Error Derivation
To track the ground guideline, the camera frame $(640\times 480)$ is sliced into a horizontal Region of Interest (ROI) at the bottom 30% of the image ($y \in [336, 480]$). 
Let $I(u, v)$ be the binary thresholded guideline image where line pixels are $1$ and background pixels are $0$.

The image moments are defined as:

$$M_{ij} = \sum_{u} \sum_{v} u^i v^j I(u, v)$$

The horizontal centroid coordinate $u_c$ of the line is:

$$u_c = \frac{M_{10}}{M_{00}}$$

The heading error $e_{\text{heading}}$ relative to the image center ($u_{\text{center}} = 320$) is normalized to $[-1.0, +1.0]$:

$$e_{\text{heading}} = \frac{u_c - 320}{320}$$

The required angular steering velocity $\omega_z$ (deg/s) is computed via a Discrete Proportional-Derivative (PD) controller:

$$\omega_z(t) = K_p \cdot e_{\text{heading}}(t) + K_d \cdot \frac{e_{\text{heading}}(t) - e_{\text{heading}}(t-1)}{\Delta t}$$

where recommended tuning gains are $K_p = 45.0$ and $K_d = 12.0$.

---

### 2.2 HSV Color Segmentation & Calibration Table

For Task #01 (Ball Color Recognition) and Task #04 (Color Sorting Junction), RGB camera frames are converted to HSV color space to eliminate illumination sensitivity.

| Target Color ID | Hue Range ($H \in [0, 179]$) | Saturation Range ($S \in [0, 255]$) | Value Range ($V \in [0, 255]$) | Competition Assignment |
| :--- | :--- | :--- | :--- | :--- |
| **`RED_01`** (Low Hue) | `0 – 10` | `120 – 255` | `70 – 255` | Task #01 Ball / Task #04 Right Zone |
| **`RED_02`** (High Hue) | `165 – 179` | `120 – 255` | `70 – 255` | Wraparound Red hue spectrum |
| **`GREEN`** | `40 – 85` | `100 – 255` | `70 – 255` | Task #01 Ball / Task #04 Center Zone|
| **`BLUE`** | `95 – 135` | `120 – 255` | `70 – 255` | Task #01 Ball / Task #04 Left Zone  |

---

### 2.3 Reference Python Mission & Vision Controller (`MasterMissionController.py`)

```python
#!/usr/bin/env python3
"""
MasterMissionController.py
Raspberry Pi 4B High-Level AI & Mission Engine for 12-Servo Quadruped
"""

import time
import struct
import cv2
import numpy as np
import serial

class UARTIPCClient:
    def __init__(self, port='/dev/serial0', baudrate=115200):
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=0.02)
        
    @staticmethod
    def calc_crc8(data: bytes) -> int:
        crc = 0x00
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = ((crc << 1) ^ 0x31) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
        return crc

    def send_velocity_cmd(self, vx: int, vy: int, omega: int, gait: int):
        """
        Sends CMD_SET_VELOCITY (0x01) frame:
        [0xAA, 0xBB, 0x01, 0x07, vx(2B), vy(2B), omega(2B), gait(1B), CRC]
        """
        payload = struct.pack('<hhhB', vx, vy, omega, gait)
        header = bytes([0xAA, 0xBB, 0x01, len(payload)])
        crc = self.calc_crc8(header[2:] + payload)
        frame = header + payload + bytes([crc])
        self.ser.write(frame)

    def send_arm_cmd(self, action_id: int):
        """
        Sends CMD_ARM_ACTION (0x02): 0=HOME, 1=GRAB, 2=STORE, 3=RELEASE, 4=PUSH
        """
        payload = bytes([action_id])
        header = bytes([0xAA, 0xBB, 0x02, len(payload)])
        crc = self.calc_crc8(header[2:] + payload)
        self.ser.write(header + payload + bytes([crc]))


class VisionLineFollower:
    def __init__(self):
        self.kp = 45.0
        self.kd = 12.0
        self.prev_error = 0.0
        self.last_time = time.time()
        self.stored_ball_color = "NONE"

    def process_frame(self, frame_bgr):
        h, w, _ = frame_bgr.shape
        roi = frame_bgr[int(h * 0.7):h, 0:w]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 65, 255, cv2.THRESH_BINARY_INV)

        M = cv2.moments(thresh)
        if M["m00"] > 500:
            cx = int(M["m10"] / M["m00"])
            error = (cx - (w / 2.0)) / (w / 2.0)
        else:
            error = self.prev_error

        now = time.time()
        dt = max(now - self.last_time, 0.001)
        derivative = (error - self.prev_error) / dt
        omega_z = int(self.kp * error + self.kd * derivative)

        self.prev_error = error
        self.last_time = now
        return error, omega_z

    def classify_ball_color(self, frame_bgr) -> str:
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        mask_red1 = cv2.inRange(hsv, (0, 120, 70), (10, 255, 255))
        mask_red2 = cv2.inRange(hsv, (165, 120, 70), (179, 255, 255))
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        mask_green = cv2.inRange(hsv, (40, 100, 70), (85, 255, 255))
        mask_blue = cv2.inRange(hsv, (95, 120, 70), (135, 255, 255))

        counts = {
            "RED": cv2.countNonZero(mask_red),
            "GREEN": cv2.countNonZero(mask_green),
            "BLUE": cv2.countNonZero(mask_blue)
        }
        best_color = max(counts, key=counts.get)
        if counts[best_color] > 500:
            self.stored_ball_color = best_color
            return best_color
        return "NONE"
```

---

## 3. PD Wall-Following & Gap Rejection Algorithm (Corridor Tasks #02 & #03)

### 3.1 Corridor Centering Mathematics
Let $d_{\text{left}}$ and $d_{\text{right}}$ be the filtered ToF sensor distances in millimeters.
The centerline error $e_{\text{corridor}}$ is defined as:

$$e_{\text{corridor}}(t) = d_{\text{left}}(t) - d_{\text{right}}(t)$$

A positive error means the robot is closer to the right wall and must steer left ($\omega_z > 0$).

### 3.2 Outlier Rejection Filter for Wall Gaps
In Task #02, a gap appears in either the left or right wall ($d > 400\text{ mm}$). If fed directly into $e_{\text{corridor}}$, the robot would steer violently into the gap.

```
       WALL FOLLOWING WITH GAP REJECTION REFLEX (Top View)
       
       ======================= Solid Left Wall =======================
                                                          
                                     /-- Gap in Right Wall --\
       -----------------------------+                         +-------
               d_left = 120 mm      |
       [ROBOT] <====================|  <-- Filter rejects d_right = 650 mm
               d_right = 120 mm     |      Locks steering to single wall!
       -----------------------------+                         +-------
       ======================= Solid Right Wall ======================
```

#### Mathematical Filter Rule:
$$\text{IF } d_{\text{left}} > 400\text{ mm AND } d_{\text{right}} \le 250\text{ mm}: \quad e_{\text{corridor}} = 2 \cdot (125 - d_{\text{right}})$$
$$\text{IF } d_{\text{right}} > 400\text{ mm AND } d_{\text{left}} \le 250\text{ mm}: \quad e_{\text{corridor}} = 2 \cdot (d_{\text{left}} - 125)$$
$$\text{IF } d_{\text{left}} \le 400\text{ mm AND } d_{\text{right}} \le 400\text{ mm}: \quad e_{\text{corridor}} = d_{\text{left}} - d_{\text{right}}$$

---

## 4. 3-DOF Leg IK & Bezier Gait Generator (C++ / STM32F401)

### 4.1 Bezier Step Trajectory Equation
During the swing phase ($t_{\text{phase}} \in [0, 1]$), foot elevation $z_{\text{foot}}(t)$ is parameterized by a quadratic Bezier curve to ensure zero vertical velocity at lift-off and landing:

$$z_{\text{foot}}(u) = -Z_{\text{stand}} + 4 \cdot H_{\text{step}} \cdot u \cdot (1 - u)$$

where $Z_{\text{stand}} = 80.0\text{ mm}$ is body height and $H_{\text{step}} = 25.0\text{ mm}$ is swing height.

### 4.2 Reference C++ STM32 Inverse Kinematics Solver (`QuadrupedIKSolver.cpp`)

```cpp
/**
 * QuadrupedIKSolver.cpp
 * STM32F401 Real-Time 3-DOF Leg IK Solver & UART Packet Parser
 */

#include <math.h>
#include <stdint.h>

#define PI 3.141592653589793f
#define L_COXA  30.0f  // mm
#define L_FEMUR 55.0f  // mm
#define L_TIBIA 60.0f  // mm

struct LegAngles {
    float coxa_deg;
    float femur_deg;
    float tibia_deg;
    bool valid;
};

/**
 * @brief Solves 3-DOF Denavit-Hartenberg Inverse Kinematics for a single leg.
 * @param x Forward/backward coordinate (mm)
 * @param y Lateral left/right coordinate (mm)
 * @param z Vertical upward coordinate (mm, negative below hip point)
 * @return LegAngles struct containing servo angles in degrees.
 */
LegAngles solveLegIK(float x, float y, float z) {
    LegAngles angles = {0.0f, 0.0f, 0.0f, true};

    // 1. Coxa Angle (Hip Yaw)
    angles.coxa_deg = atan2f(y, x) * (180.0f / PI);
    float R = sqrtf(x * x + y * y) - L_COXA;

    // 2. Straight-line distance D from hip pitch to foot tip
    float D = sqrtf(R * R + z * z);
    if (D > (L_FEMUR + L_TIBIA - 0.5f)) {
        angles.valid = false; // Target unreachable / over-extended
        D = (L_FEMUR + L_TIBIA - 0.5f);
    }

    // 3. Tibia Angle (Knee Pitch - Law of Cosines)
    float cos_gamma = (L_FEMUR * L_FEMUR + L_TIBIA * L_TIBIA - D * D) / 
                      (2.0f * L_FEMUR * L_TIBIA);
    if (cos_gamma < -1.0f) cos_gamma = -1.0f;
    if (cos_gamma > 1.0f)  cos_gamma = 1.0f;
    float gamma = acosf(cos_gamma);
    angles.tibia_deg = 180.0f - (gamma * (180.0f / PI));

    // 4. Femur Angle (Hip Pitch)
    float alpha = atan2f(-z, R);
    float cos_beta = (L_FEMUR * L_FEMUR + D * D - L_TIBIA * L_TIBIA) / 
                     (2.0f * L_FEMUR * D);
    if (cos_beta < -1.0f) cos_beta = -1.0f;
    if (cos_beta > 1.0f)  cos_beta = 1.0f;
    float beta = acosf(cos_beta);
    angles.femur_deg = (alpha - beta) * (180.0f / PI);

    return angles;
}

/**
 * @brief Calculates MAXIM CRC-8 checksum for UART packet verification.
 */
uint8_t calculateCRC8(const uint8_t* data, uint8_t length) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ 0x31);
            } else {
                crc = (uint8_t)(crc << 1);
            }
        }
    }
    return crc;
}
```
