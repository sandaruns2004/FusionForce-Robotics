# SENSOR PERCEPTION

> [!NOTE]
> This document replaces the former `COMPUTER_VISION.md`. All camera-based OpenCV perception has been removed. The RUNNER-4 now uses two embedded sensors for all perception tasks: an 8-channel TCRT5000 IR line array and a TCS34725 RGBC colour sensor.

## 1. 8-Channel TCRT5000 IR Line Array

### 1.1 Hardware
- **Sensor Type**: 8× TCRT5000 reflective infrared sensor (digital output)
- **Array Width**: ~70mm total; sensors spaced ~8.75mm centre-to-centre
- **Output**: Digital HIGH (1) = white/reflective surface detected; LOW (0) = black/absorptive surface
- **Power**: 3.3V or 5V (check module VCC rating); connect to STM32 3.3V rail
- **Mount Position**: Front-underside of robot body, centred on midline
- **Mount Height**: 5–8mm above floor surface (closer = more sensitive; too close = physical contact risk)
- **Interface**: 8 GPIO input pins on STM32 (PA0–PA7), configured as INPUT with internal pull-down

### 1.2 Line Following Algorithm — Weighted Centroid
Given the 8-bit reading bitmask (S1=bit0 ... S8=bit7):

```c
// Sensor index i: 0 (leftmost) to 7 (rightmost)
// val[i]: 1 if sensor active (white), 0 if not (black)

float sum_w = 0, sum_v = 0;
for (int i = 0; i < 8; i++) {
    sum_w += (float)val[i];
    sum_v += (float)(i * val[i]);
}

if (sum_w == 0) return NAN;  // All black — line lost

float centroid = sum_v / sum_w;      // Range 0.0 to 7.0
float error    = centroid - 3.5f;    // Range –3.5 to +3.5
                                     // Negative = line is LEFT of centre
                                     // Positive = line is RIGHT of centre
```

This `error` value is fed to the PD line-follower controller to generate `Wz` (angular velocity).

### 1.3 Intersection and Junction Detection

| Scenario | Typical Active Count | Pattern |
|----------|---------------------|---------|
| Straight line, centred | 3–4 sensors | Middle sensors active |
| Curve or corner | 2–5 sensors | Offset cluster |
| **T-junction or + cross** | **6–8 sensors** | **Wide activation** |
| Ball pedestal base (small circle) | 1–2 sensors | Edge only (ToF confirms ball) |
| Line lost | 0 sensors | All LOW |

**Intersection detection rule** (temporal filter prevents false triggers):
```c
// Called every 20ms in main loop
static uint8_t consecutive_count = 0;

uint8_t active = __builtin_popcount(line_bitmask);  // count HIGH bits
if (active >= 6) {
    consecutive_count++;
    if (consecutive_count >= 3) {
        return true;  // Confirmed intersection
    }
} else {
    consecutive_count = 0;  // Reset on partial activation
}
```

This requires ≥6 sensors active for ≥3 consecutive 20ms cycles (60ms minimum hold) — a straight 30mm line can only activate ≤4 sensors, so the margin is sufficient.

### 1.4 Line Lost Failsafe
```c
static uint32_t all_black_count = 0;

if (LineArray_IsAllBlack()) {
    all_black_count++;
    if (all_black_count >= 150) {  // 150 × 20ms = 3 seconds
        StateMachine_Transition(STATE_SAFE_STOP);
    }
} else {
    all_black_count = 0;
}
```

### 1.5 Calibration Procedure (Competition Day)
1. Power on STM32 with debug UART connected.
2. Place array over plain black surface → all sensors should read LOW. If any read HIGH, increase digital threshold potentiometer for that sensor.
3. Place array over 30mm white matte paper → sensors covering the white region should read HIGH. Verify 3–4 centre sensors activate.
4. Slide array sideways to confirm edge sensors trigger cleanly.
5. Log centroid values: centroid ≈ 3.5 when centred is correct.

---

## 2. TCS34725 RGBC Colour Sensor

### 2.1 Hardware
- **Type**: TCS34725 RGBC (Red, Green, Blue, Clear) 16-bit per channel
- **Interface**: I2C1, 7-bit address **0x29**
- **Power**: 3.3V (connect to STM32 3.3V)
- **Built-in LED**: White LED illuminator for consistent lighting; controlled via STM32 **PC0** (GPIO output)
- **Mount Position**: Gripper arm tip (see dual-mode below)
- **Effective range**: 1–3cm for reliable colour discrimination
- **Light shield**: Small 3D-printed shroud around sensor to block ambient arena light

### 2.2 Dual-Mode Arm Positioning

```
MODE A — Ball Colour Reading (Task 01):
┌──────────────────────────────────────────────────────┐
│  Arm pitch servo → 0° (horizontal forward)           │
│  Robot stops at front ToF < 80mm + intersection      │
│  Arm lowers to 5cm pedestal height                   │
│  TCS34725 sensor tip is 1–2cm from ball surface      │
│  LED ON → 50ms integration → read RGBC → classify    │
│  Arm returns to home → result stored in ball_color   │
└──────────────────────────────────────────────────────┘

MODE B — Floor Zone Reading (Task 04):
┌──────────────────────────────────────────────────────┐
│  Arm pitch servo → −70° (pointing at floor)          │
│  Robot pauses at 3-way junction (≥6 sensors, ≥3cy)  │
│  TCS34725 sensor tip is 1–3cm above floor line       │
│  LED ON → 50ms integration → read RGBC → classify   │
│  Compare result with stored ball_color → select branch│
│  Arm returns to home → robot turns onto matching branch│
└──────────────────────────────────────────────────────┘
```

**Pre-calibrated PWM constants:**
```c
#define ARM_ANGLE_HOME    90   // Degrees — resting/travel position
#define ARM_ANGLE_BALL   -10   // Degrees — forward, near pedestal height
#define ARM_ANGLE_FLOOR  -70   // Degrees — pointing at floor for zone read
```

### 2.3 I2C Register Configuration
```c
// Integration time — ATIME register
// 50ms integration (0xEB) — balance of speed and noise
TCS34725_WriteReg(TCS34725_ATIME, 0xEB);   // 50ms

// Gain — CONTROL register
// 4× gain (0x01) for typical indoor arena lighting
TCS34725_WriteReg(TCS34725_CONTROL, 0x01);  // 4× gain

// Enable — ENABLE register: power on + ADC enable
TCS34725_WriteReg(TCS34725_ENABLE, 0x03);   // PON | AEN
```

### 2.4 Colour Classification Algorithm
```c
typedef enum { COLOR_UNKNOWN = 0, COLOR_RED, COLOR_GREEN, COLOR_BLUE } ColorID_t;

ColorID_t TCS34725_ClassifyColor(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
    if (c == 0) return COLOR_UNKNOWN;

    float r_n = (float)r / c;   // Normalised red ratio
    float g_n = (float)g / c;   // Normalised green ratio
    float b_n = (float)b / c;   // Normalised blue ratio

    // RED: high red ratio, dominates green and blue
    if (r_n > 0.40f && r_n > g_n * 1.4f && r_n > b_n * 1.4f)
        return COLOR_RED;

    // GREEN: high green ratio, dominates red and blue
    if (g_n > 0.35f && g_n > r_n * 1.2f && g_n > b_n * 1.2f)
        return COLOR_GREEN;

    // BLUE: high blue ratio, dominates red and green
    if (b_n > 0.30f && b_n > r_n * 1.2f && b_n > g_n * 1.2f)
        return COLOR_BLUE;

    return COLOR_UNKNOWN;  // Retry on next cycle
}
```

> [!NOTE]
> The exact ratio thresholds (0.40, 1.4× etc.) must be tuned during the 2-minute pre-competition calibration window using actual arena Red/Green/Blue paper or objects under arena lighting.

### 2.5 Non-Blocking Read Strategy
TCS34725 integration takes 50ms — longer than one 20ms control cycle. Use a polling approach:

```c
// Global state
static bool color_read_started = false;
static uint8_t color_cycle = 0;

// In 50Hz loop:
color_cycle++;
if (!color_read_started && color_cycle >= 3) {
    // Start integration (write ENABLE with AEN)
    TCS34725_StartIntegration();
    color_read_started = true;
    color_cycle = 0;
}
if (color_read_started && color_cycle >= 3) {
    // 3 cycles × 20ms = 60ms elapsed — integration complete
    TCS34725_ReadRGBC(&r, &g, &b, &c);
    last_color = TCS34725_ClassifyColor(r, g, b, c);
    color_read_started = false;
    color_cycle = 0;
}
```
This means a new colour reading is available approximately every 120ms (6 cycles) without blocking the control loop.

### 2.6 Calibration Procedure (Competition Day)
1. Set integration time to 50ms, gain to 4×.
2. Turn LED ON (PC0 HIGH).
3. Hold sensor 2cm from RED coloured paper → log r_n, g_n, b_n → verify r_n dominates.
4. Repeat for GREEN and BLUE samples.
5. Adjust threshold multipliers in firmware if needed (adjust gain to 16× if arena is dim).
6. Flash updated constants to STM32 before competition start.

---

## 3. Sensor Fusion Per Task State

| Task State | Line Array | TCS34725 | VL53L0X ToF | MPU6050 IMU |
|-----------|-----------|----------|-------------|-------------|
| **TASK1_LINE_FOLLOW** | Primary — centroid PD steering | Off | Off | Stability only |
| **TASK1_BALL_APPROACH** | Monitors all-active | Off | Front <80mm triggers | Stability |
| **TASK1_COLOR_ID** | Holds position | **MODE A — ball read** | Confirms proximity | Stability |
| **TASK1_BALL_GRAB** | Holds position | Off | Monitors range | Stability |
| **TASK2_WALL_FOLLOW** | Backup (line visible in corridor) | Off | **Primary — dual-wall PD** | Stability |
| **TASK3_OBSTACLE_DETECT** | Off | Off | **Front <150mm = obstacle** | Stability |
| **TASK3_PUSH** | Off | Off | Front >300mm = cleared | **Tilt monitor** |
| **TASK4_LINE_FOLLOW** | Primary | Off | Off | Stability |
| **TASK4_JUNCTION_DETECT** | ≥6 active triggers MODE B | **MODE B — floor read** | Off | Stability |
| **TASK4_BRANCH_FOLLOW** | Follow coloured branch line | Off | Off | Stability |

---

## 4. Failure Modes and Mitigations

| Failure | Detection | Mitigation |
|---------|-----------|-----------|
| Line array sensors dirty | Wrong centroid or all-LOW on white line | Clean with dry cloth before each run; check threshold pots |
| TCS34725 reads UNKNOWN | Ambient light wash-out or wrong range | Increase gain (16×); verify arm angle; shield sensor |
| False intersection trigger | Single high-sensor-count reading | Temporal filter (≥3 consecutive cycles) prevents this |
| Colour memory lost on restart | `stored_ball_color` = 0 after reset | Flash emulation: write to STM32 Flash page after Task 1 ID |
| I2C bus hang (TCS34725) | I2C timeout flag in HAL | STM32 re-inits I2C1 once; if persist → skip colour states |
| Line completely off surface | All 8 sensors LOW for >3s | SAFE_STOP + ±15° rotate search pattern |
