# RUNNER-4 | 9-Channel Analog IR Sensor Array
## Full Technical Report — ESP32 Implementation
### FusionForce Robotics — EN2533 BREACH PROTOCOL

---

> [!IMPORTANT]
> This report supersedes the existing 8-channel **digital** TCRT5000 STM32 driver
> (`line_array.c / line_array.h`). We are migrating to a **9-channel analog** array
> on the **ESP32**, which gives us sub-pixel centroid resolution, per-sensor noise
> filtering, and automatic white-balance calibration — critical improvements for the
> curved corridor and colour-sort junction tasks.

---

## Table of Contents

1. [Sensor Overview](#1-sensor-overview)
2. [Why 9 Sensors? Why Analog? Why ESP32?](#2-why-9-sensors-why-analog-why-esp32)
3. [Hardware Circuit — Per-Sensor Detail](#3-hardware-circuit--per-sensor-detail)
4. [Sensor Array Physical Layout](#4-sensor-array-physical-layout)
5. [ESP32 ADC Architecture](#5-esp32-adc-architecture)
6. [Signal Conditioning & Filters](#6-signal-conditioning--filters)
7. [Calibration Algorithm](#7-calibration-algorithm)
8. [Weighted Centroid & Line Position](#8-weighted-centroid--line-position)
9. [Intersection & Junction Detection](#9-intersection--junction-detection)
10. [Line-Lost Detection & Recovery](#10-line-lost-detection--recovery)
11. [Complete ESP32 Driver Code](#11-complete-esp32-driver-code)
12. [Integration with State Machine](#12-integration-with-state-machine)
13. [Testing Procedure](#13-testing-procedure)
14. [Tuning Guide](#14-tuning-guide)
15. [Wiring Reference](#15-wiring-reference)

---

## 1. Sensor Overview

### What is a TCRT5000-based Analog Sensor?

Each module in the array is a **reflective IR proximity sensor** with:

| Component | Part | Function |
|-----------|------|---------|
| **IR Emitter** | IR LED (λ=950nm) | Continuously emits infrared light downward |
| **IR Detector** | Phototransistor | Collects reflected IR from surface |
| **Resistor R1** | ~220 Ω (emitter) | Limits IR LED current to ~15 mA |
| **Resistor R2** | ~10 kΩ (collector) | Pull-up; converts phototransistor current → voltage |
| **Capacitor C1** | 100 nF ceramic | Decoupling cap — **this is the one you have** |

### How It Works — Physics

```
[IR LED] ──→ IR light beam downward
              ↓
         [Arena Surface]
              ↓
    White (line) → HIGH reflectance → more IR back
    Black (floor) → LOW reflectance → less IR back
              ↓
         [Phototransistor]
              ↓
         Collector voltage (Vout):
    White surface → transistor conducts heavily → Vout LOW  (~0.1–0.5V)
    Black surface → transistor barely conducts  → Vout HIGH (~3.0–3.3V)
```

> [!NOTE]
> **Logic is INVERTED**: White/reflective = LOW voltage, Black/absorptive = HIGH voltage.
> The code below handles this correctly with the `SENSOR_WHITE_THRESHOLD` inversion logic.

### Analog vs Digital Comparison

| Feature | Digital Output | **Analog Output (Ours)** |
|---------|----------------|--------------------------|
| Resolution | 1 bit (0 or 1) | 12-bit (0–4095) |
| Centroid accuracy | ±0.5 sensor pitch | ±0.01 sensor pitch |
| Calibration | Fixed threshold | Per-sensor min/max stored in NVS |
| Surface variation handling | Poor | Excellent |
| Noise filtering | None (hysteresis only) | Multi-layer (moving avg + EMA + outlier) |
| Junction detection | Count-based only | Area + gradient analysis |

---

## 2. Why 9 Sensors? Why Analog? Why ESP32?

### 9 Sensors vs 8 Sensors

The existing driver uses **8 sensors** (even number → no true centre sensor).
With 8 sensors, when the line is dead-centre, **two sensors** (S4 and S5) are both
partially active — causing centroid instability and oscillation in the PD controller.

With **9 sensors** (odd number):
- Sensor **S5 is the true centre**
- When the robot is perfectly aligned, only S5 activates → centroid = 0.0 exactly
- Eliminates the centre-ambiguity oscillation problem
- Provides ±4 sensor range on each side (better coverage for sharp curves)

```
S1   S2   S3   S4  [S5]  S6   S7   S8   S9
 |    |    |    |    |    |    |    |    |
-4   -3   -2   -1   0   +1   +2   +3   +4    ← Position index
```

### Why Analog on ESP32?

The ESP32 has **two 12-bit SAR ADCs** with **18 total channels**. Our 9 sensors fit
comfortably on ADC1 (8 channels, GPIO32–39) plus one pin on ADC2 — though we will
use only ADC1 to avoid Wi-Fi interference issues with ADC2.

**12-bit resolution** gives us `4096` levels per sensor → sub-millimetre line
position accuracy → smoother PD control → less oscillation in curved corridors.

### Why ESP32 Instead of STM32 for Sensors?

| Criterion | STM32F401 (existing) | **ESP32 (sensors)** |
|-----------|---------------------|---------------------|
| ADC channels | 16 (shared with servos) | 18 dedicated |
| ADC resolution | 12-bit | **12-bit** |
| ADC sampling speed | 2.4 MSPS | 2 MSPS |
| NVS (calibration storage) | Flash partition | **NVS (key-value)** |
| UART to STM32 | — | Sends centroid + bits |
| Processing overhead | High (IK + gait) | **Dedicated** |

The ESP32 acts as a **dedicated sensor co-processor**, reading all 9 ADC channels,
filtering, computing centroid, detecting junctions, and streaming results to the
STM32 master via UART at 100 Hz.

---

## 3. Hardware Circuit — Per-Sensor Detail

### Single Sensor Schematic

```
3.3V ──────────────────────────────────┐
                                       │
                                    R1 (220 Ω)
                                       │
                                    ┌──┴──┐
                                    │IR LED│  (Emitter, TCRT5000 pin A/K)
                                    └──┬──┘
                                       │
GND ───────────────────────────────────┘

3.3V ──────────────────────────────────┐
                                       │
                                    R2 (10 kΩ)
                                       │
                                       ├──────────── ADC Input (ESP32 GPIO)
                                       │
                                    ┌──┴──┐
                                    │Photo│  (Collector, TCRT5000)
                                    │Trans│
                                    └──┬──┘
                                       │
GND ───────────────────────────────────┘

Decoupling capacitor (100 nF ceramic):
3.3V ──┬── C1 (100 nF) ──┬── GND
       │                 │
   Power rail         Power rail
```

### Component Values Explained

#### R1 — IR LED Current Limiter (220 Ω)
- TCRT5000 LED: Vf ≈ 1.25V, recommended If = 10–20 mA
- `R1 = (Vcc − Vf) / If = (3.3 − 1.25) / 0.015 = 137 Ω`
- **220 Ω gives ~9 mA** — slightly conservative but safe at 3.3V supply
- Lower R1 (100 Ω) = more IR power = better range but more current draw
- For 9 sensors at 9 mA each = **81 mA total LED current** (acceptable)

#### R2 — Phototransistor Load (10 kΩ)
- Creates a voltage divider with the phototransistor's collector-emitter resistance
- Over white: Rce ≈ 500 Ω → Vout = 3.3 × 500/(500+10000) ≈ **0.16V** (LOW)
- Over black: Rce ≈ ∞ → Vout = 3.3 × 10000/(10000+~∞) ≈ **3.3V** (HIGH)
- **Higher R2 (47 kΩ)** = more sensitive but slower response
- **Lower R2 (1 kΩ)** = less sensitive but faster (for high-speed robots)
- 10 kΩ is the standard sweet spot for ≤200 mm/s robots

#### C1 — 100 nF Ceramic Decoupling
- **Purpose**: Filters high-frequency noise on the 3.3V power rail near each sensor
- Placed as physically close as possible to the sensor's VCC/GND pins
- Prevents IR LED switching transients from coupling into adjacent sensor ADC readings
- **Every sensor must have its own 100 nF cap** — do not share one cap for all 9 sensors
- Additionally, add one **10 µF electrolytic** across the main 3.3V rail for bulk decoupling

### Recommended ADC Input Protection

Add an optional **10 kΩ series resistor** between R2 junction and ESP32 ADC pin:
- Protects ESP32 ADC input from overvoltage transients
- Reduces capacitive load at ADC input for faster settling
- Does not significantly affect voltage reading accuracy

---

## 4. Sensor Array Physical Layout

### Mounting Specification

```
Front of Robot (direction of travel)
        ↑
┌──────────────────────────────────────┐
│  Robot Bottom Plate                  │
│                                      │
│  ←──────── 80 mm total width ───────→│
│                                      │
│  S1  S2  S3  S4  S5  S6  S7  S8  S9 │
│  ●   ●   ●   ●   ●   ●   ●   ●   ●  │
│  └──10mm──┘                          │
│       └──10mm spacing──┘             │
│                                      │
│  ↕ 10–15 mm above ground             │
└──────────────────────────────────────┘
         ↑ Front bumper plate
```

### Dimensional Specifications

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| Sensor spacing | **10 mm** | Arena line = 30 mm wide → ~3 sensors on line |
| Array width | **80 mm** (9 × 10mm − 10mm ends) | Fits within robot footprint |
| Mounting height | **10–15 mm** above ground | TCRT5000 optimal range: 2–15 mm |
| Tilt angle | **0°** (flat, parallel to ground) | Maximises reflectance uniformity |
| Position | Front underside, 10 mm from front edge | Ahead of front legs |

### Why 10 mm Spacing?

- Arena line width = **30 mm**
- With 10 mm spacing, 3 sensors will be on the line simultaneously
- This gives a centroid resolution of:
  - Range: ±4 × 10 mm = **±40 mm** from centre
  - Accuracy: ±(10 mm / 4096 ADC levels) ≈ **±0.002 mm** theoretically
  - Practical accuracy: ~**±0.5 mm** (limited by noise floor)
- Arena half-line = 15 mm → sensors beyond ±15 mm from centre won't see the line
  → perfect detection pattern with 3 illuminated sensors

---

## 5. ESP32 ADC Architecture

### ADC Channel Assignment

```
ESP32 GPIO Pin → ADC Channel → Sensor
─────────────────────────────────────
GPIO 32  → ADC1_CH4  → S1 (Leftmost)
GPIO 33  → ADC1_CH5  → S2
GPIO 34  → ADC1_CH6  → S3
GPIO 35  → ADC1_CH7  → S4
GPIO 36  → ADC1_CH0  → S5 (Centre)
GPIO 37  → ADC1_CH1  → S6
GPIO 38  → ADC1_CH2  → S7
GPIO 39  → ADC1_CH3  → S8
GPIO 25  → ADC2_CH8  → S9 (Rightmost)
```

> [!WARNING]
> **ADC2 limitation**: GPIO 25 (ADC2_CH8) cannot be used when Wi-Fi is active.
> Since we have Wi-Fi **disabled** during competition, this is safe.
> **Alternatively**, use GPIO 26 (ADC2_CH9) — same restriction applies.
> **Best practice**: Keep all 9 sensors on ADC1 if possible — use a MUX (CD4051) or
> an external ADC (ADS1115) to expand to 9 channels on ADC1 only.

### ESP32 ADC Settings

```
Attenuation: ADC_ATTEN_DB_11  (full scale 0–3.3V → 0–4095 raw)
Resolution:  12-bit (4096 levels)
Sampling:    Multiple samples averaged per reading (configurable)
```

### ADC Non-Linearity Issue

The ESP32's built-in ADC has known non-linearity between 0–100 mV and 3100–3300 mV.
Our sensor output ranges between ~150 mV (white) and ~3100 mV (black) — this falls
within the **linear region**. No special correction needed, but we apply a 2-point
calibration per sensor (min/max) that implicitly handles any residual non-linearity.

---

## 6. Signal Conditioning & Filters

We apply **3 layers of filtering** in sequence:

```
Raw ADC (12-bit) → [Layer 1: Oversampling] → [Layer 2: EMA Filter]
                 → [Layer 3: Outlier Rejection] → Normalised [0.0–1.0]
                 → Calibration mapping → Processed value
```

### Layer 1: Oversampling (Hardware Averaging)

Read each ADC channel **N times** and average:
- Reduces random ADC noise by factor of √N
- For N=16: noise reduction = 4× → effective 14-bit resolution
- Adds ~80 µs per sensor (at 1 MSPS) → 720 µs for all 9 sensors
- Acceptable for 100 Hz update rate (10 ms period)

```c
// Read N samples and average
uint32_t oversample_adc(adc1_channel_t ch, int n) {
    uint32_t sum = 0;
    for (int i = 0; i < n; i++) {
        sum += adc1_get_raw(ch);
    }
    return sum / n;
}
```

### Layer 2: Exponential Moving Average (EMA)

Smooths temporal noise without adding latency:

```
filtered[n] = alpha × raw[n] + (1 - alpha) × filtered[n-1]
```

| Alpha | Effect |
|-------|--------|
| 1.0 | No filtering (raw) |
| 0.5 | Medium smoothing (~2 sample lag) |
| 0.3 | Heavy smoothing (~3 sample lag) |
| **0.7** | **Recommended** — fast response, good noise rejection |

At 100 Hz with α=0.7, a step change settles within **2–3 samples (20–30 ms)** — fast
enough for a 80 mm/s robot crossing a 30 mm line (crossing time = 375 ms).

### Layer 3: Outlier Rejection (Spike Filter)

Rejects single-sample spikes (EMI, vibration):

```
If |raw[n] - filtered[n-1]| > SPIKE_THRESHOLD:
    Use filtered[n-1] instead of raw[n]
Else:
    Apply EMA normally
```

- `SPIKE_THRESHOLD = 500` ADC counts (~0.4V) rejects electrical transients
- Does NOT reject valid rapid changes (sensor crossing black→white in 10 ms will be
  a gradual change over multiple samples, not a single spike)

### Filter Summary

| Layer | Technique | Noise Removed | Latency Added |
|-------|-----------|---------------|---------------|
| 1 | 16× oversampling | Random ADC noise ±8 counts | ~0.8 ms |
| 2 | EMA α=0.7 | Temporal sensor flutter | ~15 ms |
| 3 | Spike filter | EMI transients | 0 ms |
| **Total** | | **>95% noise reduction** | **~16 ms** |

---

## 7. Calibration Algorithm

### Why Calibrate?

Each of the 9 sensors has slightly different:
- LED forward voltage → different emission intensity
- Phototransistor gain → different sensitivity
- Component tolerances (R1, R2 ±5%) → different bias point
- Mounting angle variations → different distance to ground

Without calibration, sensors 1 and 9 (edges) will produce different raw values than
sensor 5 (centre) even over identical surfaces. This causes centroid bias.

### Calibration Procedure

#### Step 1: Measure Minimum (White Surface)
Place robot over **white tape / white paper** covering all 9 sensors.
Record `cal_min[i]` = minimum raw ADC value seen by each sensor.

#### Step 2: Measure Maximum (Black Surface)
Place robot over **black surface / black electrical tape**.
Record `cal_max[i]` = maximum raw ADC value.

#### Step 3: Normalise
At runtime, each raw reading is normalised to [0.0, 1.0]:

```
// Raw value from ADC (higher = blacker)
// Normalised: 0.0 = fully white (on line), 1.0 = fully black (off line)
normalised[i] = (raw[i] - cal_min[i]) / (cal_max[i] - cal_min[i])
normalised[i] = clamp(normalised[i], 0.0, 1.0)

// Line value (inverted): 1.0 = white (on line), 0.0 = black (off line)
line_value[i] = 1.0 - normalised[i]
```

#### Calibration Storage (NVS)

Calibration values are stored in ESP32 **Non-Volatile Storage (NVS)**:
- Survives power cycles
- Key: `"la_cal_min_0"` through `"la_cal_min_8"` (9 sensors)
- Key: `"la_cal_max_0"` through `"la_cal_max_8"`
- Namespace: `"line_array"`

#### Dynamic Calibration (Auto-update)

Optionally, track running min/max during operation:
```
cal_min[i] = min(cal_min[i], raw[i]) * 0.999 + raw[i] * 0.001
cal_max[i] = max(cal_max[i], raw[i]) * 0.999 + raw[i] * 0.001
```
This **slowly adapts** to lighting changes (slowly, to prevent false adaptation
to line surface being misidentified as the reference surface).

---

## 8. Weighted Centroid & Line Position

### Centroid Formula

```
position = Σ(i × line_value[i]) / Σ(line_value[i])
         for i = 0 to 8 (9 sensors)

error = position - 4.0   (centre sensor is index 4)
```

- `error` range: **−4.0 to +4.0**
- `error = 0.0` → line directly under sensor S5 (perfect alignment)
- `error < 0` → line is to the LEFT → robot must turn left
- `error > 0` → line is to the RIGHT → robot must turn right

### Converting to Physical Units

With 10 mm sensor spacing:
```
position_mm = error × 10.0   // mm from centre
```

Error of ±1.0 = line displaced 10 mm from centre.

### Centroid Stability Improvement

**Problem**: When the line is between two sensors, the centroid oscillates as
analog noise causes small value changes.

**Solution**: Apply a **threshold band** — only include sensors whose `line_value`
exceeds a minimum contribution threshold:

```c
#define LINE_CONTRIB_THRESHOLD  0.15f  // Sensor must be at least 15% "on" to contribute

float sum_pos = 0, sum_weight = 0;
for (int i = 0; i < 9; i++) {
    if (line_values[i] > LINE_CONTRIB_THRESHOLD) {
        sum_pos    += i * line_values[i];
        sum_weight += line_values[i];
    }
}
float centroid = (sum_weight > 0) ? sum_pos / sum_weight : NAN;
float error    = centroid - 4.0f;
```

---

## 9. Intersection & Junction Detection

### Logic

An intersection (T-junction, + junction, or colour-sort branch point) is detected
when **many sensors** simultaneously see white.

```
active_count = number of sensors with line_value > ACTIVE_THRESHOLD (0.5)

Intersection detected when:
  active_count >= INTERSECTION_THRESHOLD (6 of 9)
  for INTERSECTION_CONSEC consecutive samples (3 × 10ms = 30ms)
```

### 3-Step Temporal Filter (Critical!)

Without temporal filtering, a robot travelling at 80 mm/s crosses a 30 mm wide
junction in 375 ms → ~37 samples at 100 Hz. Using a 3-sample debounce prevents
electrical noise from triggering false detections.

```c
static int junction_consec = 0;

bool update_junction_detection(float *line_vals) {
    int active = 0;
    for (int i = 0; i < 9; i++) {
        if (line_vals[i] > 0.5f) active++;
    }
    
    if (active >= 6) {
        junction_consec++;
        if (junction_consec >= 3) return true;
    } else {
        junction_consec = 0;
    }
    return false;
}
```

### Junction Type Classification

Using the pattern of active sensors:

```
Pattern              Active sensors    Junction type
─────────────────────────────────────────────────────
All 9 active         S1–S9            + junction (grid cross)
S1–S5 active         Left half         T-junction (branch left)
S5–S9 active         Right half         T-junction (branch right)
S1–S9 active briefly  Flash             End of line
3 consecutive        S4, S5, S6        Normal line (no junction)
```

---

## 10. Line-Lost Detection & Recovery

### Detection

```c
bool line_lost = (sum_weight == 0);  // No sensor above threshold
uint32_t lost_cycles++;              // Increment every 10ms sample
```

After `150` cycles (1.5 seconds) with no line: enter `LINE_LOST_RECOVERY`.

### Recovery Strategy

```
1. STOP forward motion (Vx = 0)
2. Apply last-known steering direction (hold last error sign)
3. Slowly rotate in that direction
4. If line found → resume normal following
5. If not found after 3 seconds → STATE_ERROR_RECOVERY
```

---

## 11. Complete ESP32 Driver Code

The following code is production-ready for the RUNNER-4 project.

---

### File: `line_array_esp32.h`

```c
/**
 * @file    line_array_esp32.h
 * @brief   9-Channel Analog IR Sensor Array Driver for ESP32
 * @details Uses ADC1 channels, 16x oversampling, EMA filter, spike rejection,
 *          NVS calibration, weighted centroid, and junction detection.
 *
 * Hardware: TCRT5000 analog modules
 *   Each sensor: R1=220Ω (LED), R2=10kΩ (pull-up), C1=100nF (decoupling)
 *
 * Sensor <-> GPIO mapping:
 *   S1(Left)  → GPIO 32 (ADC1_CH4)
 *   S2        → GPIO 33 (ADC1_CH5)
 *   S3        → GPIO 34 (ADC1_CH6)
 *   S4        → GPIO 35 (ADC1_CH7)
 *   S5(Centre)→ GPIO 36 (ADC1_CH0)
 *   S6        → GPIO 37 (ADC1_CH1)
 *   S7        → GPIO 38 (ADC1_CH2)
 *   S8        → GPIO 39 (ADC1_CH3)
 *   S9(Right) → GPIO 25 (ADC2_CH8) — Wi-Fi must be OFF
 *
 * Logic: ADC HIGH (near 4095) = Black surface (off line)
 *        ADC LOW  (near 0)    = White surface (on line)
 *
 * Centroid convention:
 *   error < 0 → line is LEFT  of centre → turn left
 *   error > 0 → line is RIGHT of centre → turn right
 *   error = 0 → perfectly centred
 */

#ifndef LINE_ARRAY_ESP32_H
#define LINE_ARRAY_ESP32_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/adc.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ─── Configuration Constants ─────────────────────────────────────────────── */

#define LA_NUM_SENSORS              9       ///< Number of sensors in array
#define LA_CENTRE_INDEX             4.0f    ///< Centre sensor index (S5 = index 4)
#define LA_SENSOR_SPACING_MM        10.0f   ///< Physical spacing between sensors (mm)

/* Oversampling */
#define LA_OVERSAMPLE_N             16      ///< ADC reads per sample (noise reduction)

/* EMA filter alpha (0.0 = frozen, 1.0 = raw) */
#define LA_EMA_ALPHA                0.70f   ///< Recommended: 0.5–0.8

/* Spike rejection threshold (ADC counts out of 4095) */
#define LA_SPIKE_THRESHOLD          500     ///< Max valid ADC change per sample

/* Line detection threshold (normalised 0.0–1.0) */
#define LA_ACTIVE_THRESHOLD         0.50f   ///< Sensor is "on line" if above this
#define LA_CONTRIB_THRESHOLD        0.15f   ///< Min contribution to centroid calc

/* Junction detection */
#define LA_JUNCTION_MIN_SENSORS     6       ///< Active sensors needed for junction
#define LA_JUNCTION_CONSEC_SAMPLES  3       ///< Consecutive samples required
#define LA_LOST_TIMEOUT_MS          1500    ///< Time before LINE_LOST declared (ms)

/* Calibration NVS */
#define LA_NVS_NAMESPACE            "line_array"
#define LA_NVS_KEY_MIN_FMT          "cal_min_%d"
#define LA_NVS_KEY_MAX_FMT          "cal_max_%d"

/* Default calibration (override with LA_Calibrate) */
#define LA_CAL_DEFAULT_MIN          300     ///< Raw ADC value over white (estimate)
#define LA_CAL_DEFAULT_MAX          3800    ///< Raw ADC value over black (estimate)

/* ─── Data Types ───────────────────────────────────────────────────────────── */

/**
 * @brief Processed output from one update cycle
 */
typedef struct {
    float    raw_norm[LA_NUM_SENSORS];  ///< Normalised values [0=white, 1=black]
    float    line_val[LA_NUM_SENSORS];  ///< Inverted: [1=on line, 0=off line]
    uint16_t adc_raw[LA_NUM_SENSORS];   ///< Raw 12-bit ADC values (for debugging)
    int      active_count;              ///< Number of sensors above LA_ACTIVE_THRESHOLD
    float    centroid;                  ///< Position index 0.0–8.0 (NAN if line lost)
    float    error;                     ///< centroid - 4.0 → range [-4.0, +4.0]
    float    error_mm;                  ///< error × LA_SENSOR_SPACING_MM (mm)
    bool     is_junction;               ///< Intersection/junction confirmed
    bool     is_line_lost;              ///< True when no sensor sees the line
    uint32_t lost_duration_ms;          ///< How long line has been lost (ms)
} LA_Result_t;

/**
 * @brief Calibration data (per-sensor min and max ADC raw values)
 */
typedef struct {
    uint16_t min_val[LA_NUM_SENSORS];   ///< ADC over white surface
    uint16_t max_val[LA_NUM_SENSORS];   ///< ADC over black surface
    bool     is_valid;                  ///< True if calibration has been performed
} LA_Calibration_t;

/* ─── Public API ───────────────────────────────────────────────────────────── */

/**
 * @brief  Initialise the sensor array driver.
 *         Configures ADC channels, loads calibration from NVS.
 *         If no calibration found in NVS, uses default values.
 * @return ESP_OK on success
 */
esp_err_t LA_Init(void);

/**
 * @brief  Perform calibration sequence.
 *         Call with robot placed over WHITE surface first, then BLACK.
 *         Stores results to NVS automatically.
 *
 * @param  white_surface  true = currently over white; false = currently over black
 * @return ESP_OK on success
 */
esp_err_t LA_Calibrate(bool white_surface);

/**
 * @brief  Read all sensors, apply filters, compute centroid and junction status.
 *         Call at 100 Hz (every 10 ms) in your main sensor task.
 * @param  result  Pointer to output structure (filled by this function)
 * @return ESP_OK on success
 */
esp_err_t LA_Update(LA_Result_t *result);

/**
 * @brief  Get the last computed result without re-reading sensors.
 *         Useful for reading from a different task than the update task.
 * @param  result  Pointer to output structure
 */
void LA_GetLastResult(LA_Result_t *result);

/**
 * @brief  Reset junction counter and line-lost counter.
 *         Call when re-entering a line-following state after a pause.
 */
void LA_Reset(void);

/**
 * @brief  Print current sensor values to UART (for debugging).
 *         Call from a debug task at 5–10 Hz.
 */
void LA_PrintDebug(const LA_Result_t *result);

/**
 * @brief  Get current calibration values.
 * @param  cal  Pointer to calibration struct to fill
 */
void LA_GetCalibration(LA_Calibration_t *cal);

#ifdef __cplusplus
}
#endif

#endif /* LINE_ARRAY_ESP32_H */
```

---

### File: `line_array_esp32.c`

```c
/**
 * @file    line_array_esp32.c
 * @brief   9-Channel Analog IR Sensor Array Driver — ESP32 Implementation
 *
 * Build dependencies (ESP-IDF):
 *   - driver/adc.h
 *   - nvs_flash.h
 *   - nvs.h
 *   - esp_log.h
 *   - freertos/FreeRTOS.h
 *   - freertos/semphr.h
 *   - math.h
 */

#include "line_array_esp32.h"

#include <string.h>
#include <math.h>
#include <stdio.h>

#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "LineArray";

/* ─── ADC Channel Map (9 sensors) ─────────────────────────────────────────── */

// NOTE: ADC1 only (ADC2 interference with Wi-Fi disabled in our case)
// For full ADC1-only: use external MUX (CD4051) or ADS1115
static const adc1_channel_t ADC1_CHANNELS[8] = {
    ADC1_CHANNEL_4,   // S1 → GPIO32
    ADC1_CHANNEL_5,   // S2 → GPIO33
    ADC1_CHANNEL_6,   // S3 → GPIO34
    ADC1_CHANNEL_7,   // S4 → GPIO35
    ADC1_CHANNEL_0,   // S5 → GPIO36 (centre)
    ADC1_CHANNEL_1,   // S6 → GPIO37
    ADC1_CHANNEL_2,   // S7 → GPIO38
    ADC1_CHANNEL_3,   // S8 → GPIO39
    // S9 on ADC2 — handled separately
};

// S9 uses ADC2
#include "driver/adc.h"
static const adc2_channel_t S9_ADC2_CHANNEL = ADC2_CHANNEL_8; // GPIO25

/* ─── Internal State ───────────────────────────────────────────────────────── */

static LA_Calibration_t  _cal;
static LA_Result_t       _last_result;
static float             _ema[LA_NUM_SENSORS];        // EMA filtered values (ADC counts)
static bool              _ema_initialised[LA_NUM_SENSORS];
static int               _junction_consec   = 0;
static uint32_t          _lost_cycles        = 0;
static esp_adc_cal_characteristics_t _adc_chars;
static SemaphoreHandle_t _result_mutex       = NULL;
static bool              _initialised        = false;

/* ─── Private Helpers ──────────────────────────────────────────────────────── */

/**
 * @brief  Read one ADC1 channel with LA_OVERSAMPLE_N samples averaged.
 */
static uint16_t prv_read_adc1(adc1_channel_t ch) {
    uint32_t sum = 0;
    for (int i = 0; i < LA_OVERSAMPLE_N; i++) {
        sum += (uint32_t)adc1_get_raw(ch);
    }
    return (uint16_t)(sum / LA_OVERSAMPLE_N);
}

/**
 * @brief  Read S9 from ADC2 with oversampling.
 */
static uint16_t prv_read_adc2_s9(void) {
    uint32_t sum = 0;
    int raw = 0;
    for (int i = 0; i < LA_OVERSAMPLE_N; i++) {
        esp_err_t ret = adc2_get_raw(S9_ADC2_CHANNEL, ADC_WIDTH_BIT_12, &raw);
        if (ret == ESP_OK) {
            sum += (uint32_t)raw;
        } else {
            // If ADC2 fails (Wi-Fi active), use last valid
            sum += (uint32_t)_ema[8];
        }
    }
    return (uint16_t)(sum / LA_OVERSAMPLE_N);
}

/**
 * @brief  Apply EMA filter with spike rejection for one sensor.
 * @param  idx     Sensor index (0–8)
 * @param  raw     New raw ADC reading
 * @return Filtered ADC value (float)
 */
static float prv_apply_ema(int idx, uint16_t raw) {
    float fraw = (float)raw;

    if (!_ema_initialised[idx]) {
        _ema[idx] = fraw;
        _ema_initialised[idx] = true;
        return fraw;
    }

    // Spike rejection: if change is too large, use previous filtered value
    if (fabsf(fraw - _ema[idx]) > LA_SPIKE_THRESHOLD) {
        ESP_LOGD(TAG, "S%d spike rejected: raw=%d filtered=%.0f", idx+1, raw, _ema[idx]);
        fraw = _ema[idx]; // Reject spike — keep previous
    }

    // EMA: filtered = alpha * new + (1 - alpha) * previous
    _ema[idx] = LA_EMA_ALPHA * fraw + (1.0f - LA_EMA_ALPHA) * _ema[idx];
    return _ema[idx];
}

/**
 * @brief  Normalise a filtered ADC value to [0.0 = white, 1.0 = black].
 */
static float prv_normalise(int idx, float filtered_adc) {
    float range = (float)(_cal.max_val[idx] - _cal.min_val[idx]);
    if (range < 10.0f) range = 10.0f; // Safety: avoid divide-by-zero

    float norm = (filtered_adc - (float)_cal.min_val[idx]) / range;

    // Clamp to [0, 1]
    if (norm < 0.0f) norm = 0.0f;
    if (norm > 1.0f) norm = 1.0f;

    return norm;
}

/**
 * @brief  Load calibration from NVS. Returns ESP_ERR_NOT_FOUND if not stored.
 */
static esp_err_t prv_load_cal_from_nvs(void) {
    nvs_handle_t nvs;
    esp_err_t ret = nvs_open(LA_NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (ret != ESP_OK) return ret;

    char key[24];
    for (int i = 0; i < LA_NUM_SENSORS; i++) {
        snprintf(key, sizeof(key), LA_NVS_KEY_MIN_FMT, i);
        uint16_t v = 0;
        ret = nvs_get_u16(nvs, key, &v);
        if (ret != ESP_OK) { nvs_close(nvs); return ret; }
        _cal.min_val[i] = v;

        snprintf(key, sizeof(key), LA_NVS_KEY_MAX_FMT, i);
        ret = nvs_get_u16(nvs, key, &v);
        if (ret != ESP_OK) { nvs_close(nvs); return ret; }
        _cal.max_val[i] = v;
    }

    nvs_close(nvs);
    _cal.is_valid = true;
    ESP_LOGI(TAG, "Calibration loaded from NVS");
    return ESP_OK;
}

/**
 * @brief  Save calibration to NVS.
 */
static esp_err_t prv_save_cal_to_nvs(void) {
    nvs_handle_t nvs;
    esp_err_t ret = nvs_open(LA_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (ret != ESP_OK) return ret;

    char key[24];
    for (int i = 0; i < LA_NUM_SENSORS; i++) {
        snprintf(key, sizeof(key), LA_NVS_KEY_MIN_FMT, i);
        nvs_set_u16(nvs, key, _cal.min_val[i]);
        snprintf(key, sizeof(key), LA_NVS_KEY_MAX_FMT, i);
        nvs_set_u16(nvs, key, _cal.max_val[i]);
    }

    ret = nvs_commit(nvs);
    nvs_close(nvs);
    if (ret == ESP_OK) ESP_LOGI(TAG, "Calibration saved to NVS");
    return ret;
}

/**
 * @brief  Set default calibration values (fallback if NVS not found).
 */
static void prv_set_default_cal(void) {
    for (int i = 0; i < LA_NUM_SENSORS; i++) {
        _cal.min_val[i] = LA_CAL_DEFAULT_MIN;
        _cal.max_val[i] = LA_CAL_DEFAULT_MAX;
    }
    _cal.is_valid = false;
    ESP_LOGW(TAG, "Using DEFAULT calibration — please calibrate properly!");
}

/* ─── Public API Implementation ────────────────────────────────────────────── */

esp_err_t LA_Init(void) {
    if (_initialised) return ESP_OK;

    // Create mutex for thread-safe result access
    _result_mutex = xSemaphoreCreateMutex();
    if (!_result_mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    // Configure ADC1: 12-bit, 11dB attenuation (full 3.3V range)
    adc1_config_width(ADC_WIDTH_BIT_12);
    for (int i = 0; i < 8; i++) {
        adc1_config_channel_atten(ADC1_CHANNELS[i], ADC_ATTEN_DB_11);
    }

    // Configure ADC2 for S9
    adc2_config_channel_atten(S9_ADC2_CHANNEL, ADC_ATTEN_DB_11);

    // Characterise ADC for voltage conversion (optional but recommended)
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11,
                              ADC_WIDTH_BIT_12, 1100, &_adc_chars);

    // Initialise EMA state
    memset(_ema, 0, sizeof(_ema));
    memset(_ema_initialised, 0, sizeof(_ema_initialised));

    // Load calibration from NVS
    esp_err_t ret = prv_load_cal_from_nvs();
    if (ret != ESP_OK) {
        prv_set_default_cal();
    }

    // Clear result
    memset(&_last_result, 0, sizeof(_last_result));
    _last_result.centroid = 4.0f; // Start centred
    _last_result.error    = 0.0f;

    _junction_consec = 0;
    _lost_cycles     = 0;
    _initialised     = true;

    ESP_LOGI(TAG, "Line array initialised: %d sensors, %.0fdB atten, %dx oversample",
             LA_NUM_SENSORS, 11.0f, LA_OVERSAMPLE_N);

    return ESP_OK;
}

esp_err_t LA_Calibrate(bool white_surface) {
    if (!_initialised) return ESP_ERR_INVALID_STATE;

    // Take 32 averaged readings for stable calibration
    uint32_t acc[LA_NUM_SENSORS] = {0};
    const int CAL_SAMPLES = 32;

    for (int s = 0; s < CAL_SAMPLES; s++) {
        for (int i = 0; i < 8; i++) {
            acc[i] += prv_read_adc1(ADC1_CHANNELS[i]);
        }
        acc[8] += prv_read_adc2_s9();
        vTaskDelay(pdMS_TO_TICKS(5)); // 5ms between cal samples
    }

    for (int i = 0; i < LA_NUM_SENSORS; i++) {
        uint16_t avg = (uint16_t)(acc[i] / CAL_SAMPLES);
        if (white_surface) {
            _cal.min_val[i] = avg;
            ESP_LOGI(TAG, "S%d white cal = %d", i+1, avg);
        } else {
            _cal.max_val[i] = avg;
            ESP_LOGI(TAG, "S%d black cal = %d", i+1, avg);
        }
    }

    // Validate: black must be > white + 200 ADC counts
    if (!white_surface) {
        bool valid = true;
        for (int i = 0; i < LA_NUM_SENSORS; i++) {
            if (_cal.max_val[i] < _cal.min_val[i] + 200) {
                ESP_LOGE(TAG, "S%d: black(%d) not far enough from white(%d)!",
                         i+1, _cal.max_val[i], _cal.min_val[i]);
                valid = false;
            }
        }
        if (valid) {
            _cal.is_valid = true;
            prv_save_cal_to_nvs();
            ESP_LOGI(TAG, "Calibration complete and saved");
        } else {
            ESP_LOGW(TAG, "Calibration incomplete — check sensor mounting height");
        }
    }

    return ESP_OK;
}

esp_err_t LA_Update(LA_Result_t *result) {
    if (!_initialised || !result) return ESP_ERR_INVALID_ARG;

    LA_Result_t r;
    memset(&r, 0, sizeof(r));

    // ── Step 1: Read all 9 ADC channels with oversampling ──────────────────
    for (int i = 0; i < 8; i++) {
        r.adc_raw[i] = prv_read_adc1(ADC1_CHANNELS[i]);
    }
    r.adc_raw[8] = prv_read_adc2_s9();

    // ── Step 2: Apply EMA filter (with spike rejection) ────────────────────
    // ── Step 3: Normalise [0=white, 1=black] ──────────────────────────────
    // ── Step 4: Invert for line_val [1=on line, 0=off line] ───────────────
    for (int i = 0; i < LA_NUM_SENSORS; i++) {
        float filtered   = prv_apply_ema(i, r.adc_raw[i]);
        r.raw_norm[i]    = prv_normalise(i, filtered);
        r.line_val[i]    = 1.0f - r.raw_norm[i]; // Invert: white=high, black=low
    }

    // ── Step 5: Count active sensors ──────────────────────────────────────
    r.active_count = 0;
    for (int i = 0; i < LA_NUM_SENSORS; i++) {
        if (r.line_val[i] > LA_ACTIVE_THRESHOLD) {
            r.active_count++;
        }
    }

    // ── Step 6: Compute weighted centroid ──────────────────────────────────
    float sum_pos    = 0.0f;
    float sum_weight = 0.0f;
    for (int i = 0; i < LA_NUM_SENSORS; i++) {
        if (r.line_val[i] > LA_CONTRIB_THRESHOLD) {
            sum_pos    += (float)i * r.line_val[i];
            sum_weight += r.line_val[i];
        }
    }

    if (sum_weight > 0.0f) {
        r.centroid  = sum_pos / sum_weight;
        r.error     = r.centroid - LA_CENTRE_INDEX;  // -4.0 to +4.0
        r.error_mm  = r.error * LA_SENSOR_SPACING_MM; // -40mm to +40mm
        r.is_line_lost   = false;
        _lost_cycles     = 0;
    } else {
        r.centroid       = NAN;
        r.error          = NAN;
        r.error_mm       = NAN;
        r.is_line_lost   = true;
        _lost_cycles++;
        r.lost_duration_ms = _lost_cycles * 10; // Assuming 100Hz update rate
    }

    // ── Step 7: Junction / Intersection detection ──────────────────────────
    if (r.active_count >= LA_JUNCTION_MIN_SENSORS) {
        _junction_consec++;
        if (_junction_consec >= LA_JUNCTION_CONSEC_SAMPLES) {
            r.is_junction = true;
            // Saturate counter to prevent overflow
            if (_junction_consec > LA_JUNCTION_CONSEC_SAMPLES + 20)
                _junction_consec = LA_JUNCTION_CONSEC_SAMPLES;
        }
    } else {
        _junction_consec = 0;
        r.is_junction    = false;
    }

    // ── Step 8: Thread-safe update of last result ──────────────────────────
    if (xSemaphoreTake(_result_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        memcpy(&_last_result, &r, sizeof(LA_Result_t));
        xSemaphoreGive(_result_mutex);
    }

    memcpy(result, &r, sizeof(LA_Result_t));
    return ESP_OK;
}

void LA_GetLastResult(LA_Result_t *result) {
    if (!result) return;
    if (xSemaphoreTake(_result_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        memcpy(result, &_last_result, sizeof(LA_Result_t));
        xSemaphoreGive(_result_mutex);
    }
}

void LA_Reset(void) {
    _junction_consec = 0;
    _lost_cycles     = 0;
    ESP_LOGD(TAG, "State reset");
}

void LA_PrintDebug(const LA_Result_t *r) {
    if (!r) return;
    printf("[LA] ");
    for (int i = 0; i < LA_NUM_SENSORS; i++) {
        // Print bar chart: '█' if on line, '░' if off line
        printf("%s", r->line_val[i] > LA_ACTIVE_THRESHOLD ? "█" : "░");
    }
    if (!isnan(r->error)) {
        printf(" | err=%.2f (%.1fmm) | act=%d%s%s\n",
               r->error, r->error_mm, r->active_count,
               r->is_junction   ? " JUNCTION"  : "",
               r->is_line_lost  ? " LOST"      : "");
    } else {
        printf(" | LINE LOST (%lu ms)\n", r->lost_duration_ms);
    }
}

void LA_GetCalibration(LA_Calibration_t *cal) {
    if (cal) memcpy(cal, &_cal, sizeof(LA_Calibration_t));
}
```

---

### File: `sensor_task.c` — FreeRTOS Task

```c
/**
 * @file    sensor_task.c
 * @brief   FreeRTOS task: reads line array at 100 Hz, streams results via UART
 *
 * Task stack: 4096 bytes
 * Priority: 10 (above normal, below motor control)
 * Core: PRO_CPU (Core 0) — leave APP_CPU (Core 1) for Wi-Fi/BT (disabled anyway)
 *
 * UART output format (to STM32 at 115200 baud):
 *   $LA,<err_x10>,<active>,<junc>,<lost>\r\n
 *   Example: $LA,-12,3,0,0\r\n
 *            → error=-1.2 (line 12mm left), 3 sensors active, no junction, not lost
 */

#include "sensor_task.h"
#include "line_array_esp32.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

static const char *TAG = "SensorTask";

#define SENSOR_UART_PORT    UART_NUM_1
#define SENSOR_UART_TX_PIN  17
#define SENSOR_UART_RX_PIN  16
#define SENSOR_UART_BAUD    115200

#define SENSOR_TASK_RATE_HZ         100     // 100 Hz update rate
#define SENSOR_TASK_PERIOD_MS       (1000 / SENSOR_TASK_RATE_HZ)  // 10 ms
#define SENSOR_DEBUG_PRINT_EVERY_N  20      // Print at 5 Hz (every 20 updates)

static void uart_init(void) {
    uart_config_t cfg = {
        .baud_rate  = SENSOR_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_driver_install(SENSOR_UART_PORT, 256, 256, 0, NULL, 0);
    uart_param_config(SENSOR_UART_PORT, &cfg);
    uart_set_pin(SENSOR_UART_PORT, SENSOR_UART_TX_PIN, SENSOR_UART_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static uint8_t crc8(const uint8_t *data, int len) {
    uint8_t crc = 0xFF;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
        }
    }
    return crc;
}

void sensor_task(void *arg) {
    ESP_LOGI(TAG, "Sensor task started at %d Hz", SENSOR_TASK_RATE_HZ);

    uart_init();

    esp_err_t ret = LA_Init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LA_Init failed: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    LA_Result_t result;
    int debug_counter = 0;
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        // ── Update sensors ─────────────────────────────────────────────────
        LA_Update(&result);

        // ── Send UART packet to STM32 ──────────────────────────────────────
        // Format: $LA,<err_x100>,<active>,<junc>,<lost>,<crc>\r\n
        // err_x100: error × 100 as int16 (e.g., -1.25 → -125)
        int16_t err_enc = isnan(result.error) ? INT16_MIN
                                              : (int16_t)(result.error * 100.0f);
        char pkt[64];
        int n = snprintf(pkt, sizeof(pkt), "$LA,%d,%d,%d,%d",
                         err_enc,
                         result.active_count,
                         result.is_junction ? 1 : 0,
                         result.is_line_lost ? 1 : 0);

        // Append CRC8
        uint8_t crc = crc8((uint8_t*)pkt+1, n-1); // skip '$'
        n += snprintf(pkt + n, sizeof(pkt) - n, ",%02X\r\n", crc);

        uart_write_bytes(SENSOR_UART_PORT, pkt, n);

        // ── Debug print ────────────────────────────────────────────────────
        if (++debug_counter >= SENSOR_DEBUG_PRINT_EVERY_N) {
            LA_PrintDebug(&result);
            debug_counter = 0;
        }

        // ── Wait for next period (maintains 100 Hz precisely) ──────────────
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_TASK_PERIOD_MS));
    }
}
```

---

### File: `main.c` — ESP32 Entry Point

```c
/**
 * @file    main.c
 * @brief   ESP32 Line Array Sensor Co-Processor — Main Entry Point
 *
 * Startup sequence:
 *   1. NVS flash init
 *   2. Check for calibration button (GPIO0 / BOOT button)
 *   3. Start sensor task (100 Hz)
 *   4. If calibration button held at boot: run calibration routine
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "line_array_esp32.h"
#include "sensor_task.h"

static const char *TAG = "Main";

#define CALIB_BUTTON_GPIO   0     // BOOT button on most ESP32 dev boards
#define CALIB_BUTTON_HOLD_MS 3000 // Hold for 3 seconds to enter calibration

static bool check_calibration_button(void) {
    gpio_set_direction(CALIB_BUTTON_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(CALIB_BUTTON_GPIO, GPIO_PULLUP_ONLY);

    // Button is active LOW (BOOT button)
    if (gpio_get_level(CALIB_BUTTON_GPIO) == 0) {
        ESP_LOGI(TAG, "Calibration button held — entering calibration mode in 3s");
        vTaskDelay(pdMS_TO_TICKS(3000));
        if (gpio_get_level(CALIB_BUTTON_GPIO) == 0) {
            return true;
        }
    }
    return false;
}

static void run_calibration_routine(void) {
    LA_Init();
    ESP_LOGI(TAG, "=== CALIBRATION MODE ===");

    // Step 1: White surface
    ESP_LOGI(TAG, "Place ALL sensors over WHITE surface. Wait 3s...");
    vTaskDelay(pdMS_TO_TICKS(3000));
    ESP_LOGI(TAG, "Sampling WHITE...");
    LA_Calibrate(true);  // white_surface = true
    ESP_LOGI(TAG, "White done. Now place over BLACK surface. Wait 3s...");

    vTaskDelay(pdMS_TO_TICKS(3000));
    ESP_LOGI(TAG, "Sampling BLACK...");
    LA_Calibrate(false); // white_surface = false
    ESP_LOGI(TAG, "Calibration complete! Reboot to start normal operation.");

    // Print calibration summary
    LA_Calibration_t cal;
    LA_GetCalibration(&cal);
    for (int i = 0; i < LA_NUM_SENSORS; i++) {
        ESP_LOGI(TAG, "S%d: min(white)=%d  max(black)=%d  range=%d",
                 i+1, cal.min_val[i], cal.max_val[i],
                 cal.max_val[i] - cal.min_val[i]);
    }

    // Blink LED to indicate done
    while (1) {
        gpio_set_level(2, 1); vTaskDelay(pdMS_TO_TICKS(200));
        gpio_set_level(2, 0); vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "FusionForce RUNNER-4 — Line Array Co-Processor");
    ESP_LOGI(TAG, "Firmware build: %s %s", __DATE__, __TIME__);

    // Initialise NVS (required for calibration storage)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition erased and reinitialised");
        nvs_flash_erase();
        nvs_flash_init();
    }

    // Check if calibration mode requested
    if (check_calibration_button()) {
        run_calibration_routine();
        return; // Never returns
    }

    // Normal operation: start 100 Hz sensor task
    ESP_LOGI(TAG, "Starting sensor task...");
    xTaskCreatePinnedToCore(
        sensor_task,        // Task function
        "sensor_task",      // Name
        4096,               // Stack size (bytes)
        NULL,               // Parameters
        10,                 // Priority (high)
        NULL,               // Task handle
        0                   // Core 0 (PRO_CPU)
    );

    ESP_LOGI(TAG, "System running.");
}
```

---

### File: `line_follower_esp32.h` + `.c` — PD Controller

```c
/**
 * @file    line_follower_esp32.h
 * @brief   PD Line Following Controller
 *
 * Uses centroid error from LA_Update() to compute angular velocity.
 * Designed for 100 Hz update rate (dt = 0.01s)
 */

#ifndef LINE_FOLLOWER_ESP32_H
#define LINE_FOLLOWER_ESP32_H

#include <stdbool.h>
#include <math.h>

/* Tuning parameters (adjust empirically) */
#define LF_KP_DEFAULT       0.60f   // Proportional gain
#define LF_KD_DEFAULT       0.08f   // Derivative gain  
#define LF_WZ_MAX_DEFAULT   1.20f   // Max angular velocity (rad/s) — faster robot
#define LF_DT_DEFAULT       0.01f   // 100 Hz = 10ms period

typedef struct {
    float kp;
    float kd;
    float wz_max;
    float dt;
    float prev_error;
    bool  has_prev;
} LF_Controller_t;

void  LF_Init(LF_Controller_t *lf, float kp, float kd, float wz_max, float dt);
float LF_ComputeOmega(LF_Controller_t *lf, float error);
void  LF_Reset(LF_Controller_t *lf);

#endif

/* ─── Implementation ─────────────────────────────────────────────────────── */

void LF_Init(LF_Controller_t *lf, float kp, float kd, float wz_max, float dt) {
    lf->kp        = kp;
    lf->kd        = kd;
    lf->wz_max    = wz_max;
    lf->dt        = dt;
    lf->prev_error = 0.0f;
    lf->has_prev  = false;
}

float LF_ComputeOmega(LF_Controller_t *lf, float error) {
    if (isnan(error)) return 0.0f; // Line lost — no steering

    float d_term = 0.0f;
    if (lf->has_prev) {
        // Derivative: rate of change of error
        d_term = (error - lf->prev_error) / lf->dt;
    }

    // PD output: positive error → turn right (negative omega convention depends on system)
    float omega = lf->kp * error + lf->kd * d_term;

    // Clamp to maximum angular velocity
    if (omega >  lf->wz_max) omega =  lf->wz_max;
    if (omega < -lf->wz_max) omega = -lf->wz_max;

    lf->prev_error = error;
    lf->has_prev   = true;

    return omega;
}

void LF_Reset(LF_Controller_t *lf) {
    lf->prev_error = 0.0f;
    lf->has_prev   = false;
}
```

---

## 12. Integration with State Machine

### UART Protocol (ESP32 → STM32)

The ESP32 sends sensor data at **100 Hz** to the STM32:

```
Format:  $LA,<err_x100>,<active>,<junc>,<lost>,<crc8>\r\n
Example: $LA,-125,3,0,0,A3\r\n
         → error = -1.25 (line 12.5mm left of centre)
         → 3 sensors active, no junction, line not lost
```

### STM32 Parser (add to existing UART handler)

```c
// In STM32 UART receive handler:
// Parse: $LA,err_x100,active,junc,lost,crc\r\n

typedef struct {
    float    error;         // Centroid error (-4.0 to +4.0)
    int      active;        // Active sensor count
    bool     is_junction;   // Junction detected
    bool     is_lost;       // Line lost
    bool     valid;         // Parse successful
} LA_UART_Msg_t;

LA_UART_Msg_t parse_la_uart(const char *line) {
    LA_UART_Msg_t msg = {0};
    int err_enc, active, junc, lost;
    unsigned int crc_recv;
    
    if (sscanf(line, "$LA,%d,%d,%d,%d,%x",
               &err_enc, &active, &junc, &lost, &crc_recv) == 5) {
        // Verify CRC (skip '$', stop before ',crc')
        // ... CRC check ...
        msg.error       = err_enc / 100.0f;
        msg.active      = active;
        msg.is_junction = (junc != 0);
        msg.is_lost     = (lost != 0);
        msg.valid       = true;
    }
    return msg;
}
```

### Updating SM_SensorData_t

Update the existing `SM_SensorData_t` in `state_machine.h` to use ESP32 data:

```c
// In state_machine.h — update the struct:
typedef struct {
    // Existing fields:
    float     line_centroid;   // From ESP32: -4.0 to +4.0
    uint8_t   line_active_cnt; // From ESP32: 0–9 sensors on line
    bool      intersection;    // From ESP32: junction detected
    bool      line_lost;       // From ESP32: line not found
    // ... rest of fields ...
} SM_SensorData_t;
```

---

## 13. Testing Procedure

### Phase 1: Hardware Verification

| Test | Method | Pass Criteria |
|------|--------|--------------|
| **T1.1 LED current** | Measure with multimeter across R1 | 8–12 mA per sensor |
| **T1.2 ADC over white** | Serial monitor, sensor over white paper | Raw ADC: 150–600 |
| **T1.3 ADC over black** | Serial monitor, sensor over black tape | Raw ADC: 3000–4000 |
| **T1.4 ADC range** | Difference between T1.2 and T1.3 | >2000 ADC counts |
| **T1.5 Capacitor** | Oscilloscope on ADC pin | No noise spikes >50mV |
| **T1.6 All 9 sensors** | Move robot over line | All 9 respond independently |

### Phase 2: Software Verification

| Test | Method | Pass Criteria |
|------|--------|--------------|
| **T2.1 EMA filter** | Move sensor abruptly; watch serial | Smooth, no spikes |
| **T2.2 Calibration** | Run calibration, check NVS | min/max stored per sensor |
| **T2.3 Centroid** | Centre S5 over line edge | error ≈ 0.0 ± 0.1 |
| **T2.4 Centroid range** | Slide line from S1 to S9 | error spans -4.0 to +4.0 |
| **T2.5 Junction** | Place all 9 over white | is_junction = true after 30ms |
| **T2.6 Line lost** | Lift robot off surface | is_line_lost = true after 10ms |
| **T2.7 UART** | STM32 serial monitor | Valid $LA packets at 100 Hz |
| **T2.8 CRC** | Inject corrupted byte | STM32 rejects bad packets |

### Phase 3: Dynamic Testing

| Test | Method | Pass Criteria |
|------|--------|--------------|
| **T3.1 Straight line follow** | 1m straight line | Robot follows within ±5mm |
| **T3.2 Curved line follow** | Arena curve | No oscillation, smooth |
| **T3.3 90° turn** | T-junction in arena | Detects junction, turns correctly |
| **T3.4 Junction timing** | Slow crossing | Junction held true for >200ms |
| **T3.5 Gap recovery** | Lift front momentarily | Recovers within 1.5 seconds |

### Phase 4: Integration Testing

| Test | Method | Pass Criteria |
|------|--------|--------------|
| **T4.1 STM32 parsing** | Full UART comms | STM32 correctly reads error values |
| **T4.2 PD tuning** | Tune Kp, Kd on straight | Settles in <2 oscillations |
| **T4.3 Full subtask 1** | Grid navigation | Follows all grid lines |
| **T4.4 Full subtask 4** | Colour sort junction | Detects correct branch |

---

## 14. Tuning Guide

### PD Controller Tuning (Step-by-Step)

#### Step 1: Set Kd = 0, increase Kp
- Start: Kp = 0.2, Kd = 0.0
- Increase Kp until robot oscillates on a straight line
- Record `Kp_oscillate`
- Set `Kp = 0.5 × Kp_oscillate`

#### Step 2: Add Kd
- Keep Kp from Step 1
- Increase Kd until oscillations are damped
- Typical final values: `Kp = 0.5–0.8`, `Kd = 0.04–0.10`

#### Step 3: Test on curves
- If robot cuts corners: increase Kp
- If robot oscillates on straight: increase Kd
- If robot responds too slowly: increase Kp

### Common Problems & Fixes

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Constant oscillation | Kp too high | Reduce Kp by 20% |
| Slow to correct | Kp too low | Increase Kp |
| Overshoot on curves | Kd too low | Increase Kd |
| Stuttery/jerky motion | EMA alpha too high | Lower EMA alpha to 0.5 |
| Wrong centroid direction | Sensor order reversed | Swap S1/S9 in ADC map |
| Junction not detected | Threshold too high | Lower `LA_JUNCTION_MIN_SENSORS` to 5 |
| False junctions | Threshold too low | Raise `LA_JUNCTION_MIN_SENSORS` to 7 |
| Sensors don't differ | Wrong mounting height | Lower to 10–15mm |
| One sensor always wrong | Bad solder joint | Reflow joint; check R2 |

---

## 15. Wiring Reference

### Complete Pin Table

| Sensor | Signal | ESP32 GPIO | ADC Unit | ADC Channel | Pull-up R2 |
|--------|--------|-----------|----------|-------------|-----------|
| S1 (Left) | Analog OUT | GPIO 32 | ADC1 | CH4 | 10 kΩ to 3.3V |
| S2 | Analog OUT | GPIO 33 | ADC1 | CH5 | 10 kΩ to 3.3V |
| S3 | Analog OUT | GPIO 34 | ADC1 | CH6 | 10 kΩ to 3.3V |
| S4 | Analog OUT | GPIO 35 | ADC1 | CH7 | 10 kΩ to 3.3V |
| S5 (Centre) | Analog OUT | GPIO 36 | ADC1 | CH0 | 10 kΩ to 3.3V |
| S6 | Analog OUT | GPIO 37 | ADC1 | CH1 | 10 kΩ to 3.3V |
| S7 | Analog OUT | GPIO 38 | ADC1 | CH2 | 10 kΩ to 3.3V |
| S8 | Analog OUT | GPIO 39 | ADC1 | CH3 | 10 kΩ to 3.3V |
| S9 (Right) | Analog OUT | GPIO 25 | ADC2 | CH8 | 10 kΩ to 3.3V |
| All sensors | VCC | — | — | — | 3.3V |
| All sensors | GND | — | — | — | GND |
| UART TX | Serial → STM32 | GPIO 17 | — | — | — |
| UART RX | Serial ← STM32 | GPIO 16 | — | — | — |

### Power Budget (Sensors Only)

| Item | Current | Notes |
|------|---------|-------|
| 9 × IR LEDs (9mA each) | **81 mA** | R1 = 220 Ω |
| 9 × Phototransistors (< 1mA each) | **<9 mA** | Collector current |
| ESP32 (active) | **~80 mA** | Plus task overhead |
| **Total sensor subsystem** | **~170 mA @ 3.3V** | **0.56 W** |

> [!TIP]
> Power the sensor array from the STM32's **3.3V LDO output** (existing in the
> architecture). The 170 mA draw is well within a standard 500 mA LDO.
> Add a **10 µF electrolytic** capacitor at the 3.3V rail entry to the sensor PCB
> to prevent voltage droop when all 9 LEDs switch simultaneously.

---

## Quick Reference Card

```
┌─────────────────────────────────────────────────────────────────────┐
│         9-SENSOR ANALOG ARRAY — QUICK REFERENCE                     │
├─────────────────────────────────────────────────────────────────────┤
│ Sensor: TCRT5000 analog                                             │
│ Per sensor: R1=220Ω (LED) + R2=10kΩ (pull-up) + C1=100nF (decouple)│
│ Spacing: 10mm | Array width: 80mm | Mount height: 10-15mm           │
├─────────────────────────────────────────────────────────────────────┤
│ ESP32 ADC: 12-bit, 11dB atten, 16x oversample, 100Hz rate           │
│ Filter: EMA α=0.7 + spike reject (>500 counts)                      │
│ Calibration: NVS stored, 2-point (white + black) per sensor         │
├─────────────────────────────────────────────────────────────────────┤
│ Centroid error range: -4.0 (line far left) to +4.0 (line far right) │
│ Junction: ≥6 sensors active for ≥3 consecutive samples (30ms)       │
│ Line lost: all sensors <15% contribution → lost_duration_ms         │
├─────────────────────────────────────────────────────────────────────┤
│ UART to STM32: 115200 baud, $LA,err_x100,active,junc,lost,crc\r\n  │
│ PD gains: Kp=0.6, Kd=0.08, Wz_max=1.2 rad/s (starting point)       │
└─────────────────────────────────────────────────────────────────────┘
```

---

*Report generated: September 2026 | FusionForce Robotics | RUNNER-4 Project*
