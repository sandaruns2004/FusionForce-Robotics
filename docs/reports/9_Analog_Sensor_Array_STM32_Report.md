# RUNNER-4 | 9-Channel Analog IR Sensor Array
## Full Technical Report — STM32 HAL Implementation
### FusionForce Robotics — EN2533 BREACH PROTOCOL

> [!IMPORTANT]
> **MCU: STM32F411CEU6 (Black Pill)** — 100 MHz Cortex-M4F, 512 KB Flash, 128 KB RAM.
> This is a **planned migration**. The current production hardware uses the
> 8-channel **digital** TCRT5000 driver ([`line_array.c`](file:///c:/Users/ADMIN/Desktop/FusionForce-Robotics/firmware/Drivers/LineArray/line_array.c) /
> [`line_array.h`](file:///c:/Users/ADMIN/Desktop/FusionForce-Robotics/firmware/Drivers/LineArray/line_array.h)).
> The 9-channel analog driver ([`line_array_9ch.h`](file:///c:/Users/ADMIN/Desktop/FusionForce-Robotics/firmware/Drivers/LineArray/line_array_9ch.h) /
> [`line_array_9ch.c`](file:///c:/Users/ADMIN/Desktop/FusionForce-Robotics/firmware/Drivers/LineArray/line_array_9ch.c)) exists but
> **`state_machine.h` still includes `line_array.h`** — full integration is pending.
> Migration gives sub-pixel centroid resolution, per-sensor noise filtering,
> and automatic calibration — critical for curved corridor and colour-sort tasks.

---

## Table of Contents
1. [Sensor Overview](#1-sensor-overview)
2. [Why 9 Sensors? Why Analog? Why Stay on STM32?](#2-why-9-sensors-why-analog-why-stay-on-stm32)
3. [Hardware Circuit — Per-Sensor Detail](#3-hardware-circuit--per-sensor-detail)
4. [Sensor Array Physical Layout](#4-sensor-array-physical-layout)
5. [STM32 ADC Architecture](#5-stm32-adc-architecture)
6. [Signal Conditioning & Filters](#6-signal-conditioning--filters)
7. [Calibration Algorithm](#7-calibration-algorithm)
8. [Weighted Centroid & Line Position](#8-weighted-centroid--line-position)
9. [Intersection & Junction Detection](#9-intersection--junction-detection)
10. [Line-Lost Detection & Recovery](#10-line-lost-detection--recovery)
11. [Complete STM32 Driver Code](#11-complete-stm32-driver-code)
12. [Integration with State Machine](#12-integration-with-state-machine)
13. [Testing Procedure](#13-testing-procedure)
14. [Tuning Guide](#14-tuning-guide)
15. [Wiring Reference](#15-wiring-reference)

---

## 1. Sensor Overview

### What is a TCRT5000-based Analog Sensor?
Each module in the array is a reflective IR proximity sensor with:

| Component | Part | Function |
|-----------|------|----------|
| IR Emitter | IR LED (λ=950 nm) | Continuously emits infrared light downward |
| IR Detector | Phototransistor | Collects reflected IR from surface |
| Resistor R1 | ~220 Ω (emitter) | Limits IR LED current to ~9 mA |
| Resistor R2 | ~10 kΩ (collector) | Pull-up; converts phototransistor current → voltage |
| Capacitor C1 | 100 nF ceramic | Decoupling cap — filters power-rail noise |

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
> Logic is **INVERTED**: White/reflective = LOW voltage, Black/absorptive = HIGH
> voltage. The driver below handles this correctly with the `line_val[]`
> inversion step.

### Analog vs Digital Comparison

| Feature | Digital Output | Analog Output (Ours) |
|---------|---------------|----------------------|
| Resolution | 1 bit (0 or 1) | 12-bit (0–4095) |
| Centroid accuracy | ±0.5 sensor pitch | ±0.01 sensor pitch |
| Calibration | Fixed threshold | Per-sensor min/max in Flash |
| Surface variation handling | Poor | Excellent |
| Noise filtering | None (hysteresis only) | Multi-layer (oversampling + EMA + outlier) |
| Junction detection | Count-based only | Area + gradient analysis |

---

## 2. Why 9 Sensors? Why Analog? Why Stay on STM32?

### 9 Sensors vs 8 Sensors
The existing driver ([`line_array.h`](file:///c:/Users/ADMIN/Desktop/FusionForce-Robotics/firmware/Drivers/LineArray/line_array.h)) uses **8 sensors** (even number → no true centre sensor). With 8 sensors, when the line is dead-centre, two sensors (S4 and S5) are both partially active — causing centroid instability and oscillation in the PD controller.

With **9 sensors** (odd number):
- Sensor **S5** is the true centre
- When perfectly aligned, only S5 activates → centroid = 0.0 exactly
- Eliminates the centre-ambiguity oscillation problem
- Provides ±4 sensor range on each side (better coverage for sharp curves)

```
S1   S2   S3   S4  [S5]  S6   S7   S8   S9
 |    |    |    |    |    |    |    |    |
-4   -3   -2   -1   0   +1   +2   +3   +4    ← Position index
```

### Why Analog on STM32?
The **STM32F411CEU6** has a 12-bit SAR ADC with 16 multiplexed channels, supporting scan mode + DMA. Our 9 sensors fit on ADC1 channels IN0–IN8 (PA0–PA7 + PB0) **without any external multiplexer**.

> [!NOTE]
> **Current hardware**: PA0–PA7 are configured as digital GPIO inputs (pull-down) for the 8-channel
> TCRT5000 array. Migration to analog requires reconfiguring these pins in STM32CubeMX from
> `GPIO_Input` to `ADC1_IN0`–`ADC1_IN8`, and adding PB0 as `ADC1_IN8` for S9.

12-bit resolution gives 4096 levels per sensor → sub-millimetre line position accuracy → smoother PD control → less oscillation in curved corridors.

### Why Stay on STM32 Instead of Using an ESP32 Co-Processor?
| Criterion | Two-MCU (STM32 + ESP32) | Single STM32 (Ours) |
|-----------|------------------------|---------------------|
| ADC channels available | 18 (ESP32) | 16 (STM32F411) — sufficient |
| UART latency | +1 ms protocol overhead | Zero — direct register access |
| Synchronisation | Requires UART framing + CRC | Deterministic — same clock domain |
| Code complexity | Two firmware projects | Single codebase |
| Power consumption | +80 mA (ESP32 active) | ~5 mA ADC overhead only |
| Failure modes | UART drop, ESP32 crash | Fewer; simpler debugging |

The STM32F411CEU6 has **enough ADC channels and processing headroom** to handle 9 analog sensors natively via DMA, eliminating the co-processor approach entirely.

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
                                       ├──────────── ADC Input (STM32 GPIO)
                                       │
                                    ┌──┴──┐
                                    │Photo│  (Collector, TCRT5000)
                                    │Trans│
                                    └──┬──┘
                                       │
GND ───────────────────────────────────┘

Decoupling capacitor (100 nF ceramic — place as close as possible to sensor):
3.3V ──┬── C1 (100 nF) ──┬── GND
       │                 │
   Power rail         Power rail
```

### Component Values Explained

**R1 — IR LED Current Limiter (220 Ω)**
- TCRT5000 LED: Vf ≈ 1.25 V, recommended If = 10–20 mA
- R1 = (Vcc − Vf) / If = (3.3 − 1.25) / 0.015 = 137 Ω
- 220 Ω gives ~9 mA — slightly conservative but safe at 3.3 V supply
- For 9 sensors at 9 mA each = **81 mA total LED current** (acceptable)

**R2 — Phototransistor Load (10 kΩ)**
- Over white: Rce ≈ 500 Ω → Vout = 3.3 × 500/(500+10000) ≈ **0.16 V (LOW)**
- Over black: Rce ≈ ∞  → Vout ≈ **3.3 V (HIGH)**
- 10 kΩ is the standard sweet spot for ≤200 mm/s robots

**C1 — 100 nF Ceramic Decoupling**
- Filters high-frequency noise on the 3.3 V rail near each sensor
- Placed as physically close as possible to the sensor's VCC/GND pins
- **Every sensor must have its own 100 nF cap** — do not share one cap for all 9
- Additionally, add one 10 µF electrolytic across the main 3.3 V rail for bulk decoupling

**STM32 ADC Input Protection**
Add an optional 10 kΩ series resistor between the R2 junction and the STM32 ADC pin:
- Protects the STM32 ADC input from overvoltage transients
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
| Sensor spacing | 10 mm | Arena line = 30 mm wide → ~3 sensors on line |
| Array width | 80 mm (9 × 10 mm − 10 mm ends) | Fits within robot footprint |
| Mounting height | 10–15 mm above ground | TCRT5000 optimal range: 2–15 mm |
| Tilt angle | 0° (flat, parallel to ground) | Maximises reflectance uniformity |
| Position | Front underside, 10 mm from front edge | Ahead of front legs |

### Why 10 mm Spacing?
- Arena line width = 30 mm → 3 sensors will be on the line simultaneously
- Range: ±4 × 10 mm = ±40 mm from centre
- Theoretical accuracy: ±(10 mm / 4096 ADC levels) ≈ ±0.002 mm
- Practical accuracy: ~±0.5 mm (limited by noise floor)

---

## 5. STM32 ADC Architecture

### ADC Channel Assignment

```
STM32 GPIO Pin → ADC Channel → Sensor
────────────────────────────────────────
PA0  → ADC1_IN0  → S1 (Leftmost)
PA1  → ADC1_IN1  → S2
PA2  → ADC1_IN2  → S3
PA3  → ADC1_IN3  → S4
PA4  → ADC1_IN4  → S5 (Centre)
PA5  → ADC1_IN5  → S6
PA6  → ADC1_IN6  → S7
PA7  → ADC1_IN7  → S8
PB0  → ADC1_IN8  → S9 (Rightmost)
```

> [!NOTE]
> All 9 sensors are on **ADC1** — no second ADC or external MUX needed. This is
> a key advantage over the ESP32 approach, which required ADC2 for S9 and
> suffered from Wi-Fi interference.

### STM32CubeMX ADC1 Configuration

| Parameter | Value |
|-----------|-------|
| Mode | Scan Conversion + Continuous + DMA |
| Resolution | 12-bit (0–4095) |
| Data alignment | Right |
| Sampling time | 480 cycles (longest available — maximises SNR) |
| Number of conversions | 9 |
| DMA mode | Circular |
| DMA data width | Word (32-bit) → cast to uint16_t |
| External trigger | None (software start, runs continuously) |
| EOC selection | End of sequence (DMA TC interrupt) |

```
STM32CubeMX → Analog → ADC1
  ✓ IN0 (PA0)  ✓ IN1 (PA1)  ✓ IN2 (PA2)  ✓ IN3 (PA3)  ✓ IN4 (PA4)
  ✓ IN5 (PA5)  ✓ IN6 (PA6)  ✓ IN7 (PA7)  ✓ IN8 (PB0)
  Scan Conversion Mode: ENABLE
  Continuous Conversion Mode: ENABLE
  DMA Continuous Requests: ENABLE
  End Of Conversion Selection: EOC flag at end of all conversions

STM32CubeMX → DMA → ADC1 → DMA1 Stream0 Ch0
  Direction: Peripheral to Memory
  Increment Address: Memory only
  Data Width: Word → Word
  Mode: Circular
```

### ADC Non-Linearity Note
The STM32F401 ADC is linear from ~100 mV to ~3200 mV. Our sensor output ranges between ~150 mV (white) and ~3100 mV (black) — safely within the linear region. The 2-point per-sensor calibration (min/max) implicitly handles any residual non-linearity.

---

## 6. Signal Conditioning & Filters

We apply 3 layers of filtering in sequence:

```
DMA Buffer (16 raw values per sensor) → [Layer 1: Oversampling Average]
  → [Layer 2: EMA Filter + Layer 3: Spike Rejection] → Normalised [0.0–1.0]
  → Calibration mapping → line_val [0.0–1.0]
```

### Layer 1: Oversampling (DMA Hardware Averaging)
The DMA buffer holds `LA_DMA_BUFFER_SIZE = 9 × 16 = 144` uint32_t values (9 sensors × 16 consecutive scan cycles). Each sensor's 16 raw values are averaged in software:

```c
// Average 16 raw DMA values per sensor
uint16_t sum = 0;
for (int k = 0; k < LA_OVERSAMPLE_N; k++) {
    sum += (uint16_t)dma_buf[i + k * LA_NUM_SENSORS];
}
uint16_t avg = sum / LA_OVERSAMPLE_N;
```

- Reduces random ADC noise by √16 = 4×
- Effective resolution increase: 12-bit → ~14-bit equivalent
- Cost: zero CPU overhead (DMA runs in background)

### Layer 2: Exponential Moving Average (EMA)

```
filtered[n] = alpha × raw[n] + (1 − alpha) × filtered[n−1]
```

| Alpha | Effect |
|-------|--------|
| 1.0 | No filtering (raw) |
| 0.5 | Medium smoothing (~2 sample lag) |
| 0.7 | **Recommended** — fast response, good noise rejection |
| 0.3 | Heavy smoothing (~3 sample lag) |

At 100 Hz with α = 0.7, a step change settles within 2–3 samples (20–30 ms) — fast enough for an 80 mm/s robot crossing a 30 mm line (crossing time = 375 ms).

### Layer 3: Outlier / Spike Rejection

```
If |raw[n] − filtered[n−1]| > LA_SPIKE_THRESHOLD (500 counts):
    → Use filtered[n−1] instead (reject spike)
Else:
    → Apply EMA normally
```

`SPIKE_THRESHOLD = 500` ADC counts (~0.4 V) rejects electrical transients from motor PWM and servo switching without affecting valid rapid line crossings.

### Filter Summary

| Layer | Technique | Noise Removed | Latency Added |
|-------|-----------|--------------|--------------|
| 1 | 16× oversampling (DMA) | Random ADC noise ±8 counts | 0 ms (DMA, hardware) |
| 2 | EMA α=0.7 | Temporal sensor flutter | ~15 ms |
| 3 | Spike filter | EMI transients from motors/servos | 0 ms |
| **Total** | | **>95% noise reduction** | **~15 ms** |

---

## 7. Calibration Algorithm

### Why Calibrate?
Each of the 9 sensors has slightly different LED forward voltage, phototransistor gain, R1/R2 tolerances (±5%), and mounting angle variations. Without calibration, edge sensors produce different raw values than the centre sensor over identical surfaces, causing centroid bias.

### Calibration Procedure

**Step 1: White Surface**
Place robot over white tape / white paper covering all 9 sensors.
Call `LA_Calibrate(true)` — records `cal_min[i]` (ADC over white).

**Step 2: Black Surface**
Place robot over black electrical tape.
Call `LA_Calibrate(false)` — records `cal_max[i]` (ADC over black), validates, and writes to Flash.

**Step 3: Normalise at Runtime**

```c
// Raw value: higher = blacker
// Normalised: 0.0 = white (on line), 1.0 = black (off line)
normalised[i] = (raw[i] - cal_min[i]) / (cal_max[i] - cal_min[i]);
normalised[i] = clamp(normalised[i], 0.0f, 1.0f);

// Line value (inverted): 1.0 = on line (white), 0.0 = off line (black)
line_val[i] = 1.0f - normalised[i];
```

### Calibration Storage — STM32 Internal Flash
```c
/* Calibration Flash storage */
/* ⚠️ WARNING: On STM32F411, 0x0807F800 (Sector 7) is ALSO used by state_machine.c
 *  for ball colour persistence (BALL_COLOR_FLASH_ADDR). Choose a different sector
 *  (e.g. Sector 6 @ 0x08060000) or use sub-page offsets to avoid collision.      */
#define LA_CAL_FLASH_ADDR           0x0807F800UL  ///< Sector 7 — SHARED with ball colour!
#define LA_CAL_FLASH_SECTOR         FLASH_SECTOR_7
```
- Survives power cycles indefinitely (Flash endurance: 10,000 write cycles)
- Uses a **magic word** (`0xCAFEBEEF`) to detect uninitialized/corrupted data
- Structure: `LA_Calibration_t` (magic + 9× min + 9× max + valid flag)

> [!WARNING]
> Always erase the target Flash page before writing. Use `HAL_FLASH_Unlock()`,
> `HAL_FLASHEx_Erase()`, then `HAL_FLASH_Program()` in sequence. See
> `prv_save_cal_to_flash()` in the driver source below.

---

## 8. Weighted Centroid & Line Position

### Centroid Formula

```
position = Σ(i × line_val[i]) / Σ(line_val[i])
           for i = 0 to 8 (9 sensors)

error = position − 4.0     ← centre sensor S5 = index 4
error range: −4.0 to +4.0
error = 0.0 → line directly under S5 (perfect alignment)
error < 0   → line is LEFT  → robot must steer left
error > 0   → line is RIGHT → robot must steer right
```

### Converting to Physical Units

```c
error_mm = error × 10.0f;   // mm from centre (10 mm sensor spacing)
```

Error of ±1.0 = line displaced 10 mm from centre.

### Centroid Stability Improvement
Only include sensors whose `line_val` exceeds a minimum contribution threshold to prevent noise-dominated sensors from perturbing the centroid:

```c
#define LA_CONTRIB_THRESHOLD  0.15f  // Sensor must be ≥15% "on" to contribute

float sum_pos = 0.0f, sum_weight = 0.0f;
for (int i = 0; i < LA_NUM_SENSORS; i++) {
    if (line_val[i] > LA_CONTRIB_THRESHOLD) {
        sum_pos    += (float)i * line_val[i];
        sum_weight += line_val[i];
    }
}
float centroid = (sum_weight > 0.0f) ? (sum_pos / sum_weight) : NAN;
float error    = centroid - 4.0f;
```

---

## 9. Intersection & Junction Detection

### Logic

```
active_count = sensors with line_val > LA_ACTIVE_THRESHOLD (0.5)

Junction detected when:
  active_count  >= LA_JUNCTION_MIN_SENSORS     (6 of 9)
  for LA_JUNCTION_CONSEC_SAMPLES consecutive samples (3 × 10 ms = 30 ms)
```

### 3-Sample Temporal Filter (Critical)
Without temporal filtering, a robot at 80 mm/s crosses a 30 mm junction in 375 ms → ~37 samples at 100 Hz. A 3-sample debounce prevents motor noise spikes from triggering false detections:

```c
static int _junction_consec = 0;

bool update_junction(float *line_val) {
    int active = 0;
    for (int i = 0; i < LA_NUM_SENSORS; i++)
        if (line_val[i] > 0.5f) active++;

    if (active >= LA_JUNCTION_MIN_SENSORS) {
        _junction_consec++;
        if (_junction_consec >= LA_JUNCTION_CONSEC_SAMPLES) return true;
    } else {
        _junction_consec = 0;
    }
    return false;
}
```

### Junction Type Classification

| Pattern | Active Sensors | Junction Type |
|---------|---------------|--------------|
| All 9 active | S1–S9 | + junction (grid cross) |
| S1–S5 active, S6–S9 off | Left half | T-junction (branch left) |
| S5–S9 active, S1–S4 off | Right half | T-junction (branch right) |
| All 9 briefly then gone | Flash | End of line |
| S4, S5, S6 only | Centre 3 | Normal line (no junction) |

---

## 10. Line-Lost Detection & Recovery

### Detection

```c
bool line_lost = (sum_weight == 0.0f);  // No sensor above CONTRIB_THRESHOLD
_lost_cycles++;                          // Increment every 10 ms
// After 150 cycles (1.5 s) with no line → enter LINE_LOST_RECOVERY
```

### Recovery Strategy

```
1. STOP forward motion (Vx = 0)
2. Apply last-known steering direction (hold last error sign)
3. Slowly rotate in that direction
4. If line found → resume normal following
5. If not found after 3 seconds → STATE_ERROR_RECOVERY (state machine)
```

---

## 11. Complete STM32 Driver Code

### File: `line_array_9ch.h`

See [`line_array_9ch.h`](file:///c:/Users/ADMIN/Desktop/FusionForce-Robotics/firmware/Drivers/LineArray/line_array_9ch.h) — the header with all configuration constants, data types, and public API declarations.

### File: `line_array_9ch.c`

```c
/**
 * @file    line_array_9ch.c
 * @brief   9-Channel Analog IR Sensor Array Driver — STM32 HAL Implementation
 *
 * Dependencies (STM32CubeMX generated):
 *   - ADC1 in Scan + Continuous + DMA Circular mode, 9 channels, 12-bit
 *   - DMA1 Stream0 Ch0, Peripheral→Memory, Circular, Word width
 *   - stm32f4xx_hal.h, stm32f4xx_hal_adc.h, stm32f4xx_hal_flash_ex.h
 *
 * Build: STM32CubeIDE, C11, O2 optimisation
 */

#include "line_array_9ch.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

/* ─── DMA Buffer ───────────────────────────────────────────────────────────── */
/*  Layout: [S1_0, S2_0, ..., S9_0,  S1_1, S2_1, ..., S9_1, ... × 16]
 *  DMA fills this continuously in circular mode. We snapshot it in LA_Update().  */
static volatile uint32_t _dma_buf[LA_DMA_BUFFER_SIZE];

/* ─── Internal State ───────────────────────────────────────────────────────── */
static ADC_HandleTypeDef *_hadc = NULL;
static LA_Calibration_t   _cal;
static LA_Result_t        _last_result;
static float              _ema[LA_NUM_SENSORS];
static bool               _ema_init[LA_NUM_SENSORS];
static int                _junction_consec = 0;
static uint32_t           _lost_cycles     = 0;
static bool               _initialised     = false;

/* ─── Private Helpers ──────────────────────────────────────────────────────── */

/**
 * @brief Average LA_OVERSAMPLE_N raw DMA values for one sensor index.
 *        DMA buffer layout: sample s of sensor i = _dma_buf[s * LA_NUM_SENSORS + i]
 */
static uint16_t prv_oversample(int sensor_idx) {
    uint32_t sum = 0;
    for (int s = 0; s < LA_OVERSAMPLE_N; s++) {
        sum += _dma_buf[s * LA_NUM_SENSORS + sensor_idx];
    }
    return (uint16_t)(sum / LA_OVERSAMPLE_N);
}

/**
 * @brief Apply EMA filter with spike rejection.
 * @param idx   Sensor index 0–8
 * @param raw   New oversampled ADC value
 * @return      Filtered ADC value (float)
 */
static float prv_apply_ema(int idx, uint16_t raw) {
    float fraw = (float)raw;
    if (!_ema_init[idx]) {
        _ema[idx]      = fraw;
        _ema_init[idx] = true;
        return fraw;
    }
    /* Spike rejection */
    if (fabsf(fraw - _ema[idx]) > (float)LA_SPIKE_THRESHOLD) {
        fraw = _ema[idx];  /* Reject — keep previous */
    }
    /* EMA */
    _ema[idx] = LA_EMA_ALPHA * fraw + (1.0f - LA_EMA_ALPHA) * _ema[idx];
    return _ema[idx];
}

/**
 * @brief Normalise a filtered ADC value to [0.0=white, 1.0=black].
 */
static float prv_normalise(int idx, float filtered) {
    float range = (float)(_cal.max_val[idx] - _cal.min_val[idx]);
    if (range < 10.0f) range = 10.0f;  /* Avoid divide-by-zero */
    float norm = (filtered - (float)_cal.min_val[idx]) / range;
    if (norm < 0.0f) norm = 0.0f;
    if (norm > 1.0f) norm = 1.0f;
    return norm;
}

/**
 * @brief Set default calibration values (fallback when Flash uninitialized).
 */
static void prv_set_default_cal(void) {
    for (int i = 0; i < LA_NUM_SENSORS; i++) {
        _cal.min_val[i] = LA_CAL_DEFAULT_MIN;
        _cal.max_val[i] = LA_CAL_DEFAULT_MAX;
    }
    _cal.magic    = 0x00000000UL;
    _cal.is_valid = false;
}

/**
 * @brief Load calibration from internal Flash.
 * @return true if valid magic word found
 */
static bool prv_load_cal_from_flash(void) {
    const LA_Calibration_t *flash_cal = (const LA_Calibration_t *)LA_CAL_FLASH_ADDR;
    if (flash_cal->magic != LA_CAL_MAGIC) return false;

    memcpy(&_cal, flash_cal, sizeof(LA_Calibration_t));
    _cal.is_valid = true;
    return true;
}

/**
 * @brief Erase calibration Flash page and write current _cal struct.
 * @return HAL_OK on success
 */
static HAL_StatusTypeDef prv_save_cal_to_flash(void) {
    HAL_StatusTypeDef status;

    /* Unlock Flash */
    status = HAL_FLASH_Unlock();
    if (status != HAL_OK) return status;

    /* Erase target sector (STM32F401: sector 7 = 0x0807F800, 128 KB sector 7)
     * Adjust VoltageRange and Sector for your specific MCU variant.          */
    FLASH_EraseInitTypeDef erase = {
        .TypeErase    = FLASH_TYPEERASE_SECTORS,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3,  /* 2.7V–3.6V supply */
        .Sector       = FLASH_SECTOR_7,
        .NbSectors    = 1,
    };
    uint32_t sector_error = 0;
    status = HAL_FLASHEx_Erase(&erase, &sector_error);
    if (status != HAL_OK) { HAL_FLASH_Lock(); return status; }

    /* Write calibration struct word by word */
    _cal.magic    = LA_CAL_MAGIC;
    _cal.is_valid = true;
    const uint32_t *src  = (const uint32_t *)&_cal;
    uint32_t        addr = LA_CAL_FLASH_ADDR;
    for (size_t i = 0; i < sizeof(LA_Calibration_t) / 4; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, src[i]);
        if (status != HAL_OK) { HAL_FLASH_Lock(); return status; }
        addr += 4;
    }

    HAL_FLASH_Lock();
    return HAL_OK;
}

/* ─── Public API Implementation ────────────────────────────────────────────── */

void LA_Init(ADC_HandleTypeDef *hadc) {
    if (_initialised) return;

    _hadc = hadc;

    /* Start ADC1 in DMA circular mode — fills _dma_buf continuously */
    HAL_ADC_Start_DMA(_hadc, (uint32_t *)_dma_buf, LA_DMA_BUFFER_SIZE);

    /* Initialise EMA state */
    memset(_ema,      0, sizeof(_ema));
    memset(_ema_init, 0, sizeof(_ema_init));

    /* Load calibration from Flash; fall back to defaults if not found */
    if (!prv_load_cal_from_flash()) {
        prv_set_default_cal();
        /* Warning: use LA_PrintDebug or UART to alert operator */
    }

    /* Clear result state */
    memset(&_last_result, 0, sizeof(_last_result));
    _last_result.centroid = 4.0f;
    _last_result.error    = 0.0f;
    _junction_consec      = 0;
    _lost_cycles          = 0;

    _initialised = true;
}

void LA_Calibrate(bool white_surface) {
    if (!_initialised) return;

    /* Accumulate 32 averaged readings for stable calibration */
    const int CAL_ROUNDS = 32;
    uint32_t acc[LA_NUM_SENSORS] = {0};

    for (int round = 0; round < CAL_ROUNDS; round++) {
        /* Wait for one DMA scan cycle (~1 ms at 480-cycle sampling) */
        HAL_Delay(5);
        for (int i = 0; i < LA_NUM_SENSORS; i++) {
            acc[i] += prv_oversample(i);
        }
    }

    for (int i = 0; i < LA_NUM_SENSORS; i++) {
        uint16_t avg = (uint16_t)(acc[i] / CAL_ROUNDS);
        if (white_surface) {
            _cal.min_val[i] = avg;
        } else {
            _cal.max_val[i] = avg;
        }
    }

    /* After BLACK step: validate and write to Flash */
    if (!white_surface) {
        bool valid = true;
        for (int i = 0; i < LA_NUM_SENSORS; i++) {
            if (_cal.max_val[i] < _cal.min_val[i] + 200) {
                valid = false;  /* Black not dark enough vs white */
            }
        }
        if (valid) {
            prv_save_cal_to_flash();
            _cal.is_valid = true;
        }
    }
}

void LA_Update(LA_Result_t *result) {
    if (!_initialised || !result) return;

    LA_Result_t r;
    memset(&r, 0, sizeof(r));

    /* ── Step 1: Snapshot DMA buffer and average 16 samples per sensor ─── */
    for (int i = 0; i < LA_NUM_SENSORS; i++) {
        r.adc_raw[i] = prv_oversample(i);
    }

    /* ── Step 2–4: EMA filter → normalise → invert for line_val ──────── */
    for (int i = 0; i < LA_NUM_SENSORS; i++) {
        float filtered   = prv_apply_ema(i, r.adc_raw[i]);
        r.raw_norm[i]    = prv_normalise(i, filtered);
        r.line_val[i]    = 1.0f - r.raw_norm[i];  /* 1.0 = on line (white) */
    }

    /* ── Step 5: Count active sensors ────────────────────────────────── */
    r.active_count = 0;
    for (int i = 0; i < LA_NUM_SENSORS; i++) {
        if (r.line_val[i] > LA_ACTIVE_THRESHOLD) r.active_count++;
    }

    /* ── Step 6: Weighted centroid ────────────────────────────────────── */
    float sum_pos = 0.0f, sum_wt = 0.0f;
    for (int i = 0; i < LA_NUM_SENSORS; i++) {
        if (r.line_val[i] > LA_CONTRIB_THRESHOLD) {
            sum_pos += (float)i * r.line_val[i];
            sum_wt  += r.line_val[i];
        }
    }

    if (sum_wt > 0.0f) {
        r.centroid         = sum_pos / sum_wt;
        r.error            = r.centroid - LA_CENTRE_INDEX;  /* −4.0 to +4.0 */
        r.error_mm         = r.error * LA_SENSOR_SPACING_MM;
        r.is_line_lost     = false;
        _lost_cycles       = 0;
    } else {
        r.centroid         = NAN;
        r.error            = NAN;
        r.error_mm         = NAN;
        r.is_line_lost     = true;
        _lost_cycles++;
        r.lost_duration_ms = _lost_cycles * (1000 / LA_UPDATE_RATE_HZ);
    }

    /* ── Step 7: Junction detection ───────────────────────────────────── */
    if (r.active_count >= LA_JUNCTION_MIN_SENSORS) {
        _junction_consec++;
        if (_junction_consec >= LA_JUNCTION_CONSEC_SAMPLES) {
            r.is_junction = true;
            /* Saturate to prevent overflow */
            if (_junction_consec > LA_JUNCTION_CONSEC_SAMPLES + 20)
                _junction_consec = LA_JUNCTION_CONSEC_SAMPLES;
        }
    } else {
        _junction_consec = 0;
        r.is_junction    = false;
    }

    memcpy(&_last_result, &r, sizeof(LA_Result_t));
    memcpy(result, &r, sizeof(LA_Result_t));
}

void LA_Reset(void) {
    _junction_consec = 0;
    _lost_cycles     = 0;
}

void LA_PrintDebug(const LA_Result_t *r, UART_HandleTypeDef *huart) {
    if (!r || !huart) return;
    char buf[128];
    int  n = 0;

    n += snprintf(buf + n, sizeof(buf) - n, "[LA] ");
    for (int i = 0; i < LA_NUM_SENSORS; i++) {
        buf[n++] = (r->line_val[i] > LA_ACTIVE_THRESHOLD) ? '#' : '.';
    }
    if (!isnan(r->error)) {
        n += snprintf(buf + n, sizeof(buf) - n,
                      " | err=%.2f (%.1fmm) | act=%d%s%s\r\n",
                      r->error, r->error_mm, r->active_count,
                      r->is_junction  ? " JXN"  : "",
                      r->is_line_lost ? " LOST" : "");
    } else {
        n += snprintf(buf + n, sizeof(buf) - n,
                      " | LINE LOST (%lu ms)\r\n", r->lost_duration_ms);
    }
    HAL_UART_Transmit(huart, (uint8_t *)buf, (uint16_t)n, 10);
}

void LA_GetCalibration(LA_Calibration_t *cal) {
    if (cal) memcpy(cal, &_cal, sizeof(LA_Calibration_t));
}

/* ─── PD Line Following Controller ────────────────────────────────────────── */

void LF_Init(LF_Controller_t *lf, float kp, float kd, float wz_max, float dt) {
    lf->kp         = kp;
    lf->kd         = kd;
    lf->wz_max     = wz_max;
    lf->dt         = dt;
    lf->prev_error = 0.0f;
    lf->has_prev   = false;
}

float LF_ComputeOmega(LF_Controller_t *lf, float error) {
    if (isnan(error)) return 0.0f;  /* Line lost — no steering */

    float d_term = 0.0f;
    if (lf->has_prev) {
        d_term = (error - lf->prev_error) / lf->dt;
    }

    float omega = lf->kp * error + lf->kd * d_term;

    /* Clamp to max angular velocity */
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

### File: `sensor_task.c` — FreeRTOS Sensor Task

```c
/**
 * @file    sensor_task.c
 * @brief   FreeRTOS task: reads line array at 100 Hz, publishes result via
 *          a shared LA_Result_t (mutex protected) for the state machine.
 *
 * Task stack: 512 words (2048 bytes)
 * Priority:   osPriorityAboveNormal (6)
 * No UART to co-processor needed — all on STM32.
 */

#include "sensor_task.h"
#include "line_array_9ch.h"
#include "cmsis_os.h"
#include "main.h"

/* Shared result — state machine reads this */
static LA_Result_t         _shared_result;
static osMutexId_t         _result_mutex;

/* Published via getter only */
LA_Result_t SENSOR_GetResult(void) {
    LA_Result_t copy;
    osMutexAcquire(_result_mutex, osWaitForever);
    copy = _shared_result;
    osMutexRelease(_result_mutex);
    return copy;
}

void SensorTask(void *arg) {
    extern ADC_HandleTypeDef hadc1;
    extern UART_HandleTypeDef huart2;  /* Debug UART */

    _result_mutex = osMutexNew(NULL);

    LA_Init(&hadc1);

    LA_Result_t result;
    uint32_t    debug_tick = 0;

    for (;;) {
        uint32_t t_start = osKernelGetTickCount();

        /* ── Update sensors ──────────────────────────────────────────── */
        LA_Update(&result);

        /* ── Publish result ─────────────────────────────────────────── */
        osMutexAcquire(_result_mutex, osWaitForever);
        _shared_result = result;
        osMutexRelease(_result_mutex);

        /* ── Debug print at 5 Hz ────────────────────────────────────── */
        if (++debug_tick >= 20) {
            LA_PrintDebug(&result, &huart2);
            debug_tick = 0;
        }

        /* ── Maintain 100 Hz ────────────────────────────────────────── */
        osDelayUntil(t_start + 10);  /* 10 ms period */
    }
}
```

### File: `main.c` Integration — Calibration Entry Point

```c
/**
 * Add to app_main() or a dedicated calibration task in main.c.
 * Pressing the user button (B1, PC13) at boot triggers calibration mode.
 */

static bool check_calibration_button(void) {
    /* PC13 = USER button on Nucleo-F401RE, active LOW */
    return (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET);
}

static void run_calibration_routine(void) {
    extern ADC_HandleTypeDef  hadc1;
    extern UART_HandleTypeDef huart2;

    LA_Init(&hadc1);

    char msg[64];

    /* Step 1: White */
    snprintf(msg, sizeof(msg), "Place ALL sensors over WHITE. Waiting 3s...\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
    HAL_Delay(3000);
    snprintf(msg, sizeof(msg), "Sampling WHITE...\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
    LA_Calibrate(true);

    /* Step 2: Black */
    snprintf(msg, sizeof(msg), "Now place over BLACK. Waiting 3s...\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
    HAL_Delay(3000);
    snprintf(msg, sizeof(msg), "Sampling BLACK...\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
    LA_Calibrate(false);

    /* Print summary */
    LA_Calibration_t cal;
    LA_GetCalibration(&cal);
    for (int i = 0; i < LA_NUM_SENSORS; i++) {
        snprintf(msg, sizeof(msg), "S%d: white=%4d  black=%4d  range=%4d\r\n",
                 i+1, cal.min_val[i], cal.max_val[i],
                 cal.max_val[i] - cal.min_val[i]);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
    }

    /* Blink LED2 to indicate done */
    while (1) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);  /* PA5 = LD2 on Nucleo */
        HAL_Delay(200);
    }
}

/* In main() after HAL/Clock/Peripheral init: */
void app_start(void) {
    if (check_calibration_button()) {
        run_calibration_routine();
        /* Never returns */
    }
    /* Start FreeRTOS tasks normally */
    osKernelStart();
}
```

---

## 12. Integration with State Machine

### Current vs. Target `SM_SensorData_t`

> [!WARNING]
> The **current** [`state_machine.h`](file:///c:/Users/ADMIN/Desktop/FusionForce-Robotics/firmware/Navigation/state_machine.h)
> uses the OLD 8-channel struct and still includes `#include "line_array.h"`. The migration
> requires updating both the include and the struct. Below is the target state after migration.

**Current struct (DO NOT break this — migrate carefully):**
```c
/* Current SM_SensorData_t in state_machine.h */
typedef struct {
    uint8_t   line_bits;        // 8-bit GPIO bitmask (old driver)
    float     line_centroid;    // error range −3.5 to +3.5 (old, 8-sensor)
    bool      intersection;
    uint16_t  tof_front_mm;
    uint16_t  tof_left_mm;
    uint16_t  tof_right_mm;
    ColorID_t last_color;
    float     pitch_deg;
    float     roll_deg;
    uint16_t  batt_mv;
} SM_SensorData_t;
```

**Target struct after 9-ch migration:**
```c
/* In state_machine.h — update after migration to line_array_9ch: */
#include "line_array_9ch.h"   // Replace: #include "line_array.h"

typedef struct {
    /* Line array (9-channel analog — replaces 8-channel digital) */
    float    line_error;        ///< Centroid error −4.0 to +4.0 (NAN if lost)
    float    line_error_mm;     ///< Error in mm (−40 to +40)
    uint8_t  line_active_cnt;   ///< Active sensor count (0–9)
    bool     intersection;      ///< Junction/intersection confirmed
    bool     line_lost;         ///< True when no sensor sees line
    uint32_t line_lost_ms;      ///< Duration line has been lost (ms)
    /* ToF / Colour / IMU fields below unchanged */
    uint16_t  tof_front_mm;
    uint16_t  tof_left_mm;
    uint16_t  tof_right_mm;
    ColorID_t last_color;
    float     pitch_deg;
    float     roll_deg;
    uint16_t  batt_mv;
} SM_SensorData_t;
```

### State Machine Sensor Update

> [!NOTE]
> The **current** state machine runs at **50 Hz (20 ms period)** via TIM2 interrupt,
> not 100 Hz. `LA_Update()` should be called at 100 Hz from a FreeRTOS task,
> with the result published to a shared struct consumed at 50 Hz by the SM.
> The SM update function signature is `StateMachine_Update(const SM_SensorData_t *s)` — not `StateMachine_Tick`.

```c
/* In 50 Hz main loop (20 ms TIM2 ISR drives update_flag): */
void StateMachine_Tick_50Hz(void) {
    /* Get latest line array result (produced at 100 Hz by sensor task) */
    LA_Result_t la = SENSOR_GetResult();   /* Thread-safe mutex getter */

    /* Map to SM_SensorData_t */
    SM_SensorData_t s;
    s.line_error      = la.error;                 /* −4.0 to +4.0, NAN if lost */
    s.line_error_mm   = la.error_mm;              /* mm from centre */
    s.line_active_cnt = (uint8_t)la.active_count; /* 0–9 */
    s.intersection    = la.is_junction;
    s.line_lost       = la.is_line_lost;
    s.line_lost_ms    = la.lost_duration_ms;
    /* Populate remaining fields (ToF, colour, IMU) elsewhere */

    StateMachine_Update(&s);   /* 50 Hz state machine step */
}
```

### PD Controller in Following State

```c
/* Initialise once at startup */
LF_Controller_t lf_ctrl;
LF_Init(&lf_ctrl, LF_KP_DEFAULT, LF_KD_DEFAULT, LF_WZ_MAX_DEFAULT, LF_DT_DEFAULT);

/* Call every 100 Hz from STATE_LINE_FOLLOW */
float omega = LF_ComputeOmega(&lf_ctrl, g_sm_data.line_error);

/* Map omega to robot gait:
 *   positive omega → steer right (positive yaw rate)
 *   negative omega → steer left  (negative yaw rate) */
Gait_SetTwist(FORWARD_SPEED_MMS, 0.0f, omega);
```

---

## 13. Testing Procedure

### Phase 1: Hardware Verification

| Test | Method | Pass Criteria |
|------|--------|--------------|
| T1.1 LED current | Multimeter across R1 | 8–12 mA per sensor |
| T1.2 ADC over white | STM32 debug UART, sensor over white paper | Raw ADC: 150–600 |
| T1.3 ADC over black | STM32 debug UART, sensor over black tape | Raw ADC: 3000–4000 |
| T1.4 ADC range | Difference T1.2 vs T1.3 | >2000 ADC counts |
| T1.5 Decoupling | Oscilloscope on ADC pin | No noise spikes >50 mV |
| T1.6 All 9 sensors | Move robot over line | All 9 respond independently |
| T1.7 DMA running | HAL_ADC_GetState() | State = BUSY (DMA active) |

### Phase 2: Software Verification

| Test | Method | Pass Criteria |
|------|--------|--------------|
| T2.1 EMA filter | Move sensor abruptly; watch debug UART | Smooth, no spikes in output |
| T2.2 Calibration | Run cal routine, read Flash back | min/max stored per sensor |
| T2.3 Centroid zero | Centre S5 exactly on line | error ≈ 0.0 ± 0.1 |
| T2.4 Centroid range | Slide line from S1 to S9 | error spans −4.0 to +4.0 |
| T2.5 Junction | Place all 9 over white | is_junction = true after 30 ms |
| T2.6 Line lost | Lift robot off surface | is_line_lost = true after 10 ms |
| T2.7 Calibration Flash | Power cycle after calibrate | Cal reloads correctly from Flash |
| T2.8 Thread safety | Two tasks reading simultaneously | No corrupted results |

### Phase 3: Dynamic Testing

| Test | Method | Pass Criteria |
|------|--------|--------------|
| T3.1 Straight line | 1 m straight line | Robot follows within ±5 mm |
| T3.2 Curved line | Arena curve section | No oscillation, smooth |
| T3.3 90° turn | T-junction in arena | Detects junction, turns correctly |
| T3.4 Junction timing | Slow crossing | Junction held true for >200 ms |
| T3.5 Gap recovery | Lift front momentarily | Recovers within 1.5 seconds |

### Phase 4: Integration Testing

| Test | Method | Pass Criteria |
|------|--------|--------------|
| T4.1 SM integration | Full sensor → state machine | SM correctly reads error values |
| T4.2 PD tuning | Tune Kp, Kd on straight | Settles in <2 oscillations |
| T4.3 Full subtask 1 | Grid navigation | Follows all grid lines |
| T4.4 Full subtask 4 | Colour sort junction | Detects correct branch |

---

## 14. Tuning Guide

### PD Controller Tuning (Step-by-Step)

> [!NOTE]
> The current 8-ch driver uses `Kp = 0.80`, `Kd = 0.05`, `Wz_max = 0.80` at 50 Hz
> (see [`line_follower.h`](file:///c:/Users/ADMIN/Desktop/FusionForce-Robotics/firmware/Navigation/line_follower.h)).
> With 9-ch analog and 100 Hz rate, these need retuning — start from the defaults below.

**Step 1: Set Kd = 0, increase Kp**
- Start: `Kp = 0.2`, `Kd = 0.0`
- Increase Kp until robot oscillates on a straight line
- Record `Kp_oscillate`
- Set `Kp = 0.5 × Kp_oscillate`

**Step 2: Add Kd**
- Keep Kp from Step 1
- Increase Kd until oscillations are damped
- Typical final values: `Kp = 0.5–0.8`, `Kd = 0.04–0.10`

**Step 3: Test on curves**
- If robot cuts corners → increase Kp
- If robot oscillates on straight → increase Kd
- If robot responds too slowly → increase Kp

### Common Problems & Fixes

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Constant oscillation | Kp too high | Reduce Kp by 20% |
| Slow to correct | Kp too low | Increase Kp |
| Overshoot on curves | Kd too low | Increase Kd |
| Stuttery/jerky motion | EMA alpha too high | Lower alpha to 0.5 |
| Wrong centroid direction | Sensor order reversed in CubeMX | Swap ADC scan rank of S1/S9 |
| Junction not detected | Threshold too high | Lower `LA_JUNCTION_MIN_SENSORS` to 5 |
| False junctions | Threshold too low | Raise `LA_JUNCTION_MIN_SENSORS` to 7 |
| Sensors don't differ | Wrong mounting height | Lower to 10–15 mm |
| One sensor always wrong | Bad solder joint / R2 open | Reflow joint; check R2 continuity |
| DMA buffer stale | DMA not started | Confirm `HAL_ADC_Start_DMA()` called in `LA_Init()` |
| Flash cal lost on power cycle | Flash write failed | Check `HAL_FLASH_Unlock()` returns HAL_OK; verify sector address |

---

## 15. Wiring Reference

### Complete Pin Table

> [!WARNING]
> **Current pin state**: PA0–PA7 are `GPIO_Input` (pull-down) for the 8-ch digital driver.
> PB0 is currently unassigned. **Reconfigure all to ADC1** in STM32CubeMX before using this driver.
> The PINOUT_AND_CONNECTIONS.md authoritative document must be updated to reflect this change.

| Sensor | Signal | STM32 GPIO | ADC Unit | ADC Channel | Pull-up R2 |
|--------|--------|-----------|---------|-------------|-----------|
| S1 (Left) | Analog OUT | PA0 | ADC1 | IN0 | 10 kΩ to 3.3 V |
| S2 | Analog OUT | PA1 | ADC1 | IN1 | 10 kΩ to 3.3 V |
| S3 | Analog OUT | PA2 | ADC1 | IN2 | 10 kΩ to 3.3 V |
| S4 | Analog OUT | PA3 | ADC1 | IN3 | 10 kΩ to 3.3 V |
| S5 (Centre) | Analog OUT | PA4 | ADC1 | IN4 | 10 kΩ to 3.3 V |
| S6 | Analog OUT | PA5 | ADC1 | IN5 | 10 kΩ to 3.3 V |
| S7 | Analog OUT | PA6 | ADC1 | IN6 | 10 kΩ to 3.3 V |
| S8 | Analog OUT | PA7 | ADC1 | IN7 | 10 kΩ to 3.3 V |
| S9 (Right) | Analog OUT | PB0 | ADC1 | IN8 | 10 kΩ to 3.3 V |
| All sensors | VCC | — | — | — | 3.3 V |
| All sensors | GND | — | — | — | GND |
| Debug TX | USART1 TX | **PA9** | — | — | — |

> [!NOTE]
> Debug UART is **USART1 on PA9/PA10** (not USART2/PA2) per `STM32_ARCHITECTURE.md`.
> Disconnect at competition. PA2 conflict does NOT apply — PA2 is free for S3 ADC use.

> [!NOTE]
> PA2 is used by USART2_TX on Nucleo boards. If using PA2 as UART, remap
> S3 to another ADC1-capable pin (e.g., PC2 = ADC1_IN12) and update the
> CubeMX scan rank accordingly.

### Power Budget (Sensors Only)

| Item | Current | Notes |
|------|---------|-------|
| 9 × IR LEDs (9 mA each) | 81 mA | R1 = 220 Ω |
| 9 × Phototransistors | <9 mA | Collector current |
| STM32F401 (active, full speed) | ~25 mA | No Wi-Fi overhead |
| **Total sensor subsystem** | **~115 mA @ 3.3 V** | **0.38 W** |

> [!TIP]
> Power the sensor array from the STM32's onboard 3.3 V LDO. The 115 mA draw is
> well within a standard 500 mA LDO. Add a 10 µF electrolytic at the 3.3 V rail
> entry to the sensor PCB to prevent voltage droop when all 9 LEDs switch
> simultaneously during a scan.

### STM32CubeMX Scan Rank Order (Critical!)

The DMA fills `_dma_buf` in **scan rank order**. Assign ranks as follows to match
the `sensor_idx` ordering in the driver:

| Rank | GPIO | ADC Channel | Sensor |
|------|------|------------|--------|
| 1 | PA0 | IN0 | S1 (Left) |
| 2 | PA1 | IN1 | S2 |
| 3 | PA2 | IN2 | S3 |
| 4 | PA3 | IN3 | S4 |
| 5 | PA4 | IN4 | S5 (Centre) |
| 6 | PA5 | IN5 | S6 |
| 7 | PA6 | IN6 | S7 |
| 8 | PA7 | IN7 | S8 |
| 9 | PB0 | IN8 | S9 (Right) |

---

## Quick Reference Card

```
┌──────────────────────────────────────────────────────────────────────┐
│         9-SENSOR ANALOG ARRAY — STM32 QUICK REFERENCE                │
├──────────────────────────────────────────────────────────────────────┤
│ Sensor: TCRT5000 analog                                              │
│ Per sensor: R1=220Ω (LED) + R2=10kΩ (pull-up) + C1=100nF (decouple) │
│ Spacing: 10mm | Array width: 80mm | Mount height: 10–15mm            │
├──────────────────────────────────────────────────────────────────────┤
│ STM32 ADC: ADC1, 12-bit, IN0–IN8 (PA0–PA7, PB0), DMA Circular       │
│ Oversampling: 16x (DMA, zero CPU cost) | Update rate: 100 Hz        │
│ Filter: EMA α=0.7 + spike reject (>500 counts)                       │
│ Calibration: 2-point (white + black) per sensor, stored in Flash     │
├──────────────────────────────────────────────────────────────────────┤
│ Centroid error range: -4.0 (line far left) to +4.0 (line far right) │
│ Junction: ≥6 sensors active for ≥3 consecutive samples (30ms)        │
│ Line lost: all sensors <15% contribution → lost_duration_ms counted  │
├──────────────────────────────────────────────────────────────────────┤
│ PD gains: Kp=0.6, Kd=0.08, Wz_max=1.2 rad/s (starting point)        │
│ Current 8-ch gains: Kp=0.80, Kd=0.05, Wz_max=0.80 @ 50Hz            │
│ All onboard STM32F411CEU6 — no co-processor needed                   │
└──────────────────────────────────────────────────────────────────────┘
```

---

---

## Migration Checklist (8-ch Digital → 9-ch Analog)

- [ ] Reconfigure PA0–PA7 from `GPIO_Input` to `ADC1_IN0–IN7` in STM32CubeMX
- [ ] Add PB0 as `ADC1_IN8` for S9 in STM32CubeMX
- [ ] Enable ADC1 Scan+Continuous+DMA Circular mode, 9 ranks, 480-cycle sample time
- [ ] Configure DMA1 Stream0 Ch0, Peripheral→Memory, Circular, Word
- [ ] Replace `#include "line_array.h"` with `#include "line_array_9ch.h"` in `state_machine.h`
- [ ] Update `SM_SensorData_t` struct (remove `line_bits`, add `line_error`, `line_active_cnt` etc.)
- [ ] Resolve Flash sector conflict: move line cal to Sector 6 (0x08060000) and ball colour stays at Sector 7 (0x0807F800)
- [ ] Update `PINOUT_AND_CONNECTIONS.md` to reflect ADC pins
- [ ] Retune PD gains from Kp=0.80/Kd=0.05 (50Hz, 8-ch) to Kp=0.60/Kd=0.08 (100Hz, 9-ch)
- [ ] Run full 4-phase testing procedure

---

*Report generated: September 2026 | FusionForce Robotics | RUNNER-4 Project*
*MCU: STM32F411CEU6 | Migration report: 8-ch digital → 9-ch analog IR sensor array*
