# RUNNER-4 | Color Sensor System
## Full Technical Report — STM32 HAL Implementation
### FusionForce Robotics — EN2533 BREACH PROTOCOL

> [!IMPORTANT]
> **MCU: STM32F411CEU6 (Black Pill)** — 100 MHz Cortex-M4F, 512 KB Flash, 128 KB RAM.
> The TCS34725 is mounted on the **gripper arm tip** and operates in **dual mode**:
> - **MODE A** (arm 0°): Points at ball on pedestal → ball colour identification (Task 1)
> - **MODE B** (arm −70°): Points at floor zone → junction colour reading (Task 4)
>
> ### Real Classification Algorithm (from [`tcs34725.c`](file:///c:/Users/ADMIN/Desktop/FusionForce-Robotics/firmware/Drivers/TCS34725/tcs34725.c))
>
> The actual project uses a **ratio-dominance classifier** (not HSV lookup):
>
> ```c
> ColorID_t TCS34725_ClassifyColor(const TCS34725_RGBC_t *raw) {
>     if (raw->c == 0) return COLOR_UNKNOWN;
> 
>     float r_n = (float)raw->r / raw->c;  /* Normalised red   (ratio to clear) */
>     float g_n = (float)raw->g / raw->c;  /* Normalised green */
>     float b_n = (float)raw->b / raw->c;  /* Normalised blue  */
> 
>     /* RED: r_n must be high AND dominate both green and blue */
>     if (r_n > 0.40f && r_n > g_n * 1.4f && r_n > b_n * 1.4f)
>         return COLOR_RED;
> 
>     /* GREEN: g_n must be high AND dominate */
>     if (g_n > 0.35f && g_n > r_n * 1.2f && g_n > b_n * 1.2f)
>         return COLOR_GREEN;
> 
>     /* BLUE: b_n must be high AND dominate */
>     if (b_n > 0.30f && b_n > r_n * 1.2f && b_n > g_n * 1.2f)
>         return COLOR_BLUE;
> 
>     return COLOR_UNKNOWN;  /* Retry on next cycle */
>     /* NOTE: Thresholds MUST be tuned during 2-min pre-competition calibration
>        window using actual arena Red/Green/Blue objects under arena lighting. */
> }
> ```
>
> **Why ratio-dominance (not HSV)?**
> - Simpler to tune during 2-min calibration window
> - Consistent with task requirements (only 3 colours)
> - No trigonometric math needed (lighter on STM32)
> - Empirically tuned to actual TCRT5000 output under 3.3V LED
>
> **Typical calibrated values under arena lighting:**
>
> | Colour | r_n | g_n | b_n | Dominant condition |
> |--------|-----|-----|-----|--------------------|
> | Red | ~0.55 | ~0.20 | ~0.15 | r_n > 0.40, r_n > g_n×1.4 ✅ |
> | Green | ~0.20 | ~0.50 | ~0.20 | g_n > 0.35, g_n > r_n×1.2 ✅ |
> | Blue | ~0.15 | ~0.20 | ~0.45 | b_n > 0.30, b_n > r_n×1.2 ✅ |
>
> The real driver is [`tcs34725.c`](file:///c:/Users/ADMIN/Desktop/FusionForce-Robotics/firmware/Drivers/TCS34725/tcs34725.c) /
> [`tcs34725.h`](file:///c:/Users/ADMIN/Desktop/FusionForce-Robotics/firmware/Drivers/TCS34725/tcs34725.h).
> `ColorID_t` is defined there with only **3 colours** (RED, GREEN, BLUE) + UNKNOWN.

---

## Table of Contents
1. [Colour Sensing Overview & Physics](#1-colour-sensing-overview--physics)
2. [Sensor Type Taxonomy](#2-sensor-type-taxonomy)
3. [Full Sensor Comparison](#3-full-sensor-comparison)
4. [Recommended Sensor: TCS34725](#4-recommended-sensor-tcs34725)
5. [Hardware Circuit & Wiring](#5-hardware-circuit--wiring)
6. [STM32 I2C Driver Code](#6-stm32-i2c-driver-code)
7. [Colour Space Conversion](#7-colour-space-conversion)
8. [White-Balance & Illumination Calibration](#8-white-balance--illumination-calibration)
9. [Colour Classification Algorithms](#9-colour-classification-algorithms)
10. [Signal Filters](#10-signal-filters)
11. [Junction Colour Detection State Machine](#11-junction-colour-detection-state-machine)
12. [Integration with Line Array & State Machine](#12-integration-with-line-array--state-machine)
13. [Testing Procedure](#13-testing-procedure)
14. [Tuning Guide](#14-tuning-guide)
15. [Quick Reference Card](#15-quick-reference-card)

---

## 1. Colour Sensing Overview & Physics

### How Colour Sensors Work

A colour sensor measures the intensity of light reflected from a surface across multiple wavelength bands. The fundamental principle:

```
[White LED / Illuminant]
         ↓  Illuminates surface
    [Arena Floor Marker]
         ↓  Reflects wavelength-dependent fraction of light
    [Photodetector Array]
    (R, G, B, Clear channels)
         ↓
    [ADC → MCU → Colour Classification]
```

### Reflectance Physics
Every coloured surface has a **spectral reflectance curve** — the fraction of incident light reflected at each wavelength:

| Colour | 450 nm (Blue) | 550 nm (Green) | 650 nm (Red) |
|--------|--------------|----------------|--------------|
| Red    | ~5%          | ~10%           | ~85%         |
| Green  | ~10%         | ~80%           | ~10%         |
| Blue   | ~80%         | ~10%           | ~5%          |
| White  | ~85%         | ~85%           | ~85%         |
| Black  | ~5%          | ~5%            | ~5%          |
| Yellow | ~10%         | ~75%           | ~75%         |

By comparing relative intensities in R, G, B bands, we can identify the surface colour.

### Why Ambient Light is the Enemy
Arena lighting varies:
- Fluorescent → peaks at 436, 546, 611 nm
- LED strips → varies by colour temperature
- Natural → shifts with time of day

**Solution**: Use an **integrated LED illuminant** on the sensor module + **normalisation** to ambient-independent colour coordinates (HSV saturation / chromaticity). See Section 8.

---

## 2. Sensor Type Taxonomy

### Type A — Filtered Photodiode Arrays (RGB)
Photodiodes with on-chip optical bandpass filters in R, G, B (and optionally Clear/IR) channels. Most common for robotics.

**Examples**: TCS34725, TCS3200, VEML6040, OPT3001 (lux only)

```
[White LED]
     ↓
[Surface]
     ↓ Reflected light
┌─────────────────────────┐
│ R filter │ G filter │ B filter │ Clear │
│   PD     │   PD     │   PD    │   PD  │
│   ADC    │   ADC    │   ADC   │   ADC │
└─────────────────────────┘
     → I²C/digital output
```

**Pros**: Cheap (~$1–5), small, I2C interface, integrated LED, proven in robotics  
**Cons**: Only 3–4 bands, cannot distinguish metamerics (colours that look same under one light)

---

### Type B — LED Switching / Frequency-Based
Uses a matrix of RGB photodiodes whose output **frequency** is proportional to light intensity. No ADC needed — MCU counts pulses.

**Examples**: TCS3200, TCS230

```
[Selectable R/G/B/Clear Filter]
→ Photodiode array
→ Current-to-frequency converter
→ Digital square wave output
→ STM32 timer input capture
```

**Pros**: No ADC needed, simple digital interface, dirt cheap (~$0.50)  
**Cons**: Frequency output is slow (needs 50–100 ms per channel), sequential measurement (R then G then B), sensitive to ambient light flicker

---

### Type C — Spectral Sensors (Multi-Band)
Measures 6–18 discrete wavelength bands. True spectral fingerprinting.

**Examples**: AS7262 (6-channel visible), AS7265x (18-channel), APDS-9960 (RGBC + gesture + proximity)

**Pros**: Metameric resistance, very accurate colour matching, can detect fluorescent markers  
**Cons**: Expensive ($5–30), larger package, slower (I2C burst), overkill for most competition tasks

---

### Type D — Camera-Based
A full imaging sensor (OV7670, ESP32-CAM, Raspberry Pi Camera) captures a 2D image for colour region analysis.

**Pros**: Can detect colour position, shape, size simultaneously  
**Cons**: High complexity (OpenCV/DNN), high latency (~30–100 ms), requires significant CPU (usually off-STM32), power-hungry

---

### Type E — Phototransistor + Discrete Filters
Individual coloured optical filters (Lee filters / Roscolux) placed over a single phototransistor.

**Pros**: Ultra-cheap, customisable band selection  
**Cons**: Fragile, assembly-intensive, poor wavelength accuracy, no integrated LED

---

## 3. Full Sensor Comparison

| Feature | TCS34725 ⭐ | TCS3200 | APDS-9960 | AS7262 | OV7670 (Camera) |
|---------|------------|---------|-----------|--------|-----------------|
| **Sensor type** | RGB+Clear photodiode | R/G/B/Clear w/ freq out | RGBC + prox + gesture | 6-ch spectral | Full image sensor |
| **Interface** | I²C (400 kHz) | Digital (freq out) | I²C (400 kHz) | I²C (400 kHz) | SCCB / DVP parallel |
| **Resolution** | 16-bit per channel | ~16-bit equiv (pulse count) | 16-bit per channel | 16-bit per channel | 640×480 pixels |
| **Channels** | 4 (RGBC) | 4 (RGBC) | 4 (RGBC) | 6 (450–680 nm) | 3 (RGB Bayer) |
| **Integrated LED** | ✅ White LED | ❌ (needs external) | ✅ IR LED only | ✅ White LED | ❌ |
| **Update rate** | ~50 Hz (all channels) | ~10 Hz (sequential) | ~50 Hz | ~8 Hz | 30 fps |
| **Detection range** | 3–10 mm | 10–30 mm | 5–10 mm | 5–15 mm | Any |
| **STM32 overhead** | Low (I2C DMA) | Medium (timer IC) | Low (I2C DMA) | Low (I2C DMA) | Very High |
| **Package** | 3.9×2.4 mm DFNS | DIP module | 3.94×2.36 mm LCC | 3.94×2.36 mm LCC | 1/6" CMOS |
| **Cost** | ~$1.50 | ~$0.50 | ~$2.50 | ~$8.00 | ~$3–15 |
| **Colour accuracy** | ★★★★☆ | ★★★☆☆ | ★★★★☆ | ★★★★★ | ★★★★☆ |
| **Metameric resistance** | Low | Low | Low | High | Medium |
| **IR rejection** | ✅ On-chip IR cut | ❌ None | Partial | ✅ | Partial |
| **Ambient rejection** | Good (clear channel) | Poor | Good | Good | Good (software) |
| **Library support** | Adafruit + HAL | Basic | Adafruit | SparkFun | OpenCV |
| **Robotics suitability** | **Excellent** | Good | Good | Excellent (overkill) | Complex |

### Decision Matrix for RUNNER-4

| Criterion | Weight | TCS34725 | TCS3200 | APDS-9960 | AS7262 |
|-----------|--------|----------|---------|-----------|--------|
| Update rate ≥ 20 Hz | 30% | ✅ 50 Hz | ⚠️ 10 Hz | ✅ 50 Hz | ❌ 8 Hz |
| Integrated white LED | 25% | ✅ | ❌ | ❌ (IR only) | ✅ |
| STM32 I2C simplicity | 20% | ✅ | ⚠️ timer | ✅ | ✅ |
| IR rejection | 15% | ✅ | ❌ | Partial | ✅ |
| Cost | 10% | ✅ $1.50 | ✅ $0.50 | ⚠️ $2.50 | ❌ $8 |
| **Total Score** | | **95/100** | **60/100** | **75/100** | **70/100** |

> [!IMPORTANT]
> **TCS34725** is the recommended sensor. It provides the optimal balance of update
> rate, integrated white LED illumination, 16-bit resolution, I2C interface, IR cut
> filter, and low cost. The APDS-9960 is a viable second choice if proximity sensing
> is also needed.

---

## 4. Recommended Sensor: TCS34725

### Specifications

| Parameter | Value |
|-----------|-------|
| Part number | TCS34725FN (bare IC) or AMS TCS34725 breakout |
| Supply voltage | 2.7–3.6 V (3.3 V ✅) |
| Interface | I²C, up to 400 kHz |
| I²C address | 0x29 (fixed) |
| Channels | Red, Green, Blue, Clear (RGBC) |
| ADC resolution | 16-bit per channel (0–65535) |
| Integration time | 2.4 ms to 614 ms (programmable) |
| Gain | 1×, 4×, 16×, 60× (programmable) |
| Integrated LED | White LED (on/off via GPIO) |
| IR cut filter | On-chip — blocks >700 nm |
| Detection distance | 3–10 mm optimal |
| Package | 3.9×2.4×1.0 mm DFNS-6 |
| Current (LED on) | ~10 mA LED + 0.6 mA IC |

### Recommended Operating Mode for RUNNER-4

| Register | Value | Rationale |
|----------|-------|-----------|
| Integration time | **50 ms (ATIME = 0xEB)** | 20 Hz raw; non-blocking poll waits 3×20ms cycles |
| Gain | 4× (CONTROL = 0x01) | For white-lit arena under arm LED |
| LED | ON via PC0 GPIO when reading | Consistent illumination from arm tip |
| Interrupt | Disabled (polling mode) | Non-blocking 50ms cycle counter |
| Wait | Disabled | Maximum update rate |

### Why Integration Time = 50 ms?
```
Integration_time = (256 − ATIME) × 2.4 ms
ATIME = 0xEB → (256 − 235) × 2.4 = 21 × 2.4 = 50.4 ms
Non-blocking poll: check AVALID after 3 × 20ms = 60ms cycles (safe margin)

At 80 mm/s, robot crosses 50mm floor marker in 625ms → ~10 readings @ 1/60ms
For ball reading: robot is STOPPED → single 50ms read is sufficient
```

> [!NOTE]
> The actual driver uses `TCS34725_PollNonBlocking()` which starts integration,
> then waits 3 main-loop cycles (3 × 20ms = 60ms) before reading — this is tied
> to the **50 Hz (20ms) main loop**, not a FreeRTOS sensor task.

---

## 5. Hardware Circuit & Wiring

### Single TCS34725 Breakout Schematic

```
3.3V ──┬────────────────────────────────────────────┐
       │                                            │
      100nF (decoupling, as close to VCC as possible)
       │                                            │
      GND                                       VCC (TCS34725)
                                                SDA ──── STM32 PB7 (I2C1_SDA)
                                                SCL ──── STM32 PB6 (I2C1_SCL)
                                                INT ──── (optional, leave float if not used)
                                                LED ──── STM32 GPIO (e.g. PC0) — controls white LED

Pull-up resistors (if not on breakout):
SDA ──┤ 4.7 kΩ ├── 3.3V
SCL ──┤ 4.7 kΩ ├── 3.3V
```

### Mounting Specification

```
Gripper Arm Tip — Dual-Mode
┌──────────────────────────────────────────────────────┐
│ ARM_MODE_A (0°, horizontal):                         │
│   [TCS34725] ──→ Ball at pedestal height (~5 cm)     │
│   Distance: 1–2 cm from ball surface                 │
│   Light shroud: 3D-printed PETG to block arena light │
│                                                      │
│ ARM_MODE_B (−70°, pointing at floor):                │
│   [TCS34725] ──→ Floor zone marker                   │
│   Distance: 1–3 cm above floor surface               │
│   Servo: PCA9685 PWM12 (Arm Pitch)                   │
└──────────────────────────────────────────────────────┘

Robot Bottom View (for reference — sensor is at arm tip, NOT underside):
┌──────────────────────────────────────────────────────┐
│  [VL53L0X Front]  [VL53L0X Left]  [VL53L0X Right]   │
│  [8/9-ch IR Line Array — PA0 to PA7(+PB0)]           │
│                   ← robot body →                     │
│  Arm extends from FRONT of body with TCS34725 at tip │
└──────────────────────────────────────────────────────┘
```

> [!NOTE]
> The colour sensor is NOT mounted on the robot underside — it is at the **gripper arm tip**.
> This allows dual-mode: look forward at a ball (MODE A) or tilt down to floor (MODE B).
> A 3D-printed light shroud blocks ambient arena light at the sensor tip.

### Distance vs Performance

| Height (mm) | Spot size (mm) | SNR | Notes |
|-------------|----------------|-----|-------|
| 3 | ~4 | Excellent | Risk of contact |
| 5 | ~6 | Excellent | Recommended minimum |
| **8** | **~10** | **Very Good** | **Recommended** |
| 15 | ~18 | Good | Includes some background |
| 25 | ~30 | Fair | Ambient dominates |

### Complete Pin Table

| TCS34725 Pin | Signal | STM32 GPIO | Notes |
|-------------|--------|-----------|-------|
| VCC | Power | 3.3 V rail | 100 nF decoupling |
| GND | Ground | GND | |
| SDA | I²C Data | **PB7 (I2C1_SDA)** | 4.7 kΩ pull-up to 3.3 V — shared with PCA9685 and MPU6050 |
| SCL | I²C Clock | **PB6 (I2C1_SCL)** | 4.7 kΩ pull-up to 3.3 V — shared with PCA9685 and MPU6050 |
| LED | White LED ctrl | **PC0 (GPIO Output)** | HIGH = LED on |
| INT | Interrupt | NC (not connected) | Polling mode used — INT unused |

### Sensor Summary

| Sensor | Qty | Function |
|--------|-----|----------|
| **TCS34725 RGBC** | 1 | Arm tip dual-mode: ball colour (MODE A) + floor zone (MODE B) |

### Dual-Mode Arm Operation

```
MODE A — Ball Colour Reading (Task 1):
  Arm pitch servo → 0° (horizontal forward)
  Robot stops at junction, arm lowers to pedestal height (~5 cm)
  TCS34725 tip is 1–2 cm from ball surface
  LED ON → 50 ms integration → read RGBC → classify → store to Flash

MODE B — Floor Zone Reading (Task 4):
  Arm pitch servo → −70° (pointing at floor)
  Robot pauses at 3-way junction (≥6 sensors, ≥3 cycles)
  TCS34725 tip is 1–3 cm above floor marker
  LED ON → 50 ms integration → read RGBC → classify
  Compare with stored ball_color → select branch
  Arm returns home → robot turns onto matching branch
```

**Pre-calibrated arm angles (in [`state_machine.h`](file:///c:/Users/ADMIN/Desktop/FusionForce-Robotics/firmware/Navigation/state_machine.h)):**
```c
#define ARM_HOME    0   // Travel/rest position
#define ARM_MODE_A  1   // Ball colour reading (0°, horizontal)
#define ARM_MODE_B  2   // Floor zone reading (−70°)
```

### Power Budget (Colour Sensor)

| Item | Current |
|------|---------|
| TCS34725 IC | 0.6 mA |
| White LED (when on) | ~10 mA |
| **Total** | **~11 mA** |

---

## 6. STM32 I2C Driver Code

### File: `color_sensor.h` — Real Project `tcs34725.h`

> [!NOTE]
> The actual driver file is [`tcs34725.h`](file:///c:/Users/ADMIN/Desktop/FusionForce-Robotics/firmware/Drivers/TCS34725/tcs34725.h).
> Key differences from the generic colour sensor header below:
> - `ColorID_t` has only **4 values**: `COLOR_UNKNOWN=0, COLOR_RED=1, COLOR_GREEN=2, COLOR_BLUE=3`
> - Handle type is `TCS34725_Handle_t` (with `hi2c`, `integration_started`, `cycle_counter`)
> - Non-blocking poll via `TCS34725_PollNonBlocking(&hdev, &color_out)` called every 20ms
> - LED control via `TCS34725_SetLED(bool on)` which writes PC0
> - No FreeRTOS task — polling integrated into the 50Hz TIM2 main loop

```c
/* Real ColorID_t from tcs34725.h */
typedef enum {
    COLOR_UNKNOWN = 0,
    COLOR_RED     = 1,
    COLOR_GREEN   = 2,
    COLOR_BLUE    = 3
} ColorID_t;

/* Real driver handle from tcs34725.h */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    bool     integration_started;
    uint8_t  cycle_counter;     /* Incremented each 20ms cycle */
    TCS34725_RGBC_t last_raw;
    ColorID_t        last_color;
} TCS34725_Handle_t;

/* Real API */
HAL_StatusTypeDef TCS34725_Init(TCS34725_Handle_t *hdev, I2C_HandleTypeDef *hi2c,
                                 uint8_t atime, uint8_t gain);
bool              TCS34725_PollNonBlocking(TCS34725_Handle_t *hdev, ColorID_t *out);
ColorID_t         TCS34725_ClassifyColor(const TCS34725_RGBC_t *raw);
void              TCS34725_SetLED(bool on);
```
/**
 * @file    color_sensor.h
 * @brief   TCS34725 RGB colour sensor driver — STM32 HAL I2C
 *
 * FusionForce Robotics | RUNNER-4 | EN2533 BREACH PROTOCOL
 */
#ifndef COLOR_SENSOR_H
#define COLOR_SENSOR_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* ─── Configuration ───────────────────────────────────────────────────────── */
#define CS_I2C_ADDR         (0x29 << 1)   /* TCS34725 7-bit addr 0x29, shifted for HAL */
#define CS_I2C_TIMEOUT_MS   10
#define CS_LED_PORT         GPIOC
#define CS_LED_PIN          GPIO_PIN_0
#define CS_UPDATE_RATE_HZ   40            /* Target update rate */
#define CS_EMA_ALPHA        0.5f          /* EMA smoothing for RGBC */
#define CS_MEDIAN_N         5             /* Median filter window */
#define CS_VOTE_WINDOW      7             /* Majority vote window (samples) */

/* TCS34725 Register Map (with command bit 0x80) */
#define TCS34725_REG_CMD        0x80
#define TCS34725_REG_ENABLE     (TCS34725_REG_CMD | 0x00)
#define TCS34725_REG_ATIME      (TCS34725_REG_CMD | 0x01)
#define TCS34725_REG_CONTROL    (TCS34725_REG_CMD | 0x0F)
#define TCS34725_REG_ID         (TCS34725_REG_CMD | 0x12)
#define TCS34725_REG_STATUS     (TCS34725_REG_CMD | 0x13)
#define TCS34725_REG_CDATAL     (TCS34725_REG_CMD | 0x14)  /* Clear low */
#define TCS34725_REG_RDATAL     (TCS34725_REG_CMD | 0x16)  /* Red low */
#define TCS34725_REG_GDATAL     (TCS34725_REG_CMD | 0x18)  /* Green low */
#define TCS34725_REG_BDATAL     (TCS34725_REG_CMD | 0x1A)  /* Blue low */

/* ENABLE register bits */
#define TCS34725_ENABLE_AEN     0x02  /* RGBC enable */
#define TCS34725_ENABLE_PON     0x01  /* Power on */

/* Integration time: ATIME = 256 - (integration_ms / 2.4) */
#define TCS34725_ATIME_24MS     0xF6  /* ~24 ms → ~41 Hz */
#define TCS34725_ATIME_50MS     0xEB  /* ~50 ms → ~20 Hz */
#define TCS34725_ATIME_101MS    0xD5  /* ~101 ms → ~10 Hz */

/* Gain settings */
#define TCS34725_GAIN_1X        0x00
#define TCS34725_GAIN_4X        0x01
#define TCS34725_GAIN_16X       0x02
#define TCS34725_GAIN_60X       0x03

/* Colour IDs — extend as needed for your arena */
typedef enum {
    COLOUR_UNKNOWN  = 0,
    COLOUR_RED      = 1,
    COLOUR_GREEN    = 2,
    COLOUR_BLUE     = 3,
    COLOUR_YELLOW   = 4,
    COLOUR_WHITE    = 5,
    COLOUR_BLACK    = 6,
} CS_ColourID_t;

/* Raw RGBC reading */
typedef struct {
    uint16_t r;
    uint16_t g;
    uint16_t b;
    uint16_t c;       /* Clear (broadband) */
} CS_RawRGBC_t;

/* Normalised colour (0.0–1.0 per channel, independent of brightness) */
typedef struct {
    float r_norm;     /* r / c */
    float g_norm;     /* g / c */
    float b_norm;     /* b / c */
    float lux;        /* Approximate lux (for diagnostics) */
    float h;          /* Hue 0–360° */
    float s;          /* Saturation 0–1 */
    float v;          /* Value 0–1 */
} CS_ProcessedColour_t;

/* Full result struct */
typedef struct {
    CS_RawRGBC_t       raw;
    CS_ProcessedColour_t processed;
    CS_ColourID_t      colour_id;      /* Classified colour */
    CS_ColourID_t      voted_colour;   /* After majority vote filter */
    bool               data_valid;     /* False if sensor not ready */
    uint32_t           timestamp_ms;
} CS_Result_t;

/* Calibration — white-balance reference per colour ID */
typedef struct {
    float white_r_norm;   /* r_norm when sensor over white */
    float white_g_norm;
    float white_b_norm;
    uint32_t magic;
    bool is_valid;
} CS_Calibration_t;

/* Public API */
bool CS_Init(I2C_HandleTypeDef *hi2c);
void CS_Update(CS_Result_t *result);
void CS_CalibrateWhite(void);
void CS_GetCalibration(CS_Calibration_t *cal);
const char *CS_ColourName(CS_ColourID_t id);
void CS_PrintDebug(const CS_Result_t *r, UART_HandleTypeDef *huart);
void CS_LEDEnable(bool on);

#endif /* COLOR_SENSOR_H */
```

### File: `color_sensor.c`

```c
/**
 * @file    color_sensor.c
 * @brief   TCS34725 RGB colour sensor driver — STM32 HAL I2C
 *
 * Signal pipeline:
 *   I2C read (RGBC 16-bit) → EMA filter → Normalise (r/c, g/c, b/c)
 *   → White-balance correct → RGB-to-HSV → Nearest-neighbour classify
 *   → Majority vote (7-sample) → CS_Result_t
 */

#include "color_sensor.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

/* ─── Internal State ───────────────────────────────────────────────────────── */
static I2C_HandleTypeDef *_hi2c = NULL;
static CS_Calibration_t   _cal;
static float _ema_r = 0, _ema_g = 0, _ema_b = 0, _ema_c = 0;
static bool  _ema_init = false;
static bool  _initialised = false;

/* Vote buffer */
static CS_ColourID_t _vote_buf[CS_VOTE_WINDOW];
static int           _vote_idx = 0;
static bool          _vote_full = false;

/* ─── Colour Reference Table (HSV) ────────────────────────────────────────── */
/* Tune these after calibration on your arena surface under competition lighting */
typedef struct {
    CS_ColourID_t id;
    float h_min, h_max;   /* Hue range (degrees) */
    float s_min;          /* Minimum saturation (avoids white/grey confusion) */
    float v_min;          /* Minimum value (avoids black confusion) */
} CS_ColourRef_t;

static const CS_ColourRef_t _colour_table[] = {
    { COLOUR_RED,    340.0f, 360.0f, 0.30f, 0.15f },  /* Red (upper hue wrap) */
    { COLOUR_RED,      0.0f,  15.0f, 0.30f, 0.15f },  /* Red (lower hue wrap) */
    { COLOUR_YELLOW,  35.0f,  75.0f, 0.35f, 0.15f },  /* Yellow */
    { COLOUR_GREEN,   80.0f, 160.0f, 0.30f, 0.10f },  /* Green */
    { COLOUR_BLUE,   185.0f, 270.0f, 0.30f, 0.10f },  /* Blue */
};
#define CS_TABLE_SIZE  (sizeof(_colour_table) / sizeof(_colour_table[0]))

/* ─── I2C Helpers ──────────────────────────────────────────────────────────── */

static HAL_StatusTypeDef prv_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return HAL_I2C_Master_Transmit(_hi2c, CS_I2C_ADDR, buf, 2, CS_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef prv_read_reg(uint8_t reg, uint8_t *data, uint16_t len) {
    HAL_StatusTypeDef s;
    s = HAL_I2C_Master_Transmit(_hi2c, CS_I2C_ADDR, &reg, 1, CS_I2C_TIMEOUT_MS);
    if (s != HAL_OK) return s;
    return HAL_I2C_Master_Receive(_hi2c, CS_I2C_ADDR, data, len, CS_I2C_TIMEOUT_MS);
}

static uint16_t prv_read_word(uint8_t reg_low) {
    uint8_t buf[2];
    /* Use auto-increment command: bit5 of command byte */
    uint8_t cmd = TCS34725_REG_CMD | 0x20 | (reg_low & 0x1F);
    if (prv_read_reg(cmd, buf, 2) != HAL_OK) return 0;
    return (uint16_t)(buf[0] | (buf[1] << 8));
}

/* ─── Math Helpers ─────────────────────────────────────────────────────────── */

static float prv_clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

/**
 * @brief Convert normalised RGB (0–1 each, sum to 1 approximately) to HSV.
 * @param r,g,b  Normalised channel values
 * @param h      Output hue 0–360
 * @param s      Output saturation 0–1
 * @param v      Output value 0–1
 */
static void prv_rgb_to_hsv(float r, float g, float b, float *h, float *s, float *v) {
    float mx = fmaxf(fmaxf(r, g), b);
    float mn = fminf(fminf(r, g), b);
    float delta = mx - mn;

    *v = mx;
    *s = (mx > 1e-5f) ? (delta / mx) : 0.0f;

    if (delta < 1e-5f) {
        *h = 0.0f;
        return;
    }

    if (mx == r) {
        *h = 60.0f * fmodf((g - b) / delta, 6.0f);
    } else if (mx == g) {
        *h = 60.0f * ((b - r) / delta + 2.0f);
    } else {
        *h = 60.0f * ((r - g) / delta + 4.0f);
    }

    if (*h < 0.0f) *h += 360.0f;
}

/* ─── EMA Filter ───────────────────────────────────────────────────────────── */

static void prv_apply_ema(float r, float g, float b, float c,
                           float *fr, float *fg, float *fb, float *fc) {
    if (!_ema_init) {
        _ema_r = r; _ema_g = g; _ema_b = b; _ema_c = c;
        _ema_init = true;
    } else {
        _ema_r = CS_EMA_ALPHA * r + (1.0f - CS_EMA_ALPHA) * _ema_r;
        _ema_g = CS_EMA_ALPHA * g + (1.0f - CS_EMA_ALPHA) * _ema_g;
        _ema_b = CS_EMA_ALPHA * b + (1.0f - CS_EMA_ALPHA) * _ema_b;
        _ema_c = CS_EMA_ALPHA * c + (1.0f - CS_EMA_ALPHA) * _ema_c;
    }
    *fr = _ema_r; *fg = _ema_g; *fb = _ema_b; *fc = _ema_c;
}

/* ─── Colour Classification ────────────────────────────────────────────────── */

/**
 * @brief Classify colour using HSV lookup table.
 *        Falls back to WHITE or BLACK for low saturation / low value.
 */
static CS_ColourID_t prv_classify_hsv(float h, float s, float v) {
    /* Low value → black */
    if (v < 0.08f) return COLOUR_BLACK;
    /* Low saturation → white/grey */
    if (s < 0.15f) return COLOUR_WHITE;

    /* Check each colour band */
    for (size_t i = 0; i < CS_TABLE_SIZE; i++) {
        const CS_ColourRef_t *ref = &_colour_table[i];
        if (s >= ref->s_min && v >= ref->v_min &&
            h >= ref->h_min && h <= ref->h_max) {
            return ref->id;
        }
    }
    return COLOUR_UNKNOWN;
}

/* ─── Majority Vote Filter ─────────────────────────────────────────────────── */

static CS_ColourID_t prv_majority_vote(CS_ColourID_t new_val) {
    _vote_buf[_vote_idx] = new_val;
    _vote_idx = (_vote_idx + 1) % CS_VOTE_WINDOW;
    if (_vote_idx == 0) _vote_full = true;

    int count = _vote_full ? CS_VOTE_WINDOW : _vote_idx;
    int tally[8] = {0};  /* One slot per ColourID */
    for (int i = 0; i < count; i++) {
        if (_vote_buf[i] < 8) tally[_vote_buf[i]]++;
    }

    CS_ColourID_t best = COLOUR_UNKNOWN;
    int best_count = 0;
    for (int i = 0; i < 8; i++) {
        if (tally[i] > best_count) {
            best_count = tally[i];
            best = (CS_ColourID_t)i;
        }
    }
    return best;
}

/* ─── Public API ───────────────────────────────────────────────────────────── */

bool CS_Init(I2C_HandleTypeDef *hi2c) {
    if (_initialised) return true;
    _hi2c = hi2c;

    /* Verify device ID (TCS34725 should return 0x44 or 0x4D) */
    uint8_t id = 0;
    if (prv_read_reg(TCS34725_REG_ID, &id, 1) != HAL_OK) return false;
    if (id != 0x44 && id != 0x4D) return false;  /* Wrong device */

    /* Configure: 24 ms integration, 4× gain */
    prv_write_reg(TCS34725_REG_ATIME,   TCS34725_ATIME_24MS);
    prv_write_reg(TCS34725_REG_CONTROL, TCS34725_GAIN_4X);

    /* Power on + enable RGBC */
    prv_write_reg(TCS34725_REG_ENABLE, TCS34725_ENABLE_PON);
    HAL_Delay(3);  /* Wait for oscillator */
    prv_write_reg(TCS34725_REG_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
    HAL_Delay(25); /* Wait one integration cycle */

    /* Turn on white LED */
    CS_LEDEnable(true);

    /* Default calibration */
    _cal.white_r_norm = 0.333f;
    _cal.white_g_norm = 0.333f;
    _cal.white_b_norm = 0.333f;
    _cal.is_valid     = false;

    memset(_vote_buf, 0, sizeof(_vote_buf));
    _vote_idx  = 0;
    _vote_full = false;
    _ema_init  = false;

    _initialised = true;
    return true;
}

void CS_LEDEnable(bool on) {
    HAL_GPIO_WritePin(CS_LED_PORT, CS_LED_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void CS_Update(CS_Result_t *result) {
    if (!_initialised || !result) return;

    /* ── Step 1: Check AVALID bit (data ready) ────────────────────────── */
    uint8_t status = 0;
    prv_read_reg(TCS34725_REG_STATUS, &status, 1);
    result->data_valid = (status & 0x01) != 0;
    result->timestamp_ms = HAL_GetTick();

    if (!result->data_valid) {
        result->colour_id    = COLOUR_UNKNOWN;
        result->voted_colour = COLOUR_UNKNOWN;
        return;
    }

    /* ── Step 2: Read 16-bit RGBC ─────────────────────────────────────── */
    result->raw.c = prv_read_word(0x14);
    result->raw.r = prv_read_word(0x16);
    result->raw.g = prv_read_word(0x18);
    result->raw.b = prv_read_word(0x1A);

    /* ── Step 3: EMA filter ───────────────────────────────────────────── */
    float fr, fg, fb, fc;
    prv_apply_ema((float)result->raw.r, (float)result->raw.g,
                  (float)result->raw.b, (float)result->raw.c,
                  &fr, &fg, &fb, &fc);

    /* ── Step 4: Normalise (chromaticity) → r/c, g/c, b/c ──────────── */
    float r_n = 0, g_n = 0, b_n = 0;
    if (fc > 10.0f) {  /* Avoid divide by zero in total darkness */
        r_n = prv_clampf(fr / fc, 0.0f, 1.0f);
        g_n = prv_clampf(fg / fc, 0.0f, 1.0f);
        b_n = prv_clampf(fb / fc, 0.0f, 1.0f);
    }

    /* ── Step 5: White-balance correction ────────────────────────────── */
    if (_cal.is_valid && _cal.white_r_norm > 0.01f) {
        r_n = prv_clampf(r_n / _cal.white_r_norm * 0.333f, 0.0f, 1.0f);
        g_n = prv_clampf(g_n / _cal.white_g_norm * 0.333f, 0.0f, 1.0f);
        b_n = prv_clampf(b_n / _cal.white_b_norm * 0.333f, 0.0f, 1.0f);
    }

    result->processed.r_norm = r_n;
    result->processed.g_norm = g_n;
    result->processed.b_norm = b_n;

    /* Approximate lux (from TCS34725 datasheet formula) */
    result->processed.lux = (-0.32466f * fr) + (1.57837f * fg) + (-0.73191f * fb);
    if (result->processed.lux < 0) result->processed.lux = 0;

    /* ── Step 6: RGB → HSV ───────────────────────────────────────────── */
    prv_rgb_to_hsv(r_n, g_n, b_n,
                   &result->processed.h,
                   &result->processed.s,
                   &result->processed.v);

    /* ── Step 7: Classify ────────────────────────────────────────────── */
    result->colour_id = prv_classify_hsv(result->processed.h,
                                          result->processed.s,
                                          result->processed.v);

    /* ── Step 8: Majority vote (7-sample temporal filter) ─────────────── */
    result->voted_colour = prv_majority_vote(result->colour_id);
}

void CS_CalibrateWhite(void) {
    if (!_initialised) return;

    /* Average 20 readings over white surface */
    float acc_r = 0, acc_g = 0, acc_b = 0, acc_c = 0;
    const int N = 20;

    for (int i = 0; i < N; i++) {
        HAL_Delay(30);  /* One integration cycle */
        uint16_t c = prv_read_word(0x14);
        uint16_t r = prv_read_word(0x16);
        uint16_t g = prv_read_word(0x18);
        uint16_t b = prv_read_word(0x1A);
        if (c > 10) {
            acc_r += (float)r / c;
            acc_g += (float)g / c;
            acc_b += (float)b / c;
            acc_c++;
        }
    }

    if (acc_c > 0) {
        _cal.white_r_norm = acc_r / acc_c;
        _cal.white_g_norm = acc_g / acc_c;
        _cal.white_b_norm = acc_b / acc_c;
        _cal.is_valid     = true;
    }
}

void CS_GetCalibration(CS_Calibration_t *cal) {
    if (cal) memcpy(cal, &_cal, sizeof(CS_Calibration_t));
}

const char *CS_ColourName(CS_ColourID_t id) {
    switch (id) {
        case COLOUR_RED:     return "RED";
        case COLOUR_GREEN:   return "GREEN";
        case COLOUR_BLUE:    return "BLUE";
        case COLOUR_YELLOW:  return "YELLOW";
        case COLOUR_WHITE:   return "WHITE";
        case COLOUR_BLACK:   return "BLACK";
        default:             return "UNKNOWN";
    }
}

void CS_PrintDebug(const CS_Result_t *r, UART_HandleTypeDef *huart) {
    if (!r || !huart) return;
    char buf[128];
    int n = snprintf(buf, sizeof(buf),
        "[CS] R=%5u G=%5u B=%5u C=%5u | H=%5.1f S=%.2f V=%.2f | %s (voted: %s)\r\n",
        r->raw.r, r->raw.g, r->raw.b, r->raw.c,
        r->processed.h, r->processed.s, r->processed.v,
        CS_ColourName(r->colour_id), CS_ColourName(r->voted_colour));
    HAL_UART_Transmit(huart, (uint8_t*)buf, (uint16_t)n, 20);
}
```

---

## 7. Colour Space Conversion

### Why Use HSV Instead of Raw RGB?

| Space | Brightness Invariant? | Hue Separable? | Notes |
|-------|-----------------------|----------------|-------|
| Raw RGB (r, g, b) | ❌ | ❌ | Conflates colour + brightness |
| Normalised rgb (r/c) | ✅ | Partial | Better, but hue still encoded in 3 vars |
| **HSV** | ✅ (separate V) | ✅ (**H only**) | Best for colour ID |
| HSL | ✅ | ✅ | Similar to HSV |
| L*a*b* | ✅ | ✅ | Most perceptually uniform; complex |

HSV allows a simple **1D threshold on Hue** for colour identification, with S and V thresholds to reject white (low S) and black (low V).

### Colour-to-HSV Reference

| Colour | Hue (H°) | Saturation (S) | Value (V) |
|--------|----------|----------------|-----------|
| Red | 0° / 360° | >0.4 | >0.2 |
| Orange | 15–35° | >0.5 | >0.2 |
| Yellow | 45–65° | >0.5 | >0.2 |
| Green | 90–150° | >0.4 | >0.15 |
| Cyan | 170–200° | >0.4 | >0.15 |
| Blue | 210–260° | >0.4 | >0.1 |
| Magenta | 280–330° | >0.4 | >0.15 |
| White | any | <0.15 | >0.6 |
| Grey | any | <0.15 | 0.15–0.6 |
| Black | any | any | <0.08 |

### RGB-to-HSV Conversion (Full Algorithm)

```c
void RGB_to_HSV(float r, float g, float b, float *H, float *S, float *V) {
    // r, g, b in [0, 1]
    float Cmax = fmaxf(fmaxf(r, g), b);
    float Cmin = fminf(fminf(r, g), b);
    float delta = Cmax - Cmin;

    // Value
    *V = Cmax;

    // Saturation
    *S = (Cmax > 1e-6f) ? (delta / Cmax) : 0.0f;

    // Hue
    if (delta < 1e-6f) {
        *H = 0.0f;  // Achromatic
    } else if (Cmax == r) {
        *H = 60.0f * fmodf(((g - b) / delta), 6.0f);
    } else if (Cmax == g) {
        *H = 60.0f * (((b - r) / delta) + 2.0f);
    } else {
        *H = 60.0f * (((r - g) / delta) + 4.0f);
    }

    if (*H < 0.0f) *H += 360.0f;
}
```

### Euclidean Distance Classifier (Alternative to HSV Lookup)

For high-accuracy classification, compute Euclidean distance in HSV space to each reference colour:

```c
typedef struct { float h, s, v; CS_ColourID_t id; } ColourRefHSV_t;

static const ColourRefHSV_t references[] = {
    { 0.0f,   0.8f, 0.7f, COLOUR_RED    },
    { 60.0f,  0.9f, 0.8f, COLOUR_YELLOW },
    { 120.0f, 0.8f, 0.6f, COLOUR_GREEN  },
    { 240.0f, 0.9f, 0.7f, COLOUR_BLUE   },
};

CS_ColourID_t classify_euclidean(float h, float s, float v) {
    float best_dist = 1e9f;
    CS_ColourID_t best_id = COLOUR_UNKNOWN;

    for (size_t i = 0; i < sizeof(references)/sizeof(references[0]); i++) {
        // Hue is circular — compute angular distance
        float dh = references[i].h - h;
        if (dh > 180.0f) dh -= 360.0f;
        if (dh < -180.0f) dh += 360.0f;
        dh /= 180.0f;  // Normalise to [0, 1]

        float ds = references[i].s - s;
        float dv = references[i].v - v;

        float dist = sqrtf(dh*dh + ds*ds + dv*dv);
        if (dist < best_dist) {
            best_dist = dist;
            best_id   = references[i].id;
        }
    }
    // Reject if too far (ambiguous)
    return (best_dist < 0.25f) ? best_id : COLOUR_UNKNOWN;
}
```

---

## 8. White-Balance & Illumination Calibration

### Why White-Balance?
Under different lighting conditions (arena LEDs, sunlight bleed, fluorescent):
- The sensor's clear channel (C) varies dramatically
- Without correction, the same red marker reads different R/G/B values

### 2-Step Calibration Protocol

**Step 1: White Reference**
Place sensor over a pure white card/surface under arena lighting.
Record `r_norm_white = r/c`, `g_norm_white = g/c`, `b_norm_white = b/c`.

**Step 2: Runtime Correction**
```
r_corrected = (r_norm / r_norm_white) × 0.333
g_corrected = (g_norm / g_norm_white) × 0.333
b_corrected = (b_norm / b_norm_white) × 0.333
```
This forces white → (0.333, 0.333, 0.333) regardless of illuminant, and scales all other colours proportionally.

### Dynamic Illumination Compensation
For arenas with variable ambient light, use the **clear (C) channel** as a real-time brightness proxy:

```c
// Gain adaptation: if C channel saturates (>60000), reduce integration time
// If C channel is very low (<500), increase gain to 60x
void CS_AdaptGain(uint16_t c_val) {
    if (c_val > 60000) {
        // Reduce gain: 4x → 1x, or reduce integration time
        prv_write_reg(TCS34725_REG_CONTROL, TCS34725_GAIN_1X);
    } else if (c_val < 500) {
        // Increase gain: 4x → 16x
        prv_write_reg(TCS34725_REG_CONTROL, TCS34725_GAIN_16X);
    }
}
```

### Gain Selection Guide

| Scenario | Recommended Gain | Integration Time |
|----------|-----------------|-----------------|
| Bright arena (>500 lux) | 1× | 24 ms |
| Standard arena (~200 lux) | **4× (default)** | **24 ms** |
| Dim arena (<100 lux) | 16× | 50 ms |
| Very dark | 60× | 101 ms |

---

## 9. Colour Classification Algorithms

### Algorithm Comparison

| Algorithm | Complexity | Accuracy | Robustness | Best For |
|-----------|-----------|----------|------------|---------|
| **HSV Threshold Lookup** | O(N) | ★★★★☆ | ★★★★☆ | **Competition (recommended)** |
| Euclidean Distance (RGB) | O(N) | ★★★☆☆ | ★★★☆☆ | Simple setups |
| Euclidean Distance (HSV) | O(N) | ★★★★☆ | ★★★★☆ | Competition backup |
| k-NN (k=3, HSV) | O(N) | ★★★★★ | ★★★★★ | When samples available |
| Decision Tree | O(log N) | ★★★★☆ | ★★★★☆ | Hand-tuned environments |
| Neural Network (MLP) | O(N²) | ★★★★★ | ★★★★★ | Off-STM32 (ESP32/RPi) |

### Recommended: HSV Threshold Lookup + Majority Vote

```
Pipeline:
   Raw RGBC → EMA filter → Normalise → White-balance
   → RGB→HSV → HSV threshold table → Colour ID
   → 7-sample majority vote → Final colour decision
```

**Advantages for competition**:
- Deterministic, no training needed
- Parameters are human-interpretable
- Fast (<1 µs per sample after filtering)
- Easy to tune on actual arena surface

### Decision Tree Classifier (Alternative)

```c
CS_ColourID_t classify_decision_tree(float r, float g, float b, float c) {
    float lum = c / 65535.0f;  // Normalised brightness

    if (lum < 0.05f) return COLOUR_BLACK;

    float r_n = r / (c + 1.0f);
    float g_n = g / (c + 1.0f);
    float b_n = b / (c + 1.0f);

    if (r_n > 0.45f && g_n < 0.30f && b_n < 0.25f) return COLOUR_RED;
    if (r_n > 0.38f && g_n > 0.38f && b_n < 0.22f) return COLOUR_YELLOW;
    if (g_n > 0.40f && r_n < 0.30f && b_n < 0.30f) return COLOUR_GREEN;
    if (b_n > 0.40f && r_n < 0.25f && g_n < 0.35f) return COLOUR_BLUE;
    if (r_n > 0.30f && g_n > 0.30f && b_n > 0.28f) return COLOUR_WHITE;

    return COLOUR_UNKNOWN;
}
```

---

## 10. Signal Filters

### Three Layers of Filtering

```
Raw I2C reading (every 24 ms)
        ↓
[Layer 1: EMA Filter]       α=0.5 — smooths reading-to-reading noise
        ↓
[Layer 2: Normalisation]    r/c, g/c, b/c → brightness-invariant
        ↓
[Layer 3: Majority Vote]    7-sample window → stable colour decision
        ↓
Final Colour ID
```

### Layer 1: Exponential Moving Average (EMA)

Applied to raw R, G, B, C values **before** normalisation:

```
EMA[n] = α × raw[n] + (1 − α) × EMA[n−1]
```

| Alpha | Effect | Latency |
|-------|--------|---------|
| 0.9 | Minimal smoothing | ~1 sample (24 ms) |
| **0.5** | **Medium — Recommended** | **~2 samples (48 ms)** |
| 0.3 | Heavy smoothing | ~4 samples (96 ms) |
| 0.2 | Very heavy | ~6 samples (144 ms) |

At 40 Hz with α = 0.5, a step change (moving sensor to new colour) settles to 95% in ~5 samples (125 ms). Arena marker size = ~50 mm → at 80 mm/s, crossing time = 625 ms (25 samples) — sufficient.

### Layer 2: Normalisation

Dividing each channel by Clear removes:
- Ambient light variation (sensor gets brighter/dimmer)
- LED power variation (battery voltage drop)
- Height variation (sensor lifts slightly on carpet)

```c
float r_norm = (float)raw_r / ((float)raw_c + 1.0f);  // +1 avoids div/0
float g_norm = (float)raw_g / ((float)raw_c + 1.0f);
float b_norm = (float)raw_b / ((float)raw_c + 1.0f);
```

### Layer 3: Majority Vote (Temporal Filter)

A 7-sample sliding window: the **most frequent colour** in the last 7 samples wins. This prevents transient misclassifications from perturbing the decision:

```
Window of 7: [RED, RED, RED, UNKN, RED, RED, RED] → Voted: RED (6/7)
Window of 7: [RED, RED, BLUE, RED, RED, RED, RED] → Voted: RED (6/7) — spike rejected
```

Latency: 7 samples × 24 ms = 168 ms — acceptable for a ~625 ms crossing window.

### Filter Performance Summary

| Layer | Technique | Noise Type Removed | Added Latency |
|-------|-----------|-------------------|--------------|
| 1 | EMA α=0.5 | ADC noise, LED flicker | ~48 ms |
| 2 | Chromaticity norm (÷C) | Brightness variation | 0 ms |
| 3 | 7-sample majority vote | Transient misclassification | ~168 ms |
| **Total** | | **All noise types** | **~216 ms** |

---

## 11. Junction Colour Detection State Machine

### Overview

```
STATE_LINE_FOLLOW → [Junction detected by line array]
        ↓
STATE_COLOUR_DETECT
  - Hold position (or slow to 30% speed)
  - Enable colour sensor LED
  - Wait for stable voted colour (≥5/7 same)
  - Record colour_decision
        ↓
STATE_COLOUR_DECIDE
  - If colour_decision == target_colour → turn to that branch
  - Else → continue straight / turn other way
        ↓
STATE_LINE_FOLLOW (on new branch)
```

### Code Integration

```c
/* In state machine — colour detection state */
void State_ColourDetect(void) {
    extern CS_Result_t g_cs_result;

    CS_Update(&g_cs_result);

    /* Wait for stable reading (voted_colour ≠ UNKNOWN for 3 consecutive) */
    static int stable_count = 0;
    static CS_ColourID_t last_voted = COLOUR_UNKNOWN;

    if (g_cs_result.voted_colour != COLOUR_UNKNOWN) {
        if (g_cs_result.voted_colour == last_voted) {
            stable_count++;
        } else {
            stable_count = 0;
            last_voted = g_cs_result.voted_colour;
        }
    } else {
        stable_count = 0;
    }

    if (stable_count >= 3) {
        /* Stable colour confirmed */
        g_sm_data.detected_colour = g_cs_result.voted_colour;
        stable_count = 0;
        last_voted   = COLOUR_UNKNOWN;
        StateMachine_Transition(STATE_COLOUR_DECIDE);
    }

    /* Timeout after 2 seconds — avoid getting stuck */
    if (HAL_GetTick() - state_enter_tick > 2000) {
        g_sm_data.detected_colour = COLOUR_UNKNOWN;
        StateMachine_Transition(STATE_LINE_FOLLOW);
    }
}
```

---

## 12. Integration with Line Array & State Machine

### Shared Sensor Data Struct Update

```c
/* In state_machine.h — add colour fields: */
typedef struct {
    /* Line array (9-channel) */
    float    line_error;
    float    line_error_mm;
    uint8_t  line_active_cnt;
    bool     intersection;
    bool     line_lost;
    uint32_t line_lost_ms;

    /* Colour sensor — NEW */
    CS_ColourID_t detected_colour;    ///< Stable voted colour at junction
    float         colour_hue;         ///< HSV hue (0–360°) for logging
    float         colour_sat;         ///< HSV saturation
    float         colour_val;         ///< HSV value (brightness)
    bool          colour_valid;       ///< True when reading is reliable
} SM_SensorData_t;
```

### Non-Blocking Integration in 50Hz Main Loop

> [!IMPORTANT]
> The actual project does NOT use a FreeRTOS colour task. The TCS34725 is polled
> in the **50 Hz TIM2-driven main loop** using `TCS34725_PollNonBlocking()`.

```c
/* In STM32_ARCHITECTURE.md 50Hz loop — actual pattern: */
if (update_flag) {
    update_flag = 0;

    /* ... other sensor reads ... */

    /* TCS34725 non-blocking poll — takes 3 cycles (60ms) to get a reading */
    TCS34725_PollNonBlocking(&htcs, &last_color);  /* ~0.1ms poll, non-blocking */

    /* Arm position commanded via sm_arm_position → PCA9685 PWM12 */
    /* STATE_TASK1_COLOR_ID: arm = ARM_MODE_A (0°), LED ON, wait 3 cycles */
    /* STATE_TASK4_JUNCTION_DETECT: arm = ARM_MODE_B (−70°), LED ON, wait 3 cycles */

    StateMachine_Update(&s);
    /* ... gait, IK, servo write ... */
}
```

### SM_SensorData_t Colour Field

```c
/* In SM_SensorData_t (state_machine.h) — actual field: */
ColorID_t last_color;   /* Updated by TCS34725_PollNonBlocking() every ~60ms */

/* State machine uses it: */
if (s->last_color == stored_ball_color) {
    /* Take matching branch at junction */
}
```

### Ball Colour Flash Persistence

```c
/* In state_machine.c — after Task 1 colour ID: */
void StateMachine_SaveBallColor(ColorID_t color) {
    stored_ball_color = color;
    _SaveColorToFlash(color);   /* Writes to 0x0807F800 (Sector 7) */
}

/* On boot: */
stored_ball_color = _ReadColorFromFlash();  /* Survives reset */
/* If Flash = 0xFF → COLOR_UNKNOWN (skip Task 1 re-ID) */
```

---

## 13. Testing Procedure

### Phase 1: Hardware Verification

| Test | Method | Pass Criteria |
|------|--------|--------------|
| T1.1 I2C communication | Read TCS34725 ID register | Returns 0x44 or 0x4D |
| T1.2 LED control | Toggle CS_LED_PIN via GPIO | LED visibly on/off |
| T1.3 ADC over white paper | Read RGBC | C > 10000, R≈G≈B |
| T1.4 ADC over black tape | Read RGBC | C < 2000 |
| T1.5 ADC over red marker | Read RGBC | R >> G, R >> B |
| T1.6 ADC over green marker | Read RGBC | G >> R, G >> B |
| T1.7 ADC over blue marker | Read RGBC | B >> R, B >> G |
| T1.8 Saturation check | Hold sensor 5 mm from white LED | C < 60000 (no saturation) |

### Phase 2: Software Verification

| Test | Method | Pass Criteria |
|------|--------|--------------|
| T2.1 White-balance cal | CS_CalibrateWhite() over white | r_norm ≈ g_norm ≈ b_norm |
| T2.2 HSV conversion | Feed known RGB, check HSV | H within ±5° of expected |
| T2.3 Red classification | Sensor over red marker | colour_id = COLOUR_RED |
| T2.4 Green classification | Sensor over green marker | colour_id = COLOUR_GREEN |
| T2.5 Blue classification | Sensor over blue marker | colour_id = COLOUR_BLUE |
| T2.6 White classification | Sensor over white | colour_id = COLOUR_WHITE |
| T2.7 Black classification | Sensor over black tape | colour_id = COLOUR_BLACK |
| T2.8 Majority vote | Rapid colour changes | voted_colour stable, not flickering |
| T2.9 EMA response | Move sensor to new colour | Settles within 150 ms |

### Phase 3: Dynamic Testing

| Test | Method | Pass Criteria |
|------|--------|--------------|
| T3.1 Moving read | Robot at 50% speed over colour marker | Correct colour detected ≥4/5 passes |
| T3.2 Full speed | Robot at 100% speed over marker | Correct colour detected ≥3/5 passes |
| T3.3 Ambient variation | Test in different lighting | Same colour classified correctly |
| T3.4 Junction detect | Approach T-junction | Colour detected within 500 ms of junction |
| T3.5 Wrong branch avoidance | Wrong colour junction | Robot does not take wrong branch |

### Phase 4: Integration Testing

| Test | Method | Pass Criteria |
|------|--------|--------------|
| T4.1 Full subtask 4 | Complete colour-sort run | Correct branch selected each time |
| T4.2 Multi-colour arena | All colour markers | Each correctly identified |
| T4.3 Power cycle | Calibration stored in Flash | White-balance survives reset |
| T4.4 Competition noise | Motors running, servos active | Colour still correctly classified |

---

## 14. Tuning Guide

### Step 1: Measure HSV of Arena Colours
Using `CS_PrintDebug()` UART output, place sensor over each arena colour marker and record H, S, V values. Fill the `_colour_table[]` in `color_sensor.c` with measured min/max ranges ±10°.

### Step 2: Set Gain & Integration Time
```
If C channel > 55000 over white at competition height → reduce gain to 1×
If C channel < 3000  over white at competition height → increase gain to 16×
Target: C ≈ 20000–40000 over white surface
```

### Step 3: Tune EMA Alpha
```
If colour flickers rapidly between two readings → reduce alpha (0.4 → 0.3)
If sensor is slow to respond to new colour     → increase alpha (0.4 → 0.6)
Competition default: α = 0.5
```

### Step 4: Tune Majority Vote Window
```
If voted_colour changes too slowly (robot already past marker) → reduce to 5 samples
If voted_colour is unstable (flickers at full speed)           → increase to 9 samples
Competition default: 7 samples (168 ms)
```

### Common Problems & Fixes

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| All colours read as UNKNOWN | Sensor saturated (C > 60000) | Reduce gain to 1× or increase height to 10 mm |
| All colours read as WHITE | Gain too low / dark surface | Increase gain to 16× |
| Red confused with orange | Hue threshold too wide | Narrow COLOUR_RED range to 350–10° |
| Green confused with yellow | Hue overlap | Check G channel: green→G dominant, yellow→R+G dominant |
| Correct in static, wrong at speed | Majority vote window too large | Reduce CS_VOTE_WINDOW to 5 |
| I2C read fails | SDA/SCL lines shorted or missing pull-up | Verify 4.7 kΩ pull-ups to 3.3 V |
| data_valid always false | AVALID bit not set | Increase integration time to 50 ms |
| Colour changes with distance | No normalisation | Ensure dividing by C channel |
| Works under LED, fails in sunlight | Ambient swamps sensor | Reduce detection height to 5 mm; shield sensor |

---

## 15. Quick Reference Card

```
┌─────────────────────────────────────────────────────────────────────────┐
│               COLOUR SENSOR — STM32 QUICK REFERENCE                     │
│               TCS34725 | FusionForce RUNNER-4                           │
├─────────────────────────────────────────────────────────────────────────┤
│ Sensor:      TCS34725 (RGBC, 16-bit, integrated white LED, IR cut)      │
│ Interface:   I²C 400 kHz | Address: 0x29                               │
│ Supply:      3.3 V | LED current: ~10 mA | IC: 0.6 mA                  │
│ Pins:        SDA=PB7, SCL=PB6, LED=PC0                                 │
├─────────────────────────────────────────────────────────────────────────┤
│ Integration: 24 ms (ATIME=0xF6) → 41 Hz update                         │
│ Gain:        4× default (1× for bright arena, 16× for dim)             │
│ Mount height: 8 mm optimal (5–12 mm acceptable)                        │
│ Position:    15 mm behind line array centre                             │
├─────────────────────────────────────────────────────────────────────────┤
│ Signal pipeline:                                                         │
│   Raw RGBC → EMA(α=0.5) → Normalise(÷C) → White-balance                │
│   → RGB→HSV → HSV lookup table → Majority vote (7 samples)             │
├─────────────────────────────────────────────────────────────────────────┤
│ HSV Colour Ranges (tune for arena):                                      │
│   RED:    H=340–15°,  S>0.30, V>0.15                                   │
│   YELLOW: H=35–75°,   S>0.35, V>0.15                                   │
│   GREEN:  H=80–160°,  S>0.30, V>0.10                                   │
│   BLUE:   H=185–270°, S>0.30, V>0.10                                   │
│   WHITE:  S<0.15, V>0.60                                                │
│   BLACK:  V<0.08                                                        │
├─────────────────────────────────────────────────────────────────────────┤
│ Calibration: CS_CalibrateWhite() over white surface at competition height│
│ Update rate: 40 Hz (25 ms period, ColourTask)                           │
│ Total filter latency: ~216 ms (EMA + vote)                              │
│ Detection window: 625 ms at 80 mm/s over 50 mm marker = 25 samples     │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## Appendix A — TCS34725 Register Map Summary

| Register | Address | Default | Description |
|----------|---------|---------|-------------|
| ENABLE | 0x00 | 0x00 | Power on (PON), AEN |
| ATIME | 0x01 | 0xFF | Integration time |
| CONTROL | 0x0F | 0x00 | Gain selection |
| ID | 0x12 | 0x44 | Chip ID (verify on init) |
| STATUS | 0x13 | — | AVALID bit (bit 0) |
| CDATAL/H | 0x14–15 | — | Clear ADC data |
| RDATAL/H | 0x16–17 | — | Red ADC data |
| GDATAL/H | 0x18–19 | — | Green ADC data |
| BDATAL/H | 0x1A–1B | — | Blue ADC data |

> [!NOTE]
> All register accesses require the **command bit (0x80)** to be ORed into the
> register address. The driver handles this automatically via `TCS34725_REG_CMD`.

---

## Appendix B — Alternative Sensor Wiring (TCS3200)

If TCS34725 is unavailable, TCS3200 can be used with STM32 timer input capture:

```c
/* TCS3200 — select filter via S2/S3 pins, read frequency on OUT */
/* S2=0, S3=0 → Red | S2=1, S3=1 → Green | S2=0, S3=1 → Blue | S2=1, S3=0 → Clear */
/* S0=1, S1=0 → 20% output scale (prevents timer overflow) */

/* Measure frequency using TIM2 Input Capture (PA0) */
/* Period (µs) → frequency (Hz) → relative intensity */
/* Switch filter colour every 50 ms → 1 full reading per 200 ms (5 Hz) */
```

> [!WARNING]
> TCS3200 requires sequential measurement (one colour at a time). At 5 Hz full
> update rate, it cannot reliably detect fast-moving junction crossings above
> 40 mm/s. Use TCS34725 for full-speed operation.

---

*Report generated: September 2026 | FusionForce Robotics | RUNNER-4 Project*
*Colour Sensor System — STM32 HAL Implementation Guide*
