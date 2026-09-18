# RUNNER-4 | 3× VL53L0X Time-of-Flight (ToF) Sensor System
## Full Technical Report — STM32 HAL Implementation
### FusionForce Robotics — EN2533 BREACH PROTOCOL

> [!IMPORTANT]
> **MCU: STM32F411CEU6 (Black Pill)** — 100 MHz Cortex-M4F, 512 KB Flash, 128 KB RAM.
> Three VL53L0X ToF sensors are on **I2C2 (PB10/PB3)** — a dedicated bus separate from
> the main I2C1 (PCA9685 + MPU6050 + TCS34725).
> All three share I2C address 0x29 by default and are disambiguated at boot via
> **XSHUT pins** (PB12, PB13, PB14) into addresses 0x30, 0x31, 0x32.

---

## Table of Contents
1. [Sensor Overview & Physics](#1-sensor-overview--physics)
2. [VL53L0X Specifications & Why This Sensor](#2-vl53l0x-specifications--why-this-sensor)
3. [Sensor Comparison (ToF Options)](#3-sensor-comparison-tof-options)
4. [Hardware Circuit & Physical Placement](#4-hardware-circuit--physical-placement)
5. [I2C Address Remapping via XSHUT](#5-i2c-address-remapping-via-xshut)
6. [STM32 HAL Driver Code](#6-stm32-hal-driver-code)
7. [Wall-Following PD Controller](#7-wall-following-pd-controller)
8. [Gap Rejection Filter](#8-gap-rejection-filter)
9. [Obstacle Detection Algorithm](#9-obstacle-detection-algorithm)
10. [Corridor Centering Logic](#10-corridor-centering-logic)
11. [Sensor Fusion Per Task State](#11-sensor-fusion-per-task-state)
12. [Integration with State Machine](#12-integration-with-state-machine)
13. [Testing Procedure](#13-testing-procedure)
14. [Tuning Guide](#14-tuning-guide)
15. [Quick Reference Card](#15-quick-reference-card)

---

## 1. Sensor Overview & Physics

### Time-of-Flight Principle

VL53L0X uses **940 nm VCSEL (Vertical Cavity Surface Emitting Laser)** ranging:

```
[940nm VCSEL Laser] ──→ Emits short pulse
                            ↓
                    [Target Surface / Wall]
                            ↓
                    [SPAD Detector Array]
                            ↓
                   [TDC — Time-to-Digital Converter]
                            ↓
            Distance = c × Δt / 2
            (speed of light × time-of-flight / 2)
```

**Key advantage over IR reflectance**: Measures **actual distance** (mm), not reflectance intensity.
This means the reading is immune to:
- Wall colour variation (white vs grey walls give same reading)
- Surface texture differences
- Ambient light (IR bandpass filter around 940 nm)

### Arena Wall Context

Per competition specifications:
- Walls are **20 cm (200 mm) high**, **white** painted
- Corridor width: **~300 mm** (30 cm)
- ToF sensors mount at **~25 mm above floor** to measure wall mid-height
- Maximum wall-follow range: 0–200 mm
- Obstacle (Task 3): 25×25×20 cm block = wall-height object

```
Top view of corridor (Task 2 / Task 3):
┌─────────────────────────────────────────────────────┐ ← Left wall
│                                                     │
│  [VL53L0X Left]  ←← 150 mm →→  [Robot]  ←→→ 150 mm  [VL53L0X Right]→→│
│                                                     │
└─────────────────────────────────────────────────────┘ ← Right wall
         ↑ 300 mm corridor width
```

---

## 2. VL53L0X Specifications & Why This Sensor

### Full Specifications

| Parameter | Value |
|-----------|-------|
| Part number | VL53L0X (STMicroelectronics) |
| Technology | Time-of-Flight, VCSEL 940 nm |
| Supply voltage | 2.6–3.5 V (3.3 V ✅) |
| Logic voltage | 2.8 V (open-drain I²C — level-shift if 5V MCU) |
| Interface | I²C (up to 400 kHz) |
| I²C address | 0x29 (default; remappable via software) |
| Range | 30 mm to 1200 mm (2 m in long-range mode) |
| Field of View | 25° (cone-shaped) |
| Update rate | Up to 50 Hz (Single Ranging: ~30ms/read) |
| Accuracy | ±3% up to 1.2 m |
| Resolution | 1 mm |
| Ambient light immunity | Up to 10 klux |
| XSHUT pin | Active LOW shutdown; tristate I²C when low |
| GPIO1 | Interrupt output (optional; we use polling) |
| Package | LCC12 (2.4×4.4×1.0 mm) |
| Current | 10 mA active; 5 µA standby |
| Operating temp | –20 to +70 °C |

### Why VL53L0X for RUNNER-4?

| Criterion | Requirement | VL53L0X |
|-----------|-------------|---------|
| Range | 30–300 mm corridor width | ✅ 30–1200 mm |
| I²C interface | Must fit on STM32 I²C | ✅ I²C2 (PB10/PB3) |
| Multiple sensors same bus | 3 sensors, one bus | ✅ XSHUT address remap |
| Ambient immunity | Arena lighting 200–500 lux | ✅ Up to 10 klux |
| Wall colour independence | White walls | ✅ ToF ignores colour |
| Gap detection | 30 mm gaps in walls | ✅ Detects gap as >200mm |
| 3.3V compatible | STM32F411 logic | ✅ Direct connect |
| Update rate | 50Hz control loop | ✅ 30ms/measurement = 33Hz |

---

## 3. Sensor Comparison (ToF Options)

| Feature | **VL53L0X** ⭐ | VL53L1X | HC-SR04 | GP2Y0A21 (IR) | TFMini |
|---------|--------------|---------|---------|--------------|--------|
| **Technology** | ToF (VCSEL) | ToF (VCSEL) | Ultrasonic | IR Reflectance | ToF Lidar |
| **Interface** | I²C | I²C | GPIO (trigger+echo) | Analog ADC | UART/I²C |
| **Range** | 30–1200 mm | 40–4000 mm | 20–4000 mm | 100–800 mm | 300–12000 mm |
| **Update rate** | 33 Hz | 50 Hz | 20 Hz | 50 Hz | 100 Hz |
| **FoV** | 25° | 27° | 15° | ~5° | 2.3° |
| **Accuracy** | ±3% | ±1% | ±3 mm | ±5% | ±1% |
| **Ambient immunity** | ✅ 10 klux | ✅ 10 klux | ⚠️ Echo absorbed by foam | ❌ Sensitive | ✅ |
| **Multiple on one bus** | ✅ XSHUT remap | ✅ XSHUT remap | ❌ Need 3 GPIO pairs | ❌ Need 3 ADC | ❌ UART |
| **3.3V direct** | ✅ | ✅ | ❌ (5V typical) | ⚠️ (analogue) | ⚠️ (5V UART) |
| **Cost** | ~$2–5 | ~$5–10 | ~$0.50 | ~$2 | ~$20 |
| **STM32 overhead** | Low (I2C) | Low (I2C) | Medium (timer IC) | Low (ADC) | Medium (UART) |
| **Gap detection** | ✅ Excellent | ✅ Excellent | ⚠️ Echo scatter | ❌ Too narrow FoV | ⚠️ Too narrow |
| **Suitable for RUNNER-4** | ✅ **Best choice** | ✅ Overkill | ⚠️ No multi-bus | ❌ Range too short | ❌ Too expensive |

> [!IMPORTANT]
> **VL53L0X is the confirmed choice** for RUNNER-4, per `RUNNER4_Decision_Table.md` (Decision #10).
> Backup: Add TCA9548A I²C mux if XSHUT-based address remapping proves unreliable.

---

## 4. Hardware Circuit & Physical Placement

### Single VL53L0X Circuit

```
3.3V ──────────────────────────────────┐
                                       │
                                    100 nF (close to VDD)
                                       │
                                    VDD (VL53L0X)
                                    GND ──── GND
                                    SDA ──── STM32 PB3 (I2C2_SDA) [4.7kΩ pull-up]
                                    SCL ──── STM32 PB10 (I2C2_SCL) [4.7kΩ pull-up]
                                    XSHUT ──── STM32 GPIO (PB12/PB13/PB14)
                                    GPIO1 ──── NC (not used — polling mode)
```

> [!WARNING]
> VL53L0X I²C pins are **open-drain** and require pull-up resistors.
> Use **4.7 kΩ to 3.3 V** on both SDA and SCL. If using a breakout board,
> pull-ups are usually included — verify before adding extra resistors.

> [!NOTE]
> The VL53L0X `XSHUT` pin is **active LOW**. Pull it LOW → sensor shuts down and
> releases the I²C bus (tristate). Pull HIGH → sensor boots with its programmed address.
> This mechanism allows three sensors to share one I²C bus.

### Physical Placement

```
Front of Robot — Bottom View (not to scale)
       ← 250 mm robot width →
┌──────────────────────────────────────────────┐
│ [VL53L0X Left]  [IR Line Array] [VL53L0X Right]│
│   90° left           ●  ●  ●         90° right  │
│                   ●  ●  ●  ●  ●                 │
│                      facing forward              │
│                                                  │
│            [VL53L0X Front]                       │
│                  0° forward                      │
└──────────────────────────────────────────────────┘
                    ↑ Direction of travel
```

### Mounting Specifications

| Sensor | Position | Angle | Height | Purpose |
|--------|----------|-------|--------|---------|
| **Front** (PB12, 0x30) | Front-centre bumper | 0° (forward) | 25 mm above floor | Obstacle detection, ball approach, corridor end |
| **Left** (PB13, 0x31) | Front-left corner | 90° (left) | 25 mm above floor | Left wall distance measurement |
| **Right** (PB14, 0x32) | Front-right corner | 90° (right) | 25 mm above floor | Right wall distance measurement |

### Mounting Height Rationale
```
Arena walls: 200 mm high
Mount at: 25 mm above floor → sensor illuminates wall mid-zone (25–225mm)
25° FoV at 150mm range → spot size ≈ 70mm diameter → well within wall face
Gap: If wall has a gap, reading jumps from ~150mm to >500mm (detectable)
```

### Complete Pin Table (from [`PINOUT_AND_CONNECTIONS.md`](file:///c:/Users/ADMIN/Desktop/FusionForce-Robotics/docs/PINOUT_AND_CONNECTIONS.md))

| Signal | STM32 Pin | Notes |
|--------|----------|-------|
| I2C2 SDA | **PB3** | ToF sensor bus data |
| I2C2 SCL | **PB10** | ToF sensor bus clock |
| XSHUT — Front | **PB12** | Runtime address = 0x30 |
| XSHUT — Left | **PB13** | Runtime address = 0x31 |
| XSHUT — Right | **PB14** | Runtime address = 0x32 |
| VCC (all 3) | 3.3V | 100 nF decoupling per sensor |
| GND (all 3) | GND | |

> [!NOTE]
> The I2C address map per `PINOUT_AND_CONNECTIONS.md`:
> - Default: all 3 sensors boot at 0x29
> - After XSHUT remap: Front=0x30, Left=0x31, Right=0x32
> - TCS34725 (also 0x29) is on **I2C1** — no conflict

### Power Budget (ToF Sensors)

| Item | Current |
|------|---------|
| 3 × VL53L0X (10 mA each active) | 30 mA |
| I²C pull-ups (static current) | ~2 mA |
| **Total** | **~32 mA @ 3.3 V** |

---

## 5. I2C Address Remapping via XSHUT

### The Problem
All three VL53L0X sensors boot with the same I²C address: `0x29`. If all three are powered simultaneously, they will all respond to the same address — causing I²C bus collisions.

### The Solution: Sequential XSHUT Boot
1. Assert all XSHUT pins LOW → all sensors shut down (bus clear)
2. Release XSHUT of sensor 1 (Front, PB12) → boots at 0x29
3. Remap its address to `0x30` via I²C write
4. Release XSHUT of sensor 2 (Left, PB13) → boots at 0x29
5. Remap its address to `0x31` via I²C write
6. Release XSHUT of sensor 3 (Right, PB14) → boots at 0x29
7. Remap its address to `0x32` via I²C write
8. All three sensors are now on I2C2 at unique addresses

### XSHUT Boot Sequence Code

```c
/**
 * @brief  Boot-time address remap for all 3 VL53L0X sensors.
 *         Must be called BEFORE starting any ranging.
 *         All XSHUT pins must be configured as GPIO Output, push-pull.
 */
void VL53L0X_RemapAllAddresses(void) {
    /* Step 1: Assert all XSHUT LOW — shut down all sensors */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_Delay(10);   /* Allow sensors to power down */

    /* Step 2: Boot Front sensor at 0x29, remap to 0x30 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);  /* Release XSHUT Front */
    HAL_Delay(2);    /* Boot time: typ 1.2ms */
    VL53L0X_SetI2CAddress(&tof_front, 0x30 << 1);        /* HAL 8-bit address */

    /* Step 3: Boot Left sensor at 0x29, remap to 0x31 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);  /* Release XSHUT Left */
    HAL_Delay(2);
    VL53L0X_SetI2CAddress(&tof_left, 0x31 << 1);

    /* Step 4: Boot Right sensor at 0x29, remap to 0x32 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);  /* Release XSHUT Right */
    HAL_Delay(2);
    VL53L0X_SetI2CAddress(&tof_right, 0x32 << 1);

    /* All sensors now active at unique addresses */
}
```

> [!CAUTION]
> If power is cut and restored, all sensors return to 0x29. The address remap routine
> **must be re-executed on every boot** before reading any sensor. Failure to do so
> causes all 3 sensors to respond to 0x29 simultaneously → I²C bus lockup.

---

## 6. STM32 HAL Driver Code

### File: `vl53l0x_driver.h`

```c
/**
 * @file    vl53l0x_driver.h
 * @brief   VL53L0X ToF sensor driver — STM32 HAL I2C2
 *
 * FusionForce Robotics | RUNNER-4 | EN2533 BREACH PROTOCOL
 *
 * Wiring:
 *   I2C2_SDA  → PB3
 *   I2C2_SCL  → PB10
 *   XSHUT_F   → PB12 (Front sensor, runtime addr 0x30)
 *   XSHUT_L   → PB13 (Left  sensor, runtime addr 0x31)
 *   XSHUT_R   → PB14 (Right sensor, runtime addr 0x32)
 */
#ifndef VL53L0X_DRIVER_H
#define VL53L0X_DRIVER_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* ─── I2C Bus ──────────────────────────────────────────────────────────────── */
#define TOF_I2C_TIMEOUT_MS      10
#define TOF_ADDR_DEFAULT        (0x29 << 1)   /* Default address (boot) */
#define TOF_ADDR_FRONT          (0x30 << 1)   /* Remapped: Front sensor */
#define TOF_ADDR_LEFT           (0x31 << 1)   /* Remapped: Left  sensor */
#define TOF_ADDR_RIGHT          (0x32 << 1)   /* Remapped: Right sensor */

/* ─── XSHUT GPIO ───────────────────────────────────────────────────────────── */
#define TOF_XSHUT_PORT          GPIOB
#define TOF_XSHUT_FRONT_PIN     GPIO_PIN_12
#define TOF_XSHUT_LEFT_PIN      GPIO_PIN_13
#define TOF_XSHUT_RIGHT_PIN     GPIO_PIN_14

/* ─── Measurement Thresholds ───────────────────────────────────────────────── */
#define TOF_MAX_RANGE_MM        1200          /* VL53L0X hardware limit */
#define TOF_WALL_MAX_MM         200           /* Arena wall height (20 cm) */
#define TOF_GAP_THRESHOLD_MM    250           /* Reading > this = gap in wall */
#define TOF_GAP_CONSEC_MIN      3             /* Min consecutive gap readings */
#define TOF_OBSTACLE_MM         150           /* Front < 150mm = obstacle detected */
#define TOF_BALL_APPROACH_MM    80            /* Front < 80mm = stop at ball pedestal */
#define TOF_WALL_TARGET_MM      150           /* Desired side-wall distance (mm) */
#define TOF_UPDATE_RATE_HZ      33            /* ~30ms per read */
#define TOF_EMA_ALPHA           0.4f          /* EMA smoothing for distance readings */

/* ─── Sensor IDs ───────────────────────────────────────────────────────────── */
typedef enum {
    TOF_FRONT = 0,
    TOF_LEFT  = 1,
    TOF_RIGHT = 2,
    TOF_COUNT = 3
} TOF_SensorID_t;

/* ─── Result Struct ────────────────────────────────────────────────────────── */
typedef struct {
    uint16_t dist_mm;        /* Filtered distance in mm (0 = error/OOB) */
    uint16_t raw_mm;         /* Raw reading before EMA filter */
    bool     is_gap;         /* True when reading indicates a wall gap */
    bool     is_valid;       /* False if sensor read failed */
    uint8_t  gap_consec;     /* Consecutive gap-indicating readings */
} TOF_Reading_t;

/* ─── Global Results (filled by TOF_UpdateAll) ─────────────────────────────── */
extern TOF_Reading_t tof[TOF_COUNT];   /* tof[TOF_FRONT], tof[TOF_LEFT], tof[TOF_RIGHT] */

/* ─── Public API ───────────────────────────────────────────────────────────── */

/**
 * @brief  Remap all 3 VL53L0X sensors to unique I²C addresses via XSHUT.
 *         MUST be called at boot before TOF_Init().
 * @param  hi2c  I²C handle for I2C2 (PB10/PB3)
 */
void TOF_RemapAddresses(I2C_HandleTypeDef *hi2c);

/**
 * @brief  Initialise all 3 sensors (single-shot ranging mode, 33Hz).
 * @param  hi2c  I²C handle for I2C2
 * @return true if all 3 sensors ACK'd successfully
 */
bool TOF_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief  Read all 3 sensors, apply EMA filter, detect gaps.
 *         Call every ~30ms (33Hz) from sensor task or main loop.
 */
void TOF_UpdateAll(void);

/**
 * @brief  Get a single sensor reading (convenience accessor).
 * @param  id  TOF_FRONT, TOF_LEFT, or TOF_RIGHT
 * @return Pointer to TOF_Reading_t; dist_mm = 0 if invalid
 */
const TOF_Reading_t *TOF_Get(TOF_SensorID_t id);

/**
 * @brief  Print all 3 readings over UART for debugging.
 */
void TOF_PrintDebug(UART_HandleTypeDef *huart);

#endif /* VL53L0X_DRIVER_H */
```

### File: `vl53l0x_driver.c`

```c
/**
 * @file    vl53l0x_driver.c
 * @brief   VL53L0X ToF sensor driver — STM32 HAL I2C2 implementation
 *
 * Note: This driver uses STMicro's VL53L0X_Api or a lightweight HAL-only
 *       implementation. The HAL-only approach below communicates directly
 *       with the VL53L0X register map without the ST API for simplicity.
 *
 * VL53L0X register references: UM2039 — VL53L0X API User Manual
 */

#include "vl53l0x_driver.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* ─── VL53L0X Key Registers ────────────────────────────────────────────────── */
#define REG_IDENTIFICATION_MODEL_ID       0xC0  /* Must read 0xEE */
#define REG_SYSRANGE_START                0x00
#define REG_RESULT_INTERRUPT_STATUS       0x13
#define REG_RESULT_RANGE_STATUS           0x14  /* Status[7:3] + RangeVal[15:0] follow */
#define REG_I2C_SLAVE_DEVICE_ADDRESS      0x8A  /* Register to change I²C address */
#define REG_SYSTEM_SEQUENCE_CONFIG        0x01
#define REG_GPIO_HV_MUX_ACTIVE_HIGH       0x84

/* ─── Internal State ───────────────────────────────────────────────────────── */
static I2C_HandleTypeDef *_hi2c = NULL;
static uint8_t _addrs[TOF_COUNT] = {TOF_ADDR_FRONT, TOF_ADDR_LEFT, TOF_ADDR_RIGHT};
static float   _ema[TOF_COUNT]   = {150.0f, 150.0f, 150.0f};
static bool    _initialised      = false;

TOF_Reading_t tof[TOF_COUNT];

/* ─── I2C Helpers ──────────────────────────────────────────────────────────── */

static HAL_StatusTypeDef _WriteReg(uint8_t addr, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return HAL_I2C_Master_Transmit(_hi2c, addr, buf, 2, TOF_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef _ReadReg(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len) {
    HAL_StatusTypeDef s;
    s = HAL_I2C_Master_Transmit(_hi2c, addr, &reg, 1, TOF_I2C_TIMEOUT_MS);
    if (s != HAL_OK) return s;
    return HAL_I2C_Master_Receive(_hi2c, addr, data, len, TOF_I2C_TIMEOUT_MS);
}

static uint16_t _ReadWord(uint8_t addr, uint8_t reg) {
    uint8_t buf[2] = {0};
    _ReadReg(addr, reg, buf, 2);
    return (uint16_t)((buf[0] << 8) | buf[1]);
}

/* ─── Sensor Initialisation ────────────────────────────────────────────────── */

/**
 * @brief Minimal VL53L0X init sequence for continuous ranging.
 *        Based on ST AN4545 & VL53L0X datasheet.
 */
static bool _InitSensor(uint8_t addr) {
    uint8_t model_id = 0;

    /* Verify device ID */
    if (_ReadReg(addr, REG_IDENTIFICATION_MODEL_ID, &model_id, 1) != HAL_OK)
        return false;
    if (model_id != 0xEE) return false;  /* Wrong device */

    /* Standard init sequence (condensed) */
    _WriteReg(addr, 0x88, 0x00);   /* Set I2C standard mode */
    _WriteReg(addr, 0x80, 0x01);
    _WriteReg(addr, 0xFF, 0x01);
    _WriteReg(addr, 0x00, 0x00);
    _WriteReg(addr, 0xFF, 0x00);
    _WriteReg(addr, 0x09, 0x00);
    _WriteReg(addr, 0x10, 0x00);
    _WriteReg(addr, 0x11, 0x00);
    _WriteReg(addr, 0x24, 0x01);
    _WriteReg(addr, 0x25, 0xFF);
    _WriteReg(addr, 0xFF, 0x01);
    _WriteReg(addr, 0x75, 0x00);
    _WriteReg(addr, 0xFF, 0x00);
    _WriteReg(addr, 0x80, 0x00);

    /* Set timing budget 33ms (default = good SNR) */
    _WriteReg(addr, REG_SYSTEM_SEQUENCE_CONFIG, 0xE8);

    /* Start continuous ranging */
    _WriteReg(addr, REG_SYSRANGE_START, 0x02);   /* 0x02 = back-to-back ranging */

    return true;
}

/* ─── Single-Shot Read ─────────────────────────────────────────────────────── */

static uint16_t _ReadRangeOnce(uint8_t addr) {
    /* Poll for data ready (interrupt status bit 2) */
    uint8_t status = 0;
    uint32_t t_start = HAL_GetTick();
    while ((status & 0x07) == 0) {
        if (HAL_GetTick() - t_start > 35) return 0;  /* Timeout */
        _ReadReg(addr, REG_RESULT_INTERRUPT_STATUS, &status, 1);
    }

    /* Read range result — located at 0x14 (status byte) + 0x1E (range low) + 0x1F (range high) */
    uint8_t buf[12];
    _ReadReg(addr, REG_RESULT_RANGE_STATUS, buf, 12);
    uint16_t range_mm = (uint16_t)((buf[10] << 8) | buf[11]);

    /* Clear interrupt */
    _WriteReg(addr, 0x0B, 0x01);

    /* Saturate at TOF_MAX_RANGE_MM for consistency */
    return (range_mm > TOF_MAX_RANGE_MM) ? TOF_MAX_RANGE_MM : range_mm;
}

/* ─── EMA Filter ───────────────────────────────────────────────────────────── */

static float _ApplyEMA(int idx, float new_val) {
    _ema[idx] = TOF_EMA_ALPHA * new_val + (1.0f - TOF_EMA_ALPHA) * _ema[idx];
    return _ema[idx];
}

/* ─── Public API Implementation ────────────────────────────────────────────── */

void TOF_RemapAddresses(I2C_HandleTypeDef *hi2c) {
    _hi2c = hi2c;

    /* Assert all XSHUT LOW — power down all sensors */
    HAL_GPIO_WritePin(TOF_XSHUT_PORT,
                      TOF_XSHUT_FRONT_PIN | TOF_XSHUT_LEFT_PIN | TOF_XSHUT_RIGHT_PIN,
                      GPIO_PIN_RESET);
    HAL_Delay(10);

    /* ── Front sensor → 0x30 ── */
    HAL_GPIO_WritePin(TOF_XSHUT_PORT, TOF_XSHUT_FRONT_PIN, GPIO_PIN_SET);
    HAL_Delay(2);   /* Boot time */
    /* Write new address to REG_I2C_SLAVE_DEVICE_ADDRESS */
    uint8_t new_addr_front = TOF_ADDR_FRONT >> 1;   /* 7-bit (ST API uses 7-bit) */
    _WriteReg(TOF_ADDR_DEFAULT, REG_I2C_SLAVE_DEVICE_ADDRESS, new_addr_front);

    /* ── Left sensor → 0x31 ── */
    HAL_GPIO_WritePin(TOF_XSHUT_PORT, TOF_XSHUT_LEFT_PIN, GPIO_PIN_SET);
    HAL_Delay(2);
    uint8_t new_addr_left  = TOF_ADDR_LEFT  >> 1;
    _WriteReg(TOF_ADDR_DEFAULT, REG_I2C_SLAVE_DEVICE_ADDRESS, new_addr_left);

    /* ── Right sensor → 0x32 ── */
    HAL_GPIO_WritePin(TOF_XSHUT_PORT, TOF_XSHUT_RIGHT_PIN, GPIO_PIN_SET);
    HAL_Delay(2);
    uint8_t new_addr_right = TOF_ADDR_RIGHT >> 1;
    _WriteReg(TOF_ADDR_DEFAULT, REG_I2C_SLAVE_DEVICE_ADDRESS, new_addr_right);
}

bool TOF_Init(I2C_HandleTypeDef *hi2c) {
    _hi2c = hi2c;
    bool ok = true;

    for (int i = 0; i < TOF_COUNT; i++) {
        ok &= _InitSensor(_addrs[i]);
        tof[i].dist_mm    = 0;
        tof[i].raw_mm     = 0;
        tof[i].is_gap     = false;
        tof[i].is_valid   = false;
        tof[i].gap_consec = 0;
        _ema[i]           = 150.0f;  /* Reasonable default */
    }

    _initialised = ok;
    return ok;
}

void TOF_UpdateAll(void) {
    if (!_initialised) return;

    for (int i = 0; i < TOF_COUNT; i++) {
        uint16_t raw = _ReadRangeOnce(_addrs[i]);
        tof[i].raw_mm   = raw;
        tof[i].is_valid = (raw > 0);

        /* Apply EMA filter */
        float filtered = _ApplyEMA(i, (float)raw);
        tof[i].dist_mm  = (uint16_t)filtered;

        /* Gap detection */
        if (filtered > TOF_GAP_THRESHOLD_MM) {
            tof[i].gap_consec++;
        } else {
            tof[i].gap_consec = 0;
        }
        tof[i].is_gap = (tof[i].gap_consec >= TOF_GAP_CONSEC_MIN);
    }
}

const TOF_Reading_t *TOF_Get(TOF_SensorID_t id) {
    return &tof[id];
}

void TOF_PrintDebug(UART_HandleTypeDef *huart) {
    if (!huart) return;
    char buf[96];
    int n = snprintf(buf, sizeof(buf),
        "[ToF] F=%4umm%s L=%4umm%s R=%4umm%s\r\n",
        tof[TOF_FRONT].dist_mm, tof[TOF_FRONT].is_gap  ? "(GAP)" : "     ",
        tof[TOF_LEFT].dist_mm,  tof[TOF_LEFT].is_gap   ? "(GAP)" : "     ",
        tof[TOF_RIGHT].dist_mm, tof[TOF_RIGHT].is_gap  ? "(GAP)" : "     ");
    HAL_UART_Transmit(huart, (uint8_t*)buf, (uint16_t)n, 20);
}
```

---

## 7. Wall-Following PD Controller

### Algorithm

The wall-following PD controller uses **left and right ToF readings** to compute a centering error:

```
error = d_left − d_right        (positive = too close to right wall)
error = 0  → perfectly centred in corridor
error > 0  → robot drifted right → steer left (Wz < 0)
error < 0  → robot drifted left  → steer right (Wz > 0)
```

### PD Formula

```c
Wz = Kp_wall × error + Kd_wall × (error − prev_error) / dt
```

### Code

```c
typedef struct {
    float kp;
    float kd;
    float wz_max;
    float prev_error;
    bool  has_prev;
} WallFollower_t;

#define WALL_KP_DEFAULT     0.008f   /* rad/s per mm of error */
#define WALL_KD_DEFAULT     0.002f
#define WALL_WZ_MAX         0.60f    /* rad/s — less aggressive than line follow */
#define WALL_DT             0.02f    /* 50Hz main loop */

float WallFollower_ComputeOmega(WallFollower_t *wf,
                                 uint16_t d_left, uint16_t d_right,
                                 bool left_gap, bool right_gap) {
    float error;

    /* Gap handling — see Section 8 */
    if (left_gap && !right_gap) {
        /* Left wall has gap — maintain right wall distance only */
        error = (float)TOF_WALL_TARGET_MM - (float)d_right;
    } else if (right_gap && !left_gap) {
        /* Right wall has gap — maintain left wall distance only */
        error = (float)d_left - (float)TOF_WALL_TARGET_MM;
    } else if (left_gap && right_gap) {
        /* Both gaps — hold heading (error = 0, use last Wz) */
        error = 0.0f;
    } else {
        /* Normal: centre between both walls */
        error = (float)d_left - (float)d_right;
    }

    float d_term = 0.0f;
    if (wf->has_prev) {
        d_term = (error - wf->prev_error) / wf->dt;
    }

    float omega = wf->kp * error + wf->kd * d_term;

    /* Clamp */
    if (omega >  wf->wz_max) omega =  wf->wz_max;
    if (omega < -wf->wz_max) omega = -wf->wz_max;

    wf->prev_error = error;
    wf->has_prev   = true;
    return omega;
}
```

---

## 8. Gap Rejection Filter

### The Problem
Arena walls in Subtask 2 and 3 corridors have **gaps** (openings) on the sides:
- A gap causes the ToF reading to jump from ~150 mm to >500 mm instantly
- Without filtering, the PD controller would steer hard into the gap (false wall loss)

### The Solution: Consecutive Gap Counter

```
Single reading > 250mm → not yet confirmed as gap
3+ consecutive readings > 250mm → confirmed gap → switch to single-wall mode
Reading drops back ≤ 250mm → gap exited → resume dual-wall centering
```

```c
/* Gap detection state — per sensor */
static uint8_t _gap_consec[TOF_COUNT] = {0, 0, 0};

/* Inside TOF_UpdateAll(): */
if (filtered_mm > TOF_GAP_THRESHOLD_MM) {
    _gap_consec[i]++;
    if (_gap_consec[i] > 10) _gap_consec[i] = 10;  /* Saturate */
} else {
    _gap_consec[i] = 0;
}
tof[i].is_gap = (_gap_consec[i] >= TOF_GAP_CONSEC_MIN);  /* 3 consecutive */
```

### Gap Threshold Selection

```
Corridor width ≈ 300 mm → each wall is at ~150 mm
Gap = opening in wall → reading jumps to arena boundary or >500 mm
Threshold: 250 mm = midpoint between 150mm (wall) and 500mm+ (gap)
               → requires 3 × 30ms = 90ms to confirm (robot travels 7mm at 80mm/s)
               → robot is still inside corridor when gap confirmed ✅
```

### Single-Wall Following Mode

When one wall has a confirmed gap:
```
d_gap: Use target distance (150mm) as virtual wall reference
d_real: Use actual reading from the non-gap side

error = d_real_left − 150mm  (if right has gap)
error = 150mm − d_real_right (if left has gap)
```

This keeps the robot tracking parallel to the visible wall at 150mm offset, ignoring the gap side.

---

## 9. Obstacle Detection Algorithm

### Task 3 Obstacle Specification
- Obstacle: 25×25×20 cm block
- Arena walls also at 20 cm → obstacle fills full corridor height
- Front ToF will detect obstacle at same height as wall

### Detection Logic

```c
/* Obstacle detected when FRONT ToF reads < TOF_OBSTACLE_MM (150mm) */
/* for at least 3 consecutive readings (90ms) */

static uint8_t _obstacle_consec = 0;

bool TOF_ObstacleDetected(void) {
    if (tof[TOF_FRONT].dist_mm < TOF_OBSTACLE_MM) {
        _obstacle_consec++;
        if (_obstacle_consec >= 3) return true;
    } else {
        _obstacle_consec = 0;
    }
    return false;
}

bool TOF_ObstacleCleared(void) {
    /* Obstacle cleared when FRONT reads > 300mm for 3 consecutive readings */
    return (tof[TOF_FRONT].dist_mm > 300 &&
            tof[TOF_FRONT].gap_consec >= 3);
}
```

### Obstacle vs. Ball Pedestal Disambiguation

Both the ball pedestal (Task 1) and the obstacle (Task 3) cause front ToF to drop.

| Scenario | Front ToF | Side ToF | Context |
|----------|-----------|----------|---------|
| Ball on pedestal | < 80mm | ~150mm (wall) | SM state = TASK1_BALL_APPROACH |
| Obstacle block | < 150mm | ~150mm (wall) | SM state = TASK3_WALL_FOLLOW |
| End of corridor | < 80mm | Gap both sides | All ToF low |

The **state machine context** disambiguates — same sensor reading, different current state.

---

## 10. Corridor Centering Logic

### Dual-Wall Centering (Primary)
Used in Subtask 2 and 3 when both walls are visible:

```
Centre error = d_left − d_right
Perfect centre: d_left = d_right = 150mm → error = 0
```

### Single-Wall Tracking (Gap Mode)
Used when one side has a confirmed gap:

```
d_left = real reading
d_right = virtual 150mm (gap side replaced with target)
error = d_left − 150mm
```

### End-of-Corridor Detection

```c
/* All 3 sensors read < 150mm simultaneously → T-junction or dead-end */
bool TOF_CorridorEnd(void) {
    return (tof[TOF_FRONT].dist_mm < 150 &&
            tof[TOF_LEFT].dist_mm  < 200 &&
            tof[TOF_RIGHT].dist_mm < 200);
}
```

### Complete Wall-Following Flow Diagram

```
                    ┌─────────────────┐
                    │ TOF_UpdateAll() │  ← Every 30ms
                    └────────┬────────┘
                             │
              ┌──────────────┼──────────────┐
        Left gap?        Both?           Right gap?
              │                               │
              ▼                               ▼
    Single-wall (left)              Single-wall (right)
    error = d_L − 150              error = 150 − d_R
              │                               │
              └──────────────┬────────────────┘
                             │ No gap
                             ▼
                    Dual-wall centering
                    error = d_L − d_R
                             │
                             ▼
                    WallFollower_ComputeOmega()
                    Wz = Kp × error + Kd × Δerror
                             │
                             ▼
                    Gait_SetTwist(Vx, 0, Wz)
```

---

## 11. Sensor Fusion Per Task State

| Task State | Front ToF | Left ToF | Right ToF | Notes |
|-----------|----------|---------|---------|-------|
| `STATE_TASK1_LINE_FOLLOW` | Off | Off | Off | Line array primary |
| `STATE_TASK1_BALL_APPROACH` | **< 80mm → stop** | Off | Off | Approach pedestal |
| `STATE_TASK2_WALL_FOLLOW` | Corridor end detect | **Wall PD** | **Wall PD** | Primary: dual-wall centering |
| `STATE_TASK3_WALL_FOLLOW` | Obstacle monitor | **Wall PD** | **Wall PD** | Same as Task 2 |
| `STATE_TASK3_OBSTACLE_DETECT` | **< 150mm → push** | Gap monitor | Gap monitor | Obstacle confirmed |
| `STATE_TASK3_PUSH` | **> 300mm → cleared** | Gap monitor | Gap monitor | IMU tilt monitor primary |
| `STATE_TASK3_TURN` | Confirms corridor end | Off | Off | Turn after push |
| `STATE_TASK4_LINE_FOLLOW` | Off | Off | Off | Line array primary |
| All states | Emergency: < 50mm → `SAFE_STOP` | Off | Off | Safety only |

---

## 12. Integration with State Machine

### `SM_SensorData_t` ToF Fields

```c
/* In state_machine.h — current struct (already correct): */
typedef struct {
    /* ... line array fields ... */
    uint16_t  tof_front_mm;   /* Front sensor filtered distance (mm) */
    uint16_t  tof_left_mm;    /* Left sensor filtered distance (mm) */
    uint16_t  tof_right_mm;   /* Right sensor filtered distance (mm) */
    /* ... colour, IMU fields ... */
} SM_SensorData_t;
```

### 50Hz Main Loop Integration

```c
/* In STM32_ARCHITECTURE.md 50Hz loop: */
if (update_flag) {
    update_flag = 0;

    /* ToF reads every ~30ms — call each cycle, returns cached result if not ready */
    /* In practice: call TOF_UpdateAll() from a 33Hz timer callback or separate task */
    tof_F = tof[TOF_FRONT].dist_mm;
    tof_L = tof[TOF_LEFT].dist_mm;
    tof_R = tof[TOF_RIGHT].dist_mm;

    /* Pack into SM sensor data */
    SM_SensorData_t s;
    /* ... fill line array fields ... */
    s.tof_front_mm = tof_F;
    s.tof_left_mm  = tof_L;
    s.tof_right_mm = tof_R;
    /* ... fill colour, IMU ... */

    StateMachine_Update(&s);
    /* ... gait, IK, PCA9685 write ... */
}
```

### Wall-Following State Implementation

```c
/* In state_machine.c — Task 2 wall following handler: */
static void _HandleTask2WallFollow(const SM_SensorData_t *s) {
    bool left_gap  = tof[TOF_LEFT].is_gap;
    bool right_gap = tof[TOF_RIGHT].is_gap;

    /* Compute wall-following Wz */
    float wz = WallFollower_ComputeOmega(&wall_pd,
                                          s->tof_left_mm,
                                          s->tof_right_mm,
                                          left_gap,
                                          right_gap);
    sm_Vx = 60.0f;   /* mm/s forward, slower in corridor */
    sm_Vy = 0.0f;
    sm_Wz = wz;

    /* End of corridor: all walls close + line array sees line */
    if (s->tof_front_mm < 200 && s->intersection) {
        StateMachine_Transition(STATE_TASK3_WALL_FOLLOW);
    }
}
```

---

## 13. Testing Procedure

### Phase 1: Hardware Verification

| Test | Method | Pass Criteria |
|------|--------|--------------|
| T1.1 I2C detection | Read model ID (0xC0) from each sensor | Returns 0xEE for all 3 |
| T1.2 XSHUT remap | Boot sequence; read each address | 0x30, 0x31, 0x32 all ACK |
| T1.3 Front range | Hold target 200mm from sensor | Reads 195–205mm |
| T1.4 Left range | Parallel wall at 150mm | Reads 145–155mm |
| T1.5 Right range | Parallel wall at 150mm | Reads 145–155mm |
| T1.6 Gap detection | Open air on one sensor | is_gap = true after 90ms |
| T1.7 Power cycle | Disconnect/reconnect 3.3V | Remap runs again; all addresses valid |

### Phase 2: Software Verification

| Test | Method | Pass Criteria |
|------|--------|--------------|
| T2.1 EMA filter | Move sensor suddenly | Smooth response, no step jump |
| T2.2 Wall PD zero | d_left = d_right = 150mm | Wz output = 0.0 |
| T2.3 Wall PD correction | d_left = 100mm, d_right = 200mm | Wz < 0 (steer left) |
| T2.4 Gap detection | Remove right wall target | is_gap=true, single-wall mode |
| T2.5 Obstacle detect | Hold target 100mm from front | ObstacleDetected() = true |
| T2.6 Gap consec timer | Quick flash of gap | 1 reading — not flagged as gap |

### Phase 3: Dynamic Testing

| Test | Method | Pass Criteria |
|------|--------|--------------|
| T3.1 Straight corridor | Drive full Task 2 corridor | Robot stays within ±20mm of centre |
| T3.2 Gap navigation | Corridor with one gap on left | Robot maintains right-wall track |
| T3.3 Both-gap navigation | Gap both sides | Robot holds heading through both gaps |
| T3.4 Obstacle Task 3 | Full Task 3 corridor with obstacle | Detects, pushes, confirms cleared |
| T3.5 Ball approach | Front ToF guides to pedestal | Stops at 80mm ± 10mm |

### Phase 4: Integration Testing

| Test | Method | Pass Criteria |
|------|--------|--------------|
| T4.1 Full Task 2 | Arena corridor run | Completes without wall contact |
| T4.2 Full Task 3 | Arena obstacle run | Pushes obstacle, turns correctly |
| T4.3 SM transitions | Line→wall→line | SM transitions at correct ToF thresholds |
| T4.4 Competition noise | All motors running | ToF readings stable (EMA filters noise) |

---

## 14. Tuning Guide

### Wall-Following PD Tuning

**Step 1: Start with Kd = 0**
```
Kp = 0.005 rad/s per mm — robot oscillates slowly
Increase until robot oscillates → record Kp_osc
Set Kp = 0.5 × Kp_osc (typically 0.006–0.010)
```

**Step 2: Add Kd**
```
Increase Kd until oscillation is damped
Typical: Kd = 0.001–0.003
```

**Step 3: Verify gap handling**
```
Test single-wall mode: robot should maintain ~150mm from visible wall
Adjust TOF_WALL_TARGET_MM to match actual optimal corridor offset
```

### Common Problems & Fixes

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Sensor reads 0 at boot | XSHUT not released | Check PB12/13/14 GPIO init as Output |
| All 3 sensors at same address | Remap sequence wrong | Verify sequential XSHUT + delay + write |
| Robot drifts toward one wall | Kp asymmetry or mounting offset | Adjust `TOF_WALL_TARGET_MM` or add offset trim |
| False gap detection | Gap threshold too low | Increase `TOF_GAP_THRESHOLD_MM` to 300mm |
| Gap not detected | Gap threshold too high | Lower to 200mm; check `TOF_GAP_CONSEC_MIN` |
| Wall-following oscillates | Kp too high | Reduce by 30%; increase Kd |
| Obstacle not detected | Threshold too low | Lower `TOF_OBSTACLE_MM` to 120mm |
| I2C bus hangs | Sensor XSHUT glitch on power surge | Add 100nF decoupling per sensor; debounce XSHUT |
| EMA too slow | Alpha too low | Increase `TOF_EMA_ALPHA` from 0.4 to 0.6 |
| EMA too noisy | Alpha too high | Reduce to 0.3 |

---

## 15. Quick Reference Card

```
┌─────────────────────────────────────────────────────────────────────────┐
│              3× VL53L0X ToF SYSTEM — STM32 QUICK REFERENCE              │
│              FusionForce RUNNER-4 | EN2533 BREACH PROTOCOL              │
├─────────────────────────────────────────────────────────────────────────┤
│ Sensors: 3× VL53L0X (940nm VCSEL ToF, I²C, 1mm resolution)             │
│ I2C Bus: I2C2 → PB10 (SCL), PB3 (SDA)                                  │
│ Addresses: Front=0x30 (PB12), Left=0x31 (PB13), Right=0x32 (PB14)      │
│ Default addr: 0x29 → remapped at boot via XSHUT sequence                │
├─────────────────────────────────────────────────────────────────────────┤
│ Range: 30–1200mm | FoV: 25° | Update rate: ~33 Hz (30ms/read)          │
│ Mount height: 25mm | EMA alpha: 0.4 | Gap threshold: 250mm (3 consec)  │
│ Wall target: 150mm (each side) | Obstacle: Front < 150mm               │
│ Ball approach: Front < 80mm → stop                                      │
├─────────────────────────────────────────────────────────────────────────┤
│ Wall PD:  error = d_left − d_right  | Kp=0.008, Kd=0.002, Wz_max=0.60 │
│ Gap mode: single-wall (virtual 150mm on gap side)                       │
│ Both gap: hold last heading (error = 0)                                 │
├─────────────────────────────────────────────────────────────────────────┤
│ Task 1: Front<80mm → stop at pedestal                                   │
│ Task 2: Dual-wall PD centering + gap rejection                          │
│ Task 3: Same as Task 2 + Front<150mm → obstacle detected                │
│ Task 4: ToF idle (line array primary)                                   │
├─────────────────────────────────────────────────────────────────────────┤
│ Backup: Add TCA9548A I²C mux if XSHUT remap unreliable                 │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## Appendix A — VL53L0X Register Summary

| Register | Address | Value | Description |
|----------|---------|-------|-------------|
| IDENTIFICATION_MODEL_ID | 0xC0 | 0xEE | Verify sensor ID on init |
| SYSRANGE_START | 0x00 | 0x01 (single) / 0x02 (cont.) | Start ranging |
| RESULT_INTERRUPT_STATUS | 0x13 | Bit[2:0]=7 → ready | Poll data ready |
| RESULT_RANGE_STATUS | 0x14 | Bytes [10:11] = range (mm) | Read result |
| I2C_SLAVE_DEVICE_ADDRESS | 0x8A | New 7-bit address | Remap I²C address |

## Appendix B — I2C2 STM32CubeMX Configuration

```
STM32CubeMX → Connectivity → I2C2
  Mode: I2C
  Speed Mode: Fast Mode (400 kHz)
  
GPIO:
  PB3  → I2C2_SDA (AF4), Open-drain, pull-up
  PB10 → I2C2_SCL (AF4), Open-drain, pull-up
  PB12 → GPIO_Output, Push-Pull (XSHUT Front)
  PB13 → GPIO_Output, Push-Pull (XSHUT Left)
  PB14 → GPIO_Output, Push-Pull (XSHUT Right)
```

> [!NOTE]
> I2C2 is on a **separate bus from I2C1** (PCA9685 + MPU6050 + TCS34725).
> This isolates the slower ToF timing from the faster servo/IMU updates on I2C1.
> Per `PINOUT_AND_CONNECTIONS.md`: I2C1 = PB6/PB7, I2C2 = PB10/PB3.

---

*Report generated: September 2026 | FusionForce Robotics | RUNNER-4 Project*
*MCU: STM32F411CEU6 | 3× VL53L0X ToF Sensor System — Wall Following & Obstacle Detection*
