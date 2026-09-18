# RUNNER-4 | 2-DOF Arm + Gripper + Storage Gate System
## Full Technical Report — STM32 HAL Implementation
### FusionForce Robotics — EN2533 BREACH PROTOCOL

> [!IMPORTANT]
> **MCU: STM32F411CEU6 (Black Pill)** — 100 MHz Cortex-M4F, 512 KB Flash, 128 KB RAM.
> All three mechanism servos are driven by **PCA9685** on I2C1 (0x40, PB6/PB7):
> - **CH12** — Arm Pitch (MG90S) — ARM_HOME / ARM_MODE_A / ARM_MODE_B
> - **CH13** — Gripper (MG90S) — GRIPPER_OPEN / GRIPPER_CLOSE
> - **CH14** — Storage Gate (MG90S) — GATE_LOCKED / GATE_OPEN
>
> All motion commands are global variables set by the state machine:
> `sm_arm_position`, `sm_gripper`, `sm_gate` (from [`state_machine.h`](file:///c:/Users/ADMIN/Desktop/FusionForce-Robotics/firmware/Navigation/state_machine.h)).

---

## Table of Contents
1. [System Overview](#1-system-overview)
2. [Mechanical Design — 2-DOF Arm](#2-mechanical-design--2-dof-arm)
3. [Mechanical Design — Internal Storage Compartment](#3-mechanical-design--internal-storage-compartment)
4. [Servo Selection & Analysis](#4-servo-selection--analysis)
5. [PCA9685 PWM Configuration](#5-pca9685-pwm-configuration)
6. [STM32 HAL Driver Code](#6-stm32-hal-driver-code)
7. [Motion Sequencing & Timing](#7-motion-sequencing--timing)
8. [State Machine Integration](#8-state-machine-integration)
9. [Suggested Improvements](#9-suggested-improvements)
10. [Testing Procedure](#10-testing-procedure)
11. [Tuning Guide](#11-tuning-guide)
12. [Failure Modes & Mitigations](#12-failure-modes--mitigations)
13. [Quick Reference Card](#13-quick-reference-card)

---

## 1. System Overview

### Role in the Competition

The ball manipulation system serves **two critical tasks**:

| Task | System Role | States Involved |
|------|-------------|-----------------|
| **Task 1** — Grid + Ball Retrieval | Arm lowers to pedestal → Colour sensor reads ball → Gripper grasps ball → Arm lifts → Gate locks ball inside | `BALL_APPROACH → COLOR_ID → BALL_GRAB → STORE` |
| **Task 4** — Colour Sort Delivery | Arm points at floor → Sensor reads zone colour → Robot navigates matching branch → Gate opens → Ball drops by gravity | `JUNCTION_DETECT → BRANCH_FOLLOW → BALL_RELEASE` |

### Component Map

```
Front of Robot (direction of travel → right)
                 ↑
┌────────────────────────────────────────────────┐
│                                                │
│  [Arm Pivot at front body wall]                │
│      │                                         │
│    CH12 (Arm Pitch MG90S)                      │
│      │                                         │
│      ├──── ARM_HOME  (90°)  ← travel position  │
│      ├──── ARM_MODE_A (0°)  ← ball level       │
│      └──── ARM_MODE_B (−70°)← floor level      │
│                                                │
│  [Gripper — attached at arm tip]               │
│      CH13 (Gripper MG90S)                      │
│      GRIPPER_OPEN  → claw wide (>40mm gap)     │
│      GRIPPER_CLOSE → claw grips 40mm ball      │
│                                                │
│  [TCS34725 — rigidly at gripper tip]           │
│      Shroud blocks arena light                 │
│      Controlled via PC0 (LED enable)           │
│                                                │
│  [Storage Compartment — belly of robot]        │
│      ~50mm diameter × 50mm deep                │
│      CH14 (Gate MG90S) at bottom              │
│      GATE_LOCKED → ball held inside            │
│      GATE_OPEN   → ball drops by gravity       │
└────────────────────────────────────────────────┘
```

### System Data Flow

```
SM state commands sm_arm_position / sm_gripper / sm_gate
    ↓
ARM_Controller_Update() [called every 20ms in 50Hz loop]
    ↓
PCA9685_SetAngle(CH12, arm_angle_deg)
PCA9685_SetAngle(CH13, gripper_angle_deg)
PCA9685_SetAngle(CH14, gate_angle_deg)
    ↓
I2C1 @ 400kHz → PCA9685 → PWM CH12/13/14 → 3× MG90S servos
```

---

## 2. Mechanical Design — 2-DOF Arm

### Arm Geometry

```
Side view of arm in MODE_A (0°, horizontal):

[Body Front Wall]
       │
  ─────┤  ← ARM_HOME position (90°, pointing up)
       │
  pivot│──────────────────[Gripper Claw]──[TCS34725]
       │          ← ARM_MODE_A (0°, horizontal)
       │
      \│  ← ARM_MODE_B (−70°, tilted downward)
       \─────────────[Gripper Claw]──[TCS34725]
```

### Degrees of Freedom

| DOF | Joint | Servo | Channel | Range | Function |
|-----|-------|-------|---------|-------|----------|
| 1 | Arm Pitch | MG90S | CH12 | HOME: ~90° / MODE_A: ~0° / MODE_B: ~160° | Elevates/lowers arm and sensor |
| 2 | Gripper Open/Close | MG90S | CH13 | Open: ~60° / Close: ~130° | Grasps and releases 40mm ball |

> [!NOTE]
> The arm has **no coxa (yaw) joint** — it only pitches up/down from the front body wall.
> The robot body rotation (Wz via gait Wz) provides any left/right alignment to the ball pedestal.

### Arm Link Lengths (Recommended)

| Segment | Length | Purpose |
|---------|--------|---------|
| Pivot height above floor | ~50 mm | Arm pivot at 50mm height (mounted at body front, mid-deck level) |
| Arm link (pivot to gripper tip) | ~80 mm | Reach of 80mm from pivot → tip at ~130mm from floor in MODE_A |
| TCS34725 overhang past claw | ~10 mm | Sensor clears claw structure for unobstructed reading |

**In MODE_A (0°, horizontal):**
- Arm link is horizontal → gripper tip is at **~50mm above floor** ✅ (ball pedestal height)
- TCS34725 is **1–2 cm from ball surface** ✅

**In MODE_B (−70°, tilted down):**
- Arm tip is at floor level → sensor is **1–3 cm from floor** ✅
- Robot is at arena floor height for junction colour reading ✅

### Key Geometry Constraint: Ball Pedestal

```
Ball pedestal height: 5 cm = 50 mm
Robot arm pivot: ~50 mm above ground (mid-deck front wall)
Arm link: ~80 mm

In MODE_A (0°):
  Tip X = pivot_X + 80 cos(0°) = pivot_X + 80 mm
  Tip Z = pivot_Z + 80 sin(0°) = pivot_Z = 50 mm above floor ✅

Required reach: pedestal is at ~0–100mm in front of robot front face
  → With 80mm arm and robot body, arm tip extends ~30–40mm beyond front bumper → fits ✅
```

> [!WARNING]
> The arm must NOT collide with the passive front bumper plate when moving to MODE_A.
> Design arm pivot **above** the bumper plate top edge (~50mm), and ensure the arm clears
> the bumper on the way down. A 3D-printed guide/stop can limit downward travel.

### 3D Print Specifications

| Part | Material | Infill | Notes |
|------|----------|--------|-------|
| Arm link | PETG | 40% | Lightweight; gyroscope arm shape preferred |
| Arm pivot bracket | PETG | 50% | Attached to body front wall; must handle torque |
| Gripper left claw | PETG | 35% | Flexible enough for ball compliance |
| Gripper right claw | PETG | 35% | Mirror of left |
| Gripper hinge bridge | PETG | 50% | Connects both claws, mounts CH13 servo |
| TCS34725 tip shroud | PETG/PLA | 25% | Light shield; snap-fit onto arm tip |
| Gate trapdoor | PETG | 30% | Rotates on servo horn |

---

## 3. Mechanical Design — Internal Storage Compartment

### Compartment Geometry

```
Cross-section view (front of robot → left):

     ┌──────────────────────────┐ ← Top opening (ball drops in from arm)
     │                          │
     │     ○ 40mm ball          │
     │   (stored in belly)      │
     │                          │ ← ~50mm diameter, ~55mm deep
     └──────────┬───────────────┘
                │
           [Gate Trapdoor]  ← CH14 MG90S rotates 90°
                │
         GATE_LOCKED: trapdoor closed (ball stays in)
         GATE_OPEN:   trapdoor opens → ball drops by gravity
```

### Compartment Specifications

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| Internal diameter | ≥ 50 mm | 40mm ball + 5mm clearance each side |
| Internal depth | ≥ 55 mm | 40mm ball diameter + 15mm for gate servo travel |
| Top opening | Full-width (50mm+) | Ball drops in without getting stuck |
| Location | Belly of robot, front-centre | Ball CoM near robot CoM; gravity-assisted release |
| Gate | Servo-actuated bottom trapdoor | Same servo for lock and release (GATE_OPEN) |
| Gate travel | 90° rotation | Servo horn sweeps gate from closed to open position |
| Gate material | PETG | Rigid; ball rests on it when locked |

### Gate Servo Mechanics

```
GATE_LOCKED position (ball inside):
  Gate servo → 0° → horizontal gate plate blocks bottom opening
  Ball rests on gate plate
  
GATE_OPEN position (ball release):
  Gate servo → 90° → gate plate rotates away
  Ball drops freely by gravity through bottom opening
  Drop height: ~50mm (compartment depth) + floor clearance ← very gentle drop ✅

Gate return after release:
  Gate servo → 0° → returns to locked position (for next run)
```

### Ball Journey Sequence

```
Step 1: Robot approaches ball pedestal (front ToF < 80mm)
        ↓
Step 2: Arm lowers to MODE_A (0°) — 800ms wait
        ↓
Step 3: TCS34725 reads ball colour (50ms integration, 3 stable readings)
        ↓
Step 4: Gripper closes (CH13 → CLOSE) — 1200ms wait (full grip)
        ↓
Step 5: Arm returns to HOME (90°) — arm lifts ball up — 800ms wait
        ↓
Step 6: [Manual or passive]: arm sweeps over body top opening
        Gate stays LOCKED. Ball is released from gripper into compartment.
        Gripper opens over compartment mouth → ball drops in.
        ↓
Step 7: Gate locks, gripper opens, arm returns HOME
        Ball secured in belly compartment for Task 2 + Task 3 navigation
        ↓
Step 8 (Task 4): Robot reaches correct colour branch end
        ↓
Step 9: Gate OPENS (90°) → ball drops to floor — 1500ms wait
        ↓
Step 10: Gate closes, FINISH state
```

> [!TIP]
> **Suggested improvement**: Add a brief **arm sweep** sequence between Step 5 and 6:
> After lifting ball with arm at HOME (90°), rotate arm to `ARM_STORE` (~135°, pointing backward over body)
> and open gripper → ball drops into compartment top opening by gravity.
> This requires a 4th arm position constant and a `STATE_TASK1_ARM_SWEEP` state transition.
> See [Section 9](#9-suggested-improvements) for full design.

---

## 4. Servo Selection & Analysis

### MG90S Specifications (per servo)

| Parameter | Value |
|-----------|-------|
| Stall torque | 1.8 kg·cm @ 4.8V / **2.2 kg·cm @ 6.0V** |
| No-load speed | 0.10 sec/60° @ 4.8V / **0.08 sec/60° @ 6.0V** |
| Operating current | ~200–350 mA (loaded) |
| Stall current | ~700 mA |
| Weight | ~13.4 g |
| Control pulse | 500–2400 µs (PWM) |
| Gear type | Metal (all gears) |
| Operating voltage | 4.8–6.0 V |

### Torque Analysis — Arm Servo (CH12)

```
Ball mass: ~30 g (40mm rubber/foam ball)
Arm link length: 80 mm

Torque required to hold ball at MODE_A:
  T = m × g × L = 0.030 kg × 9.81 m/s² × 0.080 m
  T = 0.0235 N·m = 2.35 kg·cm

MG90S torque @ 6V: 2.2 kg·cm
Margin: 2.2 / 2.35 = 0.94× ← MARGINALLY INSUFFICIENT for worst case
```

> [!WARNING]
> **Arm servo (CH12) torque margin is <1× at worst case** (holding 30g ball at full arm extension).
> Mitigations:
> 1. **Shorten arm link** to 65mm → T = 1.92 kg·cm → margin = 1.15× ✅
> 2. **Run arm servo at 6V** (not 4.8V) — always required
> 3. **Minimise hold time** at MODE_A — grab quickly (≤1.5s)
> 4. **Lighter arm link** — hollow/gyroscope print to reduce arm self-weight
> 5. **Upgrade CH12 only to MG996R** (10 kg·cm) — 100g heavier but 4.5× margin ✅

### Torque Analysis — Gripper Servo (CH13)

```
Ball: 40mm diameter, rubber/foam — requires gentle but secure grip
Gripper jaw length: ~20mm (moment arm from claw hinge)
Required grip force: ~0.5N (light ball won't roll out if gripper is properly shaped)

Torque required: 0.5N × 0.020m = 0.010 N·m = 1.02 kg·cm
MG90S @ 6V: 2.2 kg·cm → Margin: 2.15× ✅
```

### Torque Analysis — Gate Servo (CH14)

```
Gate mass: ~8g (small trapdoor plate)
Ball mass on gate: 30g (ball rests on gate when LOCKED)
Gate plate width: 50mm → lever arm to servo horn ~20mm

Static load: 0.038 kg × 9.81 × 0.020m = 0.0075 N·m = 0.76 kg·cm
MG90S @ 6V: 2.2 kg·cm → Margin: 2.89× ✅
```

> [!TIP]
> Gate servo is the most relaxed of the three — margin is generous. 
> Consider adding a **locking detent** (small PETG tab) on the LOCKED position so the 
> gate servo can be de-energised during navigation to save ~200mA and reduce heat.

---

## 5. PCA9685 PWM Configuration

### PCA9685 Channel Allocation (Mechanism Channels Only)

| CH | Assignment | Servo | Position A | Angle A | Position B | Angle B | Position C | Angle C |
|----|-----------|-------|-----------|---------|-----------|---------|-----------|---------|
| **12** | Arm Pitch | MG90S | HOME | ~90° | MODE_A (ball) | ~0° | MODE_B (floor) | ~160° |
| **13** | Gripper | MG90S | OPEN | ~60° | CLOSE | ~130° | — | — |
| **14** | Gate | MG90S | LOCKED | ~0° | OPEN | ~90° | — | — |

> [!NOTE]
> Exact angles MUST be calibrated per-build on physical hardware.
> Servo horn mounting zero point varies between builds.
> Start from these approximate values and adjust using debug UART.

### PWM Timing for MG90S

```
MG90S control pulse range:
  0°   → ~500 µs pulse width
  90°  → ~1500 µs pulse width (neutral)
  180° → ~2400 µs pulse width

PCA9685 at 50Hz (20ms period), 12-bit resolution:
  Tick resolution = 20ms / 4096 = 4.88 µs per tick

  Ticks for angle θ:
  tick = (pulse_min + θ/180 × (pulse_max − pulse_min)) / 4.88µs

For MG90S (500–2400 µs):
  tick(0°)   = 500/4.88  ≈ 102
  tick(90°)  = 1500/4.88 ≈ 307
  tick(180°) = 2400/4.88 ≈ 491
```

### PWM Tick Constants (after per-servo calibration)

```c
/* CH12 — Arm Pitch */
#define ARM_PWM_HOME      307   /* ~90° — resting/travel position */
#define ARM_PWM_MODE_A    102   /* ~0°  — ball level (adjust per build) */
#define ARM_PWM_MODE_B    450   /* ~160° — floor pointing (adjust per build) */
#define ARM_PWM_STORE     380   /* ~135° — over-body ball drop position (SUGGESTED) */

/* CH13 — Gripper */
#define GRIP_PWM_OPEN     184   /* ~60°  — claw fully open (>42mm gap for 40mm ball) */
#define GRIP_PWM_CLOSE    286   /* ~115° — claw closed onto ball */

/* CH14 — Gate */
#define GATE_PWM_LOCKED   102   /* ~0°  — gate closed, ball held inside */
#define GATE_PWM_OPEN     307   /* ~90° — gate open, ball drops */
```

> [!IMPORTANT]
> These PWM tick values are **estimates only**. Physical calibration is essential:
> 1. Power servos at 6V
> 2. Manually command each angle via debug UART
> 3. Verify physical position matches intent
> 4. Adjust tick constants in firmware
> 5. Verify gripper gap is ≥42mm at OPEN and ≤38mm at CLOSE for secure hold

---

## 6. STM32 HAL Driver Code

### File: `arm_controller.h`

```c
/**
 * @file    arm_controller.h
 * @brief   2-DOF Arm + Gripper + Gate Controller for RUNNER-4
 *
 * FusionForce Robotics | EN2533 BREACH PROTOCOL
 *
 * Controls:
 *   CH12 — Arm Pitch (ARM_HOME / ARM_MODE_A / ARM_MODE_B / ARM_STORE)
 *   CH13 — Gripper   (GRIPPER_OPEN / GRIPPER_CLOSE)
 *   CH14 — Gate      (GATE_LOCKED / GATE_OPEN)
 *
 * Uses PCA9685 driver (I2C1, 0x40) for PWM output.
 * Call ARM_Update() every 20ms from main 50Hz loop.
 */

#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* ─── PCA9685 Channels ─────────────────────────────────────────────────────── */
#define ARM_CH_PITCH    12   /* CH12 — Arm elevation servo */
#define ARM_CH_GRIPPER  13   /* CH13 — Claw open/close servo */
#define ARM_CH_GATE     14   /* CH14 — Storage gate servo */

/* ─── Arm Position IDs (match state_machine.h ARM_HOME / ARM_MODE_A / ARM_MODE_B) ── */
#define ARM_HOME    0   /* Travel position — arm retracted upward ~90° */
#define ARM_MODE_A  1   /* Ball colour + grab — arm horizontal ~0° */
#define ARM_MODE_B  2   /* Floor zone read — arm downward ~160° */
#define ARM_STORE   3   /* Ball deposit — arm swept back ~135° over compartment */

/* ─── Gripper State ────────────────────────────────────────────────────────── */
#define GRIPPER_OPEN    0
#define GRIPPER_CLOSE   1

/* ─── Gate State ───────────────────────────────────────────────────────────── */
#define GATE_LOCKED     0   /* Ball contained */
#define GATE_OPEN       1   /* Ball released by gravity */

/* ─── PWM Tick Constants (calibrate per-build) ─────────────────────────────── */
#define ARM_PWM_MIN     102   /* 0° pulse  (500µs @ 50Hz, 12-bit) */
#define ARM_PWM_MID     307   /* 90° pulse (1500µs) */
#define ARM_PWM_MAX     491   /* 180° pulse (2400µs) */

/* Arm positions — adjust after physical calibration */
#define ARM_TICK_HOME   307   /* 90°  — upright, retracted */
#define ARM_TICK_MODEA  102   /* 0°   — horizontal to ball */
#define ARM_TICK_MODEB  450   /* 160° — pointing at floor */
#define ARM_TICK_STORE  380   /* 135° — over body compartment */

/* Gripper positions — adjust gap width after physical calibration */
#define GRIP_TICK_OPEN  184   /* 60°  — claw fully open */
#define GRIP_TICK_CLOSE 286   /* 115° — secure ball grip */

/* Gate positions */
#define GATE_TICK_LOCKED 102  /* 0°   — gate closed */
#define GATE_TICK_OPEN   307  /* 90°  — gate open for release */

/* ─── Motion Profile ────────────────────────────────────────────────────────── */
/**
 * @brief Slew rate limiter for smooth servo motion.
 *        Maximum tick change per 20ms cycle (prevents mechanical shock).
 *        At 4.88µs/tick: 5 ticks/cycle = ~0.49° per 20ms = 24.4°/s (smooth)
 */
#define ARM_SLEW_MAX_TICKS_PER_CYCLE   8   /* Arm: ~39°/s smooth sweep */
#define GRIP_SLEW_MAX_TICKS_PER_CYCLE  12  /* Gripper: faster close/open */
#define GATE_SLEW_MAX_TICKS_PER_CYCLE  15  /* Gate: quick open for release */

/* ─── Controller Handle ────────────────────────────────────────────────────── */
typedef struct {
    /* Target positions (from state machine) */
    uint8_t  target_arm;       /* ARM_HOME / ARM_MODE_A / ARM_MODE_B / ARM_STORE */
    uint8_t  target_gripper;   /* GRIPPER_OPEN / GRIPPER_CLOSE */
    uint8_t  target_gate;      /* GATE_LOCKED / GATE_OPEN */

    /* Current PWM tick (slew-rate limited) */
    int16_t  current_arm_tick;
    int16_t  current_grip_tick;
    int16_t  current_gate_tick;

    /* Motion complete flags */
    bool     arm_at_target;
    bool     gripper_at_target;
    bool     gate_at_target;

    /* Timing */
    uint32_t motion_start_ms;
} ARM_Controller_t;

/* ─── Public API ────────────────────────────────────────────────────────────── */

/**
 * @brief  Initialise arm controller. Moves all servos to HOME / OPEN / LOCKED.
 *         Call once after PCA9685_Init().
 * @param  ctrl  Pointer to controller handle
 */
void ARM_Init(ARM_Controller_t *ctrl);

/**
 * @brief  Update servo positions with slew-rate limiting.
 *         Call every 20ms in main 50Hz loop.
 * @param  ctrl  Controller handle
 * @param  sm_arm      Target arm position (from sm_arm_position)
 * @param  sm_gripper  Target gripper state
 * @param  sm_gate     Target gate state
 */
void ARM_Update(ARM_Controller_t *ctrl,
                uint8_t sm_arm, uint8_t sm_gripper, uint8_t sm_gate);

/**
 * @brief  Returns true when arm has reached its target position.
 *         Use to gate state machine transitions.
 */
bool ARM_ArmAtTarget(const ARM_Controller_t *ctrl);

/**
 * @brief  Returns true when gripper has fully opened or closed.
 */
bool ARM_GripperAtTarget(const ARM_Controller_t *ctrl);

/**
 * @brief  Returns true when gate has fully opened or closed.
 */
bool ARM_GateAtTarget(const ARM_Controller_t *ctrl);

/**
 * @brief  Returns true when ALL three servos are at their targets.
 */
bool ARM_AllAtTarget(const ARM_Controller_t *ctrl);

/**
 * @brief  Command a specific angle to a channel directly (for calibration/debug).
 * @param  channel  PCA9685 channel (12, 13, or 14)
 * @param  angle_deg  Target angle in degrees (0–180)
 */
void ARM_SetAngleDirect(uint8_t channel, float angle_deg);

/**
 * @brief  De-energise gate servo (if using detent locking mechanism).
 *         Call when gate is confirmed LOCKED to save power.
 */
void ARM_DeenergiseGate(void);

#endif /* ARM_CONTROLLER_H */
```

### File: `arm_controller.c`

```c
/**
 * @file    arm_controller.c
 * @brief   2-DOF Arm + Gripper + Gate Controller Implementation
 */

#include "arm_controller.h"
#include "pca9685.h"   /* PCA9685 HAL driver (I2C1, 0x40) */
#include <math.h>

/* ─── Internal helpers ──────────────────────────────────────────────────────── */

/**
 * @brief Convert angle (degrees) to PCA9685 12-bit tick value.
 *        MG90S range: 500µs (0°) to 2400µs (180°).
 *        At 50Hz, 12-bit: tick = pulse_us / (20000/4096) = pulse_us / 4.88
 */
static int16_t _AngleToTick(float angle_deg) {
    /* Clamp to 0–180° */
    if (angle_deg < 0.0f)   angle_deg = 0.0f;
    if (angle_deg > 180.0f) angle_deg = 180.0f;

    /* Linear interpolation: 0° → 102 ticks, 180° → 491 ticks */
    float tick = ARM_PWM_MIN + (angle_deg / 180.0f) * (ARM_PWM_MAX - ARM_PWM_MIN);
    return (int16_t)(tick + 0.5f);  /* Round to nearest tick */
}

/**
 * @brief Look up target PWM tick for arm position ID.
 */
static int16_t _ArmPositionToTick(uint8_t position) {
    switch (position) {
        case ARM_MODE_A: return ARM_TICK_MODEA;
        case ARM_MODE_B: return ARM_TICK_MODEB;
        case ARM_STORE:  return ARM_TICK_STORE;
        default:         return ARM_TICK_HOME;   /* ARM_HOME */
    }
}

/**
 * @brief  Slew-rate limiter. Moves current toward target by at most max_step per call.
 * @return New current value (may == target if within one step)
 */
static int16_t _Slew(int16_t current, int16_t target, int16_t max_step) {
    int16_t diff = target - current;
    if (diff >  max_step) return current + max_step;
    if (diff < -max_step) return current - max_step;
    return target;  /* Close enough — snap to target */
}

/* ─── Public Implementation ─────────────────────────────────────────────────── */

void ARM_Init(ARM_Controller_t *ctrl) {
    /* Start at safe positions */
    ctrl->target_arm     = ARM_HOME;
    ctrl->target_gripper = GRIPPER_OPEN;
    ctrl->target_gate    = GATE_LOCKED;

    /* Initialise current positions at target (no slew on boot) */
    ctrl->current_arm_tick  = ARM_TICK_HOME;
    ctrl->current_grip_tick = GRIP_TICK_OPEN;
    ctrl->current_gate_tick = GATE_TICK_LOCKED;

    ctrl->arm_at_target     = true;
    ctrl->gripper_at_target = true;
    ctrl->gate_at_target    = true;
    ctrl->motion_start_ms   = 0;

    /* Command initial positions to PCA9685 */
    PCA9685_SetPWM(ARM_CH_PITCH,   0, (uint16_t)ctrl->current_arm_tick);
    PCA9685_SetPWM(ARM_CH_GRIPPER, 0, (uint16_t)ctrl->current_grip_tick);
    PCA9685_SetPWM(ARM_CH_GATE,    0, (uint16_t)ctrl->current_gate_tick);

    HAL_Delay(800);  /* Allow servos to reach initial position before starting */
}

void ARM_Update(ARM_Controller_t *ctrl,
                uint8_t sm_arm, uint8_t sm_gripper, uint8_t sm_gate) {

    /* ── Arm Pitch (CH12) ─────────────────────────────────────────────────── */
    int16_t arm_target_tick = _ArmPositionToTick(sm_arm);
    ctrl->current_arm_tick  = _Slew(ctrl->current_arm_tick,
                                    arm_target_tick,
                                    ARM_SLEW_MAX_TICKS_PER_CYCLE);
    ctrl->arm_at_target = (ctrl->current_arm_tick == arm_target_tick);
    PCA9685_SetPWM(ARM_CH_PITCH, 0, (uint16_t)ctrl->current_arm_tick);

    /* ── Gripper (CH13) ───────────────────────────────────────────────────── */
    int16_t grip_target_tick = (sm_gripper == GRIPPER_CLOSE)
                                ? GRIP_TICK_CLOSE : GRIP_TICK_OPEN;
    ctrl->current_grip_tick  = _Slew(ctrl->current_grip_tick,
                                     grip_target_tick,
                                     GRIP_SLEW_MAX_TICKS_PER_CYCLE);
    ctrl->gripper_at_target = (ctrl->current_grip_tick == grip_target_tick);
    PCA9685_SetPWM(ARM_CH_GRIPPER, 0, (uint16_t)ctrl->current_grip_tick);

    /* ── Gate (CH14) ──────────────────────────────────────────────────────── */
    int16_t gate_target_tick = (sm_gate == GATE_OPEN)
                                ? GATE_TICK_OPEN : GATE_TICK_LOCKED;
    ctrl->current_gate_tick  = _Slew(ctrl->current_gate_tick,
                                     gate_target_tick,
                                     GATE_SLEW_MAX_TICKS_PER_CYCLE);
    ctrl->gate_at_target = (ctrl->current_gate_tick == gate_target_tick);
    PCA9685_SetPWM(ARM_CH_GATE, 0, (uint16_t)ctrl->current_gate_tick);
}

bool ARM_ArmAtTarget(const ARM_Controller_t *ctrl)     { return ctrl->arm_at_target; }
bool ARM_GripperAtTarget(const ARM_Controller_t *ctrl) { return ctrl->gripper_at_target; }
bool ARM_GateAtTarget(const ARM_Controller_t *ctrl)    { return ctrl->gate_at_target; }
bool ARM_AllAtTarget(const ARM_Controller_t *ctrl) {
    return ctrl->arm_at_target && ctrl->gripper_at_target && ctrl->gate_at_target;
}

void ARM_SetAngleDirect(uint8_t channel, float angle_deg) {
    int16_t tick = _AngleToTick(angle_deg);
    PCA9685_SetPWM(channel, 0, (uint16_t)tick);
}

void ARM_DeenergiseGate(void) {
    /* Set all 4096 ticks ON=0, OFF=4096 → servo de-energised (no PWM) */
    /* Only call when gate is confirmed LOCKED and detent holds position */
    PCA9685_SetPWM(ARM_CH_GATE, 0, 4096);  /* Full-off: de-energise */
}
```

---

## 7. Motion Sequencing & Timing

### Complete Ball Pickup Sequence

The state machine drives the arm through these states in sequence. Timings are from `state_machine.c`:

```
STATE_TASK1_BALL_APPROACH  (800ms):
  sm_arm_position = ARM_MODE_A  → ARM_Update() slews arm to 0°
  sm_gripper = GRIPPER_OPEN     → claw stays open
  sm_Vx = 0, sm_Wz = 0         → robot stops
  
  Slew time: |ARM_TICK_HOME − ARM_TICK_MODEA| / ARM_SLEW_MAX
           = |307 − 102| / 8 = 25.6 cycles × 20ms = 512ms
  → At 800ms timeout, arm has reached MODE_A with ~288ms spare ✅

STATE_TASK1_COLOR_ID  (up to 10s):
  sm_arm_position = ARM_MODE_A  → arm holds at 0°
  TCS34725_PollNonBlocking()    → 50ms integration, 3 stable readings
  Typical colour ID time: 3 × 60ms = 180ms (much less than 10s timeout) ✅
  On 3 stable colour readings → transition

STATE_TASK1_BALL_GRAB  (1200ms):
  sm_arm_position = ARM_MODE_A  → arm stays at 0°
  sm_gripper = GRIPPER_CLOSE    → claw closes on ball
  
  Slew time: |GRIP_TICK_OPEN − GRIP_TICK_CLOSE| / GRIP_SLEW_MAX
           = |184 − 286| / 12 = 8.5 cycles × 20ms = 170ms
  → At 1200ms timeout, gripper fully closed with 1030ms spare ✅
  (Generous timeout ensures full mechanical engagement with ball)

STATE_TASK1_STORE  (1000ms):
  sm_gripper = GRIPPER_CLOSE    → keep gripping while arm lifts
  sm_arm_position = ARM_HOME    → arm lifts ball to HOME (90°)
  sm_gate = GATE_LOCKED         → gate stays closed
  
  Slew time: |ARM_TICK_MODEA − ARM_TICK_HOME| / ARM_SLEW_MAX
           = |102 − 307| / 8 = 25.6 cycles × 20ms = 512ms
  
  ⚠ CURRENT GAP: State machine goes HOME then immediately exits to TASK1_GRID_EXIT.
    Ball is never deposited into compartment!
    → See Suggested Improvements (Section 9) for ARM_STORE fix.
```

### Complete Ball Release Sequence

```
STATE_TASK4_BALL_RELEASE  (1500ms):
  sm_Vx = 0, sm_Wz = 0      → robot stops at branch end
  sm_gate = GATE_OPEN        → gate opens, ball drops by gravity

  Slew time: |GATE_TICK_LOCKED − GATE_TICK_OPEN| / GATE_SLEW_MAX
           = |102 − 307| / 15 = 13.7 cycles × 20ms = 274ms
  → Ball drops within ~300ms of state entry ✅
  → 1500ms total gives 1.2s of ball-on-floor visibility (safe margin)

After 1500ms → STATE_FINISH
  sm_gate resets to GATE_LOCKED in FINISH handler (optional cleanup)
```

---

## 8. State Machine Integration

### Current State Machine Variables (from [`state_machine.h`](file:///c:/Users/ADMIN/Desktop/FusionForce-Robotics/firmware/Navigation/state_machine.h))

```c
/* These globals are set by StateMachine_Update() every 20ms */
extern float   sm_Vx;            /* Forward velocity (mm/s)          */
extern float   sm_Vy;            /* Lateral velocity (usually 0)     */
extern float   sm_Wz;            /* Angular velocity (rad/s)         */
extern uint8_t sm_arm_position;  /* 0=HOME 1=MODE_A 2=MODE_B         */
extern uint8_t sm_gripper;       /* 0=OPEN 1=CLOSE                   */
extern uint8_t sm_gate;          /* 0=LOCKED 1=OPEN                  */
```

### Integration in 50Hz Main Loop

```c
/* In main.c while(1): */
if (update_flag) {
    update_flag = 0;

    /* ── PERCEPTION ── */
    uint8_t line_bits   = LineArray_Read();
    float   line_error  = LineArray_GetCentroid(line_bits);
    bool    intersection = LineArray_IsIntersection(line_bits);
    TCS34725_PollNonBlocking(&htcs, &last_color);
    /* ... ToF, IMU reads ... */

    /* ── DECISION ── */
    SM_SensorData_t s;
    s.line_bits     = line_bits;
    s.line_centroid = line_error;
    s.intersection  = intersection;
    s.last_color    = last_color;
    /* ... fill ToF, IMU, battery ... */
    StateMachine_Update(&s);

    /* ── ARM CONTROL ← NEW: call ARM_Update() every cycle ── */
    ARM_Update(&arm_ctrl, sm_arm_position, sm_gripper, sm_gate);

    /* ── LOCOMOTION ── */
    GaitGenerator_Update(sm_Vx, sm_Vy, sm_Wz, foot_targets);
    Kinematics_SolveAll(foot_targets, joint_angles);
    PCA9685_WriteLegs(joint_angles);   /* Writes CH0–CH11 */
    /* ARM_Update() already wrote CH12–CH14 above */
}
```

### State → Arm Command Summary

| State | `sm_arm_position` | `sm_gripper` | `sm_gate` | Duration |
|-------|-----------------|-------------|---------|----------|
| `TASK1_LINE_FOLLOW` | HOME | OPEN | LOCKED | Continuous |
| `TASK1_BALL_APPROACH` | MODE_A | OPEN | LOCKED | 800ms |
| `TASK1_COLOR_ID` | MODE_A | OPEN | LOCKED | Up to 10s |
| `TASK1_BALL_GRAB` | MODE_A | **CLOSE** | LOCKED | 1200ms |
| `TASK1_STORE` | **HOME** | CLOSE | LOCKED | 1000ms |
| *(Suggested: TASK1_ARM_SWEEP)* | *(STORE)* | *(OPEN)* | LOCKED | *(600ms)* |
| `TASK1_GRID_EXIT` | HOME | OPEN | LOCKED | Continuous |
| `TASK2_WALL_FOLLOW` | HOME | OPEN | LOCKED | Continuous |
| `TASK3_*` | HOME | OPEN | LOCKED | Continuous |
| `TASK4_LINE_FOLLOW` | HOME | OPEN | LOCKED | Continuous |
| `TASK4_JUNCTION_DETECT` | **MODE_B** | OPEN | LOCKED | Up to 7s |
| `TASK4_BRANCH_FOLLOW` | HOME | OPEN | LOCKED | Continuous |
| `TASK4_BALL_RELEASE` | HOME | OPEN | **OPEN** | 1500ms |
| `FINISH` | HOME | OPEN | LOCKED | Permanent |

---

## 9. Suggested Improvements

### 9.1 ⭐ Critical Fix: ARM_STORE Position for Ball Deposit

**Problem**: The current state machine has no mechanism to physically move the ball from the gripper into the storage compartment. After `TASK1_BALL_GRAB`, the arm returns HOME (90°, pointing upward) while the gripper holds the ball — the ball is stuck in the air above the body.

**Solution**: Add `ARM_STORE` (~135°, pointing backward over compartment mouth) and a new state:

```c
/* Add to state_machine.h: */
#define ARM_STORE   3   /* ~135° — swept backward over compartment top */

/* Add new state: */
STATE_TASK1_ARM_SWEEP,  /* After TASK1_STORE, before TASK1_GRID_EXIT */

/* In state_machine.c: */
case STATE_TASK1_STORE:
    sm_Vx = 0; sm_Wz = 0;
    sm_gripper = GRIPPER_CLOSE;
    sm_arm_position = ARM_HOME;    /* First: lift arm upward */
    sm_gate = GATE_LOCKED;
    if (state_timer_ms >= 800) {   /* Wait for lift */
        StateMachine_Transition(STATE_TASK1_ARM_SWEEP);
    }
    break;

case STATE_TASK1_ARM_SWEEP:
    sm_Vx = 0; sm_Wz = 0;
    sm_arm_position = ARM_STORE;   /* Sweep arm backward over compartment */
    if (state_timer_ms >= 400) {
        sm_gripper = GRIPPER_OPEN; /* Open gripper → ball drops into compartment */
    }
    if (state_timer_ms >= 800) {
        sm_arm_position = ARM_HOME; /* Return arm to travel position */
    }
    if (state_timer_ms >= 1200) {
        StateMachine_Transition(STATE_TASK1_GRID_EXIT);
    }
    break;
```

**Required mechanical verification**: Arm sweep to 135° must clear body top plate. The compartment mouth must be directly below the 135° arm endpoint. Design the compartment mouth as a **funnel** (tapered top) to guide ball even with ±10mm drop offset.

### 9.2 ⭐ Slew-Rate Confirmation for State Transitions

**Problem**: The current state machine uses fixed timers (e.g., 800ms for BALL_APPROACH) regardless of whether the servo has actually reached position.

**Solution**: Add `ARM_ArmAtTarget()` checks to confirm servo position before transitioning:

```c
/* In STATE_TASK1_BALL_APPROACH: */
case STATE_TASK1_BALL_APPROACH:
    sm_Vx = 0; sm_Wz = 0;
    sm_arm_position = ARM_MODE_A;
    /* Wait for arm to ACTUALLY reach MODE_A (slew confirmed) */
    if (ARM_ArmAtTarget(&arm_ctrl) && state_timer_ms >= 200) {
        /* 200ms min for robot to fully stop oscillating */
        StateMachine_Transition(STATE_TASK1_COLOR_ID);
    }
    /* Timeout failsafe: 1.5s max */
    if (state_timer_ms >= 1500) {
        StateMachine_Transition(STATE_TASK1_COLOR_ID);
    }
    break;
```

### 9.3 Gate Detent + De-energisation

**Problem**: Gate servo (CH14) holds tension continuously during Task 2+3 navigation (~5–8 minutes). This causes:
- ~200mA continuous current draw
- Servo heating (possible gear wear)

**Solution A — Mechanical detent**:
- Add a small PETG bump on the gate plate that snaps over a retaining tab when fully closed
- After gate reaches LOCKED, call `ARM_DeenergiseGate()` to cut PWM
- Gate stays locked mechanically; servo only needed to open it

**Solution B — Passive gate latch**:
- Spring-loaded latch mechanism
- Gate servo only needed to release the latch
- Zero holding power required

**Recommended**: Solution A is simpler to implement in PETG 3D print.

### 9.4 Gripper Compliance for Ball Shape Variation

**Problem**: 40mm balls may vary in size (±2mm) and surface hardness. Hard close position can:
- Crush foam balls
- Fail to grip smooth hard balls (slip out)

**Solution**: Add **two gripper close levels**:
```c
#define GRIPPER_CLOSE_SOFT  0x02  /* Gentle grip — for foam balls */
#define GRIPPER_CLOSE_FIRM  0x03  /* Firm grip   — for hard rubber balls */

/* Map to different PWM ticks */
int16_t grip_target_tick;
switch (sm_gripper) {
    case GRIPPER_CLOSE_SOFT: grip_target_tick = GRIP_TICK_CLOSE_SOFT; break; /* ~110° */
    case GRIPPER_CLOSE_FIRM: grip_target_tick = GRIP_TICK_CLOSE_FIRM; break; /* ~125° */
    default:                 grip_target_tick = GRIP_TICK_OPEN;       break;
}
```

**Calibration**: Start with soft grip; if ball drops during arm lift → use firm grip.

### 9.5 Anti-Backdrive on Arm Pitch

**Problem**: If CH12 is de-energised during travel (to save power), gravity can back-drive the arm down from HOME (90°) toward MODE_A (0°), especially during aggressive gait movements.

**Solutions** (choose one):
| Option | Cost | Reliability |
|--------|------|-------------|
| Keep CH12 energised at all times (current approach) | ~200mA draw | ✅ Simple |
| Use DS3218MG servo (has metal gear + stronger holding) | ~40g servo | ✅ Better holding |
| Add a locking worm gear between servo and arm pivot | Complex 3D print | ✅ Zero backdrive |
| Software: re-command HOME every 5s during navigation | Zero cost | ✅ Sufficient |

**Recommended**: Re-command HOME every 5s (simplest software fix):
```c
/* In TASK2/TASK3 navigation states: */
if (state_timer_ms % 5000 < 20) {   /* Every 5 seconds */
    sm_arm_position = ARM_HOME;      /* Re-assert HOME */
}
```

### 9.6 Funnel-Guided Compartment Top

**Problem**: When gripper opens over compartment at ARM_STORE position, ball may miss the opening (±15mm lateral offset possible).

**Solution**: Design compartment top as a **50×50mm funnel** that tapers to the 50mm diameter compartment:
```
Top opening: 70mm × 70mm (catching funnel)
   ↓ tapered sides (30° angle)
Bottom bore: 50mm diameter
   ↓
Ball enters and settles inside
```
3D print funnel as integral to body top plate. PETG, 25% infill.

### 9.7 Ball Detection Confirmation via ToF Front

**Problem**: Robot may approach intersection without a ball on the pedestal and mistakenly try to grab air if line+ToF logic triggers incorrectly.

**Solution**: Verify ball presence via front ToF **and** colour sensor:
```c
/* In TASK1_BALL_APPROACH: */
/* Only proceed to COLOR_ID if front ToF AND colour sensor both confirm ball */
if (ARM_ArmAtTarget(&arm_ctrl) &&
    s->tof_front_mm < 60 &&        /* Very close — ball on pedestal */
    s->last_color != COLOR_UNKNOWN) /* Sensor sees a coloured object */
{
    StateMachine_Transition(STATE_TASK1_COLOR_ID);
}
```

### 9.8 Weight Optimisation

**Current mechanism weight estimate:**

| Component | Est. Weight |
|-----------|------------|
| 3× MG90S servos (CH12/13/14) | 40g |
| Arm link (80mm PETG) | ~8g |
| Gripper assembly (PETG) | ~12g |
| Compartment body (PETG) | ~15g |
| Gate plate (PETG) | ~4g |
| TCS34725 + shroud | ~6g |
| Wiring (3× servo) | ~5g |
| **Total mechanism** | **~90g** |

**Target**: ≤80g (10g savings available via hollow arm, reduced infill on gate).

---

## 10. Testing Procedure

### Phase 1: Individual Servo Calibration

| Test | Method | Pass Criteria |
|------|--------|--------------|
| T1.1 Arm HOME | Command ARM_TICK_HOME via debug UART | Arm points vertically up (or safe travel position) |
| T1.2 Arm MODE_A | Command ARM_TICK_MODEA | Arm horizontal, tip at 50mm above floor |
| T1.3 Arm MODE_B | Command ARM_TICK_MODEB | Arm points at floor, tip 1–3cm from arena surface |
| T1.4 Arm STORE | Command ARM_TICK_STORE | Arm sweeps back over compartment mouth |
| T1.5 Gripper OPEN | Command GRIP_TICK_OPEN | Claw gap ≥42mm |
| T1.6 Gripper CLOSE | Command GRIP_TICK_CLOSE | Claw closes on 40mm ball; no excessive force |
| T1.7 Gate LOCKED | Command GATE_TICK_LOCKED | Trapdoor fully closed; ball stays when placed |
| T1.8 Gate OPEN | Command GATE_TICK_OPEN | Trapdoor opens; ball drops freely by gravity |

### Phase 2: Sequence Testing (No Robot Walking)

| Test | Method | Pass Criteria |
|------|--------|--------------|
| T2.1 Full pickup | Manual trigger BALL_APPROACH→GRAB→STORE | Ball ends up inside compartment |
| T2.2 Slew smoothness | Watch ARM_Update() on debug UART | No tick jumps; smooth linear motion |
| T2.3 Arm-at-target | Print `ARM_ArmAtTarget()` during motion | False during motion, True when settled |
| T2.4 Ball hold test | Grab ball, run for 60 seconds walking | Ball stays in gripper/compartment |
| T2.5 Ball release | Trigger TASK4_BALL_RELEASE state | Ball drops within 300ms of gate command |

### Phase 3: Integration Testing

| Test | Method | Pass Criteria |
|------|--------|--------------|
| T3.1 Full Task 1 | Arena pedestal, real ball | Robot grabs, stores, exits grid correctly |
| T3.2 Colour ID + grab | Arena with RED/GREEN/BLUE balls | Correct colour stored in Flash; correct branch |
| T3.3 Vibration hold | Walk full Task 2+3 corridor with ball | Ball never falls out of compartment |
| T3.4 Full Task 4 | Arena release zone | Ball drops onto correct branch floor |
| T3.5 Competition restart | Reset STM32 mid-run | Ball colour recalled from Flash; arm returns HOME |

---

## 11. Tuning Guide

### Arm Angle Calibration Procedure

```
1. Power servos at exactly 6.0V (use BEC, not USB power)
2. Connect debug UART (USART1, 115200 baud, PA9/PA10)
3. Command ARM_SetAngleDirect(12, 90.0) → verify arm is at HOME position
4. Command ARM_SetAngleDirect(12, 0.0)  → verify arm is horizontal (MODE_A)
   → If arm hits bumper: increase ARM_TICK_MODEA until arm clears
5. Hold 40mm ball at pedestal height (50mm above floor)
   → Arm tip should be 1–2cm from ball centre → adjust ARM_TICK_MODEA
6. Command ARM_SetAngleDirect(12, 160.0) → verify arm points at floor
   → Sensor should read floor at 1–3cm range in MODE_B
7. Record final tick values → update ARM_TICK_* constants in firmware
```

### Gripper Gap Measurement

```
1. Command GRIP_TICK_OPEN → measure claw gap with calipers
   → Must be ≥42mm (2mm clearance on each side of 40mm ball)
   → If too narrow: decrease GRIP_TICK_OPEN value (open more)
2. Place 40mm ball between claws → Command GRIP_TICK_CLOSE
   → Ball must be held securely; claw must not skip over ball surface
   → If slipping: increase GRIP_TICK_CLOSE (close more)
   → If crushing: decrease GRIP_TICK_CLOSE
3. Hold robot upside down with ball gripped → ball must not fall
4. Shake robot laterally → ball must not rattle in claw
```

### Slew Rate Tuning

```
ARM_SLEW_MAX_TICKS_PER_CYCLE = 8   (current)
→ Total arm travel time: ~512ms (HOME↔MODE_A)
→ If too slow (arm lags): increase to 10–12
→ If too jerky (robot tips): decrease to 5–6

GRIP_SLEW_MAX_TICKS_PER_CYCLE = 12 (current)
→ Gripper close time: ~170ms
→ If ball slips before fully gripped: decrease to 8 (slower, more controlled)

GATE_SLEW_MAX_TICKS_PER_CYCLE = 15 (current)
→ Gate open time: ~274ms
→ If ball jams on partial gate: increase to 20 (faster sweep)
```

---

## 12. Failure Modes & Mitigations

| Failure | Detection | Mitigation |
|---------|-----------|-----------|
| **Ball missed on grab** | Gripper closes but ball not inside (no weight feedback) | Add 2-attempt retry: re-open gripper, re-close; if still fails, abort Task 1 (rescue mode) |
| **Arm servo stall** | Robot tilts, arm doesn't reach target within 2s | Detect via `ARM_ArmAtTarget()` timeout; command HOME and stop |
| **Ball falls from gripper during travel** | No direct detection | Store ball in compartment ASAP (fix ARM_STORE issue); gate then holds it securely |
| **Gate won't open** | `ARM_GateAtTarget()` stays false after 2s | Retry gate command; if stuck, vibrate (rapid LOCKED↔OPEN oscillation for 500ms) |
| **Colour misclassification** | Wrong branch selected in Task 4 | Require 3 consecutive same-colour readings; increase stable_cnt to 5 |
| **Ball drops on wrong branch** | — | Flash persistence: stored_ball_color persists → competition restart recovery |
| **Arm hits bumper** | Mechanical collision sound; arm servo stall | Increase ARM_TICK_MODEA; add physical arm travel stop |
| **Gripper servo overheats** | 12+ minutes gripping | Use soft grip level (reduce torque); de-energise between tasks |
| **Compartment ball jam** | Ball gets stuck at compartment top | Funnel top (Section 9.6); shake body via gait |

---

## 13. Quick Reference Card

```
┌──────────────────────────────────────────────────────────────────────────┐
│         2-DOF ARM + GRIPPER + GATE SYSTEM — STM32 QUICK REFERENCE       │
│             FusionForce RUNNER-4 | EN2533 BREACH PROTOCOL               │
├──────────────────────────────────────────────────────────────────────────┤
│ CH12 Arm Pitch (MG90S @ 6V):                                            │
│   HOME   = ~90°  (307 ticks) — safe travel/resting                     │
│   MODE_A = ~0°   (102 ticks) — horizontal to ball pedestal             │
│   MODE_B = ~160° (450 ticks) — pointing at floor for zone read         │
│   STORE  = ~135° (380 ticks) — over body compartment mouth             │
├──────────────────────────────────────────────────────────────────────────┤
│ CH13 Gripper (MG90S @ 6V):                                              │
│   OPEN   = ~60°  (184 ticks) — gap ≥42mm (clears 40mm ball)           │
│   CLOSE  = ~115° (286 ticks) — secure grip on ball                     │
│   Slew: 12 ticks/cycle → close time ~170ms                             │
├──────────────────────────────────────────────────────────────────────────┤
│ CH14 Gate (MG90S @ 6V):                                                 │
│   LOCKED = ~0°  (102 ticks) — gate closed; ball inside                 │
│   OPEN   = ~90° (307 ticks) — gate open; ball drops by gravity         │
│   Slew: 15 ticks/cycle → open time ~274ms                              │
├──────────────────────────────────────────────────────────────────────────┤
│ State Timings (current firmware):                                        │
│   BALL_APPROACH: 800ms (arm to MODE_A)                                 │
│   COLOR_ID:      up to 10s (3 stable colour readings)                  │
│   BALL_GRAB:     1200ms (gripper close)                                 │
│   STORE:         1000ms (arm lift to HOME)                             │
│   BALL_RELEASE:  1500ms (gate open)                                    │
├──────────────────────────────────────────────────────────────────────────┤
│ ⚠ CRITICAL OPEN ISSUE:                                                  │
│   Ball is never deposited into compartment in current state machine!    │
│   Add ARM_STORE position + STATE_TASK1_ARM_SWEEP state (Section 9.1)   │
├──────────────────────────────────────────────────────────────────────────┤
│ PCA9685: I2C1 (PB6 SCL / PB7 SDA), addr 0x40, 50Hz                    │
│ PWM ticks: 102 (0°) → 307 (90°) → 491 (180°)  [4.88µs/tick]          │
│ All values are APPROXIMATE — calibrate on physical hardware!            │
└──────────────────────────────────────────────────────────────────────────┘
```

---

*Report generated: September 2026 | FusionForce Robotics | RUNNER-4 Project*
*MCU: STM32F411CEU6 | 2-DOF Arm + Gripper + Storage Gate System*
*Key driver files: [`arm_controller.h`](file:///c:/Users/ADMIN/Desktop/FusionForce-Robotics/firmware/Drivers/) | [`state_machine.c`](file:///c:/Users/ADMIN/Desktop/FusionForce-Robotics/firmware/Navigation/state_machine.c)*
