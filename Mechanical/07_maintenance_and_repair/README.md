# 07 Maintenance & Repair
## RUNNER-4 | Pre-Run Checklists + Common Hardware Failures

## Objective
Diagnose physical hardware failures quickly during testing, and ensure the robot is competition-ready before every arena run.

## Common Hardware Failures

### 1. Stripped MG90S Servo Gears

| Field | Detail |
|-------|--------|
| **Symptom** | Leg clicks loudly under load, sags when commanded, moves freely when powered off |
| **Cause** | Impact with arena wall, leg getting stuck, or sustained stall (gait programming error) |
| **Diagnosis** | Hold servo body, gently rotate output shaft by hand — feel for grinding/loose play |
| **Fix** | Replace entire servo (gear sets for MG90S are available but time-consuming to swap under competition pressure) |
| **Prevention** | Add soft limits in firmware: clamp all joint angles to [10°, 170°] to prevent stall at extremes |

### 2. Loose Leg Linkages

| Field | Detail |
|-------|--------|
| **Symptom** | Robot wobbles, foot placement inaccurate by 10–20mm, IK appears correct but gait drifts |
| **Cause** | M3 locknuts vibrating loose over time (arena floor vibration + servo shock loads) |
| **Diagnosis** | Grab each femur/tibia link and wiggle — there should be zero rotational play |
| **Fix** | Apply one drop of **Blue Loctite 243** to each metal-to-metal pivot screw. Re-tighten locknut while Loctite is wet. Wait 30min before powering servos. **Do NOT use Loctite on plastic holes** — it dissolves PETG. |

### 3. Servo Jitter / Twitching at Neutral

| Field | Detail |
|-------|--------|
| **Symptom** | Servos vibrate or twitch rapidly when robot is standing still (Vx=0, Wz=0) |
| **Cause A** | Ground loop between BEC GND, STM32 GND, and PCA9685 GND — star ground not connected |
| **Cause B** | Servo rail voltage ripple — 470µF capacitor missing from 6V BEC rail |
| **Cause C** | I2C pull-up resistors missing — PCA9685 receiving corrupted commands |
| **Fix** | Check star ground (Module 05). Add 470µF electrolytic cap on servo rail. Verify 4.7kΩ pull-ups on I2C1. |

### 4. I2C Bus Failure (sensor offline)

| Field | Detail |
|-------|--------|
| **Symptom** | UART shows sensor not found, state machine enters SAFE_STOP |
| **Cause** | Loose DuPont connector on SDA/SCL, or missing pull-up resistor |
| **Diagnosis** | `tof_scan\r\n` returns FAIL for one sensor. I2C bus ACK failure on STM32 HAL. |
| **Fix** | Hot-glue all I2C connectors. Verify 4.7kΩ pull-up to 3.3V on both I2C1 and I2C2. Verify XSHUT pins (PB12/13/14) configured as GPIO Output in CubeMX. |

### 5. Line Array Reads All White or All Black

| Field | Detail |
|-------|--------|
| **Symptom** | `la_test\r\n` returns all sensors > 0.9 or all < 0.1 regardless of surface |
| **Cause A** | Calibration data lost (Flash not programmed) |
| **Cause B** | Sensor PCB height wrong (not at 5mm from floor) |
| **Cause C** | ADC DMA not started (`LA_Init()` not called in `App_Init()`) |
| **Fix** | Re-run `la_cal white` then `la_cal black`. Verify sensor bar height. Check `HAL_ADC_Start_DMA()` call. |

### 6. Robot Walks in Circles / Drifts

| Field | Detail |
|-------|--------|
| **Symptom** | Robot veers left or right on straight Vx command |
| **Cause** | Servo trim offsets not calibrated — one side of legs has systematic angle error |
| **Fix** | Re-do Module 06 calibration. Fill `calibration.h` with measured offsets for all 12 leg channels. |

### 7. Arm Drops Under Ball Weight

| Field | Detail |
|-------|--------|
| **Symptom** | CH12 arm cannot hold horizontal position with ball in gripper |
| **Cause** | MG90S torque margin at 0.94x — marginal at full arm extension |
| **Fix Option A** | Shorten arm link from 65mm to 50mm to reduce torque requirement |
| **Fix Option B** | Upgrade CH12 to MG996R (10 kg·cm) for 4.3x margin |

## Pre-Run Competition Checklist

Run this checklist before placing robot on the starting line:

### Mechanical
```
[ ] All 12 leg locknuts checked for tightness (attempt to rotate by hand — zero play)
[ ] Rubber feet on all 4 tibias — not worn through
[ ] Arm link M2 screws tight — TCS34725 sensor firmly mounted
[ ] Gripper fingers intact — foam/rubber lining present
[ ] Gate plate swings freely — hinge not binding
[ ] No servo horn screws missing
```

### Electrical
```
[ ] Battery voltage: > 7.4V (2S) or > 11.1V (3S)
[ ] Kill switch functional — cycle ON/OFF
[ ] 6V BEC output: 6.0V ± 0.1V (measure with multimeter)
[ ] 3.3V logic rail: 3.30V ± 0.05V
[ ] All I2C connector joints hot-glued or zip-tied
[ ] No bare wire shorts visible
```

### Firmware & Sensors
```
[ ] UART connect: SM boots to state 0→1→2 correctly
[ ] tof_scan -> FRONT OK, LEFT OK, RIGHT OK
[ ] imu_read -> |pitch| < 3° and |roll| < 3° on flat surface
[ ] la_test  -> sensors respond correctly to white and black
[ ] servo_all 90 -> all 15 servos reach neutral without jitter
[ ] Press start button (PC13) -> SM transitions to T1_LINE_FOLLOW (state 3)
[ ] Vx=0 Wz=0 command -> robot stands level, all 4 feet grounded evenly
```

### Arena Walk Test (do 2 minutes before competition run)
```
[ ] Walk forward 1m at Vx=60 mm/s -> straight, no drift
[ ] Arm cycle: HOME -> MODE_A -> CLOSE -> HOME
[ ] Gate cycle: LOCKED -> OPEN -> LOCKED
[ ] Colour sensor test: red/green/blue ball under competition lighting
[ ] ToF corridor test: wall-follow maintains 150mm ± 20mm
```

## Quick-Swap Spares to Carry

| Item | Qty to carry | Why |
|------|-------------|-----|
| MG90S servo (spare) | 3 | Gear strip under stall |
| M3×12 locknuts | 10 | Vibrate loose |
| Servo horn screws M2.5 | 10 | Fall out during impact |
| Blue Loctite 243 | 1 tube | Preventive application |
| DuPont 3-pin servo extension | 3 | Connector failure |
| USB to UART (CH340) | 1 | Debug during arena |

---
🔙 **[Back to Mechanical](../README.md)**
