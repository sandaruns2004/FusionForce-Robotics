# 02 Leg Assembly
## RUNNER-4 | 12x MG90S Metal-Gear Servos | 3-DOF per Leg

## Objective
Assemble all four 3-DOF legs using MG90S metal-gear micro-servos, with correct link lengths and PCA9685 channel assignments.

## Kinematic Structure (per leg)

```
   BODY
    |
    | 30mm  COXA  — J1 yaw:   horizontal forward/backward swing ±45°
    |               (PCA9685: CH0=FL, CH3=FR, CH6=BL, CH9=BR)
    ●  coxa pivot
    |
    | 60mm  FEMUR — J2 pitch: vertical lift/lower ±90°
    |               (PCA9685: CH1=FL, CH4=FR, CH7=BL, CH10=BR)
    ●  femur pivot
    |
    | 80mm  TIBIA — J3 pitch: knee extension ±90°
    |               (PCA9685: CH2=FL, CH5=FR, CH8=BL, CH11=BR)
    ●
   FOOT  (rubber tip contacts ground)

   Max reach from coxa pivot: 60 + 80 = 140 mm
   Min reach from coxa pivot: |60 - 80| = 20 mm
   Nominal foot depth (z):    50 mm below body centre
```

## Hardware Required Per Leg

| Part | Qty per leg | Total (4 legs) | Notes |
|------|------------|----------------|-------|
| MG90S metal-gear servo | 3 | **12** | Metal gears mandatory — plastic strips |
| Coxa bracket (PETG, 100% infill) | 1 | 4 | Servo body screwed inside |
| Femur link (PETG) | 1 | 4 | 60mm L2 |
| Tibia link (PETG) | 1 | 4 | 80mm L3, rubber foot socket at tip |
| MR83ZZ radial bearing | 1 | 4 | Opposite side of coxa servo shaft |
| M3×8 socket head | 4 | 16 | Servo to bracket |
| M3×12 socket head | 2 | 8 | Femur pivot bolt |
| M3 nylon locknut | 6 | 24 | All pivot joints |
| Silicone rubber foot cap | 1 | 4 | Tibia tip, prevents slipping |

## MG90S Servo Specifications

```
Torque:    2.2 kg·cm @ 6V  (metal gears)
Speed:     0.08 sec/60° @ 6V
Pulse:     500µs (0°) to 2400µs (180°)
Dead band: 5µs
Weight:    13.4g
```

> **Power:** Servos MUST be powered from the 6V switching BEC rail, NOT from STM32 or PCA9685 logic power. Connecting servo V+ to 3.3V logic will instantly damage the STM32.

## PCA9685 Channel Mapping

| CH | Leg | Joint | Neutral | Dir at 0° | Dir at 180° |
|----|-----|-------|---------|-----------|-------------|
| 0 | FL | Coxa J1 | 90° | Forward sweep | Backward |
| 1 | FL | Femur J2 | 45° | Leg down | Leg up |
| 2 | FL | Tibia J3 | 135° | Tibia folded | Tibia extended |
| 3 | FR | Coxa J1 | 90° | Backward* | Forward* |
| 4 | FR | Femur J2 | 135° | Leg up* | Leg down* |
| 5 | FR | Tibia J3 | 45° | Extended* | Folded* |
| 6 | BL | Coxa J1 | 90° | Forward sweep | Backward |
| 7 | BL | Femur J2 | 45° | Leg down | Leg up |
| 8 | BL | Tibia J3 | 135° | Tibia folded | Extended |
| 9 | BR | Coxa J1 | 90° | Backward* | Forward* |
| 10 | BR | Femur J2 | 135° | Leg up* | Leg down* |
| 11 | BR | Tibia J3 | 45° | Extended* | Folded* |

> `*` = Right-side legs (FR, BR) are physically mirrored. FR/BR femur and tibia angles = `180° − left_side_angle`.

## Assembly Steps

> **⚠️ CRITICAL: DO NOT attach servo horns until Module 06 (Zeroing). Attach horns only while servos are powered and holding 90°.**

1. Assemble coxa bracket — press MR83ZZ bearing into the bearing seat on the far side
2. Insert MG90S into coxa bracket, secure with M3×8 screws (do not overtighten — servo casing is aluminium)
3. Pivot-connect femur link to coxa bracket using M3×12 bolt + nylon locknut — should rotate freely with no stiffness
4. Pivot-connect tibia link to femur link — same M3×12 + locknut
5. Press silicone rubber cap onto tibia tip
6. Route servo cable through the coxa bracket cable channel, zip-tie to prevent snagging during leg swing

## Debugging

| Problem | Cause | Fix |
|---------|-------|-----|
| Joint stiff when moved by hand (servo off) | Bracket too tight / misaligned | Loosen locknut until free rotation |
| Servo hums loudly at neutral | Mechanical binding fighting servo | Check all pivot points for friction |
| Leg clicks under load | Stripped plastic gear | Replace servo — use metal gear only |
| Leg swings but no ground contact | Tibia too short / link length wrong | Verify L2=60mm L3=80mm in printed parts |
| BR leg not moving | PCA9685 channel mis-wired | Verify CH9/10/11 connectors |

---
🔙 **[Back to Mechanical](../README.md)**
