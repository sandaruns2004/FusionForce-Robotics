# 01 Chassis Design
## RUNNER-4 | Body Frame and 3D Printing Guide

## Objective
Design and 3D-print a rigid, perfectly symmetrical chassis that keeps the Center of Gravity (CoG) within the quadruped's support polygon at all times.

## Body Dimensions

```
Top view:

  FL coxa mount ----+--------+---- FR coxa mount
                    |        |
                    | BODY   |
                    | 160mm  |
                    | x      |
                    | 120mm  |
                    |        |
  BL coxa mount ----+--------+---- BR coxa mount

  Body half-length (front/rear to centre): 80 mm
  Body half-width  (left/right to centre): 60 mm
  CoG target: geometric centre of body plate
```

## 3D Printing Specifications

| Part | Material | Infill | Perimeters | Notes |
|------|----------|--------|------------|-------|
| Base plate (bottom deck) | PETG | 40% Gyroid | 4 | Battery + BEC mounting surface |
| Top plate (upper deck) | PETG | 30% Gyroid | 4 | STM32 + PCA9685 mounting surface |
| Coxa servo bracket (x4) | PETG | 100% | 5 | Highest stress — no PLA |
| Femur link (x4) | PETG | 50% Gyroid | 4 | |
| Tibia link (x4) | PETG | 50% Gyroid | 4 | Rubber foot socket at tip |
| Arm mount bracket | PETG | 100% | 4 | CH12 arm pitch servo mount |
| Gripper fingers (x2) | PETG/TPU | 80% | 3 | TPU preferred — better grip |
| Gate plate | PETG | 60% | 3 | CH14 gate servo horn mount |

> **Do NOT use PLA** for structural parts. PLA deforms under continuous servo heat (>55°C body temperature) and shatters on arena collision impacts.

## Slicer Settings (Cura / PrusaSlicer)

```
Layer height:     0.2mm standard | 0.15mm for servo brackets
Print speed:      40mm/s perimeters | 60mm/s infill
Bed temp:         80°C (PETG)
Nozzle temp:      240°C (PETG)
Supports:         Only for overhangs > 50°
Cooling:          Moderate (PETG warps with aggressive cooling)
```

## Center of Gravity Planning

The quadruped's support polygon during trot gait is the diagonal between FL+BR or FR+BL. The CoG must stay inside this triangle at all times:

```
  CoG position targets:
  - Battery (heaviest, ~200g): centre of bottom deck, LOW
  - PCA9685 + STM32: middle deck, centred front-back
  - Sensors (ToF/IMU): front of top deck, symmetric

  AVOID: mounting heavy components at back edge
  (shifts CoG backward -> front legs lose traction on acceleration)
```

## Assembly Tasks

1. Export all STLs from the `3D designs/` folder (SolidWorks assemblies)
2. Slice with PETG profiles above
3. Install M3 brass heat-set inserts into all screw holes (use soldering iron tip at 200°C)
4. Print 4x coxa brackets — test-fit servo before installing inserts
5. Assemble base plate + standoffs for middle deck
6. Assemble top deck plate

## Files Location
- CAD sources: [`3D designs/`](../3D%20designs/)
- Reference assembly: `3D designs/` SolidWorks files

---
🔙 **[Back to Mechanical](../README.md)**
