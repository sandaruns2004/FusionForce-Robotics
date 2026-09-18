# RUNNER-4 — Engineering Decisions & Implementation Status
## FusionForce | EN2533 BREACH PROTOCOL

> This document records all resolved design decisions, confirmed hardware, and current implementation status. All inconsistencies from early planning have been resolved.

---

## ✅ Resolved Design Decisions

All conflicts from early repository versions have been resolved and finalized:

| Decision | Resolved Value | Notes |
|----------|---------------|-------|
| **MCU** | **STM32F411CEU6** Black Pill | 100MHz, Cortex-M4F, HW FPU, 512KB Flash, 128KB RAM |
| **Raspberry Pi** | **Removed** | Single STM32 handles all perception + logic + motion |
| **Computer Vision** | **Removed** | Replaced by TCS34725 (colour) + TCRT5000 IR (line) |
| **Leg Servos** | **15× MG90S metal-gear** | 2.2 kg·cm @ 6V; plastic gear (SG90/DS3218) rejected |
| **Coxa L1** | **30 mm** | Confirmed hardware dimension |
| **Femur L2** | **60 mm** | Confirmed hardware dimension |
| **Tibia L3** | **80 mm** | Confirmed hardware dimension |
| **Max reach** | **140 mm** (L2+L3) | Min reach 20mm |
| **Gait** | **Diagonal Trot** | T=600ms, step=35mm, duty=50%; crawl gait rejected (Bezier) |
| **Battery** | **2S LiPo 7.4V** | ≥1300mAh, ≥25C; 6V BEC for servos, 3.3V LDO for logic |
| **Line sensor** | **9× TCRT5000 analogue** | ADC1+DMA1, 80mm/10mm pitch, 16× oversample; 8-ch digital rejected |
| **Arm MODE_B** | **160°** (down to floor) | Replaces -70° from early docs; CH12 tick=450 |
| **State count** | **21 states** | Replaces 18 from early docs (added ERROR_RECOVERY + 2 task sub-states) |
| **Start button** | **PC13** | Active LOW, internal pull-up; replaces PA0/Boot0 (conflict avoided) |

---

## 📦 Implementation Status

### ✅ Documentation — COMPLETE
- `docs/` — 17 engineering documents, all updated to final hardware specs
- `Mechanical/` — 7 assembly guides (chassis → calibration → maintenance)
- `Robot_Curriculum/` — 4 levels, L1 Fundamentals → L4 Sensors & Integration (CV removed)
- `README.md` — Root project README with architecture diagram and spec tables
- `spider-robot-roadmap.md` — 7-phase implementation roadmap

### 🔧 Mechanical — In Progress
- [ ] PETG parts printed and assembled
- [ ] Leg link lengths verified: L1=30mm L2=60mm L3=80mm
- [ ] 15-servo zeroing (Module 06) completed

### 💻 Firmware — Pending
- [ ] STM32CubeMX project: 100MHz, I2C1/2, ADC1 DMA, TIM2/3, USART1
- [ ] PCA9685 driver (50Hz, angle→tick, `calibration.h`)
- [ ] VL53L0X driver (XSHUT remap, EMA filter)
- [ ] MPU6050 driver (complementary filter, 100Hz ISR)
- [ ] TCS34725 driver (non-blocking, ratio classification)
- [ ] Line array driver (ADC DMA, EMA, centroid, junction)
- [ ] IK solver (`IK_L1=30 L2=60 L3=80`, right-side mirroring)
- [ ] Trot gait engine (diagonal pairs, parabolic arc, velocity feedforward)
- [ ] PD line follower (Kp=0.012, Kd=0.003)
- [ ] PD wall follower (Kp=0.008, Kd=0.002)
- [ ] 21-state Mission HFSM (all 4 tasks)
- [ ] Flash EEPROM emulation (`stored_ball_color`)
- [ ] Arm controller (CH12/13/14 slew, HOME/MODE_A/MODE_B positions)
- [ ] UART debug interface (`servo_all`, `la_cal`, `tof_scan`, `imu_read`)

---

## 🗺️ Key File Map

| Need | File |
|------|------|
| Start here | [`README.md`](./README.md) |
| Full technical spec | [`docs/TECHNICAL_BLUEPRINT.md`](docs/TECHNICAL_BLUEPRINT.md) |
| IK + gait math | [`docs/LOCOMOTION_AND_KINEMATICS.md`](docs/LOCOMOTION_AND_KINEMATICS.md) |
| All pin assignments | [`docs/PINOUT_AND_CONNECTIONS.md`](docs/PINOUT_AND_CONNECTIONS.md) |
| State machine table | [`docs/EMBEDDED_STATE_MACHINE.md`](docs/EMBEDDED_STATE_MACHINE.md) |
| IR + colour algorithms | [`docs/SENSOR_PERCEPTION.md`](docs/SENSOR_PERCEPTION.md) |
| Servo zeroing procedure | [`Mechanical/06_calibration_and_zeroing/README.md`](Mechanical/06_calibration_and_zeroing/README.md) |
| Pre-run checklist | [`Mechanical/07_maintenance_and_repair/README.md`](Mechanical/07_maintenance_and_repair/README.md) |
| Project roadmap | [`spider-robot-roadmap.md`](./spider-robot-roadmap.md) |
| Firmware architecture | [`docs/SOFTWARE_ARCHITECTURE.md`](docs/SOFTWARE_ARCHITECTURE.md) |

---

## 📐 Hardware Quick Reference

```
Servo tick formula (50Hz, MG90S):
  tick = 102 + (angle / 180.0) × 389
  0°  = tick 102  (500µs)
  90° = tick 307  (1500µs)
  180°= tick 491  (2400µs)

Arm positions:
  CH12 HOME   = 90°  (tick 307)  — vertical, travel safe
  CH12 MODE_A =  0°  (tick 102)  — horizontal forward, ball
  CH12 MODE_B = 160° (tick 450)  — floor-pointing, zone ID
  CH13 OPEN   = 60°  (tick 184)
  CH13 CLOSE  = 115° (tick 286)
  CH14 LOCKED =  0°  (tick 102)
  CH14 OPEN   = 90°  (tick 307)

IR centroid:
  error = centroid - 4.0  (range -4.0 to +4.0)
  error > 0 → line is RIGHT → Wz > 0 → turn right
  error < 0 → line is LEFT  → Wz < 0 → turn left

ToF thresholds:
  TOF_BALL_APPROACH_MM  =  80  (Task 1 stop)
  TOF_OBSTACLE_MM       = 150  (Task 3 detect)
  TOF_WALL_TARGET_MM    = 150  (Task 2 setpoint)
  TOF_GAP_THRESHOLD_MM  = 250  (gap confirmed)
  TOF_PUSH_CLEARED_MM   = 300  (Task 3 success)
```
