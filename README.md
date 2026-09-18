# FusionForce RUNNER-4 — BREACH PROTOCOL
## EN2533 Autonomous Quadruped Competition Robot

> **MCU:** STM32F411CEU6 (Black Pill) | **Servos:** 15× MG90S | **Links:** L1=30mm L2=60mm L3=80mm
> **Gait:** Diagonal Trot | **Sensors:** 9-ch IR (ADC DMA) + 3× ToF + IMU + Colour

A fully autonomous 12-DOF 3D-printed quadruped robot built for the EN2533 BREACH PROTOCOL competition. All perception, mission logic, and real-time motor control run on a **single STM32F411CEU6** — no Raspberry Pi, no computer vision, no external processor.

---

## 🗂️ Repository Structure

```
FusionForce-Robotics/
├── docs/                   ← Engineering documentation (start here)
├── firmware/               ← STM32F411CEU6 bare-metal C firmware
├── Mechanical/             ← 3D print guides, assembly, calibration
├── Robot_Curriculum/       ← Learning path: L1 Fundamentals → L4 Sensors & Integration
├── Testing codes/          ← Arduino/STM32 test scripts per subsystem
├── 3D designs/             ← SolidWorks CAD assemblies
├── Task/                   ← Arena task specifications
└── Proposal/               ← Competition proposal documents
```

---

## ⚡ System at a Glance

```
         9× TCRT5000 IR Array    3× VL53L0X ToF    MPU6050    TCS34725
         (ADC1+DMA PA0-PA7,PB0)  (I2C2, XSHUT)    (I2C1)    (I2C1, arm tip)
                  │                    │               │           │
         ┌────────┴────────────────────┴───────────────┴───────────┴────────┐
         │              STM32F411CEU6 Black Pill (100MHz)                    │
         │  Mission HFSM (21 states) │ Trot Gait (600ms) │ IK Solver       │
         │  PD Line Follow │ PD Wall Follow │ Flash persistence │ UART debug│
         └───────────────────────────────┬────────────────────────────────── ┘
                                         │ I2C1 (400kHz)
                                   PCA9685 (0x40)
                                         │
              ┌──────────────┬───────────┴───────────┬──────────────┐
         CH0-CH11        CH12 Arm           CH13 Gripper      CH14 Gate
         12× MG90S       MG90S (0-160°)     MG90S (60-115°)  MG90S (0-90°)
         Leg Servos
```

---

## 🦾 Hardware Specification

| Parameter | Value |
|-----------|-------|
| **MCU** | STM32F411CEU6 Black Pill — 100MHz ARM Cortex-M4F, FPU |
| **Servos** | 15× MG90S metal-gear, 2.2 kg·cm @ 6V |
| **Leg DOF** | 12 (3-DOF per leg × 4 legs) |
| **Coxa (L1)** | **30 mm** |
| **Femur (L2)** | **60 mm** |
| **Tibia (L3)** | **80 mm** |
| **Max reach** | L2+L3 = **140 mm** from coxa pivot |
| **Gait** | Diagonal trot — FL+BR / FR+BL, T=600ms, step=35mm |
| **Servo driver** | PCA9685, I2C1 (0x40), 50Hz, 500-2400µs |
| **Line array** | 9× TCRT5000, ADC1+DMA, 80mm wide, 10mm pitch |
| **ToF sensors** | 3× VL53L0X, I2C2 (0x30/0x31/0x32), XSHUT remapped |
| **IMU** | MPU6050, I2C1 (0x68), 100Hz, tilt safety watchdog |
| **Colour sensor** | TCS34725, I2C1 (0x29), arm tip mount, LED=PC0 |
| **Power** | 2S LiPo → 6V BEC (servo) + 3.3V logic, star ground |
| **Chassis** | PETG 3D-printed, body 160×120mm |

---

## 📚 Documentation

👉 **[Full Engineering Docs →](docs/README.md)**

| Doc | Description |
|-----|-------------|
| [Technical Blueprint](docs/TECHNICAL_BLUEPRINT.md) | Complete system manual — start here |
| [Hardware Architecture](docs/HARDWARE_ARCHITECTURE.md) | Compute, sensor, actuator block diagram |
| [Pinout & Connections](docs/PINOUT_AND_CONNECTIONS.md) | Authoritative STM32 pin assignment table |
| [Locomotion & Kinematics](docs/LOCOMOTION_AND_KINEMATICS.md) | IK derivation, trot gait math, PD controllers |
| [Embedded State Machine](docs/EMBEDDED_STATE_MACHINE.md) | 21-state HFSM — all 4 competition tasks |
| [Sensor Perception](docs/SENSOR_PERCEPTION.md) | 9-ch IR array + TCS34725 algorithms |
| [Software Architecture](docs/SOFTWARE_ARCHITECTURE.md) | Firmware structure, 50Hz loop |
| [Power Architecture](docs/POWER_ARCHITECTURE.md) | Dual-rail power, star ground, protection |
| [Mechanical Design](docs/MECHANICAL_DESIGN.md) | Chassis, leg assembly, arm, CoM analysis |
| [Troubleshooting Guide](docs/TROUBLESHOOTING.md) | UART debug commands, common fixes |
| [Master Checklist](docs/MASTER_IMPLEMENTATION_CHECKLIST.md) | Step-by-step build + test checklist |

---

## 🎓 Team Learning Curriculum

New team members: complete the curriculum before touching firmware code.

👉 **[Master Robotics Curriculum →](Robot_Curriculum/README.md)**

| Level | Topics | Est. Time |
|-------|---------|-----------|
| [Level 1 — Fundamentals](Robot_Curriculum/Level_1_Fundamentals/README.md) | C, Electronics, Git, CubeMX setup | 2–3 weeks |
| [Level 2 — STM32](Robot_Curriculum/Level_2_STM32/README.md) | HAL, I2C, ADC DMA, PCA9685, Timers | 2–3 weeks |
| [Level 3 — Robotics Core](Robot_Curriculum/Level_3_Robotics/README.md) | IK, trot gait, PD control, HFSM | 2–3 weeks |
| [Level 4 — Sensors & Integration](Robot_Curriculum/Level_4_Sensors_and_Gait/README.md) | Full mission, all sensors, competition run | 1–2 weeks |

---

## 🔧 Mechanical Assembly

👉 **[Mechanical Assembly Workflow →](Mechanical/README.md)**

| Module | Description |
|--------|-------------|
| [01 Chassis Design](Mechanical/01_chassis_design/README.md) | PETG print specs, CoG planning |
| [02 Leg Assembly](Mechanical/02_leg_assembly/README.md) | 12× MG90S, L1/L2/L3 links, channel map |
| [03 Gripper & Gate](Mechanical/03_gripper_and_bumper/README.md) | CH12 arm, CH13 gripper, CH14 gate |
| [04 Electronics Mounting](Mechanical/04_electronics_mounting/README.md) | STM32, PCA9685, all sensor placements |
| [05 Power Distribution](Mechanical/05_power_distribution/README.md) | 6V servo rail, 3.3V logic, star ground |
| [06 Calibration & Zeroing](Mechanical/06_calibration_and_zeroing/README.md) | 15-servo zeroing, UART calibration |
| [07 Maintenance & Repair](Mechanical/07_maintenance_and_repair/README.md) | Pre-run checklist, failure diagnosis |

---

## 🏁 Competition Tasks

| Task | Description | Primary Sensors |
|------|-------------|----------------|
| **Task 1** | Follow white line → find ball → colour ID → grab & store | IR array (line) + ToF (proximity) + TCS34725 (colour) |
| **Task 2** | Navigate corridor with gaps | ToF ×3 (wall-follow PD) |
| **Task 3** | Detect and push obstacle | ToF front (obstacle) + IMU (tilt safety) |
| **Task 4** | Follow line → junction → colour-match zone → release ball | IR array + TCS34725 (floor zone) |

---

*Repository status: Active development. All documentation is current as of hardware confirmation: L1=30mm, L2=60mm, L3=80mm, 15× MG90S, STM32F411CEU6.*