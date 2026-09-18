# FusionForce Engineering Documentation
## RUNNER-4 | EN2533 BREACH PROTOCOL | STM32F411CEU6

Authoritative technical documentation, architectural blueprints, and tracking checklists.
**No Raspberry Pi. No Computer Vision. All perception is sensor-based (IR array, ToF, colour sensor, IMU).**

## Core Planning & Tracking

| Document | Purpose |
|----------|---------|
| **[Technical Blueprint](./TECHNICAL_BLUEPRINT.md)** | Comprehensive system manual — Start Here |
| **[Technical Proposal](./TECHNICAL_PROPOSAL.md)** | Executive engineering proposal |
| **[Execution Report](./EXECUTION_REPORT.md)** | Project management tracker |
| **[Master Implementation Checklist](./MASTER_IMPLEMENTATION_CHECKLIST.md)** | Step-by-step implementation guide |
| **[Risk Register (FMEA)](./RISK_REGISTER.md)** | Risk analysis and mitigation |
| **[Requirements](./REQUIREMENTS.md)** | System requirements and constraints |

## Technical Architecture

| Document | Purpose |
|----------|---------|
| **[System Overview](./SYSTEM_OVERVIEW.md)** | Single-domain STM32 architecture — why no Raspberry Pi |
| **[Hardware Architecture](./HARDWARE_ARCHITECTURE.md)** | Compute + sensor + actuator layers, block diagram |
| **[STM32 Architecture](./STM32_ARCHITECTURE.md)** | STM32F411CEU6 peripherals, CubeMX config |
| **[Software Architecture](./SOFTWARE_ARCHITECTURE.md)** | 50Hz loop, firmware directory structure |
| **[Power Architecture](./POWER_ARCHITECTURE.md)** | 6V servo rail, 3.3V logic rail, star ground |
| **[Locomotion & Kinematics](./LOCOMOTION_AND_KINEMATICS.md)** | IK solver (L1=30 L2=60 L3=80mm), trot gait, PD controllers |
| **[Mechanical Design](./MECHANICAL_DESIGN.md)** | PETG chassis, leg assembly, arm+gripper, line array bracket |
| **[Embedded State Machine](./EMBEDDED_STATE_MACHINE.md)** | 21-state HFSM, all 4 tasks, transition tables |

## Sensors & Perception

| Document | Purpose |
|----------|---------|
| **[Sensor Perception](./SENSOR_PERCEPTION.md)** | 9-ch IR array (ADC DMA) + TCS34725 algorithms |
| **[Pinout and Connections](./PINOUT_AND_CONNECTIONS.md)** | Complete STM32 pin assignments (authoritative) |

## Safety & Maintenance

| Document | Purpose |
|----------|---------|
| **[Safety and Failure Modes](./SAFETY_AND_FAILURE_MODES.md)** | Failsafes, hardware protection, SAFE_STOP |
| **[Troubleshooting Guide](./TROUBLESHOOTING.md)** | Common issues, UART debug commands, fixes |

## Quick Reference

### Hardware at a Glance
```
MCU:    STM32F411CEU6 Black Pill — 100MHz, 512KB Flash, 128KB RAM
Servos: 15× MG90S | CH0-11 legs | CH12 arm | CH13 gripper | CH14 gate
Links:  L1 (Coxa)=30mm | L2 (Femur)=60mm | L3 (Tibia)=80mm
Gait:   Trot | T=600ms | Step height=35mm | Vx_max=100mm/s
Power:  6V BEC (servo) | 3.3V logic | 2S LiPo | Star ground
```

### I2C Bus Map
```
I2C1 (PB6/PB7): PCA9685@0x40 | MPU6050@0x68 | TCS34725@0x29
I2C2 (PB10/PB3): VL53L0X Front@0x30 | Left@0x31 | Right@0x32
```

### ADC Line Array
```
ADC1 + DMA1 circular | S1-S8: PA0-PA7 | S9: PB0
9× TCRT5000 | 80mm wide | 10mm pitch | 16× oversample | 5mm from floor
Centroid range: 0-8 | error = centroid - 4.0
```

### Arm Positions
```
CH12 HOME  = 90°  (tick 307) — vertical, safe for walking
CH12 MODE_A = 0°  (tick 102) — horizontal forward, ball at pedestal
CH12 MODE_B = 160° (tick 450) — angled down to floor, zone ID
CH13 OPEN   = 60°  (tick 184) — gripper open (40mm ball)
CH13 CLOSE  = 115° (tick 286) — gripper gripping
CH14 LOCKED = 0°   (tick 102) — gate closed
CH14 OPEN   = 90°  (tick 307) — gate open (ball exits)
```

---
🔙 **[Back to Main Repository README](../README.md)**
