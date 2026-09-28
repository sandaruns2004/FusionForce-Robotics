# RUNNER-4 Project Roadmap
## FusionForce | EN2533 BREACH PROTOCOL | STM32F411CEU6 Single-Controller Architecture

> **Architecture Decision (Final):** All perception, decision-making, and motor control on a **single STM32F411CEU6**. No Raspberry Pi. No computer vision. All sensing via embedded hardware sensors.

---

## 1. System Architecture (Final)

```
         ┌──────────────────────────────────────────────────────────────┐
         │                  RUNNER-4 SYSTEM                             │
         │                                                              │
         │  SENSORS                                                     │
         │  9× TCRT5000 IR (ADC1+DMA)   → Line follow / junction       │
         │  TCS34725 Colour (I2C1 0x29)  → Ball + floor zone colour ID │
         │  MPU6050 IMU (I2C1 0x68)      → Tilt safety watchdog 100Hz  │
         │  VL53L0X ×3 (I2C2 0x30-0x32) → Wall-follow / obstacle ToF  │
         │                                                              │
         │  BRAIN: STM32F411CEU6 Black Pill (100MHz, Cortex-M4F + FPU) │
         │  Mission HFSM (21 states) | Trot gait (T=600ms) | IK solver │
         │  PD Line + Wall controllers | Flash persistence              │
         │                                                              │
         │  ACTUATORS                                                   │
         │  PCA9685 (I2C1 0x40) → 15× MG90S servos                    │
         │  CH0-11: 12 leg servos (Coxa/Femur/Tibia ×4 legs)           │
         │  CH12: Arm pitch | CH13: Gripper | CH14: Ball gate           │
         └──────────────────────────────────────────────────────────────┘
```

**Why single-controller?**
- No inter-CPU failure point (Pi→STM32 UART was a single point of failure)
- Deterministic 50Hz loop — no Linux preemption, no boot delay
- No Pi weight (+46g) or power (+1.5A) penalty
- All sensors produce stable, low-latency values — colour via TCS34725, not camera HSV

---

## 2. Bill of Materials (Final, Confirmed)

| Component | Qty | Spec | Role |
|-----------|-----|------|------|
| **STM32F411CEU6** Black Pill | 1 | 100MHz, 512KB Flash, 128KB RAM, HW FPU | Sole compute node |
| **PCA9685** PWM driver | 1 | I2C1 (0x40), 50Hz, 12-bit | 15-servo PWM generation |
| **MG90S** metal-gear servo | 15 | 2.2 kg·cm @ 6V, 500-2400µs | 12 legs + arm + gripper + gate |
| **TCRT5000** IR reflective | 9 | Analogue, 10kΩ pull-up | Line array (ADC1+DMA, 80mm wide) |
| **TCS34725** RGBC sensor | 1 | I2C1 (0x29), 50ms integration | Ball + floor zone colour ID |
| **MPU6050** IMU | 1 | I2C1 (0x68), ±500 dps | Tilt safety watchdog |
| **VL53L0X** ToF sensor | 3 | I2C2, XSHUT remap | Wall-follow (L/R) + obstacle (Front) |
| **2S LiPo** battery | 1 | 7.4V, ≥1300mAh, ≥25C | Primary power |
| **Switching BEC** | 1 | ≥5A continuous, 6V output | Servo power rail |
| **3.3V LDO/SMPS** | 1 | ≥500mA | STM32 + sensor logic rail |
| **PETG filament** | ~400g | 1.75mm | Chassis, brackets, links |
| Start button | 1 | Tactile, to GND, PC13 | Competition start trigger |

---

## 3. Link Lengths (Hardware Confirmed)

| Segment | Length | Max reach | Min reach |
|---------|--------|-----------|-----------|
| **Coxa (L1)** | **30 mm** | — | — |
| **Femur (L2)** | **60 mm** | — | — |
| **Tibia (L3)** | **80 mm** | — | — |
| **Total reach** | — | **140 mm** | **20 mm** |

---

## 4. Phase-by-Phase Roadmap

### Phase 0 — Planning & Design ✅ COMPLETE
- [x] Link lengths confirmed: L1=30, L2=60, L3=80mm
- [x] Servo selected: MG90S (metal gear, 2.2 kg·cm @ 6V)
- [x] Architecture decided: Single STM32F411CEU6, no Raspberry Pi
- [x] Full engineering documentation suite written (`docs/`)
- [x] Mechanical assembly guides written (`Mechanical/`)
- [x] Team curriculum written (4 levels, `Robot_Curriculum/`)

### Phase 1 — Mechanical Build 🔧
- [ ] 3D print all PETG parts (chassis plates, 4× coxa brackets, 4× femur, 4× tibia, arm link)
- [ ] Install M3 brass heat-set inserts (soldering iron, 200°C)
- [ ] Assemble one complete leg first — verify L1/L2/L3 range, no binding
- [ ] Assemble remaining 3 legs; mount MR83ZZ bearings
- [ ] Mount arm link + gripper fingers + gate plate (belly)
- [ ] Mount STM32, PCA9685, sensors to chassis decks

### Phase 2 — Electrical & Power 🔌
- [ ] Solder XT60 to LiPo leads + inline 15A kill switch + 10A servo fuse
- [ ] Wire switching BEC: input from battery, output 6V to PCA9685 V+ terminal
- [ ] Wire 3.3V LDO: output to STM32, PCA9685 VCC, all sensor VCC
- [ ] Star ground: BEC GND + LDO GND + STM32 GND + sensor GNDs joined
- [ ] Install 4× 4.7kΩ I2C pull-ups (2 per bus, I2C1 + I2C2)
- [ ] Install 9× 10kΩ IR sensor pull-ups (PA0-PA7, PB0)
- [ ] Install 470µF electrolytic cap on servo rail (BEC output)
- [ ] Voltage checks: 6.0V on BEC, 3.3V logic, no shorts

### Phase 3 — STM32 Firmware (Low-Level) 💻
- [ ] STM32CubeMX: 100MHz HSE→PLL, I2C1 (PB6/7), I2C2 (PB10/3), ADC1 DMA circular (PA0-7, PB0), TIM2 50Hz, TIM3 100Hz, USART1 115200
- [ ] PCA9685 driver: `pca9685_init()`, `pca9685_set_angle(ch, deg)`, 50Hz prescaler=121
- [ ] VL53L0X driver: XSHUT remap boot sequence (PB12/13/14), range read, EMA filter
- [ ] MPU6050 driver: raw accel+gyro read, complementary filter, 100Hz ISR
- [ ] TCS34725 driver: non-blocking start+poll, `classify_color()` ratio-dominance
- [ ] Line array driver: ADC1 DMA 144-word buffer, EMA filter, centroid, junction detect
- [ ] Servo zeroing: `servo_all 90\r\n` UART command, all 15 channels tick=307

### Phase 4 — Mechanical Calibration 🎯
- [ ] Servo zeroing (Module 06): power all 15 servos → hold 90° → attach horns
- [ ] Measure each servo angle with digital protractor → fill `calibration.h` offsets
- [ ] IR calibration: `la_cal white` then `la_cal black` → saved to Flash
- [ ] Verify centroid ≈ 4.0 when centred on 30mm white line
- [ ] ToF scan: `tof_scan\r\n` → FRONT 0x30, LEFT 0x31, RIGHT 0x32 all OK
- [ ] IMU: `imu_read\r\n` → |pitch| < 3° and |roll| < 3° on flat surface

### Phase 5 — Gait & IK Engine 🦿
- [ ] IK solver: `IK_SolveAll(feet[4], joints[12])` using L1=30, L2=60, L3=80
- [ ] Neutral stance: call IK on all 4 neutral positions → robot stands level
- [ ] Trot gait: `GaitEngine_Update(Vx, Vy, Wz)` → diagonal FL+BR / FR+BL
- [ ] Slow trot at Vx=20mm/s, Wz=0 → verify no toe drag, no body pitch
- [ ] Increase Vx to 60mm/s, Wz=0 → stable for 30s on flat surface

### Phase 6 — Navigation & State Machine 🧠
- [ ] PD line follower: tune Kp=0.012, Kd=0.003 on real arena black tape
- [ ] PD wall follower: tune Kp=0.008, Kd=0.002 in corridor with 300mm gap
- [ ] 21-state HFSM: implement and test each task state in isolation
- [ ] Flash EEPROM: `stored_ball_color` written after Task 1, recalled after reset
- [ ] Arm sequencer: ARM_HOME→MODE_A→GRIPPER_CLOSE→HOME→GATE_OPEN

### Phase 7 — Full Integration & Competition 🏁
- [ ] Task 1 full sequence: line follow → ball approach → colour ID → grab → store
- [ ] Task 2 full sequence: corridor wall-follow, gap detection
- [ ] Task 3 full sequence: obstacle detect → push → turn
- [ ] Task 4 full sequence: line follow → junction → floor colour → release ball
- [ ] Full circuit run: all 4 tasks, <3 minutes
- [ ] Pre-competition calibration procedure: IR cal + colour cal under arena lighting
- [ ] Endurance: 15-minute run without servo brownout or watchdog trigger

---

## 5. Key Risks

| Risk | Severity | Mitigation |
|------|---------|-----------|
| MG90S arm torque margin <1.0x | HIGH | Shorten arm link to 50mm or upgrade CH12 to MG996R |
| Servo rail brownout under trot | HIGH | 470µF cap on BEC output; verify BEC rating ≥5A continuous |
| I2C connector vibration failure | MEDIUM | Hot-glue all I2C DuPont connectors after testing |
| IR calibration drift under arena lighting | MEDIUM | Re-calibrate `la_cal` at venue before every run |
| TCS34725 mis-classify under warm LEDs | MEDIUM | Recalibrate colour thresholds under actual arena lighting |
| Line lost at corridor exit | LOW | Line-lost failsafe: SAFE_STOP after 3s, ±15° search |
| Servo horn comes off | LOW | Loctite all horn screws; re-zero if replaced |

---

## 6. Pin Reference (Quick Card)

```
I2C1  SDA=PB7  SCL=PB6   → PCA9685(0x40) + MPU6050(0x68) + TCS34725(0x29)
I2C2  SDA=PB3  SCL=PB10  → VL53L0X Front(0x30) + Left(0x31) + Right(0x32)
ADC1  PA0-PA7, PB0       → IR S1-S9 (DMA1 Stream0 CH0, circular, 144 words)
GPIO  PB12/13/14          → VL53L0X XSHUT Front/Left/Right
GPIO  PC0                 → TCS34725 LED (HIGH=ON)
GPIO  PC13                → Start button (Active LOW, pull-up)
UART  PA9(TX) PA10(RX)    → 115200 baud debug (disconnect at competition)
ADC   PC1                 → Battery voltage divider (low-battery monitor)
```

---
🔙 **[Back to Main README](./README.md)** | 📚 **[Engineering Docs](./docs/README.md)**
